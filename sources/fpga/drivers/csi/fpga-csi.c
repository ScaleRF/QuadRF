// fpga-csi.c - CSI-2 RAW8/RAW10 char device for Raspberry Pi 5 (RP1, Cam1)
//
// Policy / high-level behavior:
//
//  - Geometry is fixed at probe from DT/driver defaults:
//      - acme,bytes-per-line
//      - acme,lines
//    If missing/invalid, we fall back to 1024x1024.
//    CSI_IOC_SET_GEOMETRY is kept only for ABI compatibility and is a no-op.
//  - RAW8 vs RAW10 is inferred from:
//      - DT overlay (acme,raw-bits / acme,dt), or
//      - userspace DT filter (CSI_IOC_SET_FILTER).
//    RAW8 enables CH_CTRL_PACK_BYTES; RAW10 leaves it clear.
//  - CSI2-DMA runs as a single channel with explicit buffer ownership:
//
//      - Two DMA addresses are always posted: the current span and a shadow
//        address for the immediately following span.
//      - FE_ACK (or the next FS if FE_ACK coalesced) retires the completed
//        buffer to an ordered high-priority copy workqueue.
//      - DMA buffers move through FREE -> QUEUED -> ACTIVE -> COPY_PENDING.
//      - With the default policy a full userspace ring drops the incoming
//        complete span; a continuous CSI source cannot be backpressured.
//  - No explicit STOPSTATE wait or watchdog.
//  - Userspace typically reads from /dev/csi_stream0 via read(), poll(), or
//    can mmap() the ring and use the ring IOCTLs defined in fpga_csi.h.
//
// Copyright (C) 2025
// License: GPL v2

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/pio_rp1.h>

#include "fpga_csi.h"

#define DRV_NAME "fpga-csi"
#define DEV_NAME "csi_stream0"

/* -------- Module params -------- */

static bool drop_oldest;
module_param(drop_oldest, bool, 0444);
MODULE_PARM_DESC(drop_oldest,
                 "Overwrite oldest ring data when full "
                 "(default: 0=drop incoming span)");

static bool jtag_rio = true;
module_param(jtag_rio, bool, 0644);
MODULE_PARM_DESC(jtag_rio,
                 "Bit-bang JTAG with direct RP1 RIO stores instead of gpiolib "
                 "(default: 1, falls back to gpiolib if RIO is not found)");

static bool jtag_pio = true;
module_param(jtag_pio, bool, 0644);
MODULE_PARM_DESC(jtag_pio,
                 "Clock paced JTAG batches out of an RP1 PIO state machine "
                 "(default: 1, falls back to timed RIO bit-bang)");

/* -------- Userspace byte ring -------- */

#define RING_ORDER   9      /* 512 pages: 8 MiB with Pi 5's 16 KiB pages */
#define RING_PAGES   (1u << RING_ORDER)
#define RING_SIZE    (RING_PAGES << PAGE_SHIFT)

/* -------- JTAG GPIO read/write width -------- */

#define DR_WIDTH     25                /* ECP5 USER1/ER1 chain width */

struct byte_ring {
    u8          *data;
    size_t       size;                 /* power-of-two */
    size_t       rpos;                 /* modulo size */
    size_t       wpos;                 /* modulo size */
    spinlock_t   lock;
    wait_queue_head_t wq_read;         /* readers wait for data */
};

static inline size_t r_used(const struct byte_ring *r)
{
    return (r->wpos - r->rpos) & (r->size - 1);
}

static inline size_t r_space(const struct byte_ring *r)
{
    return r->size - 1 - r_used(r);
}

/* Copy a complete span at wpos without publishing it to readers yet. */
static void r_copy_in(struct byte_ring *r, size_t wpos,
                      const u8 *src, size_t len)
{
    size_t first = min(len, r->size - wpos);

    memcpy(r->data + wpos, src, first);
    if (first != len)
        memcpy(r->data, src + first, len - first);
}

/* -------- RP1 CSI2-DMA / D-PHY / MIPIC -------- */

struct rp1_regs {
    void __iomem *csi2;  /* reg[0] CSI2-DMA */
    void __iomem *dphy;  /* reg[1] D-PHY (optional) */
    void __iomem *mipic; /* reg[2] MIPI CFG */
    void __iomem *fe;    /* reg[3] FE (optional) */
};

static inline void __iomem *ch_base(void __iomem *csi2, u32 ch)
{
    return csi2 + (0x028 + (ch * 0x40));
}

/* CSI2-DMA global regs (base = csi2) */
#define CSI2_STATUS              0x000
#define CSI2_QOS                 0x004
#define CSI2_DISCARDS_OVERFLOW   0x008
#define CSI2_DISCARDS_INACTIVE   0x00c
#define CSI2_DISCARDS_UNMATCHED  0x010
#define CSI2_DISCARDS_LEN_LIMIT  0x014
#define CSI2_LLEV_PANICS         0x018
#define CSI2_ULEV_PANICS         0x01c
#define CSI2_IRQ_MASK            0x020
#define CSI2_CTRL                0x024
#define  CSI2_CTRL_EOP_IS_EOL    BIT(0)

/* Discard field masks (matches upstream reference) */
#define CSI2_DISCARDS_AMOUNT_SHIFT 0
#define CSI2_DISCARDS_AMOUNT_MASK  GENMASK(23, 0)
#define CSI2_DISCARDS_DT_SHIFT     24
#define CSI2_DISCARDS_DT_MASK      GENMASK(29, 24)
#define CSI2_DISCARDS_VC_SHIFT     30
#define CSI2_DISCARDS_VC_MASK      GENMASK(31, 30)

/* Per-channel register block offsets (relative to channel base) */
#define CH_CTRL              0x000
#define CH_ADDR0             0x004
#define CH_STRIDE            0x008  /* units of 16 bytes */
#define CH_LENGTH            0x00c  /* units of 16 bytes */
#define CH_DEBUG             0x010  /* frame/line counters (ro) */
#define CH_ADDR1             0x014  /* upper address bits (>>36) */
#define CH_FRAME_SIZE        0x018  /* (h<<16)|w when CH_MODE != 0 */
#define CH_COMP_CTRL         0x01c
#define CH_FE_FRAME_ID       0x020

/* MIPI CFG (base = mipic) */
#define MIPIC_CFG            0x004
#define  MIPIC_CFG_SEL_CSI   BIT(0)
#define MIPIC_INTR           0x028 /* Raw interrupt status */
#define MIPIC_INTE           0x02c /* Interrupt Enable */
#define MIPIC_INTF           0x030 /* Interrupt Force */
#define MIPIC_INTS           0x034 /* Masked interrupt status */
#define  MIPIC_INT_CSI_DMA   BIT(0)
#define  MIPIC_INT_CSI_HOST  BIT(2)
#define  MIPIC_INT_PISP_FE   BIT(4)

/* CSI2_STATUS bits (global) */
#define CSI2_STATUS_IRQ_FS(ch)     (BIT(0)  << (ch))
#define CSI2_STATUS_IRQ_FE(ch)     (BIT(4)  << (ch))
#define CSI2_STATUS_IRQ_FE_ACK(ch) (BIT(8)  << (ch))
#define CSI2_STATUS_IRQ_LE(ch)     (BIT(12) << (ch))
#define CSI2_STATUS_IRQ_LE_ACK(ch) (BIT(16) << (ch))
#define CSI2_STATUS_IRQ_OVERFLOW           BIT(20)
#define CSI2_STATUS_IRQ_DISCARD_OVERFLOW   BIT(21)
#define CSI2_STATUS_IRQ_DISCARD_LEN_LIMIT  BIT(22)
#define CSI2_STATUS_IRQ_DISCARD_UNMATCHED  BIT(23)
#define CSI2_STATUS_IRQ_DISCARD_INACTIVE   BIT(24)

/* CH_CTRL layout */
#define CH_CTRL_DMA_EN        BIT(0)
#define CH_CTRL_MODE_SHIFT    1
#define CH_CTRL_MODE_MASK     (0x3 << CH_CTRL_MODE_SHIFT)
#define CH_CTRL_FORCE         BIT(3)
#define CH_CTRL_AUTO_ARM      BIT(4)
#define CH_CTRL_VC_SHIFT      5
#define CH_CTRL_DT_SHIFT      7
#define CH_CTRL_DT_MASK       (0x3f << CH_CTRL_DT_SHIFT)
#define CH_CTRL_IRQ_FS_EN     BIT(13)
#define CH_CTRL_IRQ_FE_EN     BIT(14)
#define CH_CTRL_IRQ_FE_ACK_EN BIT(15)
#define CH_CTRL_IRQ_LE_EN     BIT(16)
#define CH_CTRL_IRQ_LE_ACK_EN BIT(17)
#define CH_CTRL_LC_SHIFT      18
#define CH_CTRL_LC_MASK       (0x3ff << CH_CTRL_LC_SHIFT)
#define CH_CTRL_FLUSH_FE      BIT(28)
#define CH_CTRL_PACK_LINE     BIT(29)
#define CH_CTRL_PACK_BYTES    BIT(30)   /* RAW8 only */

#define CSI2_MODE_NORMAL       0
#define CSI2_MODE_REMAP        1
#define CSI2_MODE_COMPRESSED   2
#define CSI2_MODE_FE_STREAMING 3

#define CSI_DMA_CHANNEL 0
#define DMA_BUF_COUNT   16
#define DMA_BUF_NONE    DMA_BUF_COUNT

enum dma_buf_state {
    DMA_BUF_FREE = 0,
    DMA_BUF_QUEUED,
    DMA_BUF_ACTIVE,
    DMA_BUF_COPY_PENDING,
};

struct fpga_csi_dev;

struct csi_dma_buffer {
    void       *cpu;
    dma_addr_t dma;
    size_t      size;

    struct work_struct copy_work;
    struct fpga_csi_dev *cd;
    u32          index;
    enum dma_buf_state state;
};

struct csi_file_ctx;

/* Slot numbers are absolute since stream start; the ring holds slot & (size-1). */
struct jtag_stream {
    spinlock_t            lock;            /* everything below; taken from DMA tasklet */
    wait_queue_head_t     wq;              /* feeder, queuers waiting for space, stop */
    struct task_struct   *feeder;
    struct csi_file_ctx  *owner;
    u32                  *ring;            /* queued words, 0 = idle slot */
    u32                  *chunkbuf;        /* feeder scratch, one DMA chunk */
    u32                   pio_delay;       /* delay loop count in Y, U32_MAX = unknown */
    u32                   period_ns;       /* quantized to 5 ns */
    u32                   chunk;           /* slots per DMA chunk */
    u64                   committed;       /* slots handed to DMA so far */
    u64                   queued_end;      /* one past the last queued write */
    int                   in_flight;       /* DMA chunks not yet completed */
    u64                   t_empty_ns;      /* when in_flight last dropped to 0 */
    u64                   t_slot0_ns;
    u32                   t_slot0_err_ns;
    u32                   underruns;
    bool                  active, running, stopping, fed_all, failed;
};

struct fpga_csi_dev {
    struct device       *dev;
    struct rp1_regs      regs;
    int                  irq;

    struct clk          *clk_core;    /* DT clocks[0] */
    struct clk          *clk_phy;     /* DT clocks[1] (optional) */
    struct reset_control *rst_core;   /* optional */
    struct reset_control *rst_phy;    /* optional */

    struct byte_ring     ring;
    struct mutex         read_lock;
    atomic_t             stopping;

    struct csi_dma_buffer dbuf[DMA_BUF_COUNT];
    spinlock_t           dma_lock;
    struct workqueue_struct *copy_wq;
    u32                  active_idx;
    u32                  next_idx;
    u32                  alloc_cursor;
    bool                 fs_seen;
    bool                 streaming;
    size_t               dma_span;     /* bytes pushed to ring per FE span */

    /* Complete-span losses in the software DMA/ring pipeline. */
    atomic64_t           software_drops;

    struct csi_filter_cfg filter;      /* current VC/DT filter */
    struct csi_geometry   geom;        /* current geometry */

    /* overlay defaults (read at probe): */
    u32                  ov_raw_bits;  /* 8 or 10; 0 = "unknown" */
    u32                  ov_dt;        /* 0x2A/0x2B when provided, else 0 */

    struct csi_stats      stats;

    /* Diagnostics */
    u64                   mipic_irq_total;
    u64                   mipic_irq_dma;
    u64                   mipic_irq_host;
    u64                   mipic_irq_other;
    u64                   ch_irq_total;
    u64                   ch_irq_fs;
    u64                   ch_irq_fe;
    u64                   ch_irq_both;
    u64                   inferred_fe;
    u64                   recovery_count;
    u64                   no_buffer_count;
    u64                   inactive_irq_count;

    /* JTAG / FPGA register access via ECP5 USER1 (ER1) */
    struct gpio_desc     *jtag_tck;
    struct gpio_desc     *jtag_tms;
    struct gpio_desc     *jtag_tdi;
    struct gpio_desc     *jtag_tdo;
    bool                  jtag_pins_acquired;
    bool                  jtag_ready;
    bool                  jtag_write_gpio_cansleep;
    void __iomem         *jtag_rio_base;   /* RP1 RIO bank 0, NULL = gpiolib only */
    bool                  jtag_rio_logged; /* userspace re-runs setup every ~30 s */
    u32                   rio_tck, rio_tms, rio_tdi, rio_tdo;

    /* JTAG streams and paced batches on an RP1 PIO SM, set up on first use */
    struct rp1_pio_client *pio;
    int                   pio_sm;
    uint                  pio_off;         /* program load offset */
    bool                  pio_failed;      /* stay on RIO until the module reloads */
    u32                   pio_fs;          /* PIO funcsel, read back after gpio_init */
    void __iomem         *iobank;          /* RP1 IO_BANK0, GPIOn_CTRL funcsel */
    struct jtag_stream    js;

    struct mutex          jtag_lock;
    atomic_t             jtag_users;

    struct csi_file_ctx  *jtag_owner;

    struct miscdevice     miscdev;
};

/* Per-open-file context (misc_open initially sets file->private_data = miscdevice) */
struct csi_file_ctx {
    struct miscdevice      *mdev;
    struct fpga_csi_dev    *cd;
    bool                    jtag_attached;
};

static inline struct csi_file_ctx *csi_ctx(struct file *f)
{
    return (struct csi_file_ctx *)f->private_data;
}

static inline struct fpga_csi_dev *csi_cd(struct file *f)
{
    return csi_ctx(f)->cd;
}

/* ---- CSR helpers ---- */

static inline u32 rd(struct fpga_csi_dev *cd, u32 off)
{
    return readl(cd->regs.csi2 + off);
}

static inline void wr(struct fpga_csi_dev *cd, u32 off, u32 v)
{
    writel(v, cd->regs.csi2 + off);
}

static inline u32 rd_ch(struct fpga_csi_dev *cd, u32 ch_off, u32 off)
{
    return readl(ch_base(cd->regs.csi2, ch_off) + off);
}

static inline void wr_ch(struct fpga_csi_dev *cd, u32 ch_off, u32 off, u32 v)
{
    writel(v, ch_base(cd->regs.csi2, ch_off) + off);
}

/* MIPI CFG accessors */
static inline u32 rd_mipic(struct fpga_csi_dev *cd, u32 off)
{
    return readl(cd->regs.mipic + off);
}

static inline void wr_mipic(struct fpga_csi_dev *cd, u32 off, u32 v)
{
    writel(v, cd->regs.mipic + off);
}

/* ---- devm action helpers ---- */

static void clk_disable_unprepare_put(void *p)
{
    struct clk *c = p;

    clk_disable_unprepare(c);
    clk_put(c);
}

static void reset_assert_action(void *p)
{
    struct reset_control *r = p;

    reset_control_assert(r);
}

/* ---- D-PHY TEST interface helpers ---- */

static uint8_t dphy_xact(void __iomem *d, u8 code, u8 data)
{
    u32 ctrl0, ctrl1;

    ctrl0 = readl(d + 0x050);
    writel((ctrl0 & ~0x2) | 0x2, d + 0x050); /* TESTCLK=1 */

    ctrl1 = readl(d + 0x054);
    writel((ctrl1 & ~(1u << 16)) | 0, d + 0x054); /* TESTEN=0 */

    ctrl1 = readl(d + 0x054);
    writel((ctrl1 & ~0xffu) | code, d + 0x054);   /* DATA=code */

    ctrl1 = readl(d + 0x054);
    writel((ctrl1 & ~(1u << 16)) | (1u << 16), d + 0x054); /* TESTEN=1 */

    ctrl0 = readl(d + 0x050);
    writel(ctrl0 & ~0x2, d + 0x050); /* TESTCLK=0 */

    /* Present data */
    ctrl1 = readl(d + 0x054);
    writel(ctrl1 & ~(1u << 16), d + 0x054); /* TESTEN=0 */

    ctrl1 = readl(d + 0x054);
    writel((ctrl1 & ~0xffu) | data, d + 0x054);

    ctrl0 = readl(d + 0x050);
    writel((ctrl0 & ~0x2) | 0x2, d + 0x050); /* TESTCLK=1 */

    return ((readl(d + 0x054) >> 8) & 0xff);
}

/* ===== D-PHY init (no STOPSTATE wait enforcement) ===== */

static int dphy_init(struct fpga_csi_dev *cd)
{
    void __iomem *d = cd->regs.dphy;
    u32 nlanes = 1;
    /* 640 selects the stable HSFREQRANGE bucket for this 350 MHz source. */
    u32 mbps = 640;
    u32 nlanes_reg;

    if (!d)
        return -ENODEV;

    of_property_read_u32(cd->dev->of_node, "acme,nlanes", &nlanes);
    of_property_read_u32(cd->dev->of_node, "acme,mbps", &mbps);
    if (nlanes == 0)
        nlanes = 1;
    nlanes_reg = (nlanes - 1) & 0x3;

    dev_info(cd->dev,
             "DPHY VERSION=0x%08x, lanes=%u (N_LANES=%u), %u Mbps",
             readl(d + 0x000), nlanes, nlanes_reg, mbps);

    /* Configure the number of lanes before the reset sequence. */
    writel(nlanes_reg, d + 0x004); /* N_LANES = lanes-1 */

    /* Reset sequence */
    writel(0, d + 0x044); /* PHY_RSTZ=0 */
    writel(0, d + 0x040); /* PHY_SHUTDOWNZ=0 */

    /* Pulse TSTCLR while TSTCLK high (per ref driver) */
    {
        u32 c0, c1;

        c0 = readl(d + 0x050);
        writel((c0 & ~2) | 2, d + 0x050); /* TSTCLK=1 */

        c1 = readl(d + 0x054);
        writel(c1 & ~(1u << 16), d + 0x054); /* TESTEN=0 */

        c0 = readl(d + 0x050);
        writel((c0 & ~1) | 1, d + 0x050); /* TSTCLR=1 */
        usleep_range(15, 20);

        c0 = readl(d + 0x050);
        writel(c0 & ~1, d + 0x050);       /* TSTCLR=0 */
        usleep_range(15, 20);
    }

    /* HSFREQRANGE table (mbps threshold, code) */
    static const u16 htab[][2] = {
        {  89, 0b000000 }, {  99, 0b010000 }, { 109, 0b100000 },
        { 129, 0b000001 }, { 139, 0b010001 }, { 149, 0b100001 },
        { 169, 0b000010 }, { 179, 0b010010 }, { 199, 0b100010 },
        { 219, 0b000011 }, { 239, 0b010011 }, { 249, 0b100011 },
        { 269, 0b000100 }, { 299, 0b010100 }, { 329, 0b000101 },
        { 359, 0b010101 }, { 399, 0b100101 }, { 449, 0b000110 },
        { 499, 0b010110 }, { 549, 0b000111 }, { 599, 0b010111 },
        { 649, 0b001000 }, { 699, 0b011000 }, { 749, 0b001001 },
        { 799, 0b011001 }, { 849, 0b101001 }, { 899, 0b111001 },
        { 949, 0b001010 }, { 999, 0b011010 }, {1049, 0b101010 },
        {1099, 0b111010 }, {1149, 0b001011 }, {1199, 0b011011 },
        {1249, 0b101011 }, {1299, 0b111011 }, {1349, 0b001100 },
        {1399, 0b011100 }, {1449, 0b101100 }, {1500, 0b111100 },
    };
    u8 code = 0b001000; /* 640 Mbps default */

    for (size_t i = 0; i < ARRAY_SIZE(htab) - 1; ++i) {
        if (mbps <= htab[i][0]) {
            code = htab[i][1];
            break;
        }
    }

    dphy_xact(d, 0x44, code << 1); /* HS_RX_CTRL_LANE0 */

    /* Bring PHY out of reset */
    usleep_range(5, 10);
    writel(1, d + 0x040); /* SHUTDOWNZ=1 */
    usleep_range(5, 10);
    writel(1, d + 0x044); /* RSTZ=1 */

    /* De-assert controller reset */
    writel(0xffffffff, d + 0x008); /* RESETN deassert */
    usleep_range(10, 50);

    /* EOP => EOL so FE IRQ aligns to frame end */
    wr(cd, CSI2_CTRL, CSI2_CTRL_EOP_IS_EOL);

    return 0;
}

/* ---- RAW helpers ---- */

static inline bool raw8_selected(const struct fpga_csi_dev *cd)
{
    if (cd->filter.enable_dt_filter)
        return (cd->filter.dt == 0x2A);

    if (cd->ov_dt)
        return (cd->ov_dt == 0x2A);

    if (cd->ov_raw_bits)
        return (cd->ov_raw_bits == 8);

    /* Default to RAW10-style packing */
    return false;
}

/* ---- Stats helpers ---- */

static inline void stats_add_discard(struct fpga_csi_dev *cd, u32 which_reg)
{
    u32 v = rd(cd, which_reg);
    u32 amount, dt, vc;

    if (!v)
        return;

    wr(cd, which_reg, 0); /* clear */

    amount = (v & CSI2_DISCARDS_AMOUNT_MASK) >> CSI2_DISCARDS_AMOUNT_SHIFT;
    dt     = (v & CSI2_DISCARDS_DT_MASK)     >> CSI2_DISCARDS_DT_SHIFT;
    vc     = (v & CSI2_DISCARDS_VC_MASK)     >> CSI2_DISCARDS_VC_SHIFT;

    switch (which_reg) {
    case CSI2_DISCARDS_OVERFLOW:
        cd->stats.discards_overflow[vc] += amount;
        cd->stats.discards_overflow_dt  = dt;
        break;
    case CSI2_DISCARDS_LEN_LIMIT:
        cd->stats.discards_len_limit[vc] += amount;
        cd->stats.discards_len_limit_dt  = dt;
        break;
    case CSI2_DISCARDS_UNMATCHED:
        cd->stats.discards_unmatched[vc] += amount;
        cd->stats.discards_unmatched_dt  = dt;
        break;
    case CSI2_DISCARDS_INACTIVE:
        cd->stats.discards_inactive[vc] += amount;
        cd->stats.discards_inactive_dt  = dt;
        break;
    }
}

/* ---- mmap(): expose vmalloc-backed userspace ring ---- */

static int csi_mmap(struct file *f, struct vm_area_struct *vma)
{
    struct fpga_csi_dev *cd = csi_cd(f);

    size_t want   = vma->vm_end - vma->vm_start;
    size_t ring   = cd->ring.size;
    size_t palign = PAGE_ALIGN(ring);

    /* accept exact or page-aligned mapping sizes */
    if (want != ring && want != palign)
        return -EINVAL;
    if (!cd->ring.data)
        return -ENXIO;

    vma->vm_pgoff = 0;
    vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);

    /* ring.data must be vmalloc_user()?d; see probe() */
    return remap_vmalloc_range(vma, cd->ring.data, 0);
}

/* ---- Channel programming derived from geometry ---- */

static void csi_start_channel(struct fpga_csi_dev *cd, u32 ch)
{
    void __iomem *base = ch_base(cd->regs.csi2, ch);
    u32 ctrl;

    /*
     * IRQs: FS + FE_ACK
     *  - The current and shadow addresses are posted before they are needed.
     *  - FE_ACK retires current, promotes shadow, and posts a new shadow.
     *  - AUTO_ARM remains disabled for direct CSI2-DMA operation.
     *  - PACK_LINE groups the received payload by line.
     */
    ctrl = CH_CTRL_IRQ_FS_EN | CH_CTRL_IRQ_FE_ACK_EN |
           CH_CTRL_PACK_LINE; // | CH_CTRL_AUTO_ARM;

    if (raw8_selected(cd))
        ctrl |= CH_CTRL_PACK_BYTES;

    if (cd->filter.enable_vc_filter)
        ctrl |= ((cd->filter.vc & 0x3) << CH_CTRL_VC_SHIFT);
    if (cd->filter.enable_dt_filter)
        ctrl |= ((cd->filter.dt & 0x3f) << CH_CTRL_DT_SHIFT);

    ctrl = (ctrl & ~CH_CTRL_MODE_MASK) |
           (CSI2_MODE_NORMAL << CH_CTRL_MODE_SHIFT);

    if (cd->geom.lines) {
        u32 width_hint = raw8_selected(cd) ? cd->geom.bytes_per_line : 0;

        writel((cd->geom.lines << 16) | (width_hint & 0xffff),
               base + CH_FRAME_SIZE);
    }

    /* Length/stride are in units of 16 bytes */
    {
        u32 stride16 = cd->geom.bytes_per_line >> 4;
        u32 span_lines = cd->geom.lines ? cd->geom.lines : 512; /* ~1/2 1024 default */
        u32 len16;

        if (cd->geom.bytes_per_line & 0xF)
            stride16++;
        len16 = stride16 * span_lines;

        writel(stride16, base + CH_STRIDE);
        writel(len16,    base + CH_LENGTH);
    }

    /* Configure the channel while keeping DMA stopped.  Probe programs a
     * valid initial address before enabling DMA. */
    writel(ctrl, base + CH_CTRL);
}

/*
 * Program one CSI2-DMA channel for the next span.
 *
 * The HW requires CH_ADDR1 (upper bits) to be written before CH_ADDR0,
 * and CH_ADDR0 write latches & arms the channel.
 */
static void csi_dma_program_addr(struct fpga_csi_dev *cd, u32 ch, u32 buf_idx)
{
    void __iomem *base = ch_base(cd->regs.csi2, ch);
    u64 addr = cd->dbuf[buf_idx].dma;

    /* Address: CH_ADDR1 then CH_ADDR0 (ADDR0 last latches/arms) */
    writel((u32)(addr >> 36),                base + CH_ADDR1);
    wmb();
    writel((u32)((addr >> 4) & 0xffffffffu), base + CH_ADDR0);
    wmb();
}

static u32 csi_ack_status(struct fpga_csi_dev *cd)
{
    u32 status = rd(cd, CSI2_STATUS);

    /* Clearing the CSI2 source deasserts the MIPIC summary. */
    if (status)
        wr(cd, CSI2_STATUS, status);
    return status;
}

static void csi_clear_pending_irqs(struct fpga_csi_dev *cd)
{
    csi_ack_status(cd);
    wmb();
}

static void csi_hw_quiesce(struct fpga_csi_dev *cd)
{
    void __iomem *ch = ch_base(cd->regs.csi2, CSI_DMA_CHANNEL);

    /* Stop delivery first, then stop the producer. */
    wr(cd, CSI2_IRQ_MASK, 0xffffffff);
    wr_mipic(cd, MIPIC_INTE,
             rd_mipic(cd, MIPIC_INTE) & ~MIPIC_INT_CSI_DMA);

    /* RP1 requires an ADDR0 write to latch CH_CTRL.  FORCE permits stopping
     * mid-frame; the second zero write is required by the reference driver. */
    writel(CH_CTRL_FORCE, ch + CH_CTRL);
    writel(0, ch + CH_ADDR0);
    writel(0, ch + CH_ADDR0);
    wmb();

    csi_clear_pending_irqs(cd);
}

/* dma_lock must be held. */
static u32 csi_find_free_buffer_locked(struct fpga_csi_dev *cd)
{
    u32 n;

    for (n = 0; n < DMA_BUF_COUNT; n++) {
        u32 idx = (cd->alloc_cursor + n) % DMA_BUF_COUNT;

        if (cd->dbuf[idx].state == DMA_BUF_FREE) {
            cd->alloc_cursor = (idx + 1) % DMA_BUF_COUNT;
            return idx;
        }
    }

    return DMA_BUF_NONE;
}

/* dma_lock must be held.  Post the front/current hardware queue entry. */
static bool csi_queue_current_buffer_locked(struct fpga_csi_dev *cd)
{
    u32 idx;

    if (!cd->streaming || cd->active_idx != DMA_BUF_NONE)
        return cd->active_idx != DMA_BUF_NONE;

    idx = csi_find_free_buffer_locked(cd);
    if (idx == DMA_BUF_NONE) {
        cd->no_buffer_count++;
        return false;
    }

    cd->dbuf[idx].state = DMA_BUF_ACTIVE;
    cd->active_idx = idx;
    csi_dma_program_addr(cd, CSI_DMA_CHANNEL, idx);
    return true;
}

/* dma_lock must be held.  Post the shadow entry for the following frame. */
static bool csi_queue_next_buffer_locked(struct fpga_csi_dev *cd)
{
    u32 idx;

    if (!cd->streaming || cd->next_idx != DMA_BUF_NONE)
        return cd->next_idx != DMA_BUF_NONE;

    idx = csi_find_free_buffer_locked(cd);
    if (idx == DMA_BUF_NONE) {
        cd->no_buffer_count++;
        return false;
    }

    cd->dbuf[idx].state = DMA_BUF_QUEUED;
    cd->next_idx = idx;
    csi_dma_program_addr(cd, CSI_DMA_CHANNEL, idx);
    return true;
}

static void csi_process_frame_events(struct fpga_csi_dev *cd,
                                     bool fs_event, bool fe_event);

static int csi_enable_streaming(struct fpga_csi_dev *cd)
{
    void __iomem *ch = ch_base(cd->regs.csi2, CSI_DMA_CHANNEL);
    unsigned long flags;
    u32 ctrl;
    u32 i;
    u32 status;
    bool primed;

    /* Keep global CSI2 and MIPIC delivery masked until an address is valid. */
    csi_clear_pending_irqs(cd);
    wr(cd, CSI2_DISCARDS_OVERFLOW, 0);
    wr(cd, CSI2_DISCARDS_INACTIVE, 0);
    wr(cd, CSI2_DISCARDS_UNMATCHED, 0);
    wr(cd, CSI2_DISCARDS_LEN_LIMIT, 0);

    spin_lock_irqsave(&cd->dma_lock, flags);
    for (i = 0; i < DMA_BUF_COUNT; i++)
        cd->dbuf[i].state = DMA_BUF_FREE;
    cd->active_idx = DMA_BUF_NONE;
    cd->next_idx = DMA_BUF_NONE;
    cd->alloc_cursor = 0;
    cd->fs_seen = false;
    cd->streaming = true;
    spin_unlock_irqrestore(&cd->dma_lock, flags);

    /* RP1 has a front address and a shadow address.  Both must be posted
     * before IRQ delivery is enabled: waiting until FS/FE to post the second
     * address leaves a frame-boundary race when IRQ service is delayed. */
    ctrl = readl(ch + CH_CTRL);
    writel(ctrl | CH_CTRL_DMA_EN, ch + CH_CTRL);

    spin_lock_irqsave(&cd->dma_lock, flags);
    primed = csi_queue_current_buffer_locked(cd) &&
             csi_queue_next_buffer_locked(cd);
    spin_unlock_irqrestore(&cd->dma_lock, flags);

    if (!primed) {
        spin_lock_irqsave(&cd->dma_lock, flags);
        cd->streaming = false;
        spin_unlock_irqrestore(&cd->dma_lock, flags);
        csi_hw_quiesce(cd);
        return -ENOBUFS;
    }

    /* Process any frame event that arrived after the address became valid,
     * then enable interrupt delivery.  This closes the startup FS race. */
    status = csi_ack_status(cd);
    csi_process_frame_events(cd,
                             status & CSI2_STATUS_IRQ_FS(CSI_DMA_CHANNEL),
                             status & CSI2_STATUS_IRQ_FE_ACK(CSI_DMA_CHANNEL));
    wr(cd, CSI2_IRQ_MASK, 0x00000000);
    wr_mipic(cd, MIPIC_INTE,
             rd_mipic(cd, MIPIC_INTE) | MIPIC_INT_CSI_DMA);
    wmb();
    return 0;
}

/*
 * Publish one complete DMA span to userspace.  There is one ordered producer,
 * so the memcpy can run without holding ring.lock: wpos is advanced only after
 * the complete span is present.  This also avoids disabling or delaying the
 * frame-start hard IRQ during a 128 KiB copy.
 */
static bool push_dma_to_ring(struct fpga_csi_dev *cd, u32 buf_idx,
                             size_t nbytes)
{
    struct byte_ring *r = &cd->ring;
    const u8 *src = cd->dbuf[buf_idx].cpu;
    unsigned long flags;
    size_t wpos;
    u64 lost_spans = 0;

    if (atomic_read(&cd->stopping))
        return false;

    spin_lock_irqsave(&r->lock, flags);
    if (r_space(r) < nbytes) {
        if (!drop_oldest) {
            spin_unlock_irqrestore(&r->lock, flags);
            atomic64_inc(&cd->software_drops);
            return false;
        }

        /* Free at least one span and normally half the ring. */
        {
            size_t used = r_used(r);
            size_t discard = min(used, max(nbytes, r->size >> 1));

            r->rpos = (r->rpos + discard) & (r->size - 1);
            lost_spans = DIV_ROUND_UP(discard, nbytes);
        }
    }

    if (WARN_ON_ONCE(r_space(r) < nbytes)) {
        spin_unlock_irqrestore(&r->lock, flags);
        atomic64_add(lost_spans + 1, &cd->software_drops);
        return false;
    }

    wpos = r->wpos;
    spin_unlock_irqrestore(&r->lock, flags);

    r_copy_in(r, wpos, src, nbytes);

    spin_lock_irqsave(&r->lock, flags);
    if (atomic_read(&cd->stopping)) {
        spin_unlock_irqrestore(&r->lock, flags);
        if (lost_spans)
            atomic64_add(lost_spans, &cd->software_drops);
        return false;
    }
    if (WARN_ON_ONCE(r->wpos != wpos)) {
        spin_unlock_irqrestore(&r->lock, flags);
        atomic64_add(lost_spans + 1, &cd->software_drops);
        return false;
    }
    r->wpos = (wpos + nbytes) & (r->size - 1);
    spin_unlock_irqrestore(&r->lock, flags);

    if (lost_spans)
        atomic64_add(lost_spans, &cd->software_drops);
    wake_up_interruptible(&r->wq_read);
    return true;
}

static void csi_copy_work(struct work_struct *work)
{
    struct csi_dma_buffer *db =
        container_of(work, struct csi_dma_buffer, copy_work);
    struct fpga_csi_dev *cd = db->cd;
    unsigned long flags;
    size_t nbytes = min(cd->dma_span, db->size);

    if (!atomic_read(&cd->stopping) &&
        push_dma_to_ring(cd, db->index, nbytes))
        cd->stats.dma_bytes += nbytes;

    spin_lock_irqsave(&cd->dma_lock, flags);
    if (WARN_ON_ONCE(db->state != DMA_BUF_COPY_PENDING)) {
        spin_unlock_irqrestore(&cd->dma_lock, flags);
        return;
    }

    db->state = DMA_BUF_FREE;
    if (!atomic_read(&cd->stopping) && cd->streaming) {
        /* Normally only the shadow can be absent.  If staging was completely
         * exhausted, the first worker to return also restarts the front slot. */
        if (cd->active_idx == DMA_BUF_NONE)
            csi_queue_current_buffer_locked(cd);
        if (cd->active_idx != DMA_BUF_NONE &&
            cd->next_idx == DMA_BUF_NONE)
            csi_queue_next_buffer_locked(cd);
    }
    spin_unlock_irqrestore(&cd->dma_lock, flags);
}

/* dma_lock must be held. */
static bool csi_promote_next_buffer_locked(struct fpga_csi_dev *cd)
{
    u32 idx = cd->next_idx;

    if (idx == DMA_BUF_NONE)
        return false;

    cd->next_idx = DMA_BUF_NONE;
    if (WARN_ON_ONCE(cd->dbuf[idx].state != DMA_BUF_QUEUED))
        return false;

    cd->dbuf[idx].state = DMA_BUF_ACTIVE;
    cd->active_idx = idx;
    return true;
}

/* dma_lock must be held. */
static void csi_handle_fs_locked(struct fpga_csi_dev *cd)
{
    /* Current and shadow were posted ahead of time.  FS is therefore only an
     * ordering marker; doing MMIO here would recreate the IRQ-latency race. */
    if (cd->active_idx == DMA_BUF_NONE) {
        cd->no_buffer_count++;
        atomic64_inc(&cd->software_drops);
    }
    cd->fs_seen = true;
}

/* dma_lock must be held. */
static void csi_handle_fe_locked(struct fpga_csi_dev *cd)
{
    u32 idx;

    idx = cd->active_idx;
    cd->active_idx = DMA_BUF_NONE;
    cd->fs_seen = false;

    /* No current entry means this frame was already counted lost at FS.
     * Refill both hardware queue positions for the following frame. */
    if (idx == DMA_BUF_NONE) {
        csi_queue_current_buffer_locked(cd);
        if (cd->active_idx != DMA_BUF_NONE)
            csi_queue_next_buffer_locked(cd);
        return;
    }

    if (WARN_ON_ONCE(cd->dbuf[idx].state != DMA_BUF_ACTIVE)) {
        atomic64_inc(&cd->software_drops);
        cd->no_buffer_count++;
        csi_queue_current_buffer_locked(cd);
        if (cd->active_idx != DMA_BUF_NONE)
            csi_queue_next_buffer_locked(cd);
        return;
    }

    cd->stats.frame_count++;
    cd->dbuf[idx].state = DMA_BUF_COPY_PENDING;

    /* The shadow is already armed in hardware.  Promote it first, then post
     * its replacement before scheduling any lower-priority copy activity. */
    if (!csi_promote_next_buffer_locked(cd)) {
        cd->no_buffer_count++;
        atomic64_inc(&cd->software_drops);
        csi_queue_current_buffer_locked(cd);
    }
    if (cd->active_idx != DMA_BUF_NONE)
        csi_queue_next_buffer_locked(cd);

    if (WARN_ON_ONCE(!queue_work(cd->copy_wq,
                                 &cd->dbuf[idx].copy_work))) {
        atomic64_inc(&cd->software_drops);
    }
}

static void csi_process_frame_events(struct fpga_csi_dev *cd,
                                     bool fs_event, bool fe_event)
{
    unsigned long flags;

    if (atomic_read(&cd->stopping) || (!fs_event && !fe_event))
        return;

    spin_lock_irqsave(&cd->dma_lock, flags);

    if (!cd->streaming) {
        spin_unlock_irqrestore(&cd->dma_lock, flags);
        return;
    }

    /* If both bits accumulated before service, their order depends on whether
     * software had already observed the preceding FS. */
    if (fs_event && fe_event && !cd->fs_seen) {
        csi_handle_fs_locked(cd);
        csi_handle_fe_locked(cd);
    } else {
        if (fe_event)
            csi_handle_fe_locked(cd);
        if (fs_event) {
            /* A new FS proves the prior DMA span has ended.  RP1's reference
             * driver retires that buffer when the prior FE IRQ was missed. */
            if (cd->fs_seen) {
                cd->inferred_fe++;
                csi_handle_fe_locked(cd);
            }
            csi_handle_fs_locked(cd);
        }
    }

    spin_unlock_irqrestore(&cd->dma_lock, flags);
}

/* A fatal discard without FE can leave the direct-DMA channel unarmed.  Drop
 * only the incomplete front span, flush the hardware queue, and restore the
 * same current+shadow invariant used at startup. */
static void csi_reprime_after_error(struct fpga_csi_dev *cd)
{
    void __iomem *ch = ch_base(cd->regs.csi2, CSI_DMA_CHANNEL);
    unsigned long flags;
    u32 ctrl;
    u32 i;

    spin_lock_irqsave(&cd->dma_lock, flags);
    if (!cd->streaming || atomic_read(&cd->stopping)) {
        spin_unlock_irqrestore(&cd->dma_lock, flags);
        return;
    }

    ctrl = readl(ch + CH_CTRL);
    writel(ctrl | CH_CTRL_FORCE | CH_CTRL_FLUSH_FE, ch + CH_CTRL);
    writel(0, ch + CH_ADDR0);
    writel(0, ch + CH_ADDR0);
    wmb();

    if (cd->active_idx != DMA_BUF_NONE)
        atomic64_inc(&cd->software_drops);

    for (i = 0; i < DMA_BUF_COUNT; i++) {
        if (cd->dbuf[i].state == DMA_BUF_ACTIVE ||
            cd->dbuf[i].state == DMA_BUF_QUEUED)
            cd->dbuf[i].state = DMA_BUF_FREE;
    }
    cd->active_idx = DMA_BUF_NONE;
    cd->next_idx = DMA_BUF_NONE;
    cd->fs_seen = false;
    cd->recovery_count++;

    /* CH_CTRL is latched by the following CH_ADDR0 write. */
    ctrl &= ~(CH_CTRL_FORCE | CH_CTRL_FLUSH_FE);
    writel(ctrl | CH_CTRL_DMA_EN, ch + CH_CTRL);
    csi_queue_current_buffer_locked(cd);
    if (cd->active_idx != DMA_BUF_NONE)
        csi_queue_next_buffer_locked(cd);

    spin_unlock_irqrestore(&cd->dma_lock, flags);
}

/* Hard IRQ: acknowledge quickly, update ownership, and defer only the copy. */
static irqreturn_t fpga_csi_irq(int irq, void *data)
{
    struct fpga_csi_dev *cd = data;
    u32 ints = rd_mipic(cd, MIPIC_INTS);
    u32 status = csi_ack_status(cd);
    bool fs_event;
    bool fe_event;
    bool recover_event;

    if (!ints && !status)
        return IRQ_NONE;

    cd->mipic_irq_total++;
    if (ints & MIPIC_INT_CSI_DMA)
        cd->mipic_irq_dma++;
    if (ints & MIPIC_INT_CSI_HOST)
        cd->mipic_irq_host++;
    if (ints & ~(MIPIC_INT_CSI_DMA | MIPIC_INT_CSI_HOST))
        cd->mipic_irq_other++;

    /* CSI2_STATUS was cleared above, deasserting the MIPIC summary. */
    if (status)
        cd->ch_irq_total++;

    /* Error/discard accounting */
    if (status & CSI2_STATUS_IRQ_OVERFLOW)
        cd->stats.overflows++;
    if (status & CSI2_STATUS_IRQ_DISCARD_OVERFLOW)
        stats_add_discard(cd, CSI2_DISCARDS_OVERFLOW);
    if (status & CSI2_STATUS_IRQ_DISCARD_LEN_LIMIT)
        stats_add_discard(cd, CSI2_DISCARDS_LEN_LIMIT);
    if (status & CSI2_STATUS_IRQ_DISCARD_UNMATCHED)
        stats_add_discard(cd, CSI2_DISCARDS_UNMATCHED);
    if (status & CSI2_STATUS_IRQ_DISCARD_INACTIVE) {
        cd->inactive_irq_count++;
        stats_add_discard(cd, CSI2_DISCARDS_INACTIVE);
    }

    fs_event = status & CSI2_STATUS_IRQ_FS(CSI_DMA_CHANNEL);
    fe_event = status & CSI2_STATUS_IRQ_FE_ACK(CSI_DMA_CHANNEL);
    recover_event = status & (CSI2_STATUS_IRQ_OVERFLOW |
                              CSI2_STATUS_IRQ_DISCARD_OVERFLOW |
                              CSI2_STATUS_IRQ_DISCARD_INACTIVE);

    if (fs_event)
        cd->ch_irq_fs++;
    if (fe_event)
        cd->ch_irq_fe++;
    if (fs_event && fe_event)
        cd->ch_irq_both++;

    csi_process_frame_events(cd, fs_event, fe_event);

    /* A valid FE already advances and refills the two-entry queue. */
    if (recover_event && !fe_event)
        csi_reprime_after_error(cd);

    return IRQ_HANDLED;
}

/* -------- Char device ops -------- */

static ssize_t csi_read(struct file *f, char __user *ubuf, size_t len, loff_t *ppos)
{
    struct fpga_csi_dev *cd = csi_cd(f);
    ssize_t ret = 0;

    if (!len)
        return 0;

    if (mutex_lock_interruptible(&cd->read_lock))
        return -ERESTARTSYS;

    while (len) {
        size_t used;
        spin_lock_irq(&cd->ring.lock);
        used = r_used(&cd->ring);
        if (!used) {
            spin_unlock_irq(&cd->ring.lock);
            if (ret) break; /* return partial */
            if (f->f_flags & O_NONBLOCK) { ret = -EAGAIN; break; }
            if (wait_event_interruptible(cd->ring.wq_read,
                                         atomic_read(&cd->stopping) ||
                                         r_used(&cd->ring) > 0))
            { ret = -ERESTARTSYS; break; }
            if (atomic_read(&cd->stopping))
                break;
            continue;
        }

        /* Snapshot & copy */
        {
            const size_t mask = cd->ring.size - 1;
            size_t to = min(len, used);
            size_t rpos = cd->ring.rpos;

            if (rpos + to <= cd->ring.size) {
                spin_unlock_irq(&cd->ring.lock);
                if (copy_to_user(ubuf, cd->ring.data + rpos, to)) {
                    ret = ret ? ret : -EFAULT;
                    break;
                }
                spin_lock_irq(&cd->ring.lock);
                cd->ring.rpos = (rpos + to) & mask;
            } else {
                size_t first = cd->ring.size - rpos;
                size_t second = to - first;
                spin_unlock_irq(&cd->ring.lock);
                if (copy_to_user(ubuf, cd->ring.data + rpos, first) ||
                    copy_to_user(ubuf + first, cd->ring.data, second)) {
                    ret = ret ? ret : -EFAULT;
                    break;
                }
                spin_lock_irq(&cd->ring.lock);
                {
                    size_t cur = cd->ring.rpos;
                    size_t new_rpos = (rpos + to) & mask;
                    size_t fwd = (new_rpos - cur) & mask;
                    if (fwd && fwd < cd->ring.size)
                        cd->ring.rpos = new_rpos;
                }
            }
            spin_unlock_irq(&cd->ring.lock);

            ubuf += to; len -= to; ret += to;
            cd->stats.bytes_out += to;
        }
    }

    mutex_unlock(&cd->read_lock);
    return ret;
}

static __poll_t csi_poll(struct file *f, poll_table *wait)
{
    struct fpga_csi_dev *cd = csi_cd(f);
    __poll_t mask = 0;

    poll_wait(f, &cd->ring.wq_read, wait);
    spin_lock_irq(&cd->ring.lock);
    if (r_used(&cd->ring) > 0)
        mask |= POLLIN | POLLRDNORM;
    spin_unlock_irq(&cd->ring.lock);

    return mask;
}

/* -------- DMA buffer sizing helpers -------- */

static size_t compute_span_bytes(const struct csi_geometry *g)
{
    /*
     * If g->lines is non-zero, we treat one span as a full frame.
     * Otherwise, we use ~half of the default height (512 lines) as
     * the span for streaming modes that do not care about exact height.
     */
    if (g->lines)
        return (size_t)g->bytes_per_line * (size_t)g->lines;

    return (size_t)g->bytes_per_line * 512u; /* ~1/2 1024 height */
}

static int alloc_dma_buffers(struct fpga_csi_dev *cd, size_t span)
{
    int i, ret;
    size_t alloc_sz = PAGE_ALIGN(span);

    ret = dma_set_mask_and_coherent(cd->dev, DMA_BIT_MASK(64));
    if (ret) {
        dev_warn(cd->dev, "64-bit DMA mask failed, trying 40-bit");
        ret = dma_set_mask_and_coherent(cd->dev, DMA_BIT_MASK(40));
        if (ret) {
            dev_warn(cd->dev, "40-bit DMA mask failed, trying 32-bit");
            ret = dma_set_mask_and_coherent(cd->dev, DMA_BIT_MASK(32));
            if (ret) {
                dev_err(cd->dev, "Failed to set DMA mask: %d", ret);
                return ret;
            }
        }
    }

    for (i = 0; i < DMA_BUF_COUNT; ++i) {
        cd->dbuf[i].cd = cd;
        cd->dbuf[i].index = i;
        cd->dbuf[i].state = DMA_BUF_FREE;
        INIT_WORK(&cd->dbuf[i].copy_work, csi_copy_work);
        cd->dbuf[i].size = alloc_sz;
        cd->dbuf[i].cpu = dma_alloc_coherent(cd->dev, alloc_sz,
                                             &cd->dbuf[i].dma, GFP_KERNEL);
        if (!cd->dbuf[i].cpu) {
            dev_err(cd->dev, "DMA alloc failed at %d", i);
            goto err;
        }
        memset(cd->dbuf[i].cpu, 0, alloc_sz);
    }

    dev_info(cd->dev, "DMA buffers allocated: %d x %zu (span=%zu)",
             DMA_BUF_COUNT, alloc_sz, span);
    return 0;
err:
    while (--i >= 0)
        dma_free_coherent(cd->dev, cd->dbuf[i].size, cd->dbuf[i].cpu, cd->dbuf[i].dma);
    memset(cd->dbuf, 0, sizeof(cd->dbuf));
    return -ENOMEM;
}


static void free_dma_buffers(struct fpga_csi_dev *cd)
{
    int i;
    for (i = 0; i < DMA_BUF_COUNT; ++i) {
        if (cd->dbuf[i].cpu)
            dma_free_coherent(cd->dev, cd->dbuf[i].size, cd->dbuf[i].cpu, cd->dbuf[i].dma);
        cd->dbuf[i].cpu = NULL;
    }
}

static void csi_stop(struct fpga_csi_dev *cd)
{
    unsigned long flags;

    atomic_set(&cd->stopping, 1);

    spin_lock_irqsave(&cd->dma_lock, flags);
    cd->streaming = false;
    spin_unlock_irqrestore(&cd->dma_lock, flags);

    if (!cd->regs.csi2)
        return;

    csi_hw_quiesce(cd);

    /* Unblock userspace readers during teardown. */
    wake_up_all(&cd->ring.wq_read);
}

/* ---- JTAG bit-bang helpers (25-bit DR, ECP5 USER1/ER1) ---- */

/*
 * Single TCK cycle with specified TMS/TDIX, returning sampled TDO.
 * The FPGA logic treats the DR scan as 25 bits wide (DR_WIDTH).
 */
static inline int jtag_clock(struct fpga_csi_dev *cd, int tms, int tdi)
{
    int tdo;

    gpiod_set_value_cansleep(cd->jtag_tms, tms ? 1 : 0);
    gpiod_set_value_cansleep(cd->jtag_tdi, tdi ? 1 : 0);

    /* Rising edge TCK */
    gpiod_set_value_cansleep(cd->jtag_tck, 1);

    /* Sample TDO */
    tdo = gpiod_get_value_cansleep(cd->jtag_tdo);

    /* Falling edge TCK */
    gpiod_set_value_cansleep(cd->jtag_tck, 0);

    return tdo ? 1 : 0;
}

static void jtag_reset_tap(struct fpga_csi_dev *cd)
{
    int i;

    /* 5 cycles with TMS=1 to enter Test-Logic-Reset; then one RTI cycle */
    for (i = 0; i < 5; i++)
        jtag_clock(cd, 1, 0);
    jtag_clock(cd, 0, 0);
}

/* Shift 8-bit IR, LSB-first, into the TAP and return to Run-Test/Idle. */
static void jtag_shift_ir(struct fpga_csi_dev *cd, u8 ir)
{
    int i;

    /* RTI -> Select-DR -> Select-IR -> Capture-IR -> Shift-IR */
    jtag_clock(cd, 1, 0); /* RTI -> Select-DR */
    jtag_clock(cd, 1, 0); /* Select-DR -> Select-IR */
    jtag_clock(cd, 0, 0); /* Select-IR -> Capture-IR */
    jtag_clock(cd, 0, 0); /* Capture-IR -> Shift-IR */

    for (i = 0; i < 8; i++) {
        int tms = (i == 7); /* last bit: Exit1-IR */

        jtag_clock(cd, tms, (ir >> i) & 1);
    }

    /* Exit1-IR -> Update-IR -> RTI */
    jtag_clock(cd, 1, 0);
    jtag_clock(cd, 0, 0);
}

/*
 * DR scan for the fixed-width 25-bit user register chain.
 *
 * ECP5 user logic:
 *  - First DR clock after Capture-DR is a "ghost" clock used to latch
 *    user_reg_in at the start of the scan.
 *  - Following DR_WIDTH data clocks shift user_reg_out/user_reg_in.
 */
static void jtag_shift_dr25(struct fpga_csi_dev *cd, u32 out, u32 *in)
{
    u32 res = 0;
    int i;

    /* 1. Move to Capture-DR */
    jtag_clock(cd, 1, 0); /* Select-DR */
    jtag_clock(cd, 0, 0); /* Capture-DR */

    /* 2. Ghost clock: FPGA captures user_reg_in here */
    jtag_clock(cd, 0, 0);

    /* 3. Data clocks: cycles 1..DR_WIDTH */
    for (i = 0; i < DR_WIDTH; i++) {
        int bit = jtag_clock(cd, 0, (out >> i) & 1);

        if (bit)
            res |= (1u << i);
    }

    /* Extra idle clocks (no effect on FPGA past DR_WIDTH) */
    jtag_clock(cd, 0, 0);
    jtag_clock(cd, 0, 0);

    /* 4. Exit sequence */
    jtag_clock(cd, 1, 0); /* Exit1-DR */
    jtag_clock(cd, 1, 0); /* Update-DR */
    jtag_clock(cd, 0, 0); /* RTI */

    if (in)
        *in = res & ((1u << DR_WIDTH) - 1u);
}


/*
 * Fast write-only DR scan.
 *
 * IMPORTANT: keep the exact proven 33-clock sequence used by
 * jtag_shift_dr25():
 *   2 clocks to Capture-DR
 *   1 ghost clock
 *   DR_WIDTH (25) data clocks
 *   2 extra post-data clocks
 *   3 exit clocks (including the separately required Exit1 clock)
 *
 * The speedup comes only from eliminating GPIO work that is redundant
 * for a write:
 *   - no TDO sampling (write readback is discarded),
 *   - no repeated TMS assignments while TMS is unchanged,
 *   - no repeated TDI assignments while TDI is unchanged,
 *   - use non-sleeping GPIO calls when supported by the GPIO provider.
 */
static inline void jtag_write_pulse_fast(struct fpga_csi_dev *cd)
{
    gpiod_set_value(cd->jtag_tck, 1);
    gpiod_set_value(cd->jtag_tck, 0);
}

static inline void jtag_write_pulse_sleep(struct fpga_csi_dev *cd)
{
    gpiod_set_value_cansleep(cd->jtag_tck, 1);
    gpiod_set_value_cansleep(cd->jtag_tck, 0);
}

static void jtag_shift_dr25_write_fast(struct fpga_csi_dev *cd, u32 out)
{
    int i;
    int last_tdi = 0;

    /*
     * All existing JTAG helpers return to RTI with TDI=0; setup also starts
     * TDI low.  Avoid redundantly writing that already-known state here.
     */
    /* 1. Move to Capture-DR: RTI -> Select-DR -> Capture-DR */
    gpiod_set_value(cd->jtag_tms, 1);
    jtag_write_pulse_fast(cd);

    gpiod_set_value(cd->jtag_tms, 0);
    jtag_write_pulse_fast(cd);

    /* 2. Ghost clock: FPGA captures user_reg_in here */
    jtag_write_pulse_fast(cd);

    /* 3. Data clocks: cycles 1..DR_WIDTH; TMS remains low */
    for (i = 0; i < DR_WIDTH; i++) {
        int bit = (out >> i) & 1;

        if (bit != last_tdi) {
            gpiod_set_value(cd->jtag_tdi, bit);
            last_tdi = bit;
        }

        jtag_write_pulse_fast(cd);
    }

    /* Preserve the two established post-data clocks exactly. */
    if (last_tdi)
        gpiod_set_value(cd->jtag_tdi, 0);

    jtag_write_pulse_fast(cd);
    jtag_write_pulse_fast(cd);

    /*
     * 4. Preserve the established three-clock exit sequence exactly.
     * The separately required Exit1 clock is intentionally retained.
     */
    gpiod_set_value(cd->jtag_tms, 1);
    jtag_write_pulse_fast(cd); /* Exit1-DR */

    jtag_write_pulse_fast(cd); /* Update-DR */

    gpiod_set_value(cd->jtag_tms, 0);
    jtag_write_pulse_fast(cd); /* RTI */
}

static void jtag_shift_dr25_write_sleep(struct fpga_csi_dev *cd, u32 out)
{
    int i;
    int last_tdi = 0;

    /* TDI is already low on entry by the same driver invariant. */

    /* 1. Move to Capture-DR: RTI -> Select-DR -> Capture-DR */
    gpiod_set_value_cansleep(cd->jtag_tms, 1);
    jtag_write_pulse_sleep(cd);

    gpiod_set_value_cansleep(cd->jtag_tms, 0);
    jtag_write_pulse_sleep(cd);

    /* 2. Ghost clock: FPGA captures user_reg_in here */
    jtag_write_pulse_sleep(cd);

    /* 3. Data clocks: cycles 1..DR_WIDTH; TMS remains low */
    for (i = 0; i < DR_WIDTH; i++) {
        int bit = (out >> i) & 1;

        if (bit != last_tdi) {
            gpiod_set_value_cansleep(cd->jtag_tdi, bit);
            last_tdi = bit;
        }

        jtag_write_pulse_sleep(cd);
    }

    /* Preserve the two established post-data clocks exactly. */
    if (last_tdi)
        gpiod_set_value_cansleep(cd->jtag_tdi, 0);

    jtag_write_pulse_sleep(cd);
    jtag_write_pulse_sleep(cd);

    /* Preserve the established three-clock exit sequence exactly. */
    gpiod_set_value_cansleep(cd->jtag_tms, 1);
    jtag_write_pulse_sleep(cd); /* Exit1-DR */

    jtag_write_pulse_sleep(cd); /* Update-DR */

    gpiod_set_value_cansleep(cd->jtag_tms, 0);
    jtag_write_pulse_sleep(cd); /* RTI */
}

/*
 * RP1 RIO path.
 *
 * RP1 sits behind PCIe. A gpiod_set_value() through gpiolib and pinctrl-rp1
 * costs ~230 ns; a posted store straight to the RIO XOR alias costs ~25 ns.
 * TMS/TDI changes ride on the falling TCK edge (the TAP samples them on the
 * rising edge, half a period later), so every clock is exactly two stores and
 * a DR scan is 67 stores: ~1.7 us instead of ~19 us, TCK ~20 MHz. The clock
 * sequence is the same 33 clocks as jtag_shift_dr25().
 *
 * Stores to one Device-nGnRE peripheral stay in program order, and a read
 * cannot pass the posted writes ahead of it on PCIe, so no barriers are
 * needed between edges. Reads sample TDO via SYNC_IN after the rising edge,
 * like jtag_clock(); TDO was launched on the previous falling edge.
 *
 * RP1 RIO bank 0: OUT +0x00, SYNC_IN +0x08, XOR alias +0x1000, CLR +0x3000.
 */
#define RIO_OUT     0x0000
#define RIO_SYNC_IN 0x0008
#define RIO_XOR     0x1000
#define RIO_CLR     0x3000
#define RIO_SPAN    0x4000

/*
 * @nin: TDO bits to capture from the first data clocks. Each costs a ~1.2 us
 * non-posted PCIe round trip, so callers ask only for what they use.
 */
static void jtag_rio_scan(struct fpga_csi_dev *cd, u32 out, u32 *in, int nin)
{
    void __iomem *base = cd->jtag_rio_base;
    void __iomem *x = base + RIO_XOR;
    const u32 tck = cd->rio_tck, tms = cd->rio_tms, tdi = cd->rio_tdi;
    u32 seq[3 + DR_WIDTH + 5];
    u32 cur = 0, res = 0;
    int c, k = 0, i;

    seq[k++] = tms;                 /* RTI -> Select-DR */
    seq[k++] = 0;                   /* Capture-DR */
    seq[k++] = 0;                   /* ghost clock */
    for (i = 0; i < DR_WIDTH; i++)
        seq[k++] = (out >> i) & 1 ? tdi : 0;
    seq[k++] = 0;                   /* two post-data clocks */
    seq[k++] = 0;
    seq[k++] = tms;                 /* Exit1-DR */
    seq[k++] = tms;                 /* Update-DR */
    seq[k++] = 0;                   /* RTI */

    /*
     * Every path leaves TCK/TMS/TDI low between scans and the XOR stores rely
     * on that; CLR re-asserts it without making an edge.
     */
    writel_relaxed(tck | tms | tdi, base + RIO_CLR);
    for (c = 0; c < k; c++) {
        writel_relaxed((c ? tck : 0) | (cur ^ seq[c]), x);
        cur = seq[c];
        writel_relaxed(tck, x);
        if (in && c >= 3 && c < 3 + nin &&
            (readl_relaxed(base + RIO_SYNC_IN) & cd->rio_tdo))
            res |= 1u << (c - 3);
    }
    writel_relaxed(tck | cur, x);

    if (in)
        *in = res;
}

/* Wait until the posted stores have reached the pins. */
static inline void jtag_rio_drain(struct fpga_csi_dev *cd)
{
    if (cd->jtag_rio_base && jtag_rio)
        (void)readl(cd->jtag_rio_base + RIO_SYNC_IN);
}

static inline void jtag_shift_dr25_write(struct fpga_csi_dev *cd, u32 out)
{
    /* Branch once per complete DR write, never once per GPIO edge. */
    if (likely(cd->jtag_rio_base && jtag_rio))
        jtag_rio_scan(cd, out, NULL, 0);
    else if (likely(!cd->jtag_write_gpio_cansleep))
        jtag_shift_dr25_write_fast(cd, out);
    else
        jtag_shift_dr25_write_sleep(cd, out);
}

static int jtag_rio_bit(struct gpio_desc *d, u32 *mask)
{
    struct gpio_device *gdev = gpiod_to_gpio_device(d);
    int off;

    if (!gdev)
        return -ENODEV;
    off = desc_to_gpio(d) - gpio_device_get_base(gdev);
    if (off < 0 || off >= 28)       /* RIO bank 0 holds GPIO 0..27 */
        return -ERANGE;
    *mask = BIT(off);
    return 0;
}

static bool jtag_rio_check(void __iomem *rio, struct gpio_desc *d, u32 mask, int idle)
{
    u32 a, b;

    gpiod_set_value(d, !idle);
    a = readl(rio + RIO_OUT);
    gpiod_set_value(d, idle);
    b = readl(rio + RIO_OUT);
    return !!(a & mask) == !idle && !!(b & mask) == idle;
}

/*
 * Map RIO bank 0 and confirm, on the pins themselves, that each JTAG output
 * lands on the bit its gpio offset predicts. Called before the TAP reset in
 * setup, so the TCK edges made here are harmless.
 */
static void jtag_rio_map(struct fpga_csi_dev *cd)
{
    struct device_node *np;
    struct resource res;
    void __iomem *rio;
    int ret;

    cd->jtag_rio_base = NULL;
    if (cd->jtag_write_gpio_cansleep ||
        jtag_rio_bit(cd->jtag_tck, &cd->rio_tck) ||
        jtag_rio_bit(cd->jtag_tms, &cd->rio_tms) ||
        jtag_rio_bit(cd->jtag_tdi, &cd->rio_tdi) ||
        jtag_rio_bit(cd->jtag_tdo, &cd->rio_tdo))
        goto gpiolib;

    np = of_find_compatible_node(NULL, NULL, "raspberrypi,rp1-gpio");
    if (!np)
        goto gpiolib;
    ret = of_address_to_resource(np, 1, &res);     /* reg[1] = RIO */
    of_node_put(np);
    if (ret || resource_size(&res) < RIO_SPAN)
        goto gpiolib;
    rio = ioremap(res.start, RIO_SPAN);
    if (!rio)
        goto gpiolib;

    if (!jtag_rio_check(rio, cd->jtag_tck, cd->rio_tck, 0) ||
        !jtag_rio_check(rio, cd->jtag_tdi, cd->rio_tdi, 0) ||
        !jtag_rio_check(rio, cd->jtag_tms, cd->rio_tms, 1)) {
        dev_warn(cd->dev, "RIO bit check failed, JTAG stays on gpiolib\n");
        iounmap(rio);
        goto gpiolib;
    }
    cd->jtag_rio_base = rio;
    if (!cd->jtag_rio_logged)
        dev_info(cd->dev, "JTAG via RP1 RIO at %pa (tck %d tms %d tdi %d tdo %d)\n",
                 &res.start, __ffs(cd->rio_tck), __ffs(cd->rio_tms),
                 __ffs(cd->rio_tdi), __ffs(cd->rio_tdo));
    cd->jtag_rio_logged = true;
    return;

gpiolib:
    if (!cd->jtag_rio_logged)
        dev_info(cd->dev, "JTAG via gpiolib\n");
    cd->jtag_rio_logged = true;
}

static void jtag_pio_teardown(struct fpga_csi_dev *cd);

static void jtag_release_locked(struct fpga_csi_dev *cd)
{
    if (!cd->jtag_pins_acquired)
        return;

    jtag_pio_teardown(cd);

    if (cd->jtag_rio_base) {
        iounmap(cd->jtag_rio_base);
        cd->jtag_rio_base = NULL;
    }

    if (cd->jtag_tck) {
        gpiod_put(cd->jtag_tck);
        cd->jtag_tck = NULL;
    }
    if (cd->jtag_tms) {
        gpiod_put(cd->jtag_tms);
        cd->jtag_tms = NULL;
    }
    if (cd->jtag_tdi) {
        gpiod_put(cd->jtag_tdi);
        cd->jtag_tdi = NULL;
    }
    if (cd->jtag_tdo) {
        gpiod_put(cd->jtag_tdo);
        cd->jtag_tdo = NULL;
    }

    cd->jtag_pins_acquired = false;
    cd->jtag_ready = false;
}

static int jtag_setup_locked(struct fpga_csi_dev *cd)
{
    int ret;

    if (cd->jtag_ready)
        return 0;

    if (cd->jtag_tck || cd->jtag_tms || cd->jtag_tdi || cd->jtag_tdo) {
        dev_warn(cd->dev,
                 "JTAG pins in partial state, releasing and retrying\n");
        jtag_release_locked(cd);
    }

    cd->jtag_tck = gpiod_get(cd->dev, "jtag-tck", GPIOD_OUT_LOW);
    if (IS_ERR(cd->jtag_tck)) {
        ret = PTR_ERR(cd->jtag_tck);
        cd->jtag_tck = NULL;
        dev_err(cd->dev, "gpiod_get(jtag-tck) failed: %d\n", ret);
        return ret;
    }

    cd->jtag_tms = gpiod_get(cd->dev, "jtag-tms", GPIOD_OUT_HIGH);
    if (IS_ERR(cd->jtag_tms)) {
        ret = PTR_ERR(cd->jtag_tms);
        dev_err(cd->dev, "gpiod_get(jtag-tms) failed: %d\n", ret);
        goto err_put_tck;
    }

    cd->jtag_tdi = gpiod_get(cd->dev, "jtag-tdi", GPIOD_OUT_LOW);
    if (IS_ERR(cd->jtag_tdi)) {
        ret = PTR_ERR(cd->jtag_tdi);
        dev_err(cd->dev, "gpiod_get(jtag-tdi) failed: %d\n", ret);
        goto err_put_tms;
    }

    cd->jtag_tdo = gpiod_get(cd->dev, "jtag-tdo", GPIOD_IN);
    if (IS_ERR(cd->jtag_tdo)) {
        ret = PTR_ERR(cd->jtag_tdo);
        dev_err(cd->dev, "gpiod_get(jtag-tdo) failed: %d\n", ret);
        goto err_put_tdi;
    }

    /*
     * Cache GPIO sleepability once.  The write hot path selects the fast
     * or sleeping implementation once per complete DR scan.
     */
    cd->jtag_write_gpio_cansleep =
        gpiod_cansleep(cd->jtag_tck) ||
        gpiod_cansleep(cd->jtag_tms) ||
        gpiod_cansleep(cd->jtag_tdi);

    jtag_rio_map(cd);

    /* Configure idle levels and TAP */
    gpiod_set_value_cansleep(cd->jtag_tck, 0);
    gpiod_set_value_cansleep(cd->jtag_tms, 1);
    gpiod_set_value_cansleep(cd->jtag_tdi, 0);

    jtag_reset_tap(cd);          /* 5x TMS=1 + RTI */
    jtag_shift_ir(cd, 0x32);     /* USER1/ER1 */

    cd->jtag_ready = true;
    cd->jtag_pins_acquired = true;

    //dev_info(cd->dev, "JTAG setup complete (%s write GPIO path)\n",
    //         cd->jtag_write_gpio_cansleep ? "sleeping" : "fast non-sleeping");
    return 0;

err_put_tdi:
    gpiod_put(cd->jtag_tdi);
    cd->jtag_tdi = NULL;
err_put_tms:
    gpiod_put(cd->jtag_tms);
    cd->jtag_tms = NULL;
err_put_tck:
    gpiod_put(cd->jtag_tck);
    cd->jtag_tck = NULL;
    return ret;
}

/* JTAG register access: 25-bit word [write_flag|addr|value] */

static int jtag_reg_write(struct fpga_csi_dev *cd, u8 addr, u16 val)
{
    u32 jtag_val = (1u << 24) | ((u32)addr << 16) | (u32)val;

    /*
     * Write readback has always been discarded.  Use the dedicated
     * write-only scan: identical JTAG clocks/state transitions, less GPIO I/O.
     */
    jtag_shift_dr25_write(cd, jtag_val);

    return 0;
}

static int jtag_reg_read(struct fpga_csi_dev *cd, u8 addr, u16 *out_val)
{
    u32 jtag_val = ((u32)addr << 16); /* write flag = 0 */
    u32 readback = 0;

    if (cd->jtag_rio_base && jtag_rio) {
        /* the first scan only loads the address; its readback is stale */
        jtag_rio_scan(cd, jtag_val, NULL, 0);
        jtag_rio_scan(cd, jtag_val, &readback, 16);
        *out_val = (u16)(readback & 0xffff);
        return 0;
    }

    /* First DR scan: pipeline fill, discard result */
    jtag_shift_dr25(cd, jtag_val, &readback);

    /* Second DR scan: readback now valid */
    jtag_shift_dr25(cd, jtag_val, &readback);

    *out_val = (u16)(readback & 0xffff);

    return 0;
}

#define CSI_DEFAULT_BYTES_PER_LINE 1024
#define CSI_DEFAULT_LINES          1024

/*
 * Apply a geometry:
 *  - Validates bytes_per_line.
 *  - Disallows live geometry *changes* once DMA buffers exist.
 *  - Routes MIPIC to CSI while keeping interrupt delivery disabled.
 *  - Initializes the D-PHY (if present).
 *  - Allocates DMA buffers (first time) and configures the channel stopped.
 *
 * Today this is called once at probe using DT-derived defaults; the
 * CSI_IOC_SET_GEOMETRY ioctl is intentionally a no-op for ABI stability.
 */
static int csi_apply_geometry(struct fpga_csi_dev *cd,
                              const struct csi_geometry *g)
{
    struct csi_geometry geom = *g;
    size_t span;
    int ret;

    if (geom.bytes_per_line < 64)
        return -EINVAL;

    /* We rely on CH_STRIDE rounding being a no-op: require 16-byte alignment. */
    if (geom.bytes_per_line & 0xf) {
        dev_err(cd->dev,
                "bytes_per_line (%u) must be a multiple of 16\n",
                geom.bytes_per_line);
        return -EINVAL;
    }

    span = compute_span_bytes(&geom);

    /* Ensure a single DMA span can always fit in the user ring. */
    if (span > cd->ring.size - 1) {
        dev_err(cd->dev,
                "DMA span %zu exceeds ring capacity %zu\n",
                span, cd->ring.size - 1);
        return -EINVAL;
    }

    /* No live geometry changes once DMA buffers are allocated */
    if (cd->dbuf[0].cpu &&
        (cd->geom.bytes_per_line != geom.bytes_per_line ||
         cd->geom.lines          != geom.lines)) {
        return -EBUSY;
    }

    /* Route MIPI-CFG to CSI.  IRQ delivery remains disabled until the
     * handler is registered and every buffer is valid. */
    wr_mipic(cd, MIPIC_CFG,  MIPIC_CFG_SEL_CSI);

    /* Init the D-PHY (no STOPSTATE gating). Log failures but do not abort
     * for the optional D-PHY window (e.g. FPGA-driven CSI).
     */
    {
        int pret = dphy_init(cd);
        if (pret && pret != -ENODEV)
            dev_warn(cd->dev, "DPHY init failed: %d\n", pret);
    }

    if (!cd->dbuf[0].cpu) {
        /* First-time setup: allocate DMA and start the channel */
        cd->geom     = geom;
        cd->dma_span = span;

        ret = alloc_dma_buffers(cd, cd->dma_span);
        if (ret)
            return ret;

        csi_start_channel(cd, CSI_DMA_CHANNEL);

    } else {
        /* Same geometry as before: just refresh span in case lines changed from 0 */
        cd->geom     = geom;
        cd->dma_span = span;
    }

    return 0;
}

/* -------- IOCTLs -------- */


/* --- JTAG shared-use helpers (forward decls) --- */
static int csi_jtag_attach_locked(struct csi_file_ctx *ctx);
static void csi_jtag_detach_locked(struct fpga_csi_dev *cd);

/*
 * JTAG write streams on an RP1 PIO state machine.
 *
 * The RIO busy-wait keeps a schedule only while its stores reach RP1
 * promptly; DRAM traffic stretches each store and writes start late. Here a
 * PIO SM is a slot clock: every period it takes one word from its TX FIFO
 * without waiting. A word with the write flag (bit 24) runs one DR scan; 0, or
 * an empty FIFO, runs an idle slot of exactly the same length with TCK low
 * and the TAP parked in Run-Test/Idle. Slot k therefore starts at
 * t_slot0 + k * period, set by the 200 MHz PIO clock, for as long as the
 * stream runs.
 *
 * A feeder kthread keeps up to JSTREAM_NBUF DMA chunks in flight, built from
 * a ring of JSTREAM_RING slots; slots nobody queued go out as idle words.
 * That keeps the word -> slot mapping exact. If the feeder ever falls behind
 * by more than the FIFO, the SM runs idle slots on its own, the mapping
 * shifts, and the driver counts an underrun and re-estimates t_slot0.
 *
 * TCK and TMS are the two side-set pins (TMS must sit on TCK + 1), TDI the
 * single OUT pin; TMS follows the fixed 33-clock sequence of jtag_rio_scan().
 * The pins sit on the PIO function only while a stream runs; everything else
 * uses RIO. Both leave TCK/TMS/TDI low in Run-Test/Idle, so the handover
 * makes no edge.
 *
 * Slot length: pull + mov + jmp + mov + (x + 1) delay + 33 clocks of
 * 2 * JPIO_HALF = x + JPIO_FIXED PIO clocks of 5 ns. The idle path burns the
 * same 66 * JPIO_HALF clocks in two counted loops.
 */
#define JPIO_HALF     6                     /* PIO clocks per half TCK: 16.7 MHz */
#define JPIO_SCAN     (66 * JPIO_HALF)
#define JPIO_FIXED    (5 + JPIO_SCAN)
#define JPIO_NS       5
#define JPIO_FIFO     16                    /* TX FIFO joined: 2 x 8 words */
#define JSTREAM_NBUF  3                     /* of the 4 bounce buffers rp1-pio allows */
#define JSTREAM_RING  (256 * 1024)          /* slots, power of two: 1.3 s at 5 us */
#define JSTREAM_QUEUE_MAX 16384             /* writes per START / QUEUE call */
#define JSTREAM_CHUNK_MAX 1024              /* words per DMA chunk / bounce buffer */
#define JSTREAM_LEAD_NS   (1500 * 1000)     /* default DMA horizon */
#define IOB_CTRL(n)   (8 * (n) + 4)         /* IO_BANK0 GPIOn_CTRL, funcsel [4:0] */
#define FS_RIO        5

static void jtag_pio_program(u16 *p)
{
    const uint d = pio_encode_delay(JPIO_HALF - 1);
    const uint nop = pio_encode_mov(pio_y, pio_y);
    int k = 0;

#define SS(v) pio_encode_sideset(2, v)      /* bit0 TCK, bit1 TMS */
    p[k++] = pio_encode_pull(false, false) | SS(0);        /* 0: empty FIFO -> OSR = X = 0 */
    p[k++] = pio_encode_mov(pio_x, pio_osr) | SS(0);
    p[k++] = pio_encode_jmp_not_x(19) | SS(0);             /* idle slot */
    p[k++] = pio_encode_mov(pio_x, pio_y) | SS(0);
    p[k++] = pio_encode_jmp_x_dec(4) | SS(0);              /* 4: period padding */
    p[k++] = nop | SS(2) | d;
    p[k++] = nop | SS(3) | d;                               /* Select-DR */
    p[k++] = nop | SS(0) | d;
    p[k++] = nop | SS(1) | d;                               /* Capture-DR */
    p[k++] = nop | SS(0) | d;
    p[k++] = pio_encode_set(pio_x, 26) | SS(1) | d;         /* ghost clock */
    p[k++] = pio_encode_out(pio_pins, 1) | SS(0) | d;      /* 11: TDI on falling edge */
    p[k++] = pio_encode_jmp_x_dec(11) | SS(1) | d;         /* 25 data + 2 clocks */
    p[k++] = nop | SS(2) | d;
    p[k++] = nop | SS(3) | d;                               /* Exit1-DR */
    p[k++] = nop | SS(2) | d;
    p[k++] = nop | SS(3) | d;                               /* Update-DR */
    p[k++] = nop | SS(0) | d;
    p[k++] = pio_encode_set(pio_x, 0) | SS(1) | d;          /* 18: RTI; wrap -> 0 */
    p[k++] = pio_encode_mov(pio_x, pio_y) | SS(0);         /* 19: idle slot */
    p[k++] = pio_encode_jmp_x_dec(20) | SS(0);
    p[k++] = pio_encode_set(pio_x, 31) | SS(0) | pio_encode_delay(7);   /* 8 */
    p[k++] = pio_encode_jmp_x_dec(22) | SS(0) | pio_encode_delay(7);    /* 32 * 8 */
    p[k++] = pio_encode_set(pio_x, 15) | SS(0) | pio_encode_delay(1);   /* 2 */
    p[k++] = pio_encode_jmp_x_dec(24) | SS(0) | pio_encode_delay(7);    /* 16 * 8 */
    p[k++] = pio_encode_set(pio_x, 0) | SS(0);                          /* 1 */
    p[k++] = pio_encode_jmp(0) | SS(0);                                 /* 1: 396 */
#undef SS
}
#define JPIO_PROG_LEN 27
#define JPIO_WRAP     18

static void jtag_pio_funcsel(struct fpga_csi_dev *cd, u32 fs)
{
    const u32 bits[3] = { cd->rio_tck, cd->rio_tms, cd->rio_tdi };
    void __iomem *r;
    int i;

    for (i = 0; i < 3; i++) {
        r = cd->iobank + IOB_CTRL(__ffs(bits[i]));
        writel((readl(r) & ~0x1fu) | fs, r);
    }
    (void)readl(cd->iobank + IOB_CTRL(__ffs(cd->rio_tck)));
}

static inline u64 jstream_slot_ns(const struct jtag_stream *js, u64 slot)
{
    return js->t_slot0_ns + slot * js->period_ns;
}

/* DMA completion, tasklet context */
static void jstream_dma_cb(void *param)
{
    struct jtag_stream *js = param;
    unsigned long flags;

    spin_lock_irqsave(&js->lock, flags);
    js->in_flight--;
    if (!js->in_flight)
        js->t_empty_ns = ktime_get_ns();
    spin_unlock_irqrestore(&js->lock, flags);
    wake_up(&js->wq);
}

static bool jstream_feeder_ready(struct jtag_stream *js)
{
    bool r;

    spin_lock_irq(&js->lock);
    r = js->active && !js->fed_all && js->in_flight < JSTREAM_NBUF;
    spin_unlock_irq(&js->lock);
    return r;
}

static int jstream_feeder(void *arg)
{
    struct fpga_csi_dev *cd = arg;
    struct jtag_stream *js = &cd->js;
    u32 n, i, head;
    u64 now, lag;
    int ret;

    sched_set_fifo(current);
    while (!kthread_should_stop()) {
        if (wait_event_interruptible(js->wq, kthread_should_stop() || jstream_feeder_ready(js)))
            continue;
        if (kthread_should_stop())
            break;

        spin_lock_irq(&js->lock);
        if (js->stopping && js->committed >= js->queued_end) {
            js->fed_all = true;
            spin_unlock_irq(&js->lock);
            wake_up(&js->wq);
            continue;
        }
        n = js->chunk;
        if (js->stopping)
            n = min_t(u64, n, js->queued_end - js->committed);
        head = js->committed & (JSTREAM_RING - 1);
        for (i = 0; i < n; i++) {
            js->chunkbuf[i] = js->ring[(head + i) & (JSTREAM_RING - 1)];
            js->ring[(head + i) & (JSTREAM_RING - 1)] = 0;
        }
        /*
         * Nothing in flight for longer than the FIFO lasts: the SM has been
         * running idle slots of its own, so these words land later than
         * their slot numbers say. Re-anchor on when they will reach the FIFO.
         */
        now = ktime_get_ns();
        if (js->running && !js->in_flight && js->t_empty_ns) {
            lag = now - js->t_empty_ns;
            if (lag > (u64)(JPIO_FIFO - 2) * js->period_ns) {
                js->underruns++;
                js->t_slot0_ns = now + 30 * NSEC_PER_USEC - js->committed * js->period_ns;
                js->t_slot0_err_ns = max_t(u32, js->t_slot0_err_ns, 30 * NSEC_PER_USEC + js->period_ns);
            }
        }
        js->committed += n;
        js->in_flight++;
        js->t_empty_ns = 0;
        spin_unlock_irq(&js->lock);
        wake_up(&js->wq);           /* ring space for queuers */

        ret = pio_sm_xfer_data(cd->pio, cd->pio_sm, PIO_DIR_TO_SM, n * sizeof(u32),
                               js->chunkbuf, 0, jstream_dma_cb, js);
        if (ret) {
            spin_lock_irq(&js->lock);
            js->in_flight--;
            js->failed = true;
            js->fed_all = true;
            spin_unlock_irq(&js->lock);
            wake_up(&js->wq);
            dev_warn(cd->dev, "JTAG stream DMA submit failed (%d)\n", ret);
        }
    }
    return 0;
}

static void jtag_pio_teardown(struct fpga_csi_dev *cd)
{
    struct jtag_stream *js = &cd->js;

    if (js->feeder) {
        kthread_stop(js->feeder);
        js->feeder = NULL;
    }
    if (READ_ONCE(js->in_flight) > 0)
        wait_event_timeout(js->wq, !READ_ONCE(js->in_flight), msecs_to_jiffies(500));
    if (cd->pio) {
        pio_close(cd->pio);
        cd->pio = NULL;
    }
    if (cd->iobank) {
        jtag_pio_funcsel(cd, FS_RIO);
        iounmap(cd->iobank);
        cd->iobank = NULL;
    }
    vfree(js->ring);
    js->ring = NULL;
    kfree(js->chunkbuf);
    js->chunkbuf = NULL;
}

static int jtag_pio_init(struct fpga_csi_dev *cd)
{
    const uint tck = __ffs(cd->rio_tck), tms = __ffs(cd->rio_tms), tdi = __ffs(cd->rio_tdi);
    const u32 mask = cd->rio_tck | cd->rio_tms | cd->rio_tdi;
    struct jtag_stream *js = &cd->js;
    struct device_node *np;
    struct resource res;
    pio_sm_config c;
    u16 prog[JPIO_PROG_LEN];
    /*
     * Pinned at 0: jmp targets above are absolute, and whether RP1 firmware
     * relocates them on load is not documented.
     */
    pio_program_t pp = { .instructions = prog, .length = JPIO_PROG_LEN, .origin = 0 };
    uint off;
    int ret, sm;

    if (tms != tck + 1)
        return -EINVAL;

    js->ring = vzalloc(JSTREAM_RING * sizeof(u32));
    js->chunkbuf = kmalloc_array(JSTREAM_CHUNK_MAX, sizeof(u32), GFP_KERNEL);
    if (!js->ring || !js->chunkbuf) {
        ret = -ENOMEM;
        goto err;
    }

    np = of_find_compatible_node(NULL, NULL, "raspberrypi,rp1-gpio");
    if (!np) {
        ret = -ENODEV;
        goto err;
    }
    ret = of_address_to_resource(np, 0, &res);     /* reg[0] = IO_BANK0 */
    of_node_put(np);
    if (ret)
        goto err;
    cd->iobank = ioremap(res.start, IOB_CTRL(27) + 4);
    if (!cd->iobank) {
        ret = -ENOMEM;
        goto err;
    }

    cd->pio = pio_open();
    if (IS_ERR_OR_NULL(cd->pio)) {
        ret = cd->pio ? PTR_ERR(cd->pio) : -ENODEV;
        cd->pio = NULL;
        goto err;
    }
    sm = pio_claim_unused_sm(cd->pio, false);
    if (sm < 0) {
        ret = -EBUSY;
        goto err;
    }
    cd->pio_sm = sm;

    jtag_pio_program(prog);
    off = pio_add_program(cd->pio, &pp);
    if (off == PIO_ORIGIN_INVALID) {
        ret = -ENOSPC;
        goto err;
    }
    cd->pio_off = off;

    c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, off, off + JPIO_WRAP);
    sm_config_set_sideset(&c, 2, false, false);
    sm_config_set_sideset_pins(&c, tck);
    sm_config_set_out_pins(&c, tdi, 1);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    ret = pio_sm_init(cd->pio, sm, off, &c);
    if (ret < 0)
        goto err;
    pio_sm_set_pins_with_mask(cd->pio, sm, 0, mask);
    pio_sm_set_pindirs_with_mask(cd->pio, sm, mask, mask);
    pio_sm_exec(cd->pio, sm, pio_encode_set(pio_x, 0) | pio_encode_sideset(2, 0));

    ret = pio_gpio_init(cd->pio, tck);
    if (!ret)
        ret = pio_gpio_init(cd->pio, tms);
    if (!ret)
        ret = pio_gpio_init(cd->pio, tdi);
    cd->pio_fs = readl(cd->iobank + IOB_CTRL(tck)) & 0x1f;
    jtag_pio_funcsel(cd, FS_RIO);
    if (ret < 0)
        goto err;

    ret = pio_sm_config_xfer(cd->pio, sm, PIO_DIR_TO_SM, JSTREAM_CHUNK_MAX * sizeof(u32),
                             JSTREAM_NBUF);
    if (ret < 0)
        goto err;

    spin_lock_init(&js->lock);
    init_waitqueue_head(&js->wq);
    js->pio_delay = U32_MAX;
    js->feeder = kthread_run(jstream_feeder, cd, "fpga-csi-jtag");
    if (IS_ERR(js->feeder)) {
        ret = PTR_ERR(js->feeder);
        js->feeder = NULL;
        goto err;
    }
    return 0;

err:
    jtag_pio_teardown(cd);
    return ret;
}

/* Lazily bring the PIO up; false means stay on RIO. */
static bool jtag_pio_usable(struct fpga_csi_dev *cd)
{
    int ret;

    if (!jtag_pio || cd->pio_failed || !cd->jtag_rio_base || !jtag_rio)
        return false;
    if (cd->pio)
        return true;
    ret = jtag_pio_init(cd);
    if (ret) {
        dev_info(cd->dev, "PIO JTAG unavailable (%d), streams and paced batches stay on RIO\n", ret);
        cd->pio_failed = true;
        return false;
    }
    dev_info(cd->dev, "JTAG streams via RP1 PIO SM %d, TCK %d kHz\n",
             cd->pio_sm, 200000 / (2 * JPIO_HALF));
    return true;
}

/*
 * Register access other than through the stream: refused while another file
 * holds the lease, and while any stream runs, since the pins then belong to
 * the PIO. jtag_lock held.
 */
static int jtag_busy_locked(struct fpga_csi_dev *cd, struct csi_file_ctx *ctx)
{
    if (cd->jtag_owner && cd->jtag_owner != ctx)
        return -EBUSY;
    if (cd->js.active)
        return -EBUSY;
    return 0;
}

static inline u32 jstream_word(const struct csi_jtag_reg *r)
{
    return BIT(24) | ((u32)r->addr << 16) | r->value;
}

/* Copy words into the ring at slot..slot+n; js->lock held, space checked */
static void jstream_put_locked(struct jtag_stream *js, u64 slot, const u32 *w, u32 n)
{
    u32 i;

    for (i = 0; i < n; i++)
        js->ring[(slot + i) & (JSTREAM_RING - 1)] = w[i];
    js->queued_end = max(js->queued_end, slot + n);
}

static int jstream_stop_locked(struct fpga_csi_dev *cd, bool drain);

/*
 * Start a stream; jtag_lock held. @pre (optional) fills slots 0.. before the
 * SM runs. Returns -EAGAIN if the PIO is not usable, so callers can fall back.
 */
static int jstream_start_locked(struct fpga_csi_dev *cd, struct csi_file_ctx *owner,
                                u32 period_ns, u64 start_ns, u32 lead_ns,
                                const struct csi_jtag_reg *pre, u32 npre)
{
    struct jtag_stream *js = &cd->js;
    u64 cycles = div_u64(period_ns, JPIO_NS), t0, t1, now;
    u32 x, i, chunk;

    if (js->active)
        return -EBUSY;
    if (cycles < JPIO_FIXED || cycles > U32_MAX)
        return -EINVAL;
    if (npre > JSTREAM_QUEUE_MAX)
        return -E2BIG;
    /* the wait for start_ns holds jtag_lock */
    if (start_ns > ktime_get_ns() + 10 * NSEC_PER_SEC)
        return -EINVAL;
    if (!jtag_pio_usable(cd))
        return -EAGAIN;

    x = (u32)cycles - JPIO_FIXED;
    if (!lead_ns)
        lead_ns = JSTREAM_LEAD_NS;
    chunk = clamp_t(u64, div_u64(lead_ns, JSTREAM_NBUF * cycles * JPIO_NS),
                    JPIO_FIFO, JSTREAM_CHUNK_MAX);

    /* SM is disabled and parked on its pull at offset 0 between streams */
    if (x != js->pio_delay) {
        pio_sm_put(cd->pio, cd->pio_sm, x);
        pio_sm_exec(cd->pio, cd->pio_sm, pio_encode_pull(false, true) | pio_encode_sideset(2, 0));
        pio_sm_exec(cd->pio, cd->pio_sm, pio_encode_mov(pio_y, pio_osr) | pio_encode_sideset(2, 0));
        js->pio_delay = x;
    }

    /* the feeder leaves the ring alone while !active */
    memset(js->ring, 0, JSTREAM_RING * sizeof(u32));
    for (i = 0; i < npre; i++)
        js->ring[i] = jstream_word(&pre[i]);

    spin_lock_irq(&js->lock);
    js->owner = owner;
    js->period_ns = (u32)(cycles * JPIO_NS);
    js->chunk = chunk;
    js->committed = 0;
    js->queued_end = 0;
    js->in_flight = 0;
    js->underruns = 0;
    js->t_empty_ns = 0;
    js->t_slot0_ns = 0;
    js->t_slot0_err_ns = 0;
    js->stopping = false;
    js->fed_all = false;
    js->failed = false;
    js->running = false;
    js->queued_end = npre;
    js->active = true;
    spin_unlock_irq(&js->lock);

    jtag_rio_drain(cd);
    jtag_pio_funcsel(cd, cd->pio_fs);

    /* let the feeder queue the first chunk so the FIFO is full at slot 0 */
    wake_up(&js->wq);
    wait_event_timeout(js->wq, READ_ONCE(js->committed) || READ_ONCE(js->failed),
                       msecs_to_jiffies(50));
    if (READ_ONCE(js->failed) || !READ_ONCE(js->committed)) {
        jstream_stop_locked(cd, false);
        return -EIO;
    }
    usleep_range(100, 150);

    now = ktime_get_ns();
    if (start_ns > now + 300 * NSEC_PER_USEC)
        usleep_range((start_ns - now) / 1000 - 200, (start_ns - now) / 1000 - 150);
    while (ktime_get_ns() < start_ns)
        cpu_relax();

    t0 = ktime_get_ns();
    pio_sm_set_enabled(cd->pio, cd->pio_sm, true);
    t1 = ktime_get_ns();

    spin_lock_irq(&js->lock);
    js->t_slot0_ns = t0 + (t1 - t0) / 2;
    js->t_slot0_err_ns = (u32)min_t(u64, (t1 - t0) / 2, U32_MAX);
    js->running = true;
    spin_unlock_irq(&js->lock);
    return 0;
}

/*
 * Stop; jtag_lock held. drain: every queued write goes out first.
 * Otherwise writes not yet handed to DMA are dropped. Chunks already in
 * flight always complete: disabling the SM under a live DMA wedges the
 * channel until reboot.
 */
static int jstream_stop_locked(struct fpga_csi_dev *cd, bool drain)
{
    struct jtag_stream *js = &cd->js;
    u64 queued, end_ns, now;
    long t;
    int ret = 0;

    if (!js->active)
        return 0;

    spin_lock_irq(&js->lock);
    if (!drain)
        js->queued_end = js->committed;
    js->stopping = true;
    queued = js->queued_end > js->committed ? js->queued_end - js->committed : 0;
    spin_unlock_irq(&js->lock);
    wake_up(&js->wq);

    t = wait_event_timeout(js->wq,
                           READ_ONCE(js->fed_all) && !READ_ONCE(js->in_flight),
                           nsecs_to_jiffies((queued + JSTREAM_NBUF * js->chunk) * js->period_ns) +
                           msecs_to_jiffies(200));
    if (!t) {
        /* leave the SM and DMA alone; remove() closes the client */
        dev_warn(cd->dev, "JTAG stream did not drain, PIO disabled until reload\n");
        cd->pio_failed = true;
        ret = -ETIMEDOUT;
        goto out;
    }

    /* the last words sit in the FIFO; wait out the slot after them */
    spin_lock_irq(&js->lock);
    end_ns = js->underruns ? js->t_empty_ns + (JPIO_FIFO + 2ull) * js->period_ns
                           : jstream_slot_ns(js, js->committed + 1);
    end_ns = max(end_ns, js->t_empty_ns + (JPIO_FIFO + 2ull) * js->period_ns);
    spin_unlock_irq(&js->lock);
    now = ktime_get_ns();
    if (end_ns > now)
        usleep_range((end_ns - now) / 1000 + 1, (end_ns - now) / 1000 + 20);

    /*
     * Park on the pull with X = 0: an idle loop cut short leaves X nonzero,
     * and the next stream's first empty pull would take it for a write. An
     * exec'd jmp is not relocated, hence the absolute offset.
     */
    pio_sm_set_enabled(cd->pio, cd->pio_sm, false);
    pio_sm_exec(cd->pio, cd->pio_sm, pio_encode_set(pio_x, 0) | pio_encode_sideset(2, 0));
    pio_sm_exec(cd->pio, cd->pio_sm, pio_encode_jmp(cd->pio_off) | pio_encode_sideset(2, 0));

out:
    jtag_pio_funcsel(cd, FS_RIO);
    if (js->failed && !ret)
        ret = -EIO;
    if (ret) {
        jtag_reset_tap(cd);
        jtag_shift_ir(cd, 0x32);
    }
    spin_lock_irq(&js->lock);
    js->active = false;
    js->running = false;
    js->owner = NULL;
    spin_unlock_irq(&js->lock);
    wake_up(&js->wq);
    return ret;
}

/*
 * Queue writes. Without CSI_JTAG_STREAM_AT they follow the last queued write
 * (or the first slot not yet handed to DMA, whichever is later). Blocks while
 * the ring is full unless CSI_JTAG_STREAM_NONBLOCK. Called without jtag_lock.
 */
static int jstream_queue(struct fpga_csi_dev *cd, struct csi_file_ctx *ctx,
                         struct csi_jtag_stream_queue *q, const u32 *w)
{
    struct jtag_stream *js = &cd->js;
    u64 slot;
    int ret = 0;

    spin_lock_irq(&js->lock);
    for (;;) {
        if (!js->active || js->owner != ctx || js->stopping) {
            ret = js->active && js->owner != ctx ? -EBUSY : -ESHUTDOWN;
            break;
        }
        if (js->failed) {
            ret = -EIO;
            break;
        }
        if (q->flags & CSI_JTAG_STREAM_AT) {
            slot = q->slot;
            if (slot < js->committed) {
                ret = -ETIME;
                break;
            }
            if (slot < js->queued_end) {
                ret = -EINVAL;      /* would land on queued writes */
                break;
            }
        } else {
            slot = max(js->queued_end, js->committed);
        }
        if (slot + q->count - js->committed <= JSTREAM_RING) {
            jstream_put_locked(js, slot, w, q->count);
            q->slot = slot;
            q->t_first_ns = jstream_slot_ns(js, slot);
            break;
        }
        if (q->flags & CSI_JTAG_STREAM_NONBLOCK) {
            ret = -EAGAIN;
            break;
        }
        spin_unlock_irq(&js->lock);
        ret = wait_event_interruptible(js->wq, !READ_ONCE(js->active) || READ_ONCE(js->stopping) || READ_ONCE(js->failed) ||
                                       slot + q->count - READ_ONCE(js->committed) <= JSTREAM_RING);
        spin_lock_irq(&js->lock);
        if (ret)
            break;
    }
    spin_unlock_irq(&js->lock);
    return ret;
}

static void jstream_status(struct fpga_csi_dev *cd, struct csi_jtag_stream_status *s)
{
    struct jtag_stream *js = &cd->js;
    u64 now = ktime_get_ns();

    memset(s, 0, sizeof(*s));
    spin_lock_irq(&js->lock);
    s->active = js->active;
    if (js->active) {
        s->period_ns = js->period_ns;
        s->t_slot0_ns = js->t_slot0_ns;
        s->t_slot0_err_ns = js->t_slot0_err_ns;
        s->underruns = js->underruns;
        s->committed = js->committed;
        s->queued_end = js->queued_end;
        s->lead_slots = JSTREAM_NBUF * js->chunk;
        if (js->running && now > js->t_slot0_ns)
            s->slot_now = div_u64(now - js->t_slot0_ns, js->period_ns);
    }
    spin_unlock_irq(&js->lock);
}

/*
 * Paced batch on the stream: slots 0..count-1 from start, then drain.
 * 0: done via PIO. -EAGAIN: nothing clocked, use RIO. Other: failed part way.
 */
static int jtag_pio_batch(struct fpga_csi_dev *cd, struct csi_file_ctx *ctx,
                          const struct csi_jtag_reg *regs, struct csi_jtag_batch_paced *b)
{
    int ret;

    if (div_u64(b->period_ns, JPIO_NS) < JPIO_FIXED)
        return -EAGAIN;
    ret = jstream_start_locked(cd, ctx, b->period_ns, 0, 0, regs, b->count);
    if (ret)
        return ret == -EBUSY ? -EBUSY : -EAGAIN;
    b->t_first_ns = cd->js.t_slot0_ns;
    ret = jstream_stop_locked(cd, true);
    b->slips = cd->js.underruns;
    b->max_late_ns = cd->js.t_slot0_err_ns;
    if (ret)
        dev_warn(cd->dev, "PIO JTAG batch failed (%d)\n", ret);
    return ret;
}

/*
 * Busy-waits to each deadline with the JTAG lock held, as the unpaced batch
 * already does for the length of its bit-banging. ktime_get_ns() reads the
 * arch counter, so the schedule is crystal-timed; only the start of each scan
 * can be late (preemption, or stores backed up behind DRAM traffic).
 */
static long csi_jtag_batch_paced(struct fpga_csi_dev *cd, struct csi_file_ctx *ctx,
                                 void __user *arg)
{
    struct csi_jtag_batch_paced b;
    struct csi_jtag_reg *regs;
    u64 next, now, late;
    u32 i;
    long ret;

    if (copy_from_user(&b, arg, sizeof(b)))
        return -EFAULT;
    if (b.count == 0 || b.count > 1024 || b.period_ns > 100 * NSEC_PER_MSEC)
        return -EINVAL;
    regs = memdup_user((void __user *)(uintptr_t)b.regs_ptr,
                       b.count * sizeof(struct csi_jtag_reg));
    if (IS_ERR(regs))
        return PTR_ERR(regs);

    b.slips = 0;
    b.max_late_ns = 0;
    mutex_lock(&cd->jtag_lock);
    ret = jtag_busy_locked(cd, ctx);
    if (ret)
        goto out;
    ret = csi_jtag_attach_locked(ctx);
    if (ret)
        goto out;

    ret = jtag_pio_batch(cd, ctx, regs, &b);
    if (ret != -EAGAIN)
        goto out;
    ret = 0;

    next = ktime_get_ns();
    b.t_first_ns = next;
    for (i = 0; i < b.count; i++) {
        now = ktime_get_ns();
        if (next > now + 50 * NSEC_PER_USEC) {
            usleep_range((next - now) / 1000 - 20, (next - now) / 1000);
            now = ktime_get_ns();
        }
        while (now < next) {
            cpu_relax();
            now = ktime_get_ns();
        }
        late = now - next;
        if (late > b.max_late_ns)
            b.max_late_ns = (u32)min_t(u64, late, U32_MAX);
        if (b.period_ns && late > b.period_ns / 4) {
            b.slips++;
            next = now;
        }
        jtag_reg_write(cd, regs[i].addr, regs[i].value);
        next += b.period_ns;
    }
    jtag_rio_drain(cd);

out:
    mutex_unlock(&cd->jtag_lock);
    kfree(regs);
    if (!ret && copy_to_user(arg, &b, sizeof(b)))
        ret = -EFAULT;
    return ret;
}

static long csi_jtag_stream_start(struct fpga_csi_dev *cd, struct csi_file_ctx *ctx,
                                  void __user *arg)
{
    struct csi_jtag_stream_start s;
    struct csi_jtag_reg *regs = NULL;
    long ret;

    if (copy_from_user(&s, arg, sizeof(s)))
        return -EFAULT;
    if (s.count > JSTREAM_QUEUE_MAX)
        return -E2BIG;
    if (s.count) {
        regs = memdup_user((void __user *)(uintptr_t)s.regs_ptr,
                           s.count * sizeof(struct csi_jtag_reg));
        if (IS_ERR(regs))
            return PTR_ERR(regs);
    }

    mutex_lock(&cd->jtag_lock);
    ret = jtag_busy_locked(cd, ctx);
    if (!ret)
        ret = csi_jtag_attach_locked(ctx);
    if (!ret) {
        ret = jstream_start_locked(cd, ctx, s.period_ns, s.start_ns, s.lead_ns, regs, s.count);
        if (ret == -EAGAIN)
            ret = -EOPNOTSUPP;      /* no PIO: jtag_pio=0, RIO off, or init failed */
    }
    if (!ret) {
        s.period_ns = cd->js.period_ns;
        s.t_slot0_ns = cd->js.t_slot0_ns;
        s.t_slot0_err_ns = cd->js.t_slot0_err_ns;
        s.lead_slots = JSTREAM_NBUF * cd->js.chunk;
    }
    mutex_unlock(&cd->jtag_lock);
    kfree(regs);
    if (!ret && copy_to_user(arg, &s, sizeof(s)))
        ret = -EFAULT;
    return ret;
}

static long csi_jtag_stream_queue(struct fpga_csi_dev *cd, struct csi_file_ctx *ctx,
                                  void __user *arg)
{
    struct csi_jtag_stream_queue q;
    struct csi_jtag_reg *regs;
    u32 *w, i;
    long ret;

    if (copy_from_user(&q, arg, sizeof(q)))
        return -EFAULT;
    if (!q.count || q.count > JSTREAM_QUEUE_MAX)
        return -EINVAL;
    regs = memdup_user((void __user *)(uintptr_t)q.regs_ptr,
                       q.count * sizeof(struct csi_jtag_reg));
    if (IS_ERR(regs))
        return PTR_ERR(regs);
    w = kvmalloc_array(q.count, sizeof(*w), GFP_KERNEL);
    if (!w) {
        kfree(regs);
        return -ENOMEM;
    }
    for (i = 0; i < q.count; i++)
        w[i] = jstream_word(&regs[i]);
    kfree(regs);

    ret = jstream_queue(cd, ctx, &q, w);
    kvfree(w);
    if (!ret && copy_to_user(arg, &q, sizeof(q)))
        ret = -EFAULT;
    return ret;
}

static long csi_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    struct fpga_csi_dev *cd = csi_cd(f);

    switch (cmd) {
    case CSI_IOC_SET_FILTER: {
        struct csi_filter_cfg cfg;
        if (copy_from_user(&cfg, (void __user *)arg, sizeof(cfg))) return -EFAULT;
        cfg.vc &= 0x3;
        /* If user disables DT filter, keep a sensible default for later enable */
        if (!cfg.enable_dt_filter) {
            if (cd->ov_dt)
                cfg.dt = cd->ov_dt;             /* honor overlay default */
            else
                cfg.dt = 0x2B;                  /* default RAW10 */
        }
        cd->filter = cfg;
        return 0;
    }
    case CSI_IOC_GET_STATS: {
        struct csi_stats st = cd->stats;

        st.ch_irq_total = cd->ch_irq_total;
        st.ch_irq_fe = cd->ch_irq_fe;
        st.overflows_ring = atomic64_read(&cd->software_drops);
        if (copy_to_user((void __user *)arg, &st, sizeof(st))) return -EFAULT;
        return 0;
    }
    case CSI_IOC_GET_EVENTS: {
        struct csi_event_stats ev = {
            .irq_fs = cd->ch_irq_fs,
            .irq_fe = cd->ch_irq_fe,
            .irq_both = cd->ch_irq_both,
            .inferred_fe = cd->inferred_fe,
            .recoveries = cd->recovery_count,
            .no_buffer = cd->no_buffer_count,
            .inactive_irqs = cd->inactive_irq_count,
        };

        if (copy_to_user((void __user *)arg, &ev, sizeof(ev)))
            return -EFAULT;
        return 0;
    }
    case CSI_IOC_DBG_PHY: {
        struct csi_debug dbg;
        if (cd->regs.dphy) {
            dbg.phy_rx = readl(cd->regs.dphy + 0x48);
            dbg.stopstate = readl(cd->regs.dphy + 0x4C);
        } else {
            dbg.phy_rx = 0;
            dbg.stopstate = 0;
        }
        if (copy_to_user((void __user *)arg, &dbg, sizeof(dbg))) return -EFAULT;
        return 0;
    }
    case CSI_IOC_GET_LINK: {
        struct csi_link_info info = (struct csi_link_info){0};
        void __iomem *d = cd->regs.dphy;
        if (d) {
            info.dphy_version   = readl(d + 0x000);
            info.dphy_n_lanes   = readl(d + 0x004);
            info.dphy_resetn    = readl(d + 0x008);
            info.dphy_shutdownz = readl(d + 0x040);
            info.dphy_rstz      = readl(d + 0x044);
            info.dphy_phy_rx    = readl(d + 0x048);
            info.dphy_stopstate = readl(d + 0x04c);
        }
        info.csi2_status        = rd(cd, CSI2_STATUS);
        info.csi2_discards_overflow  = rd(cd, CSI2_DISCARDS_OVERFLOW);
        info.csi2_discards_inactive  = rd(cd, CSI2_DISCARDS_INACTIVE);
        info.csi2_discards_unmatched = rd(cd, CSI2_DISCARDS_UNMATCHED);
        info.csi2_discards_len_limit = rd(cd, CSI2_DISCARDS_LEN_LIMIT);
        info.mipic_cfg          = rd_mipic(cd, MIPIC_CFG);
        info.mipic_intr         = rd_mipic(cd, MIPIC_INTR);
        info.mipic_inte         = rd_mipic(cd, MIPIC_INTE);
        info.mipic_ints         = rd_mipic(cd, MIPIC_INTS);
        info.mipic_irq_total    = cd->mipic_irq_total;
        info.mipic_irq_dma      = cd->mipic_irq_dma;
        info.mipic_irq_host     = cd->mipic_irq_host;
        info.mipic_irq_other    = cd->mipic_irq_other;
        info.ch_irq_total       = cd->ch_irq_total;
        info.ch_irq_fe          = cd->ch_irq_fe;
        if (copy_to_user((void __user *)arg, &info, sizeof(info))) return -EFAULT;
        return 0;
    }
    case CSI_IOC_SET_GEOMETRY: {
        /* Geometry is fixed at probe; accept but ignore for compatibility. */
        return 0;
    }
    case CSI_IOC_RESET: {
        /* Reserved for future soft-reset semantics; currently a no-op. */
        return 0;
    }
    case CSI_IOC_GET_RING_INFO:
    {
        struct csi_ring_info info = {
            .ring_size  = PAGE_ALIGN(cd->ring.size),
            .span_bytes = cd->dma_span,
        };
        spin_lock_irq(&cd->ring.lock);
        info.head = cd->ring.wpos;
        info.tail = cd->ring.rpos;
        spin_unlock_irq(&cd->ring.lock);
        if (copy_to_user((void __user *)arg, &info, sizeof(info)))
            return -EFAULT;
        return 0;
    }
    case CSI_IOC_CONSUME_BYTES:
    {
        u32 n;
        if (copy_from_user(&n, (void __user *)arg, sizeof(n))) return -EFAULT;

        spin_lock_irq(&cd->ring.lock);
        {
            size_t used = r_used(&cd->ring);
            if (n > used) { spin_unlock_irq(&cd->ring.lock); return -EINVAL; }
            cd->ring.rpos = (cd->ring.rpos + n) & (cd->ring.size - 1);
            cd->stats.bytes_out += n;
        }
        spin_unlock_irq(&cd->ring.lock);

        return 0;
    }

case CSI_IOC_JTAG_SETUP: {
        int ret;
        struct csi_file_ctx *ctx = csi_ctx(f);

        mutex_lock(&cd->jtag_lock);
        ret = csi_jtag_attach_locked(ctx);
        mutex_unlock(&cd->jtag_lock);
        return ret;
    }

    case CSI_IOC_JTAG_RELEASE: {
        struct csi_file_ctx *ctx = csi_ctx(f);

        mutex_lock(&cd->jtag_lock);
        if (cd->js.active && cd->js.owner == ctx)
            jstream_stop_locked(cd, false);
        if (ctx->jtag_attached) {
            ctx->jtag_attached = false;
            csi_jtag_detach_locked(cd);
        }
        mutex_unlock(&cd->jtag_lock);
        return 0;
    }

    case CSI_IOC_JTAG_REG_WRITE: {
        struct csi_jtag_reg r;
        int ret = 0;

        if (copy_from_user(&r, (void __user *)arg, sizeof(r)))
            return -EFAULT;

        mutex_lock(&cd->jtag_lock);

        /* THE MISSING LINK: Fail if someone else holds the lease */
        ret = jtag_busy_locked(cd, csi_ctx(f));
        if (!ret) {
            ret = csi_jtag_attach_locked(csi_ctx(f));
            if (!ret) {
                ret = jtag_reg_write(cd, r.addr, r.value);
                jtag_rio_drain(cd);
            }
        }

        mutex_unlock(&cd->jtag_lock);
        return ret;
    }

    case CSI_IOC_JTAG_REG_READ: {
        struct csi_jtag_reg r;
        int ret = 0;
        u16 val;

        if (copy_from_user(&r, (void __user *)arg, sizeof(r)))
            return -EFAULT;

        mutex_lock(&cd->jtag_lock);

        /* THE MISSING LINK: Fail if someone else holds the lease */
        ret = jtag_busy_locked(cd, csi_ctx(f));
        if (!ret) {
            ret = csi_jtag_attach_locked(csi_ctx(f));
            if (!ret)
                ret = jtag_reg_read(cd, r.addr, &val);
        }

        mutex_unlock(&cd->jtag_lock);

        if (ret)
            return ret;

        r.value = val;
        if (copy_to_user((void __user *)arg, &r, sizeof(r)))
            return -EFAULT;
        return 0;
    }

    /* --- Locked transactions IOCTL --- */
    case CSI_IOC_JTAG_ACQUIRE_LEASE: {
        int ret = 0;
        struct csi_file_ctx *ctx = csi_ctx(f);

        mutex_lock(&cd->jtag_lock);
        if ((cd->jtag_owner && cd->jtag_owner != ctx) || cd->js.active) {
            ret = -EBUSY;
        } else {
            cd->jtag_owner = ctx;
        }
        mutex_unlock(&cd->jtag_lock);
        return ret;
    }

    case CSI_IOC_JTAG_RELEASE_LEASE: {
        struct csi_file_ctx *ctx = csi_ctx(f);

        mutex_lock(&cd->jtag_lock);
        if (cd->jtag_owner == ctx) {
            cd->jtag_owner = NULL;
        }
        mutex_unlock(&cd->jtag_lock);
        return 0;
    }

    /* --- BATCH IOCTL --- */
    case CSI_IOC_JTAG_BATCH_WRITE: {
        struct csi_file_ctx *ctx = csi_ctx(f);
        struct csi_jtag_batch batch;
        struct csi_jtag_reg small_regs[8];
        struct csi_jtag_reg *regs;
        bool regs_heap = false;
        size_t regs_bytes;
        u32 i;
        int ret = 0;

        if (copy_from_user(&batch, (void __user *)arg, sizeof(batch)))
            return -EFAULT;

        if (batch.count == 0 || batch.count > 1024)
            return -EINVAL;

        regs_bytes = batch.count * sizeof(struct csi_jtag_reg);

        /*
         * Frequency programming normally sends only a few registers.
         * Avoid memdup_user()/kmalloc()/kfree() on that hot path.
         */
        if (batch.count <= ARRAY_SIZE(small_regs)) {
            regs = small_regs;
            if (copy_from_user(regs,
                               (void __user *)(uintptr_t)batch.regs_ptr,
                               regs_bytes))
                return -EFAULT;
        } else {
            regs = memdup_user((void __user *)(uintptr_t)batch.regs_ptr,
                               regs_bytes);
            if (IS_ERR(regs))
                return PTR_ERR(regs);
            regs_heap = true;
        }

        mutex_lock(&cd->jtag_lock);

        /* Enforce lease protection */
        ret = jtag_busy_locked(cd, ctx);
        if (ret)
            goto batch_out;

        ret = csi_jtag_attach_locked(ctx);
        if (!ret) {
            for (i = 0; i < batch.count; i++) {
                ret = jtag_reg_write(cd, regs[i].addr, regs[i].value);
                if (ret)
                    break;
                if (batch.delay_us) {
                    jtag_rio_drain(cd);
                    udelay(batch.delay_us);
                }
            }
            jtag_rio_drain(cd);
        }

    batch_out:
        mutex_unlock(&cd->jtag_lock);
        if (regs_heap)
            kfree(regs);
        return ret;
    }

    case CSI_IOC_JTAG_BATCH_PACED:
        return csi_jtag_batch_paced(cd, csi_ctx(f), (void __user *)arg);

    case CSI_IOC_JTAG_STREAM_START:
        return csi_jtag_stream_start(cd, csi_ctx(f), (void __user *)arg);

    case CSI_IOC_JTAG_STREAM_QUEUE:
        return csi_jtag_stream_queue(cd, csi_ctx(f), (void __user *)arg);

    case CSI_IOC_JTAG_STREAM_STATUS: {
        struct csi_jtag_stream_status st;

        jstream_status(cd, &st);
        return copy_to_user((void __user *)arg, &st, sizeof(st)) ? -EFAULT : 0;
    }

    case CSI_IOC_JTAG_STREAM_STOP: {
        __u32 flags;
        int ret;

        if (get_user(flags, (__u32 __user *)arg))
            return -EFAULT;
        mutex_lock(&cd->jtag_lock);
        if (!cd->js.active)
            ret = 0;
        else if (cd->js.owner != csi_ctx(f))
            ret = -EBUSY;
        else
            ret = jstream_stop_locked(cd, flags & CSI_JTAG_STREAM_DRAIN);
        mutex_unlock(&cd->jtag_lock);
        return ret;
    }

    default:
        return -ENOTTY;
    }
}

/* -------- File ops -------- */

static int csi_open(struct inode *inode, struct file *f)
{
    struct miscdevice *mdev = f->private_data;
    struct fpga_csi_dev *cd = container_of(mdev, struct fpga_csi_dev, miscdev);
    struct csi_file_ctx *ctx;

    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return -ENOMEM;

    ctx->mdev = mdev;
    ctx->cd   = cd;
    ctx->jtag_attached = false;

    f->private_data = ctx;
    return 0;
}

static void csi_jtag_detach_locked(struct fpga_csi_dev *cd)
{
    if (atomic_dec_and_test(&cd->jtag_users))
        jtag_release_locked(cd);
}

static int csi_release(struct inode *inode, struct file *f)
{
    struct csi_file_ctx *ctx = csi_ctx(f);
    struct fpga_csi_dev *cd;

    if (!ctx)
        return 0;

    cd = ctx->cd;

    mutex_lock(&cd->jtag_lock);

    if (cd->js.active && cd->js.owner == ctx)
        jstream_stop_locked(cd, false);

    /* Drop lease if this process owned it */
    if (cd->jtag_owner == ctx) {
        cd->jtag_owner = NULL;
    }

    if (ctx->jtag_attached) {
        ctx->jtag_attached = false;
        csi_jtag_detach_locked(cd);
    }
    mutex_unlock(&cd->jtag_lock);

    kfree(ctx);
    return 0;
}

static int csi_jtag_attach_locked(struct csi_file_ctx *ctx)
{
    int ret;

    if (ctx->jtag_attached)
        return 0;

    if (atomic_inc_return(&ctx->cd->jtag_users) == 1) {
        ret = jtag_setup_locked(ctx->cd);
        if (ret) {
            atomic_dec(&ctx->cd->jtag_users);
            return ret;
        }
    }
    ctx->jtag_attached = true;
    return 0;
}


static const struct file_operations csi_fops = {
    .owner          = THIS_MODULE,
    .open           = csi_open,
    .release        = csi_release,
    .read           = csi_read,
    .poll           = csi_poll,
    .unlocked_ioctl = csi_ioctl,
    .compat_ioctl   = compat_ptr_ioctl,
    .mmap           = csi_mmap,
    .llseek         = noop_llseek,
};

/* -------- Probe/remove -------- */

static int fpga_csi_probe(struct platform_device *pdev)
{
    struct fpga_csi_dev *cd;
    struct resource *res;
    int ret;
    u32 ov_bpl   = 0;
    u32 ov_lines = 0;

    cd = devm_kzalloc(&pdev->dev, sizeof(*cd), GFP_KERNEL);
    if (!cd)
        return -ENOMEM;
    cd->dev = &pdev->dev;

    /* Read overlay defaults (optional) */
    of_property_read_u32(pdev->dev.of_node, "acme,raw-bits",
                         &cd->ov_raw_bits); /* 8 or 10 */
    of_property_read_u32(pdev->dev.of_node, "acme,dt",
                         &cd->ov_dt);       /* 0x2A/0x2B */

    /* Clocks */
    cd->clk_core = of_clk_get(pdev->dev.of_node, 0);
    if (IS_ERR(cd->clk_core))
        return dev_err_probe(&pdev->dev, PTR_ERR(cd->clk_core),
                             "Failed to get clock[0] (core)\n");

    cd->clk_phy = of_clk_get(pdev->dev.of_node, 1);
    if (IS_ERR(cd->clk_phy)) {
        if (PTR_ERR(cd->clk_phy) == -ENOENT ||
            PTR_ERR(cd->clk_phy) == -EINVAL) {
            dev_warn(&pdev->dev,
                     "No clock[1] (dphy) in DT; treating as optional");
            cd->clk_phy = NULL;
        } else {
            ret = dev_err_probe(&pdev->dev, PTR_ERR(cd->clk_phy),
                                "Failed to get clock[1] (dphy)\n");
            clk_put(cd->clk_core);
            return ret;
        }
    }

    ret = clk_prepare_enable(cd->clk_core);
    if (ret) {
        clk_put(cd->clk_core);
        if (cd->clk_phy)
            clk_put(cd->clk_phy);
        return ret;
    }
    ret = devm_add_action_or_reset(&pdev->dev,
                                   clk_disable_unprepare_put,
                                   cd->clk_core);
    if (ret)
        return ret;

    if (cd->clk_phy) {
        ret = clk_prepare_enable(cd->clk_phy);
        if (ret)
            return ret;
        ret = devm_add_action_or_reset(&pdev->dev,
                                       clk_disable_unprepare_put,
                                       cd->clk_phy);
        if (ret)
            return ret;
    }

    /* Optional resets */
    cd->rst_core =
        devm_reset_control_get_optional_exclusive(&pdev->dev, "core");
    if (IS_ERR(cd->rst_core))
        return dev_err_probe(&pdev->dev, PTR_ERR(cd->rst_core),
                             "Failed to get core reset\n");
    if (cd->rst_core) {
        reset_control_deassert(cd->rst_core);
        ret = devm_add_action_or_reset(&pdev->dev,
                                       reset_assert_action,
                                       cd->rst_core);
        if (ret)
            return ret;
    } else {
        dev_warn(&pdev->dev,
                 "No core reset in DT; proceeding without it");
    }

    cd->rst_phy =
        devm_reset_control_get_optional_exclusive(cd->dev, "dphy");
    if (IS_ERR(cd->rst_phy))
        return dev_err_probe(&pdev->dev, PTR_ERR(cd->rst_phy),
                             "Failed to get dphy reset\n");
    if (cd->rst_phy) {
        reset_control_deassert(cd->rst_phy);
        ret = devm_add_action_or_reset(&pdev->dev,
                                       reset_assert_action,
                                       cd->rst_phy);
        if (ret)
            return ret;
    } else {
        dev_warn(&pdev->dev,
                 "No dphy reset in DT; proceeding without it");
    }

    /* Map reg windows; require reg[0] and reg[2] */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
        return -ENODEV;
    cd->regs.csi2 = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(cd->regs.csi2))
        return PTR_ERR(cd->regs.csi2);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if (res)
        cd->regs.dphy = devm_ioremap(&pdev->dev, res->start,
                                     resource_size(res));

    res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
    if (!res)
        return -ENODEV;
    cd->regs.mipic = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(cd->regs.mipic))
        return PTR_ERR(cd->regs.mipic);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
    if (res)
        cd->regs.fe = devm_ioremap(&pdev->dev, res->start,
                                   resource_size(res));

    dev_info(cd->dev,
             "reg0 CSI2=%p, reg1 DPHY=%p, reg2 MIPIC=%p, reg3 FE=%p",
             cd->regs.csi2, cd->regs.dphy,
             cd->regs.mipic, cd->regs.fe);

    cd->irq = platform_get_irq(pdev, 0);
    if (cd->irq < 0)
        return cd->irq;

    /* Userspace ring */
    cd->ring.size = RING_SIZE;
    cd->ring.data = vmalloc_user(cd->ring.size);
    if (!cd->ring.data)
        return -ENOMEM;
    spin_lock_init(&cd->ring.lock);
    init_waitqueue_head(&cd->ring.wq_read);
    spin_lock_init(&cd->dma_lock);
    mutex_init(&cd->read_lock);
    mutex_init(&cd->jtag_lock);
    atomic_set(&cd->jtag_users, 0);
    atomic_set(&cd->stopping, 0);
    atomic64_set(&cd->software_drops, 0);
    cd->active_idx = DMA_BUF_NONE;
    cd->next_idx = DMA_BUF_NONE;

    cd->copy_wq = alloc_ordered_workqueue("%s-copy",
                                          WQ_HIGHPRI | WQ_MEM_RECLAIM,
                                          dev_name(cd->dev));
    if (!cd->copy_wq) {
        vfree(cd->ring.data);
        return -ENOMEM;
    }

    /* Defaults */
    cd->filter.enable_vc_filter = 0;
    cd->filter.vc = 0;
    cd->filter.enable_dt_filter = 0;              /* off by default */
    if (cd->ov_dt)
        cd->filter.dt = cd->ov_dt;                /* preload from overlay */
    else if (cd->ov_raw_bits == 8)
        cd->filter.dt = 0x2A;
    else
        cd->filter.dt = 0x2B;

    /* Geometry defaults: from overlay when present, else 1024x1024 */
    of_property_read_u32(pdev->dev.of_node,
                         "acme,bytes-per-line", &ov_bpl);
    of_property_read_u32(pdev->dev.of_node,
                         "acme,lines", &ov_lines);

    if (ov_bpl < 64) {
        dev_warn(&pdev->dev,
                 "acme,bytes-per-line missing/invalid (%u), defaulting to %u\n",
                 ov_bpl, CSI_DEFAULT_BYTES_PER_LINE);
        ov_bpl = CSI_DEFAULT_BYTES_PER_LINE;
    }
    if (ov_lines == 0) {
        dev_warn(&pdev->dev,
                 "acme,lines missing/zero, defaulting to %u\n",
                 CSI_DEFAULT_LINES);
        ov_lines = CSI_DEFAULT_LINES;
    }

    cd->geom.bytes_per_line = ov_bpl;
    cd->geom.lines          = ov_lines;
    cd->dma_span            = compute_span_bytes(&cd->geom);

    dev_info(&pdev->dev,
             "geometry default from DT/driver: bytes_per_line=%u, lines=%u, span=%zu\n",
             cd->geom.bytes_per_line, cd->geom.lines, cd->dma_span);

    pm_runtime_enable(&pdev->dev);
    ret = pm_runtime_get_sync(&pdev->dev);
    if (ret < 0) {
        pm_runtime_put_noidle(&pdev->dev);
        pm_runtime_disable(&pdev->dev);
        destroy_workqueue(cd->copy_wq);
        cd->copy_wq = NULL;
        vfree(cd->ring.data);
        return ret;
    }

    /*
     * Auto-apply the overlay/driver default geometry at probe time.
     * First stop and acknowledge any state retained from an earlier module
     * instance.  Geometry setup allocates every DMA buffer and configures the
     * channel, but deliberately leaves DMA and IRQ delivery off.
     */
    csi_hw_quiesce(cd);
    ret = csi_apply_geometry(cd, &cd->geom);
    if (ret) {
        dev_err(&pdev->dev,
                "failed to apply default geometry (bytes_per_line=%u, lines=%u): %d\n",
                cd->geom.bytes_per_line, cd->geom.lines, ret);
        goto err_dma;
    }

    /* The hard handler may run immediately.  The ring, DMA pool, workqueue,
     * and all ownership state are valid at this point. */
    ret = devm_request_irq(&pdev->dev, cd->irq, fpga_csi_irq,
                           0, DRV_NAME, cd);
    if (ret) {
        dev_err(&pdev->dev, "irq request failed: %d", ret);
        goto err_dma;
    }

    ret = csi_enable_streaming(cd);
    if (ret) {
        dev_err(&pdev->dev, "failed to enable streaming: %d", ret);
        goto err_irq;
    }

    cd->miscdev.minor = MISC_DYNAMIC_MINOR;
    cd->miscdev.name  = DEV_NAME;
    cd->miscdev.fops  = &csi_fops;
    cd->miscdev.mode  = 0666;

    ret = misc_register(&cd->miscdev);
    if (ret) {
        dev_err(&pdev->dev, "misc register failed: %d", ret);
        goto err_irq;
    }

    platform_set_drvdata(pdev, cd);
    dev_info(&pdev->dev,
             "mapped reg[0..3], irq=%d; /dev/%s ready with geometry %u x %u (span=%zu)",
             cd->irq, DEV_NAME,
             cd->geom.bytes_per_line, cd->geom.lines, cd->dma_span);
    return 0;

err_irq:
    csi_stop(cd);
    devm_free_irq(&pdev->dev, cd->irq, cd);
err_dma:
    flush_workqueue(cd->copy_wq);
    destroy_workqueue(cd->copy_wq);
    cd->copy_wq = NULL;
    free_dma_buffers(cd);
    vfree(cd->ring.data);
    pm_runtime_put_sync(&pdev->dev);
    pm_runtime_disable(&pdev->dev);
    return ret;
}

static void fpga_csi_remove(struct platform_device *pdev)
{
    struct fpga_csi_dev *cd = platform_get_drvdata(pdev);

    /* Prevent new opens before stopping the producer. */
    misc_deregister(&cd->miscdev);

    mutex_lock(&cd->jtag_lock);
    jstream_stop_locked(cd, false);
    jtag_pio_teardown(cd);
    mutex_unlock(&cd->jtag_lock);

    /* Quiesce the hardware before synchronizing the hard IRQ. */
    csi_stop(cd);

    /* Explicitly synchronize and release the IRQ before freeing DMA/ring memory. */
    devm_free_irq(&pdev->dev, cd->irq, cd);

    /* No new copy work can be queued after free_irq(). */
    flush_workqueue(cd->copy_wq);
    destroy_workqueue(cd->copy_wq);
    cd->copy_wq = NULL;

    free_dma_buffers(cd);
    vfree(cd->ring.data);
    pm_runtime_put_sync(&pdev->dev);
    pm_runtime_disable(&pdev->dev);
}

/* -------- OF/driver glue -------- */

static const struct of_device_id fpga_csi_of_match[] = {
    { .compatible = "acme,fpga-csi-rx" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpga_csi_of_match);

static struct platform_driver fpga_csi_driver = {
    .driver = {
        .name           = DRV_NAME,
        .of_match_table = fpga_csi_of_match,
    },
    .probe  = fpga_csi_probe,
    .remove = fpga_csi_remove,
};
module_platform_driver(fpga_csi_driver);

MODULE_AUTHOR("Martin McCormick, Scale RF Inc.");
MODULE_DESCRIPTION("RP1 CSI-2 RAW8/RAW10 DMA char device (/dev/csi_stream0) "
                   "with two-deep DMA queue and ordered copy pipeline");
MODULE_LICENSE("GPL");
