// fpga-csi.c - CSI-2 RAW8/RAW10 char device for Raspberry Pi 5 (RP1, Cam1)
//
// Policy / high-level behavior:
//
//  ? Geometry is fixed at probe from DT/driver defaults:
//      - acme,bytes-per-line
//      - acme,lines
//    If missing/invalid, we fall back to 1024x1024.
//    CSI_IOC_SET_GEOMETRY is kept only for ABI compatibility and is a no-op.
//  ? RAW8 vs RAW10 is inferred from:
//      - DT overlay (acme,raw-bits / acme,dt), or
//      - userspace DT filter (CSI_IOC_SET_FILTER).
//    RAW8 enables CH_CTRL_PACK_BYTES; RAW10 leaves it clear.
//  ? CSI2-DMA runs as a single channel:
//
//      - Per-span DMA into coherent buffers (DMA_BUF_COUNT).
//      - Each FE/FE_ACK event pushes one span into a userspace byte ring,
//        then immediately re-arms the next buffer.
//      - Discard/overflow conditions trigger channel recovery + re-arm,
//        even if FE never arrives.
//  ? No explicit STOPSTATE wait or watchdog.
//  ? Userspace typically reads from /dev/csi_stream0 via read(), poll(), or
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

#include "fpga_csi.h"

#define DRV_NAME "fpga-csi"
#define DEV_NAME "csi_stream0"

/* -------- Module params -------- */

static bool drop_oldest;
module_param(drop_oldest, bool, 0444);
MODULE_PARM_DESC(drop_oldest,
                 "Drop oldest data when ring is full (default: 0=block)");

/* -------- Userspace byte ring -------- */

#define RING_ORDER   9      /* 2 MiB */
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
    wait_queue_head_t wq_space;        /* producer waits for space */
};

static inline size_t r_used(const struct byte_ring *r)
{
    return (r->wpos - r->rpos) & (r->size - 1);
}

static inline size_t r_space(const struct byte_ring *r)
{
    return r->size - 1 - r_used(r);
}

/*
 * Internal helper: copy from src into the ring, honoring drop_oldest.
 *
 * When drop_oldest==false:
 *   - May block until enough contiguous space is available for @len bytes.
 *   - Returns number of bytes actually written (==len unless interrupted).
 *
 * When drop_oldest==true:
 *   - On full ring, discards half the ring (amortizes wakeups) and
 *     increments *overflows.
 */
static size_t r_write(struct byte_ring *r, const u8 *src, size_t len, bool allow_drop, u64 *overflows)
{
    size_t written = 0;
    while (len) {
        size_t space = r_space(r);
        if (!space) {
            if (allow_drop) {
                size_t give = r->size >> 1;      /* amortize wakeups */
                r->rpos = (r->rpos + give) & (r->size - 1);
                if (overflows) (*overflows)++;
                continue;
            }
            break;
        }

        size_t mask = r->size - 1;
        size_t wpos = r->wpos;
        size_t chunk = min(len, min(space, r->size - (wpos & mask)));

        memcpy(r->data + (wpos & mask), src, chunk);
        src    += chunk;
        len    -= chunk;
        written += chunk;
        r->wpos = (wpos + chunk) & mask;
    }
    if (written)
        wake_up_interruptible(&r->wq_read);
    return written;
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
#define MIPIC_INTS           0x034 /* Masked Status (R) / Clear (W1C) */
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
#define DMA_BUF_COUNT   4

struct dma_buf {
    void   *cpu;
    dma_addr_t dma;
    size_t  size;
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

    struct dma_buf       dbuf[DMA_BUF_COUNT];
    u32                  cur_idx;
    size_t               dma_span;     /* bytes pushed to ring per FE span */

    struct csi_filter_cfg filter;      /* current VC/DT filter */
    struct csi_geometry   geom;        /* current geometry */

    /* overlay defaults (read at probe): */
    u32                  ov_raw_bits;  /* 8 or 10; 0 = "unknown" */
    u32                  ov_dt;        /* 0x2A/0x2B when provided, else 0 */

    struct csi_stats      stats;
    u32                   last_frame_id;

    /* Diagnostics */
    u64                   mipic_irq_total;
    u64                   mipic_irq_dma;
    u64                   mipic_irq_host;
    u64                   mipic_irq_other;
    u64                   ch_irq_total;
    u64                   ch_irq_fe;

    /* JTAG / FPGA register access via ECP5 USER1 (ER1) */
    struct gpio_desc     *jtag_tck;
    struct gpio_desc     *jtag_tms;
    struct gpio_desc     *jtag_tdi;
    struct gpio_desc     *jtag_tdo;
    bool                  jtag_pins_acquired;
    bool                  jtag_ready;
    bool                  jtag_write_gpio_cansleep;
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
    u32 mbps = 640; /* default lane rate */
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
    u8 code = 0b001000; /* ~640 Mbps default */

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
     *  - FE/FE_ACK used as span boundary and frame counter.
     *  - PACK_LINE groups by line; AUTO_ARM advances automatically.
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

    writel(ctrl | CH_CTRL_DMA_EN, base + CH_CTRL);
}

/*
 * Program one CSI2-DMA channel for the next span.
 *
 * The HW requires CH_ADDR1 (upper bits) to be written before CH_ADDR0,
 * and CH_ADDR0 write latches & arms the channel.
 */
static void csi_dma_arm(struct fpga_csi_dev *cd, u32 ch, u32 buf_idx)
{
    void __iomem *base = ch_base(cd->regs.csi2, ch);
    u64 addr = cd->dbuf[buf_idx].dma;

    /* Address: CH_ADDR1 then CH_ADDR0 (ADDR0 last latches/arms) */
    writel((u32)(addr >> 36),                base + CH_ADDR1);
    wmb();
    writel((u32)((addr >> 4) & 0xffffffffu), base + CH_ADDR0);
    wmb();

    /* Make sure DMA_EN stays asserted */
    {
        u32 ctrl = readl(base + CH_CTRL);

        if (!(ctrl & CH_CTRL_DMA_EN))
            writel(ctrl | CH_CTRL_DMA_EN, base + CH_CTRL);
    }
}

/* Push current DMA span to userspace ring */

static void push_dma_to_ring(struct fpga_csi_dev *cd, u32 buf_idx, size_t nbytes)
{
    const u8 *src = cd->dbuf[buf_idx].cpu;

    spin_lock(&cd->ring.lock);
    if (!drop_oldest) {
        while (r_space(&cd->ring) < nbytes) {
            spin_unlock(&cd->ring.lock);
            if (wait_event_interruptible(cd->ring.wq_space, r_space(&cd->ring) >= nbytes))
                return; /* interrupted */
            spin_lock(&cd->ring.lock);
        }
    }
    /* Track ring drops (if enabled) separately from CSI overflows. */
    r_write(&cd->ring, src, nbytes, drop_oldest,
            drop_oldest ? &cd->stats.overflows_ring : NULL);
    spin_unlock(&cd->ring.lock);
}

/* ---- Channel recovery (no FE) ---- */

static void csi_channel_recover(struct fpga_csi_dev *cd, u32 ch)
{
    void __iomem *base = ch_base(cd->regs.csi2, ch);
    u32 ctrl = readl(base + CH_CTRL);

    /*
     * Nudge HW to realign to an FE boundary and ensure DMA stays on.
     * FLUSH_FE drops current in-flight frame; FORCE kicks the state
     * machine to restart.
     */
    writel(ctrl | CH_CTRL_FLUSH_FE | CH_CTRL_FORCE, base + CH_CTRL);
    wmb();

    /* Re-assert DMA_EN if it dropped */
    ctrl = readl(base + CH_CTRL);
    if (!(ctrl & CH_CTRL_DMA_EN))
        writel(ctrl | CH_CTRL_DMA_EN, base + CH_CTRL);

    /* Re-post the current buffer so HW is armed again */
    csi_dma_arm(cd, ch, cd->cur_idx);
}

/* ---- IRQs ---- */

static irqreturn_t fpga_csi_irq_thread(int irq, void *data)
{
    struct fpga_csi_dev *cd = data;
    void __iomem *ch_reg_base = ch_base(cd->regs.csi2, CSI_DMA_CHANNEL);
    u32 ints;
    u32 status;
    bool fe_event;
    bool fatal_err;

    cd->mipic_irq_total++;

    ints = rd_mipic(cd, MIPIC_INTS);
    if (!ints)
        return IRQ_NONE;

    if (ints & MIPIC_INT_CSI_DMA)
        cd->mipic_irq_dma++;
    if (ints & MIPIC_INT_CSI_HOST)
        cd->mipic_irq_host++;
    if (ints & ~(MIPIC_INT_CSI_DMA | MIPIC_INT_CSI_HOST))
        cd->mipic_irq_other++;

    /* Clear summary first (W1C) */
    wr_mipic(cd, MIPIC_INTS, ints);

    /* If no CSI-DMA summary, still try to recover on HOST events */
    if (!(ints & MIPIC_INT_CSI_DMA)) {
        if (ints & MIPIC_INT_CSI_HOST)
            csi_channel_recover(cd, CSI_DMA_CHANNEL);
        return IRQ_HANDLED;
    }

    /* Read CSI2 status and clear (W1C) */
    status = rd(cd, CSI2_STATUS);
    if (status)
        cd->ch_irq_total++;
    wr(cd, CSI2_STATUS, status);

    /* Error/discard accounting */
    if (status & CSI2_STATUS_IRQ_OVERFLOW)
        cd->stats.overflows++;
    if (status & CSI2_STATUS_IRQ_DISCARD_OVERFLOW)
        stats_add_discard(cd, CSI2_DISCARDS_OVERFLOW);
    if (status & CSI2_STATUS_IRQ_DISCARD_LEN_LIMIT)
        stats_add_discard(cd, CSI2_DISCARDS_LEN_LIMIT);
    if (status & CSI2_STATUS_IRQ_DISCARD_UNMATCHED)
        stats_add_discard(cd, CSI2_DISCARDS_UNMATCHED);
    if (status & CSI2_STATUS_IRQ_DISCARD_INACTIVE)
        stats_add_discard(cd, CSI2_DISCARDS_INACTIVE);

    fe_event = status & (CSI2_STATUS_IRQ_FE_ACK(CSI_DMA_CHANNEL)); // | CSI2_STATUS_IRQ_FE(CSI_DMA_CHANNEL));
    fatal_err = status & (CSI2_STATUS_IRQ_OVERFLOW |
                          CSI2_STATUS_IRQ_DISCARD_OVERFLOW |
                          CSI2_STATUS_IRQ_DISCARD_LEN_LIMIT |
                          CSI2_STATUS_IRQ_DISCARD_UNMATCHED |
                          CSI2_STATUS_IRQ_DISCARD_INACTIVE);

    if (fe_event) {
        /* Update frame count from FE_FRAME_ID */
        u32 fid = readl(ch_reg_base + CH_FE_FRAME_ID);

        if (fid != cd->last_frame_id) {
            cd->stats.frame_count++;
            cd->last_frame_id = fid;
        }
        cd->ch_irq_fe++;

        /* Transfer current DMA span and arm next */
        {
            u32 idx = cd->cur_idx;
            size_t n = cd->dma_span;

            if (n > cd->dbuf[idx].size)
                n = cd->dbuf[idx].size;

            push_dma_to_ring(cd, idx, n);
            cd->stats.dma_bytes += n;

            /* Advance the tracker to the currently active buffer */
            cd->cur_idx = (idx + 1) % DMA_BUF_COUNT;
            
            /* Queue the NEXT buffer into the shadow register */
            u32 shadow_idx = (cd->cur_idx + 1) % DMA_BUF_COUNT;
            csi_dma_arm(cd, CSI_DMA_CHANNEL, shadow_idx);
        }

        /*
         * If it was both FE and fatal, we've already advanced and
         * re-armed; nothing else to do.
         */
        return IRQ_HANDLED;
    }

    /* No FE, but fatal errors latched: recover and re-arm */
    if (fatal_err)
        csi_channel_recover(cd, CSI_DMA_CHANNEL);

    return IRQ_HANDLED;
}

static irqreturn_t fpga_csi_irq_top(int irq, void *data)
{
    return IRQ_WAKE_THREAD;
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
            if (wait_event_interruptible(cd->ring.wq_read, r_used(&cd->ring) > 0))
            { ret = -ERESTARTSYS; break; }
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
            wake_up_interruptible(&cd->ring.wq_space);
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
    if (r_used(&cd->ring) > 0)
        mask |= POLLIN | POLLRDNORM;

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
        cd->dbuf[i].size = alloc_sz;
        cd->dbuf[i].cpu = dma_alloc_coherent(cd->dev, alloc_sz,
                                             &cd->dbuf[i].dma, GFP_KERNEL);
        if (!cd->dbuf[i].cpu) {
            dev_err(cd->dev, "DMA alloc failed at %d", i);
            goto err;
        }
        memset(cd->dbuf[i].cpu, 0, alloc_sz);
    }

    /* Unmask all CSI2 IRQs for this block (0 = unmask on this HW) */
    wr(cd, CSI2_IRQ_MASK, 0x00000000);

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
    void __iomem *ch;

    if (!cd->regs.csi2)
        return;

    ch = ch_base(cd->regs.csi2, CSI_DMA_CHANNEL);

    /* Mask CSI2 IRQs and disable MIPIC CSI-DMA summary interrupt. */
    wr(cd, CSI2_IRQ_MASK, 0xffffffff);
    wr_mipic(cd, MIPIC_INTE, rd_mipic(cd, MIPIC_INTE) & ~MIPIC_INT_CSI_DMA);

    /* Best-effort DMA channel shutdown. */
    if (ch) {
        u32 ctrl = readl(ch + CH_CTRL);
        if (ctrl & CH_CTRL_DMA_EN)
            writel(ctrl & ~CH_CTRL_DMA_EN, ch + CH_CTRL);
    }
    wmb();
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

static inline void jtag_shift_dr25_write(struct fpga_csi_dev *cd, u32 out)
{
    /* Branch once per complete DR write, never once per GPIO edge. */
    if (likely(!cd->jtag_write_gpio_cansleep))
        jtag_shift_dr25_write_fast(cd, out);
    else
        jtag_shift_dr25_write_sleep(cd, out);
}

static void jtag_release_locked(struct fpga_csi_dev *cd)
{
    if (!cd->jtag_pins_acquired)
        return;

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
 *  - Routes MIPIC to CSI and enables CSI-DMA summary IRQ.
 *  - Initializes the D-PHY (if present).
 *  - Allocates DMA buffers (first time) and starts the channel.
 *
 * Today this is called once at probe using DT-derived defaults; the
 * CSI_IOC_SET_GEOMETRY ioctl is intentionally a no-op for ABI stability.
 */
static int csi_apply_geometry(struct fpga_csi_dev *cd,
                              const struct csi_geometry *g)
{
    struct csi_geometry geom = *g;
    size_t span;

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

    /* Route MIPI-CFG to CSI and enable CSI-DMA summary interrupt */
    wr_mipic(cd, MIPIC_CFG,  MIPIC_CFG_SEL_CSI);
    wr_mipic(cd, MIPIC_INTE, MIPIC_INT_CSI_DMA);

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

        if (alloc_dma_buffers(cd, cd->dma_span))
            return -ENOMEM;

        cd->cur_idx = 0;
        csi_start_channel(cd, CSI_DMA_CHANNEL);
        
        /* Arm Active and Shadow buffers */
        csi_dma_arm(cd, CSI_DMA_CHANNEL, 0); 
        csi_dma_arm(cd, CSI_DMA_CHANNEL, 1);
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
        if (copy_to_user((void __user *)arg, &st, sizeof(st))) return -EFAULT;
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
        }
        spin_unlock_irq(&cd->ring.lock);

        /* Unblock producer if it was space-limited */
        wake_up_interruptible(&cd->ring.wq_space);
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
        if (cd->jtag_owner && cd->jtag_owner != csi_ctx(f)) {
            ret = -EBUSY;
        } else {
            ret = csi_jtag_attach_locked(csi_ctx(f));
            if (!ret)
                ret = jtag_reg_write(cd, r.addr, r.value);
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
        if (cd->jtag_owner && cd->jtag_owner != csi_ctx(f)) {
            ret = -EBUSY;
        } else {
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
        if (cd->jtag_owner && cd->jtag_owner != ctx) {
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
        int i, ret = 0;

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
        if (cd->jtag_owner && cd->jtag_owner != ctx) {
            ret = -EBUSY;
            goto batch_out;
        }

        ret = csi_jtag_attach_locked(ctx);
        if (!ret) {
            for (i = 0; i < batch.count; i++) {
                ret = jtag_reg_write(cd, regs[i].addr, regs[i].value);
                if (ret)
                    break;
                if (batch.delay_us)
                    udelay(batch.delay_us);
            }
        }

    batch_out:
        mutex_unlock(&cd->jtag_lock);
        if (regs_heap)
            kfree(regs);
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
    init_waitqueue_head(&cd->ring.wq_space);
    mutex_init(&cd->read_lock);
    mutex_init(&cd->jtag_lock);
    atomic_set(&cd->jtag_users, 0);

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
    pm_runtime_get_sync(&pdev->dev);

    cd->miscdev.minor = MISC_DYNAMIC_MINOR;
    cd->miscdev.name  = DEV_NAME;
    cd->miscdev.fops  = &csi_fops;
    cd->miscdev.mode  = 0666;

    ret = misc_register(&cd->miscdev);
    if (ret) {
        dev_err(&pdev->dev, "misc register failed: %d", ret);
        goto err_misc;
    }

    ret = devm_request_threaded_irq(&pdev->dev, cd->irq,
                                    fpga_csi_irq_top,
                                    fpga_csi_irq_thread,
                                    IRQF_ONESHOT, DRV_NAME, cd);
    if (ret) {
        dev_err(&pdev->dev, "irq request failed: %d", ret);
        goto err_irq;
    }

    /*
     * Auto-apply the overlay/driver default geometry at probe time.
     * This allocates DMA buffers, initializes the D-PHY, and starts
     * the CSI2 DMA channel using cd->geom as set above.
     */
    ret = csi_apply_geometry(cd, &cd->geom);
    if (ret && ret != -EBUSY) {
        dev_err(&pdev->dev,
                "failed to apply default geometry (bytes_per_line=%u, lines=%u): %d\n",
                cd->geom.bytes_per_line, cd->geom.lines, ret);
        goto err_irq;
    }

    platform_set_drvdata(pdev, cd);
    dev_info(&pdev->dev,
             "mapped reg[0..3], irq=%d; /dev/%s ready with geometry %u x %u (span=%zu)",
             cd->irq, DEV_NAME,
             cd->geom.bytes_per_line, cd->geom.lines, cd->dma_span);
    return 0;

err_irq:
    misc_deregister(&cd->miscdev);
err_misc:
    vfree(cd->ring.data);
    pm_runtime_put_sync(&pdev->dev);
    pm_runtime_disable(&pdev->dev);
    return ret;
}

static void fpga_csi_remove(struct platform_device *pdev)
{
    struct fpga_csi_dev *cd = platform_get_drvdata(pdev);

    /* Quiesce the hardware before freeing DMA resources. */
    csi_stop(cd);

    misc_deregister(&cd->miscdev);
    vfree(cd->ring.data);
    free_dma_buffers(cd);
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
                   "with error recovery and single-geometry setup");
MODULE_LICENSE("GPL");
