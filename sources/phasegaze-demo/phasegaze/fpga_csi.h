#pragma once
#include <linux/ioctl.h>

#ifdef __KERNEL__
    #include <linux/types.h>
#endif

#ifndef __FPGA_CSI_RING_UAPI__
#define __FPGA_CSI_RING_UAPI__

#define CSI_VC_MAX 4

/* ---- Ring info for mmap + user-space management ---- */
struct csi_ring_info {
    __u32 ring_size;     // bytes (page-aligned) you can mmap()
    __u32 span_bytes;    // producer writes in multiples of this
    __u32 head;          // producer position (driver writes)
    __u32 tail;          // consumer position (userspace writes)
};

/* Userspace can advance tail via CSI_IOC_CONSUME_BYTES. */
#define CSI_IOC_CONSUME_BYTES   _IOW(CSI_IOC_MAGIC, 0x41, __u32)

#endif /* __FPGA_CSI_RING_UAPI__ */


/* ---- IOCTLs ---- */
#define CSI_IOC_MAGIC   'C'

#define CSI_IOC_SET_FILTER   _IOW(CSI_IOC_MAGIC, 0x01, struct csi_filter_cfg)
#define CSI_IOC_GET_STATS    _IOR(CSI_IOC_MAGIC, 0x02, struct csi_stats)
#define CSI_IOC_GET_LINK     _IOR(CSI_IOC_MAGIC, 0x03, struct csi_link_info)
#define CSI_IOC_DBG_PHY      _IOR(CSI_IOC_MAGIC, 0x04, struct csi_debug)
#define CSI_IOC_SET_GEOMETRY _IOW(CSI_IOC_MAGIC, 0x05, struct csi_geometry)
/* Note: accepted but ignored; geometry is fixed at probe from DT. */
#define CSI_IOC_RESET        _IO(CSI_IOC_MAGIC,  0x06)
/* Reserved for future soft-reset semantics; currently a no-op. */

#define CSI_IOC_GET_RING_INFO   _IOR(CSI_IOC_MAGIC, 0x40, struct csi_ring_info)


/* JTAG control + register access */
struct csi_jtag_reg {
    __u8  addr;   /* 0..255 register address on FPGA side */
    __u16 value;  /* 16-bit register payload */
    __u16 _pad;   /* reserved / alignment */
};

#define CSI_IOC_JTAG_SETUP      _IO(CSI_IOC_MAGIC,  0x10)
#define CSI_IOC_JTAG_RELEASE    _IO(CSI_IOC_MAGIC,  0x11)
#define CSI_IOC_JTAG_REG_WRITE  _IOW(CSI_IOC_MAGIC, 0x12, struct csi_jtag_reg)
#define CSI_IOC_JTAG_REG_READ   _IOWR(CSI_IOC_MAGIC,0x13, struct csi_jtag_reg)

/* ---- Config / filter knobs ---- */
struct csi_filter_cfg {
    __u8  enable_vc_filter;  // 0/1
    __u8  vc;                // 0..3 (only used if enable_vc_filter=1)
    __u8  enable_dt_filter;  // 0/1
    __u8  dt;                // 6-bit DT (only used if enable_dt_filter=1)
};

struct csi_geometry {
    __u32 bytes_per_line;    // RAW8: width; RAW10: ceil(10*width/8)
    __u32 lines;             // 0 allowed (driver uses ~1/2 frame span); else full frame
};

/* ---- Runtime stats (driver -> userspace) ----
 * Counters are cumulative since driver load.
 */
struct csi_stats {
    __u64 dma_bytes;         // bytes copied from DMA buffers to user ring
    __u64 bytes_out;         // bytes read() by userspace
    __u64 overflows;         // CSI overflow IRQs (not ring overflows)
    __u32 frame_count;       // frames completed (FE-ACK observed)
    __u64 ch_irq_total;      // total channel-related IRQs
    __u64 ch_irq_fe;         // FE-ACK IRQs observed

    // Per-VC discard counts (unpacked from the HW registers in the ISR)
    __u32 discards_overflow[CSI_VC_MAX];
    __u32 discards_len_limit[CSI_VC_MAX];
    __u32 discards_unmatched[CSI_VC_MAX];
    __u32 discards_inactive[CSI_VC_MAX];

    // Latest DT observed for each discard reason (debug convenience)
    __u8  discards_overflow_dt;
    __u8  discards_len_limit_dt;
    __u8  discards_unmatched_dt;
    __u8  discards_inactive_dt;

    // Optional bookkeeping for ring buffer drops when drop_oldest=1
    __u64 overflows_ring;
};

/* ---- Link snapshot for quick diagnostics ---- */
struct csi_link_info {
    // D-PHY
    __u32 dphy_version;
    __u32 dphy_n_lanes;      // N_LANES register value (lanes = (value & 3) + 1)
    __u32 dphy_resetn;
    __u32 dphy_shutdownz;
    __u32 dphy_rstz;
    __u32 dphy_phy_rx;
    __u32 dphy_stopstate;

    // CSI2 top
    __u32 csi2_status;       // raw CSI2_STATUS

    // Raw discard registers, as read at snapshot time
    __u32 csi2_discards_overflow;   // 0x008
    __u32 csi2_discards_inactive;   // 0x00c
    __u32 csi2_discards_unmatched;  // 0x010
    __u32 csi2_discards_len_limit;  // 0x014

    // MIPIC summary + IRQ accounting
    __u32 mipic_cfg;
    __u32 mipic_intr;
    __u32 mipic_inte;
    __u32 mipic_ints;
    __u64 mipic_irq_total;
    __u64 mipic_irq_dma;
    __u64 mipic_irq_host;
    __u64 mipic_irq_other;
    __u64 ch_irq_total;
    __u64 ch_irq_fe;
};

/* ---- Simple PHY debug ---- */
struct csi_debug {
    __u32 phy_rx;
    __u32 stopstate;
};
