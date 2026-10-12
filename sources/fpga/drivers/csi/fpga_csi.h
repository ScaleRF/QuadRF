#pragma once
#include <linux/ioctl.h>
#include <linux/types.h>

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
#define CSI_IOC_GET_EVENTS    _IOR(CSI_IOC_MAGIC, 0x07, struct csi_event_stats)

#define CSI_IOC_GET_RING_INFO   _IOR(CSI_IOC_MAGIC, 0x40, struct csi_ring_info)


/* JTAG control + register access */
struct csi_jtag_reg {
    __u8  addr;   /* 0..255 register address on FPGA side */
    __u16 value;  /* 16-bit register payload */
    __u16 _pad;   /* reserved / alignment */
};

/* batch JTAG transactions */
struct csi_jtag_batch {
    __u64 regs_ptr;  /* User pointer to array of struct csi_jtag_reg */
    __u32 count;     /* Number of registers to process */
    __u32 delay_us;  /* Optional microsecond delay between writes */
};

#define CSI_IOC_JTAG_SETUP      _IO(CSI_IOC_MAGIC,  0x10)
#define CSI_IOC_JTAG_RELEASE    _IO(CSI_IOC_MAGIC,  0x11)
#define CSI_IOC_JTAG_REG_WRITE  _IOW(CSI_IOC_MAGIC, 0x12, struct csi_jtag_reg)
#define CSI_IOC_JTAG_REG_READ   _IOWR(CSI_IOC_MAGIC,0x13, struct csi_jtag_reg)

#define CSI_IOC_JTAG_BATCH_WRITE   _IOW(CSI_IOC_MAGIC, 0x14, struct csi_jtag_batch)
#define CSI_IOC_JTAG_ACQUIRE_LEASE _IO(CSI_IOC_MAGIC,  0x15)
#define CSI_IOC_JTAG_RELEASE_LEASE _IO(CSI_IOC_MAGIC,  0x16)

/*
 * Paced batch: write i starts at t_first + i * period_ns (CLOCK_MONOTONIC).
 * A write that cannot start within period_ns / 4 of its slot starts as soon
 * as possible and the rest of the schedule moves with it, so the spacing
 * never drops below period_ns. period_ns = 0 runs back to back.
 *
 * With period_ns >= 2005 ns the driver runs the batch as a JTAG stream (see
 * below) and returns once it has drained. The spacing is then exact in PIO
 * clocks (period rounded down to 5 ns), t_first_ns is slot 0 of the stream,
 * max_late_ns is the uncertainty of t_first_ns (a few us), and slips counts
 * stream underruns (0 unless the feeder thread was starved).
 */
struct csi_jtag_batch_paced {
    __u64 regs_ptr;     /* in: struct csi_jtag_reg array */
    __u32 count;        /* in: 1..1024 */
    __u32 period_ns;    /* in */
    __u64 t_first_ns;   /* out: CLOCK_MONOTONIC at the start of write 0 */
    __u32 slips;        /* out: writes that started late and moved the schedule */
    __u32 max_late_ns;  /* out: worst start lateness, slips included */
};

#define CSI_IOC_JTAG_BATCH_PACED _IOWR(CSI_IOC_MAGIC, 0x17, struct csi_jtag_batch_paced)

/*
 * JTAG write stream: an RP1 PIO state machine opens one write slot every
 * period_ns, slot k at t_slot0 + k * period_ns (CLOCK_MONOTONIC, PIO crystal
 * time, no drift relative to the arch counter beyond ppm). Each slot carries
 * one queued register write or nothing. Writes keep the same 25-bit USER1 DR
 * format as CSI_IOC_JTAG_REG_WRITE; TCK runs at 16.7 MHz, one write takes
 * 1.98 us, so period_ns >= 2005.
 *
 * Writes reach DMA lead_slots ahead of their slot; until then they can be
 * queued. A queue call whose slot is already committed gets -ETIME. The
 * driver holds 262144 slots ahead of the DMA (1.3 s at 5 us); a write further
 * out blocks the caller until it fits, or gets -EAGAIN with NONBLOCK.
 *
 * While a stream runs it owns the pins: REG_WRITE, REG_READ, BATCH_WRITE and
 * BATCH_PACED return -EBUSY for everyone, the owner included. Only the file
 * that started the stream can queue or stop it; closing that file or
 * JTAG_RELEASE stops it without draining. Needs jtag_rio=1 and jtag_pio=1;
 * START returns -EOPNOTSUPP otherwise.
 *
 * Underruns: if the feeder thread falls behind by more than the 16-word PIO
 * FIFO, the SM inserts idle slots on its own, every later slot moves, and
 * t_slot0_ns is re-estimated with a larger t_slot0_err_ns.
 */
struct csi_jtag_stream_start {
    __u32 period_ns;      /* in; out: rounded down to 5 ns */
    __u32 lead_ns;        /* in: DMA horizon, 0 = 1.5 ms */
    __u64 start_ns;       /* in: CLOCK_MONOTONIC for slot 0, 0 = asap (~0.2 ms) */
    __u64 regs_ptr;       /* in: optional struct csi_jtag_reg array for slots 0.. */
    __u32 count;          /* in: 0..16384 */
    __u32 lead_slots;     /* out: slots handed to DMA ahead of time */
    __u64 t_slot0_ns;     /* out */
    __u32 t_slot0_err_ns; /* out: +/- on t_slot0_ns */
    __u32 reserved;
};

#define CSI_JTAG_STREAM_AT       (1u << 0)  /* queue at .slot, else after the last write */
#define CSI_JTAG_STREAM_NONBLOCK (1u << 1)  /* -EAGAIN instead of waiting for ring space */

struct csi_jtag_stream_queue {
    __u64 regs_ptr;       /* in: struct csi_jtag_reg array, consecutive slots */
    __u32 count;          /* in: 1..16384 */
    __u32 flags;          /* in: CSI_JTAG_STREAM_* */
    __u64 slot;           /* in with AT; out: slot of the first write */
    __u64 t_first_ns;     /* out: slot start, CLOCK_MONOTONIC */
};

struct csi_jtag_stream_status {
    __u32 active;
    __u32 period_ns;
    __u64 t_slot0_ns;
    __u32 t_slot0_err_ns;
    __u32 underruns;
    __u64 committed;      /* slots handed to DMA; queue at or after this */
    __u64 queued_end;     /* one past the last queued write */
    __u64 slot_now;       /* slot being clocked out now (estimate) */
    __u32 lead_slots;
    __u32 reserved;
};

#define CSI_JTAG_STREAM_DRAIN    (1u << 0)  /* STOP: send every queued write first */

#define CSI_IOC_JTAG_STREAM_START  _IOWR(CSI_IOC_MAGIC, 0x18, struct csi_jtag_stream_start)
#define CSI_IOC_JTAG_STREAM_QUEUE  _IOWR(CSI_IOC_MAGIC, 0x19, struct csi_jtag_stream_queue)
#define CSI_IOC_JTAG_STREAM_STATUS _IOR(CSI_IOC_MAGIC,  0x1a, struct csi_jtag_stream_status)
#define CSI_IOC_JTAG_STREAM_STOP   _IOW(CSI_IOC_MAGIC,  0x1b, __u32)

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
    __u32 frame_count;       // DMA spans completed (FE-ACK or inferred at next FS)
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

    // Complete spans lost in the software DMA/ring pipeline
    __u64 overflows_ring;
};

/* ---- Frame-event / DMA-queue diagnostics (driver -> userspace) ----
 * This is a separate ioctl so the existing csi_stats ABI and ioctl number
 * remain unchanged for deployed SoapySDR binaries.
 */
struct csi_event_stats {
    __u64 irq_fs;              // IRQ samples containing FS
    __u64 irq_fe;              // IRQ samples containing FE_ACK
    __u64 irq_both;            // IRQ samples containing both FS and FE_ACK
    __u64 inferred_fe;         // FE completions inferred from the following FS
    __u64 recoveries;          // channel re-primes after a fatal discard
    __u64 no_buffer;           // failures to keep current + next addresses posted
    __u64 inactive_irqs;       // IRQ samples reporting DISCARD_INACTIVE
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
