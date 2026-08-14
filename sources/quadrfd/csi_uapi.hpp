#pragma once
#include <sys/ioctl.h>
#include <linux/types.h>

// --- RX (CSI-2) UAPI ---
#ifndef CSI_IOC_MAGIC
#define CSI_IOC_MAGIC 'C'
#endif

/* Matches fpga_csi.h */
struct csi_ring_info {
    __u32 ring_size;     
    __u32 span_bytes;    
    __u32 head;          
    __u32 tail;          
};

#ifndef CSI_IOC_GET_RING_INFO
#define CSI_IOC_GET_RING_INFO   _IOR(CSI_IOC_MAGIC, 0x40, struct csi_ring_info)
#define CSI_IOC_CONSUME_BYTES   _IOW(CSI_IOC_MAGIC, 0x41, __u32)
#endif

struct csi_geometry {
    __u32 bytes_per_line;
    __u32 lines;
    __u32 _pad0;
    __u32 _pad1;
};

#ifndef CSI_IOC_SET_GEOMETRY
#define CSI_IOC_SET_GEOMETRY _IOW(CSI_IOC_MAGIC, 0x05, struct csi_geometry)
#endif

struct csi_filter_cfg {
    __u8  enable_vc_filter;
    __u8  vc;
    __u8  enable_dt_filter;
    __u8  dt;
};

#ifndef CSI_IOC_SET_FILTER
#define CSI_IOC_SET_FILTER _IOW(CSI_IOC_MAGIC, 0x01, struct csi_filter_cfg)
#define CSI_IOC_RESET      _IO(CSI_IOC_MAGIC, 0x06)
#endif

// --- TX (DSI) UAPI ---
#ifndef DSI_IOC_MAGIC
#define DSI_IOC_MAGIC 'D'
#endif

struct dsi_fb_info {
    __u64 fb_bytes;
    __u32 fb_count;
    __u32 head;
    __u32 tail;
    __s32 queued;
    __u32 _pad;
};

#ifndef DSI_IOC_GET_FB_INFO
#define DSI_IOC_GET_FB_INFO _IOR(DSI_IOC_MAGIC, 0x10, struct dsi_fb_info)
#define DSI_IOC_QUEUE_NEXT  _IO(DSI_IOC_MAGIC, 0x11)
#define DSI_IOC_DEQUEUE     _IO(DSI_IOC_MAGIC, 0x12)
#endif