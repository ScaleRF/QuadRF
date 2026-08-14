#pragma once
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <complex>
#include <cstring>
#include <utility>
#include <mutex>
#include <stdexcept>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <arm_neon.h> 

#include "Farrow.hpp" 

static constexpr const double RX_LINE_RATE = ((131072.0/142638.0)*87.5e6);

// DSI TX default timing.  Userspace writes only active RGB payload bytes, but
// the physical DSI frame also contains per-line and per-frame overhead.  When
// deriving the native TX sample rate from the DSI byte clock, use payload/total.
static constexpr const double TX_DSI_BYTE_RATE_DEFAULT = 175.0e6;
static constexpr const double TX_DEFAULT_PAYLOAD_BYTES_PER_FRAME = 3072.0 * 1080.0;
static constexpr const double TX_DEFAULT_TOTAL_BYTES_PER_FRAME =
    TX_DEFAULT_PAYLOAD_BYTES_PER_FRAME + (4.0 + 2.0 + 36.0) * 1080.0 + 9342.0;
static constexpr const double TX_DEFAULT_PAYLOAD_FRACTION =
    TX_DEFAULT_PAYLOAD_BYTES_PER_FRAME / TX_DEFAULT_TOTAL_BYTES_PER_FRAME;

// Prefer the exact kernel UAPI header when it is available.  The fallback
// below intentionally matches fpga_csi.h from the kernel driver: keep the
// struct layouts and ioctl numbers in lockstep with the driver.
#if defined(__has_include)
#  if __has_include("fpga_csi.h")
#    include "fpga_csi.h"
#    define MIPI_DEVICE_HAS_EXTERNAL_CSI_UAPI 1
#  endif
#endif

#ifndef MIPI_DEVICE_HAS_EXTERNAL_CSI_UAPI
#ifndef _UAPI_MIPI_CSI_RING_H_
#define _UAPI_MIPI_CSI_RING_H_

#ifndef CSI_IOC_MAGIC
#define CSI_IOC_MAGIC 'C'
#endif

#define CSI_VC_MAX 4

struct csi_ring_info {
    __u32 ring_size;     // bytes, page-aligned, mmap() length
    __u32 span_bytes;    // producer writes in multiples of this
    __u32 head;          // producer position, driver writes
    __u32 tail;          // consumer position, userspace advances
};

#ifndef CSI_IOC_CONSUME_BYTES
#define CSI_IOC_CONSUME_BYTES   _IOW(CSI_IOC_MAGIC, 0x41, __u32)
#endif
#ifndef CSI_IOC_GET_RING_INFO
#define CSI_IOC_GET_RING_INFO   _IOR(CSI_IOC_MAGIC, 0x40, struct csi_ring_info)
#endif

struct csi_filter_cfg {
    __u8  enable_vc_filter;
    __u8  vc;
    __u8  enable_dt_filter;
    __u8  dt;
};
#ifndef CSI_IOC_SET_FILTER
#define CSI_IOC_SET_FILTER      _IOW(CSI_IOC_MAGIC, 0x01, struct csi_filter_cfg)
#endif

struct csi_geometry {
    __u32 bytes_per_line;
    __u32 lines;
};
#ifndef CSI_IOC_SET_GEOMETRY
#define CSI_IOC_SET_GEOMETRY    _IOW(CSI_IOC_MAGIC, 0x05, struct csi_geometry)
#endif

#ifndef CSI_IOC_RESET
#define CSI_IOC_RESET           _IO(CSI_IOC_MAGIC,  0x06)
#endif

#endif // _UAPI_MIPI_CSI_RING_H_
#endif // !MIPI_DEVICE_HAS_EXTERNAL_CSI_UAPI

#ifndef _UAPI_MIPI_DSI_MAP_H_
#define _UAPI_MIPI_DSI_MAP_H_
#ifndef DSI_IOC_MAGIC
#define DSI_IOC_MAGIC 'D'
#endif
struct dsi_fb_info {
    __u64 fb_bytes;     /* frame_bytes */
    __u32 fb_count;     /* number of staging buffers */
    __u32 head;         /* enqueue index */
    __u32 tail;         /* dequeue index */
    __s32 queued;       /* number of queued frames (>= 0) */
    __u32 _pad;
};
#ifndef DSI_IOC_GET_FB_INFO
#define DSI_IOC_GET_FB_INFO _IOR(DSI_IOC_MAGIC, 0x10, struct dsi_fb_info)
#endif
#ifndef DSI_IOC_QUEUE_NEXT
#define DSI_IOC_QUEUE_NEXT _IO(DSI_IOC_MAGIC, 0x11)
#endif
#ifndef DSI_IOC_DEQUEUE
#define DSI_IOC_DEQUEUE    _IO(DSI_IOC_MAGIC, 0x12)
#endif
#endif // _UAPI_MIPI_DSI_MAP_H_

class MipiDevice : public SoapySDR::Device
{
public:
    // Antenna API
    std::vector<std::string> listAntennas(const int dir, const size_t chan) const override;
    void setAntenna(const int dir, const size_t chan, const std::string &name) override;
    std::string getAntenna(const int dir, const size_t chan) const override;

    // Gain API (named)
    std::vector<std::string> listGains(const int dir, const size_t chan) const override;
    SoapySDR::Range getGainRange(const int dir, const size_t chan, const std::string &name) const override;
    void setGain(const int dir, const size_t chan, const std::string &name, const double value) override;
    double getGain(const int dir, const size_t chan, const std::string &name) const override;

    // Frequency API (named)
    std::vector<std::string> listFrequencies(const int dir, const size_t chan) const override;
    void setFrequency(const int dir, const size_t chan, const std::string &name, const double freq, const SoapySDR::Kwargs &args) override;
    double getFrequency(const int dir, const size_t chan, const std::string &name) const override;
    SoapySDR::RangeList getFrequencyRange(const int dir, const size_t chan, const std::string &name) const override;

    // Sample-rate API
    std::vector<double> listSampleRates(const int dir, const size_t chan) const override;
    void setSampleRate(const int dir, const size_t chan, const double rate) override;
    
    // Capabilities exposure
    SoapySDR::RangeList getSampleRateRange(const int dir, const size_t chan) const override;
    SoapySDR::RangeList getFrequencyRange(const int dir, const size_t chan) const override;
    SoapySDR::Range getGainRange(const int dir, const size_t chan) const override;
    SoapySDR::Kwargs getHardwareInfo() const override;
    
    explicit MipiDevice(const SoapySDR::Kwargs &args);
    ~MipiDevice() override;

    // Identification
    std::string getDriverKey(void) const override { return "mipi"; }
    std::string getHardwareKey(void) const override { return "rpi5-mipi"; }

    // Channels
    size_t getNumChannels(const int dir) const override;
    std::vector<std::string> getStreamFormats(const int dir, const size_t) const override;
    std::string getNativeStreamFormat(const int dir, const size_t, double &fullScale) const override;
    double getSampleRate(const int dir, const size_t) const override;

    // Settings (portable subset)
    void writeSetting(const std::string &key, const std::string &value) override;
    std::string readSetting(const std::string &key) const override;
    
    // Streams
    SoapySDR::Stream *setupStream(const int dir,
                                  const std::string &format,
                                  const std::vector<size_t> &channels,
                                  const SoapySDR::Kwargs &args) override;
    void closeStream(SoapySDR::Stream *stream) override;
    int activateStream(SoapySDR::Stream *stream, const int flags,
                       const long long timeNs, const size_t numElems) override;
    int deactivateStream(SoapySDR::Stream *stream, const int flags,
                         const long long timeNs) override;
    int readStream(SoapySDR::Stream *stream, void * const *buffs,
                   const size_t numElems, int &flags,
                   long long &timeNs, const long timeoutUs) override;
    int writeStream(SoapySDR::Stream *stream, const void * const *buffs,
                    const size_t numElems, int &flags,
                    const long long timeNs, const long timeoutUs) override;
    size_t getStreamMTU(SoapySDR::Stream *stream) const override;

private:

    struct MipiStream
    {
        int dir = 0;
        std::string format;
        std::vector<size_t> channels;
        bool active = false;
        uint64_t id = 0;
    };

    void initRx();
    void initTx();
    // utility
    static int xopen(const char *path, int flags);
    static int xpoll(int fd, short events, int timeoutMs);

    // rx helpers
    ssize_t rx_read_legacy(void *dst, size_t bytes, long timeoutUs);
    ssize_t rx_read_ring(void *dst, size_t bytes, long timeoutUs);

    // tx helpers
    ssize_t tx_write_legacy(const void *src, size_t bytes, long timeoutUs);
    ssize_t tx_write_staging(const void *src, size_t bytes, long timeoutUs);
    
    // NEON conversion helper (TX)
    static void convert_CF32_to_CS8_NEON(const float *input, int8_t *output, size_t count);
    // NEON conversion helper (RX)
    static void convert_CS8_to_CF32_NEON(const int8_t *input, float *output, size_t count);

    // NEON deinterleave helpers (RX 4-channel)
    static void deinterleave_CS8_NEON(const int8_t *input, void * const *buffs, size_t numElems);
    static void deinterleave_CS8_to_CF32_NEON(const int8_t *input, void * const *buffs, size_t numElems);

    // common state
    mutable std::recursive_mutex deviceMutex_;
    mutable std::recursive_mutex rxMutex_;
    mutable std::recursive_mutex txMutex_;

    uint64_t nextStreamId_{1};
    bool rxStreamOpen_{false};
    bool txStreamOpen_{false};

    std::string rxPath_, txPath_;
    int fdRx_{-1}, fdTx_{-1};

    // RX geometry
    uint32_t rxBytesPerLine_{0}, rxLines_{0}, rxFps_{0};

    // TX geometry
    uint32_t txBytesPerLine_{0}, txLines_{0}, txFps_{0};
    uint32_t txOverheadBytesPerLine_{42};
    uint32_t txFrameExtraBytes_{9342};
    double   txDsiByteRate_{TX_DSI_BYTE_RATE_DEFAULT};
    double   txPayloadFraction_{TX_DEFAULT_PAYLOAD_FRACTION};

    // Formats
    bool txIsCF32_{false};

    // Zero-copy mappings
    size_t rxMapLen_{0};
    size_t txMapLen_{0};
    void *rxRing_{nullptr};
    size_t rxRingSize_{0};
    size_t rxSpanBytes_{0};

    void *txStaging_{nullptr};
    size_t txFbBytes_{0};
    uint32_t txFbCount_{0};

    // mmap TX accumulation cursors (per-instance) ---
    size_t txHeadIndex_{0}; 
    size_t txHeadOff_{0};   

    // Cached chunk sizes
    size_t rxChunkBytes_{0};
    size_t txFrameBytes_{0};
    
    // Compatibility state
    std::string rxRequestedFormat_ = SOAPY_SDR_CS8;
    std::string txRequestedFormat_ = SOAPY_SDR_CS8; 
    
    std::vector<size_t> rxChannels_; 
    
    mutable std::vector<uint8_t> rxScratch_;
    mutable std::vector<uint8_t> txScratch_;
    mutable std::vector<float> resampScratch_;

    // ---------------- Generic Ring Buffer (For Raw Hardware I/O) ----------------
    template <typename T>
    struct RingBuffer
    {
        std::vector<T> buf;
        size_t mask = 0;   
        size_t head = 0;   
        size_t tail = 0;   

        void reset() { head = tail = 0; }
        size_t capacity() const { return mask ? (mask + 1) : 0; }
        size_t size() const { return head - tail; }
        size_t free() const { return capacity() - size(); }

        static size_t round_pow2(size_t n) {
            size_t p = 1;
            while (p < n) p <<= 1;
            return p;
        }

        void init(size_t minCapacity) {
            size_t cap = round_pow2(std::max<size_t>(minCapacity, 4096));
            buf.resize(cap);
            mask = cap - 1;
            reset();
        }

        size_t write(const T *p, size_t n) {
            size_t w = std::min(n, free());
            size_t cap = capacity();
            size_t h = head & mask;
            size_t n1 = std::min(w, cap - h);
            if (n1) std::memcpy(&buf[h], p, n1 * sizeof(T));
            if (w > n1) std::memcpy(&buf[0], p + n1, (w - n1) * sizeof(T));
            head += w;
            return w;
        }

        std::pair<const T*, size_t> readSpan() const {
            size_t avail = size();
            if (!avail) return {nullptr, 0};
            size_t cap = capacity();
            size_t t = tail & mask;
            size_t n1 = std::min(avail, cap - t);
            return { &buf[t], n1 };
        }

        void consume(size_t n) { tail += std::min(n, size()); }
    };

    // ---------------- Linear DSP Buffer (For Contiguous Block Processing) ----------------
    // Solves the "wrap-around deadlock" for algorithms like Farrow that need minimum lookahead,
    // without the cache-thrashing penalty of constant memmoves. It shifts the tiny residual 
    // leftover samples to the front only when the entire memory allocation is exhausted.
    template <typename T>
    struct LinearDSPBuffer
    {
        std::vector<T> buf;
        size_t head = 0;   
        size_t tail = 0;   

        void init(size_t capacity) {
            buf.resize(std::max<size_t>(capacity, 8192));
            head = tail = 0;
        }

        void reset() { head = tail = 0; }
        size_t capacity() const { return buf.size(); }
        size_t readAvail() const { return head - tail; }
        const T* readPtr() const { return &buf[tail]; }

        T* prepareWrite(size_t n) {
            if (head + n > buf.size()) {
                size_t residual = head - tail;
                if (residual > 0 && tail > 0) {
                    std::memmove(&buf[0], &buf[tail], residual * sizeof(T));
                }
                tail = 0;
                head = residual;
                if (head + n > buf.size()) buf.resize(head + n);
            }
            return &buf[head];
        }

        void commitWrite(size_t n) { head += n; }

        void consume(size_t n) {
            tail += n;
            if (tail == head) { tail = head = 0; }
        }
    };

    RingBuffer<uint8_t> txRing_;
    size_t txRingMaxBytes_ = 0;     // high-water cap (bytes)
    long   txDrainTimeoutUs_ = 0;   // optional flush timeout (us)

    void txRingInit_();
    void txRingFlush_(long timeoutUs);

    double gain_ = 0.0;
    std::string rxAntennaSel_ = "RX";
    std::string txAntennaSel_ = "TX";
    double lastRxRate_ = 0.0;
    double sampleRateRatio_ = 1.0; // F_in / F_out

    // ---------- Stereo Elliptic Filter & Resampler (RX) ----------
    static constexpr double kFsIn = RX_LINE_RATE;
    DSP::FarrowResampler resampler_;

    // DSP RX State
    mutable LinearDSPBuffer<float> rxFloatBuf_; 

    // ---------- TX Resampler (Host -> DSI line rate) ----------
    double lastTxRate_ = 0.0;               // Host-selected TX sample rate (Soapy visible)
    double txLineRate_ = 0.0;               // Fixed DSI line rate (samples/sec, complex)
    double txSampleRateRatio_ = 1.0;        // F_in(host) / F_out(line)

    DSP::FarrowResampler txResampler_;

    // DSP TX State
    LinearDSPBuffer<float> txFloatBuf_;
    std::vector<uint8_t> txOutBytes_;
    std::vector<float> txOutFloatScratch_;
    size_t txOutOff_ = 0;

    void txResampler_config_(double hostRate);
    // Produce resampled CS8 bytes into txRing_ from txFloatBuf_ using Farrow resampler.
    void txProduceToRing_(long timeoutUs);
    void txFlush_(long timeoutUs);

    void rxFilter_config_(double fsOut);
};