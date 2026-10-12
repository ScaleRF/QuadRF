#pragma once
// Thin C++ wrapper over the fpga-csi JTAG ioctls: immediate register access
// over RIO, and the PIO write stream (fixed-period write slots on
// CLOCK_MONOTONIC). Errors throw std::system_error with the driver's errno.
//
// Owns one fd on /dev/csi_stream0. The driver ties a stream to the fd that
// started it, so destroying this object stops a running stream without
// draining it.
#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <ctime>
#include <string>
#include <system_error>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#if __has_include("fpga_csi.h")
#  include "fpga_csi.h"
#elif __has_include("../fpga/drivers/csi/fpga_csi.h")
#  include "../fpga/drivers/csi/fpga_csi.h"
#elif __has_include("../drivers/csi/fpga_csi.h")
#  include "../drivers/csi/fpga_csi.h"
#else
#  include <quadrf/fpga_csi.h>
#endif

#ifndef CSI_IOC_JTAG_STREAM_START
#error "fpga_csi.h predates JTAG streams; build against the matching fpga-csi driver"
#endif

namespace quadrf {

class FpgaJtag
{
public:
    static constexpr uint32_t kMinPeriodNs = 2005;     // one 33-clock DR scan at 16.7 MHz TCK
    static constexpr uint32_t kMaxQueue = 16384;       // writes per queue ioctl

    explicit FpgaJtag(std::string path = "/dev/csi_stream0") : path_(std::move(path)) {}
    ~FpgaJtag() { if (fd_ >= 0) ::close(fd_); }
    FpgaJtag(const FpgaJtag &) = delete;
    FpgaJtag &operator=(const FpgaJtag &) = delete;

    static uint64_t nowNs()
    {
        timespec t{};
        ::clock_gettime(CLOCK_MONOTONIC, &t);
        return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
    }

    static uint32_t word(uint8_t addr, uint16_t value) { return (uint32_t(addr) << 16) | value; }

    // Immediate access. Another process's lease (quadrf-jtag, the ScaleRF
    // service) shows up as EBUSY for a few ms, so retry for up to ~0.5 s.
    void write(uint8_t addr, uint16_t value)
    {
        csi_jtag_reg r{};
        r.addr = addr;
        r.value = value;
        retryBusy(CSI_IOC_JTAG_REG_WRITE, &r, "JTAG write");
    }

    uint16_t read(uint8_t addr)
    {
        csi_jtag_reg r{};
        r.addr = addr;
        retryBusy(CSI_IOC_JTAG_REG_READ, &r, "JTAG read");
        return r.value;
    }

    // Stream. start_ns = 0 starts as soon as possible.
    csi_jtag_stream_start start(uint32_t periodNs, uint64_t startNs = 0, uint32_t leadNs = 0,
                                const std::vector<csi_jtag_reg> &pre = {})
    {
        csi_jtag_stream_start s{};
        s.period_ns = periodNs;
        s.lead_ns = leadNs;
        s.start_ns = startNs;
        s.regs_ptr = uintptr_t(pre.data());
        s.count = uint32_t(pre.size());
        if (::ioctl(fd(), CSI_IOC_JTAG_STREAM_START, &s))
            fail("JTAG stream start");
        period_ = s.period_ns;
        slot0_ = s.t_slot0_ns;
        return s;
    }

    // Writes go to consecutive slots, after everything already queued, or
    // from `slot` on. Returns the first slot. Blocks while the ring is full.
    uint64_t queue(const csi_jtag_reg *r, size_t n, int64_t slot = -1)
    {
        uint64_t first = 0;
        for (size_t done = 0; done < n;) {
            const uint32_t k = uint32_t(std::min<size_t>(n - done, kMaxQueue));
            csi_jtag_stream_queue q{};
            q.regs_ptr = uintptr_t(r + done);
            q.count = k;
            if (slot >= 0) {
                q.flags = CSI_JTAG_STREAM_AT;
                q.slot = uint64_t(slot) + done;
            }
            if (::ioctl(fd(), CSI_IOC_JTAG_STREAM_QUEUE, &q))
                fail(errno == ETIME ? "JTAG stream slot already sent" : "JTAG stream queue");
            if (!done)
                first = q.slot;
            done += k;
        }
        return first;
    }

    uint64_t queueWords(const uint32_t *w, size_t n, int64_t slot = -1)
    {
        std::vector<csi_jtag_reg> r(n);
        for (size_t i = 0; i < n; i++) {
            r[i].addr = uint8_t(w[i] >> 16);
            r[i].value = uint16_t(w[i]);
        }
        return queue(r.data(), n, slot);
    }

    // First slot starting at or after t (CLOCK_MONOTONIC ns). Exact only
    // while the stream has had no underruns.
    uint64_t slotAt(uint64_t tNs) const
    {
        if (!period_ || tNs <= slot0_) return 0;
        return (tNs - slot0_ + period_ - 1) / period_;
    }

    uint64_t slotTime(uint64_t slot) const { return slot0_ + slot * period_; }

    void stop(bool drain)
    {
        __u32 flags = drain ? CSI_JTAG_STREAM_DRAIN : 0;
        if (fd_ >= 0 && ::ioctl(fd_, CSI_IOC_JTAG_STREAM_STOP, &flags))
            fail("JTAG stream stop");
        period_ = 0;
    }

    csi_jtag_stream_status status()
    {
        csi_jtag_stream_status st{};
        if (::ioctl(fd(), CSI_IOC_JTAG_STREAM_STATUS, &st))
            fail("JTAG stream status");
        if (st.active) {
            period_ = st.period_ns;
            slot0_ = st.t_slot0_ns;     // moves on underrun
        }
        return st;
    }

    // Started by this object and not stopped since; the driver may still
    // have stopped it (failed DMA), status() tells.
    bool streaming() const { return period_ != 0; }

private:
    int fd()
    {
        if (fd_ < 0) {
            fd_ = ::open(path_.c_str(), O_RDWR | O_CLOEXEC);
            if (fd_ < 0)
                fail("open " + path_);
            if (::ioctl(fd_, CSI_IOC_JTAG_SETUP))
                fail("JTAG setup");
        }
        return fd_;
    }

    void retryBusy(unsigned long req, csi_jtag_reg *r, const char *what)
    {
        for (int i = 0;; i++) {
            if (::ioctl(fd(), req, r) == 0) return;
            if ((errno != EBUSY && errno != EINTR) || i >= 2500) fail(what);
            ::usleep(200);
        }
    }

    [[noreturn]] static void fail(const std::string &what)
    {
        throw std::system_error(errno, std::generic_category(), what);
    }

    std::string path_;
    int fd_ = -1;
    uint32_t period_ = 0;
    uint64_t slot0_ = 0;
};

} // namespace quadrf
