#include "MipiDevice.hpp"
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Types.hpp>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <limits>
#include <atomic>
#include <complex>
#include <stdexcept>
#include <type_traits>

#ifndef DSI_IOC_GET_FB_INFO
#define DSI_IOC_GET_FB_INFO _IOR('D', 0x10, struct dsi_fb_info)
#endif

static constexpr const char *kDefaultRxPath = "/dev/csi_stream0";
static constexpr const char *kDefaultTxPath = "/dev/dsi_stream0";

static_assert(sizeof(csi_ring_info) == 16, "csi_ring_info ABI mismatch with fpga-csi driver");
static_assert(sizeof(csi_geometry) == 8, "csi_geometry ABI mismatch with fpga-csi driver");

MipiDevice::MipiDevice(const SoapySDR::Kwargs &args)
{
    rxPath_ = args.count("rx_dev") ? args.at("rx_dev") : kDefaultRxPath;
    txPath_ = args.count("tx_dev") ? args.at("tx_dev") : kDefaultTxPath;

    // only used for buffer chunk sizing, not rates
    rxBytesPerLine_ = args.count("bytes_per_line") ? uint32_t(std::stoul(args.at("bytes_per_line"))) : 1024;
    rxLines_        = args.count("lines") ? uint32_t(std::stoul(args.at("lines"))) : 1024;
    //rxFps_ = round(2.0 * 640.0e6 / (rxBytesPerLine_ * rxLines_ * 8.0));

    // TX timing parameters are still used for *rate math*; framebuffer geometry is owned by driver/DTS.
    txBytesPerLine_ = args.count("tx_bytes_per_line") ? uint32_t(std::stoul(args.at("tx_bytes_per_line"))) : 3072;
    txLines_        = args.count("tx_lines") ? uint32_t(std::stoul(args.at("tx_lines"))) : 1080;
    txFps_          = args.count("tx_fps") ? uint32_t(std::stoul(args.at("tx_fps"))) : 26;

    txOverheadBytesPerLine_ = args.count("tx_overhead_bytes_per_line")
        ? uint32_t(std::stoul(args.at("tx_overhead_bytes_per_line"))) : 42; // 4 header + 2 CRC + 36 porch bytes
    txFrameExtraBytes_ = args.count("tx_frame_extra_bytes")
        ? uint32_t(std::stoul(args.at("tx_frame_extra_bytes"))) : 9342;

    const double txPayloadBytes = double(txBytesPerLine_) * double(txLines_);
    const double txTotalBytes   = txPayloadBytes
                                + double(txOverheadBytesPerLine_) * double(txLines_)
                                + double(txFrameExtraBytes_);
    txPayloadFraction_ = (txTotalBytes > 0.0) ? (txPayloadBytes / txTotalBytes) : 1.0;

    // Preferred interpretation: derive the native payload IQ rate from the DSI
    // byte stream, then apply payload/total.  If the caller explicitly supplies
    // tx_dsi_byte_rate, use it.  Otherwise use the measured/nominal 87.5 MHz
    // byte stream.  If you want legacy behavior, pass tx_dsi_byte_rate equal to
    // tx_fps * txTotalBytes.
    txDsiByteRate_ = args.count("tx_dsi_byte_rate")
        ? std::stod(args.at("tx_dsi_byte_rate")) : TX_DSI_BYTE_RATE_DEFAULT;

    txLineRate_     = 0.5 * txDsiByteRate_ * txPayloadFraction_;
    lastTxRate_     = 0.0;
    txSampleRateRatio_ = 1.0;
    txResampler_.setEnabled(false);

    txHeadIndex_ = 0;
    txHeadOff_   = 0;

    txFrameBytes_ = size_t(txBytesPerLine_) * size_t(txLines_);

    // Pre-allocate buffers to reduce RP5 fragmentation.
    txOutBytes_.reserve(std::max<size_t>(txFrameBytes_ * 2, size_t(1) << 20));
    rxChunkBytes_ = size_t(rxBytesPerLine_) * std::max<size_t>(1, size_t(rxLines_));
}

MipiDevice::~MipiDevice()
{
    std::lock_guard<std::recursive_mutex> dlock(deviceMutex_);
    std::lock_guard<std::recursive_mutex> rlock(rxMutex_);
    std::lock_guard<std::recursive_mutex> tlock(txMutex_);

    if (rxRing_ && rxMapLen_) ::munmap(rxRing_, rxMapLen_);
    rxRing_ = nullptr;
    rxMapLen_ = 0;

    if (txStaging_ && txMapLen_) ::munmap(txStaging_, txMapLen_);
    txStaging_ = nullptr;
    txMapLen_ = 0;

    if (fdRx_ >= 0) ::close(fdRx_);
    if (fdTx_ >= 0) ::close(fdTx_);
    fdRx_ = -1;
    fdTx_ = -1;
}

void MipiDevice::initRx()
{
    std::lock_guard<std::recursive_mutex> lock(deviceMutex_);
    if (fdRx_ >= 0) return; // Already initialized

    fdRx_ = xopen(rxPath_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fdRx_ < 0) throw std::runtime_error("RX device not available: " + rxPath_);

    size_t page = size_t(sysconf(_SC_PAGESIZE));
    auto page_align = [&](size_t n){ return (n + page - 1) & ~(page - 1); };

    csi_ring_info ri{};
    if (ioctl(fdRx_, CSI_IOC_GET_RING_INFO, &ri) == 0)
    {
        rxRingSize_  = size_t(ri.ring_size);
        rxSpanBytes_ = size_t(ri.span_bytes);
        size_t mapLen = page_align(rxRingSize_);
        void *p = ::mmap(nullptr, mapLen, PROT_READ, MAP_SHARED, fdRx_, 0);
        if (p != MAP_FAILED) {
            rxRing_ = p;
            rxMapLen_ = mapLen;
            SoapySDR::logf(SOAPY_SDR_INFO, "CSI zero-copy ring mapped: size=%zu span=%zu", rxRingSize_, rxSpanBytes_);
        } else {
            SoapySDR::logf(SOAPY_SDR_WARNING, "CSI mmap failed: %s", std::strerror(errno));
        }
    }
}

void MipiDevice::initTx()
{
    std::lock_guard<std::recursive_mutex> lock(deviceMutex_);
    if (fdTx_ >= 0) return; // Already initialized

    fdTx_ = xopen(txPath_.c_str(), O_RDWR | O_NONBLOCK);
    if (fdTx_ < 0) throw std::runtime_error("TX device not available: " + txPath_);

    size_t page = size_t(sysconf(_SC_PAGESIZE));
    auto page_align = [&](size_t n){ return (n + page - 1) & ~(page - 1); };

    dsi_fb_info info{};
    if (::ioctl(fdTx_, DSI_IOC_GET_FB_INFO, &info) == 0 && info.fb_bytes && info.fb_count)
    {
        txFbBytes_  = size_t(info.fb_bytes);
        txFbCount_  = uint32_t(info.fb_count);
        txFrameBytes_ = txFbBytes_;
    }
    else
    {
        txFbBytes_  = txFrameBytes_;
        txFbCount_  = 4;
        SoapySDR::logf(SOAPY_SDR_WARNING,
                       "DSI GET_FB_INFO fallback (errno=%d %s). bytes/frame=%zu count=%u",
                       errno, std::strerror(errno), txFbBytes_, txFbCount_);
    }

    size_t total  = txFbBytes_ * size_t(txFbCount_);
    size_t mapLen = page_align(total);

    void *p = ::mmap(nullptr, mapLen, PROT_WRITE, MAP_SHARED, fdTx_, 0);
    if (p != MAP_FAILED) {
        txStaging_ = p;
        txMapLen_  = mapLen;
        SoapySDR::logf(SOAPY_SDR_INFO, "DSI staging mapped: frames=%u bytes/frame=%zu mapLen=%zu",
                       txFbCount_, txFbBytes_, txMapLen_);
    } else {
        SoapySDR::logf(SOAPY_SDR_WARNING, "DSI mmap failed: %s", std::strerror(errno));
    }
}

size_t MipiDevice::getNumChannels(const int dir) const
{
    if (dir == SOAPY_SDR_RX) return 4;
    return 1;
}

std::vector<std::string> MipiDevice::getStreamFormats(const int dir, const size_t) const
{
    (void)dir;
    return {SOAPY_SDR_CS8, SOAPY_SDR_CF32};
}

std::string MipiDevice::getNativeStreamFormat(const int dir, const size_t, double &fullScale) const
{
    (void)dir;
    fullScale = 127.0;
    return SOAPY_SDR_CS8;
}

double MipiDevice::getSampleRate(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_TX) {
        if (lastTxRate_ > 0.0) return lastTxRate_;
        return txLineRate_;
    }

    if (dir == SOAPY_SDR_RX) {
        if (lastRxRate_ > 0.0) return lastRxRate_;
        return RX_LINE_RATE;
    }
    return 0.0;
}

SoapySDR::RangeList MipiDevice::getSampleRateRange(const int dir, const size_t) const
{
    return { SoapySDR::Range(1.0e6, 80.0e6) };
}

SoapySDR::RangeList MipiDevice::getFrequencyRange(const int, const size_t) const { return { SoapySDR::Range(4.9e9, 6.0e9) }; }
SoapySDR::Range MipiDevice::getGainRange(const int, const size_t) const { return SoapySDR::Range(-30.0, 10.0); }

void MipiDevice::writeSetting(const std::string &key, const std::string &value) 
{
    (void)value;
    if (key == "bypass_iir") {
        SoapySDR::log(SOAPY_SDR_INFO, "RX IIR Filter is permanently bypassed/removed in this build.");
    }
}

std::string MipiDevice::readSetting(const std::string &key) const 
{
    if (key == "rx_dev") return rxPath_;
    if (key == "tx_dev") return txPath_;
    if (key == "bypass_iir") return "true";
    return "";
}

SoapySDR::Stream *MipiDevice::setupStream(const int dir, const std::string &format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args)
{
    (void)args;

    std::lock_guard<std::recursive_mutex> dlock(deviceMutex_);

    auto normalizedFormat = format.empty() ? std::string(SOAPY_SDR_CS8) : format;
    if (normalizedFormat == "fc32") normalizedFormat = SOAPY_SDR_CF32;

    if (normalizedFormat != SOAPY_SDR_CS8 && normalizedFormat != SOAPY_SDR_CF32) {
        throw std::runtime_error("Unsupported stream format: " + normalizedFormat);
    }

    std::unique_ptr<MipiStream> holder(new MipiStream);
    auto *s = holder.get();
    s->dir = dir;
    s->format = normalizedFormat;
    s->channels = channels;
    s->active = false;
    s->id = nextStreamId_++;

    try {
        if (dir == SOAPY_SDR_RX) {
            std::lock_guard<std::recursive_mutex> rlock(rxMutex_);
            if (rxStreamOpen_) {
                throw std::runtime_error("RX stream is already open; close the existing RX stream before setupStream(RX)");
            }

            initRx();
            rxRequestedFormat_ = normalizedFormat;
            rxChannels_ = channels;

            if (rxChannels_.empty() || rxChannels_.size() == 1) {
                rxFilter_config_(lastRxRate_);
            }

            rxStreamOpen_ = true;
            SoapySDR::logf(SOAPY_SDR_INFO,
                           "MipiDevice::setupStream RX stream=%llu format=%s channels=%zu",
                           (unsigned long long)s->id, s->format.c_str(), s->channels.size());
            return reinterpret_cast<SoapySDR::Stream*>(holder.release());
        }

        if (dir == SOAPY_SDR_TX) {
            std::lock_guard<std::recursive_mutex> tlock(txMutex_);
            if (txStreamOpen_) {
                throw std::runtime_error("TX stream is already open; close the existing TX stream before setupStream(TX)");
            }

            initTx();
            txRequestedFormat_ = normalizedFormat;
            txResampler_config_((lastTxRate_ > 0.0) ? lastTxRate_ : txLineRate_);

            txStreamOpen_ = true;
            SoapySDR::logf(SOAPY_SDR_INFO,
                           "MipiDevice::setupStream TX stream=%llu format=%s channels=%zu",
                           (unsigned long long)s->id, s->format.c_str(), s->channels.size());
            return reinterpret_cast<SoapySDR::Stream*>(holder.release());
        }

        throw std::runtime_error("Unsupported stream direction");
    } catch (...) {
        throw;
    }
}

void MipiDevice::closeStream(SoapySDR::Stream *stream)
{
    auto *s = reinterpret_cast<MipiStream*>(stream);
    if (!s) return;

    if (s->dir == SOAPY_SDR_RX) {
        std::lock_guard<std::recursive_mutex> lock(rxMutex_);
        if (s->active) s->active = false;
        rxChannels_.clear();
        resampler_.reset();
        rxFloatBuf_.reset();
        rxStreamOpen_ = false;
        SoapySDR::logf(SOAPY_SDR_INFO,
                       "MipiDevice::closeStream RX stream=%llu",
                       (unsigned long long)s->id);
    } else if (s->dir == SOAPY_SDR_TX) {
        std::lock_guard<std::recursive_mutex> lock(txMutex_);
        if (s->active) {
            txRingFlush_(100000);
            s->active = false;
        }
        txResampler_.reset();
        txFloatBuf_.reset();
        txOutBytes_.clear();
        txOutOff_ = 0;
        txRing_.reset();
        txHeadIndex_ = 0;
        txHeadOff_ = 0;
        txStreamOpen_ = false;
        SoapySDR::logf(SOAPY_SDR_INFO,
                       "MipiDevice::closeStream TX stream=%llu",
                       (unsigned long long)s->id);
    }

    delete s;
}

int MipiDevice::activateStream(SoapySDR::Stream *stream, const int, const long long, const size_t)
{
    auto *s = reinterpret_cast<MipiStream*>(stream);
    if (!s) return SOAPY_SDR_STREAM_ERROR;

    if (s->dir == SOAPY_SDR_RX) {
        std::lock_guard<std::recursive_mutex> lock(rxMutex_);
        if (s->active) return 0;
        rxFloatBuf_.reset();
        s->active = true;
        SoapySDR::logf(SOAPY_SDR_INFO,
                       "MipiDevice::activateStream RX stream=%llu fdRx_=%d rxRing_=%p rxRingSize_=%zu sampleRateRatio_=%.9f",
                       (unsigned long long)s->id, fdRx_, rxRing_, rxRingSize_, sampleRateRatio_);
        return 0;
    }

    if (s->dir != SOAPY_SDR_TX) return SOAPY_SDR_STREAM_ERROR;

    std::lock_guard<std::recursive_mutex> lock(txMutex_);
    if (s->active) return 0;

    SoapySDR::logf(
        SOAPY_SDR_INFO,
        "MipiDevice::activateStream TX stream=%llu fdTx_=%d txStaging_=%p txFbBytes_=%zu txFbCount_=%u txSampleRateRatio_=%.9f",
        (unsigned long long)s->id, fdTx_, txStaging_, txFbBytes_, txFbCount_, txSampleRateRatio_);

    if (fdTx_ >= 0)
    {
        txRingInit_();
        txRing_.reset();
        txHeadIndex_ = 0;
        txHeadOff_   = 0;

        for (int tries = 0; tries < 20; ++tries)
        {
            int rc = xpoll(fdTx_, POLLOUT, 10);
            if (rc > 0) break;
            ::usleep(10000);
        }

        if (txStaging_ && txFbBytes_)
        {
            const unsigned warmFrames = 2;
            const long warmTimeoutUs  = 200000;
            const int perFrameRetries = 4;

            SoapySDR::logf(SOAPY_SDR_INFO,
                           "MipiDevice::activateStream TX stream=%llu: starting TX warm-up (%u frames, %zu bytes/frame)",
                           (unsigned long long)s->id, warmFrames, txFbBytes_);

            std::vector<uint8_t> zeroFrame(txFbBytes_, 0);

            for (unsigned i = 0; i < warmFrames; ++i)
            {
                ssize_t w = -EAGAIN;
                for (int r = 0; r < perFrameRetries; ++r)
                {
                    w = tx_write_staging(zeroFrame.data(), txFbBytes_, warmTimeoutUs);
                    if (w == (ssize_t)txFbBytes_) break;
                    if (w == -EAGAIN) { ::usleep(10000); continue; }
                    break;
                }

                if (w < 0)
                {
                    SoapySDR::logf(SOAPY_SDR_WARNING,
                                   "MipiDevice::activateStream TX stream=%llu: warm-up frame %u failed: %zd (errno=%d %s)",
                                   (unsigned long long)s->id, i, w, errno, std::strerror(errno));
                    break;
                }
                if (size_t(w) < txFbBytes_)
                {
                    SoapySDR::logf(SOAPY_SDR_WARNING,
                                   "MipiDevice::activateStream TX stream=%llu: warm-up frame %u short write: %zd of %zu",
                                   (unsigned long long)s->id, i, w, txFbBytes_);
                    break;
                }
            }

            SoapySDR::logf(SOAPY_SDR_INFO,
                           "MipiDevice::activateStream TX stream=%llu: TX warm-up sequence complete",
                           (unsigned long long)s->id);
        }
    }

    s->active = true;
    return 0;
}

int MipiDevice::deactivateStream(SoapySDR::Stream *stream, const int, const long long)
{
    auto *s = reinterpret_cast<MipiStream*>(stream);
    if (!s) return SOAPY_SDR_STREAM_ERROR;

    if (s->dir == SOAPY_SDR_RX) {
        std::lock_guard<std::recursive_mutex> lock(rxMutex_);
        s->active = false;
        SoapySDR::logf(SOAPY_SDR_INFO,
                       "MipiDevice::deactivateStream RX stream=%llu",
                       (unsigned long long)s->id);
        return 0;
    }

    if (s->dir == SOAPY_SDR_TX) {
        std::lock_guard<std::recursive_mutex> lock(txMutex_);
        txRingFlush_(100000);
        s->active = false;
        SoapySDR::logf(SOAPY_SDR_INFO,
                       "MipiDevice::deactivateStream TX stream=%llu",
                       (unsigned long long)s->id);
        return 0;
    }

    return SOAPY_SDR_STREAM_ERROR;
}

std::vector<std::string> MipiDevice::listAntennas(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_TX) return {"TX"};
    return {"RX"};
}

void MipiDevice::setAntenna(const int dir, const size_t, const std::string &name)
{
    if (dir == SOAPY_SDR_TX) {
        if (name != "TX") throw std::runtime_error("Unsupported TX antenna: " + name);
        txAntennaSel_ = name;
        return;
    }
    if (name != "RX") throw std::runtime_error("Unsupported RX antenna: " + name);
    rxAntennaSel_ = name;
}

std::string MipiDevice::getAntenna(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_TX) return txAntennaSel_;
    return rxAntennaSel_;
}

std::vector<std::string> MipiDevice::listGains(const int, const size_t) const { return {"RF"}; }
SoapySDR::Range MipiDevice::getGainRange(const int, const size_t, const std::string &) const { return SoapySDR::Range(+1.0, +63.0); }
void MipiDevice::setGain(const int, const size_t, const std::string &, const double value) { gain_ = value; }
double MipiDevice::getGain(const int, const size_t, const std::string &) const { return gain_; }
std::vector<std::string> MipiDevice::listFrequencies(const int, const size_t) const { return {"BB"}; }
void MipiDevice::setFrequency(const int, const size_t, const std::string &, const double, const SoapySDR::Kwargs &) { }
double MipiDevice::getFrequency(const int, const size_t, const std::string &) const { return 0.0; }
SoapySDR::RangeList MipiDevice::getFrequencyRange(const int, const size_t, const std::string &name) const {
    if (name == "BB") return { SoapySDR::Range(0.0, 0.0) };
    return { SoapySDR::Range(0.0, 0.0) };
}

std::vector<double> MipiDevice::listSampleRates(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_RX) return { 30.72e6, 40e6, 80e6 }; // We support any sample rate, but give a few examples
    if (dir == SOAPY_SDR_TX) return { 40e6 }; // Also support any rate
    return {};
}

void MipiDevice::setSampleRate(const int dir, const size_t, const double rate)
{
    if (dir == SOAPY_SDR_RX) {
        std::lock_guard<std::recursive_mutex> lock(rxMutex_);
        lastRxRate_ = rate;
        if (rate > 0.0) sampleRateRatio_ = kFsIn / rate;
        else            sampleRateRatio_ = 1.0;

        const double blockPairs = 32768.0 * sampleRateRatio_ + 1024.0;
        size_t rxMaxPairs = size_t(std::max(4096.0, blockPairs * 8.0));
        rxFloatBuf_.init(rxMaxPairs * 2);

        rxFilter_config_(rate);
        SoapySDR::logf(SOAPY_SDR_INFO,
                       "MipiDevice::setSampleRate RX %.6f Msps ratio=%.9f",
                       rate / 1e6, sampleRateRatio_);
        return;
    }

    if (dir == SOAPY_SDR_TX) {
        std::lock_guard<std::recursive_mutex> lock(txMutex_);
        lastTxRate_ = rate;
        txResampler_config_(rate);
        SoapySDR::logf(SOAPY_SDR_INFO,
                       "MipiDevice::setSampleRate TX %.6f Msps ratio=%.9f",
                       rate / 1e6, txSampleRateRatio_);
        return;
    }
}

int MipiDevice::readStream(SoapySDR::Stream *stream, void * const *buffs,
                           const size_t numElems, int &flags,
                           long long &timeNs, const long timeoutUs)
{
    auto *s = reinterpret_cast<MipiStream*>(stream);
    if (!s || s->dir != SOAPY_SDR_RX) return SOAPY_SDR_STREAM_ERROR;

    std::lock_guard<std::recursive_mutex> lock(rxMutex_);

    flags = 0;
    timeNs = 0;

    if (fdRx_ < 0) return SOAPY_SDR_NOT_SUPPORTED;
    if (buffs == nullptr || buffs[0] == nullptr) return SOAPY_SDR_STREAM_ERROR;
    if (numElems == 0) return 0;

    const bool wantCF32 = (rxRequestedFormat_ == SOAPY_SDR_CF32);
    
    if (rxChannels_.size() == 4) return SOAPY_SDR_NOT_SUPPORTED; 

    float* fBufOut = nullptr;
    if (wantCF32) {
        fBufOut = static_cast<float*>(buffs[0]);
    } else {
        if (resampScratch_.size() < numElems * 2) resampScratch_.resize(numElems * 2);
        fBufOut = resampScratch_.data();
    }

    if (rxFloatBuf_.capacity() == 0) {
        rxFloatBuf_.init(32768 * 2); 
    }

    size_t totalProduced = 0;
    long loopTimeoutUs = timeoutUs;

    while (totalProduced < numElems) {
        size_t remainingOutput = numElems - totalProduced;
        
        size_t inputNeeded = (size_t)ceil(remainingOutput * sampleRateRatio_) + 16;
        inputNeeded = (inputNeeded + 7) & ~7; 

        size_t mtu = this->getStreamMTU(stream);
        if (inputNeeded > mtu) inputNeeded = mtu;

        const size_t bytesReq = inputNeeded * 2; 
        if (rxScratch_.size() < bytesReq) rxScratch_.resize(bytesReq);

        const ssize_t gotBytes = (rxRing_ ? rx_read_ring(rxScratch_.data(), bytesReq, loopTimeoutUs)
                                          : rx_read_legacy(rxScratch_.data(), bytesReq, loopTimeoutUs));

        if (gotBytes == -EAGAIN) {
            if (totalProduced > 0) break;
            return SOAPY_SDR_TIMEOUT;
        }
        if (gotBytes < 0) {
            if (totalProduced > 0) break;
            return SOAPY_SDR_STREAM_ERROR;
        }

        const size_t gotSamples = size_t(gotBytes / 2); 
        
        if (gotSamples > 0) {
            float* writePtr = rxFloatBuf_.prepareWrite(gotSamples * 2);
            convert_CS8_to_CF32_NEON(
                reinterpret_cast<const int8_t*>(rxScratch_.data()), 
                writePtr, 
                gotSamples
            ); 
            rxFloatBuf_.commitWrite(gotSamples * 2);
        }

        while (totalProduced < numElems && rxFloatBuf_.readAvail() >= 8) {
            size_t inAvailPairs = rxFloatBuf_.readAvail() / 2;
            int inConsumedPairs = 0;
            int produced = resampler_.process(
                rxFloatBuf_.readPtr(), inAvailPairs, 
                fBufOut + (totalProduced * 2), remainingOutput, 
                sampleRateRatio_, inConsumedPairs
            );

            rxFloatBuf_.consume(inConsumedPairs * 2);
            totalProduced += produced;
            remainingOutput -= produced;
            
            if (produced == 0 && inConsumedPairs == 0) break;
        }

        loopTimeoutUs = 2000; 
    }

    if (!wantCF32 && totalProduced > 0) {
        convert_CF32_to_CS8_NEON(fBufOut, static_cast<int8_t*>(buffs[0]), totalProduced);
    }

    return totalProduced;
}

void MipiDevice::txRingInit_()
{
    const double lineBytesPerSec = 2.0 * txLineRate_;
    const double defaultQueueSeconds = 0.10; 
    size_t want = size_t(lineBytesPerSec * defaultQueueSeconds);

    const size_t minFrames = 2;
    size_t minBytes = txFbBytes_ ? (txFbBytes_ * minFrames) : size_t(1<<20);
    want = std::max(want, minBytes);
    want = std::min<size_t>(want, 32u * 1024u * 1024u);

    txRing_.init(want);
    txRingMaxBytes_ = txRing_.capacity();

    const size_t outPairsCap = txRingMaxBytes_ / 2; 
    const double r = (txSampleRateRatio_ > 0.0) ? txSampleRateRatio_ : 1.0;
    size_t txInMaxPairs = size_t(double(outPairsCap) * r) + 256; 
    
    txFloatBuf_.init(txInMaxPairs * 2);
    txOutFloatScratch_.reserve((outPairsCap + 256) * 2);
    txScratch_.reserve(txRingMaxBytes_);
}

void MipiDevice::txRingFlush_(long timeoutUs)
{
    if (fdTx_ < 0) return;
    if (txRing_.capacity() == 0) return;

    while (txRing_.size() != 0)
    {
        auto span = txRing_.readSpan();
        if (!span.first || span.second == 0) break;

        size_t toWrite = span.second;
        if (txFbBytes_) toWrite = std::min(toWrite, txFbBytes_);

        ssize_t w = 0;
        if (txStaging_ && txFbBytes_) w = tx_write_staging(span.first, toWrite, timeoutUs);
        else                          w = tx_write_legacy (span.first, toWrite, timeoutUs);

        if (w > 0) {
            txRing_.consume(size_t(w));
            timeoutUs = 0;
            continue;
        }
        if (w == -EAGAIN) break;
        break;
    }
}

int MipiDevice::writeStream(
    SoapySDR::Stream *stream,
    const void * const *buffs,
    const size_t numElems,
    int &flags,
    const long long /*timeNs*/,
    const long timeoutUs)
{
    auto *s = reinterpret_cast<MipiStream*>(stream);
    if (!s || s->dir != SOAPY_SDR_TX) return SOAPY_SDR_STREAM_ERROR;

    std::lock_guard<std::recursive_mutex> lock(txMutex_);

    (void)flags;
    if (fdTx_ < 0) return SOAPY_SDR_NOT_SUPPORTED;
    if (!buffs || !buffs[0]) return SOAPY_SDR_STREAM_ERROR;
    if (numElems == 0) return 0;

    if (numElems > (std::numeric_limits<size_t>::max()/2)) return SOAPY_SDR_STREAM_ERROR;

    if (txRing_.capacity() == 0) txRingInit_();
    txRingFlush_(0);

    // PATH A: Resampling Enabled (Host Rate != Line Rate)
    if (txResampler_.isEnabled())
    {
        if (txFloatBuf_.capacity() == 0) {
            const size_t outPairsCap = txRingMaxBytes_ ? (txRingMaxBytes_ / 2) : (txRing_.capacity() / 2);
            const double r = (txSampleRateRatio_ > 0.0) ? txSampleRateRatio_ : 1.0;
            size_t txInMaxPairs = size_t(double(outPairsCap) * r) + 256;
            txFloatBuf_.init(txInMaxPairs * 2);
        }

        size_t logicalFreePairs = (txFloatBuf_.capacity() - txFloatBuf_.readAvail()) / 2;
        if (logicalFreePairs == 0) {
            txProduceToRing_(timeoutUs);
            logicalFreePairs = (txFloatBuf_.capacity() - txFloatBuf_.readAvail()) / 2;
            if (logicalFreePairs == 0) return SOAPY_SDR_TIMEOUT;
        }

        const size_t mtu = this->getStreamMTU(stream);
        size_t acceptPairs = std::min(numElems, logicalFreePairs);
        if (acceptPairs > mtu) acceptPairs = mtu;
        if (acceptPairs == 0) return SOAPY_SDR_TIMEOUT;

        float* writePtr = txFloatBuf_.prepareWrite(acceptPairs * 2);
        
        if (txRequestedFormat_ == SOAPY_SDR_CF32) {
            const float *src = static_cast<const float*>(buffs[0]);
            std::memcpy(writePtr, src, acceptPairs * 2 * sizeof(float));
        } else {
            convert_CS8_to_CF32_NEON(
                static_cast<const int8_t*>(buffs[0]),
                writePtr,
                acceptPairs
            );
        }
        
        txFloatBuf_.commitWrite(acceptPairs * 2);
        txProduceToRing_(0);
        return (int)acceptPairs;
    }

    // PATH B: Native Rate / Bypass (No Resampling)
    {
        const size_t mtuElems = this->getStreamMTU(stream);
        const size_t reqElems = std::min(numElems, mtuElems);

        const size_t reqBytes = reqElems * 2;
        if (txRing_.free() < reqBytes) {
            txRingFlush_(timeoutUs);
            if (txRing_.free() < reqBytes) return SOAPY_SDR_TIMEOUT;
        }

        if (txRequestedFormat_ == SOAPY_SDR_CS8) {
            const uint8_t *src = static_cast<const uint8_t*>(buffs[0]);
            const size_t wrote = txRing_.write(src, reqBytes);
            if (wrote != reqBytes) return SOAPY_SDR_TIMEOUT;
        } else {
            if (txScratch_.size() < reqBytes) txScratch_.resize(reqBytes);
            convert_CF32_to_CS8_NEON(
                static_cast<const float*>(buffs[0]),
                reinterpret_cast<int8_t*>(txScratch_.data()),
                reqElems
            );
            const size_t wrote = txRing_.write(txScratch_.data(), reqBytes);
            if (wrote != reqBytes) return SOAPY_SDR_TIMEOUT;
        }

        txRingFlush_(0);
        return (int)reqElems;
    }
}

SoapySDR::Kwargs MipiDevice::getHardwareInfo() const
{
    SoapySDR::Kwargs k;
    k["label"] = "RP1 MIPI (CS8)";
    k["rx_dev"] = rxPath_;
    k["tx_dev"] = txPath_;
    return k;
}

size_t MipiDevice::getStreamMTU(SoapySDR::Stream *stream) const
{
    const auto *s = reinterpret_cast<const MipiStream*>(stream);

    if (s && s->dir == SOAPY_SDR_RX) {
        const size_t rxFrameSize = (s->channels.size() == 4) ? 8 : 2;
        size_t rxElems = rxSpanBytes_ ? (rxSpanBytes_ / rxFrameSize) :
                         (rxChunkBytes_ ? (rxChunkBytes_ / rxFrameSize) : size_t(16384));
        return rxElems ? rxElems : size_t(16384);
    }

    if (s && s->dir == SOAPY_SDR_TX) {
        size_t txElems = txFbBytes_ ? (txFbBytes_ / 2) : size_t(16384);
        return txElems ? txElems : size_t(16384);
    }

    // Backward-compatible fallback for internal calls that do not have a stream handle.
    size_t txElems = txFbBytes_ ? (txFbBytes_ / 2) : size_t(16384);
    size_t rxFrameSize = (rxChannels_.size() == 4) ? 8 : 2;
    size_t rxElems = rxSpanBytes_ ? (rxSpanBytes_ / rxFrameSize) :
                     (rxChunkBytes_ ? (rxChunkBytes_ / rxFrameSize) : size_t(16384));
    size_t safeElems = txElems;
    if (rxElems) safeElems = std::min(txElems, rxElems);
    if (safeElems == 0) safeElems = 16384;
    return safeElems;
}

int MipiDevice::xopen(const char *path, int flags)
{
    int fd = ::open(path, flags);
    if (fd < 0)
        SoapySDR::logf(SOAPY_SDR_WARNING, "open(%s) failed: %s", path, std::strerror(errno));
    return fd;
}

int MipiDevice::xpoll(int fd, short events, int timeoutMs)
{
    struct pollfd pfd{fd, events, 0};
    int rc = ::poll(&pfd, 1, timeoutMs);
    if (rc <= 0) return rc;

    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
    {
        SoapySDR::logf(SOAPY_SDR_WARNING,
                       "xpoll(fd=%d, events=0x%x) -> rc=%d revents=0x%x (ERR/HUP/NVAL)",
                       fd, unsigned(events), rc, unsigned(pfd.revents));
        errno = EIO;
        return -1;
    }
    return rc;
}

ssize_t MipiDevice::rx_read_legacy(void *dst, size_t bytes, long timeoutUs)
{
    int timeoutMs = (timeoutUs > 0) ? std::max<int>(1, int(timeoutUs / 1000)) : 0;
    int rc = xpoll(fdRx_, POLLIN, timeoutMs);
    if (rc == 0) return -EAGAIN;
    if (rc < 0)  return -EIO;

    ssize_t g = ::read(fdRx_, dst, bytes);
    if (g < 0) { 
        if (errno == EAGAIN) { ::usleep(500); return -EAGAIN; } 
        return -EIO; 
    }
    return g;
}

ssize_t MipiDevice::rx_read_ring(void *dst, size_t bytes, long timeoutUs)
{
    int timeoutMs = (timeoutUs > 0) ? std::max<int>(1, int(timeoutUs / 1000)) : 0;
    int rc = xpoll(fdRx_, POLLIN, timeoutMs);
    if (rc == 0) return -EAGAIN;
    if (rc < 0)  return -EIO;

    csi_ring_info ri{};
    if (ioctl(fdRx_, CSI_IOC_GET_RING_INFO, &ri) != 0) return -EIO;

    size_t head = ri.head, tail = ri.tail;
    size_t used = (head >= tail) ? (head - tail) : (rxRingSize_ - (tail - head));
    if (!used) {
        ::usleep(500);
        return -EAGAIN;
    }

    size_t n1 = std::min(used, rxRingSize_ - tail);
    size_t want = std::min(n1, bytes);

    std::memcpy(dst, static_cast<const uint8_t*>(rxRing_) + tail, want);

    __u32 cons = (__u32)want;
    if (ioctl(fdRx_, CSI_IOC_CONSUME_BYTES, &cons) != 0) return -EIO;

    return (ssize_t)want;
}

ssize_t MipiDevice::tx_write_legacy(const void *src, size_t bytes, long timeoutUs)
{
    int timeoutMs = (timeoutUs > 0) ? std::max<int>(1, int(timeoutUs / 1000)) : 0;
    int rc = xpoll(fdTx_, POLLOUT, timeoutMs);
    if (rc == 0) return -EAGAIN;
    if (rc < 0)  return -EIO;

    ssize_t w = ::write(fdTx_, src, bytes);
    if (w < 0) { 
        if (errno == EAGAIN) { ::usleep(500); return -EAGAIN; } 
        return -EIO; 
    }
    return w;
}

ssize_t MipiDevice::tx_write_staging(const void *src, size_t bytes, long timeoutUs)
{
    if (!txStaging_ || txFbBytes_ == 0 || txFbCount_ == 0 || txMapLen_ == 0)
        return -EIO;

    const uint8_t *p = static_cast<const uint8_t*>(src);
    size_t remaining = bytes;
    size_t written   = 0;
    int timeoutMs = (timeoutUs > 0) ? std::max<int>(1, int(timeoutUs / 1000)) : 0;

    while (remaining)
    {
        if (txHeadOff_ == 0)
        {
            dsi_fb_info info{};
            if (::ioctl(fdTx_, DSI_IOC_GET_FB_INFO, &info) == 0)
            {
                const size_t drvBytes = size_t(info.fb_bytes);
                const size_t drvCnt   = size_t(info.fb_count);
                const size_t drvTotal = drvBytes * drvCnt;

                if (drvBytes == 0 || drvCnt == 0) return (written ? (ssize_t)written : -EAGAIN);

                if (drvTotal > txMapLen_)
                {
                    SoapySDR::logf(SOAPY_SDR_ERROR,
                                   "tx_write_staging: driver fb_total=%zu exceeds mapped txMapLen_=%zu. Refusing to write.",
                                   drvTotal, txMapLen_);
                    return (written ? (ssize_t)written : -EIO);
                }

                if (drvBytes != txFbBytes_ || drvCnt != txFbCount_)
                {
                    txFbBytes_ = drvBytes;
                    txFbCount_ = uint32_t(drvCnt);
                }

                txHeadIndex_ = info.head;
            }
            else
            {
                return (written ? (ssize_t)written : -EAGAIN);
            }

            int rc = xpoll(fdTx_, POLLOUT, timeoutMs);
            if (rc == 0) return (written ? (ssize_t)written : -EAGAIN);
            if (rc < 0)  return (written ? (ssize_t)written : -EIO);
        }

        size_t space  = txFbBytes_ - txHeadOff_;
        size_t toCopy = std::min(remaining, space);

        uint8_t *dst = static_cast<uint8_t*>(txStaging_)
                     + (size_t(txHeadIndex_) % size_t(txFbCount_)) * txFbBytes_
                     + txHeadOff_;

        std::memcpy(dst, p, toCopy);
        p          += toCopy;
        remaining  -= toCopy;
        written    += toCopy;
        txHeadOff_ += toCopy;

        if (txHeadOff_ == txFbBytes_)
        {
            std::atomic_thread_fence(std::memory_order_seq_cst);

            if (ioctl(fdTx_, DSI_IOC_QUEUE_NEXT) != 0)
            {
                if (errno == EAGAIN) {
                    ::usleep(500); 
                    return (written ? (ssize_t)written : -EAGAIN);
                }
                return (written ? (ssize_t)written : -EIO);
            }

            txHeadIndex_ = (txHeadIndex_ + 1) % txFbCount_;
            txHeadOff_   = 0;
        }
    }

    return (ssize_t)written;
}

void MipiDevice::txResampler_config_(double hostRate)
{
    if (txLineRate_ <= 0.0) {
        const double txPayloadBytes = double(txBytesPerLine_) * double(txLines_);
        const double txTotalBytes   = txPayloadBytes
                                    + double(txOverheadBytesPerLine_) * double(txLines_)
                                    + double(txFrameExtraBytes_);
        txPayloadFraction_ = (txTotalBytes > 0.0) ? (txPayloadBytes / txTotalBytes) : 1.0;
        txLineRate_ = 0.5 * txDsiByteRate_ * txPayloadFraction_;
    }

    txResampler_.reset();
    txFloatBuf_.reset();
    txOutBytes_.clear();
    txOutOff_ = 0;

    if (txLineRate_ <= 0.0 || hostRate <= 0.0 || std::abs(hostRate - txLineRate_) < 1.0e3) {
        txResampler_.setEnabled(false);
        txSampleRateRatio_ = 1.0;
        SoapySDR::log(SOAPY_SDR_INFO, "TX DSP bypassed (native line rate)");
        return;
    }

    txSampleRateRatio_ = hostRate / txLineRate_;
    txResampler_.setEnabled(true);

    if (txRingMaxBytes_ > 0) {
        const size_t outPairsCap = txRingMaxBytes_ / 2;
        size_t txInMaxPairs = size_t(double(outPairsCap) * txSampleRateRatio_) + 256;
        txFloatBuf_.init(txInMaxPairs * 2);
    }

    SoapySDR::logf(SOAPY_SDR_INFO,
                   "TX resampler enabled: host=%.6f Msps -> payload=%.6f Msps (ratio=%.6f, dsi_byte=%.6f MHz, payload_fraction=%.9f)",
                   hostRate/1e6, txLineRate_/1e6, txSampleRateRatio_,
                   txDsiByteRate_/1e6, txPayloadFraction_);
}

void MipiDevice::txProduceToRing_(long timeoutUs)
{
    if (!txResampler_.isEnabled() || txRing_.capacity() == 0) return;

    for (;;)
    {
        size_t inPairsAvail = txFloatBuf_.readAvail() / 2;
        
        if (inPairsAvail < 4) break; 

        size_t freeBytes = txRing_.free();
        if (freeBytes < 256) {
            txRingFlush_(0);
            freeBytes = txRing_.free();
        }
        if (freeBytes < 256 && timeoutUs > 0) {
            txRingFlush_(timeoutUs);
            freeBytes = txRing_.free();
        }
        if (freeBytes < 256) break;

        size_t maxOutPairs = std::min<size_t>(freeBytes / 2, 8192); 
        if (maxOutPairs < 64) break;

        if (txOutFloatScratch_.size() < maxOutPairs * 2) txOutFloatScratch_.resize(maxOutPairs * 2);

        int inConsumedPairs = 0;
        int produced = txResampler_.process(
            txFloatBuf_.readPtr(), (int)inPairsAvail,
            txOutFloatScratch_.data(), (int)maxOutPairs,
            txSampleRateRatio_, inConsumedPairs
        );

        if (inConsumedPairs > 0) {
            txFloatBuf_.consume(inConsumedPairs * 2);
        }

        if (produced > 0) {
            const size_t producedBytes = (size_t)produced * 2;
            if (txScratch_.size() < producedBytes) txScratch_.resize(producedBytes);

            convert_CF32_to_CS8_NEON(
                txOutFloatScratch_.data(),
                reinterpret_cast<int8_t*>(txScratch_.data()),
                (size_t)produced
            );

            txRing_.write(txScratch_.data(), producedBytes);
        } else if (inConsumedPairs == 0) {
            break;
        }
    }

    txRingFlush_(0);
}

void MipiDevice::txFlush_(long timeoutUs)
{
    if (txOutOff_ >= txOutBytes_.size()) {
        txOutBytes_.clear();
        txOutOff_ = 0;
        return;
    }

    while (txOutOff_ < txOutBytes_.size()) {
        const uint8_t *src = txOutBytes_.data() + txOutOff_;
        const size_t remaining = txOutBytes_.size() - txOutOff_;

        ssize_t w = 0;
        if (txStaging_ && txFbBytes_) {
            const size_t toCopy = std::min(remaining, txFbBytes_);
            w = tx_write_staging(src, toCopy, timeoutUs);
        } else {
            w = tx_write_legacy(src, remaining, timeoutUs);
        }

        if (w > 0) {
            txOutOff_ += size_t(w);
            continue;
        }
        if (w == -EAGAIN) break;

        SoapySDR::logf(SOAPY_SDR_ERROR, "TX write failed: %zd", w);
        txOutBytes_.clear();
        txOutOff_ = 0;
        return;
    }

    if (txOutOff_ == txOutBytes_.size()) {
        txOutBytes_.clear();
        txOutOff_ = 0;
    } else if (txOutOff_ > (1u<<20)) { 
        const size_t remaining = txOutBytes_.size() - txOutOff_;
        std::memmove(txOutBytes_.data(), txOutBytes_.data() + txOutOff_, remaining);
        txOutBytes_.resize(remaining);
        txOutOff_ = 0;
    }
}

void MipiDevice::rxFilter_config_(double fsOut)
{
    if (std::abs(fsOut - kFsIn) < 1.0e3 || fsOut <= 0.0 || fsOut >= kFsIn) {
        resampler_.setEnabled(false);
        sampleRateRatio_ = 1.0;
        rxFloatBuf_.reset();
        SoapySDR::log(SOAPY_SDR_INFO, "RX DSP bypassed (native rate requested)");
        return;
    }

    resampler_.setEnabled(true);
    rxFloatBuf_.reset();

    SoapySDR::logf(SOAPY_SDR_INFO, "Configuring RX Farrow Resampler: In=%.2f MHz, Out=%.2f MHz", 
                   kFsIn/1e6, fsOut/1e6);
}