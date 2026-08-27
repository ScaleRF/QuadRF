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
#include <sstream>
#include <cctype>
#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <sys/wait.h>

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

    if (args.count("jtag")) {
        jtagPath_ = args.at("jtag");
    } else if (const char *env = std::getenv("QUADRF_JTAG"); env && env[0]) {
        jtagPath_ = env;
    } else {
        jtagPath_ = "/usr/bin/quadrf-jtag";
    }

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

namespace {

constexpr double kLoMinHz = 4.9e9;
constexpr double kLoMaxHz = 6.0e9;
constexpr double kRxBwMinHz = 240.0e6 / 63.0;
constexpr double kRxBwMaxHz = 240.0e6 / 5.0;
constexpr double kRateMinHz = 1.0e6;
constexpr double kRateMaxHz = 100.0e6;
constexpr uint64_t kStatusTtlNs = 150000000ull;

double clampVal(double v, double lo, double hi)
{
    return std::min(hi, std::max(lo, v));
}

double clampRfHz(double freq)
{
    return clampVal(freq, kLoMinHz, kLoMaxHz);
}

uint64_t monotonicNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000000ull + uint64_t(ts.tv_nsec);
}

std::string fmtDouble(double v, int prec)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
    return buf;
}

std::string toLower(std::string s)
{
    for (char &c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string trim(const std::string &s)
{
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) {
        start++;
    }
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        end--;
    }
    return s.substr(start, end - start);
}

bool parseBool(const std::string &s, bool def = false)
{
    const std::string v = toLower(trim(s));
    if (v.empty()) return def;
    if (v == "1" || v == "true" || v == "on" || v == "yes" || v == "enable" || v == "enabled") return true;
    if (v == "0" || v == "false" || v == "off" || v == "no" || v == "disable" || v == "disabled") return false;
    SoapySDR::logf(SOAPY_SDR_WARNING, "parseBool: invalid boolean value '%s', defaulting to %s",
                   s.c_str(), def ? "true" : "false");
    return def;
}

double parseDouble(const std::string &s, const char *what)
{
    const std::string t = trim(s);
    char *end = nullptr;
    errno = 0;
    const double v = std::strtod(t.c_str(), &end);
    if (errno != 0 || end == t.c_str() || (end && *end != '\0') || !std::isfinite(v)) {
        throw std::runtime_error(std::string("invalid ") + what + ": " + s);
    }
    return v;
}

long parseLong(const std::string &s, const char *what)
{
    const std::string t = trim(s);
    char *end = nullptr;
    errno = 0;
    const long v = std::strtol(t.c_str(), &end, 0);
    if (errno != 0 || end == t.c_str() || (end && *end != '\0')) {
        throw std::runtime_error(std::string("invalid ") + what + ": " + s);
    }
    return v;
}

bool isRfComponent(const std::string &name)
{
    return name.empty() || name == "RF";
}

SoapySDR::ArgInfo makeArg(const std::string &key, const std::string &def,
                         const std::string &name, const std::string &desc,
                         SoapySDR::ArgInfo::Type type,
                         const std::string &units = {},
                         const std::vector<std::string> &options = {},
                         SoapySDR::Range range = {})
{
    SoapySDR::ArgInfo a;
    a.key = key;
    a.value = def;
    a.name = name;
    a.description = desc;
    a.type = type;
    a.units = units;
    a.options = options;
    a.optionNames = options;
    a.range = range;
    return a;
}

double agcThrToDbfs(double thr)
{
    if (thr <= 0.0) return -40.0;
    return 20.0 * std::log10(thr / 180.0);
}

std::vector<SoapySDR::ArgInfo> rxSettingInfo()
{
    return {
        makeArg("pol", "rhcp", "Polarization", "RX polarization", SoapySDR::ArgInfo::STRING, {}, {"rhcp", "lhcp"}),
        makeArg("interleave", "false", "Interleave", "4-channel interleaved RX", SoapySDR::ArgInfo::BOOL),
        makeArg("antennas", "15", "Antennas", "RX antenna mask (bit0=Rx1)", SoapySDR::ArgInfo::INT, {}, {}, SoapySDR::Range(0, 15, 1)),
        makeArg("autosteer", "false", "Auto steer", "FPGA auto beam steer", SoapySDR::ArgInfo::BOOL),
        makeArg("tone_en", "false", "Test tone", "Enable RX test tone", SoapySDR::ArgInfo::BOOL),
        makeArg("tone_freq", "0", "Tone frequency", "RX test tone", SoapySDR::ArgInfo::FLOAT, "Hz"),
        makeArg("p1", "0", "Phase 1", "RX element 1 phase", SoapySDR::ArgInfo::FLOAT, "deg", {}, SoapySDR::Range(0, 360)),
        makeArg("p2", "0", "Phase 2", "RX element 2 phase", SoapySDR::ArgInfo::FLOAT, "deg", {}, SoapySDR::Range(0, 360)),
        makeArg("p3", "0", "Phase 3", "RX element 3 phase", SoapySDR::ArgInfo::FLOAT, "deg", {}, SoapySDR::Range(0, 360)),
        makeArg("p4", "0", "Phase 4", "RX element 4 phase", SoapySDR::ArgInfo::FLOAT, "deg", {}, SoapySDR::Range(0, 360)),
        makeArg("agc_setpoint", "-15", "AGC setpoint", "AGC target when gain mode is automatic", SoapySDR::ArgInfo::FLOAT, "dBFS", {}, SoapySDR::Range(-40, -6)),
        makeArg("analog_bw", "40000000", "Analog bandwidth", "MAX2851 analog filter (read-only)", SoapySDR::ArgInfo::FLOAT, "Hz"),
    };
}

std::vector<SoapySDR::ArgInfo> txSettingInfo()
{
    return {
        makeArg("enable", "false", "TX enable", "Enter TX mode (antennas on) or standby", SoapySDR::ArgInfo::BOOL),
        makeArg("follow_rx", "false", "TX follow RX", "Lock TX LO to RX LO", SoapySDR::ArgInfo::BOOL),
        makeArg("antennas", "15", "Antennas", "TX antenna mask (bit0=Tx1)", SoapySDR::ArgInfo::INT, {}, {}, SoapySDR::Range(0, 15, 1)),
        makeArg("tone_en", "false", "Test tone", "Enable TX test tone", SoapySDR::ArgInfo::BOOL),
        makeArg("tone_freq", "0", "Tone frequency", "TX test tone", SoapySDR::ArgInfo::FLOAT, "Hz"),
        makeArg("p1", "0", "Phase 1", "TX element 1 phase", SoapySDR::ArgInfo::FLOAT, "deg", {}, SoapySDR::Range(0, 360)),
        makeArg("p2", "0", "Phase 2", "TX element 2 phase", SoapySDR::ArgInfo::FLOAT, "deg", {}, SoapySDR::Range(0, 360)),
        makeArg("p3", "0", "Phase 3", "TX element 3 phase", SoapySDR::ArgInfo::FLOAT, "deg", {}, SoapySDR::Range(0, 360)),
        makeArg("p4", "0", "Phase 4", "TX element 4 phase", SoapySDR::ArgInfo::FLOAT, "deg", {}, SoapySDR::Range(0, 360)),
        makeArg("analog_bw", "40000000", "Analog bandwidth", "MAX2850 analog filter 20 or 40 MHz", SoapySDR::ArgInfo::FLOAT, "Hz", {"20000000", "40000000"}),
    };
}

} // namespace

void MipiDevice::parseFrontendStatus(const std::string &text, bool isRx, FrontendStatus &st)
{
    st.valid = true;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string key = line.substr(0, colon);
        std::string val = line.substr(colon + 1);
        while (!key.empty() && (key.front() == ' ' || key.front() == '-')) key.erase(key.begin());
        while (!val.empty() && val.front() == ' ') val.erase(val.begin());

        if (key == "PLL Lock") {
            st.pllLocked = val.find("LOCKED") != std::string::npos && val.find("UNLOCKED") == std::string::npos;
        } else if (key == "LO Frequency") {
            st.freqHz = std::strtod(val.c_str(), nullptr) * 1e6;
        } else if (key == "Gain") {
            st.gainDb = std::strtod(val.c_str(), nullptr);
        } else if (key == "Analog Bandwidth") {
            st.analogBwHz = std::strtod(val.c_str(), nullptr) * 1e6;
        } else if (key.rfind("Digital Bandwidth", 0) == 0) {
            if (val.find("Disabled") == std::string::npos) {
                const auto act = val.find("actual ");
                if (act != std::string::npos) {
                    st.digitalBwHz = std::strtod(val.c_str() + act + 7, nullptr) * 1e6;
                } else {
                    const double k = std::strtod(val.c_str(), nullptr);
                    if (k > 0.0) st.digitalBwHz = 240.0e6 / k;
                }
            }
        } else if (key == "AGC") {
            st.agc = val.find("Enabled") != std::string::npos;
            const auto sp = val.find("Setpoint:");
            if (sp != std::string::npos) {
                const double thr = std::strtod(val.c_str() + sp + 9, nullptr);
                st.agcSetpointDbfs = agcThrToDbfs(thr);
            }
        } else if (key == "Polarization") {
            st.pol = (val.find("RHCP") != std::string::npos) ? "rhcp" : "lhcp";
        } else if (key == "Interleaved Mode") {
            st.interleave = val.find("ON") != std::string::npos;
        } else if (key == "Auto Steer") {
            st.autosteer = val.find("ON") != std::string::npos;
        } else if (key == "Test Tone") {
            st.toneEn = val.find("ON") != std::string::npos;
        } else if (key == "Tone Freq") {
            st.toneFreqHz = std::strtod(val.c_str(), nullptr) * 1e6;
        } else if (key == "Phases") {
            std::istringstream ps(val);
            for (int i = 0; i < 4; ++i) {
                std::string tok;
                if (!std::getline(ps, tok, ',')) break;
                st.phases[i] = std::strtod(tok.c_str(), nullptr);
            }
        } else if (key == "Antennas enabled") {
            st.antennas = 0;
            if (val.find("None") == std::string::npos) {
                if (val.find('1') != std::string::npos) st.antennas |= 1u;
                if (val.find('2') != std::string::npos) st.antennas |= 2u;
                if (val.find('3') != std::string::npos) st.antennas |= 4u;
                if (val.find('4') != std::string::npos) st.antennas |= 8u;
            }
        } else if (key == "Tx is") {
            st.txOn = val.find("ON") != std::string::npos;
        } else if (key == "Tx follow Rx") {
            st.txFollowRx = val.find("ON") != std::string::npos;
        }
        (void)isRx;
    }
}

std::string MipiDevice::jtagRun(const std::vector<std::string> &args) const
{
    std::string lastErr;
    for (int attempt = 0; attempt < 5; ++attempt) {
        int pipefd[2];
        if (::pipe(pipefd) != 0) {
            throw std::runtime_error(std::string("pipe failed: ") + std::strerror(errno));
        }

        const pid_t pid = ::fork();
        if (pid < 0) {
            ::close(pipefd[0]);
            ::close(pipefd[1]);
            throw std::runtime_error(std::string("fork failed: ") + std::strerror(errno));
        }

        if (pid == 0) {
            ::dup2(pipefd[1], STDOUT_FILENO);
            ::dup2(pipefd[1], STDERR_FILENO);
            ::close(pipefd[0]);
            ::close(pipefd[1]);
            std::vector<const char *> argv;
            argv.push_back(jtagPath_.c_str());
            for (const auto &a : args) argv.push_back(a.c_str());
            argv.push_back(nullptr);
            ::execv(jtagPath_.c_str(), const_cast<char * const *>(argv.data()));
            ::_exit(127);
        }

        ::close(pipefd[1]);
        std::string out;
        char buf[512];
        ssize_t n = 0;
        while ((n = ::read(pipefd[0], buf, sizeof(buf))) > 0) out.append(buf, size_t(n));
        ::close(pipefd[0]);

        int status = 0;
        while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {}

        if (WIFEXITED(status) && WEXITSTATUS(status) == 0) return out;

        lastErr = out.empty() ? ("quadrf-jtag exit " + std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -1)) : out;
        if (attempt < 4) ::usleep(50000 * (attempt + 1));
    }
    throw std::runtime_error(lastErr);
}

void MipiDevice::jtagApply(int dir, const std::string &spec)
{
    std::lock_guard<std::mutex> lock(jtagMutex_);
    const char *flag = (dir == SOAPY_SDR_TX) ? "--tx" : "--rx";
    SoapySDR::logf(SOAPY_SDR_INFO, "MipiDevice jtag %s %s", flag, spec.c_str());
    jtagRun({flag, spec});
    if (dir == SOAPY_SDR_TX) txHwNs_ = 0;
    else rxHwNs_ = 0;
}

MipiDevice::FrontendStatus &MipiDevice::hw(int dir) const
{
    return (dir == SOAPY_SDR_TX) ? txHw_ : rxHw_;
}

void MipiDevice::refreshFrontend(int dir, bool force) const
{
    std::lock_guard<std::mutex> lock(jtagMutex_);
    uint64_t &stamp = (dir == SOAPY_SDR_TX) ? txHwNs_ : rxHwNs_;
    FrontendStatus &st = hw(dir);
    const uint64_t now = monotonicNs();
    if (!force && st.valid && stamp != 0 && (now - stamp) < kStatusTtlNs) return;

    const char *which = (dir == SOAPY_SDR_TX) ? "tx" : "rx";
    const std::string out = jtagRun({"--status", which});
    parseFrontendStatus(out, dir != SOAPY_SDR_TX, st);
    if (dir == SOAPY_SDR_TX && lastTxBwHz_ > 0.0) st.analogBwHz = lastTxBwHz_;
    stamp = monotonicNs();
}

size_t MipiDevice::getNumChannels(const int dir) const
{
    if (dir == SOAPY_SDR_RX) return kRxHwChannels;
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
        const double native = rxNativeRate_();
        if (lastRxRate_ <= 0.0 || lastRxRate_ >= native) return native;
        return lastRxRate_;
    }
    return 0.0;
}

SoapySDR::RangeList MipiDevice::getSampleRateRange(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_TX) {
        return { SoapySDR::Range(1.0e6, 90.0e6) };
    }
    return { SoapySDR::Range(1.0e6, 85.0e6) };
}

SoapySDR::RangeList MipiDevice::getFrequencyRange(const int, const size_t) const
{
    return { SoapySDR::Range(kLoMinHz, kLoMaxHz) };
}

SoapySDR::Range MipiDevice::getGainRange(const int, const size_t) const
{
    return SoapySDR::Range(0.0, 63.0, 1.0);
}

SoapySDR::ArgInfoList MipiDevice::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList out;
    out.push_back(makeArg("bypass_iir", "true", "Bypass IIR", "RX IIR is removed in this build", SoapySDR::ArgInfo::BOOL));
    for (auto a : rxSettingInfo()) { out.push_back(a); a.key = "rx_" + a.key; out.push_back(std::move(a)); }
    for (auto a : txSettingInfo()) {
        if (a.key == "follow_rx") a.key = "tx_follow_rx";
        else a.key = "tx_" + a.key;
        out.push_back(std::move(a));
    }
    return out;
}

void MipiDevice::writeSetting(const std::string &key, const std::string &value)
{
    if (key == "bypass_iir") {
        SoapySDR::log(SOAPY_SDR_INFO, "RX IIR Filter is permanently bypassed/removed in this build.");
        return;
    }
    if (key.rfind("rx_", 0) == 0) {
        writeChannelSetting(SOAPY_SDR_RX, key.substr(3), value);
        return;
    }
    if (key == "tx_follow_rx" || key == "follow_rx") {
        writeChannelSetting(SOAPY_SDR_TX, "follow_rx", value);
        return;
    }
    if (key.rfind("tx_", 0) == 0) {
        writeChannelSetting(SOAPY_SDR_TX, key.substr(3), value);
        return;
    }
    if (key == "pol" || key == "interleave" || key == "autosteer" || key == "agc_setpoint" ||
        key == "antennas" || key == "tone_en" || key == "tone_freq" ||
        key == "p1" || key == "p2" || key == "p3" || key == "p4") {
        writeChannelSetting(SOAPY_SDR_RX, key, value);
        return;
    }
    throw std::runtime_error("unknown setting: " + key);
}

std::string MipiDevice::readSetting(const std::string &key) const
{
    if (key == "rx_dev") return rxPath_;
    if (key == "tx_dev") return txPath_;
    if (key == "bypass_iir") return "true";
    if (key == "jtag") return jtagPath_;
    if (key.rfind("rx_", 0) == 0) return readChannelSetting(SOAPY_SDR_RX, key.substr(3));
    if (key == "tx_follow_rx" || key == "follow_rx") return readChannelSetting(SOAPY_SDR_TX, "follow_rx");
    if (key.rfind("tx_", 0) == 0) return readChannelSetting(SOAPY_SDR_TX, key.substr(3));
    if (key == "pol" || key == "interleave" || key == "autosteer" || key == "agc_setpoint" ||
        key == "antennas" || key == "tone_en" || key == "tone_freq" ||
        key == "p1" || key == "p2" || key == "p3" || key == "p4") {
        return readChannelSetting(SOAPY_SDR_RX, key);
    }
    return "";
}

SoapySDR::ArgInfoList MipiDevice::getSettingInfo(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_TX) return txSettingInfo();
    return rxSettingInfo();
}

void MipiDevice::writeSetting(const int dir, const size_t, const std::string &key, const std::string &value)
{
    writeChannelSetting(dir, key, value);
}

std::string MipiDevice::readSetting(const int dir, const size_t, const std::string &key) const
{
    return readChannelSetting(dir, key);
}

void MipiDevice::writeChannelSetting(int dir, const std::string &key, const std::string &value)
{
    if (dir == SOAPY_SDR_RX) {
        if (key == "pol") {
            const std::string v = toLower(value);
            if (v != "rhcp" && v != "lhcp") throw std::runtime_error("pol must be rhcp or lhcp");
            jtagApply(SOAPY_SDR_RX, "pol=" + v);
            return;
        }
        if (key == "interleave") {
            jtagApply(SOAPY_SDR_RX, std::string("interleave=") + (parseBool(value) ? "1" : "0"));
            return;
        }
        if (key == "antennas") {
            const long mask = parseLong(value, "antennas");
            if (mask < 0 || mask > 15) throw std::runtime_error("antennas mask must be 0..15");
            jtagApply(SOAPY_SDR_RX, "antennas=" + std::to_string(mask));
            return;
        }
        if (key == "autosteer") {
            jtagApply(SOAPY_SDR_RX, std::string("autosteer=") + (parseBool(value) ? "1" : "0"));
            return;
        }
        if (key == "tone_en") {
            jtagApply(SOAPY_SDR_RX, std::string("tone_en=") + (parseBool(value) ? "1" : "0"));
            return;
        }
        if (key == "tone_freq") {
            jtagApply(SOAPY_SDR_RX, "tone_freq=" + fmtDouble(parseDouble(value, "tone_freq") / 1e6, 6));
            return;
        }
        if (key == "p1" || key == "p2" || key == "p3" || key == "p4") {
            jtagApply(SOAPY_SDR_RX, key + "=" + fmtDouble(parseDouble(value, "phase"), 1));
            return;
        }
        if (key == "agc_setpoint") {
            agcTargetDbfs_ = parseDouble(value, "agc_setpoint");
            refreshFrontend(SOAPY_SDR_RX, false);
            if (rxHw_.agc) jtagApply(SOAPY_SDR_RX, "agc=" + fmtDouble(agcTargetDbfs_, 2));
            return;
        }
        if (key == "analog_bw") {
            throw std::runtime_error("RX analog_bw is read-only; use setBandwidth()");
        }
        throw std::runtime_error("unknown RX setting: " + key);
    }

    if (dir == SOAPY_SDR_TX) {
        if (key == "enable") {
            if (parseBool(value)) {
                refreshFrontend(SOAPY_SDR_TX, false);
                unsigned mask = txHw_.antennas ? txHw_.antennas : 15u;
                jtagApply(SOAPY_SDR_TX, "antennas=" + std::to_string(mask));
            } else {
                jtagApply(SOAPY_SDR_TX, "off");
            }
            return;
        }
        if (key == "follow_rx") {
            jtagApply(SOAPY_SDR_TX, std::string("tx_follow_rx=") + (parseBool(value) ? "1" : "0"));
            return;
        }
        if (key == "antennas") {
            const long mask = parseLong(value, "antennas");
            if (mask < 0 || mask > 15) throw std::runtime_error("antennas mask must be 0..15");
            jtagApply(SOAPY_SDR_TX, "antennas=" + std::to_string(mask));
            return;
        }
        if (key == "tone_en") {
            jtagApply(SOAPY_SDR_TX, std::string("tone_en=") + (parseBool(value) ? "1" : "0"));
            return;
        }
        if (key == "tone_freq") {
            jtagApply(SOAPY_SDR_TX, "tone_freq=" + fmtDouble(parseDouble(value, "tone_freq") / 1e6, 6));
            return;
        }
        if (key == "p1" || key == "p2" || key == "p3" || key == "p4") {
            jtagApply(SOAPY_SDR_TX, key + "=" + fmtDouble(parseDouble(value, "phase"), 1));
            return;
        }
        if (key == "analog_bw") {
            setBandwidth(SOAPY_SDR_TX, 0, parseDouble(value, "analog_bw"));
            return;
        }
        throw std::runtime_error("unknown TX setting: " + key);
    }

    throw std::runtime_error("unsupported setting direction");
}

std::string MipiDevice::readChannelSetting(int dir, const std::string &key) const
{
    refreshFrontend(dir, false);
    const FrontendStatus &st = hw(dir);

    auto b = [](bool v) { return v ? std::string("true") : std::string("false"); };

    if (dir == SOAPY_SDR_RX) {
        if (key == "pol") return st.pol;
        if (key == "interleave") return b(st.interleave);
        if (key == "antennas") return std::to_string(st.antennas);
        if (key == "autosteer") return b(st.autosteer);
        if (key == "tone_en") return b(st.toneEn);
        if (key == "tone_freq") return fmtDouble(st.toneFreqHz, 3);
        if (key == "p1") return fmtDouble(st.phases[0], 1);
        if (key == "p2") return fmtDouble(st.phases[1], 1);
        if (key == "p3") return fmtDouble(st.phases[2], 1);
        if (key == "p4") return fmtDouble(st.phases[3], 1);
        if (key == "agc_setpoint") return fmtDouble(st.agc ? st.agcSetpointDbfs : agcTargetDbfs_, 2);
        if (key == "analog_bw") return fmtDouble(st.analogBwHz, 3);
        return "";
    }

    if (key == "enable") return b(st.txOn);
    if (key == "follow_rx") return b(st.txFollowRx);
    if (key == "antennas") return std::to_string(st.antennas);
    if (key == "tone_en") return b(st.toneEn);
    if (key == "tone_freq") return fmtDouble(st.toneFreqHz, 3);
    if (key == "p1") return fmtDouble(st.phases[0], 1);
    if (key == "p2") return fmtDouble(st.phases[1], 1);
    if (key == "p3") return fmtDouble(st.phases[2], 1);
    if (key == "p4") return fmtDouble(st.phases[3], 1);
    if (key == "analog_bw") return fmtDouble(st.analogBwHz, 3);
    return "";
}

SoapySDR::Stream *MipiDevice::setupStream(const int dir, const std::string &format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args)
{
    (void)args;

    std::lock_guard<std::recursive_mutex> dlock(deviceMutex_);

    auto normalizedFormat = format.empty() ? std::string(SOAPY_SDR_CS8) : format;
    if (normalizedFormat == "fc32" || normalizedFormat == "FC32" || normalizedFormat == "Complex Float32") {
        normalizedFormat = SOAPY_SDR_CF32;
    } else if (normalizedFormat == "sc8" || normalizedFormat == "SC8" || normalizedFormat == "Complex Byte" || normalizedFormat == "CS8") {
        normalizedFormat = SOAPY_SDR_CS8;
    }

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

            std::vector<size_t> chans = channels;
            if (chans.empty()) chans = {0};

            bool seen[kRxHwChannels] = {};
            for (size_t ch : chans) {
                if (ch >= kRxHwChannels) {
                    throw std::runtime_error("RX channel index out of range (0-3): " + std::to_string(ch));
                }
                if (seen[ch]) {
                    throw std::runtime_error("duplicate RX channel: " + std::to_string(ch));
                }
                seen[ch] = true;
            }

            initRx();
            rxRequestedFormat_ = normalizedFormat;
            rxChannels_ = chans;
            s->channels = chans;

            applyRxInterleave_(rxChannels_.size() > 1);
            if (rxChannels_.size() > 1) {
                writeChannelSetting(SOAPY_SDR_RX, "antennas", "15");
            }
            rxInitRxBuffers_();
            rxFilter_config_(lastRxRate_);

            rxStreamOpen_ = true;
            SoapySDR::logf(SOAPY_SDR_INFO,
                           "MipiDevice::setupStream RX stream=%llu format=%s channels=%zu interleaved=%s native=%.3f Msps",
                           (unsigned long long)s->id, s->format.c_str(), s->channels.size(),
                           rxInterleaveEnabled_ ? "yes" : "no", rxNativeRate_() / 1e6);
            return reinterpret_cast<SoapySDR::Stream*>(holder.release());
        }

        if (dir == SOAPY_SDR_TX) {
            std::lock_guard<std::recursive_mutex> tlock(txMutex_);
            if (txStreamOpen_) {
                throw std::runtime_error("TX stream is already open; close the existing TX stream before setupStream(TX)");
            }

            std::vector<size_t> chans = channels;
            if (chans.empty()) chans = {0};
            if (chans.size() != 1 || chans[0] != 0) {
                throw std::runtime_error("TX supports a single channel (0)");
            }
            s->channels = chans;

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
        rxInterleaveRemainder_.clear();
        for (size_t i = 0; i < kRxHwChannels; ++i) {
            rxChResampler_[i].reset();
            rxChFloatBuf_[i].reset();
        }
        if (rxInterleaveEnabled_) {
            try { applyRxInterleave_(false); }
            catch (const std::exception &e) {
                SoapySDR::logf(SOAPY_SDR_WARNING, "closeStream RX: could not disable interleave: %s", e.what());
            }
        }
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
        rxInterleaveRemainder_.clear();
        for (size_t i = 0; i < kRxHwChannels; ++i) {
            rxChResampler_[i].reset();
            rxChFloatBuf_[i].reset();
        }
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

void MipiDevice::setAntenna(const int, const size_t, const std::string &)
{
    // No RF mux. Apps pass Auto, TX/RX, or an empty field; ignore them.
}

std::string MipiDevice::getAntenna(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_TX) return "TX";
    return "RX";
}

std::vector<std::string> MipiDevice::listGains(const int, const size_t) const { return {"RF"}; }

SoapySDR::Range MipiDevice::getGainRange(const int, const size_t, const std::string &) const
{
    return SoapySDR::Range(0.0, 63.0, 1.0);
}

void MipiDevice::setGain(const int dir, const size_t, const std::string &, const double value)
{
    double g = clampVal(value, 0.0, 63.0);
    if (g != value) {
        SoapySDR::logf(SOAPY_SDR_WARNING, "setGain %.3f dB snapped to %.0f dB", value, g);
    }
    if (dir == SOAPY_SDR_RX) lastRxManualGain_ = g;
    jtagApply(dir, "gain=" + std::to_string(int(std::lround(g))));
}

double MipiDevice::getGain(const int dir, const size_t, const std::string &) const
{
    refreshFrontend(dir, false);
    return hw(dir).gainDb;
}

bool MipiDevice::hasGainMode(const int dir, const size_t) const
{
    return dir == SOAPY_SDR_RX;
}

void MipiDevice::setGainMode(const int dir, const size_t, const bool automatic)
{
    if (dir != SOAPY_SDR_RX) {
        if (automatic) {
            SoapySDR::log(SOAPY_SDR_WARNING, "setGainMode: TX AGC is not supported, ignored");
        }
        return;
    }
    refreshFrontend(SOAPY_SDR_RX, false);
    if (automatic) {
        if (rxHw_.agc) return;
        lastRxManualGain_ = rxHw_.gainDb;
        jtagApply(SOAPY_SDR_RX, "agc=" + fmtDouble(agcTargetDbfs_, 2));
        return;
    }
    if (!rxHw_.agc) return;
    const int g = int(std::lround(std::max(0.0, std::min(63.0, lastRxManualGain_))));
    jtagApply(SOAPY_SDR_RX, "gain=" + std::to_string(g));
}

bool MipiDevice::getGainMode(const int dir, const size_t) const
{
    if (dir != SOAPY_SDR_RX) return false;
    refreshFrontend(SOAPY_SDR_RX, false);
    return rxHw_.agc;
}

std::vector<std::string> MipiDevice::listFrequencies(const int, const size_t) const { return {"RF"}; }

void MipiDevice::setFrequency(const int dir, const size_t, const std::string &name, const double freq, const SoapySDR::Kwargs &)
{
    if (!isRfComponent(name)) {
        SoapySDR::logf(SOAPY_SDR_WARNING, "setFrequency ignoring component '%s'", name.c_str());
        return;
    }
    const double hz = clampRfHz(freq);
    if (hz != freq) {
        SoapySDR::logf(SOAPY_SDR_WARNING, "setFrequency %.6f Hz clamped to %.0f Hz", freq, hz);
    }
    jtagApply(dir, "freq=" + fmtDouble(hz / 1e6, 6));
}

double MipiDevice::getFrequency(const int dir, const size_t, const std::string &name) const
{
    if (!isRfComponent(name)) return 0.0;
    refreshFrontend(dir, false);
    return hw(dir).freqHz;
}

SoapySDR::RangeList MipiDevice::getFrequencyRange(const int, const size_t, const std::string &name) const
{
    if (!isRfComponent(name)) return {};
    return { SoapySDR::Range(kLoMinHz, kLoMaxHz) };
}

void MipiDevice::setBandwidth(const int dir, const size_t, const double bw)
{
    // GRC Soapy blocks default bandwidth to 0 meaning "do not retune the analog/digital filter".
    if (bw <= 0.0) return;

    if (dir == SOAPY_SDR_TX) {
        const double mhz = (std::fabs(bw - 20e6) < std::fabs(bw - 40e6)) ? 20.0 : 40.0;
        lastTxBwHz_ = mhz * 1e6;
        jtagApply(SOAPY_SDR_TX, "bw=" + fmtDouble(mhz, 0));
        return;
    }
    jtagApply(SOAPY_SDR_RX, "bw=" + fmtDouble(bw / 1e6, 6));
}

double MipiDevice::getBandwidth(const int dir, const size_t) const
{
    refreshFrontend(dir, false);
    if (dir == SOAPY_SDR_TX) return hw(dir).analogBwHz;
    return hw(dir).digitalBwHz;
}

std::vector<double> MipiDevice::listBandwidths(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_TX) return {20e6, 40e6};
    return {kRxBwMinHz, 10e6, 20e6, 40e6, kRxBwMaxHz};
}

SoapySDR::RangeList MipiDevice::getBandwidthRange(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_TX) return { SoapySDR::Range(20e6, 40e6) };
    return { SoapySDR::Range(kRxBwMinHz, kRxBwMaxHz) };
}

std::vector<double> MipiDevice::listSampleRates(const int dir, const size_t) const
{
    if (dir == SOAPY_SDR_RX) return { 5e6, 10e6, 20e6, 20.101e6, 30.72e6, 40e6, 80e6, 80.405e6 };
    if (dir == SOAPY_SDR_TX) return { 5e6, 10e6, 20e6, 40e6, 86.08e6 };
    return {};
}

void MipiDevice::setSampleRate(const int dir, const size_t, const double rate)
{
    double r = rate;
    if (r > 0.0 && (r < kRateMinHz || r > kRateMaxHz)) {
        r = clampVal(r, kRateMinHz, kRateMaxHz);
        SoapySDR::logf(SOAPY_SDR_WARNING, "setSampleRate %.3f Hz snapped to %.0f Hz", rate, r);
    }

    if (dir == SOAPY_SDR_RX) {
        std::lock_guard<std::recursive_mutex> lock(rxMutex_);
        lastRxRate_ = r;
        rxInitRxBuffers_();
        rxFilter_config_(r);
        SoapySDR::logf(SOAPY_SDR_INFO,
                       "MipiDevice::setSampleRate RX %.6f Msps native=%.6f Msps ratio=%.9f",
                       r / 1e6, rxNativeRate_() / 1e6, sampleRateRatio_);
        return;
    }

    if (dir == SOAPY_SDR_TX) {
        std::lock_guard<std::recursive_mutex> lock(txMutex_);
        lastTxRate_ = r;
        txResampler_config_(r);
        SoapySDR::logf(SOAPY_SDR_INFO,
                       "MipiDevice::setSampleRate TX %.6f Msps ratio=%.9f",
                       r / 1e6, txSampleRateRatio_);
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
    if (buffs == nullptr) return SOAPY_SDR_STREAM_ERROR;
    if (numElems == 0) return 0;

    if (rxChannels_.size() > 1) {
        return readStreamInterleaved_(stream, buffs, numElems, timeoutUs);
    }

    if (buffs[0] == nullptr) return SOAPY_SDR_STREAM_ERROR;

    const bool wantCF32 = (rxRequestedFormat_ == SOAPY_SDR_CF32); 

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
    k["jtag"] = jtagPath_;
    k["frontend"] = "MAX2850/MAX2851";
    return k;
}

size_t MipiDevice::getStreamMTU(SoapySDR::Stream *stream) const
{
    const auto *s = reinterpret_cast<const MipiStream*>(stream);

    if (s && s->dir == SOAPY_SDR_RX) {
        const size_t rxFrameSize = (s->channels.size() > 1) ? kRxInterleaveFrameBytes : 2;
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
    size_t rxFrameSize = (rxChannels_.size() > 1) ? kRxInterleaveFrameBytes : 2;
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

double MipiDevice::rxNativeRate_() const
{
    return (rxChannels_.size() > 1) ? (kFsIn / double(kRxHwChannels)) : kFsIn;
}

void MipiDevice::rxInitRxBuffers_()
{
    const double native = rxNativeRate_();
    const double r = lastRxRate_;
    const double ratio = (r > 0.0 && r < native) ? (native / r) : 1.0;
    const double blockPairs = 32768.0 * ratio + 1024.0;
    size_t rxMaxPairs = size_t(std::max(4096.0, blockPairs * 8.0));
    if (rxChannels_.size() > 1) {
        for (size_t i = 0; i < kRxHwChannels; ++i)
            rxChFloatBuf_[i].init(rxMaxPairs * 2);
    } else {
        rxFloatBuf_.init(rxMaxPairs * 2);
    }
}

void MipiDevice::applyRxInterleave_(bool enable)
{
    if (enable) {
        writeChannelSetting(SOAPY_SDR_RX, "interleave", "true");
        rxInterleaveEnabled_ = true;
        return;
    }
    try {
        writeChannelSetting(SOAPY_SDR_RX, "interleave", "false");
        rxInterleaveEnabled_ = false;
    } catch (const std::exception &e) {
        if (rxInterleaveEnabled_) throw;
        SoapySDR::logf(SOAPY_SDR_WARNING, "applyRxInterleave_: could not disable: %s", e.what());
    }
}

void MipiDevice::mapRxHwBuffs_(void * const *buffs, size_t produced, size_t bytesPerElem,
                              void *hwBuffs[kRxHwChannels]) const
{
    for (size_t i = 0; i < kRxHwChannels; ++i) hwBuffs[i] = nullptr;
    for (size_t i = 0; i < rxChannels_.size(); ++i) {
        const size_t ch = rxChannels_[i];
        if (ch >= kRxHwChannels || buffs[i] == nullptr) continue;
        hwBuffs[ch] = static_cast<uint8_t *>(buffs[i]) + produced * bytesPerElem;
    }
}

ssize_t MipiDevice::rxGatherInterleaved_(size_t wantFrames, long timeoutUs)
{
    if (wantFrames == 0) return 0;
    const size_t wantBytes = wantFrames * kRxInterleaveFrameBytes;
    const size_t have = rxInterleaveRemainder_.size();
    if (rxScratch_.size() < wantBytes) rxScratch_.resize(wantBytes);
    if (have) {
        if (have > wantBytes) {
            std::memcpy(rxScratch_.data(), rxInterleaveRemainder_.data(), wantBytes);
            rxInterleaveRemainder_.erase(rxInterleaveRemainder_.begin(),
                                         rxInterleaveRemainder_.begin() + ssize_t(wantBytes));
            return ssize_t(wantFrames);
        }
        std::memcpy(rxScratch_.data(), rxInterleaveRemainder_.data(), have);
    }

    const size_t need = wantBytes - have;
    const ssize_t gotBytes = (rxRing_ ? rx_read_ring(rxScratch_.data() + have, need, timeoutUs)
                                      : rx_read_legacy(rxScratch_.data() + have, need, timeoutUs));
    if (gotBytes < 0) {
        if (have >= kRxInterleaveFrameBytes) {
            const size_t frames = have / kRxInterleaveFrameBytes;
            const size_t used = frames * kRxInterleaveFrameBytes;
            rxInterleaveRemainder_.assign(rxScratch_.data() + used, rxScratch_.data() + have);
            return ssize_t(frames);
        }
        return gotBytes;
    }

    const size_t total = have + size_t(gotBytes);
    const size_t frames = total / kRxInterleaveFrameBytes;
    const size_t used = frames * kRxInterleaveFrameBytes;
    rxInterleaveRemainder_.assign(rxScratch_.data() + used, rxScratch_.data() + total);
    return ssize_t(frames);
}

int MipiDevice::readStreamInterleaved_(SoapySDR::Stream *stream, void * const *buffs,
                                       size_t numElems, long timeoutUs)
{
    for (size_t i = 0; i < rxChannels_.size(); ++i) {
        if (buffs[i] == nullptr) return SOAPY_SDR_STREAM_ERROR;
    }

    const bool wantCF32 = (rxRequestedFormat_ == SOAPY_SDR_CF32);
    const size_t outBytesPerElem = wantCF32 ? (2 * sizeof(float)) : 2;
    const bool resample = rxChResampler_[0].isEnabled();

    if (resample) {
        for (size_t ch : rxChannels_) {
            if (rxChFloatBuf_[ch].capacity() == 0)
                rxChFloatBuf_[ch].init(32768 * 2);
        }
    }

    size_t totalProduced = 0;
    long loopTimeoutUs = timeoutUs;

    while (totalProduced < numElems) {
        const size_t remainingOutput = numElems - totalProduced;

        size_t inputNeeded = (size_t)ceil(remainingOutput * sampleRateRatio_) + (resample ? 16 : 0);
        if (resample) inputNeeded = (inputNeeded + 7) & ~7;
        if (inputNeeded > remainingOutput && !resample) inputNeeded = remainingOutput;

        size_t mtu = this->getStreamMTU(stream);
        if (inputNeeded > mtu) inputNeeded = mtu;
        if (inputNeeded == 0) inputNeeded = resample ? 8 : remainingOutput;

        const ssize_t gotFrames = rxGatherInterleaved_(inputNeeded, loopTimeoutUs);
        if (gotFrames == -EAGAIN) {
            if (totalProduced > 0) break;
            return SOAPY_SDR_TIMEOUT;
        }
        if (gotFrames < 0) {
            if (totalProduced > 0) break;
            return SOAPY_SDR_STREAM_ERROR;
        }
        if (gotFrames == 0) {
            if (totalProduced > 0) break;
            return SOAPY_SDR_TIMEOUT;
        }

        const size_t frames = size_t(gotFrames);
        const int8_t *src = reinterpret_cast<const int8_t *>(rxScratch_.data());

        if (!resample) {
            const size_t use = std::min(frames, remainingOutput);
            void *hwBuffs[kRxHwChannels];
            mapRxHwBuffs_(buffs, totalProduced, outBytesPerElem, hwBuffs);
            if (wantCF32) deinterleave_CS8_to_CF32_NEON(src, hwBuffs, use);
            else          deinterleave_CS8_NEON(src, hwBuffs, use);
            if (use < frames) {
                const uint8_t *extra = rxScratch_.data() + use * kRxInterleaveFrameBytes;
                rxInterleaveRemainder_.insert(rxInterleaveRemainder_.begin(), extra,
                                              extra + (frames - use) * kRxInterleaveFrameBytes);
            }
            totalProduced += use;
            if (totalProduced >= numElems) break;
            loopTimeoutUs = 2000;
            continue;
        }

        void *hwBuffs[kRxHwChannels];
        for (size_t i = 0; i < kRxHwChannels; ++i) hwBuffs[i] = nullptr;
        for (size_t ch : rxChannels_) {
            hwBuffs[ch] = rxChFloatBuf_[ch].prepareWrite(frames * 2);
        }
        deinterleave_CS8_to_CF32_NEON(src, hwBuffs, frames);
        for (size_t ch : rxChannels_) {
            rxChFloatBuf_[ch].commitWrite(frames * 2);
        }

        int producedCommon = -1;
        for (size_t i = 0; i < rxChannels_.size(); ++i) {
            const size_t ch = rxChannels_[i];
            float *dst = nullptr;
            if (wantCF32) {
                dst = static_cast<float *>(buffs[i]) + (totalProduced * 2);
            } else {
                if (resampScratch_.size() < remainingOutput * 2)
                    resampScratch_.resize(remainingOutput * 2);
                dst = resampScratch_.data();
            }

            int inConsumedPairs = 0;
            int produced = rxChResampler_[ch].process(
                rxChFloatBuf_[ch].readPtr(), int(rxChFloatBuf_[ch].readAvail() / 2),
                dst, int(remainingOutput),
                sampleRateRatio_, inConsumedPairs);

            rxChFloatBuf_[ch].consume(size_t(inConsumedPairs) * 2);

            if (!wantCF32 && produced > 0) {
                convert_CF32_to_CS8_NEON(
                    dst,
                    static_cast<int8_t *>(buffs[i]) + (totalProduced * 2),
                    size_t(produced));
            }

            if (producedCommon < 0) producedCommon = produced;
            else if (produced != producedCommon) {
                producedCommon = std::min(producedCommon, produced);
            }
        }

        if (producedCommon > 0) totalProduced += size_t(producedCommon);
        else if (frames == 0) break;

        loopTimeoutUs = 2000;
    }

    return int(totalProduced);
}

void MipiDevice::rxFilter_config_(double fsOut)
{
    const double fsIn = rxNativeRate_();
    const bool enable = !(std::abs(fsOut - fsIn) < 1.0e3 || fsOut <= 0.0 || fsOut >= fsIn);
    sampleRateRatio_ = enable ? (fsIn / fsOut) : 1.0;

    resampler_.setEnabled(enable);
    resampler_.reset();
    rxFloatBuf_.reset();
    for (size_t i = 0; i < kRxHwChannels; ++i) {
        rxChResampler_[i].setEnabled(enable);
        rxChResampler_[i].reset();
        rxChFloatBuf_[i].reset();
    }

    if (!enable) {
        SoapySDR::logf(SOAPY_SDR_INFO, "RX DSP bypassed (native %.3f Msps, requested %.3f Msps, channels=%zu)",
                       fsIn / 1e6, fsOut / 1e6, rxChannels_.size() ? rxChannels_.size() : 1);
        return;
    }

    SoapySDR::logf(SOAPY_SDR_INFO, "Configuring RX Farrow Resampler: In=%.2f MHz, Out=%.2f MHz, channels=%zu",
                   fsIn / 1e6, fsOut / 1e6, rxChannels_.size() ? rxChannels_.size() : 1);
}