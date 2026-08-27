#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>

#include <zmq.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// Optional frame header support (daemon may send header+payload multipart)
#pragma pack(push, 1)
struct FrameHeader
{
    uint32_t magic;
    uint16_t version;
    uint16_t channels;
    uint32_t sample_rate;
    uint32_t format;         // 1 = CF32 interleaved IQ
    uint32_t elems;
    uint64_t seq;
    uint64_t time_ns;
    uint32_t payload_bytes;
    uint32_t flags;
};
#pragma pack(pop)

static inline bool looks_like_header(const void* p, size_t n)
{
    if (n != sizeof(FrameHeader)) return false;
    const auto* h = static_cast<const FrameHeader*>(p);
    // matches daemon magic we discussed earlier; tolerate either if you changed it
    return (h->magic == 0x53515246u /*'FRQS'*/ || h->magic == 0x46525153u /*'SQR F' swapped*/ || h->magic == 0x53515246u) && (h->version == 1);
}

class ThinSoapyDevice : public SoapySDR::Device
{
private:
    void* zmq_ctx_ = nullptr;
    void* zmq_sub_ = nullptr;
    void* zmq_push_ = nullptr;
    void* zmq_rpc_ = nullptr;

    // Buffered receive state (payload only)
    zmq_msg_t rx_msg_;
    bool rx_msg_valid_ = false;
    size_t rx_msg_offset_ = 0; // complex samples already consumed

    // Cached Hardware State
    double gain_ = 0.0;
    std::string antennaSel_ = "RX";
    double sampleRate_ = 80.95076e6;
    bool rxIirBypass_ = true;

    // Endpoints (override via device args)
    std::string sub_ep_;
    std::string rpc_ep_;
    std::string push_ep_;

    // Settings
    size_t stream_mtu_elems_ = 65536; // default should match daemon chunking
    bool sub_conflate_ = false;       // optional: keep only latest message

    void rpc_send(const std::string& cmd)
    {
        if (!zmq_rpc_) return;
        zmq_send(zmq_rpc_, cmd.c_str(), (int)cmd.size(), 0);
        char reply[16];
        int rc = zmq_recv(zmq_rpc_, reply, (int)sizeof(reply) - 1, 0);
        if (rc > 0)
        {
            reply[rc] = '\0';
            if (std::strncmp(reply, "ERR", 3) == 0)
            {
                SoapySDR::logf(SOAPY_SDR_ERROR, "Daemon rejected command: '%s'", cmd.c_str());
            }
        }
        else
        {
            SoapySDR::log(SOAPY_SDR_ERROR, "RPC timeout.");
        }
    }

    static std::string kwarg_or(const SoapySDR::Kwargs& a, const std::string& k, const std::string& def)
    {
        auto it = a.find(k);
        return (it == a.end()) ? def : it->second;
    }

    static bool kwarg_bool(const SoapySDR::Kwargs& a, const std::string& k, bool def)
    {
        auto it = a.find(k);
        if (it == a.end()) return def;
        const std::string v = it->second;
        return (v == "1" || v == "true" || v == "TRUE" || v == "yes" || v == "on");
    }

    static size_t kwarg_sizet(const SoapySDR::Kwargs& a, const std::string& k, size_t def)
    {
        auto it = a.find(k);
        if (it == a.end()) return def;
        try { return (size_t)std::stoull(it->second); }
        catch (...) { return def; }
    }

public:
    ThinSoapyDevice(const SoapySDR::Kwargs& args)
    {
        // Defaults:
        // - prefer abstract IPC for local lowest overhead, no sudo/permissions
        // - keep TCP options available for remote
        sub_ep_  = kwarg_or(args, "sub", "ipc://@mipi_sdr_cf32");
        rpc_ep_  = kwarg_or(args, "rpc", "tcp://127.0.0.1:5557");
        push_ep_ = kwarg_or(args, "push", "tcp://127.0.0.1:5556");

        // Optional tuning
        stream_mtu_elems_ = kwarg_sizet(args, "mtu", stream_mtu_elems_);
        sub_conflate_     = kwarg_bool(args, "conflate", false);

        zmq_ctx_ = zmq_ctx_new();
        if (!zmq_ctx_) throw std::runtime_error("zmq_ctx_new failed");

        zmq_rpc_ = zmq_socket(zmq_ctx_, ZMQ_REQ);
        if (!zmq_rpc_) throw std::runtime_error("zmq_socket(ZMQ_REQ) failed");

        int timeout = 500; // ms
        zmq_setsockopt(zmq_rpc_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
        int linger = 0;
        zmq_setsockopt(zmq_rpc_, ZMQ_LINGER, &linger, sizeof(linger));

        if (zmq_connect(zmq_rpc_, rpc_ep_.c_str()) != 0)
        {
            SoapySDR::logf(SOAPY_SDR_WARNING, "Could not connect to Daemon RPC at %s", rpc_ep_.c_str());
            zmq_close(zmq_rpc_);
            zmq_rpc_ = nullptr;
        }
    }

    ~ThinSoapyDevice() override
    {
        if (rx_msg_valid_) zmq_msg_close(&rx_msg_);
        if (zmq_rpc_) zmq_close(zmq_rpc_);
        if (zmq_sub_) zmq_close(zmq_sub_);
        if (zmq_push_) zmq_close(zmq_push_);
        if (zmq_ctx_) zmq_ctx_destroy(zmq_ctx_);
    }

    // --- Identification ---
    std::string getDriverKey() const override { return "quadrf"; }
    std::string getHardwareKey() const override { return "rpi5-mipi-zmq"; }
    size_t getNumChannels(const int) const override { return 1; }

    SoapySDR::Kwargs getHardwareInfo() const override
    {
        SoapySDR::Kwargs k;
        k["label"] = "QuadRF (Thin Client)";
        k["sub"] = sub_ep_;
        return k;
    }

    // --- Formats ---
    std::vector<std::string> getStreamFormats(const int, const size_t) const override
    {
        return {SOAPY_SDR_CF32};
    }

    std::string getNativeStreamFormat(const int, const size_t, double& fullScale) const override
    {
        fullScale = 1.0;
        return SOAPY_SDR_CF32;
    }

    // --- Sample Rates ---
    std::vector<double> listSampleRates(const int, const size_t) const override
    {
        return {30.72e6, 40e6, 80.95076e6};
    }

    SoapySDR::RangeList getSampleRateRange(const int, const size_t) const override
    {
        return {SoapySDR::Range(1.0e6, 80.95076e6)};
    }

    void setSampleRate(const int, const size_t, const double rate) override
    {
        sampleRate_ = rate;
        rpc_send("RATE " + std::to_string(rate));
    }

    double getSampleRate(const int, const size_t) const override
    {
        return sampleRate_;
    }

    // --- Antennas ---
    std::vector<std::string> listAntennas(const int, const size_t) const override
    {
        return {"RX"};
    }

    void setAntenna(const int, const size_t, const std::string& name) override
    {
        antennaSel_ = name;
        rpc_send("ANTENNA " + name);
    }

    std::string getAntenna(const int, const size_t) const override
    {
        return antennaSel_;
    }

    // --- Gain ---
    std::vector<std::string> listGains(const int, const size_t) const override
    {
        return {"RF"};
    }

    SoapySDR::Range getGainRange(const int, const size_t, const std::string&) const override
    {
        return SoapySDR::Range(1.0, 63.0);
    }

    SoapySDR::Range getGainRange(const int, const size_t) const override
    {
        return SoapySDR::Range(1.0, 63.0);
    }

    void setGain(const int, const size_t, const std::string&, const double value) override
    {
        gain_ = value;
        rpc_send("GAIN " + std::to_string(value));
    }

    double getGain(const int, const size_t, const std::string&) const override
    {
        return gain_;
    }

    // --- Frequencies ---
    std::vector<std::string> listFrequencies(const int, const size_t) const override
    {
        return {"BB"};
    }

    SoapySDR::RangeList getFrequencyRange(const int, const size_t) const override
    {
        return {SoapySDR::Range(4.9e9, 6.0e9)};
    }

    SoapySDR::RangeList getFrequencyRange(const int, const size_t, const std::string&) const override
    {
        return {SoapySDR::Range(0.0, 0.0)};
    }

    void setFrequency(const int, const size_t, const std::string&, const double freq, const SoapySDR::Kwargs&) override
    {
        rpc_send("FREQ " + std::to_string(freq));
    }

    double getFrequency(const int, const size_t, const std::string&) const override
    {
        return 0.0;
    }

    // --- Settings ---
    void writeSetting(const std::string& key, const std::string& value) override
    {
        if (key == "bypass_iir")
        {
            rxIirBypass_ = (value == "true");
            rpc_send("SET bypass_iir " + value);
        }
    }

    std::string readSetting(const std::string& key) const override
    {
        if (key == "bypass_iir") return rxIirBypass_ ? "true" : "false";
        return "";
    }

    // --- Stream API ---
    size_t getStreamMTU(SoapySDR::Stream*) const override
    {
        return stream_mtu_elems_;
    }

    SoapySDR::Stream* setupStream(const int dir, const std::string& format,
                                  const std::vector<size_t>& channels,
                                  const SoapySDR::Kwargs& args) override
    {
        (void)args;
        if (format != SOAPY_SDR_CF32) throw std::runtime_error("Only CF32 supported.");

        if (channels.size() == 4) rpc_send("MODE 4CH");
        else rpc_send("MODE 2CH");

        if (dir == SOAPY_SDR_RX)
        {
            zmq_sub_ = zmq_socket(zmq_ctx_, ZMQ_SUB);
            if (!zmq_sub_) throw std::runtime_error("zmq_socket(ZMQ_SUB) failed");

            int linger = 0;
            zmq_setsockopt(zmq_sub_, ZMQ_LINGER, &linger, sizeof(linger));

            int rcvhwm = 2;
            zmq_setsockopt(zmq_sub_, ZMQ_RCVHWM, &rcvhwm, sizeof(rcvhwm));

#ifdef ZMQ_CONFLATE
            if (sub_conflate_) {
                int one = 1;
                zmq_setsockopt(zmq_sub_, ZMQ_CONFLATE, &one, sizeof(one));
            }
#endif

            // subscribe to everything
            zmq_setsockopt(zmq_sub_, ZMQ_SUBSCRIBE, "", 0);

            if (zmq_connect(zmq_sub_, sub_ep_.c_str()) != 0)
            {
                std::string e = std::string("zmq_connect SUB failed: ") + zmq_strerror(zmq_errno());
                zmq_close(zmq_sub_);
                zmq_sub_ = nullptr;
                throw std::runtime_error(e);
            }

            return reinterpret_cast<SoapySDR::Stream*>(this);
        }
        else if (dir == SOAPY_SDR_TX)
        {
            zmq_push_ = zmq_socket(zmq_ctx_, ZMQ_PUSH);
            if (!zmq_push_) throw std::runtime_error("zmq_socket(ZMQ_PUSH) failed");

            int linger = 0;
            zmq_setsockopt(zmq_push_, ZMQ_LINGER, &linger, sizeof(linger));

            if (zmq_connect(zmq_push_, push_ep_.c_str()) != 0)
            {
                std::string e = std::string("zmq_connect PUSH failed: ") + zmq_strerror(zmq_errno());
                zmq_close(zmq_push_);
                zmq_push_ = nullptr;
                throw std::runtime_error(e);
            }

            return reinterpret_cast<SoapySDR::Stream*>(this);
        }

        throw std::runtime_error("Invalid direction.");
    }

    void closeStream(SoapySDR::Stream*) override
    {
        if (rx_msg_valid_) {
            zmq_msg_close(&rx_msg_);
            rx_msg_valid_ = false;
            rx_msg_offset_ = 0;
        }
        if (zmq_sub_) {
            zmq_close(zmq_sub_);
            zmq_sub_ = nullptr;
        }
        if (zmq_push_) {
            zmq_close(zmq_push_);
            zmq_push_ = nullptr;
        }
    }

    int activateStream(SoapySDR::Stream*, const int, const long long, const size_t) override { return 0; }
    int deactivateStream(SoapySDR::Stream*, const int, const long long) override { return 0; }

    int readStream(SoapySDR::Stream*, void* const* buffs, const size_t numElems,
                   int& flags, long long& timeNs, const long timeoutUs) override
    {
        flags = 0;
        timeNs = 0;

        if (!zmq_sub_) return SOAPY_SDR_STREAM_ERROR;
        if (!buffs || !buffs[0]) return SOAPY_SDR_STREAM_ERROR;

        // If no buffered message, receive next payload (support optional header+payload multipart)
        if (!rx_msg_valid_)
        {
            zmq_msg_init(&rx_msg_);

            zmq_pollitem_t items[] = {{zmq_sub_, 0, ZMQ_POLLIN, 0}};
            int rc = zmq_poll(items, 1, (timeoutUs < 0) ? -1 : (int)(timeoutUs / 1000));
            if (rc <= 0 || !(items[0].revents & ZMQ_POLLIN))
            {
                zmq_msg_close(&rx_msg_);
                return SOAPY_SDR_TIMEOUT;
            }

            // Blocking recv (poll already indicated readiness)
            if (zmq_msg_recv(&rx_msg_, zmq_sub_, 0) == -1)
            {
                zmq_msg_close(&rx_msg_);
                return SOAPY_SDR_TIMEOUT;
            }

            int more = 0;
            size_t more_sz = sizeof(more);
            zmq_getsockopt(zmq_sub_, ZMQ_RCVMORE, &more, &more_sz);

            // If multipart header+payload, discard header part and keep payload in rx_msg_
            if (more && looks_like_header(zmq_msg_data(&rx_msg_), zmq_msg_size(&rx_msg_)))
            {
                zmq_msg_close(&rx_msg_);
                zmq_msg_init(&rx_msg_);
                if (zmq_msg_recv(&rx_msg_, zmq_sub_, 0) == -1)
                {
                    zmq_msg_close(&rx_msg_);
                    return SOAPY_SDR_TIMEOUT;
                }
            }

            rx_msg_valid_ = true;
            rx_msg_offset_ = 0;
        }

        const size_t msg_samples = zmq_msg_size(&rx_msg_) / (2 * sizeof(float));
        if (msg_samples == 0)
        {
            zmq_msg_close(&rx_msg_);
            rx_msg_valid_ = false;
            rx_msg_offset_ = 0;
            return SOAPY_SDR_STREAM_ERROR;
        }

        const size_t samples_avail = msg_samples - rx_msg_offset_;
        const size_t to_copy = std::min(samples_avail, numElems);

        const float* msg_ptr = static_cast<const float*>(zmq_msg_data(&rx_msg_));
        std::memcpy(buffs[0], msg_ptr + (rx_msg_offset_ * 2), to_copy * 2 * sizeof(float));

        rx_msg_offset_ += to_copy;
        if (rx_msg_offset_ >= msg_samples)
        {
            zmq_msg_close(&rx_msg_);
            rx_msg_valid_ = false;
            rx_msg_offset_ = 0;
        }

        return (int)to_copy;
    }

    int writeStream(SoapySDR::Stream*, const void* const* buffs, const size_t numElems,
                    int& flags, const long long, const long) override
    {
        (void)flags;
        if (!zmq_push_) return SOAPY_SDR_STREAM_ERROR;
        if (!buffs || !buffs[0]) return SOAPY_SDR_STREAM_ERROR;
        if (zmq_send(zmq_push_, buffs[0], (int)(numElems * 2 * sizeof(float)), 0) < 0)
            return SOAPY_SDR_STREAM_ERROR;
        return (int)numElems;
    }
};

// --- Registration ---
static SoapySDR::KwargsList find(const SoapySDR::Kwargs&)
{
    SoapySDR::Kwargs k;
    k["driver"] = "quadrf";
    k["label"] = "QuadRF (Thin Client)";
    return {k};
}

static SoapySDR::Device* make(const SoapySDR::Kwargs& args)
{
    return new ThinSoapyDevice(args);
}

static SoapySDR::Registry registerSoapy("quadrf", &find, &make, SOAPY_SDR_ABI_VERSION);

