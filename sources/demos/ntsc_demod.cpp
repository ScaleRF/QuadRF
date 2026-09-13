// ntsc_demod.cpp
// apt-get install ffmpeg
// g++ -O3 -march=native -ffast-math -std=c++17 ntsc_demod.cpp -o ntsc_demod -lSoapySDR
// Efficient NTSC (composite) over FM demodulator using SoapySDR IQ input.
// Focus: simple, robust steady-state decode on Raspberry Pi 5 (ARMv8 + NEON).

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <memory>
#include <csignal>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#endif

// --------------------------
// Simple CLI parsing helpers
// --------------------------
static bool has_arg(int argc, char** argv, const char* key) {
    for (int i = 1; i < argc; i++) if (std::string(argv[i]) == key) return true;
    return false;
}

static std::string get_str(int argc, char** argv, const char* key, const std::string& def) {
    for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == key) return argv[i + 1];
    return def;
}

static double get_dbl(int argc, char** argv, const char* key, double def) {
    for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == key) return std::stod(argv[i + 1]);
    return def;
}

static long long get_i64(int argc, char** argv, const char* key, long long def) {
    for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == key) return std::stoll(argv[i + 1]);
    return def;
}

struct FpvCh {
    const char* name;
    double mhz;
};
static const FpvCh kFpvChannels[] = {
    // Raceband (R1..R8)
    {"R1", 5658}, {"R2", 5695}, {"R3", 5732}, {"R4", 5769},
    {"R5", 5806}, {"R6", 5843}, {"R7", 5880}, {"R8", 5917},
    // Band A (Boscam A)
    {"A1", 5865}, {"A2", 5845}, {"A3", 5825}, {"A4", 5805},
    {"A5", 5785}, {"A6", 5765}, {"A7", 5745}, {"A8", 5725},
    // Band B (Boscam B)
    {"B1", 5733}, {"B2", 5752}, {"B3", 5771}, {"B4", 5790},
    {"B5", 5809}, {"B6", 5828}, {"B7", 5847}, {"B8", 5866},
    // Band E (DJI / Foxeer)
    {"E1", 5705}, {"E2", 5685}, {"E3", 5665}, {"E4", 5645},
    {"E5", 5885}, {"E6", 5905}, {"E7", 5925}, {"E8", 5945},
    // Band F (FatShark / ImmersionRC)
    {"F1", 5740}, {"F2", 5760}, {"F3", 5780}, {"F4", 5800},
    {"F5", 5820}, {"F6", 5840}, {"F7", 5860}, {"F8", 5880},
};

static double fpv_channel_hz(const std::string& s) {
    for (const auto& c : kFpvChannels) {
        if (strcasecmp(s.c_str(), c.name) == 0) return c.mhz * 1e6;
    }
    return 0.0;
}

static void usage() {
    std::cerr
        << "Usage: ntsc_demod [options] > out.yuv\n"
        << "Options:\n"
        << "  --driver <string>        Soapy driver key\n"
        << "  --chan <n>               RX channel (default: 0)\n"
        << "  --ch <R1..R8|A/B/E/F1..8> Standard 5.8 GHz FPV channel name\n"
        << "  --freq <Hz>              Tune frequency (overrides --ch)\n"
        << "  --rate <sps>             IQ sample rate (default: 28.63636e6 = 8*fSC)\n"
        << "  --gain <dB>              RX gain\n"
        << "  --disc <ratio|atan2>     FM discriminator (default: atan2)\n"
        << "  --no_deemph              Disable CCIR-405 de-emphasis\n"
        << "  --no_dpll                Disable line DPLL (assume exact line timing)\n"
        << "  --no_color               Force grayscale\n"
        << "  --stdout                 Force writing raw YUV to stdout (auto-disabled if stdout is a TTY)\n"
        << "  --no_stdout              Disable writing raw YUV to stdout\n"
        << "  --dump_ppm <dir>         Write decoded frames as PPM stills\n"
        << "  --dump_count <n>         Stop after N dumped frames (with --dump_ppm)\n"
        << "  --dump_after <n>         Skip this many frames before dumping (default: 8)\n"
        << "  --dump_fm <path>         Write float32 FM after 2:1 decim (rad / IQ sample)\n"
        << "  --dump_fm_samps <n>      FM samples to write (default: 2000000)\n"
        << "  --dump_iq <path>         Write CF32 IQ (interleaved I,Q float32)\n"
        << "  --dump_iq_samps <n>      IQ samples to write (default: 262144)\n"
        << "  --dump_lines <path>      Write locked composite lines (float32)\n"
        << "  --dump_nlines <n>        Lines to write after lock (default: 32)\n"
        << "  --dump_stop              Exit when dump quotas are filled\n"
        << "  --duration <s>           Stop after this many seconds\n"
        << "  --help                   Show this\n"
        << "\n"
        << "PPM stills: --dump_ppm DIR --dump_after 8 --dump_count N --duration S\n"
        << "IQ/FM/line dumps for ntsc_analyze.py: --dump_iq|--dump_fm|--dump_lines PATH --dump_stop\n";
}

// --------------------------
// Lock-Free SPSC Queue
// --------------------------
template <typename T>
class SpscQueue {
private:
    std::vector<T> buffer;
    const size_t capacity;
    const size_t mask;

    alignas(64) std::atomic<size_t> head{0}; 
    alignas(64) std::atomic<size_t> tail{0}; 

public:
    SpscQueue(size_t cap_power_of_two) 
        : buffer(cap_power_of_two), capacity(cap_power_of_two), mask(cap_power_of_two - 1) {}

    size_t write_available() const {
        return capacity - (head.load(std::memory_order_acquire) - tail.load(std::memory_order_relaxed));
    }

    size_t read_available() const {
        return head.load(std::memory_order_relaxed) - tail.load(std::memory_order_acquire);
    }

    void push(const T* data, size_t count) {
        size_t current_head = head.load(std::memory_order_relaxed);
        size_t idx = current_head & mask;
        size_t first = std::min(count, capacity - idx);
        std::memcpy(&buffer[idx], data, first * sizeof(T));
        if (count > first) {
            std::memcpy(&buffer[0], data + first, (count - first) * sizeof(T));
        }
        head.store(current_head + count, std::memory_order_release);
    }

    void pop(T* data, size_t count) {
        size_t current_tail = tail.load(std::memory_order_relaxed);
        size_t idx = current_tail & mask;
        size_t first = std::min(count, capacity - idx);
        std::memcpy(data, &buffer[idx], first * sizeof(T));
        if (count > first) {
            std::memcpy(data + first, &buffer[0], (count - first) * sizeof(T));
        }
        tail.store(current_tail + count, std::memory_order_release);
    }
};

// --------------------------
// Math Helpers & Filters
// --------------------------
static inline float fast_atan2f(float y, float x) {
    constexpr float PI_2 = 1.57079632679489661923f;
    if (x == 0.0f) {
        if (y > 0.0f) return PI_2;
        if (y < 0.0f) return -PI_2;
        return 0.0f;
    }
    float abs_y = std::fabs(y) + 1e-20f;
    float r, angle;
    if (x > 0.0f) {
        r = (x - abs_y) / (x + abs_y);
        angle = 0.78539816339f; 
    } else {
        r = (x + abs_y) / (abs_y - x);
        angle = 2.35619449019f; 
    }
    angle += (0.1963f * r * r - 0.9817f) * r;
    return (y < 0.0f) ? -angle : angle;
}

static inline float wrap_pm_pi(float x) {
    while (x <= -(float)M_PI) x += 2.0f * (float)M_PI;
    while (x >  (float)M_PI) x -= 2.0f * (float)M_PI;
    return x;
}

// --------------------------
// 3-Tap Median Filter
// Removes 1-sample impulsive phase wrap spikes
// --------------------------
struct MedianFilter3 {
    float z1 = 0.0f;
    float z2 = 0.0f;

    inline void process_block(float* data, int n) {
        float l_z1 = z1;
        float l_z2 = z2;
        for (int i = 0; i < n; i++) {
            float x = data[i];
            
            // Fast median of 3 values: x, l_z1, l_z2
            // median = max(min(a,b), min(max(a,b),c))
            float min_ab = (x < l_z1) ? x : l_z1;
            float max_ab = (x > l_z1) ? x : l_z1;
            float min_max_c = (max_ab < l_z2) ? max_ab : l_z2;
            float m = (min_ab > min_max_c) ? min_ab : min_max_c;

            l_z2 = l_z1;
            l_z1 = x;
            data[i] = m;
        }
        z1 = l_z1;
        z2 = l_z2;
    }
};

struct IIR1 {
    float y = 0.0f, a = 0.01f, b = 0.99f;
    inline void set_a(float alpha) { a = alpha; b = 1.0f - alpha; }
    inline float step(float x) { y = a * x + b * y; return y; }
};

struct ScalarFir7 {
    float c[4];
    float hist[6];
    ScalarFir7(float h0, float h1, float h2, float h3) {
        c[0]=h0; c[1]=h1; c[2]=h2; c[3]=h3;
        std::memset(hist, 0, sizeof(hist));
    }
    inline float step(float x) {
        float y = (x + hist[5]) * c[0] + (hist[0] + hist[4]) * c[1] + (hist[1] + hist[3]) * c[2] + hist[2] * c[3];
        for(int i=5; i>0; i--) hist[i] = hist[i-1];
        hist[0] = x;
        return y;
    }
};

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
struct NeonFir7 {
    float32x4_t c0, c1, c2, c3;
    float32x4_t prev1, prev2;

    NeonFir7(float h0, float h1, float h2, float h3) {
        c0 = vdupq_n_f32(h0); c1 = vdupq_n_f32(h1);
        c2 = vdupq_n_f32(h2); c3 = vdupq_n_f32(h3);
        prev1 = vdupq_n_f32(0.0f); prev2 = vdupq_n_f32(0.0f);
    }

    inline float32x4_t step(float32x4_t v_in) {
        float32x4_t x_0 = v_in;
        float32x4_t x_1 = vextq_f32(prev1, v_in, 3);
        float32x4_t x_2 = vextq_f32(prev1, v_in, 2);
        float32x4_t x_3 = vextq_f32(prev1, v_in, 1);
        float32x4_t x_4 = prev1;
        float32x4_t x_5 = vextq_f32(prev2, prev1, 3);
        float32x4_t x_6 = vextq_f32(prev2, prev1, 2);

        float32x4_t sum06 = vaddq_f32(x_0, x_6);
        float32x4_t sum15 = vaddq_f32(x_1, x_5);
        float32x4_t sum24 = vaddq_f32(x_2, x_4);

        float32x4_t y = vmulq_f32(sum06, c0);
        y = vmlaq_f32(y, sum15, c1);
        y = vmlaq_f32(y, sum24, c2);
        y = vmlaq_f32(y, x_3, c3);

        prev2 = prev1; prev1 = v_in;
        return y;
    }
};
#endif

static inline uint8_t u8_sat(int v) { return (uint8_t)std::clamp(v, 0, 255); }

// --------------------------
// NTSC constants
// --------------------------
static constexpr double FSC = 3579545.0;
static constexpr double FS_IQ_DEFAULT = 8.0 * FSC;   
static constexpr double FS_VID_DEFAULT = 4.0 * FSC;  

static constexpr int OUT_W = 640;
static constexpr int OUT_H = 480;

// Timing depends on fs_vid. We size all internal line buffers to a safe maximum
// to avoid dynamic allocation in the hot path.
static constexpr int MAX_LINE_SAMPS = 2048;

struct VideoTiming {
    int samp_per_line = 0;
    int sync_samp = 0;
    int bp_samp = 0;
    int active_samp = 0;
    int active_start = 0;
    int burst_start = 0;
    int burst_len = 0;

    explicit VideoTiming(double fs_vid) {
        samp_per_line = (int)std::llround(fs_vid * 63.555e-6);
        sync_samp     = (int)std::llround(fs_vid * 4.7e-6);
        bp_samp       = (int)std::llround(fs_vid * 4.7e-6);
        active_samp   = (int)std::llround(fs_vid * 52.6e-6);
        active_start  = sync_samp + bp_samp;
        burst_start   = sync_samp + (int)std::llround(fs_vid * 0.7e-6);
        burst_len     = (int)std::llround(fs_vid * 2.5e-6);

        // Clamp to our fixed buffer budget.
        samp_per_line = std::clamp(samp_per_line, 1, MAX_LINE_SAMPS);
        sync_samp     = std::clamp(sync_samp, 0, samp_per_line);
        bp_samp       = std::clamp(bp_samp, 0, samp_per_line);
        active_samp   = std::clamp(active_samp, 0, samp_per_line);
        active_start  = std::clamp(active_start, 0, samp_per_line);
        burst_start   = std::clamp(burst_start, 0, samp_per_line);
        burst_len     = std::clamp(burst_len, 0, samp_per_line - burst_start);
    }
};

// --------------------------
// Diagnostics
// --------------------------
struct Diag {
    std::atomic<uint64_t> read_calls{0}, read_samps{0}, read_timeouts{0}, read_errors{0};
    std::atomic<uint64_t> overflow_flags{0}, underflow_flags{0};
    std::atomic<uint64_t> lines_out{0}, hsync_ok{0}, hsync_bad{0}, vsync_like{0}, frames_out{0};
    std::atomic<uint64_t> lock_drops{0}, forced_frames{0}, color_killed_lines{0};
    
    // Thread Health
    std::atomic<uint64_t> queue_full_stalls{0}, queue_empty_stalls{0};
    std::atomic<uint64_t> iq_queue_full_stalls{0}, iq_queue_empty_stalls{0};

    float burst_amp = 0.0f, avg_burst_amp = 0.0f;
    float sync_mag = 0.0f, blank_level = 0.0f, sync_level = -0.5f;
    float white_lvl = 0.0f, y_scale = 0.0f;
    
    float dpll_n_est = 0.0f, dpll_err = 0.0f, sync_thr = 0.0f;
    float dpll_tip = 0.0f, dpll_peak = 0.0f; 
    int   hsync_w = 0, lock = 0, color_locked = 0;
    int   sync_locked = 0, is_mono = 0;
    float curr_sat = 1.0f, curr_hue = 0.0f;

    float iq_mag_min = 1e9f;
    float iq_mag_acc = 0.0f;
    float fm_acc = 0.0f;
    uint64_t fm_count = 0;

    uint32_t current_dropout_len = 0;
    uint32_t max_dropout_len = 0;
    
    std::atomic<size_t> current_q_level{0};
    std::atomic<uint64_t> dropped_frames{0};

    // Signal Quality Metrics
    float subcarrier_err_hz = 0.0f, noise_floor = 0.0f;
    float snr_db = 0.0f, fm_peak = 0.0f, chroma_jitter = 0.0f;

    double t_rx = 0.0, t_fm = 0.0, t_decim = 0.0, t_pre = 0.0, t_ntsc = 0.0;

    void print(double fs_iq, double fs_vid, double wall_s, double proc_samps_per_s) {
        float mean_fm = (fm_count > 0) ? (fm_acc / fm_count) : 0.0f;
        float mean_mag = (fm_count > 0) ? (iq_mag_acc / fm_count) : 0.0f;
        float freq_offset_hz = mean_fm * (fs_iq / (2.0f * (float)M_PI));

        std::cerr
            << "[diag] wall=" << wall_s << "s  proc=" << proc_samps_per_s / 1e6 << " Msps  "
            << "iq_full=" << iq_queue_full_stalls.exchange(0) << "  vid_full=" << queue_full_stalls.exchange(0)
            << "  q_empty=" << queue_empty_stalls.exchange(0) << "\n"
            << "       lines=" << lines_out.load() << "  frames=" << frames_out.load() 
            << " (" << forced_frames.load() << " forced, " << dropped_frames.load() << " dropped)  v_sync=" << vsync_like.load() << "\n"
            << "       h_ok=" << hsync_ok.load() << "  h_bad=" << hsync_bad.load()
            << "  drops=" << lock_drops.load() << "  dpllE=" << dpll_err << "\n"
            << "       FM_peak=" << fm_peak << "  FM_DC=" << mean_fm << " (" << freq_offset_hz / 1e6 << " MHz)\n"
            << "       IQ_mag_min=" << iq_mag_min << "  IQ_mag_avg=" << mean_mag 
            << "  max_drop_samps=" << max_dropout_len << "\n"
            << "       SNR=" << snr_db << " dB  white=" << white_lvl << "  sync=" << sync_level << "\n"
            << "       c_lock=" << color_locked << "  fsc_err=" << subcarrier_err_hz << " Hz  c_jitter=" << chroma_jitter << "\n";
        
        t_rx = t_fm = t_decim = t_pre = t_ntsc = 0.0;
        fm_peak = 0.0f; 
        iq_mag_min = 1e9f;
        iq_mag_acc = 0.0f;
        fm_acc = 0.0f;
        fm_count = 0;
        max_dropout_len = 0; // Reset max for the next period

        // Export telemetry to status file for mpv Diagnostic HUD
        FILE* sf = std::fopen("/dev/shm/quadrf-ntsc-status.tmp", "w");
        const char* final_path = "/dev/shm/quadrf-ntsc-status";
        if (!sf) {
            sf = std::fopen("/tmp/quadrf-ntsc-status.tmp", "w");
            final_path = "/tmp/quadrf-ntsc-status";
        }
        if (sf) {
            float clean_snr = (sync_locked && snr_db > 2.0f && snr_db < 55.0f) ? snr_db : 0.0f;
            std::fprintf(sf,
                "sync_locked=%d\n"
                "color_locked=%d\n"
                "snr=%.1f\n"
                "freq_offset=%.3f\n"
                "h_ok=%lu\n"
                "h_bad=%lu\n"
                "dpll_err=%.2f\n"
                "fsc_err=%.1f\n"
                "c_jitter=%.3f\n"
                "drops=%lu\n"
                "lines=%lu\n"
                "frames=%lu\n"
                "sat=%.2f\n"
                "hue=%.1f\n"
                "mono=%d\n"
                "wall=%.1f\n",
                sync_locked,
                color_locked,
                clean_snr,
                freq_offset_hz / 1e6f,
                (unsigned long)hsync_ok.load(),
                (unsigned long)hsync_bad.load(),
                dpll_err,
                subcarrier_err_hz,
                chroma_jitter,
                (unsigned long)lock_drops.load(),
                (unsigned long)lines_out.load(),
                (unsigned long)frames_out.load(),
                curr_sat,
                curr_hue,
                is_mono,
                wall_s
            );
            std::fclose(sf);
            if (std::strcmp(final_path, "/dev/shm/quadrf-ntsc-status") == 0) {
                std::rename("/dev/shm/quadrf-ntsc-status.tmp", "/dev/shm/quadrf-ntsc-status");
            } else {
                std::rename("/tmp/quadrf-ntsc-status.tmp", "/tmp/quadrf-ntsc-status");
            }
        }
    }
};

// --------------------------
// FM discriminator 
// --------------------------
enum class DiscMode { Ratio, Atan2 };

static inline float fm_disc_ratio_scalar(float I, float Q, float& pI, float& pQ) {
    float cross = pI * Q - pQ * I;
    float dot   = pI * I + pQ * Q;
    pI = I; pQ = Q;
    constexpr float DOT_MIN = 1e-3f;
    if (dot > -DOT_MIN && dot < DOT_MIN) dot = (dot >= 0.0f) ? DOT_MIN : -DOT_MIN;
    float y = cross / dot;
    constexpr float DCLAMP = 5.0f;
    if (y >  DCLAMP) y =  DCLAMP;
    if (y < -DCLAMP) y = -DCLAMP;
    return y;
}

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
static inline float32x4_t vrcp_refine(float32x4_t x) {
    float32x4_t r = vrecpeq_f32(x);
    r = vmulq_f32(r, vrecpsq_f32(x, r));
    r = vmulq_f32(r, vrecpsq_f32(x, r));
    return r;
}

static inline float32x4_t fast_atan2q_f32(float32x4_t y, float32x4_t x) {
    float32x4_t abs_y = vaddq_f32(vabsq_f32(y), vdupq_n_f32(1e-20f));
    uint32x4_t x_gt_0 = vcgtq_f32(x, vdupq_n_f32(0.0f));

    // x > 0: r1 = (x - abs_y) / (x + abs_y), angle1 = pi/4
    float32x4_t num1 = vsubq_f32(x, abs_y);
    float32x4_t den1 = vaddq_f32(x, abs_y);
    float32x4_t angle1 = vdupq_n_f32(0.78539816339f);

    // x <= 0: r2 = (x + abs_y) / (abs_y - x), angle2 = 3pi/4
    float32x4_t num2 = vaddq_f32(x, abs_y);
    float32x4_t den2 = vsubq_f32(abs_y, x);
    float32x4_t angle2 = vdupq_n_f32(2.35619449019f);

    // Branchless selection based on x > 0
    float32x4_t num = vbslq_f32(x_gt_0, num1, num2);
    float32x4_t den = vbslq_f32(x_gt_0, den1, den2);
    float32x4_t base_angle = vbslq_f32(x_gt_0, angle1, angle2);

    // r = num / den (using Newton-Raphson refinement)
    float32x4_t inv_den = vrecpeq_f32(den);
    inv_den = vmulq_f32(inv_den, vrecpsq_f32(den, inv_den));
    inv_den = vmulq_f32(inv_den, vrecpsq_f32(den, inv_den));
    float32x4_t r = vmulq_f32(num, inv_den);

    // angle += (0.1963f * r * r - 0.9817f) * r
    float32x4_t r2 = vmulq_f32(r, r);
    float32x4_t poly = vmlaq_f32(vdupq_n_f32(-0.9817f), vdupq_n_f32(0.1963f), r2);
    float32x4_t offset = vmulq_f32(poly, r);
    float32x4_t angle = vaddq_f32(base_angle, offset);

    // if y < 0, return -angle
    uint32x4_t y_lt_0 = vcltq_f32(y, vdupq_n_f32(0.0f));
    return vbslq_f32(y_lt_0, vnegq_f32(angle), angle);
}

static inline void fm_disc_atan2_block_neon(std::complex<float>& prev, const std::complex<float>* in, int n, float* out) {
    const float* p = reinterpret_cast<const float*>(in);
    float pI = prev.real(), pQ = prev.imag();
    int i = 0;
    
    float32x4_t prev_I_vec = vdupq_n_f32(pI);
    float32x4_t prev_Q_vec = vdupq_n_f32(pQ);

    for (; i + 4 <= n; i += 4) {
        float32x4x2_t iq = vld2q_f32(p + 2*i);
        float32x4_t I = iq.val[0], Q = iq.val[1];
        float32x4_t Iprev = vextq_f32(prev_I_vec, I, 3);
        float32x4_t Qprev = vextq_f32(prev_Q_vec, Q, 3);
        
        float32x4_t cross = vmlsq_f32(vmulq_f32(Iprev, Q), Qprev, I);
        float32x4_t dot   = vmlaq_f32(vmulq_f32(Iprev, I), Qprev, Q);
        
        float32x4_t y = fast_atan2q_f32(cross, dot);
        vst1q_f32(out + i, y);
        
        prev_I_vec = I;
        prev_Q_vec = Q;
    }
    if (i > 0) {
        pI = vgetq_lane_f32(prev_I_vec, 3);
        pQ = vgetq_lane_f32(prev_Q_vec, 3);
    }
    for (; i < n; i++) {
        float I = in[i].real(), Q = in[i].imag();
        float cross = pI * Q - pQ * I, dot   = pI * I + pQ * Q;
        out[i] = fast_atan2f(cross, dot);
        pI = I; pQ = Q;
    }
    prev = {pI, pQ};
}

static inline void fm_disc_ratio_block_neon(std::complex<float>& prev, const std::complex<float>* in, int n, float* out) {
    const float* p = reinterpret_cast<const float*>(in);
    float pI = prev.real(), pQ = prev.imag();
    int i = 0;
    for (; i + 4 <= n; i += 4) {
        float32x4x2_t iq = vld2q_f32(p + 2*i);
        float32x4_t I = iq.val[0], Q = iq.val[1];
        float32x4_t Iprev = vextq_f32(vdupq_n_f32(pI), I, 3);
        float32x4_t Qprev = vextq_f32(vdupq_n_f32(pQ), Q, 3);
        float32x4_t cross = vmlsq_f32(vmulq_f32(Iprev, Q), Qprev, I);
        float32x4_t dot   = vmlaq_f32(vmulq_f32(Iprev, I), Qprev, Q);
        constexpr float DOT_MIN = 1e-3f;
        float32x4_t absdot = vabsq_f32(dot);
        absdot = vmaxq_f32(absdot, vdupq_n_f32(DOT_MIN));
        uint32x4_t negmask = vcltq_f32(dot, vdupq_n_f32(0.0f));
        dot = vbslq_f32(negmask, vnegq_f32(absdot), absdot);
        float32x4_t inv = vrcp_refine(dot);
        float32x4_t y = vmulq_f32(cross, inv);
        constexpr float DCLAMP = 5.0f;
        y = vminq_f32(vmaxq_f32(y, vdupq_n_f32(-DCLAMP)), vdupq_n_f32(DCLAMP));
        vst1q_f32(out + i, y);
        pI = vgetq_lane_f32(I, 3);
        pQ = vgetq_lane_f32(Q, 3);
    }
    for (; i < n; i++) {
        float I = in[i].real(), Q = in[i].imag();
        out[i] = fm_disc_ratio_scalar(I, Q, pI, pQ);
    }
    prev = {pI, pQ};
}
#endif

static inline void fm_disc_block(std::complex<float>& prev, const std::complex<float>* in, int n, float* out, DiscMode mode) {
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    if (mode == DiscMode::Atan2) fm_disc_atan2_block_neon(prev, in, n, out);
    else fm_disc_ratio_block_neon(prev, in, n, out);
#else
    float pI = prev.real(), pQ = prev.imag();
    if (mode == DiscMode::Atan2) {
        for (int i = 0; i < n; i++) {
            float I = in[i].real(), Q = in[i].imag();
            out[i] = fast_atan2f(pI * Q - pQ * I, pI * I + pQ * Q);
            pI = I; pQ = Q;
        }
    } else {
        for (int i = 0; i < n; i++) out[i] = fm_disc_ratio_scalar(in[i].real(), in[i].imag(), pI, pQ);
    }
    prev = {pI, pQ};
#endif
}

// --------------------------
// Decimator & De-emphasis
// --------------------------
struct FastDecim2 {
    static constexpr int HIST = 10; 
    std::vector<float> buf;
    float hist[HIST];

    FastDecim2() { std::memset(hist, 0, sizeof(hist)); buf.reserve(131072); }

    int process_block(const float* in, int n, float* out) {
        if (n == 0) return 0;
        buf.resize(n + HIST + 32);
        std::memcpy(buf.data(), hist, HIST * sizeof(float));
        std::memcpy(buf.data() + HIST, in, n * sizeof(float));
        std::memset(buf.data() + HIST + n, 0, 32 * sizeof(float));

        int pairs = n / 2;
        const float* p = buf.data();
        int i = 0;
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
        const float32x4_t c5 = vdupq_n_f32(0.5f);
        const float32x4_t c4 = vdupq_n_f32(0.302f);
        const float32x4_t c2 = vdupq_n_f32(-0.076f);
        const float32x4_t c0 = vdupq_n_f32(0.024f);
        for (; i + 4 <= pairs; i += 4) {
            float32x4x2_t v0 = vld2q_f32(p);
            float32x4x2_t v1 = vld2q_f32(p + 8);
            float32x4_t e2 = vld1q_f32(p + 16);

            float32x4_t e0 = v0.val[0];
            float32x4_t e1 = v1.val[0];
            float32x4_t o0 = v0.val[1];
            float32x4_t o1 = v1.val[1];

            float32x4_t t0  = e0;
            float32x4_t t2  = vextq_f32(e0, e1, 1);
            float32x4_t t4  = vextq_f32(e0, e1, 2);
            float32x4_t t6  = vextq_f32(e0, e1, 3);
            float32x4_t t8  = e1;
            float32x4_t t10 = vextq_f32(e1, e2, 1);
            float32x4_t t5  = vextq_f32(o0, o1, 2);

            float32x4_t y = vmulq_f32(t5, c5);
            y = vmlaq_f32(y, vaddq_f32(t4, t6), c4);
            y = vmlaq_f32(y, vaddq_f32(t2, t8), c2);
            y = vmlaq_f32(y, vaddq_f32(t0, t10), c0);
            vst1q_f32(out + i, y);
            p += 8;
        }
#endif
        for (; i < pairs; i++) {
            out[i] = 0.5f * p[5] + 0.302f * (p[4] + p[6]) - 0.076f * (p[2] + p[8]) + 0.024f * (p[0] + p[10]);
            p += 2;
        }
        std::memcpy(hist, in + n - HIST, HIST * sizeof(float));
        return pairs;
    }
};

struct DeemphasisCCIR405 {
    float b0 = 1.0f, b1 = 0.0f, a1 = 0.0f;
    float x1 = 0.0f, y1 = 0.0f;

    DeemphasisCCIR405(double fs) {
        constexpr double tau_z = 0.85084e-6; 
        constexpr double tau_p = 0.18188e-6; 
        double alpha_z = 2.0 * tau_p * fs; 
        double alpha_p = 2.0 * tau_z * fs;
        b0 = (float)((1.0 + alpha_z) / (1.0 + alpha_p));
        b1 = (float)((1.0 - alpha_z) / (1.0 + alpha_p));
        a1 = (float)((1.0 - alpha_p) / (1.0 + alpha_p));
    }
    inline float step(float x) {
        float y = b0 * x + b1 * x1 - a1 * y1;
        x1 = x; y1 = y;
        return y;
    }
};

// --------------------------
// DPLL
// --------------------------
// --------------------------
// DPLL
struct LineDpll {
    Diag* diag = nullptr;
    VideoTiming t;

    int hsync_min = 20;
    int hsync_max = 100;
    float falling_phi = 0.0f;

    float Kp = 0.015f;
    float Ki = 0.00005f;

    float N_est;
    float N_min;
    float N_max;

    float phi = 0.0f; 
    IIR1 lp_signal;
    float boxcar[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int box_idx = 0;

    float sync_tip = -0.5f;
    float blank_level = 0.0f;
    float white_peak = 0.7f;
    float min_pulse_s = 0.0f;
    bool initial_dc_set = false;
    int back_porch_samps = 0;
    float back_porch_acc = 0.0f;

    uint64_t total_samples = 0;
    uint64_t last_hsync_falling_sample = 0;

    bool in_sync = false;
    int sync_width = 0, last_sync_width = 0;
    int samples_in_line = 0, bad_syncs_in_a_row = 0, vsync_integrator = 0;
    
    int missed_syncs = 0;
    bool sync_seen_this_line = false;
    bool is_locked = false;
    int lines_locked = 0;

    int cur_frame_h_ok = 0;
    int cur_frame_h_bad = 0;
    int cur_frame_missed = 0;
    int cur_frame_v_half = 0;

    static constexpr int BUF_CAP  = 4096;
    static constexpr int BUF_MASK = BUF_CAP - 1;
    alignas(64) float buf[BUF_CAP];
    int head = 0, write_idx = 0;

    LineDpll(Diag* d, const VideoTiming& timing, float fs_vid_hz)
        : diag(d), t(timing) {
        N_est = (float)t.samp_per_line;
        N_min = N_est * 0.95f;
        N_max = N_est * 1.05f;

        const float fs = (fs_vid_hz > 1.0f) ? fs_vid_hz : (float)FS_VID_DEFAULT;
        lp_signal.set_a(1.0f - std::exp(-2.0f * (float)M_PI * 500.0e3f / fs));
        std::memset(buf, 0, sizeof(buf));
    }

    inline void set_gains(float kp, float ki) { Kp = kp; Ki = ki; }

    inline bool push(float x_in, float* out_line, bool& is_vsync, int& orig_len) {
        is_vsync = false;
        bool emit = false;

        total_samples++;
        buf[write_idx] = x_in;
        write_idx = (write_idx + 1) & BUF_MASK;
        samples_in_line++; 

        // 4-sample boxcar filter: exact null at 3.579545 MHz (fs / 4) to eliminate color burst
        boxcar[box_idx] = x_in;
        box_idx = (box_idx + 1) & 3;
        float x_notch = 0.25f * (boxcar[0] + boxcar[1] + boxcar[2] + boxcar[3]);
        float s = lp_signal.step(x_notch);

        if (!initial_dc_set) {
            if (samples_in_line == 1) {
                sync_tip = s;
                blank_level = s;
                white_peak = s;
            } else {
                if (s < sync_tip && s > -1.50f) sync_tip = s;
                if (s > blank_level && s < 1.00f) blank_level = s;
                if (s > white_peak && s < 2.00f) white_peak = s;
            }
            if (samples_in_line >= 910) {
                initial_dc_set = true;
                sync_tip = std::clamp(sync_tip, -1.40f, 0.00f);
                blank_level = std::clamp(blank_level, sync_tip + 0.20f, 0.50f);
                white_peak = std::clamp(white_peak, 0.40f, 1.80f);
            }
            phi += (1.0f / N_est);
            if (phi >= 1.0f) {
                phi -= 1.0f;
                emit = true;
                orig_len = samples_in_line;
                samples_in_line = 0;
                int max_copy = std::min(orig_len, t.samp_per_line);
                int part1 = std::min(max_copy, BUF_CAP - head);
                std::memcpy(out_line, &buf[head], part1 * sizeof(float));
                if (max_copy > part1) std::memcpy(out_line + part1, &buf[0], (max_copy - part1) * sizeof(float));
                head = (head + orig_len) & BUF_MASK;
            }
            return emit;
        }

        // Standard EIA-170 NTSC 50% sync-to-blanking slicing:
        // Sync tip = -40 IRE, Blanking = 0 IRE. Slicing at 50% = -20 IRE.
        // Independent of scene video content / active video brightness.
        float sync_depth = std::clamp(blank_level - sync_tip, 0.20f, 0.70f);
        float slice_thr = sync_tip + sync_depth * 0.50f;
        float hyst = sync_depth * 0.12f;

        if (!in_sync && s < (slice_thr - hyst)) {
            in_sync = true;
            sync_width = 0;
            falling_phi = phi; 
            min_pulse_s = s;
        } else if (in_sync) {
            if (s < min_pulse_s) min_pulse_s = s;
            if (s > (slice_thr + hyst) || sync_width > 450) {
                in_sync = false;
                last_sync_width = sync_width;

                // Robust sync_tip tracking: only update from validated sync pulse bottoms (width >= 20).
                if (last_sync_width >= 20) {
                    float clamped_tip = std::clamp(min_pulse_s, -1.40f, 0.10f);
                    sync_tip = 0.95f * sync_tip + 0.05f * clamped_tip;
                }
                back_porch_samps = 0;
                back_porch_acc = 0.0f;

                // EIA-170 sync pulse widths at 14.318 MHz:
                // Equalizing pulse: 2.3 us = 33 samples (range 18..45)
                // Horizontal sync: 4.7 us = 67 samples (range 48..95)
                if (last_sync_width >= hsync_min && last_sync_width <= hsync_max) {
                    uint64_t falling_edge_sample = total_samples - (uint64_t)last_sync_width;

                    // Direct line period measurement between consecutive genuine H-sync pulses (48..85):
                    if (last_sync_width >= 48 && last_sync_width <= 85) {
                        if (last_hsync_falling_sample > 0) {
                            uint64_t delta_s = falling_edge_sample - last_hsync_falling_sample;
                            if (delta_s >= 880 && delta_s <= 940) {
                                if (!is_locked) {
                                    N_est = 0.85f * N_est + 0.15f * (float)delta_s;
                                } else {
                                    N_est += 0.002f * ((float)delta_s - N_est);
                                }
                            }
                        }
                        last_hsync_falling_sample = falling_edge_sample;
                    }

                    float phase_err = falling_phi;
                    if (phase_err > 0.5f) phase_err -= 1.0f; 
                    float samp_err = phase_err * N_est;

                    float phase_half_err = falling_phi - 0.5f;
                    if (phase_half_err > 0.5f) phase_half_err -= 1.0f;
                    else if (phase_half_err < -0.5f) phase_half_err += 1.0f;
                    float samp_half_err = phase_half_err * N_est;

                    if (!is_locked) {
                        // ACQUISITION: When unlocked, genuine H-sync (48..85) snaps phase
                        if (last_sync_width >= 48 && last_sync_width <= 85) {
                            phi = (float)last_sync_width / N_est;
                            bad_syncs_in_a_row = 0;
                            missed_syncs = 0;
                            sync_seen_this_line = true;
                            lines_locked++;
                            if (lines_locked >= 5) is_locked = true;
                            cur_frame_h_ok++;
                            if (diag) { diag->hsync_ok++; diag->lock = is_locked ? 1 : 0; diag->dpll_err = 0.0f; }
                        }
                    } else {
                        // TRACKING: Adaptive flywheel PLL
                        // When well-locked (>100 lines), widen tracking window to absorb thermal drift
                        float track_window = (lines_locked > 100) ? 200.0f : 140.0f;
                        // Adaptive Ki: increase when drift is consistently in one direction
                        float adaptive_ki = Ki;
                        if (lines_locked > 50) {
                            adaptive_ki = Ki * 4.0f; // Faster integral tracking when stably locked
                        }
                        if (std::fabs(samp_err) < track_window) {
                            N_est += (adaptive_ki * samp_err); 
                            N_est = std::clamp(N_est, N_min, N_max);
                            phi -= (Kp * phase_err); 
                            bad_syncs_in_a_row = 0; 
                            missed_syncs = 0;
                            sync_seen_this_line = true;
                            lines_locked++;
                            cur_frame_h_ok++;
                            if (diag) { diag->hsync_ok++; diag->lock = 1; diag->dpll_err = samp_err; }
                        } else if (std::fabs(samp_half_err) < track_window) {
                            // Mid-line equalizing pulse during VBI
                            sync_seen_this_line = true;
                            bad_syncs_in_a_row = 0;
                            cur_frame_v_half++;
                        } else if (std::fabs(samp_err) < 300.0f && last_sync_width >= 48 && last_sync_width <= 85) {
                            // Soft snap: phase error is large but still plausible for a genuine H-sync.
                            // Apply a strong proportional correction to pull back without full snap-reset.
                            phi -= 0.5f * phase_err;
                            N_est += 0.01f * ((phase_err * N_est > 0 ? 1.0f : -1.0f) * std::fabs(samp_err) * 0.001f);
                            N_est = std::clamp(N_est, N_min, N_max);
                            bad_syncs_in_a_row = 0;
                            missed_syncs = 0;
                            sync_seen_this_line = true;
                            cur_frame_h_ok++;
                            if (diag) { diag->hsync_ok++; diag->lock = 1; diag->dpll_err = samp_err; }
                        } else {
                            cur_frame_h_bad++;
                            if (diag) diag->hsync_bad++;
                            if (last_sync_width >= 48 && last_sync_width <= 85) {
                                bad_syncs_in_a_row++;
                                if (bad_syncs_in_a_row >= 2) {
                                    // Phase slipped — snap-reset after just 2 bad syncs (was 3)
                                    phi = (float)last_sync_width / N_est;
                                    bad_syncs_in_a_row = 0;
                                    sync_seen_this_line = true;
                                    missed_syncs = 0;
                                    cur_frame_h_ok++;
                                    if (diag) { diag->hsync_ok++; diag->lock = 1; diag->dpll_err = 0.0f; }
                                }
                            }
                        }
                    }
                }

                const int vs_min = std::max(3 * t.sync_samp, 50);
                const int vs_max = std::max(10 * t.sync_samp, vs_min + 1);

                if (last_sync_width >= vs_min && last_sync_width <= vs_max) {
                    // Vertical serration pulse: mark sync seen!
                    sync_seen_this_line = true;
                    bad_syncs_in_a_row = 0;
                    vsync_integrator++;
                } else {
                    vsync_integrator = 0; 
                }

                if (vsync_integrator == 2) {
                    is_vsync = true;
                    vsync_integrator = 3; 
                    if (diag) diag->vsync_like++;
                }
            }
        }

        if (in_sync) {
            sync_width++;
        } else {
            // Measure blanking level on the back porch (breezeway, 10..30 samples after sync rising edge)
            back_porch_samps++;
            if (back_porch_samps >= 10 && back_porch_samps <= 30) {
                back_porch_acc += s;
                if (back_porch_samps == 30) {
                    float bp_mean = back_porch_acc * (1.0f / 21.0f);
                    blank_level = 0.95f * blank_level + 0.05f * bp_mean;
                }
            }
            if (s > white_peak) {
                float clamped_white = std::min(s, 2.00f);
                white_peak = 0.98f * white_peak + 0.02f * clamped_white;
            }
        }

        phi += (1.0f / N_est);

        if (phi >= 1.0f) {
            phi -= 1.0f;

            // Adaptive leak rates: very slow when stably locked, faster when hunting
            float leak = (is_locked && lines_locked > 100) ? 0.00001f : (is_locked ? 0.00005f : 0.0002f);
            sync_tip += leak;
            blank_level -= leak;
            white_peak -= leak;
            sync_tip = std::clamp(sync_tip, -1.40f, 0.00f);
            blank_level = std::clamp(blank_level, -0.70f, 0.50f);
            white_peak = std::clamp(white_peak, 0.20f, 2.00f);
            
            if (!sync_seen_this_line) {
                missed_syncs++;
                cur_frame_missed++;
                // More flywheel tolerance: 60 missed syncs before unlock (was 40)
                // This helps during VBI and brief interference
                if (missed_syncs > 60) {
                    missed_syncs = 0;
                    is_locked = false;
                    lines_locked = 0;
                    if (diag) { diag->lock = 0; diag->lock_drops++; }
                }
            } else {
                missed_syncs = 0;
            }
            sync_seen_this_line = false;

            emit = true;
            orig_len = samples_in_line;
            samples_in_line = 0; 

            int max_copy = std::min(orig_len, t.samp_per_line);
            int part1 = std::min(max_copy, BUF_CAP - head);
            std::memcpy(out_line, &buf[head], part1 * sizeof(float));
            if (max_copy > part1) std::memcpy(out_line + part1, &buf[0], (max_copy - part1) * sizeof(float));
            
            float pad_val = (max_copy > 0) ? out_line[max_copy - 1] : 0.0f;
            for (int i = max_copy; i < t.samp_per_line; i++) out_line[i] = pad_val;

            head = (head + orig_len) & BUF_MASK;

            if (diag) {
                diag->dpll_n_est = N_est;
                diag->sync_thr = slice_thr;
                diag->dpll_tip = sync_tip;
                diag->dpll_peak = blank_level;
                diag->hsync_w = last_sync_width;
                diag->lines_out++;
            }
        }
        return emit;
    }
};

// --------------------------
// Decoupled Non-blocking Display Worker
// --------------------------
struct DisplayWorker {
    std::vector<uint8_t> buf;
    std::atomic<bool> ready{false};
    std::atomic<bool> running{true};
    std::thread th;
    int flush_frames = 0;
    int frames_since_flush = 0;
    Diag* diag = nullptr;

    DisplayWorker(int flushN, Diag* d) : flush_frames(flushN), diag(d) {
        buf.resize((size_t)OUT_W * OUT_H * 2);
        th = std::thread([this]() {
            while (running.load(std::memory_order_relaxed)) {
                if (!ready.load(std::memory_order_acquire)) {
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                    continue;
                }
                size_t written = std::fwrite(buf.data(), 1, buf.size(), stdout);
                ready.store(false, std::memory_order_release);
                if (written == buf.size()) {
                    frames_since_flush++;
                    if (flush_frames > 0 && frames_since_flush >= flush_frames) {
                        std::fflush(stdout);
                        frames_since_flush = 0;
                    }
                    if (diag) diag->frames_out++;
                }
            }
        });
    }

    ~DisplayWorker() {
        running.store(false, std::memory_order_release);
        if (th.joinable()) th.join();
    }

    inline void submit(const uint8_t* yuv) {
        if (ready.load(std::memory_order_relaxed)) {
            // Downstream display consumer (mpv/pipe) is lagging behind real-time.
            // Drop this display frame to guarantee the real-time DSP consumer thread NEVER stalls!
            if (diag) diag->dropped_frames++;
            return;
        }
        std::memcpy(buf.data(), yuv, buf.size());
        ready.store(true, std::memory_order_release);
    }
};

// --------------------------
// Robust Low-SNR NTSC Decoder
// --------------------------
struct NtscDecoder {
    std::unique_ptr<DisplayWorker> display;
    bool no_color = false;
    Diag* diag = nullptr;

    float blank_level = 0.0f, sync_level = -0.5f;
    float white_level = 0.7f, hue_rad = 0.0f, saturation = 1.0f;
    float hue_deg = 0.0f;
    int ctrl_check_countdown = 0;
    VideoTiming t;
    float fsc_norm = 0.25f, fs_vid_rate = 14318180.0f;
    float fsc_phase = 0.0f, fsc_dp = 0.0f;
    float last_phase_err = 0.0f;
    float noise_variance = 0.005f;
    float notch_z1 = 0.0f, notch_z2 = 0.0f;

    std::vector<uint8_t> frame_yuv;
    int out_line = 0, lines_since_v = 0;
    bool vsync_locked = false;
    int flush_frames = 0, frames_since_flush = 0;
    float avg_burst_amp = 0.0f;
    int xmap[OUT_W];
    bool write_stdout = true;

    std::string dump_dir;
    int dump_count = 0;
    int dump_after = 8;
    int dump_written = 0;
    int frames_seen = 0;
    std::atomic<bool>* stop_flag = nullptr;

    LineDpll* dpll = nullptr;
    FILE* frame_log_f = nullptr;
    std::chrono::steady_clock::time_point t_epoch;

    alignas(64) float Y_line[MAX_LINE_SAMPS];
    alignas(64) float U_line[MAX_LINE_SAMPS];
    alignas(64) float V_line[MAX_LINE_SAMPS];
    alignas(64) float U_prev_raw[MAX_LINE_SAMPS];
    alignas(64) float V_prev_raw[MAX_LINE_SAMPS];
    alignas(64) float C_prev_raw[MAX_LINE_SAMPS];
    alignas(64) float prev_comp_line[MAX_LINE_SAMPS];

    // Precomputed per-line oscillator (computed once per line; stored to enable NEON vector loads)
    alignas(64) float osc_cos[MAX_LINE_SAMPS + 4];
    alignas(64) float osc_sin[MAX_LINE_SAMPS + 4];

    void update_controls() {
        if (--ctrl_check_countdown > 0) return;
        ctrl_check_countdown = 30; // Check every ~30 lines (~1.9 ms)

        FILE* cf = std::fopen("/dev/shm/quadrf-ntsc-ctrl", "r");
        if (!cf) cf = std::fopen("/tmp/quadrf-ntsc-ctrl", "r");
        if (cf) {
            char line[128];
            float s_val = saturation;
            float h_val = hue_deg;
            int m_val = no_color ? 1 : 0;
            while (std::fgets(line, sizeof(line), cf)) {
                float fv = 0.0f;
                int iv = 0;
                if (std::sscanf(line, "sat=%f", &fv) == 1) s_val = fv;
                else if (std::sscanf(line, "hue=%f", &fv) == 1) h_val = fv;
                else if (std::sscanf(line, "mono=%d", &iv) == 1) m_val = iv;
            }
            std::fclose(cf);

            saturation = std::clamp(s_val, 0.0f, 3.0f);
            hue_deg = h_val;
            hue_rad = hue_deg * (float)M_PI / 180.0f;
            no_color = (m_val == 1) || (saturation <= 0.001f);
        }
    }

    NtscDecoder(bool grayscale, Diag* d, int flushN, float h_deg, float sat, double fs_vid)
        : no_color(grayscale), diag(d), t(fs_vid), flush_frames(flushN), saturation(sat), hue_deg(h_deg) {
        
        fs_vid_rate = (float)fs_vid;
        hue_rad = h_deg * (float)M_PI / 180.0f;
        fsc_norm = (float)(FSC / fs_vid);
        noise_variance = 0.005f;
        frame_yuv.resize((size_t)OUT_W * OUT_H * 2);

        std::memset(U_prev_raw, 0, sizeof(U_prev_raw));
        std::memset(V_prev_raw, 0, sizeof(V_prev_raw));
        std::memset(C_prev_raw, 0, sizeof(C_prev_raw));
        std::memset(prev_comp_line, 0, sizeof(prev_comp_line));
        
        int a0 = std::clamp(t.active_start - 7, 0, t.samp_per_line);
        int a1 = std::clamp(a0 + t.active_samp, 0, t.samp_per_line);
        int active_len = std::max(1, a1 - a0);
        for (int x = 0; x < OUT_W; x++) {
            int si = a0 + (int)((long long)x * active_len / OUT_W);
            xmap[x] = std::clamp(si, 0, t.samp_per_line - 1);
        }
    }

    void dump_ppm(const char* path) {
        FILE* f = std::fopen(path, "wb");
        if (!f) {
            std::cerr << "dump_ppm: cannot write " << path << "\n";
            return;
        }
        std::fprintf(f, "P6\n%d %d\n255\n", OUT_W, OUT_H);
        std::vector<uint8_t> rgb((size_t)OUT_W * OUT_H * 3);
        uint8_t* dst_pix = rgb.data();
        const uint8_t* p = frame_yuv.data();
        for (int i = 0; i < OUT_W * OUT_H; i += 2) {
            int y0 = p[0], u = p[1], y1 = p[2], v = p[3];
            p += 4;
            int c0 = y0 - 16, d = u - 128, e = v - 128;
            int c1 = y1 - 16;
            dst_pix[0] = u8_sat((298 * c0 + 409 * e + 128) >> 8);
            dst_pix[1] = u8_sat((298 * c0 - 100 * d - 208 * e + 128) >> 8);
            dst_pix[2] = u8_sat((298 * c0 + 516 * d + 128) >> 8);
            dst_pix[3] = u8_sat((298 * c1 + 409 * e + 128) >> 8);
            dst_pix[4] = u8_sat((298 * c1 - 100 * d - 208 * e + 128) >> 8);
            dst_pix[5] = u8_sat((298 * c1 + 516 * d + 128) >> 8);
            dst_pix += 6;
        }
        std::fwrite(rgb.data(), 1, rgb.size(), f);
        std::fclose(f);
        std::cerr << "wrote " << path << "\n";
    }

    inline void emit_frame() {
        frames_seen++;
        if (!dump_dir.empty() && frames_seen > dump_after) {
            if (dump_count <= 0 || dump_written < dump_count) {
                char path[768];
                std::snprintf(path, sizeof(path), "%s/frame_%04d.ppm", dump_dir.c_str(), dump_written);
                dump_ppm(path);
                dump_written++;
                if (dump_count > 0 && dump_written >= dump_count && stop_flag) {
                    stop_flag->store(false, std::memory_order_release);
                }
            }
        }

        if (frame_log_f) {
            double now_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_epoch).count();
            int ok = dpll ? dpll->cur_frame_h_ok : 0;
            int bad = dpll ? dpll->cur_frame_h_bad : 0;
            int missed = dpll ? dpll->cur_frame_missed : 0;
            int vhalf = dpll ? dpll->cur_frame_v_half : 0;
            float dpll_e = diag ? diag->dpll_err : 0.0f;
            float nest = dpll ? dpll->N_est : 0.0f;
            int locked = dpll ? (dpll->is_locked ? 1 : 0) : (diag ? (diag->lock == 1 ? 1 : 0) : 0);
            std::fprintf(frame_log_f, "%d,%.4f,%d,%d,%d,%d,%d,%.3f,%.3f,%d\n",
                         frames_seen, now_s, ok, bad, missed, vhalf, out_line, dpll_e, nest, locked);
            std::fflush(frame_log_f);
            if (dpll) {
                dpll->cur_frame_h_ok = 0;
                dpll->cur_frame_h_bad = 0;
                dpll->cur_frame_missed = 0;
                dpll->cur_frame_v_half = 0;
            }
        }

        if (write_stdout && display) {
            display->submit(frame_yuv.data());
        }
    }

    inline void process_line(const float* ln, bool is_vsync, int orig_len) {
        update_controls();
        bool valid_vsync = is_vsync || (lines_since_v >= 275);

        if (valid_vsync) {
            // Emits the completed raster if sufficient lines have been drawn
            if (out_line >= 200) {
                emit_frame();
            }
            out_line = 0;
            lines_since_v = 0;
            // 2-line comb must not average field 2 against field 1's last line.
            std::memset(U_prev_raw, 0, sizeof(U_prev_raw));
            std::memset(V_prev_raw, 0, sizeof(V_prev_raw));
            std::memset(C_prev_raw, 0, sizeof(C_prev_raw));
            return;
        }
        if (!ln) return;

        const int lineN = t.samp_per_line;

        // Guard AGC tracking: only update levels when DPLL is locked.
        // When unlocked, garbled line data would contaminate the level estimates.
        bool dpll_locked = dpll ? dpll->is_locked : true;

        // Sample breezeway (after sync tip, before burst) for clean blanking level
        int bz0 = t.sync_samp + 2;
        int bz1 = std::min(lineN, t.burst_start - 2);
        if (bz1 > bz0 && dpll_locked) {
            float bz_acc = 0.0f;
            float bz_sq_acc = 0.0f;
            for (int i = bz0; i < bz1; i++) {
                bz_acc += ln[i];
                bz_sq_acc += ln[i] * ln[i];
            }
            float bz_n = (float)(bz1 - bz0);
            float bz_mean = bz_acc / bz_n;
            float cur_noise = std::max(0.0f, (bz_sq_acc / bz_n) - (bz_mean * bz_mean));
            noise_variance = 0.98f * noise_variance + 0.02f * cur_noise;
            blank_level = 0.90f * blank_level + 0.10f * bz_mean;
        }

        const int sN = std::min(t.sync_samp, lineN);
        if (dpll_locked) {
            float s_mean = 0.0f;
            for (int i = 0; i < sN; i++) s_mean += ln[i];
            s_mean /= (float)std::max(1, sN);
            sync_level  = 0.90f * sync_level + 0.10f * s_mean;
        }

        float current_blank = blank_level; 

        lines_since_v++;
        if (lines_since_v <= 17 || out_line >= OUT_H) return; 

        // 100 IRE = 2.5 * (blank - sync)
        float sync_amp = std::clamp(current_blank - sync_level, 0.10f, 0.60f);
        float target_white = current_blank + sync_amp * 2.5f;

        // Peak white tracking across active video line (protects camera highlights from blowout)
        float line_peak = current_blank;
        int p0 = std::clamp(t.active_start, 0, lineN);
        int p1 = std::clamp(p0 + t.active_samp, 0, lineN);
        if (dpll_locked) {
            for (int i = p0; i < p1; i += 4) {
                if (ln[i] > line_peak) line_peak = ln[i];
            }
        }
        float peak_white_cand = std::max(target_white, line_peak);
        if (peak_white_cand > white_level) {
            white_level = 0.95f * white_level + 0.05f * peak_white_cand;
        } else {
            // Faster white_level decay (0.98 vs 0.995) so brightness tracks properly
            // when FM deviation changes due to thermal drift
            white_level = 0.98f * white_level + 0.02f * target_white;
        }
        white_level = std::max(white_level, target_white);

        // Excursion is (white - blank), with floor based on measured sync amplitude
        // The sync_amp-based floor prevents Y-scale blowup when levels drift
        float min_excursion = std::max(0.10f, sync_amp * 2.0f);
        float video_excursion = std::max(min_excursion, white_level - current_blank);
        float y_scale = 219.0f / video_excursion;
        float gain_sat = saturation * y_scale;

        if (diag) {
            diag->sync_locked = dpll_locked ? 1 : 0;
            diag->curr_sat = saturation;
            diag->curr_hue = hue_deg;
            diag->is_mono = no_color ? 1 : 0;
            diag->white_lvl = white_level;
            diag->sync_level = sync_level;
            diag->y_scale = y_scale;
        }

        float d_phi = 2.0f * (float)M_PI * fsc_norm + fsc_dp;
        float d_cos = std::cos(d_phi), d_sin = std::sin(d_phi);

        int safe_burst_start = t.sync_samp + (int)std::llround(fs_vid_rate * 0.7e-6);
        int safe_burst_len   = (int)std::llround(fs_vid_rate * 2.0e-6);
        int b0 = std::clamp(safe_burst_start, 8, lineN); // Ensure >= 8 for history
        int b1 = std::clamp(safe_burst_start + safe_burst_len, 8, lineN);
        int b_len = std::max(1, b1 - b0);

        bool color_valid = false;
        float burst_amp = 0.0f;

        if (!no_color) {
            float c_cos = std::cos(fsc_phase + hue_rad);
            float c_sin = std::sin(fsc_phase + hue_rad);
            for (int i = 0; i < b1; i++) {
                osc_cos[i] = c_cos; osc_sin[i] = c_sin;
                float n_cos = c_cos * d_cos - c_sin * d_sin;
                float n_sin = c_sin * d_cos + c_cos * d_sin;
                c_cos = n_cos; c_sin = n_sin;
            }

            float u_acc = 0.0f, v_acc = 0.0f;
            for (int i = b0; i < b1; i++) {
                // 9-Tap FIR Bandpass Filter for pure burst extraction
                float bp_sample = 0.125f * (ln[i] + ln[i-8]) - 0.25f * (ln[i-2] + ln[i-6]) + 0.25f * ln[i-4];
                u_acc += bp_sample * osc_sin[i];
                v_acc += bp_sample * osc_cos[i];
            }

            burst_amp = (2.0f * std::sqrt(u_acc*u_acc + v_acc*v_acc)) / (float)b_len;
            avg_burst_amp = 0.95f * avg_burst_amp + 0.05f * burst_amp;
            color_valid = (avg_burst_amp > 0.015f); 

            bool in_vbi = (lines_since_v <= 10);
            if (color_valid && !in_vbi) {
                float measured_phase = std::atan2(v_acc, u_acc);
                float phase_err = wrap_pm_pi(measured_phase - (float)M_PI);

                float current_jitter = std::fabs(wrap_pm_pi(phase_err - last_phase_err));
                last_phase_err = phase_err;
                if (diag) diag->chroma_jitter = 0.95f * diag->chroma_jitter + 0.05f * current_jitter;

                // 2nd-order PLL with clean physical bounds
                fsc_dp += 0.00002f * phase_err;
                fsc_dp = std::clamp(fsc_dp, -0.0005f, 0.0005f);
                fsc_phase += 0.20f * phase_err;
                fsc_phase = wrap_pm_pi(fsc_phase);

                d_phi = 2.0f * (float)M_PI * fsc_norm + fsc_dp;
                d_cos = std::cos(d_phi);
                d_sin = std::sin(d_phi);
            }
            
            if (diag) {
                diag->burst_amp = burst_amp;
                diag->avg_burst_amp = avg_burst_amp;
                diag->color_locked = color_valid ? 1 : 0;
                diag->subcarrier_err_hz = (fsc_dp / (2.0f * (float)M_PI)) * fs_vid_rate;

                float v_pp = std::max(0.1f, white_level - sync_level);
                float rms_noise = std::sqrt(std::max(1e-6f, noise_variance));
                float current_snr = 20.0f * std::log10(v_pp / rms_noise);
                if (dpll_locked && current_snr > 2.0f && current_snr < 55.0f) {
                    diag->snr_db = 0.98f * diag->snr_db + 0.02f * current_snr;
                }
            }
        }

        // Generate aligned oscillator across the entire line for demodulation
        float c_cos = std::cos(fsc_phase + hue_rad);
        float c_sin = std::sin(fsc_phase + hue_rad);
        for (int i = 0; i < lineN; i++) {
            osc_cos[i] = c_cos; osc_sin[i] = c_sin;
            float n_cos = c_cos * d_cos - c_sin * d_sin;
            float n_sin = c_sin * d_cos + c_cos * d_sin;
            if ((i & 63) == 0) {
                float norm = 0.5f * (3.0f - (n_cos * n_cos + n_sin * n_sin)); 
                n_cos *= norm; n_sin *= norm;
            }
            c_cos = n_cos; c_sin = n_sin;
        }

        int actual_len = (orig_len > 0) ? orig_len : lineN;
        fsc_phase += (float)actual_len * d_phi;
        fsc_phase = wrap_pm_pi(fsc_phase);

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
        // 7-tap FIR with exact mathematical zero at fSC (3.579545 MHz) and -6 dB rolloff at Nyquist:
        // H(w_sc) = h3 - 2*h1 = 0.375 - 2*(0.1875) = 0.000000 (-inf dB subcarrier rejection)
        // H(pi) = h3 - 2*h2 + 2*h1 - 2*h0 = 0.500 (-6.0 dB attenuation at Nyquist to reject high-freq FM discriminator noise)
        NeonFir7 fir_y(-0.0625f * y_scale, 0.1875f * y_scale, 0.1875f * y_scale, 0.375f * y_scale);
        NeonFir7 fir_u(0.052f, 0.124f, 0.205f, 0.238f);
        NeonFir7 fir_v(0.052f, 0.124f, 0.205f, 0.238f);

        float32x4_t v_blank = vdupq_n_f32(current_blank);
        float32x4_t v_half  = vdupq_n_f32(0.5f);
        
        float32x4_t v_remod_gain = vdupq_n_f32(color_valid ? 1.0f : 0.0f);
        float32x4_t v_gain_u_2x = vdupq_n_f32(gain_sat * 1.146f * 2.0f);
        float32x4_t v_gain_v_2x = vdupq_n_f32(gain_sat * 0.813f * 2.0f);

        float32x4_t v_h0 = vdupq_n_f32(0.125f);
        float32x4_t v_h2 = vdupq_n_f32(0.25f); 

        float init_hist[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float32x4_t prev1 = vld1q_f32(init_hist);
        float32x4_t prev2 = vld1q_f32(init_hist);

        int i = 0;
        for (; i <= lineN - 4; i += 4) {
            float32x4_t v_in = vld1q_f32(&ln[i]);
            float32x4_t v_comp = vsubq_f32(v_in, v_blank);

            // Shift 8 samples of history across registers (0 memory loads)
            float32x4_t v_z0 = v_comp;
            float32x4_t v_z2 = vextq_f32(prev1, v_comp, 2);
            float32x4_t v_z4 = prev1;
            float32x4_t v_z6 = vextq_f32(prev2, prev1, 2);
            float32x4_t v_z8 = prev2;

            prev2 = prev1;
            prev1 = v_comp;

            // 9-Tap Horizontal Bandpass (Odd taps are zero, reducing MACs)
            float32x4_t sum08 = vaddq_f32(v_z0, v_z8);
            float32x4_t sum26 = vaddq_f32(v_z2, v_z6);
            float32x4_t v_chroma = vmulq_f32(sum08, v_h0);
            v_chroma = vmlsq_f32(v_chroma, sum26, v_h2); // Subtract taps 2 and 6
            v_chroma = vmlaq_f32(v_chroma, v_z4, v_h2);  // Add tap 4
            
            float32x4_t v_c_prev = vld1q_f32(&C_prev_raw[i]);
            vst1q_f32(&C_prev_raw[i], v_chroma);

            // 2-line vertical comb filter in RF/chroma domain:
            // Subcarrier inverts 180° on adjacent lines, so C[k] - C[k-1] adds chroma and cancels luma!
            float32x4_t v_c_clean = vmulq_f32(vsubq_f32(v_chroma, v_c_prev), v_half);

            // Clean luma by subtracting comb-filtered chroma from delayed center tap:
            // (Y + C) - C = Y (exact subcarrier notch with ZERO remodulation beat moiré)
            float32x4_t v_luma = vsubq_f32(v_z4, vmulq_f32(v_c_clean, v_remod_gain));

            float32x4_t v_sin = vld1q_f32(&osc_sin[i]);
            float32x4_t v_cos = vld1q_f32(&osc_cos[i]);
            
            // Demodulate clean chroma with phase-locked subcarrier:
            float32x4_t v_u_raw = vmulq_f32(v_c_clean, v_sin);
            float32x4_t v_v_raw = vmulq_f32(v_c_clean, v_cos);

            float32x4_t v_u_out = vmulq_f32(v_u_raw, v_gain_u_2x);
            float32x4_t v_v_out = vmulq_f32(v_v_raw, v_gain_v_2x);

            float32x4_t v_y_filt = fir_y.step(v_luma);
            float32x4_t v_u_filt = fir_u.step(v_u_out);
            float32x4_t v_v_filt = fir_v.step(v_v_out);

            vst1q_f32(&Y_line[i], v_y_filt);
            vst1q_f32(&U_line[i], v_u_filt);
            vst1q_f32(&V_line[i], v_v_filt);
        }

        // Scalar fallback for the tail (Flushes the NEON history buffers)
        float hist[8];
        hist[7] = vgetq_lane_f32(prev2, 0);
        hist[6] = vgetq_lane_f32(prev2, 1);
        hist[5] = vgetq_lane_f32(prev2, 2);
        hist[4] = vgetq_lane_f32(prev2, 3);
        hist[3] = vgetq_lane_f32(prev1, 0);
        hist[2] = vgetq_lane_f32(prev1, 1);
        hist[1] = vgetq_lane_f32(prev1, 2);
        hist[0] = vgetq_lane_f32(prev1, 3);

        for (; i < lineN; i++) {
            float comp = ln[i] - current_blank;
            
            float z8 = hist[7];
            float z6 = hist[5];
            float z4 = hist[3];
            float z2 = hist[1];
            float z0 = comp;

            for(int k=7; k>0; k--) hist[k] = hist[k-1];
            hist[0] = comp;

            float chroma = 0.125f * (z0 + z8) - 0.25f * (z2 + z6) + 0.25f * z4;
            float c_prev = C_prev_raw[i];
            C_prev_raw[i] = chroma;
            float c_clean = (chroma - c_prev) * 0.5f;

            float y_raw = z4 - (color_valid ? c_clean : 0.0f);
            float u_raw = c_clean * osc_sin[i];
            float v_raw = c_clean * osc_cos[i];

            Y_line[i] = y_raw * y_scale;
            U_line[i] = u_raw * gain_sat * 1.146f * 2.0f;
            V_line[i] = v_raw * gain_sat * 0.813f * 2.0f;
        }
#else
        // 7-tap FIR with exact mathematical zero at fSC (3.579545 MHz) and -6 dB rolloff at Nyquist:
        ScalarFir7 fir_y(-0.0625f * y_scale, 0.1875f * y_scale, 0.1875f * y_scale, 0.375f * y_scale);
        ScalarFir7 fir_u(0.052f, 0.124f, 0.205f, 0.238f);
        ScalarFir7 fir_v(0.052f, 0.124f, 0.205f, 0.238f);

        float hist[8] = {0.0f};
        for (int i = 0; i < lineN; i++) {
            float comp = ln[i] - current_blank;
            
            float z8 = hist[7];
            float z6 = hist[5];
            float z4 = hist[3];
            float z2 = hist[1];
            float z0 = comp;

            for(int k=7; k>0; k--) hist[k] = hist[k-1];
            hist[0] = comp;

            float chroma = 0.125f * (z0 + z8) - 0.25f * (z2 + z6) + 0.25f * z4;
            float c_prev = C_prev_raw[i];
            C_prev_raw[i] = chroma;
            float c_clean = (chroma - c_prev) * 0.5f;

            float y_raw = z4 - (color_valid ? c_clean : 0.0f);
            float u_raw = c_clean * osc_sin[i];
            float v_raw = c_clean * osc_cos[i];

            float u_out = u_raw * gain_sat * 1.146f * 2.0f;
            float v_out = v_raw * gain_sat * 0.813f * 2.0f;

            Y_line[i] = fir_y.step(y_raw);
            U_line[i] = fir_u.step(u_out);
            V_line[i] = fir_v.step(v_out);
        }
#endif

        if (out_line < OUT_H) {
            uint8_t* dst = frame_yuv.data();

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
            float32x4_t v_y_off = vdupq_n_f32(16.0f);
            float32x4_t v_uv_off = vdupq_n_f32(128.0f);
            v_half = vdupq_n_f32(0.5f);
            float32x4_t v_color_en = vdupq_n_f32((!no_color && color_valid) ? 1.0f : 0.0f);

            int x = 0;
            for (; x <= OUT_W - 16; x += 16) {
                float ye_arr[8], yo_arr[8], u_arr[8], v_arr[8];

                for (int k = 0; k < 8; k++) {
                    int si0 = xmap[x + k*2];
                    int si1 = xmap[x + k*2 + 1];
                    ye_arr[k] = Y_line[si0];
                    yo_arr[k] = Y_line[si1];
                    u_arr[k]  = U_line[si0] + U_line[si1];
                    v_arr[k]  = V_line[si0] + V_line[si1];
                }

                float32x4_t ye1 = vaddq_f32(vld1q_f32(&ye_arr[0]), v_y_off);
                float32x4_t ye2 = vaddq_f32(vld1q_f32(&ye_arr[4]), v_y_off);
                int16x8_t ye_16 = vcombine_s16(vqmovn_s32(vcvtaq_s32_f32(ye1)), vqmovn_s32(vcvtaq_s32_f32(ye2)));
                uint8x8_t ye_8 = vqmovun_s16(ye_16);

                float32x4_t yo1 = vaddq_f32(vld1q_f32(&yo_arr[0]), v_y_off);
                float32x4_t yo2 = vaddq_f32(vld1q_f32(&yo_arr[4]), v_y_off);
                int16x8_t yo_16 = vcombine_s16(vqmovn_s32(vcvtaq_s32_f32(yo1)), vqmovn_s32(vcvtaq_s32_f32(yo2)));
                uint8x8_t yo_8 = vqmovun_s16(yo_16);

                float32x4_t u_in1 = vmulq_f32(vld1q_f32(&u_arr[0]), v_color_en);
                float32x4_t u_in2 = vmulq_f32(vld1q_f32(&u_arr[4]), v_color_en);
                float32x4_t u1 = vmlaq_f32(v_uv_off, u_in1, v_half);
                float32x4_t u2 = vmlaq_f32(v_uv_off, u_in2, v_half);
                int16x8_t u_16 = vcombine_s16(vqmovn_s32(vcvtaq_s32_f32(u1)), vqmovn_s32(vcvtaq_s32_f32(u2)));
                uint8x8_t u_8 = vqmovun_s16(u_16);

                float32x4_t v_in1 = vmulq_f32(vld1q_f32(&v_arr[0]), v_color_en);
                float32x4_t v_in2 = vmulq_f32(vld1q_f32(&v_arr[4]), v_color_en);
                float32x4_t v1 = vmlaq_f32(v_uv_off, v_in1, v_half);
                float32x4_t v2 = vmlaq_f32(v_uv_off, v_in2, v_half);
                int16x8_t v_16 = vcombine_s16(vqmovn_s32(vcvtaq_s32_f32(v1)), vqmovn_s32(vcvtaq_s32_f32(v2)));
                uint8x8_t v_8 = vqmovun_s16(v_16);

                uint8x8x4_t yuyv = {ye_8, u_8, yo_8, v_8};
                int idx1 = (out_line * OUT_W + x) * 2;
                int idx2 = ((out_line + 1) * OUT_W + x) * 2;
                vst4_u8(&dst[idx1], yuyv);
                vst4_u8(&dst[idx2], yuyv);
            }

            for (; x < OUT_W; x += 2) {
#else
            for (int x = 0; x < OUT_W; x += 2) {
#endif
                int si0 = xmap[x], si1 = xmap[x+1];

                int y0 = 16 + (int)(Y_line[si0]);
                int y1 = 16 + (int)(Y_line[si1]);

                int u = 128, v = 128;
                if (!no_color && color_valid) {
                    u += (int)((U_line[si0] + U_line[si1]) * 0.5f);
                    v += (int)((V_line[si0] + V_line[si1]) * 0.5f);
                }

                int idx1 = (out_line * OUT_W + x) * 2;
                int idx2 = ((out_line + 1) * OUT_W + x) * 2;

                dst[idx1 + 0] = dst[idx2 + 0] = u8_sat(y0);
                dst[idx1 + 1] = dst[idx2 + 1] = u8_sat(u);
                dst[idx1 + 2] = dst[idx2 + 2] = u8_sat(y1);
                dst[idx1 + 3] = dst[idx2 + 3] = u8_sat(v);
            }
            out_line += 2;
        }
    }
};

static inline DiscMode parse_disc(const std::string& s) {
    if (s == "atan2") return DiscMode::Atan2;
    return DiscMode::Ratio;
}

static std::atomic<bool>* g_stop_ptr = nullptr;
static void sig_handler(int) {
    if (g_stop_ptr) g_stop_ptr->store(false, std::memory_order_release);
}

int main(int argc, char** argv) {
    if (has_arg(argc, argv, "--help")) { usage(); return 0; }

    static char stdout_buf[1 << 20];
    std::setvbuf(stdout, stdout_buf, _IOFBF, sizeof(stdout_buf));

    const std::string driver = get_str(argc, argv, "--driver", "");
    const std::string serial = get_str(argc, argv, "--serial", "");
    const std::string antenna = get_str(argc, argv, "--antenna", "");
    const std::string extraArgs = get_str(argc, argv, "--args", "");

    int chan = (int)get_i64(argc, argv, "--chan", 0);
    const std::string ch = get_str(argc, argv, "--ch", "R5");
    double freq = get_dbl(argc, argv, "--freq", 0.0);
    if (freq <= 0.0 && !ch.empty()) {
        freq = fpv_channel_hz(ch);
        if (freq <= 0.0) {
            std::cerr << "Unknown channel '" << ch << "'. Use R1..R8, A1..A8, B1..B8, E1..E8, F1..F8.\n";
        }
    }
    double rate = get_dbl(argc, argv, "--rate", FS_IQ_DEFAULT);
    double gain = get_dbl(argc, argv, "--gain", -1.0);
    double bw   = get_dbl(argc, argv, "--bw", -1.0);

    bool no_color = has_arg(argc, argv, "--no_color");
    bool no_dpll = has_arg(argc, argv, "--no_dpll");

    std::string bypass_iir = get_str(argc, argv, "--bypass_iir", "");
    std::string disc_s = get_str(argc, argv, "--disc", "atan2");
    DiscMode disc = parse_disc(disc_s);

    size_t read_samps = (size_t)get_i64(argc, argv, "--read_samps", 65536);
    if (read_samps < 4096) read_samps = 4096;

    int flush_frames = (int)get_i64(argc, argv, "--flush_frames", 0);
    int hsync_min = (int)get_i64(argc, argv, "--hsync_min", 20);
    int hsync_max = (int)get_i64(argc, argv, "--hsync_max", 100);
    
    float dpll_kp = (float)get_dbl(argc, argv, "--dpll_kp", 0.015);
    float dpll_ki = (float)get_dbl(argc, argv, "--dpll_ki", 0.00005);
    float hue_deg    = (float)get_dbl(argc, argv, "--hue", 0.0);
    float saturation = (float)get_dbl(argc, argv, "--sat", 1.0);

    double diag_hz = get_dbl(argc, argv, "--diag_hz", 2.0);
    if (diag_hz < 0.1) diag_hz = 0.1;

    const std::string dump_ppm_dir = get_str(argc, argv, "--dump_ppm", "");
    int dump_count = (int)get_i64(argc, argv, "--dump_count", 0);
    int dump_after = (int)get_i64(argc, argv, "--dump_after", 8);
    const std::string dump_fm_path = get_str(argc, argv, "--dump_fm", "");
    long long dump_fm_samps = get_i64(argc, argv, "--dump_fm_samps", 2000000);
    const std::string dump_iq_path = get_str(argc, argv, "--dump_iq", "");
    long long dump_iq_samps = get_i64(argc, argv, "--dump_iq_samps", 262144);
    const std::string dump_lines_path = get_str(argc, argv, "--dump_lines", "");
    int dump_nlines = (int)get_i64(argc, argv, "--dump_nlines", 32);
    bool dump_stop = has_arg(argc, argv, "--dump_stop");
    double duration = get_dbl(argc, argv, "--duration", 0.0);

    long long bytes_per_line = get_i64(argc, argv, "--bytes_per_line", -1);
    long long lines = get_i64(argc, argv, "--lines", -1);
    long long dt = get_i64(argc, argv, "--dt", -1);

    SoapySDR::Kwargs args;
    if (!driver.empty()) args["driver"] = driver;
    if (!serial.empty()) args["serial"] = serial;

    if (!extraArgs.empty()) {
        size_t start = 0;
        while (start < extraArgs.size()) {
            size_t comma = extraArgs.find(',', start);
            std::string kv = extraArgs.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
            size_t eq = kv.find('=');
            if (eq != std::string::npos) args[kv.substr(0, eq)] = kv.substr(eq + 1);
            if (comma == std::string::npos) break;
            start = comma + 1;
        }
    }

    if (bytes_per_line > 0) args["bytes_per_line"] = std::to_string(bytes_per_line);
    if (lines > 0)          args["lines"] = std::to_string(lines);
    if (dt > 0)             args["dt"] = std::to_string(dt);

    try {
        SoapySDR::Device* dev = SoapySDR::Device::make(args);
        if (!dev) { std::cerr << "SoapySDR::Device::make failed.\n"; return 1; }

        if (freq > 0.0) dev->setFrequency(SOAPY_SDR_RX, chan, freq);
        dev->setSampleRate(SOAPY_SDR_RX, chan, rate);
        if (bw > 0.0) dev->setBandwidth(SOAPY_SDR_RX, chan, bw);
        if (gain >= 0.0) dev->setGain(SOAPY_SDR_RX, chan, gain);
        if (!antenna.empty()) dev->setAntenna(SOAPY_SDR_RX, chan, antenna);

        if (!bypass_iir.empty()) dev->writeSetting("bypass_iir", bypass_iir);

        SoapySDR::Stream* stream = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, {(long unsigned int)chan});
        if (!stream) {
            std::cerr << "setupStream failed.\n";
            SoapySDR::Device::unmake(dev);
            return 1;
        }

        dev->activateStream(stream);

        double fs_vid = rate / 2.0;
        VideoTiming timing(fs_vid);

        std::cerr << "SoapySDR RX started (CF32)\n";
        std::cerr << "  rate(iq)  = " << rate << "\n";
        std::cerr << "  rate(vid) = " << fs_vid << "\n";
        std::cerr << "  no_color  = " << (no_color ? "true" : "false") << "\n";
        std::cerr << "  dpll      = " << (!no_dpll ? "true" : "false") << " (Kp=" << dpll_kp << ", Ki=" << dpll_ki << ")\n";
        std::cerr << "  read_samps= " << read_samps << "\n";

        Diag diag;
        FastDecim2 hb;
        DeemphasisCCIR405 deemph(fs_vid);
        MedianFilter3 fm_med;
        NtscDecoder ntsc(no_color, &diag, flush_frames, hue_deg, saturation, fs_vid);
        bool write_stdout = has_arg(argc, argv, "--stdout") || 
            (!has_arg(argc, argv, "--no_stdout") && dump_ppm_dir.empty() && !isatty(fileno(stdout)));
        ntsc.write_stdout = write_stdout;
        if (write_stdout) {
#ifdef F_SETPIPE_SZ
            fcntl(fileno(stdout), F_SETPIPE_SZ, 1048576);
#endif
            ntsc.display = std::make_unique<DisplayWorker>(flush_frames, &diag);
        }
        std::cerr << "  stdout    = " << (write_stdout ? "enabled (decoupled async non-blocking)" : "disabled (TTY/dump_ppm/suppressed)") << "\n";

        LineDpll dpll(&diag, timing, (float)fs_vid);
        dpll.set_gains(dpll_kp, dpll_ki);
        dpll.hsync_min = hsync_min;
        dpll.hsync_max = hsync_max;
        ntsc.dpll = &dpll;

        const std::string frame_log_path = get_str(argc, argv, "--frame_log", "");
        FILE* frame_log_f = nullptr;
        if (!frame_log_path.empty()) {
            frame_log_f = std::fopen(frame_log_path.c_str(), "w");
            if (frame_log_f) {
                std::fprintf(frame_log_f, "frame,time,h_ok,h_bad,h_missed,v_half,lines,dpll_err,n_est,locked\n");
                ntsc.frame_log_f = frame_log_f;
                ntsc.t_epoch = std::chrono::steady_clock::now();
                std::cerr << "  frame_log = " << frame_log_path << "\n";
            }
        }

        // Buffers
        std::vector<std::complex<float>> rxbuf(read_samps);
        std::vector<float> dembuf(read_samps);
        std::vector<float> vidbuf(read_samps); 

        // Queues:
        // iq_queue: 262,144 complex floats (2MB) - holds ~9.15ms of SDR samples
        // video_queue: 524,288 floats (2MB) - holds ~36.6ms of decimated video samples
        SpscQueue<std::complex<float>> iq_queue(262144);
        SpscQueue<float> video_queue(524288);
        std::atomic<bool> running{true};
        g_stop_ptr = &running;
        std::signal(SIGINT, sig_handler);
        std::signal(SIGTERM, sig_handler);
        std::atomic<int> dumps_pending{0};

        FILE* dump_iq_f = nullptr;
        FILE* dump_fm_f = nullptr;
        FILE* dump_lines_f = nullptr;
        long long iq_left = 0;
        long long fm_left = 0;
        std::atomic<int> lines_left{0};

        auto open_dump = [&](FILE** fp, const std::string& path, const char* tag) -> bool {
            if (path.empty()) return false;
            *fp = std::fopen(path.c_str(), "wb");
            if (!*fp) {
                std::cerr << "cannot write " << tag << " " << path << "\n";
                return false;
            }
            dumps_pending.fetch_add(1, std::memory_order_relaxed);
            std::cerr << "  " << tag << " = " << path << "\n";
            return true;
        };
        if (open_dump(&dump_iq_f, dump_iq_path, "dump_iq")) iq_left = dump_iq_samps;
        if (open_dump(&dump_fm_f, dump_fm_path, "dump_fm")) fm_left = dump_fm_samps;
        if (open_dump(&dump_lines_f, dump_lines_path, "dump_lines"))
            lines_left.store(dump_nlines, std::memory_order_relaxed);

        auto finish_dump = [&](FILE** fp) {
            if (*fp) {
                std::fclose(*fp);
                *fp = nullptr;
                dumps_pending.fetch_sub(1, std::memory_order_relaxed);
            }
        };

        if (!dump_ppm_dir.empty()) {
            if (mkdir(dump_ppm_dir.c_str(), 0755) != 0 && errno != EEXIST) {
                std::cerr << "mkdir " << dump_ppm_dir << " failed\n";
            }
            ntsc.dump_dir = dump_ppm_dir;
            ntsc.dump_count = dump_count;
            ntsc.dump_after = dump_after;
            ntsc.stop_flag = &running;
            std::cerr << "  dump_ppm = " << dump_ppm_dir
                      << " after " << dump_after << " frames, count=" << dump_count << "\n";
        }

        bool use_deemph = !has_arg(argc, argv, "--no_deemph");

        // ---------------------------------------------------------
        // THREAD 3: THE CONSUMER (De-emphasis & NTSC Demodulator)
        // ---------------------------------------------------------
        std::thread consumer_thread([&]() {
            alignas(64) float consumer_buf[8192];
            alignas(64) float line_fixed[MAX_LINE_SAMPS];
            
            auto t0 = std::chrono::steady_clock::now();
            auto t_last = t0;
            double diag_period = 1.0 / diag_hz;
            uint64_t samps_since = 0;

            while (running.load(std::memory_order_relaxed) || video_queue.read_available() > 0) {
                size_t available = video_queue.read_available();
                diag.current_q_level.store(available, std::memory_order_relaxed);
                if (available == 0) {
                    diag.queue_empty_stalls.fetch_add(1, std::memory_order_relaxed);
                    std::this_thread::sleep_for(std::chrono::microseconds(50));
                    continue;
                }
                // Process in chunks to maintain cache warmth
                size_t chunk = std::min(available, (size_t)8192);
                video_queue.pop(consumer_buf, chunk);
                samps_since += chunk; 

                // 4. PRE-PROCESSING
                auto t_op_start = std::chrono::steady_clock::now();
                if (use_deemph) {
                    for (size_t i = 0; i < chunk; i++) {
                        consumer_buf[i] = deemph.step(consumer_buf[i]);
                    }
                }
                auto t_op_end = std::chrono::steady_clock::now();
                diag.t_pre += std::chrono::duration<double>(t_op_end - t_op_start).count();

                // 5. DPLL & NTSC DEMODULATION
                t_op_start = std::chrono::steady_clock::now();
                if (no_dpll) {
                    static float tmp[MAX_LINE_SAMPS];
                    static int tp = 0;
                    for (size_t i = 0; i < chunk; i++) {
                        tmp[tp++] = consumer_buf[i];
                        if (tp == timing.samp_per_line) {
                            ntsc.process_line(tmp, false, timing.samp_per_line);
                            tp = 0;
                            diag.lines_out++;
                        }
                    }
                } else {
                    for (size_t i = 0; i < chunk; i++) {
                        bool is_vsync = false;
                        int orig_len = 0; 
                        if (dpll.push(consumer_buf[i], line_fixed, is_vsync, orig_len)) {
                            int left = lines_left.load(std::memory_order_relaxed);
                            if (dump_lines_f && left > 0 && diag.lock == 1) {
                                std::fwrite(line_fixed, sizeof(float), (size_t)timing.samp_per_line, dump_lines_f);
                                if (lines_left.fetch_sub(1, std::memory_order_relaxed) == 1) {
                                    finish_dump(&dump_lines_f);
                                    if (dump_stop && dumps_pending.load(std::memory_order_relaxed) == 0)
                                        running.store(false, std::memory_order_release);
                                }
                            }
                            ntsc.process_line(line_fixed, is_vsync, orig_len);
                        } else if (is_vsync) {
                            ntsc.process_line(nullptr, true, 0);
                        }
                    }
                }
                t_op_end = std::chrono::steady_clock::now();
                diag.t_ntsc += std::chrono::duration<double>(t_op_end - t_op_start).count();

                // Print Diagnostics from the Consumer Thread
                auto now = std::chrono::steady_clock::now();
                double wall = std::chrono::duration<double>(now - t0).count();
                double since = std::chrono::duration<double>(now - t_last).count();
                if (since >= diag_period) {
                    double proc_sps = ((double)samps_since * 2.0) / since;
                    samps_since = 0;
                    t_last = now;
                    diag.print(rate, fs_vid, wall, proc_sps);
                }
            }
        });

        // ---------------------------------------------------------
        // THREAD 2: FM DEMODULATOR & DECIMATOR WORKER
        // ---------------------------------------------------------
        std::thread fm_worker_thread([&]() {
            std::complex<float> fm_prev(1.0f, 0.0f);
            alignas(64) std::complex<float> local_iq[8192];
            alignas(64) float dembuf[8192];
            alignas(64) float vidbuf[4096];
            FastDecim2 hb;

            while (running.load(std::memory_order_relaxed) || iq_queue.read_available() > 0) {
                size_t avail = iq_queue.read_available();
                if (avail == 0) {
                    diag.iq_queue_empty_stalls.fetch_add(1, std::memory_order_relaxed);
                    std::this_thread::sleep_for(std::chrono::microseconds(50));
                    continue;
                }
                size_t chunk = std::min(avail, (size_t)8192);
                iq_queue.pop(local_iq, chunk);

                // 2. FM DISCRIMINATOR
                auto t_op_start = std::chrono::steady_clock::now();
                fm_disc_block(fm_prev, local_iq, (int)chunk, dembuf, disc);

                // Fast diagnostic telemetry tracking
                float local_fm_peak = 0.0f;
                float local_fm_acc = 0.0f;
                float local_mag_min = diag.iq_mag_min;
                float local_mag_acc = 0.0f;
                uint32_t local_current_drop = diag.current_dropout_len;
                uint32_t local_max_drop = diag.max_dropout_len;
                constexpr float DROP_THR_SQ = 0.05f * 0.05f;

                for (size_t i = 0; i < chunk; i += 8) {
                    float abs_v = std::fabs(dembuf[i]);
                    if (abs_v > local_fm_peak) local_fm_peak = abs_v;
                    local_fm_acc += dembuf[i] * 8.0f;

                    float I = local_iq[i].real();
                    float Q = local_iq[i].imag();
                    float mag_sq = I * I + Q * Q;

                    // Continuous dropout tracker
                    if (mag_sq < DROP_THR_SQ) {
                        local_current_drop += 8;
                        if (local_current_drop > local_max_drop) local_max_drop = local_current_drop;
                    } else {
                        local_current_drop = 0;
                    }

                    // Downsampled magnitude average & min (3.58 Msps telemetry rate)
                    float mag = std::sqrt(mag_sq);
                    if (mag < local_mag_min) local_mag_min = mag;
                    local_mag_acc += mag * 8.0f;
                }

                if (local_fm_peak > diag.fm_peak) diag.fm_peak = local_fm_peak;
                diag.iq_mag_min = local_mag_min;
                diag.iq_mag_acc += local_mag_acc;
                diag.fm_acc += local_fm_acc;
                diag.fm_count += chunk;
                diag.current_dropout_len = local_current_drop;
                diag.max_dropout_len = local_max_drop;

                auto t_op_end = std::chrono::steady_clock::now();
                diag.t_fm += std::chrono::duration<double>(t_op_end - t_op_start).count();

                // 3. DECIMATION
                t_op_start = std::chrono::steady_clock::now();
                int nv = hb.process_block(dembuf, (int)chunk, vidbuf);
                t_op_end = std::chrono::steady_clock::now();
                diag.t_decim += std::chrono::duration<double>(t_op_end - t_op_start).count();

                if (dump_fm_f && fm_left > 0 && nv > 0) {
                    long long take = std::min<long long>(fm_left, nv);
                    std::fwrite(vidbuf, sizeof(float), (size_t)take, dump_fm_f);
                    fm_left -= take;
                    if (fm_left <= 0) {
                        finish_dump(&dump_fm_f);
                        if (dump_stop && dumps_pending.load(std::memory_order_relaxed) == 0)
                            running.store(false, std::memory_order_release);
                    }
                }

                // Push to video queue for Consumer Thread
                size_t can_write = video_queue.write_available();
                if (can_write < (size_t)nv) {
                    diag.queue_full_stalls.fetch_add(1, std::memory_order_relaxed);
                    int keep = (int)can_write;
                    if (keep > 0) video_queue.push(vidbuf, (size_t)keep);
                } else {
                    video_queue.push(vidbuf, (size_t)nv);
                }
            }
        });

        // ---------------------------------------------------------
        // THREAD 1: SDR READ PRODUCER
        // ---------------------------------------------------------
        std::thread sdr_reader_thread([&]() {
            std::vector<std::complex<float>> rxbuf(read_samps);
            while (running.load(std::memory_order_relaxed)) {
                auto t_op_start = std::chrono::steady_clock::now();

                void* buffs[] = {rxbuf.data()};
                int flags = 0;
                long long timeNs = 0;
                int n = dev->readStream(stream, buffs, (int)read_samps, flags, timeNs, 100000);
                diag.read_calls++;

                auto t_op_end = std::chrono::steady_clock::now();
                diag.t_rx += std::chrono::duration<double>(t_op_end - t_op_start).count();

                if (n == SOAPY_SDR_TIMEOUT) { diag.read_timeouts++; continue; }
                if (n < 0) {
                    diag.read_errors++;
                    std::cerr << "readStream error: " << n << "\n";
                    running.store(false, std::memory_order_release);
                    break;
                }
                if (n == 0) continue;

                diag.read_samps += (uint64_t)n;

                if (dump_iq_f && iq_left > 0) {
                    long long take = std::min<long long>(iq_left, n);
                    std::fwrite(rxbuf.data(), sizeof(std::complex<float>), (size_t)take, dump_iq_f);
                    iq_left -= take;
                    if (iq_left <= 0) {
                        finish_dump(&dump_iq_f);
                        if (dump_stop && dumps_pending.load(std::memory_order_relaxed) == 0)
                            running.store(false, std::memory_order_release);
                    }
                }

                // Push raw IQ to FM Worker Thread
                size_t can_write = iq_queue.write_available();
                if (can_write < (size_t)n) {
                    diag.iq_queue_full_stalls.fetch_add(1, std::memory_order_relaxed);
                    int keep = (int)can_write;
                    if (keep > 0) iq_queue.push(rxbuf.data(), (size_t)keep);
                } else {
                    iq_queue.push(rxbuf.data(), (size_t)n);
                }
            }
        });

        // ---------------------------------------------------------
        // MAIN THREAD: COORDINATOR & SUPERVISOR
        // ---------------------------------------------------------
        auto t_start = std::chrono::steady_clock::now();
        while (running.load(std::memory_order_relaxed)) {
            if (duration > 0.0) {
                double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
                if (wall >= duration) break;
            }
            if (dump_stop && dumps_pending.load(std::memory_order_relaxed) == 0 &&
                (dump_iq_f != nullptr || dump_fm_f != nullptr || dump_lines_f != nullptr)) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        running.store(false, std::memory_order_release);
        if (sdr_reader_thread.joinable()) sdr_reader_thread.join();
        if (fm_worker_thread.joinable()) fm_worker_thread.join();
        if (consumer_thread.joinable()) consumer_thread.join();
        finish_dump(&dump_iq_f);
        finish_dump(&dump_fm_f);
        finish_dump(&dump_lines_f);
        if (frame_log_f) {
            std::fclose(frame_log_f);
            frame_log_f = nullptr;
        }
        
        dev->deactivateStream(stream);
        dev->closeStream(stream);
        SoapySDR::Device::unmake(dev);
        return 0;

    } catch (const std::exception& ex) {
        std::cerr << "Exception: " << ex.what() << "\n";
        return 1;
    }
}