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

static void usage() {
    std::cerr
        << "Usage: ntsc_demod [options] > out.yuv\n"
        << "Options:\n"
        << "  --driver <string>        Soapy driver key\n"
        << "  --chan <n>               RX channel (default: 0)\n"
        << "  --freq <Hz>              Tune frequency\n"
        << "  --rate <sps>             IQ sample rate (default: 28.63636e6 = 8*fSC)\n"
        << "  --gain <dB>              RX gain\n"
        << "  --disc <ratio|atan2>     FM discriminator (default: ratio)\n"
        << "  --no_deemph              Disable CCIR-405 de-emphasis\n"
        << "  --no_dpll                Disable line DPLL (assume exact line timing)\n"
        << "  --no_color               Force grayscale\n"
        << "  --help                   Show this\n";
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
        for (size_t i = 0; i < count; ++i) {
            buffer[(current_head + i) & mask] = data[i];
        }
        head.store(current_head + count, std::memory_order_release);
    }

    void pop(T* data, size_t count) {
        size_t current_tail = tail.load(std::memory_order_relaxed);
        for (size_t i = 0; i < count; ++i) {
            data[i] = buffer[(current_tail + i) & mask];
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
        bp_samp       = (int)std::llround(fs_vid * 5.8e-6);
        active_samp   = (int)std::llround(fs_vid * 52.6e-6);
        active_start  = sync_samp + bp_samp;
        burst_start   = sync_samp + (int)std::llround(fs_vid * 0.9e-6);
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

    float burst_amp = 0.0f, avg_burst_amp = 0.0f;
    float sync_mag = 0.0f, blank_level = 0.0f, sync_level = -0.5f;
    float white_lvl = 0.0f, y_scale = 0.0f;
    
    float dpll_n_est = 0.0f, dpll_err = 0.0f, sync_thr = 0.0f;
    float dpll_tip = 0.0f, dpll_peak = 0.0f; 
    int   hsync_w = 0, lock = 0, color_locked = 0;

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
            << "q_empty=" << queue_empty_stalls.exchange(0) << "  q_full=" << queue_full_stalls.exchange(0) << "\n"
            << "       lines=" << lines_out.load() << "  frames=" << frames_out.load() 
            << " (" << forced_frames.load() << " forced)  v_sync=" << vsync_like.load() << "\n"
            << "       h_ok=" << hsync_ok.load() << "  h_bad=" << hsync_bad.load()
            << "  drops=" << lock_drops.load() << "  dpllE=" << dpll_err << "\n"
            << "       FM_peak=" << fm_peak << "  FM_DC=" << mean_fm << " (" << freq_offset_hz / 1e6 << " MHz)\n"
            << "       IQ_mag_min=" << iq_mag_min << "  IQ_mag_avg=" << mean_mag 
            << "  max_drop_samps=" << max_dropout_len << "\n"
	    << "       lines=" << lines_out.load() << "  frames=" << frames_out.load() 
    	    << " (" << forced_frames.load() << " forced, " << dropped_frames.load() << " dropped)  v_sync=" << vsync_like.load() << "\n"
            << "       SNR=" << snr_db << " dB  white=" << white_lvl << "  sync=" << sync_level << "\n"
            << "       c_lock=" << color_locked << "  fsc_err=" << subcarrier_err_hz << " Hz  c_jitter=" << chroma_jitter << "\n";
        
        t_rx = t_fm = t_decim = t_pre = t_ntsc = 0.0;
        fm_peak = 0.0f; 
        iq_mag_min = 1e9f;
        iq_mag_acc = 0.0f;
        fm_acc = 0.0f;
        fm_count = 0;
        max_dropout_len = 0; // Reset max for the next period
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
    
    for (; i + 4 <= n; i += 4) {
        float32x4x2_t iq = vld2q_f32(p + 2*i);
        float32x4_t I = iq.val[0], Q = iq.val[1];
        float32x4_t Iprev = vextq_f32(vdupq_n_f32(pI), I, 3);
        float32x4_t Qprev = vextq_f32(vdupq_n_f32(pQ), Q, 3);
        
        float32x4_t cross = vmlsq_f32(vmulq_f32(Iprev, Q), Qprev, I);
        float32x4_t dot   = vmlaq_f32(vmulq_f32(Iprev, I), Qprev, Q);
        
        float32x4_t y = fast_atan2q_f32(cross, dot);
        vst1q_f32(out + i, y);
        
        pI = vgetq_lane_f32(I, 3);
        pQ = vgetq_lane_f32(Q, 3);
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
        buf.resize(n + HIST);
        std::memcpy(buf.data(), hist, HIST * sizeof(float));
        std::memcpy(buf.data() + HIST, in, n * sizeof(float));

        int pairs = n / 2;
        const float* p = buf.data();
        for (int i = 0; i < pairs; i++) {
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
// --------------------------
struct LineDpll {
    Diag* diag = nullptr;
    VideoTiming t;

    int hsync_min = 25;
    int hsync_max = 160;
    float falling_phi = 0.0f;

    float Kp = 0.005f;
    float Ki = 0.00001f;

    float N_est;
    float N_min;
    float N_max;

    float phi = 0.0f; 
    IIR1 lp_signal;
    float sync_tip = -1.5f;
    float white_peak = 1.0f;
    bool in_sync = false;
    int sync_width = 0, last_sync_width = 0;
    int samples_in_line = 0, bad_syncs_in_a_row = 0, vsync_integrator = 0;
    
    int missed_syncs = 0;
    int half_line_syncs = 0; // NEW: Tracker for out-of-phase condition
    bool sync_seen_this_line = false;

    static constexpr int BUF_CAP  = 4096;
    static constexpr int BUF_MASK = BUF_CAP - 1;
    alignas(64) float buf[BUF_CAP];
    int head = 0, write_idx = 0;

    LineDpll(Diag* d, const VideoTiming& timing, float fs_vid_hz)
        : diag(d), t(timing) {
        N_est = (float)t.samp_per_line;
        N_min = N_est * 0.8f;
        N_max = N_est * 1.2f;

        const float fs = (fs_vid_hz > 1.0f) ? fs_vid_hz : (float)FS_VID_DEFAULT;
        lp_signal.set_a(1.0f - std::exp(-2.0f * (float)M_PI * 500.0e3f / fs));
        std::memset(buf, 0, sizeof(buf));
    }

    inline void set_gains(float kp, float ki) { Kp = kp; Ki = ki; }

    inline bool push(float x_in, float* out_line, bool& is_vsync, int& orig_len) {
        is_vsync = false;
        bool emit = false;

        buf[write_idx] = x_in;
        write_idx = (write_idx + 1) & BUF_MASK;
        samples_in_line++; 

        float s = lp_signal.step(x_in);
        bool is_locked = (diag && diag->lock == 1);
        
        float leak_rate = is_locked ? 5e-7f : 1e-4f; 

        if (s < sync_tip) sync_tip = s; else sync_tip += leak_rate; 
        if (s > white_peak) white_peak = s; else white_peak -= leak_rate;
        if (white_peak < sync_tip + 0.5f) white_peak = sync_tip + 0.5f;

        float slice_thr = sync_tip + (white_peak - sync_tip) * 0.25f;
        float hyst = (white_peak - sync_tip) * 0.06f; 

        if (!in_sync && s < (slice_thr - hyst)) {
            in_sync = true;
            sync_width = 0;
            falling_phi = phi; 
        } else if (in_sync && s > (slice_thr + hyst)) {
            in_sync = false;
            last_sync_width = sync_width;

            if (last_sync_width >= hsync_min && last_sync_width <= hsync_max) {
                float phase_err = falling_phi;
                if (phase_err > 0.5f) phase_err -= 1.0f; 
                float samp_err = phase_err * N_est;

                float phase_half_err = falling_phi - 0.5f;
                if (phase_half_err > 0.5f) phase_half_err -= 1.0f;
                else if (phase_half_err < -0.5f) phase_half_err += 1.0f;
                float samp_half_err = phase_half_err * N_est;

                if (std::fabs(samp_err) < 100.0f) {
                    float active_kp = is_locked ? Kp : Kp * 10.0f;
                    float active_ki = is_locked ? Ki : Ki * 10.0f;

                    N_est += (active_ki * samp_err); 
                    N_est = std::clamp(N_est, N_min, N_max);
                    phi -= (active_kp * phase_err); 
                    bad_syncs_in_a_row = 0; 
                    missed_syncs = 0;
                    half_line_syncs = 0; // Reset normal pulse
                    sync_seen_this_line = true;
                    
                    if (diag) { diag->hsync_ok++; diag->lock = 1; diag->dpll_err = samp_err; }
                } else if (std::fabs(samp_half_err) < 100.0f) {
                    // Track consecutive half-lines
                    half_line_syncs++;
                    if (half_line_syncs > 20) {
                        // We are 180 degrees out of phase! Seamlessly flip the phase 180 deg.
                        phi -= 0.5f;
                        if (phi < 0.0f) phi += 1.0f;
                        half_line_syncs = 0;
                    }
                } else {
                    bad_syncs_in_a_row++;
                    if (diag) diag->hsync_bad++;
                    if (bad_syncs_in_a_row > 60) {
                        phi -= phase_err; 
                        if (diag) { diag->lock = 0; diag->lock_drops++; }
                    }
                }
            }

            const int vs_min = std::max(3 * t.sync_samp, 50);
            const int vs_max = std::max(10 * t.sync_samp, vs_min + 1);

            if (last_sync_width >= vs_min && last_sync_width <= vs_max) vsync_integrator++;
            else vsync_integrator = 0; 

            if (vsync_integrator == 2) {
                is_vsync = true;
                vsync_integrator = 3; 
                if (diag) diag->vsync_like++;
            }
        }

        if (in_sync) sync_width++;
        phi += (1.0f / N_est);

        if (phi >= 1.0f) {
            phi -= 1.0f;
            
            if (!sync_seen_this_line) {
                missed_syncs++;
                if (missed_syncs > 15) {
                    // Force a hard snap if we are completely blinded
                    bad_syncs_in_a_row = 100; 
                    if (diag) diag->lock = 0;
                }
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
                diag->hsync_w = last_sync_width;
                diag->lines_out++;
            }
        }
        return emit;
    }
};

// --------------------------
// Robust Low-SNR NTSC Decoder
// --------------------------
struct NtscDecoder {
    bool no_color = false;
    Diag* diag = nullptr;

    bool is_even_field = false; // Add interlacing tracker

    float blank_level = 0.0f, sync_level = -0.5f;
    float white_level = 0.7f, hue_rad = 0.0f, saturation = 1.0f;
    VideoTiming t;
    float fsc_norm = 0.25f, fs_vid_rate = 14318180.0f;
    float fsc_phase = 0.0f, fsc_dp = 0.0f;    
    float noise_variance = 0.0f;
    float notch_z1 = 0.0f, notch_z2 = 0.0f;

    std::vector<uint8_t> frame_yuv;
    int out_line = 0, lines_since_v = 0;
    int flush_frames = 0, frames_since_flush = 0;
    float avg_burst_amp = 0.0f;
    int xmap[OUT_W];

    alignas(64) float Y_line[MAX_LINE_SAMPS];
    alignas(64) float U_line[MAX_LINE_SAMPS];
    alignas(64) float V_line[MAX_LINE_SAMPS];
    alignas(64) float U_prev_raw[MAX_LINE_SAMPS];
    alignas(64) float V_prev_raw[MAX_LINE_SAMPS];
    alignas(64) float prev_comp_line[MAX_LINE_SAMPS];

    // Precomputed per-line oscillator (computed once per line; stored to enable NEON vector loads)
    alignas(64) float osc_cos[MAX_LINE_SAMPS + 4];
    alignas(64) float osc_sin[MAX_LINE_SAMPS + 4];

    NtscDecoder(bool grayscale, Diag* d, int flushN, float h_deg, float sat, double fs_vid)
        : no_color(grayscale), diag(d), t(fs_vid), flush_frames(flushN), saturation(sat) {
        
        fs_vid_rate = (float)fs_vid;
        hue_rad = h_deg * (float)M_PI / 180.0f;
        fsc_norm = (float)(FSC / fs_vid);
        frame_yuv.resize((size_t)OUT_W * OUT_H * 2);

        std::memset(U_prev_raw, 0, sizeof(U_prev_raw));
        std::memset(V_prev_raw, 0, sizeof(V_prev_raw));
        std::memset(prev_comp_line, 0, sizeof(prev_comp_line));
        
        int a0 = std::clamp(t.active_start, 0, t.samp_per_line);
        int a1 = std::clamp(t.active_start + t.active_samp, 0, t.samp_per_line);
        int active_len = std::max(1, a1 - a0);
        for (int x = 0; x < OUT_W; x++) {
            int si = a0 + (int)((long long)x * active_len / OUT_W);
            xmap[x] = std::clamp(si, 0, t.samp_per_line - 1);
        }
    }

    inline void emit_frame() {

	if (diag && diag->current_q_level.load(std::memory_order_relaxed) > 262144) {
            diag->dropped_frames++;
            return;// Skip writing the frame to instantly clear the pipe bottleneck and catch up.
        }

        (void)std::fwrite(frame_yuv.data(), 1, frame_yuv.size(), stdout);
        frames_since_flush++;
        if (flush_frames > 0 && frames_since_flush >= flush_frames) {
            std::fflush(stdout);
            frames_since_flush = 0;
        }
        if (diag) diag->frames_out++;
    }

    inline void process_line(const float* ln, bool is_vsync, int orig_len) {
        if (is_vsync) {
            if (lines_since_v > 100) {
                emit_frame(); 
                is_even_field = !is_even_field;
                out_line = is_even_field ? 1 : 0; 
            }
            lines_since_v = 0;
            return;
        }
        if (!ln) return;

        const int lineN = t.samp_per_line;

        float bp_acc = 0.0f, bp_sq_acc = 0.0f;
        int bp0 = t.sync_samp + (int)std::llround(fs_vid_rate * 0.5e-6);
        int bp1 = std::min(t.burst_start - 2, lineN);
        for (int i = bp0; i < bp1; i++) {
            bp_acc += ln[i];
            bp_sq_acc += ln[i] * ln[i];
        }
        
        float bp_n = std::max(1.0f, (float)(bp1 - bp0));
        float bp_mean = bp_acc / bp_n;
        float current_noise_var = std::max(0.0f, (bp_sq_acc / bp_n) - (bp_mean * bp_mean));
        noise_variance = 0.98f * noise_variance + 0.02f * current_noise_var;

        const int sN = std::min(t.sync_samp, lineN);
        constexpr int K = 8;
        float low[K];
        for (int k = 0; k < K; k++) low[k] = 1e9f;
        for (int i = 0; i < sN; i++) {
            float v = ln[i];
            if (v >= low[K-1]) continue;
            int j = K - 1;
            while (j > 0 && v < low[j-1]) { low[j] = low[j-1]; j--; }
            low[j] = v;
        }
        const int kUse = std::min(K, sN);
        float s_trim = 0.0f;
        for (int k = 0; k < kUse; k++) s_trim += low[k];
        s_trim *= 1.0f / (float)kUse;

        blank_level = 0.7f * blank_level + 0.3f * bp_mean; 
        sync_level  = 0.95f * sync_level + 0.05f * s_trim;
        float current_blank = blank_level; 

        if (lines_since_v > 400) {
            out_line = 0; lines_since_v = 0;
            if (diag) diag->forced_frames++;
            emit_frame();
        }

        lines_since_v++;
        if (lines_since_v <= 20 || out_line >= OUT_H - 1) return; 

        float sync_amp = std::max(0.1f, current_blank - sync_level);
        float target_white = current_blank + sync_amp * 2.5f; 
        white_level = 0.995f * white_level + 0.005f * target_white;
        float y_scale = 219.0f / std::max(0.1f, white_level);
        float gain_sat = saturation * y_scale;

        if (diag) {
            diag->white_lvl = white_level;
            diag->sync_level = sync_level;
            diag->y_scale = y_scale;
        }

        float d_phi = 2.0f * (float)M_PI * fsc_norm + fsc_dp;
        float d_cos = std::cos(d_phi), d_sin = std::sin(d_phi);
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

        bool color_valid = false;
        float burst_amp = 0.0f;

        if (!no_color) {
            float u_acc = 0.0f, v_acc = 0.0f;
            
            int safe_burst_start = t.sync_samp + (int)std::llround(fs_vid_rate * 0.7e-6);
            int safe_burst_len   = (int)std::llround(fs_vid_rate * 2.0e-6);
            int b0 = std::clamp(safe_burst_start, 8, lineN); // Ensure >= 8 for history
            int b1 = std::clamp(safe_burst_start + safe_burst_len, 8, lineN);
            int b_len = std::max(1, b1 - b0);

            for (int i = b0; i < b1; i++) {
                // 9-Tap FIR Bandpass Filter for pure burst extraction
                float bp_sample = 0.125f * (ln[i] + ln[i-8]) - 0.25f * (ln[i-2] + ln[i-6]) + 0.25f * ln[i-4];
                u_acc += bp_sample * osc_sin[i];
                v_acc += bp_sample * osc_cos[i];
            }

            burst_amp = (2.0f * std::sqrt(u_acc*u_acc + v_acc*v_acc)) / (float)b_len;
            avg_burst_amp = 0.95f * avg_burst_amp + 0.05f * burst_amp;
            color_valid = (avg_burst_amp > 0.015f); 

            if (color_valid) {
                float measured_phase = std::atan2(v_acc, u_acc);
                float phase_err = wrap_pm_pi(measured_phase - (float)M_PI);

                static float last_phase_err = 0.0f;
                float current_jitter = std::fabs(phase_err - last_phase_err);
                last_phase_err = phase_err;
                if (diag) diag->chroma_jitter = 0.95f * diag->chroma_jitter + 0.05f * current_jitter;

                float chroma_kp = (diag && diag->color_locked) ? 0.002f : 0.01f;
                float chroma_ki = (diag && diag->color_locked) ? 0.0000005f : 0.000005f;

                fsc_dp += chroma_ki * phase_err;             
                fsc_dp = std::clamp(fsc_dp, -0.05f, 0.05f); 
                fsc_phase += chroma_kp * phase_err;       
            }
            
            fsc_phase += (float)orig_len * (2.0f * (float)M_PI * fsc_norm + fsc_dp);
            fsc_phase = wrap_pm_pi(fsc_phase);
            
            if (diag) {
                diag->burst_amp = burst_amp;
                diag->avg_burst_amp = avg_burst_amp;
                diag->color_locked = color_valid ? 1 : 0;
                diag->subcarrier_err_hz = (fsc_dp / (2.0f * (float)M_PI)) * fs_vid_rate;

                float v_pp = std::max(0.1f, white_level - sync_level);
                float rms_noise = std::sqrt(std::max(1e-9f, noise_variance));
                float current_snr = 20.0f * std::log10(v_pp / rms_noise);
                diag->snr_db = 0.98f * diag->snr_db + 0.02f * current_snr;
            }
        }

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
        NeonFir7 fir_y(-0.031f * y_scale, 0.044f * y_scale, 0.278f * y_scale, 0.418f * y_scale);
        NeonFir7 fir_u(0.052f, 0.124f, 0.205f, 0.238f);
        NeonFir7 fir_v(0.052f, 0.124f, 0.205f, 0.238f);

        float32x4_t v_blank = vdupq_n_f32(current_blank);
        float32x4_t v_half  = vdupq_n_f32(0.5f);
        
        float32x4_t v_remod_gain = vdupq_n_f32(color_valid ? 2.0f : 0.0f);
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
            
            float32x4_t v_sin = vld1q_f32(&osc_sin[i]);
            float32x4_t v_cos = vld1q_f32(&osc_cos[i]);
            
            float32x4_t v_u_raw = vmulq_f32(v_chroma, v_sin);
            float32x4_t v_v_raw = vmulq_f32(v_chroma, v_cos);
            
            float32x4_t v_u_prev = vld1q_f32(&U_prev_raw[i]);
            float32x4_t v_v_prev = vld1q_f32(&V_prev_raw[i]);
            
            vst1q_f32(&U_prev_raw[i], v_u_raw);
            vst1q_f32(&V_prev_raw[i], v_v_raw);
            
            float32x4_t v_u_clean = vmulq_f32(vaddq_f32(v_u_raw, v_u_prev), v_half);
            float32x4_t v_v_clean = vmulq_f32(vaddq_f32(v_v_raw, v_v_prev), v_half);
            
            float32x4_t v_c_remod = vmulq_f32(v_u_clean, v_sin);
            v_c_remod = vmlaq_f32(v_c_remod, v_v_clean, v_cos);
            v_c_remod = vmulq_f32(v_c_remod, v_remod_gain);
            
            // Subtract perfectly aligned clean chroma from the delayed luma center tap
            float32x4_t v_luma = vsubq_f32(v_z4, v_c_remod);
            
            float32x4_t v_u_out = vmulq_f32(v_u_clean, v_gain_u_2x);
            float32x4_t v_v_out = vmulq_f32(v_v_clean, v_gain_v_2x);
            
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
            
            float u_raw = chroma * osc_sin[i];
            float v_raw = chroma * osc_cos[i];
            
            float u_clean = (u_raw + U_prev_raw[i]) * 0.5f;
            float v_clean = (v_raw + V_prev_raw[i]) * 0.5f;
            
            U_prev_raw[i] = u_raw;
            V_prev_raw[i] = v_raw;
            
            float c_remod = (u_clean * osc_sin[i] + v_clean * osc_cos[i]) * 2.0f;
            if (!color_valid) c_remod = 0.0f;
            
            Y_line[i] = (z4 - c_remod) * y_scale;
            U_line[i] = u_clean * gain_sat * 1.146f * 2.0f;
            V_line[i] = v_clean * gain_sat * 0.813f * 2.0f;
        }
#else
        ScalarFir7 fir_y(-0.031f * y_scale, 0.044f * y_scale, 0.278f * y_scale, 0.418f * y_scale);
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
            
            float u_raw = chroma * osc_sin[i];
            float v_raw = chroma * osc_cos[i];
            
            float u_clean = (u_raw + U_prev_raw[i]) * 0.5f;
            float v_clean = (v_raw + V_prev_raw[i]) * 0.5f;
            
            U_prev_raw[i] = u_raw;
            V_prev_raw[i] = v_raw;
            
            float c_remod = (u_clean * osc_sin[i] + v_clean * osc_cos[i]) * 2.0f;
            if (!color_valid) c_remod = 0.0f;
            
            float y_raw = z4 - c_remod;
            float u_out = u_clean * gain_sat * 1.146f * 2.0f;
            float v_out = v_clean * gain_sat * 0.813f * 2.0f;

            Y_line[i] = fir_y.step(y_raw);
            U_line[i] = fir_u.step(u_out);
            V_line[i] = fir_v.step(v_out);
        }
#endif

        uint8_t* dst = frame_yuv.data();

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
        float32x4_t v_y_off = vdupq_n_f32(16.0f);
        float32x4_t v_uv_off = vdupq_n_f32(128.0f);
        v_half = vdupq_n_f32(0.5f);
        float32x4_t v_color_en = vdupq_n_f32(color_valid ? 1.0f : 0.0f);

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
            if (color_valid) {
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
};

static inline DiscMode parse_disc(const std::string& s) {
    if (s == "atan2") return DiscMode::Atan2;
    return DiscMode::Ratio;
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
    double freq = get_dbl(argc, argv, "--freq", 0.0);
    double rate = get_dbl(argc, argv, "--rate", FS_IQ_DEFAULT);
    double gain = get_dbl(argc, argv, "--gain", -1.0);
    double bw   = get_dbl(argc, argv, "--bw", -1.0);

    bool no_color = has_arg(argc, argv, "--no_color");
    bool no_dpll = has_arg(argc, argv, "--no_dpll");

    std::string bypass_iir = get_str(argc, argv, "--bypass_iir", "");
    std::string disc_s = get_str(argc, argv, "--disc", "ratio");
    DiscMode disc = parse_disc(disc_s);

    size_t read_samps = (size_t)get_i64(argc, argv, "--read_samps", 65536);
    if (read_samps < 4096) read_samps = 4096;

    int flush_frames = (int)get_i64(argc, argv, "--flush_frames", 0);
    int hsync_min = (int)get_i64(argc, argv, "--hsync_min", 25);
    int hsync_max = (int)get_i64(argc, argv, "--hsync_max", 160);
    
    float dpll_kp = (float)get_dbl(argc, argv, "--dpll_kp", 0.005);
    float dpll_ki = (float)get_dbl(argc, argv, "--dpll_ki", 0.00001);
    float hue_deg    = (float)get_dbl(argc, argv, "--hue", 0.0);
    float saturation = (float)get_dbl(argc, argv, "--sat", 1.0);

    double diag_hz = get_dbl(argc, argv, "--diag_hz", 2.0);
    if (diag_hz < 0.1) diag_hz = 0.1;

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
        LineDpll dpll(&diag, timing, (float)fs_vid);
        dpll.set_gains(dpll_kp, dpll_ki);
        dpll.hsync_min = hsync_min;
        dpll.hsync_max = hsync_max;

        // Buffers
        std::vector<std::complex<float>> rxbuf(read_samps);
        std::vector<float> dembuf(read_samps);
        std::vector<float> vidbuf(read_samps); 

        // 512k float capacity (2MB memory footprint, must be power of 2)
        SpscQueue<float> queue(524288); 
        std::atomic<bool> running{true};

        bool use_deemph = !has_arg(argc, argv, "--no_deemph");

        // ---------------------------------------------------------
        // THREAD 2: THE CONSUMER (De-emphasis & NTSC Demodulator)
        // ---------------------------------------------------------
        std::thread consumer_thread([&]() {
            alignas(64) float consumer_buf[8192];
            alignas(64) float line_fixed[MAX_LINE_SAMPS];
            
            auto t0 = std::chrono::steady_clock::now();
            auto t_last = t0;
            double diag_period = 1.0 / diag_hz;
            uint64_t samps_since = 0;

            while (running.load(std::memory_order_relaxed)) {
                size_t available = queue.read_available();
                diag.current_q_level.store(available, std::memory_order_relaxed); // <-- Add this line
                if (available == 0) {
                    diag.queue_empty_stalls.fetch_add(1, std::memory_order_relaxed);
                    std::this_thread::sleep_for(std::chrono::microseconds(50));
                    continue;
                }
                // Process in chunks to maintain cache warmth
                size_t chunk = std::min(available, (size_t)8192);
                queue.pop(consumer_buf, chunk);
                samps_since += chunk; 

                // 4. PRE-PROCESSING
                auto t_op_start = std::chrono::steady_clock::now();
                for (size_t i = 0; i < chunk; i++) {
                    float v = consumer_buf[i];
                    if (use_deemph) v = deemph.step(v);
                    consumer_buf[i] = v;
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
        // THREAD 1: THE PRODUCER (SDR Read, FM Discrim, Decimation)
        // ---------------------------------------------------------
        std::complex<float> fm_prev(1.0f, 0.0f);

        while (true) {
            auto t_op_start = std::chrono::steady_clock::now();

            // 1. SDR READ
            void* buffs[] = {rxbuf.data()};
            int flags = 0;
            long long timeNs = 0;
            int n = dev->readStream(stream, buffs, (int)read_samps, flags, timeNs, 100000);
            diag.read_calls++;

            auto t_op_end = std::chrono::steady_clock::now();
            diag.t_rx += std::chrono::duration<double>(t_op_end - t_op_start).count();

            if (n == SOAPY_SDR_TIMEOUT) { diag.read_timeouts++; continue; }
            if (n < 0) { diag.read_errors++; std::cerr << "readStream error: " << n << "\n"; break; }
            if (n == 0) continue;

            diag.read_samps += (uint64_t)n;

            // 2. FM DISCRIMINATOR
            t_op_start = std::chrono::steady_clock::now();
            fm_disc_block(fm_prev, rxbuf.data(), n, dembuf.data(), disc);
   
            //fm_med.process_block(dembuf.data(), n); // no benefit seen with median filter

            float local_fm_peak = 0.0f;
            float local_mag_min = diag.iq_mag_min;
            float local_mag_acc = 0.0f;
            float local_fm_acc = 0.0f;
            
            uint32_t local_current_drop = diag.current_dropout_len;
            uint32_t local_max_drop = diag.max_dropout_len;
            constexpr float DROP_THR = 0.05f; // Threshold for a deep fade

            for (int i = 0; i < n; i++) {
                float abs_v = std::fabs(dembuf[i]);
                if (abs_v > local_fm_peak) local_fm_peak = abs_v;
                local_fm_acc += dembuf[i];

                float mag = std::abs(rxbuf[i]); 
                if (mag < local_mag_min) local_mag_min = mag;
                local_mag_acc += mag;

                // Track continuous width of the fade
                if (mag < DROP_THR) {
                    local_current_drop++;
                    if (local_current_drop > local_max_drop) {
                        local_max_drop = local_current_drop;
                    }
                } else {
                    local_current_drop = 0;
                }
            }
            
            if (local_fm_peak > diag.fm_peak) diag.fm_peak = local_fm_peak;
            diag.iq_mag_min = local_mag_min;
            diag.iq_mag_acc += local_mag_acc;
            diag.fm_acc += local_fm_acc;
            diag.fm_count += n;
            
            diag.current_dropout_len = local_current_drop;
            diag.max_dropout_len = local_max_drop;

            t_op_end = std::chrono::steady_clock::now();

            diag.t_fm += std::chrono::duration<double>(t_op_end - t_op_start).count();

            // 3. DECIMATION
            t_op_start = std::chrono::steady_clock::now();
            int nv = hb.process_block(dembuf.data(), n, vidbuf.data());
            t_op_end = std::chrono::steady_clock::now();
            diag.t_decim += std::chrono::duration<double>(t_op_end - t_op_start).count();

            // Push to Consumer Thread
            size_t can_write = queue.write_available();
            if (can_write < (size_t)nv) {
                diag.queue_full_stalls.fetch_add(1, std::memory_order_relaxed);
                int keep = (int)can_write;
                if (keep > 0) queue.push(vidbuf.data(), (size_t)keep);
            } else {
                queue.push(vidbuf.data(), (size_t)nv);
            }
        }

        running.store(false, std::memory_order_release);
        if (consumer_thread.joinable()) consumer_thread.join();
        
        dev->deactivateStream(stream);
        dev->closeStream(stream);
        SoapySDR::Device::unmake(dev);
        return 0;

    } catch (const std::exception& ex) {
        std::cerr << "Exception: " << ex.what() << "\n";
        return 1;
    }
}