// csi_sweep.c
//
// Fast pipelined LO sweep phase-scatter visualization for 4-channel CSI SDR.
// Telemetry measures time spent in:
//   - CSI block acquisition (read)
//   - LO programming (LO)
//   - FFT execution (FFT)
//   - processing + selection + stamping (proc)
//
// Build:
//   gcc csi_sweep.c mongoose.c -O3 -o csi_sweep -lfftw3f -lSDL2 -lm -lpthread
//
// Run:
//   ./csi_sweep [--headless]

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <signal.h>

#include <sys/mman.h>
#include <sys/ioctl.h>

#include <pthread.h>

#include <SDL2/SDL.h>
#include <fftw3.h>

#include "../fpga/drivers/csi/fpga_csi.h"
#include "delay_cal.h"

#include "mongoose.h"

// 16 bytes per point.
typedef struct {
    float tx;       
    float ty;       
    float hue;      
    float strength; 
} rf_point_t;

// Shared buffer for the web stream
#define MAX_WEB_POINTS 2048
rf_point_t web_points_buf[MAX_WEB_POINTS];
int web_points_count = 0;
bool web_frame_ready = false;
pthread_mutex_t web_mtx = PTHREAD_MUTEX_INITIALIZER;

// Global signal trap for headless mode termination
volatile sig_atomic_t sig_quit = 0;
static void handle_sig(int sig) {
    sig_quit = 1;
}

// ------------------------------------------------------------
// Configuration
// ------------------------------------------------------------

#define DEVICE_PATH      "/dev/csi_stream0"

// Changed to 65536 for high-resolution testing (can be changed back to 4096)
#define FFT_SIZE         4096
#define CHANNELS         4
#define BYTES_PER_IQ     2
#define BYTES_PER_FRAME  (CHANNELS * BYTES_PER_IQ)
#define BLOCK_BYTES      (FFT_SIZE * BYTES_PER_FRAME)

// Logical render resolution (small => "fatter" points when scaled up)
#define CANVAS_W         256
#define CANVAS_H         256

// Actual window size on screen
#define WIN_WIDTH        1920
#define WIN_HEIGHT       1024

// Sweep parameters (MHz)
#define LO_START_MHZ     4900.0
#define LO_END_MHZ       6000.0
#define LO_STEP_MHZ      18.0
#define FS_MHZ           18.0

// Antenna Geometry & Tiling Constants
#define SPEED_OF_LIGHT          299792458.0f
#define ANTENNA_SPACING_MM      45.5f
#define ANTENNA_SPACING_M       (ANTENNA_SPACING_MM * 1e-3f)

// d/lambda = d_m * f_mhz / c, where c = 299.792458 m/us
#define D_LAMBDA_PER_MHZ        (ANTENNA_SPACING_M / 299.792458f)

// 2*pi * d/lambda at a given LO frequency in MHz
#define SCALE_FACTOR_AT_MHZ(f)  (2.0f * 3.14159265358979f * D_LAMBDA_PER_MHZ * (f))

// Frequency Range (for color mapping)
#define FREQ_MIN_MHZ          4900.0
#define FREQ_MAX_MHZ          6000.0
#define CALIB_PHASE_V    (3.14159f)  
#define CALIB_PHASE_H    (3.14159f)

// If ring backlog grows, drop older data to keep latency bounded.
#define MAX_QUEUED_BLOCKS   2

// Aggressive ring-wait tuning:
#define WAIT_SPIN_ITERS     2000   // spin iterations before sleeping
#define WAIT_SLEEP_US       20     // short sleep after spin phase

// Visualization tuning
#define ENABLE_DECAY        1
#define DECAY_FACTOR        0.50f
#define POINT_GAIN          10000.0f

// Telemetry
#define TELEMETRY_PRINT_EVERY_N_FRAMES  10000   // set to 10 for less spam

bool  mirror_display = true;
#define BTN_SIZE 75
#define BTN_MARGIN 25

// ------------------------------------------------------------
// Lock Modes
// ------------------------------------------------------------
#define MODE_SWEEP  0
#define MODE_SEARCH 1
#define MODE_LOCK   2

// ------------------------------------------------------------

static void die(const char *msg)
{
    perror(msg);
    exit(EXIT_FAILURE);
}

static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static inline double ns_to_ms(uint64_t ns) { return (double)ns / 1e6; }

static inline uint32_t ring_used_bytes(uint32_t head, uint32_t tail, uint64_t ring_size)
{
    return (head >= tail) ? (head - tail)
                          : ((uint32_t)ring_size - (tail - head));
}

static void consume_bytes(int fd, uint32_t nbytes)
{
    if (!nbytes) return;
    if (ioctl(fd, CSI_IOC_CONSUME_BYTES, &nbytes) < 0)
        die("CSI_IOC_CONSUME_BYTES");
}

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline void cpu_relax(void)
{
#if defined(__aarch64__) || defined(__arm__)
    __asm__ __volatile__("yield" ::: "memory");
#elif defined(__x86_64__) || defined(__i386__)
    __asm__ __volatile__("pause" ::: "memory");
#else
    __asm__ __volatile__("" ::: "memory");
#endif
}

static void hsv_to_rgb(float h, float s, float v, float *r, float *g, float *b)
{
    if (s <= 0.0f) { *r = *g = *b = v; return; }
    h = fmodf(h, 1.0f); if (h < 0) h += 1.0f;

    float hf = h * 6.0f;
    int i = (int)hf;
    float f = hf - i;

    float p = v * (1 - s);
    float q = v * (1 - s * f);
    float t = v * (1 - s * (1 - f));

    switch (i % 6)
    {
    case 0: *r = v; *g = t; *b = p; break;
    case 1: *r = q; *g = v; *b = p; break;
    case 2: *r = p; *g = v; *b = t; break;
    case 3: *r = p; *g = q; *b = v; break;
    case 4: *r = t; *g = p; *b = v; break;
    default:*r = v; *g = p; *b = q; break;
    }
}

// Reverse the RGB mapping to find the Hue [0.0, 1.0]
static float rgb_to_hue(float r, float g, float b) {
    float max_c = fmaxf(r, fmaxf(g, b));
    float min_c = fminf(r, fminf(g, b));
    float d = max_c - min_c;
    float h = 0.0f;
    
    if (d > 0.001f) {
        if (max_c == r) {
            h = fmodf((g - b) / d, 6.0f);
            if (h < 0.0f) h += 6.0f;
        }
        else if (max_c == g) h = (b - r) / d + 2.0f;
        else if (max_c == b) h = (r - g) / d + 4.0f;
        h /= 6.0f;
    }
    return h;
}

static inline float wrap_pi_f(float x) {
    float const inv2pi = 0.15915494309189535f;
    float const twopi = 6.283185307179586f;
    x -= twopi * floorf(x * inv2pi + 0.5f);
    return x;
}

static inline float circular_mean2_f(float a, float b) {
    return atan2f(sinf(a) + sinf(b), cosf(a) + cosf(b));
}

static void decay_accum(uint16_t *acc)
{
    size_t n = (size_t)CANVAS_W * CANVAS_H * 3;
    for (size_t i = 0; i < n; ++i)
        acc[i] = (uint16_t)((float)acc[i] * DECAY_FACTOR);
}

static void accum_to_pixels(const uint16_t *acc, uint32_t *pix)
{
    for (size_t i = 0; i < (size_t)CANVAS_W * CANVAS_H; ++i)
    {
        uint32_t r = acc[i*3+0] >> 6;
        uint32_t g = acc[i*3+1] >> 6;
        uint32_t b = acc[i*3+2] >> 6;

        if (r > 255) r = 255;
        if (g > 255) g = 255;
        if (b > 255) b = 255;

        uint32_t a = (r | g | b) ? 0xFFu : 0x00u;
        pix[i] = (a<<24) | (r<<16) | (g<<8) | b;
    }
}


// Gaussian splatting for efficient, pre-computed visual blur.
static inline void stamp_gaussian_add(uint16_t *acc, int cx, int cy,
                                      float cr, float cg, float cb, float strength)
{
    #define KERNEL_R 3
    static const float kernel[7][7] = {
        {0.02f, 0.05f, 0.10f, 0.15f, 0.10f, 0.05f, 0.02f},
        {0.05f, 0.15f, 0.35f, 0.50f, 0.35f, 0.15f, 0.05f},
        {0.10f, 0.35f, 0.70f, 0.85f, 0.70f, 0.35f, 0.10f},
        {0.15f, 0.50f, 0.85f, 1.00f, 0.85f, 0.50f, 0.15f},
        {0.10f, 0.35f, 0.70f, 0.85f, 0.70f, 0.35f, 0.10f},
        {0.05f, 0.15f, 0.35f, 0.50f, 0.35f, 0.15f, 0.05f},
        {0.02f, 0.05f, 0.10f, 0.15f, 0.10f, 0.05f, 0.02f}
    };

    for (int dy = -KERNEL_R; dy <= KERNEL_R; dy++) {
        for (int dx = -KERNEL_R; dx <= KERNEL_R; dx++) {
            int px = cx + dx;
            int py = cy + dy;

            if ((unsigned)px >= (unsigned)CANVAS_W) continue;
            if ((unsigned)py >= (unsigned)CANVAS_H) continue;

            float k_w = kernel[dy + KERNEL_R][dx + KERNEL_R];
            float s = strength * k_w;

            size_t idx = ((size_t)py * CANVAS_W + (size_t)px) * 3;

            uint32_t ir = (uint32_t)(cr * s);
            uint32_t ig = (uint32_t)(cg * s);
            uint32_t ib = (uint32_t)(cb * s);

            uint32_t nr = acc[idx+0] + ir;
            uint32_t ng = acc[idx+1] + ig;
            uint32_t nb = acc[idx+2] + ib;

            acc[idx+0] = (nr > 65535u) ? 65535u : (uint16_t)nr;
            acc[idx+1] = (ng > 65535u) ? 65535u : (uint16_t)ng;
            acc[idx+2] = (nb > 65535u) ? 65535u : (uint16_t)nb;
        }
    }
}

// Direct single-pixel additive blending. Extremely fast.
static inline void stamp_pixel_add(uint16_t *acc, int px, int py,
                                   float cr, float cg, float cb, float strength)
{
    // Fast bounds check
    if ((unsigned)px >= (unsigned)CANVAS_W) return;
    if ((unsigned)py >= (unsigned)CANVAS_H) return;

    size_t idx = ((size_t)py * CANVAS_W + (size_t)px) * 3;

    uint32_t ir = (uint32_t)(cr * strength);
    uint32_t ig = (uint32_t)(cg * strength);
    uint32_t ib = (uint32_t)(cb * strength);

    uint32_t nr = acc[idx+0] + ir;
    uint32_t ng = acc[idx+1] + ig;
    uint32_t nb = acc[idx+2] + ib;

    // Clamp to 16-bit max and store
    acc[idx+0] = (nr > 65535u) ? 65535u : (uint16_t)nr;
    acc[idx+1] = (ng > 65535u) ? 65535u : (uint16_t)ng;
    acc[idx+2] = (nb > 65535u) ? 65535u : (uint16_t)nb;
}

// ------------------------------------------------------------
// Read telemetry (per frame)
// ------------------------------------------------------------

typedef struct {
    uint64_t ns_getinfo;
    uint64_t ns_copy;
    uint64_t ns_consume;
    uint64_t calls_getinfo;
    uint64_t calls_consume;
    uint64_t wraps;
} read_telem_t;

static void ring_wait_and_get_one_block_fast(int fd, const void *ring, uint64_t ring_size,
                                             uint8_t *dst, read_telem_t *rt)
{
    int spins = WAIT_SPIN_ITERS;

    while (1)
    {
        uint64_t t0 = now_ns();
        struct csi_ring_info ri;
        if (ioctl(fd, CSI_IOC_GET_RING_INFO, &ri) < 0)
            die("CSI_IOC_GET_RING_INFO");
        uint64_t t1 = now_ns();

        rt->ns_getinfo += (t1 - t0);
        rt->calls_getinfo++;

        uint32_t used = ring_used_bytes(ri.head, ri.tail, ring_size);
        used -= (used % BYTES_PER_FRAME);

        uint32_t max_keep = (uint32_t)(MAX_QUEUED_BLOCKS * BLOCK_BYTES);
        if (used > max_keep)
        {
            uint32_t drop = used - max_keep;
            drop -= (drop % BYTES_PER_FRAME);
            if (drop)
            {
                uint64_t tc0 = now_ns();
                consume_bytes(fd, drop);
                uint64_t tc1 = now_ns();
                rt->ns_consume += (tc1 - tc0);
                rt->calls_consume++;
            }
            spins = WAIT_SPIN_ITERS;
            continue;
        }

        if (used >= BLOCK_BYTES)
        {
            uint32_t tail = ri.tail;

            uint32_t n1 = BLOCK_BYTES;
            if ((uint64_t)tail + (uint64_t)n1 > ring_size)
            {
                n1 = (uint32_t)ring_size - tail;
                rt->wraps++;
            }

            uint64_t tm0 = now_ns();
            memcpy(dst, (const uint8_t*)ring + tail, n1);
            uint32_t rem = BLOCK_BYTES - n1;
            if (rem) memcpy(dst + n1, (const uint8_t*)ring, rem);
            uint64_t tm1 = now_ns();
            rt->ns_copy += (tm1 - tm0);

            uint64_t tc0 = now_ns();
            consume_bytes(fd, BLOCK_BYTES);
            uint64_t tc1 = now_ns();
            rt->ns_consume += (tc1 - tc0);
            rt->calls_consume++;

            return;
        }

        if (spins-- > 0) cpu_relax();
        else usleep(WAIT_SLEEP_US);
    }
}

static void cs8_to_fftw_ch(const int8_t *src, int ch, fftwf_complex *dst)
{
    for (int n = 0; n < FFT_SIZE; ++n)
    {
        size_t base = (size_t)n * BYTES_PER_FRAME + (size_t)ch * BYTES_PER_IQ;
        dst[n][0] = (float)src[base + 0] / 127.0f;
        dst[n][1] = (float)src[base + 1] / 127.0f;
    }
}

// ------------------------------------------------------------
// JTAG LO programming
// ------------------------------------------------------------

static uint16_t max2851_lna_band_reg2_for_freq(double mhz)
{
    /* Main2: LNA_BAND[1:0] at D[6:5]; keep reserved bits at default. */
    if (mhz < 5200.0) return 0x180;  /* 00: 4.9?5.2 GHz */
    if (mhz < 5500.0) return 0x1A0;  /* 01: 5.2?5.5 GHz (default) */
    if (mhz < 5800.0) return 0x1C0;  /* 10: 5.5?5.8 GHz */
    return 0x1E0;                    /* 11: 5.8?5.9 GHz */
}

static int program_set_freq(int fd, double freq_mhz)
{
    double ratio = freq_mhz / 80.0;
    int idiv = (int)floor(ratio);
    int fdiv = (int)llround((ratio - idiv) * (double)(1u << 20));

    // The MAX2851 SPI is mapped through FPGA register 0x43
    struct csi_jtag_reg regs[4];
    
    regs[0].addr  = 0x43;
    regs[0].value = (uint16_t)((15u<<10) | (1u<<9) | (idiv & 0x7f));

    regs[1].addr  = 0x43;
    regs[1].value = (uint16_t)((16u<<10) | ((fdiv >> 10) & 0x3ff));

    regs[2].addr  = 0x43;
    regs[2].value = (uint16_t)((17u<<10) | (fdiv & 0x3ff));

    uint16_t reg2 = max2851_lna_band_reg2_for_freq(freq_mhz);
    regs[3].addr  = 0x43;
    regs[3].value = (uint16_t)((2u << 10) | (reg2 & 0x3FF));

    // Attempt the fast/atomic Batch IOCTL
    struct csi_jtag_batch batch;
    batch.regs_ptr = (uint64_t)(uintptr_t)regs;
    batch.count = 4;
    batch.delay_us = 0; // No delay needed between synthesizer words

    if (ioctl(fd, CSI_IOC_JTAG_BATCH_WRITE, &batch) == 0) {
        return 0; // Success via atomic batch!
    }

    // Graceful fallback for older kernels: send individually
    for (int i = 0; i < 4; i++) {
        if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &regs[i])) return -1;
    }

    return 0;
}

int jtag_write_u16(int fd, uint8_t addr, uint16_t value)
{
    struct csi_jtag_reg r;
    memset(&r, 0, sizeof(r));
    r.addr  = addr;
    r.value = value;
    
    uint64_t start_ns = now_ns();

    /* Retry if another process holds the JTAG lease, up to 1 second */
    while (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r) != 0) {
        if (errno == EBUSY) {
            if (now_ns() - start_ns > 1000000000ULL) {
                fprintf(stderr, "Error: JTAG write timeout (1s) for addr 0x%02X\n", addr);
                return -1;
            }
            usleep(1000); // Sleep 1ms and try again
        } else {
            return -1; // Fail on genuine hardware/driver errors
        }
    }
    return 0;
}

int jtag_read_u16(int fd, uint8_t addr, uint16_t *out_value)
{
    if (!out_value) {
        errno = EINVAL;
        return -1;
    }
    
    struct csi_jtag_reg r;
    memset(&r, 0, sizeof(r));
    r.addr = addr;
    
    uint64_t start_ns = now_ns();

    /* Retry if another process holds the JTAG lease, up to 1 second */
    while (ioctl(fd, CSI_IOC_JTAG_REG_READ, &r) != 0) {
        if (errno == EBUSY) {
            if (now_ns() - start_ns > 1000000000ULL) {
                fprintf(stderr, "Error: JTAG read timeout (1s) for addr 0x%02X\n", addr);
                return -1;
            }
            usleep(1000); // Sleep 1ms and try again
        } else {
            return -1; // Fail on genuine hardware/driver errors
        }
    }
    
    *out_value = r.value;
    return 0;
}

static int adjust_rx_gain(int fd, int adjust)
{
    struct csi_jtag_reg r;
    r.addr = 0x6A;

    jtag_read_u16(fd, r.addr, &r.value);
    r.value += adjust;

    if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r)) return -1;
    return 0;
}

static void jtag_cli(const char *args)
{
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "quadrf-jtag %s", args);
    int r = system(cmd);
    if (r != 0)
        fprintf(stderr, "Warning: jtag call returned %d: %s\n", r, cmd);
}

// Same --rx as the control panel's "4 ants interleaved": all four MAX2851
// paths on, FPGA packing four channels, no test tone, AGC off via manual gain.
static void rf_vision_lockdown_rx(void)
{
    jtag_cli("--rx antennas=15,interleave=1,tone_en=0,autosteer=0,gain=34,bw=16.0,pol=rhcp");
    jtag_cli("--tx off");
}

// ------------------------------------------------------------
// Telemetry structures (worker -> UI)
// ------------------------------------------------------------

typedef struct {
    uint64_t frame_idx;

    double t_read_ms;
    double t_lo_ms;
    double t_fft_ms;
    double t_proc_ms;
    double t_frame_ms;

    // Read breakdown
    double t_read_getinfo_ms;
    double t_read_copy_ms;
    double t_read_consume_ms;
    uint64_t read_calls_getinfo;
    uint64_t read_calls_consume;
    uint64_t read_wraps;

    uint32_t blocks;
    uint32_t points;

    float rf_activity;
    double fps;
    
    // --- DOA FIELDS ---
    float azimuth;
    float elevation;
    float centroid_u;
    float centroid_v;
    int has_lock_angles;
} telemetry_t;

// ------------------------------------------------------------
// State Machine Context
// ------------------------------------------------------------

typedef struct {
    int fd;
    void *ring;
    uint64_t ring_size;

    fftwf_complex *fft_in;
    fftwf_complex *fft_out[CHANNELS];
    fftwf_plan plan[CHANNELS];

    uint16_t *front, *back;
    pthread_mutex_t mtx;
    volatile int quit;

    int   mirror_display;
    float point_gain;

    // --- State Machine ---
    volatile int sweep_mode;
    volatile double target_freq; 
    volatile double locked_freq; 
    
    // Dynamically scale the top-K bins
    volatile float output_fraction;

    int headless;
    volatile int ws_clients;
    volatile int ws_ever;

    telemetry_t telem;
} ctx_t;

// ------------------------------------------------------------
// Top-K min-heap selection
// ------------------------------------------------------------

typedef struct {
    float v;
    int   k;
} heap_item_t;

static inline void heap_swap(heap_item_t *a, heap_item_t *b)
{
    heap_item_t t = *a; *a = *b; *b = t;
}

static inline void heap_sift_up(heap_item_t *h, int idx)
{
    while (idx > 0)
    {
        int p = (idx - 1) >> 1;
        if (h[p].v <= h[idx].v) break;
        heap_swap(&h[p], &h[idx]);
        idx = p;
    }
}

static inline void heap_sift_down(heap_item_t *h, int n, int idx)
{
    while (1)
    {
        int l = idx * 2 + 1;
        int r = l + 1;
        int m = idx;

        if (l < n && h[l].v < h[m].v) m = l;
        if (r < n && h[r].v < h[m].v) m = r;
        if (m == idx) break;
        heap_swap(&h[m], &h[idx]);
        idx = m;
    }
}

static inline int heap_push_topk(heap_item_t *h, int size, int cap, float v, int k)
{
    if (size < cap)
    {
        h[size].v = v;
        h[size].k = k;
        heap_sift_up(h, size);
        return size + 1;
    }
    if (v <= h[0].v) return size;
    h[0].v = v;
    h[0].k = k;
    heap_sift_down(h, cap, 0);
    return size;
}

// ------------------------------------------------------------
// Mongoose Web Stream
// ------------------------------------------------------------

#define AR_SETTINGS_PATH "/var/lib/quadrf/demos/ar_settings.json"
#define AR_WEB_ROOT      "/usr/share/quadrf/ar"
#define AR_SETTINGS_MAX  512

static char ar_settings_json[AR_SETTINGS_MAX];
static int  ar_settings_len = 0;

static void ar_settings_load_file(void)
{
    FILE *f = fopen(AR_SETTINGS_PATH, "r");
    if (!f) return;
    size_t n = fread(ar_settings_json, 1, sizeof(ar_settings_json) - 1, f);
    fclose(f);
    while (n > 0 && (ar_settings_json[n - 1] == '\n' || ar_settings_json[n - 1] == '\r'))
        n--;
    ar_settings_json[n] = '\0';
    ar_settings_len = (int)n;
}

static void ar_settings_save_file(const char *json, int len)
{
    if (len <= 0 || len >= AR_SETTINGS_MAX) return;
    memcpy(ar_settings_json, json, (size_t)len);
    ar_settings_json[len] = '\0';
    ar_settings_len = len;
    FILE *f = fopen(AR_SETTINGS_PATH, "w");
    if (!f) return;
    fwrite(ar_settings_json, 1, (size_t)ar_settings_len, f);
    fputc('\n', f);
    fclose(f);
}

static void ar_settings_send(struct mg_connection *c)
{
    if (ar_settings_len <= 0) return;
    char msg[AR_SETTINGS_MAX + 16];
    int n = snprintf(msg, sizeof(msg), "settings:%s", ar_settings_json);
    if (n > 0 && n < (int)sizeof(msg))
        mg_ws_send(c, msg, (size_t)n, WEBSOCKET_OP_TEXT);
}

// Handle incoming HTTP and WebSocket events
static void web_ev_handler(struct mg_connection *c, int ev, void *ev_data) {
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message *hm = (struct mg_http_message *) ev_data;
        if (mg_match(hm->uri, mg_str("/ws"), NULL)) {
            mg_ws_upgrade(c, hm, NULL);
            printf("[Web] Client connected to WS stream.\n");
            ar_settings_send(c);
        } else {
            struct mg_http_serve_opts opts = {.root_dir = AR_WEB_ROOT};
            mg_http_serve_dir(c, ev_data, &opts);
        }
    } else if (ev == MG_EV_WS_MSG) {
        struct mg_ws_message *wm = (struct mg_ws_message *) ev_data;
        ctx_t *ctx = (ctx_t *)c->fn_data;
        char buf[AR_SETTINGS_MAX + 16];

        int len = wm->data.len < sizeof(buf) - 1 ? (int)wm->data.len : (int)sizeof(buf) - 1;
        memcpy(buf, wm->data.buf, len);
        buf[len] = '\0';

        if (strncmp(buf, "mirror:", 7) == 0) {
            const char *arg = buf + 7;
            if (strcmp(arg, "toggle") == 0)
                ctx->mirror_display = !ctx->mirror_display;
            else if (arg[0] == '0' || arg[0] == '1')
                ctx->mirror_display = (arg[0] == '1');
            else
                ctx->mirror_display = !ctx->mirror_display;
            printf("[Web] Mirror display: %d\n", ctx->mirror_display);
        } else if (strncmp(buf, "gain:", 5) == 0) {
            int val = atoi(buf + 5);
            struct csi_jtag_reg r;
            r.addr = 0x6A;
            if (jtag_read_u16(ctx->fd, r.addr, &r.value) == 0) {
                r.value = (r.value & 0xFF80) | (val & 0x7F);
                ioctl(ctx->fd, CSI_IOC_JTAG_REG_WRITE, &r);
                printf("[Web] Hardware RX Gain set to: %d\n", val & 0x7F);
            }
        } else if (strncmp(buf, "settings:", 9) == 0) {
            if (strcmp(buf + 9, "get") == 0)
                ar_settings_send(c);
            else
                ar_settings_save_file(buf + 9, len - 9);
        } else if (strncmp(buf, "lock:", 5) == 0) {
            double target_freq = atof(buf + 5);
            ctx->target_freq = target_freq;
            ctx->sweep_mode = MODE_SEARCH;
            printf("[Web] Target frequency lock requested: %.2f MHz\n", target_freq);
        } else if (strncmp(buf, "unlock", 6) == 0) {
            ctx->sweep_mode = MODE_SWEEP;
            printf("[Web] Resumed LO sweep\n");
        }

    }
}

// Dedicated network thread
static void *web_thread(void *arg) {
    ctx_t *ctx = (ctx_t *)arg;
    struct mg_mgr mgr;
    struct timespec zero_since = {0, 0};
    mg_mgr_init(&mgr);
    
    // Listen on port 8000
    mg_http_listen(&mgr, "http://0.0.0.0:8000", web_ev_handler, ctx);
    printf("[Web] Listening on http://0.0.0.0:8000\n");

    while (!ctx->quit) {
        mg_mgr_poll(&mgr, 10); // 10ms timeout

       // Check if the DSP worker has a new frame ready
pthread_mutex_lock(&web_mtx);
if (web_frame_ready) {
    size_t payload_len = web_points_count * sizeof(rf_point_t);
    
    // Broadcast the binary buffer to all connected WebSocket clients (even if empty)
    for (struct mg_connection *c = mgr.conns; c != NULL; c = c->next) {
        if (c->is_websocket) {
            
            // --- ANTI-BUFFERBLOAT LOGIC ---
            // If payload_len is 0, check against a reasonable baseline (e.g., 1024)
            size_t limit = (payload_len > 0) ? payload_len : 1024;
            if (c->send.len > limit) {
                continue; 
            }
            
            mg_ws_send(c, web_points_buf, payload_len, WEBSOCKET_OP_BINARY);
        }
    }
    web_frame_ready = false;
}
pthread_mutex_unlock(&web_mtx);

        int nws = 0;
        for (struct mg_connection *c = mgr.conns; c != NULL; c = c->next) {
            if (c->is_websocket) nws++;
        }
        ctx->ws_clients = nws;
        if (nws > 0) ctx->ws_ever = 1;

        // Headless AR: last browser tab gone -> drop CSI and exit.
        // 2s grace so a refresh can reopen /ws. Do not exit if nobody
        // ever connected (Launch from the panel with a blocked popup).
        if (ctx->headless && ctx->ws_ever && ctx->ws_clients <= 0) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            if (zero_since.tv_sec == 0)
                zero_since = now;
            else if ((now.tv_sec - zero_since.tv_sec) * 1000L +
                     (now.tv_nsec - zero_since.tv_nsec) / 1000000L >= 2000) {
                printf("[Web] last AR client gone, stopping sweep\n");
                ctx->quit = 1;
                sig_quit = 1;
            }
        } else {
            zero_since.tv_sec = 0;
            zero_since.tv_nsec = 0;
        }
    }
    
    mg_mgr_free(&mgr);
    return NULL;
}

// ------------------------------------------------------------
// Worker thread
// ------------------------------------------------------------

// Helper function to drain the ring buffer completely
static void flush_ring_buffer(int fd, uint64_t ring_size) {
    struct csi_ring_info ri;
    if (ioctl(fd, CSI_IOC_GET_RING_INFO, &ri) == 0) {
        uint32_t used = ring_used_bytes(ri.head, ri.tail, ring_size);
        used -= (used % BYTES_PER_FRAME);
        if (used) consume_bytes(fd, used);
    }
}

static void *worker(void *arg)
{
    ctx_t *c = (ctx_t*)arg;

    uint8_t *blk = (uint8_t*)malloc(BLOCK_BYTES);
    if (!blk) die("malloc blk");

    // Move large arrays to heap to prevent stack overflow when FFT_SIZE is large
    float *Vraw = (float*)malloc(FFT_SIZE * sizeof(float));
    if (!Vraw) die("malloc Vraw");

    // Scale TOPK capacity dynamically
    int cfar_scale = (FFT_SIZE >= 4096) ? (FFT_SIZE / 4096) : 1;
    int base_topk_capacity = 512 * cfar_scale;
    
    heap_item_t *topk = (heap_item_t*)malloc(base_topk_capacity * sizeof(heap_item_t));
    if (!topk) die("malloc topk");

    const int half = FFT_SIZE / 2;
    const float TWO_PI = 2.0f * (float)M_PI;

    // Precompute sweep list (MHz)
    const int nsteps = (int)ceil((LO_END_MHZ - LO_START_MHZ) / LO_STEP_MHZ);
    double *lo_list = (double*)malloc((size_t)nsteps * sizeof(double));
    if (!lo_list) die("malloc lo_list");
    for (int i = 0; i < nsteps; ++i)
        lo_list[i] = LO_START_MHZ + (double)i * LO_STEP_MHZ;

    uint64_t frame_idx = 0;
    float vmax = 1e-9f;
    float vmax_next = 1e-9f;
    
    // Initialize pipeline
    double pipelined_lo = lo_list[0];
    program_set_freq(c->fd, pipelined_lo);

    // --- 100ms Integration Accumulators ---
    uint64_t last_doa_print_ns = now_ns();
    double sum_u_100ms = 0.0;
    double sum_v_100ms = 0.0;
    double sum_w_100ms = 0.0;
    
    // --- UI EMA State ---
    float ema_u = 0.0f;
    float ema_v = 0.0f;
    int ema_init = 0;

    while (!c->quit)
    {
        // ========================================================
        // Intercept: State Machine Transition (MULTI-STEP SEARCH)
        // ========================================================
        if (c->sweep_mode == MODE_SEARCH) {
            
            float global_best_v = -1.0f;
            int global_best_k = -1;
            double global_best_lo = c->target_freq;

            // Search 7 steps. Bias the search downwards (-4 to +2 steps) 
            // to account for the UI visually overshooting the true frequency 
            // due to pipeline lag during the fast sweep.
            #define SEARCH_STEPS 7
            double search_steps[SEARCH_STEPS];
            for (int s = 0; s < SEARCH_STEPS; s++) {
                search_steps[s] = c->target_freq + (s - 4) * LO_STEP_MHZ;
            }

            for (int s = 0; s < SEARCH_STEPS; s++) {
                double test_lo = search_steps[s];
                
                // Keep within valid hardware tuning bounds
                if (test_lo < LO_START_MHZ) test_lo = LO_START_MHZ;
                if (test_lo > LO_END_MHZ) test_lo = LO_END_MHZ;

                // 1. Program to the test frequency
                program_set_freq(c->fd, test_lo);

                // 2. Wait a LONG time (50ms). This guarantees the LO is fully settled
                // and the hardware FIFOs/USB pipelines are completely saturated 
                // with data exclusively from this new frequency.
                usleep(50000); 

                // 3. Nuke everything currently sitting in the host ring buffer
                flush_ring_buffer(c->fd, c->ring_size);

                // 4. Acquire ONE undeniably fresh block
                read_telem_t dummy_rt = {0};
                ring_wait_and_get_one_block_fast(c->fd, c->ring, c->ring_size, blk, &dummy_rt);

                // 5. Run FFTs
                for (int ch = 0; ch < CHANNELS; ++ch) {
                    cs8_to_fftw_ch((const int8_t*)blk, ch, c->fft_in);
                    fftwf_execute(c->plan[ch]);
                }

                // 6. Find the peak energy bin for this LO step
                for (int k = 0; k < FFT_SIZE; ++k) {
                    // Ignore the center 5 bins (DC Block)
                    if (k >= half - 2 && k <= half + 2) continue;

                    int i = (k + half) % FFT_SIZE;
                    float s0 = c->fft_out[0][i][0]*c->fft_out[0][i][0] + c->fft_out[0][i][1]*c->fft_out[0][i][1];
                    float s1 = c->fft_out[1][i][0]*c->fft_out[1][i][0] + c->fft_out[1][i][1]*c->fft_out[1][i][1];
                    float s2 = c->fft_out[2][i][0]*c->fft_out[2][i][0] + c->fft_out[2][i][1]*c->fft_out[2][i][1];
                    float s3 = c->fft_out[3][i][0]*c->fft_out[3][i][0] + c->fft_out[3][i][1]*c->fft_out[3][i][1];

                    float v = s0 + s1 + s2 + s3;
                    if (v > global_best_v) {
                        global_best_v = v;
                        global_best_k = k;
                        global_best_lo = test_lo; 
                    }
                }
            }

            // 7. Calculate refined true center frequency
            double offset_mhz = FS_MHZ * ((double)global_best_k - (double)half) / (double)FFT_SIZE;
            c->locked_freq = global_best_lo + offset_mhz;
            
            printf("-> Slow search complete. Peak found at base LO %.2f MHz | Offset: %.3f MHz | Refined Centered LO: %.2f MHz\n", 
                    global_best_lo, offset_mhz, c->locked_freq);

            // 8. Transition to LOCK mode
            c->sweep_mode = MODE_LOCK;
            pipelined_lo = c->locked_freq;
            program_set_freq(c->fd, pipelined_lo);

            // 9. Final wait and flush to ensure the locked frames start perfectly clean
            usleep(50000);
            flush_ring_buffer(c->fd, c->ring_size);
            
            // 10. Reset accumulators on lock
            sum_u_100ms = 0.0;
            sum_v_100ms = 0.0;
            sum_w_100ms = 0.0;
            last_doa_print_ns = now_ns();
            ema_init = 0;
            
            continue; 
        }
        // ========================================================
        // Standard Processing Pipeline 
        // ========================================================
        uint64_t t_frame0 = now_ns();
        uint64_t ns_read = 0, ns_lo = 0, ns_fft = 0, ns_proc = 0;
        uint32_t blocks = 0, points = 0;
        float activity_sum = 0.0f;
        read_telem_t rt = (read_telem_t){0};

        // --- DOA ACCUMULATORS ---
        float sum_u = 0.0f;
        float sum_v = 0.0f;
        float sum_w = 0.0f;

#if ENABLE_DECAY
        pthread_mutex_lock(&c->mtx);
        memcpy(c->back, c->front, (size_t)CANVAS_W * CANVAS_H * 3 * sizeof(uint16_t));
        pthread_mutex_unlock(&c->mtx);
        decay_accum(c->back);
#else
        memset(c->back, 0, (size_t)CANVAS_W * CANVAS_H * 3 * sizeof(uint16_t));
#endif
    
        vmax_next = 1e-9f;
        
        int iter_limit = (c->sweep_mode == MODE_LOCK) ? 1 : nsteps;

        rf_point_t local_points[MAX_WEB_POINTS];
        int local_count = 0;
        
        // Dynamically scale down the output target count
        int active_topk = (int)((float)base_topk_capacity * c->output_fraction);
        if (active_topk < 1) active_topk = 1;

        for (int idx = 0; idx < iter_limit && !c->quit && c->sweep_mode != MODE_SEARCH; ++idx)
        {
            double lo_curr = pipelined_lo;
            double lo_next = (c->sweep_mode == MODE_LOCK) ? c->locked_freq : 
                             ((idx + 1 < nsteps) ? lo_list[idx + 1] : lo_list[0]);

            int hsz = 0;

            // 1. Acquire Block
            uint64_t tr0 = now_ns();
            ring_wait_and_get_one_block_fast(c->fd, c->ring, c->ring_size, blk, &rt);
            ns_read += (now_ns() - tr0);
            blocks++;

            // Program next LO
            uint64_t tl0 = now_ns();
            (void)program_set_freq(c->fd, lo_next);
            pipelined_lo = lo_next; 
            ns_lo += (now_ns() - tl0);

            // 2. FFT (delay cal rotates ch1-3 in time domain before each FFT)
            uint64_t tf0 = now_ns();
            for (int ch = 0; ch < CHANNELS; ++ch) {
                cs8_to_fftw_ch((const int8_t*)blk, ch, c->fft_in);
                if (ch > 0) {
                    delay_cal_rotate_buf(c->fft_in, FFT_SIZE, ch - 1);
                }
                fftwf_execute(c->plan[ch]);
            }
            ns_fft += (now_ns() - tf0);

            // 3. Process
            uint64_t tp0 = now_ns();

            // Compute Vraw across all bins to run CFAR
            for (int k = 0; k < FFT_SIZE; ++k) {
                if (c->sweep_mode == MODE_LOCK && k >= half - 2 && k <= half + 2) {
                    Vraw[k] = -1e9f;
                    continue;
                }

                int i = (k + half) % FFT_SIZE;
                float re0 = c->fft_out[0][i][0], im0 = c->fft_out[0][i][1];
                float re1 = c->fft_out[1][i][0], im1 = c->fft_out[1][i][1];
                float re2 = c->fft_out[2][i][0], im2 = c->fft_out[2][i][1];
                float re3 = c->fft_out[3][i][0], im3 = c->fft_out[3][i][1];

                float s0 = re0*re0 + im0*im0;
                float s1 = re1*re1 + im1*im1;
                float s2 = re2*re2 + im2*im2;
                float s3 = re3*re3 + im3*im3;

                float v = logf(1.0f + (s0 + s1 + s2 + s3));
                if (k >= half - 2 && k <= half + 2) v = 0.0f;

                Vraw[k] = v;
                if (v > vmax_next) vmax_next = v;
            }

            // Run CFAR to find top targets (Dynamically Scaled)
            const int CFAR_WIN = 32 * cfar_scale;        
            const int CFAR_GUARD = 4 * cfar_scale;       
            const float CFAR_THRESH = 1.2f; 
            const int num_noise_cells = (CFAR_WIN - CFAR_GUARD) * 2;
            
            float window_sum = 0.0f;
            for (int k = 0; k < CFAR_WIN * 2 + 1; ++k) window_sum += Vraw[k];

            for (int k = CFAR_WIN; k < FFT_SIZE - CFAR_WIN; ++k) {
                if (k > CFAR_WIN) window_sum += Vraw[k + CFAR_WIN] - Vraw[k - CFAR_WIN - 1];

                float guard_sum = 0.0f;
                for (int g = -CFAR_GUARD; g <= CFAR_GUARD; ++g) guard_sum += Vraw[k + g];

                float noise_floor = (window_sum - guard_sum) / (float)num_noise_cells;

                if (Vraw[k] > noise_floor + CFAR_THRESH) {
                    hsz = heap_push_topk(topk, hsz, active_topk, Vraw[k], k);
                }
            }
            
            // Calculate Phase & DOA ONLY for the target bins 
            float scale_factor = SCALE_FACTOR_AT_MHZ((float)lo_curr);

            for (int t = 0; t < hsz; ++t) {
                int k = topk[t].k;
                int i = (k + half) % FFT_SIZE;

                float re0 = c->fft_out[0][i][0], im0 = c->fft_out[0][i][1];
                float re1 = c->fft_out[1][i][0], im1 = c->fft_out[1][i][1];
                float re2 = c->fft_out[2][i][0], im2 = c->fft_out[2][i][1];
                float re3 = c->fft_out[3][i][0], im3 = c->fft_out[3][i][1];

                float cre10 = re1*re0 + im1*im0; float cim10 = im1*re0 - re1*im0;
                float phi10 = atan2f(cim10, cre10);

                float cre23 = re2*re3 + im2*im3; float cim23 = im2*re3 - re2*im3;
                float phi23 = atan2f(cim23, cre23);

                float cre20 = re2*re0 + im2*im0; float cim20 = im2*re0 - re2*im0;
                float phi20 = atan2f(cim20, cre20);

                float cre30 = re3*re0 + im3*im0; float cim30 = im3*re0 - re3*im0;
                float phi30 = atan2f(cim30, cre30);

                float cre21 = re2*re1 + im2*im1; float cim21 = im2*re1 - re2*im1;
                float phi21 = atan2f(cim21, cre21);

                phi10 = circular_mean2_f(phi10, phi23);
                phi30 = circular_mean2_f(phi30, phi21);

                float best_gx = 0.0f, best_gy = 0.0f;
                {
                    const float inv_sqrt3 = (float)(1.0 / 1.7320508075688772);
                    const float reg = 1e-3f;
                    float best_cost = 1e30f;

                    for (int n10 = -2; n10 <= 2; ++n10)
                    for (int n20 = -2; n20 <= 2; ++n20)
                    for (int n30 = -2; n30 <= 2; ++n30)
                    {
                        float y1 = phi10 + (float)(2.0 * M_PI) * (float)n10;
                        float y2 = phi20 + (float)(2.0 * M_PI) * (float)n20;
                        float y3 = phi30 + (float)(2.0 * M_PI) * (float)n30;

                        float gx = (y3 - y1) * inv_sqrt3;
                        float gy = (y1 + 2.0f*y2 + y3) * (1.0f / 3.0f);

                        float p1 = -0.8660254037844386f * gx + 0.5f * gy;
                        float p2 = gy;
                        float p3 =  0.8660254037844386f * gx + 0.5f * gy;

                        float r1 = y1 - p1;
                        float r2 = y2 - p2;
                        float r3 = y3 - p3;
                        float cost = r1*r1 + r2*r2 + r3*r3 + reg * (gx*gx + gy*gy);
                        
                        if (cost < best_cost) {
                            best_cost = cost;
                            best_gx = gx;
                            best_gy = gy;
                        }
                    }
                }

                float u_dir = best_gx / scale_factor;
                float v_dir = best_gy / scale_factor;
                
                if (u_dir * u_dir + v_dir * v_dir > 1.0f) continue;
                
                float tx = u_dir * 0.5f + 0.5f;
                float ty = v_dir * 0.5f + 0.5f;

                if (c->mirror_display) tx = 1.0f - tx;
                if (tx < 0.0f) tx = 0.0f; else if (tx > 1.0f) tx = 1.0f;
                if (ty < 0.0f) ty = 0.0f; else if (ty > 1.0f) ty = 1.0f;

                int x = (int)(tx * (float)(CANVAS_W - 1));
                int y = (int)(ty * (float)(CANVAS_H - 1));
                y = (CANVAS_H - 1) - y;
                
                double rf = lo_curr + FS_MHZ * ((double)k - (double)half) / (double)FFT_SIZE;
                float hue = (float)((rf - FREQ_MIN_MHZ) / (FREQ_MAX_MHZ - FREQ_MIN_MHZ));
                hue = clampf(hue, 0.0f, 1.0f);

                float v = Vraw[k] / vmax;
                if (v < 0.0f) v = 0.0f;
                if (v > 1.0f) v = 1.0f;

                // --- WEIGHTED DOA ACCUMULATION (Quartic Weighting for Outlier Suppression) ---
                if (c->sweep_mode == MODE_LOCK) {
                    // Suppress outliers by aggressively weighting the brightest points
                    float weight = v * v * v * v;
                    
                    sum_u += u_dir * weight;
                    sum_v += v_dir * weight;
                    sum_w += weight;
                    
                    // Feed the persistent 100ms window
                    sum_u_100ms += u_dir * weight;
                    sum_v_100ms += v_dir * weight;
                    sum_w_100ms += weight;
                }

                if (local_count < MAX_WEB_POINTS) {
                    local_points[local_count].tx = tx;
                    local_points[local_count].ty = ty;
                    local_points[local_count].hue = hue;
                    local_points[local_count].strength = v; 
                    local_count++;
                }

                float r_base, g_base, b_base;
                hsv_to_rgb(hue, 1.0f, 1.0f, &r_base, &g_base, &b_base);

                float str = POINT_GAIN * v;
                stamp_pixel_add(c->back, x, y, r_base, g_base, b_base, str);

                points++;
                activity_sum += v;
            }
            vmax = vmax_next;
            ns_proc += (now_ns() - tp0);
        }

        // --- DOA AVERAGING (Frame Level with EMA for Smooth UI) ---
        if (c->sweep_mode == MODE_LOCK && sum_w > 1e-9f) {
            float u_avg = sum_u / sum_w;
            float v_avg = sum_v / sum_w;
            
            // Clamp to valid physical [-1.0, 1.0] domain
            if (u_avg > 1.0f) u_avg = 1.0f; else if (u_avg < -1.0f) u_avg = -1.0f;
            if (v_avg > 1.0f) v_avg = 1.0f; else if (v_avg < -1.0f) v_avg = -1.0f;
            
            if (!ema_init) {
                ema_u = u_avg;
                ema_v = v_avg;
                ema_init = 1;
            } else {
                float alpha = 0.15f; // EMA smoothing factor (lower is smoother)
                ema_u = ema_u * (1.0f - alpha) + u_avg * alpha;
                ema_v = ema_v * (1.0f - alpha) + v_avg * alpha;
            }

            c->telem.centroid_u = ema_u;
            c->telem.centroid_v = ema_v;
            
            // Boresight Mapping: Broadside (0,0) = 0? Az, 0? El
            c->telem.azimuth = asinf(ema_u) * (180.0f / (float)M_PI);
            c->telem.elevation = asinf(ema_v) * (180.0f / (float)M_PI);
            c->telem.has_lock_angles = 1;
        } else {
            c->telem.has_lock_angles = 0;
        }

        // Publish
        pthread_mutex_lock(&c->mtx);
        uint16_t *tmp = c->front;
        c->front = c->back;
        c->back  = tmp;

        uint64_t t_frame1 = now_ns();
        double t_frame_ms = ns_to_ms(t_frame1 - t_frame0);
        double fps = (t_frame_ms > 1e-9) ? (1000.0 / t_frame_ms) : 0.0;

        c->telem.frame_idx = frame_idx;
        c->telem.t_read_ms  = ns_to_ms(ns_read);
        c->telem.t_lo_ms    = ns_to_ms(ns_lo);
        c->telem.t_fft_ms   = ns_to_ms(ns_fft);
        c->telem.t_proc_ms  = ns_to_ms(ns_proc);
        c->telem.t_frame_ms = t_frame_ms;
        c->telem.blocks = blocks;
        c->telem.points = points;
        
        float denom = (float)iter_limit * (float)base_topk_capacity;
        float act = (denom > 1e-9f) ? (activity_sum / denom) : 0.0f;
        c->telem.rf_activity = clampf(act, 0.0f, 1.0f);
        c->telem.fps = fps;
        pthread_mutex_unlock(&c->mtx);

        // Push the completed frame to the web thread
        pthread_mutex_lock(&web_mtx);
        memcpy(web_points_buf, local_points, local_count * sizeof(rf_point_t));
        web_points_count = local_count;
        web_frame_ready = true;
        pthread_mutex_unlock(&web_mtx);

        frame_idx++;

        if ((frame_idx % TELEMETRY_PRINT_EVERY_N_FRAMES) == 0)
        {
            fprintf(stdout,
                "[frame %llu] total=%.1f ms (%.2f fps)  points=%u  rf=%.2f\n",
                (unsigned long long)frame_idx,
                c->telem.t_frame_ms, c->telem.fps,
                c->telem.points, c->telem.rf_activity);
            fflush(stdout);
        }

        // --- 10Hz FULL-WINDOW DOA OUTPUT ---
        if (c->sweep_mode == MODE_LOCK) {
            uint64_t current_ns = now_ns();
            if (current_ns - last_doa_print_ns >= 100000000ULL) { // 100ms interval
                if (sum_w_100ms > 1e-9) {
                    double u_avg = sum_u_100ms / sum_w_100ms;
                    double v_avg = sum_v_100ms / sum_w_100ms;
                    
                    // Clamp to valid physical [-1.0, 1.0] domain to prevent NaN from asin()
                    if (u_avg > 1.0) u_avg = 1.0; else if (u_avg < -1.0) u_avg = -1.0;
                    if (v_avg > 1.0) v_avg = 1.0; else if (v_avg < -1.0) v_avg = -1.0;
                    
                    // Boresight Mapping: Broadside (0,0) = 0? Az, 0? El
                    float az = asin(u_avg) * (180.0 / M_PI);
                    float el = asin(v_avg) * (180.0 / M_PI);
                    
                    fprintf(stdout, "  >>> [LOCK DOA] Freq: %.2f MHz | Azimuth: %6.1f? | Elevation: %5.1f?\n",
                            c->locked_freq, az, el);
                    fflush(stdout);
                }
                
                // Reset accumulators for the next 100ms window
                sum_u_100ms = 0.0;
                sum_v_100ms = 0.0;
                sum_w_100ms = 0.0;
                last_doa_print_ns = current_ns;
            }
        }
    }

    free(topk);
    free(Vraw);
    free(lo_list);
    free(blk);
    return NULL;
}

// ------------------------------------------------------------
// Camera underlay (Raspberry Pi 5 + OV5647)
// ------------------------------------------------------------
typedef struct {
    int enable;
    int w;
    int h;
    int fps;
    const char *cmd_override;
    FILE *pipe;
    pthread_t th;
    pthread_mutex_t mtx;
    volatile int quit;
    uint32_t *argb;
    size_t argb_bytes;
    volatile int have_frame;
} cam_t;

static inline uint8_t clip_u8(int x)
{
    if (x < 0) return 0;
    if (x > 255) return 255;
    return (uint8_t)x;
}

static void i420_to_argb(const uint8_t *yuv, int w, int h, uint32_t *dst_argb)
{
    const uint8_t *Y = yuv;
    const uint8_t *U = Y + (size_t)w * h;
    const uint8_t *V = U + (size_t)(w * h) / 4;

    for (int yy = 0; yy < h; ++yy)
    {
        const uint8_t *yrow = Y + (size_t)yy * w;
        const uint8_t *urow = U + (size_t)(yy / 2) * (w / 2);
        const uint8_t *vrow = V + (size_t)(yy / 2) * (w / 2);

        for (int xx = 0; xx < w; ++xx)
        {
            int y = (int)yrow[xx];
            int u = (int)urow[xx / 2] - 128;
            int v = (int)vrow[xx / 2] - 128;

            int c = y - 16;
            int d = u;
            int e = v;

            int r = (298 * c + 409 * e + 128) >> 8;
            int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            int b = (298 * c + 516 * d + 128) >> 8;

            uint32_t R = clip_u8(r);
            uint32_t G = clip_u8(g);
            uint32_t B = clip_u8(b);

            dst_argb[(size_t)yy * w + (size_t)xx] = 0xFF000000u | (R<<16) | (G<<8) | B;
        }
    }
}

static int fread_exact(FILE *f, void *buf, size_t n)
{
    uint8_t *p = (uint8_t*)buf;
    size_t got = 0;
    while (got < n)
    {
        size_t r = fread(p + got, 1, n - got, f);
        if (r == 0)
        {
            if (feof(f)) return -1;
            if (ferror(f)) return -1;
        }
        got += r;
    }
    return 0;
}

static void *camera_thread(void *arg)
{
    cam_t *cam = (cam_t*)arg;

    const size_t frame_bytes = (size_t)cam->w * cam->h * 3 / 2;
    uint8_t *yuv = (uint8_t*)malloc(frame_bytes);
    if (!yuv) return NULL;

    char cmd[512];
    if (cam->cmd_override && cam->cmd_override[0])
    {
        snprintf(cmd, sizeof(cmd), "%s", cam->cmd_override);
    }
    else
    {
        snprintf(cmd, sizeof(cmd),
                 "rpicam-vid -n --timeout 0 --width %d --height %d --framerate %d --codec yuv420 -o -",
                 cam->w, cam->h, cam->fps);
    }

    cam->pipe = popen(cmd, "r");
    if (!cam->pipe)
    {
        fprintf(stderr, "camera: popen failed for cmd: %s\n", cmd);
        free(yuv);
        return NULL;
    }

    while (!cam->quit)
    {
        if (fread_exact(cam->pipe, yuv, frame_bytes) < 0)
            break;

        pthread_mutex_lock(&cam->mtx);
        i420_to_argb(yuv, cam->w, cam->h, cam->argb);
        cam->have_frame = 1;
        pthread_mutex_unlock(&cam->mtx);
    }

    if (cam->pipe) pclose(cam->pipe);
    cam->pipe = NULL;
    free(yuv);
    return NULL;
}

static void camera_init_defaults(cam_t *cam)
{
    memset(cam, 0, sizeof(*cam));
    cam->enable = 0;
    cam->w = 640;
    cam->h = 480;
    cam->fps = 30;
}

static int camera_start(cam_t *cam)
{
    if (!cam->enable) return 0;

    pthread_mutex_init(&cam->mtx, NULL);

    cam->argb_bytes = (size_t)cam->w * cam->h * sizeof(uint32_t);
    cam->argb = (uint32_t*)malloc(cam->argb_bytes);
    if (!cam->argb)
    {
        fprintf(stderr, "camera: malloc argb failed\n");
        return -1;
    }
    memset(cam->argb, 0, cam->argb_bytes);

    cam->quit = 0;
    cam->have_frame = 0;

    if (pthread_create(&cam->th, NULL, camera_thread, cam) != 0)
    {
        fprintf(stderr, "camera: pthread_create failed\n");
        free(cam->argb);
        cam->argb = NULL;
        return -1;
    }
    return 0;
}

static void camera_stop(cam_t *cam)
{
    if (!cam->enable) return;

    cam->quit = 1;
    pthread_join(cam->th, NULL);

    pthread_mutex_destroy(&cam->mtx);

    free(cam->argb);
    cam->argb = NULL;
}

static void draw_ui_buttons(SDL_Renderer *ren, int win_w, int win_h, 
                            float current_gain, int is_swapped)
{
    int x = win_w - BTN_SIZE - BTN_MARGIN;
    int y_start = (win_h - (3 * BTN_SIZE + 2 * BTN_MARGIN)) / 2; // Vertically centered

    SDL_Rect rects[3];
    for(int i=0; i<3; i++) {
        rects[i].x = x;
        rects[i].y = y_start + i * (BTN_SIZE + BTN_MARGIN);
        rects[i].w = BTN_SIZE;
        rects[i].h = BTN_SIZE;
        
        // Background: Semi-transparent gray
        SDL_SetRenderDrawColor(ren, 50, 50, 50, 200);
        SDL_RenderFillRect(ren, &rects[i]);
        
        // Border: White
        SDL_SetRenderDrawColor(ren, 200, 200, 200, 255);
        SDL_RenderDrawRect(ren, &rects[i]);
    }

    // Draw Symbols (Simple lines)
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    int cx = x + BTN_SIZE/2;

    // 1. UP Arrow
    int y1 = rects[0].y + BTN_SIZE/2;
    SDL_RenderDrawLine(ren, cx, rects[0].y + 5, cx - 15, y1 + 5);
    SDL_RenderDrawLine(ren, cx, rects[0].y + 5, cx + 15, y1 + 5);

    // 2. DOWN Arrow
    int y2 = rects[1].y + BTN_SIZE/3;
    SDL_RenderDrawLine(ren, cx, rects[1].y + BTN_SIZE - 25, cx - 15, y2 - 5);
    SDL_RenderDrawLine(ren, cx, rects[1].y + BTN_SIZE - 25, cx + 15, y2 - 5);

    // 3. SWAP (Arrows <->)
    int y3 = rects[2].y + BTN_SIZE/2;
    SDL_RenderDrawLine(ren, x + 15, y3, x + BTN_SIZE - 15, y3); // Horizontal
    SDL_RenderDrawLine(ren, x + 15, y3, x + 25, y3 - 15);       // Left tip
    SDL_RenderDrawLine(ren, x + 15, y3, x + 25, y3 + 15);
    SDL_RenderDrawLine(ren, x + BTN_SIZE - 15, y3, x + BTN_SIZE - 25, y3 - 15); // Right tip
    SDL_RenderDrawLine(ren, x + BTN_SIZE - 15, y3, x + BTN_SIZE - 25, y3 + 15);
}

// ------------------------------------------------------------
// Main / SDL
// ------------------------------------------------------------

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s [--headless] [--camera] [--cam_w W] [--cam_h H] [--cam_fps FPS] [--cam_cmd 'CMD']\n"
        "  --headless         Run without opening SDL GUI (web streaming only)\n"
        "  --camera           Enable camera underlay via libcamera-vid\n"
        "  --cam_w/--cam_h    Camera capture resolution (default 640x480)\n"
        "  --cam_fps          Camera framerate (default 30)\n"
        "  --cam_cmd          Override capture command (must output raw I420 frames)\n",
        argv0);
}

int main(int argc, char **argv)
{
    cam_t cam;
    camera_init_defaults(&cam);
    int headless = 0;

    for (int ai = 1; ai < argc; ++ai)
    {
        if (!strcmp(argv[ai], "--headless"))
        {
            headless = 1;
        }
        else if (!strcmp(argv[ai], "--camera"))
        {
            cam.enable = 1;
        }
        else if (!strcmp(argv[ai], "--cam_w") && ai + 1 < argc)
        {
            cam.w = atoi(argv[++ai]);
        }
        else if (!strcmp(argv[ai], "--cam_h") && ai + 1 < argc)
        {
            cam.h = atoi(argv[++ai]);
        }
        else if (!strcmp(argv[ai], "--cam_fps") && ai + 1 < argc)
        {
            cam.fps = atoi(argv[++ai]);
        }
        else if (!strcmp(argv[ai], "--cam_cmd") && ai + 1 < argc)
        {
            cam.cmd_override = argv[++ai];
            cam.enable = 1;
        }
        else if (!strcmp(argv[ai], "--help") || !strcmp(argv[ai], "-h"))
        {
            usage(argv[0]);
            return 0;
        }
        else
        {
            fprintf(stderr, "Unknown arg: %s\n", argv[ai]);
            usage(argv[0]);
            return 1;
        }
    }

    // Bind interrupt signals for graceful headless exit
    signal(SIGINT, handle_sig);
    signal(SIGTERM, handle_sig);

    int fd = open(DEVICE_PATH, O_RDWR | O_NONBLOCK);
    if (fd < 0) die("open");

    ioctl(fd, CSI_IOC_JTAG_SETUP);

    rf_vision_lockdown_rx();

    struct csi_ring_info ri;
    if (ioctl(fd, CSI_IOC_GET_RING_INFO, &ri) < 0) die("CSI_IOC_GET_RING_INFO");
    if (!ri.ring_size) { fprintf(stderr, "ring_size=0\n"); return 1; }

    long page = sysconf(_SC_PAGESIZE);
    size_t map_len = (size_t)((ri.ring_size + page - 1) & ~((uint64_t)page - 1));

    void *ring = mmap(NULL, map_len, PROT_READ, MAP_SHARED, fd, 0);
    if (ring == MAP_FAILED) die("mmap");

    SDL_Window *win = NULL;
    SDL_Renderer *ren = NULL;
    SDL_Texture *cam_tex = NULL;
    SDL_Texture *tex = NULL;

    if (!headless) {
        if (SDL_Init(SDL_INIT_VIDEO) != 0) {
            fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
            return 1;
        }

        // Nearest-neighbor scaling: makes CANVAS pixels appear as big blocks in the window.
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");

        SDL_Rect bounds;
        if (SDL_GetDisplayUsableBounds(0, &bounds) != 0) {
            // Fallback just in case the display query fails
            bounds.w = WIN_WIDTH; 
            bounds.h = WIN_HEIGHT;
        }

        win = SDL_CreateWindow("Telemetry: starting...",
            SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
            bounds.w/2.0, bounds.h/2.0, SDL_WINDOW_RESIZABLE);
        if (!win) die("SDL_CreateWindow");

        ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
        if (!ren) die("SDL_CreateRenderer");

        /* Camera underlay texture (optional) */
        if (cam.enable) {
            if (camera_start(&cam) != 0) {
                fprintf(stderr, "camera: failed to start; continuing without camera underlay\n");
                cam.enable = 0;
            } else {
                cam_tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
                    SDL_TEXTUREACCESS_STREAMING, cam.w, cam.h);
                if (!cam_tex) {
                    fprintf(stderr, "camera: SDL_CreateTexture failed: %s\n", SDL_GetError());
                    camera_stop(&cam);
                    cam.enable = 0;
                }
            }
        }

        // Texture is logical canvas size; we scale it to the window each frame.
        tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
            SDL_TEXTUREACCESS_STREAMING, CANVAS_W, CANVAS_H);
        if (!tex) die("SDL_CreateTexture");
        SDL_SetTextureBlendMode(tex, SDL_BLENDMODE_ADD); /* additive over camera underlay */
    } else {
        printf("[Headless] Mode enabled. Suppressing SDL GUI.\n");
        if (cam.enable && camera_start(&cam) != 0) {
            cam.enable = 0;
        }
    }

    uint16_t *front = (uint16_t*)calloc((size_t)CANVAS_W * CANVAS_H * 3, sizeof(uint16_t));
    uint16_t *back  = (uint16_t*)calloc((size_t)CANVAS_W * CANVAS_H * 3, sizeof(uint16_t));
    uint32_t *pix   = (uint32_t*)malloc((size_t)CANVAS_W * CANVAS_H * sizeof(uint32_t));
    if (!front || !back || !pix) die("alloc buffers");

    fftwf_complex *in = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
    if (!in) die("fftwf_malloc in");

    fftwf_complex *out[CHANNELS];
    fftwf_plan plan[CHANNELS];
    for (int ch = 0; ch < CHANNELS; ++ch)
    {
        out[ch] = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
        if (!out[ch]) die("fftwf_malloc out");
        plan[ch] = fftwf_plan_dft_1d(FFT_SIZE, in, out[ch], FFTW_FORWARD, FFTW_ESTIMATE /*FFTW_MEASURE*/);
        if (!plan[ch]) die("fftw plan");
    }

    ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.fd = fd;
    ctx.ring = ring;
    ctx.ring_size = ri.ring_size;
    ctx.fft_in = in;
    for (int ch = 0; ch < CHANNELS; ++ch) { ctx.fft_out[ch] = out[ch]; ctx.plan[ch] = plan[ch]; }
    ctx.front = front;
    ctx.back  = back;
    pthread_mutex_init(&ctx.mtx, NULL);

    ctx.mirror_display = 1;
    ctx.sweep_mode = MODE_SWEEP;
    ctx.output_fraction = 0.25f; // 1.0: output 100% of top bins
    ctx.headless = headless;

    ar_settings_load_file();

    // Drop samples captured before --rx took effect.
    flush_ring_buffer(fd, ri.ring_size);

    // main worker thread
    pthread_t th;
    if (pthread_create(&th, NULL, worker, &ctx) != 0) die("pthread_create");

    // web streaming thread
    pthread_t th_web;
    if (pthread_create(&th_web, NULL, web_thread, &ctx) != 0) die("pthread_create web");

    bool quit = false;
    telemetry_t telem_last = (telemetry_t){0};

    float rf_act_ema = 0.0f;
    float cam_gain_f = 200.0f; /* 0..255 */

    const float CAM_GAIN_BASE = 200.0f;  /* baseline brightness (try 160..220) */
    const float CAM_GAIN_MIN  = 70.0f;   /* minimum brightness under heavy RF */
    const float CAM_EMA_ALPHA = 0.10f;   /* RF activity smoothing (0.05..0.2) */
    const float GAIN_ALPHA    = 0.15f;   /* camera gain smoothing */
    const float FADE_GAMMA    = 1.2f;    /* nonlinear fade response */

    SDL_Rect dst = {0, 0, WIN_WIDTH, WIN_HEIGHT};

    while (!quit && !sig_quit && !ctx.quit)
    {
        if (!headless) {
            int win_w, win_h;
            SDL_GetRendererOutputSize(ren, &win_w, &win_h); // Get size for button placement

            SDL_Event e;
            while (SDL_PollEvent(&e))
            {
                if (e.type == SDL_QUIT) quit = true;
                if (e.type == SDL_KEYDOWN) {
                    if (e.key.keysym.sym == SDLK_ESCAPE) quit = true;
                    
                    // Fraction scaling
                    if (e.key.keysym.sym == SDLK_UP) {
                        ctx.output_fraction += 0.05f;
                        if (ctx.output_fraction > 1.0f) ctx.output_fraction = 1.0f;
                        printf("-> Output Fraction: %.0f%%\n", ctx.output_fraction * 100.0f);
                    }
                    if (e.key.keysym.sym == SDLK_DOWN) {
                        ctx.output_fraction -= 0.05f;
                        if (ctx.output_fraction < 0.01f) ctx.output_fraction = 0.01f;
                        printf("-> Output Fraction: %.0f%%\n", ctx.output_fraction * 100.0f);
                    }
                }

                // --- MOUSE CLICKS ---
                if (e.type == SDL_MOUSEBUTTONDOWN) {
                    int mx = e.button.x;
                    int my = e.button.y;
                    
                    int bx = win_w - BTN_SIZE - BTN_MARGIN;
                    int by_start = (win_h - (3 * BTN_SIZE + 2 * BTN_MARGIN)) / 2;
                    
                    // LEFT CLICK
                    if (e.button.button == SDL_BUTTON_LEFT) {
                        // 1. Check if user clicked the UI buttons on the right
                        if (mx >= bx && mx <= bx + BTN_SIZE && my >= by_start && my <= by_start + 3*BTN_SIZE + 2*BTN_MARGIN) {
                            if (my >= by_start && my <= by_start + BTN_SIZE) {
                                adjust_rx_gain(fd, +2);
                            }
                            else if (my >= by_start + BTN_SIZE + BTN_MARGIN && my <= by_start + 2*BTN_SIZE + BTN_MARGIN) {
                                adjust_rx_gain(fd, -2);
                            }
                            else if (my >= by_start + 2*(BTN_SIZE + BTN_MARGIN) && my <= by_start + 3*BTN_SIZE + 2*BTN_MARGIN) {
                                ctx.mirror_display = !ctx.mirror_display;
                                printf("Swap: %d\n", ctx.mirror_display);
                            }
                        } 
                        // 2. Otherwise, user clicked the canvas area
                        else {
                            // Reverse the window cropping coordinates back to the logical CANVAS coordinates
                            int cur_w, cur_h;
                            SDL_GetRendererOutputSize(ren, &cur_w, &cur_h);
                            
                            int src_w = CANVAS_W;
                            int src_h = 200; 
                            int src_x = (CANVAS_W - src_w) / 2; 
                            int src_y = (CANVAS_H - src_h) / 2; 

                            int cx = src_x + (mx * src_w) / cur_w;
                            int cy = src_y + (my * src_h) / cur_h;

                            // Sample an 11x11 neighborhood around the click for average color
                            int radius = 5; 
                            float sum_r = 0, sum_g = 0, sum_b = 0;
                            int count = 0;

                            for (int dy = -radius; dy <= radius; dy++) {
                                for (int dx = -radius; dx <= radius; dx++) {
                                    int px = cx + dx;
                                    int py = cy + dy;
                                    if (px >= 0 && px < CANVAS_W && py >= 0 && py < CANVAS_H) {
                                        uint32_t c_val = pix[py * CANVAS_W + px];
                                        uint8_t a = (c_val >> 24) & 0xFF;
                                        
                                        // Only sample pixels that have actually been stamped (alpha > 0)
                                        if (a > 0) { 
                                            sum_r += (c_val >> 16) & 0xFF;
                                            sum_g += (c_val >> 8) & 0xFF;
                                            sum_b += c_val & 0xFF;
                                            count++;
                                        }
                                    }
                                }
                            }
                            
                            // If we found stamped points, trigger the SEARCH mode
                            if (count > 0) {
                                float avg_r = sum_r / count / 255.0f;
                                float avg_g = sum_g / count / 255.0f;
                                float avg_b = sum_b / count / 255.0f;
                                
                                float h = rgb_to_hue(avg_r, avg_g, avg_b);
                                
                                // The raw visual frequency on the screen
                                double visual_freq = FREQ_MIN_MHZ + h * (FREQ_MAX_MHZ - FREQ_MIN_MHZ);
                                
                                // 1. Provide the ballpark estimate compensating for 1-step lag
                                ctx.target_freq = visual_freq - LO_STEP_MHZ;
                                
                                // 2. Hand off to the worker thread to flush, search, and lock
                                ctx.sweep_mode = MODE_SEARCH;
                                printf("-> Initiating Peak Search near %.2f MHz\n", ctx.target_freq);
                            }
                        }
                    } 
                    // RIGHT CLICK -> Resume Sweeping
                    else if (e.button.button == SDL_BUTTON_RIGHT) {
                        ctx.sweep_mode = MODE_SWEEP;
                        printf("-> Resumed LO sweep\n");
                    }
                }
            }

            // --- Dynamic sizing ---
            int cur_w, cur_h;
            SDL_GetRendererOutputSize(ren, &cur_w, &cur_h);
            dst.w = cur_w; dst.h = cur_h;
            // ---------------------------
        } else {
            // Keep CPU usage low in headless mode when skipping the SDL event loop
            usleep(20000); 
        }

        pthread_mutex_lock(&ctx.mtx);
        accum_to_pixels(ctx.front, pix);
        telem_last = ctx.telem;
        pthread_mutex_unlock(&ctx.mtx);

        /* Update RF activity EMA and compute a dimming gain for the camera.
         * telem_last.rf_activity is already normalized to [0,1].
         */
        rf_act_ema = (1.0f - CAM_EMA_ALPHA) * rf_act_ema + CAM_EMA_ALPHA * telem_last.rf_activity;
        rf_act_ema = clampf(rf_act_ema, 0.0f, 1.0f);

        float fade = powf(rf_act_ema, FADE_GAMMA);
        float target_gain = CAM_GAIN_BASE - (CAM_GAIN_BASE - CAM_GAIN_MIN) * fade;
        target_gain = clampf(target_gain, CAM_GAIN_MIN, 255.0f);
        cam_gain_f += GAIN_ALPHA * (target_gain - cam_gain_f);
        cam_gain_f = clampf(cam_gain_f, CAM_GAIN_MIN, 255.0f);

        if (!headless) {
            char title[256];
            snprintf(title, sizeof(title),
                     "fps=%.2f total=%.0fms read=%.0f LO=%.0f FFT=%.0f proc=%.0f points=%u rf=%.2f frac=%.2f",
                     telem_last.fps,
                     telem_last.t_frame_ms,
                     telem_last.t_read_ms,
                     telem_last.t_lo_ms,
                     telem_last.t_fft_ms,
                     telem_last.t_proc_ms,
                     telem_last.points,
                     telem_last.rf_activity,
                     ctx.output_fraction);
            SDL_SetWindowTitle(win, title);

            SDL_UpdateTexture(tex, NULL, pix, CANVAS_W * (int)sizeof(uint32_t));
            SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
            SDL_RenderClear(ren);

            /* Underlay: camera */
            if (cam.enable && cam_tex)
            {
                int have = 0;
                pthread_mutex_lock(&cam.mtx);
                have = cam.have_frame;
                if (have)
                    SDL_UpdateTexture(cam_tex, NULL, cam.argb, cam.w * (int)sizeof(uint32_t));
                pthread_mutex_unlock(&cam.mtx);

                if (have)
                {
                    /* Dim camera dynamically based on RF activity (scatter visibility). */
                    uint8_t g = (uint8_t)cam_gain_f;
                    SDL_SetTextureColorMod(cam_tex, g, g, g);
                    SDL_RenderCopy(ren, cam_tex, NULL, &dst);
                }
            }

            SDL_Rect src;
            src.h = 200;           // The "points tall" user requested
            src.w = CANVAS_W;           // Keep aspect ratio 1:1 (square crop)
            src.x = (CANVAS_W - src.w) / 2; // Center horizontally
            src.y = (CANVAS_H - src.h) / 2; // Center vertically

            // 2. Render specifically this crop to the full window (dst)
            SDL_RenderCopy(ren, tex, &src, &dst);

            /* Overlay: Crosshair for DOA */
            if (telem_last.has_lock_angles) {
                float tx = telem_last.centroid_u * 0.5f + 0.5f;
                float ty = telem_last.centroid_v * 0.5f + 0.5f;
                if (ctx.mirror_display) tx = 1.0f - tx;
                
                float canvas_x = tx * (CANVAS_W - 1);
                float canvas_y = ty * (CANVAS_H - 1);
                canvas_y = (CANVAS_H - 1) - canvas_y; // Invert Y exactly like stamp logic
                
                // Project Canvas coordinates directly to Window coordinates
                float screen_x = ((canvas_x - src.x) / (float)src.w) * dst.w + dst.x;
                float screen_y = ((canvas_y - src.y) / (float)src.h) * dst.h + dst.y;
                
                int cx = (int)screen_x;
                int cy = (int)screen_y;
                int r = 20; // Size of crosshair
                
                // Only draw if roughly within screen bounds
                if (cx >= 0 && cx <= dst.w && cy >= 0 && cy <= dst.h) {
                    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
                    
                    // Draw bold black outline for visibility
                    SDL_SetRenderDrawColor(ren, 0, 0, 0, 180);
                    SDL_RenderDrawLine(ren, cx - r, cy - 1, cx + r, cy - 1);
                    SDL_RenderDrawLine(ren, cx - r, cy + 1, cx + r, cy + 1);
                    SDL_RenderDrawLine(ren, cx - 1, cy - r, cx - 1, cy + r);
                    SDL_RenderDrawLine(ren, cx + 1, cy - r, cx + 1, cy + r);
                    
                    // Draw pure white inner core
                    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
                    SDL_RenderDrawLine(ren, cx - r, cy, cx + r, cy);
                    SDL_RenderDrawLine(ren, cx, cy - r, cx, cy + r);
                }
            }

            /* Overlay: UI Buttons */
            draw_ui_buttons(ren, dst.w, dst.h, ctx.point_gain, ctx.mirror_display);
            SDL_RenderPresent(ren);

            SDL_Delay(10);
        }
    }

    ctx.quit = 1;
    pthread_join(th, NULL);
    pthread_join(th_web, NULL);

    pthread_mutex_destroy(&ctx.mtx);

    for (int ch = 0; ch < CHANNELS; ++ch)
    {
        fftwf_destroy_plan(plan[ch]);
        fftwf_free(out[ch]);
    }
    fftwf_free(in);

    free(front);
    free(back);
    
    if (!headless) {
        if (cam_tex) SDL_DestroyTexture(cam_tex);
        free(pix);
        SDL_DestroyTexture(tex);
        SDL_DestroyRenderer(ren);
        SDL_DestroyWindow(win);
        SDL_Quit();
    } else {
        free(pix);
    }
    
    camera_stop(&cam);
    ioctl(fd, CSI_IOC_JTAG_RELEASE);
    munmap(ring, map_len);
    close(fd);

    return 0;
}