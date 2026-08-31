// nearfield.c
//
// 4x4 MIMO coherent near-field phasor (digital down-converter).
// Cycles the four TX antennas in TDM, flushes analog switching transients,
// and plots bistatic RX/TX phasors. Monostatic paths (rx == tx) are blanked
// to avoid LNA saturation and skip 25% of the DDC inner loop.
//
// Build:
//   gcc nearfield.c -O3 -o quadrf-nearfield -lSDL2 -lm -lpthread
//
// Run:
//   ./quadrf-nearfield

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <SDL2/SDL.h>

#include "fpga_csi.h"

#define MAX2850_REG_ADDR 0x42
#define MAX2851_REG_ADDR 0x43
#define RF_FREQ_MHZ      5800.0
#define RX_GAIN_WORD     30
#define TX_GAIN_WORD     20 

// Config 
#define RX_DEVICE "/dev/csi_stream0"
#define TX_DEVICE "/dev/dsi_stream0"
#define TONE_FREQ_HZ 1000000.0
#define NUM_CHANNELS 4
#define BYTES_PER_FRAME 8 

// Exact hardware native rates (DDS phase-step only; devices run at line rate).
// TX: 2-lane DSI, 175 MHz byte clock, 3072x1080 payload / (payload + 42 B/line + 9342).
// RX: 4-lane CSI, 175 MHz byte clock; /4 for 4-channel interleaved CS8.
#define EXACT_TX_RATE (0.5 * 175000000.0 * (3317760.0 / 3372462.0))
#define EXACT_RX_RATE (((65536.0 / 76714.0) * 175000000.0) / 4.0)
#define TX_PAYLOAD_BYTES 3317760

// DSP State Machine Settings
#define FLUSH_SAMPLES       8192         // ~0.22 ms analog settling at 37.4 Msps
#define INTEGRATION_SAMPLES (65536 * 4)  // ~7.0 ms integration (~35 Hz 4-TX cycle)
#define EMA_ALPHA           0.2f  

#define WIN_W 800
#define WIN_H 800

// Thread Synchronization
static volatile bool g_running = true;
static volatile bool g_radio_ready = false;
static pthread_mutex_t g_phasor_mutex = PTHREAD_MUTEX_INITIALIZER;
static float g_phasor_i[NUM_CHANNELS][NUM_CHANNELS] = {0}; // [RX][TX]
static float g_phasor_q[NUM_CHANNELS][NUM_CHANNELS] = {0};

// --- Fast DDS Look-Up Table (LUT) ---
#define LUT_BITS 16
#define LUT_SIZE (1 << LUT_BITS)
static double cos_lut[LUT_SIZE];
static double sin_lut[LUT_SIZE];

void init_luts() {
    for (int i = 0; i < LUT_SIZE; i++) {
        double angle = (2.0 * M_PI * i) / LUT_SIZE;
        cos_lut[i] = cos(angle);
        sin_lut[i] = sin(angle);
    }
}

static int jtag_write_u16(int fd, uint8_t addr, uint16_t value) {
    struct csi_jtag_reg r = { .addr = addr, .value = value };
    for (int i = 0; i < 100; i++) {
        if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r) == 0) return 0;
        if (errno == EINTR) continue;
        if (errno != EBUSY) break;
        usleep(1000);
    }
    return -1;
}

static int jtag_read_u16(int fd, uint8_t addr, uint16_t *out_value) {
    if (!out_value) return -1;
    struct csi_jtag_reg r = { .addr = addr, .value = 0 };
    for (int i = 0; i < 100; i++) {
        if (ioctl(fd, CSI_IOC_JTAG_REG_READ, &r) == 0) {
            *out_value = r.value;
            return 0;
        }
        if (errno == EINTR) continue;
        if (errno != EBUSY) break;
        usleep(1000);
    }
    return -1;
}

static int jtag_acquire_lease(int fd) {
    for (;;) {
        if (ioctl(fd, CSI_IOC_JTAG_ACQUIRE_LEASE) == 0) return 0;
        if (errno == ENOTTY) return 0;
        if (errno != EBUSY) return -1;
        usleep(1000);
    }
}

static int spi_word(int fd, uint8_t fpga_addr, uint16_t main_reg, uint16_t data10) {
    uint16_t w = (uint16_t)(((main_reg & 0x3Fu) << 10) | (data10 & 0x3FFu));
    return jtag_write_u16(fd, fpga_addr, w);
}

static int set_lo_mhz(int fd, uint8_t spi_addr, double mhz) {
    double ratio = mhz / 80.0;
    long long idiv = (long long)floor(ratio);
    long long fdiv = llround((ratio - (double)idiv) * (double)(1u << 20));
    if (fdiv == (1LL << 20)) {
        idiv++;
        fdiv = 0;
    }
    uint16_t w15 = (uint16_t)((15u << 10) | (1u << 9) | ((unsigned)idiv & 0x7Fu));
    uint16_t w16 = (uint16_t)((16u << 10) | (((unsigned)fdiv >> 10) & 0x3FFu));
    uint16_t w17 = (uint16_t)((17u << 10) | ((unsigned)fdiv & 0x3FFu));
    if (jtag_write_u16(fd, spi_addr, w15) < 0) return -1;
    if (jtag_write_u16(fd, spi_addr, w16) < 0) return -1;
    if (jtag_write_u16(fd, spi_addr, w17) < 0) return -1;
    // MAX2851 Main2 LNA_BAND[1:0] at D[6:5]; 11 = 5.8-5.9 GHz
    if (spi_addr == MAX2851_REG_ADDR)
        return spi_word(fd, spi_addr, 2, 0x1E0);
    return 0;
}

// MAX2850 Main0: MODE=TX (011), 40 MHz BW, one E_TX bit. FPGA 0x02 is the DSI channel mask.
static void switch_tx_antenna(int fd, int tx_idx) {
    uint16_t mask = (uint16_t)(1u << tx_idx);
    uint16_t reg0 = 0x00E | (uint16_t)(mask << 5);
    if (spi_word(fd, MAX2850_REG_ADDR, 0, reg0) < 0)
        perror("[!] MAX2850 SPI Write Failed");
    if (jtag_write_u16(fd, 0x02, mask) < 0)
        perror("[!] FPGA Mask Write Failed");
}

// 4-channel interleave, analog TX, both LOs at RF_FREQ_MHZ. Hold the JTAG lease
// for the whole run so TDM antenna switches are not interrupted.
static int setup_radio(int fd) {
    int failed = 0;

    if (ioctl(fd, CSI_IOC_JTAG_SETUP) != 0)
        perror("Warning: CSI_IOC_JTAG_SETUP");

    if (jtag_write_u16(fd, 0x25, 0x0001) < 0) {
        perror("[!] FPGA interleave enable failed");
        failed++;
    }
    uint16_t val_2e = 0;
    if (jtag_read_u16(fd, 0x2E, &val_2e) == 0) {
        val_2e &= (uint16_t)~0x0002;
        jtag_write_u16(fd, 0x2E, val_2e);
    }
    // Digital filter k=6 -> 40 MHz. FPGA TX test-tone off (IQ comes from DSI).
    jtag_write_u16(fd, 0x27, 6);
    uint16_t val_26 = 0;
    if (jtag_read_u16(fd, 0x26, &val_26) == 0)
        jtag_write_u16(fd, 0x26, (uint16_t)(val_26 & ~0x0001));

    // MAX2851: all four RX channels, MODE=RX, 40 MHz analog, LO + LNA band
    if (spi_word(fd, MAX2851_REG_ADDR, 6, 0x3FF) < 0) {
        perror("[!] MAX2851 RX enable failed");
        failed++;
    }
    if (spi_word(fd, MAX2851_REG_ADDR, 0, 0x00A) < 0) {
        perror("[!] MAX2851 MODE=RX failed");
        failed++;
    }
    if (set_lo_mhz(fd, MAX2851_REG_ADDR, RF_FREQ_MHZ) < 0) {
        perror("[!] MAX2851 LO failed");
        failed++;
    }
    if (jtag_write_u16(fd, 0x6A, RX_GAIN_WORD) < 0)
        perror("[!] FPGA RX gain failed");

    // Analog TX path, PA bias, TX LO follows RX, LO + gain
    if (jtag_write_u16(fd, 0x23, 0x0000) < 0) {
        perror("[!] FPGA TX path enable failed");
        failed++;
    }
    if (spi_word(fd, MAX2850_REG_ADDR, 10, 0x0001) < 0)
        perror("[!] MAX2850 PA bias failed");
    jtag_write_u16(fd, 0x6D, 0x0001);
    if (set_lo_mhz(fd, MAX2850_REG_ADDR, RF_FREQ_MHZ) < 0) {
        perror("[!] MAX2850 LO failed");
        failed++;
    }
    if (spi_word(fd, MAX2850_REG_ADDR, 9, (uint16_t)((TX_GAIN_WORD << 4) | 0xFu)) < 0)
        perror("[!] MAX2850 TX gain failed");

    // Polarization RHCP, 4-ch interleave last so later SPI traffic cannot clobber 0x25
    jtag_write_u16(fd, 0x24, 0x0001);
    if (jtag_write_u16(fd, 0x25, 0x0001) < 0)
        perror("[!] FPGA interleave restrobe failed");

    switch_tx_antenna(fd, 0);
    // VAS + PLL settle, then force DOUT = lock detect before sampling it
    spi_word(fd, MAX2851_REG_ADDR, 14, 0x160);
    spi_word(fd, MAX2850_REG_ADDR, 14, 0x160);
    usleep(20000);

    uint16_t r25 = 0, r23 = 0, r6a = 0, rx_dout = 0, tx_dout = 0;
    jtag_read_u16(fd, 0x25, &r25);
    jtag_read_u16(fd, 0x23, &r23);
    jtag_read_u16(fd, 0x6A, &r6a);
    jtag_read_u16(fd, MAX2851_REG_ADDR, &rx_dout);
    jtag_read_u16(fd, MAX2850_REG_ADDR, &tx_dout);

    fprintf(stderr,
            "radio: %.0f MHz, 0x25=%u 0x23=%u 0x6A=%u, RX DOUT=0x%04X TX DOUT=0x%04X\n",
            RF_FREQ_MHZ, r25, r23, r6a, rx_dout, tx_dout);
    fflush(stderr);
    return failed ? -1 : 0;
}

// --- Thread 1: Continuous TX Generation ---
void* tx_thread_func(void* arg) {
    (void)arg;
    while (g_running && !g_radio_ready)
        usleep(1000);
    if (!g_running) return NULL;

    int fd = open(TX_DEVICE, O_WRONLY | O_NONBLOCK);
    if (fd < 0) { perror("TX open"); return NULL; }

    const size_t tx_bytes = TX_PAYLOAD_BYTES; 
    const size_t tx_samples = tx_bytes / 2; 
    int8_t* tx_buf = malloc(tx_bytes);
    
    double scale_64 = 18446744073709551616.0; 
    uint64_t phase_step = (uint64_t)((TONE_FREQ_HZ / EXACT_TX_RATE) * scale_64);
    uint64_t phase_acc = 0;
    
    struct pollfd pfd = { .fd = fd, .events = POLLOUT };

    while (g_running) {
        if (poll(&pfd, 1, 10) > 0) {
            uint64_t temp_acc = phase_acc;
            for (size_t i = 0; i < tx_samples; i++) {
                uint32_t phase_32 = (uint32_t)(temp_acc >> 32);
                uint32_t idx = phase_32 >> (32 - LUT_BITS);
                
                tx_buf[i*2 + 0] = (int8_t)(127.0 * cos_lut[idx]); 
                tx_buf[i*2 + 1] = (int8_t)(127.0 * sin_lut[idx]); 
                temp_acc += phase_step; 
            }
            
            ssize_t w = write(fd, tx_buf, tx_bytes);
            if (w > 0) {
                phase_acc += (w / 2) * phase_step;
            } else if (w < 0 && errno != EAGAIN) {
                break;
            }
        }
    }
    free(tx_buf);
    close(fd);
    return NULL;
}

// --- Thread 2: Dedicated RX DDC & TDM Control ---
void* rx_thread_func(void* arg) {
    (void)arg;
    int fd_rx = open(RX_DEVICE, O_RDWR | O_NONBLOCK);
    if (fd_rx < 0) { perror("RX open"); return NULL; }

    struct csi_ring_info ri;
    if (ioctl(fd_rx, CSI_IOC_GET_RING_INFO, &ri) < 0) { perror("RX ring info"); return NULL; }

    long page_size = sysconf(_SC_PAGESIZE);
    size_t map_len = (size_t)((ri.ring_size + page_size - 1) & ~((uint64_t)page_size - 1));
    void *ring = mmap(NULL, map_len, PROT_READ, MAP_SHARED, fd_rx, 0);
    if (ring == MAP_FAILED) { perror("RX mmap"); return NULL; }

    if (jtag_acquire_lease(fd_rx) != 0)
        perror("Warning: Failed to acquire JTAG lease");
    if (setup_radio(fd_rx) != 0)
        fprintf(stderr, "Warning: radio setup incomplete; phasors may stay at the origin\n");

    if (ioctl(fd_rx, CSI_IOC_GET_RING_INFO, &ri) == 0) {
        uint32_t stale = (ri.head >= ri.tail) ? (ri.head - ri.tail)
                                              : (ri.ring_size - (ri.tail - ri.head));
        if (stale > 0)
            ioctl(fd_rx, CSI_IOC_CONSUME_BYTES, &stale);
    }
    g_radio_ready = true;

    struct pollfd pfd_rx = { .fd = fd_rx, .events = POLLIN };
    
    double scale_64 = 18446744073709551616.0; 
    uint64_t phase_step = (uint64_t)((TONE_FREQ_HZ / EXACT_RX_RATE) * scale_64);
    uint64_t phase_acc = 0;
    
    // TDM State Machine Variables
    int current_tx = 0;
    bool is_flushing = true;
    uint32_t flush_count = 0;
    uint32_t flush_target = FLUSH_SAMPLES; // Dynamic target
    int sample_count = 0;
    double acc_i[NUM_CHANNELS] = {0};
    double acc_q[NUM_CHANNELS] = {0};
    
    float local_phasor_i[NUM_CHANNELS][NUM_CHANNELS] = {0};
    float local_phasor_q[NUM_CHANNELS][NUM_CHANNELS] = {0};

    switch_tx_antenna(fd_rx, current_tx);

    uint64_t frames_seen = 0;
    bool logged_progress = false;
    bool logged_rms = false;
    int idle_polls = 0;

    while (g_running) {
        int pr = poll(&pfd_rx, 1, 10);
        if (pr <= 0) {
            idle_polls++;
            if (idle_polls == 50) {
                struct csi_stats st;
                struct csi_ring_info ri2;
                memset(&st, 0, sizeof(st));
                memset(&ri2, 0, sizeof(ri2));
                ioctl(fd_rx, CSI_IOC_GET_STATS, &st);
                ioctl(fd_rx, CSI_IOC_GET_RING_INFO, &ri2);
                fprintf(stderr,
                        "rx: no CSI data after 500 ms, head=%u tail=%u size=%u dma_bytes=%llu frames=%u ovf=%llu\n",
                        ri2.head, ri2.tail, ri2.ring_size,
                        (unsigned long long)st.dma_bytes, st.frame_count,
                        (unsigned long long)st.overflows);
                fflush(stderr);
            }
            continue;
        }
        idle_polls = 0;
        if (ioctl(fd_rx, CSI_IOC_GET_RING_INFO, &ri) == 0) { //
                uint32_t head = ri.head, tail = ri.tail; //
                uint32_t used = (head >= tail) ? (head - tail) : (ri.ring_size - (tail - head));

                if (used >= ri.ring_size - BYTES_PER_FRAME) {
                    fprintf(stderr, "\n[!] CRITICAL: Hardware Overrun. Phase lock permanently lost.\n");
                }

                uint32_t bytes_to_process = used - (used % BYTES_PER_FRAME);

                if (bytes_to_process > 0) {
                    uint32_t frames = bytes_to_process / BYTES_PER_FRAME;
                    uint32_t frames_contig = (tail + bytes_to_process > ri.ring_size) ? 
                                             ((ri.ring_size - tail) / BYTES_PER_FRAME) : frames;

                    const int8_t *src = (const int8_t*)ring + tail;

                    if (!logged_rms) {
                        double acc_abs = 0.0;
                        uint32_t nrms = frames_contig < 256 ? frames_contig : 256;
                        for (uint32_t n = 0; n < nrms; n++) {
                            for (int b = 0; b < BYTES_PER_FRAME; b++)
                                acc_abs += fabs((double)src[n * BYTES_PER_FRAME + b]);
                        }
                        fprintf(stderr, "rx: first-chunk mean|cs8|=%.1f over %u frames\n",
                                acc_abs / (double)(nrms * BYTES_PER_FRAME), nrms);
                        fflush(stderr);
                        logged_rms = true;
                    }

                    frames_seen += frames_contig;
                    if (!logged_progress && frames_seen >= 1000) {
                        fprintf(stderr, "rx: ring used=%u frames_seen=%llu flush=%d/%u sample=%d\n",
                                used, (unsigned long long)frames_seen, is_flushing,
                                flush_target, sample_count);
                        fflush(stderr);
                        logged_progress = true;
                    }
                    
                    for (uint32_t n = 0; n < frames_contig; n++) {
                        uint32_t phase_32 = (uint32_t)(phase_acc >> 32);
                        uint32_t idx = phase_32 >> (32 - LUT_BITS);
                        double ref_c = cos_lut[idx];
                        double ref_s = sin_lut[idx];

                        if (!is_flushing) {
                            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                                // Skip monostatic self-coupling (saves 25% CPU overhead in inner loop)
                                if (ch == current_tx) continue;

                                double rx_i = (double)src[n * BYTES_PER_FRAME + ch * 2 + 0] / 127.0;
                                double rx_q = (double)src[n * BYTES_PER_FRAME + ch * 2 + 1] / 127.0;
                                acc_i[ch] += (rx_i * ref_c + rx_q * ref_s);
                                acc_q[ch] += (rx_q * ref_c - rx_i * ref_s);
                            }
                            sample_count++;
                            
                            if (sample_count >= INTEGRATION_SAMPLES) {
                                pthread_mutex_lock(&g_phasor_mutex);
                                for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                                    if (ch == current_tx) continue;

                                    float norm_i = (float)(acc_i[ch] / INTEGRATION_SAMPLES);
                                    float norm_q = (float)(acc_q[ch] / INTEGRATION_SAMPLES);
                                    
                                    local_phasor_i[ch][current_tx] = (1.0f - EMA_ALPHA) * local_phasor_i[ch][current_tx] + EMA_ALPHA * norm_i;
                                    local_phasor_q[ch][current_tx] = (1.0f - EMA_ALPHA) * local_phasor_q[ch][current_tx] + EMA_ALPHA * norm_q;
                                    
                                    g_phasor_i[ch][current_tx] = local_phasor_i[ch][current_tx];
                                    g_phasor_q[ch][current_tx] = local_phasor_q[ch][current_tx];
                                    
                                    acc_i[ch] = 0.0;
                                    acc_q[ch] = 0.0;
                                }
                                pthread_mutex_unlock(&g_phasor_mutex);

                                if (current_tx == 0) {
                                    float peak = 0.0f;
                                    for (int ch = 1; ch < NUM_CHANNELS; ch++) {
                                        float m = hypotf(local_phasor_i[ch][0], local_phasor_q[ch][0]);
                                        if (m > peak) peak = m;
                                    }
                                    static int mag_logs;
                                    if (mag_logs < 4) {
                                        fprintf(stderr, "peak |phasor| TX0 bistatic = %.5f (scale 1000 -> %.1f px)\n",
                                                peak, peak * 1000.0f);
                                        fflush(stderr);
                                        mag_logs++;
                                    }
                                }
                                
                                sample_count = 0;
                                current_tx = (current_tx + 1) % NUM_CHANNELS;
                                switch_tx_antenna(fd_rx, current_tx);
                                
                                // --- CRITICAL FIX: Dynamic Pipeline Purging ---
                                // Query the ring buffer immediately after the SPI switch.
                                // Any data currently in RAM was captured BEFORE the switch finished.
                                struct csi_ring_info sync_ri; //
                                uint32_t stale_frames = 0;
                                if (ioctl(fd_rx, CSI_IOC_GET_RING_INFO, &sync_ri) == 0) { //
                                    uint32_t stale_bytes = (sync_ri.head >= sync_ri.tail) ? //
                                                           (sync_ri.head - sync_ri.tail) : 
                                                           (sync_ri.ring_size - (sync_ri.tail - sync_ri.head));
                                    stale_frames = stale_bytes / BYTES_PER_FRAME;
                                }
                                
                                // Flush the past (stale backlog) + the future (analog settling)
                                flush_target = stale_frames + FLUSH_SAMPLES;
                                is_flushing = true;
                                flush_count = 0;
                            }
                        } else {
                            flush_count++;
                            if (flush_count >= flush_target) {
                                is_flushing = false;
                            }
                        }
                        
                        phase_acc += phase_step; 
                    }

                    uint32_t consumed_bytes = frames_contig * BYTES_PER_FRAME;
                    if (ioctl(fd_rx, CSI_IOC_CONSUME_BYTES, &consumed_bytes) < 0) perror("Consume error"); //
                }
        }
    }

    ioctl(fd_rx, CSI_IOC_JTAG_RELEASE_LEASE);
    munmap(ring, map_len);
    close(fd_rx);
    return NULL;
}

// GUI Helper
void draw_phasor(SDL_Renderer *ren, int cx, int cy, float i_val, float q_val, float scale) {
    int ex = cx + (int)lroundf(i_val * scale);
    int ey = cy - (int)lroundf(q_val * scale);
    SDL_RenderDrawLine(ren, cx, cy, ex, ey);
    SDL_RenderDrawLine(ren, cx + 1, cy, ex + 1, ey);
    SDL_Rect tip = { ex - 3, ey - 3, 6, 6 };
    SDL_RenderFillRect(ren, &tip);
}

// --- Thread 3: Main UI Thread ---
int main(void) {
    init_luts();

    if (SDL_Init(SDL_INIT_VIDEO) != 0) { fprintf(stderr, "SDL_Init Error\n"); return EXIT_FAILURE; }
    SDL_Window *win = SDL_CreateWindow("4x4 MIMO Near-Field Phasors", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, SDL_WINDOW_SHOWN);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RaiseWindow(win);

    pthread_t tx_thread, rx_thread;
    pthread_create(&tx_thread, NULL, tx_thread_func, NULL);
    pthread_create(&rx_thread, NULL, rx_thread_func, NULL);

    float display_scale = 1000.0f; 
    
    float ui_i[NUM_CHANNELS][NUM_CHANNELS] = {0};
    float ui_q[NUM_CHANNELS][NUM_CHANNELS] = {0};
    float cal_i[NUM_CHANNELS][NUM_CHANNELS] = {0};
    float cal_q[NUM_CHANNELS][NUM_CHANNELS] = {0};
    
    SDL_Color tx_colors[4] = {
        {255, 50,  50,  255},  // TX0: Red
        {50,  255, 50,  255},  // TX1: Green
        {50,  200, 255, 255},  // TX2: Cyan
        {255, 255, 50,  255}   // TX3: Yellow
    };

    SDL_Event ev;
    
    while (g_running) {
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT || (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE)) {
                g_running = false;
            }
            else if (ev.type == SDL_KEYDOWN) {
                if (ev.key.keysym.sym == SDLK_SPACE) {
                    pthread_mutex_lock(&g_phasor_mutex);
                    for (int rx = 0; rx < NUM_CHANNELS; rx++) {
                        for (int tx = 0; tx < NUM_CHANNELS; tx++) {
                            // Only calibrate the active bistatic pathways
                            if (rx == tx) continue;
                            cal_i[rx][tx] = g_phasor_i[rx][tx];
                            cal_q[rx][tx] = g_phasor_q[rx][tx];
                        }
                    }
                    pthread_mutex_unlock(&g_phasor_mutex);
                    printf("Calibration Latched! Direct path offset zeroed for bistatic pathways.\n");
                    fflush(stdout);
                }
                else if (ev.key.keysym.sym == SDLK_UP || ev.key.keysym.sym == SDLK_KP_PLUS
                         || ev.key.keysym.sym == SDLK_EQUALS || ev.key.keysym.sym == SDLK_PLUS) {
                    display_scale *= 1.2f;
                    printf("Scale: %.1f\n", display_scale);
                    fflush(stdout);
                }
                else if (ev.key.keysym.sym == SDLK_DOWN || ev.key.keysym.sym == SDLK_KP_MINUS
                         || ev.key.keysym.sym == SDLK_MINUS) {
                    display_scale /= 1.2f;
                    printf("Scale: %.1f\n", display_scale);
                    fflush(stdout);
                }
            }
        }

        pthread_mutex_lock(&g_phasor_mutex);
        for (int rx = 0; rx < NUM_CHANNELS; rx++) {
            for (int tx = 0; tx < NUM_CHANNELS; tx++) {
                if (rx == tx) continue;
                ui_i[rx][tx] = g_phasor_i[rx][tx];
                ui_q[rx][tx] = g_phasor_q[rx][tx];
            }
        }
        pthread_mutex_unlock(&g_phasor_mutex);

        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);

        int half_w = WIN_W / 2;
        int half_h = WIN_H / 2;
        
        SDL_SetRenderDrawColor(ren, 70, 70, 70, 255);
        SDL_RenderDrawLine(ren, half_w, 0, half_w, WIN_H);
        SDL_RenderDrawLine(ren, 0, half_h, WIN_W, half_h);

        for (int rx = 0; rx < NUM_CHANNELS; rx++) {
            int cx = (rx % 2 == 0) ? (half_w / 2) : (half_w + half_w / 2);
            int cy = (rx < 2)      ? (half_h / 2) : (half_h + half_h / 2);
            
            SDL_SetRenderDrawColor(ren, 100, 100, 100, 255);
            SDL_RenderDrawLine(ren, cx - 10, cy, cx + 10, cy);
            SDL_RenderDrawLine(ren, cx, cy - 10, cx, cy + 10);
            
            for (int tx = 0; tx < NUM_CHANNELS; tx++) {
                // Do not render the saturated monostatic pathway
                if (rx == tx) continue;

                float disp_i = ui_i[rx][tx] - cal_i[rx][tx];
                float disp_q = ui_q[rx][tx] - cal_q[rx][tx];
                
                SDL_SetRenderDrawColor(ren, tx_colors[tx].r, tx_colors[tx].g, tx_colors[tx].b, 255);
                draw_phasor(ren, cx, cy, disp_i, disp_q, display_scale);
            }
        }

        SDL_RenderPresent(ren);
    }

    pthread_join(tx_thread, NULL);
    pthread_join(rx_thread, NULL);
    
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    
    return 0;
}