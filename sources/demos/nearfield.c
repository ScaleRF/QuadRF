/* ============================================================================
 * 4x4 MIMO Coherent Near-Field Phasor (Digital Down-Converter)
 *
 * Adds Time-Division Multiplexing (TDM) to the TX array, rapidly cycling
 * through the 4 TX antennas. Uses a strict DSP state machine to flush analog
 * switching transients while maintaining absolute phase lock.
 *
 * Optimizations:
 * - 64-bit DDS Phase Accumulator for zero-drift LO generation.
 * - Monostatic pathway blanking (rx == tx) to prevent LNA saturation 
 * interference and save 25% CPU overhead in the DDC loop.
 * - Dynamic Pipeline Purging: Queries ring buffer backlog directly after 
 * switching to prevent stale data from bleeding into the new integration bucket.
 * ============================================================================
 */

// sudo apt install libsdl2-dev
// gcc nearfield.c -O3 -o nearfield -lSDL2 -lm -lpthread

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

// Driver UAPI mappings
#define CSI_IOC_MAGIC 'C'
#define CSI_IOC_GET_RING_INFO _IOR(CSI_IOC_MAGIC, 0x40, struct csi_ring_info)
#define CSI_IOC_CONSUME_BYTES _IOW(CSI_IOC_MAGIC, 0x41, __u32)

// JTAG Hardware Locks
#ifndef CSI_IOC_JTAG_ACQUIRE_LEASE
#define CSI_IOC_JTAG_ACQUIRE_LEASE _IO(CSI_IOC_MAGIC, 0x47) 
#endif
#ifndef CSI_IOC_JTAG_RELEASE_LEASE
#define CSI_IOC_JTAG_RELEASE_LEASE _IO(CSI_IOC_MAGIC, 0x48) 
#endif

#include "../fpga/drivers/csi/fpga_csi.h"

#define MAX2850_REG_ADDR 0x42 

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

// --- JTAG Helpers ---
int jtag_write_u16(int fd, uint8_t addr, uint16_t value) {
    struct csi_jtag_reg r = { .addr = addr, .value = value }; //
    return ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r); //
}

int jtag_read_u16(int fd, uint8_t addr, uint16_t *out_value) {
    if (!out_value) return -1;
    struct csi_jtag_reg r = { .addr = addr, .value = 0 }; //
    if (ioctl(fd, CSI_IOC_JTAG_REG_READ, &r) < 0) return -1; //
    *out_value = r.value;
    return 0;
}

void switch_tx_antenna(int fd, int tx_idx) {
    uint16_t mask = 1 << tx_idx;
    uint16_t reg0 = 0x00E | (mask << 5); 
    uint16_t w = (0 << 10) | (reg0 & 0x3FF);
    
    // Write new state
    if (jtag_write_u16(fd, MAX2850_REG_ADDR, w) < 0) perror("[!] MAX2850 SPI Write Failed");
    if (jtag_write_u16(fd, 0x02, mask) < 0)          perror("[!] FPGA Mask Write Failed");
}

// --- Thread 1: Continuous TX Generation ---
void* tx_thread_func(void* arg) {
    (void)arg;
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
    int fd_rx = open(RX_DEVICE, O_RDONLY | O_NONBLOCK);
    if (fd_rx < 0) { perror("RX open"); return NULL; }

    // Acquire Exclusive JTAG Lease
    while (ioctl(fd_rx, CSI_IOC_JTAG_ACQUIRE_LEASE) != 0) { 
        if (errno == ENOTTY) {
            break; 
        } else if (errno == EBUSY) {
            usleep(1000); 
        } else {
            perror("Warning: Failed to acquire JTAG lease");
            break;
        }
    }

    struct csi_ring_info ri; //
    if (ioctl(fd_rx, CSI_IOC_GET_RING_INFO, &ri) < 0) { perror("RX ring info"); return NULL; } //

    long page_size = sysconf(_SC_PAGESIZE);
    size_t map_len = (size_t)((ri.ring_size + page_size - 1) & ~((uint64_t)page_size - 1));
    void *ring = mmap(NULL, map_len, PROT_READ, MAP_SHARED, fd_rx, 0);
    if (ring == MAP_FAILED) { perror("RX mmap"); return NULL; }

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

    while (g_running) {
        if (poll(&pfd_rx, 1, 10) > 0) {
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
    }

    ioctl(fd_rx, CSI_IOC_JTAG_RELEASE_LEASE); //
    munmap(ring, map_len);
    close(fd_rx);
    return NULL;
}

// GUI Helper
void draw_phasor(SDL_Renderer *ren, int cx, int cy, float i_val, float q_val, float scale) {
    int ex = cx + (int)(i_val * scale);
    int ey = cy - (int)(q_val * scale);
    SDL_RenderDrawLine(ren, cx, cy, ex, ey);
    SDL_Rect tip = { ex - 3, ey - 3, 6, 6 };
    SDL_RenderFillRect(ren, &tip);
}

// --- Thread 3: Main UI Thread ---
int main(void) {
    init_luts();

    if (SDL_Init(SDL_INIT_VIDEO) != 0) { fprintf(stderr, "SDL_Init Error\n"); return EXIT_FAILURE; }
    SDL_Window *win = SDL_CreateWindow("4x4 MIMO Near-Field Phasors", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, SDL_WINDOW_SHOWN);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

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
                }
                else if (ev.key.keysym.sym == SDLK_UP || ev.key.keysym.sym == SDLK_KP_PLUS) {
                    display_scale *= 1.2f;
                    printf("Scale: %.1f\n", display_scale);
                }
                else if (ev.key.keysym.sym == SDLK_DOWN || ev.key.keysym.sym == SDLK_KP_MINUS) {
                    display_scale /= 1.2f;
                    printf("Scale: %.1f\n", display_scale);
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