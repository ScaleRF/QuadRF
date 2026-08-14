// sudo apt install libsdl2-dev libfftw3-3 libfftw3-dev
// gcc psd_switchable.c -O3 -o psd_switchable -lfftw3f -lSDL2 -lm

// psd_switchable.c
//
// Live PSD plotter from /dev/csi_stream0 using mmap ring + SDL2 + FFTW3f.
//
// Features:
//   - Default: Assumes 4-channel interleaved CS8 (I0,Q0, I1,Q1, I2,Q2, I3,Q3...)
//   - Spacebar: Toggles between 4-channel view and 1-channel view.
//     In 1-channel mode, assumes packed CS8 (I,Q, I,Q...)
//
// Build:
//   gcc psd_switchable.c -O3 -o psd_switchable -lfftw3f -lSDL2 -lm
//
// Run:
//   ./psd_switchable

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <errno.h>

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <SDL2/SDL.h>
#include <fftw3.h>

#include "../csi/fpga_csi.h"

// ---------------- Config ----------------

#define DEVICE_PATH      "/dev/csi_stream0"

#define SAMPLE_RATE_HZ   20000000.0f  // For reference only

#define FFT_SIZE         4096
#define MAX_CHANNELS     4
#define BYTES_PER_IQ     2            // CS8: 1 byte I + 1 byte Q

// Time-domain histogram (I/Q) display
// Use a moderate bin count to keep rendering cheap.
#define HIST_BINS        64

// Default view settings
#define WIN_DEFAULT_W    1000
#define WIN_DEFAULT_H    800

// PSD smoothing (EMA)
#define PSD_EMA_ALPHA    0.20f

// Fixed dB axis limits
#define DB_MIN_DEFAULT   ( -20.0f)
#define DB_MAX_DEFAULT   (  80.0f)
#define EPS_POWER        (1e-20f)

// Logic threshold to flush backlog
#define FLUSH_FACTOR     4

// ---------------- Helpers ----------------

static void die(const char *msg)
{
    perror(msg);
    exit(EXIT_FAILURE);
}

static void consume_bytes(int fd, uint32_t nbytes)
{
    if (ioctl(fd, CSI_IOC_CONSUME_BYTES, &nbytes) < 0)
        die("CSI_IOC_CONSUME_BYTES");
}

// Convert raw CS8 buffer to FFTW input.
// stride_bytes: distance in bytes between samples for this specific channel.
//               (2 for single-channel packed, 8 for 4-channel interleaved)
static void cs8_to_fftw_ch(const int8_t *src, size_t nBytes, int ch, fftwf_complex *dst, int stride_bytes)
{
    // nBytes is the total chunk size read from ring.
    // Calculate how many frames (time steps) fit in this chunk.
    size_t frames = nBytes / stride_bytes;
    if (frames > FFT_SIZE) frames = FFT_SIZE;

    // Offset for this channel (always 0 if single channel mode, ch*2 if 4-chan)
    // But since the caller passes 'src' pointing to the start of the buffer,
    // we need to add the channel offset here.
    // However, for simplicity, we'll assume 'src' is the base pointer and we add offset.
    // In 4-ch mode: offset = ch * 2. In 1-ch mode: offset = 0 (ch is 0).
    size_t ch_offset = (stride_bytes == 2) ? 0 : (ch * 2);

    for (size_t n = 0; n < frames; ++n)
    {
        size_t idx = n * stride_bytes + ch_offset;
        int8_t i8 = src[idx + 0];
        int8_t q8 = src[idx + 1];
        dst[n][0] = (float)i8 / 127.0f;
        dst[n][1] = (float)q8 / 127.0f;
    }
    // Zero-pad if not enough frames to fill FFT
    for (size_t n = frames; n < FFT_SIZE; ++n)
    {
        dst[n][0] = 0.0f;
        dst[n][1] = 0.0f;
    }
}

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline int db_to_y(float db, float db_min, float db_max, int H)
{
    float t = (db - db_min) / (db_max - db_min);
    t = clampf(t, 0.0f, 1.0f);
    float yf = (1.0f - t) * (float)(H - 1);
    return (int)lrintf(yf);
}

static void draw_axes(SDL_Renderer *ren, int W, int H)
{
    SDL_RenderDrawRect(ren, &(SDL_Rect){0, 0, W, H});
    // Y ticks
    for (int i = 1; i <= 4; ++i)
    {
        int y = (int)lrintf((float)i * (float)(H - 1) / 5.0f);
        SDL_RenderDrawLine(ren, 0, y, 6, y);
    }
    // X ticks
    for (int i = 1; i <= 4; ++i)
    {
        int x = (int)lrintf((float)i * (float)(W - 1) / 5.0f);
        SDL_RenderDrawLine(ren, x, H - 1, x, H - 1 - 6);
    }
    // Center lines
    int x_dc = W / 2;
    for (int y = 2; y < H - 2; y += 8) SDL_RenderDrawLine(ren, x_dc, y, x_dc, y + 3);
    int y_mid = H / 2;
    for (int x = 2; x < W - 2; x += 10) SDL_RenderDrawLine(ren, x, y_mid, x + 4, y_mid);
}

static void draw_psd(SDL_Renderer *ren, const float *psd_db, int n, int W, int H, float db_min, float db_max)
{
    if (n <= 1) return;
    int prev_x = 0;
    int prev_y = db_to_y(psd_db[0], db_min, db_max, H);

    for (int k = 1; k < n; ++k)
    {
        int x = (int)lrintf((float)k * (float)(W - 1) / (float)(n - 1));
        int y = db_to_y(psd_db[k], db_min, db_max, H);
        SDL_RenderDrawLine(ren, prev_x, prev_y, x, y);
        prev_x = x;
        prev_y = y;
    }
}

// Compute histogram over the most recent time-domain block.
// Bins cover the full int8 range [-128,127] uniformly.
static void cs8_hist_ch(const int8_t *src, size_t nBytes, int ch, int stride_bytes, uint32_t *hist_i, uint32_t *hist_q)
{
    memset(hist_i, 0, sizeof(uint32_t) * HIST_BINS);
    memset(hist_q, 0, sizeof(uint32_t) * HIST_BINS);

    size_t frames = nBytes / stride_bytes;
    if (frames > FFT_SIZE) frames = FFT_SIZE;

    size_t ch_offset = (stride_bytes == 2) ? 0 : (ch * 2);

    for (size_t n = 0; n < frames; ++n)
    {
        size_t idx = n * (size_t)stride_bytes + ch_offset;
        int i8 = (int)src[idx + 0];
        int q8 = (int)src[idx + 1];

        // Map [-128,127] -> [0,HIST_BINS-1]
        int bi = (int)(((i8 + 128) * HIST_BINS) >> 8);
        int bq = (int)(((q8 + 128) * HIST_BINS) >> 8);
        if (bi < 0) bi = 0; else if (bi >= HIST_BINS) bi = HIST_BINS - 1;
        if (bq < 0) bq = 0; else if (bq >= HIST_BINS) bq = HIST_BINS - 1;
        hist_i[bi]++;
        hist_q[bq]++;
    }
}

static void draw_histogram(SDL_Renderer *ren, const uint32_t *hist_i, const uint32_t *hist_q, int W, int H)
{
    SDL_RenderDrawRect(ren, &(SDL_Rect){0, 0, W, H});
    int y_mid = H / 2;
    for (int x = 2; x < W - 2; x += 10) SDL_RenderDrawLine(ren, x, y_mid, x + 4, y_mid);

    uint32_t maxv = 1;
    for (int b = 0; b < HIST_BINS; ++b)
    {
        if (hist_i[b] > maxv) maxv = hist_i[b];
        if (hist_q[b] > maxv) maxv = hist_q[b];
    }

    const int pad = 2;
    const int plot_w = (W > 2*pad) ? (W - 2*pad) : W;
    const int plot_h = (H > 2*pad) ? (H - 2*pad) : H;

    float bin_wf = (float)plot_w / (float)HIST_BINS;
    int half_h = plot_h / 2;

    for (int b = 0; b < HIST_BINS; ++b)
    {
        int x0 = pad + (int)floorf((float)b * bin_wf);
        int x1 = pad + (int)floorf((float)(b + 1) * bin_wf);
        int bw = x1 - x0;
        if (bw <= 0) continue;

        int subw = (bw >= 2) ? (bw / 2) : 1;
        int hi = (int)lrintf((float)half_h * (float)hist_i[b] / (float)maxv);
        int hq = (int)lrintf((float)half_h * (float)hist_q[b] / (float)maxv);

        SDL_SetRenderDrawColor(ren, 0, 180, 255, 255); // I
        SDL_Rect ri = { x0, pad + (half_h - hi), subw, hi };
        SDL_RenderFillRect(ren, &ri);

        SDL_SetRenderDrawColor(ren, 255, 0, 180, 255); // Q
        SDL_Rect rq = { x0 + subw, pad + half_h, subw, hq };
        SDL_RenderFillRect(ren, &rq);
    }
}

// ---------------- Main ----------------

int main(void)
{
    int fd = open(DEVICE_PATH, O_RDONLY | O_NONBLOCK);
    if (fd < 0) die("open /dev/csi_stream0");

    struct csi_ring_info ri;
    if (ioctl(fd, CSI_IOC_GET_RING_INFO, &ri) < 0) die("CSI_IOC_GET_RING_INFO");

    uint64_t ring_size = ri.ring_size;
    if (ring_size == 0) { fprintf(stderr, "Ring size 0\n"); return 1; }

    long page_size = sysconf(_SC_PAGESIZE);
    size_t map_len = (size_t)((ring_size + page_size - 1) & ~((uint64_t)page_size - 1));
    void *ring = mmap(NULL, map_len, PROT_READ, MAP_SHARED, fd, 0);
    if (ring == MAP_FAILED) die("mmap");

    // FFTW
    fftwf_complex *fft_in  = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
    fftwf_complex *fft_out = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
    //fftwf_plan plan = fftwf_plan_dft_1d(FFT_SIZE, fft_in, fft_out, FFTW_FORWARD, FFTW_MEASURE);
    fftwf_plan plan = fftwf_plan_dft_1d(FFT_SIZE, fft_in, fft_out, FFTW_FORWARD, FFTW_ESTIMATE);

    // PSD Buffers (Always allocate for max channels)
    float *psd_db = (float *)calloc(MAX_CHANNELS * FFT_SIZE, sizeof(float));
    for (int i = 0; i < MAX_CHANNELS * FFT_SIZE; ++i) psd_db[i] = DB_MIN_DEFAULT;

    // Histogram buffers (latest block)
    uint32_t hist_i[MAX_CHANNELS][HIST_BINS];
    uint32_t hist_q[MAX_CHANNELS][HIST_BINS];
    memset(hist_i, 0, sizeof(hist_i));
    memset(hist_q, 0, sizeof(hist_q));

    // SDL
    if (SDL_Init(SDL_INIT_VIDEO) != 0) die("SDL_Init");
    SDL_Window *win = SDL_CreateWindow("CSI SDR PSD", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_DEFAULT_W, WIN_DEFAULT_H, SDL_WINDOW_RESIZABLE);
    // Prefer a GPU renderer, but fall back to software so we still draw under
    // KasmVNC/Xvnc (no GLX there) instead of silently leaving a blank window.
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!ren) ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
    if (!ren) die("SDL_CreateRenderer");

    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    bool quit = false;
    float db_min = DB_MIN_DEFAULT;
    float db_max = DB_MAX_DEFAULT;

    // State toggles
    bool single_chan_mode = true; // Start in 1-channel mode

    while (!quit)
    {
        SDL_Event ev;
        while (SDL_PollEvent(&ev))
        {
            if (ev.type == SDL_QUIT) quit = true;
            else if (ev.type == SDL_KEYDOWN)
            {
                if (ev.key.keysym.sym == SDLK_ESCAPE) quit = true;

                // Toggle between 1-channel and 4-channel
                if (ev.key.keysym.sym == SDLK_SPACE) {
                    single_chan_mode = !single_chan_mode;
                    printf("Switched to %s mode\n", single_chan_mode ? "1-Channel (Packed)" : "4-Channel (Interleaved)");

                    // Reset PSD history to avoid ghosting
                    for (int i = 0; i < MAX_CHANNELS * FFT_SIZE; ++i) psd_db[i] = DB_MIN_DEFAULT;
                }

                if (ev.key.keysym.sym == SDLK_UP)    { db_min += 5.0f; db_max += 5.0f; }
                if (ev.key.keysym.sym == SDLK_DOWN)  { db_min -= 5.0f; db_max -= 5.0f; }
                if (ev.key.keysym.sym == SDLK_RIGHT) { db_max += 5.0f; }
                if (ev.key.keysym.sym == SDLK_LEFT)  { db_max -= 5.0f; }
                if (ev.key.keysym.sym == SDLK_r)     { db_min = DB_MIN_DEFAULT; db_max = DB_MAX_DEFAULT; }
            }
        }

        int ret = poll(&pfd, 1, 50);
        if (ret < 0 && errno != EINTR) die("poll");

        if (ret > 0)
        {
            if (ioctl(fd, CSI_IOC_GET_RING_INFO, &ri) < 0) die("CSI_IOC_GET_RING_INFO");
            uint32_t head = ri.head;
            uint32_t tail = ri.tail;

            uint32_t used = (head >= tail) ? (head - tail) : ((uint32_t)ring_size - (tail - head));

            // Define "Frame" size based on mode
            // 4-Chan: Frame = 4 * 2 bytes = 8 bytes
            // 1-Chan: Frame = 1 * 2 bytes = 2 bytes
            int bytes_per_frame = single_chan_mode ? 2 : 8;
            uint32_t bytes_per_fft_block = FFT_SIZE * bytes_per_frame;

            if (used >= (uint32_t)bytes_per_frame)
            {
                // Flush logic: Dynamic based on current mode block size
                uint32_t max_backlog = FLUSH_FACTOR * bytes_per_fft_block;

                if (used > max_backlog)
                {
                    uint32_t drop = used - bytes_per_fft_block;
                    drop -= (drop % bytes_per_frame); // Align to frame boundary
                    if (drop > 0) consume_bytes(fd, drop);

                    // Re-read pointers after drop
                    ioctl(fd, CSI_IOC_GET_RING_INFO, &ri);
                    head = ri.head; tail = ri.tail;
                    used = (head >= tail) ? (head - tail) : ((uint32_t)ring_size - (tail - head));
                }

                uint32_t want = used;
                if (want > bytes_per_fft_block) want = bytes_per_fft_block;
                want -= (want % bytes_per_frame);

                if (want > 0)
                {
                    uint32_t n1 = want;
                    // Handle ring wrap for the read
                    if ((uint64_t)tail + n1 > ring_size) n1 = (uint32_t)ring_size - tail;

                    const int8_t *src = (const int8_t *)ring + tail;
                    consume_bytes(fd, n1);

                    // Determine active channels
                    int active_ch = single_chan_mode ? 1 : 4;
                    int stride    = bytes_per_frame; // 2 or 8

                    for (int ch = 0; ch < active_ch; ++ch)
                    {
                        cs8_hist_ch(src, n1, ch, stride, hist_i[ch], hist_q[ch]);

                        cs8_to_fftw_ch(src, n1, ch, fft_in, stride);
                        fftwf_execute(plan);

                        float *psd_ch = psd_db + ch * FFT_SIZE;
                        int half = FFT_SIZE / 2;

                        for (int k = 0; k < FFT_SIZE; ++k)
                        {
                            int idx = (k + half) % FFT_SIZE;
                            float re = fft_out[idx][0];
                            float im = fft_out[idx][1];
                            float p = re * re + im * im;
                            float db = 10.0f * log10f(p + EPS_POWER);
                            psd_ch[k] = (1.0f - PSD_EMA_ALPHA) * psd_ch[k] + PSD_EMA_ALPHA * db;
                        }
                    }
                }
            }
        }

        // ---- Render ----
        int win_w, win_h;
        SDL_GetWindowSize(win, &win_w, &win_h);

        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        SDL_RenderClear(ren);

        if (single_chan_mode)
        {
            // Draw single full screen plot (Channel 0 data)
            SDL_Rect vp = { 0, 0, win_w, win_h };
            const int M = 20;
            SDL_Rect inner = { vp.x + M, vp.y + M, vp.w - 2*M, vp.h - 2*M };

            if (inner.w > 50 && inner.h > 50) {
                // Split: PSD on top, histogram on bottom.
                int hist_h = inner.h / 4;          // 25% height
                int psd_h  = inner.h - hist_h - 8; // small gap
                if (psd_h < 40) psd_h = inner.h;   // fallback

                SDL_Rect psd_vp  = { inner.x, inner.y, inner.w, psd_h };
                SDL_Rect hist_vp = { inner.x, inner.y + psd_h + 8, inner.w, inner.h - psd_h - 8 };

                SDL_RenderSetViewport(ren, &psd_vp);
                SDL_SetRenderDrawColor(ren, 70, 70, 70, 255);
                draw_axes(ren, psd_vp.w, psd_vp.h);
                SDL_SetRenderDrawColor(ren, 0, 255, 0, 255); // PSD trace
                draw_psd(ren, psd_db, FFT_SIZE, psd_vp.w, psd_vp.h, db_min, db_max);

                if (hist_vp.h >= 30) {
                    SDL_RenderSetViewport(ren, &hist_vp);
                    SDL_SetRenderDrawColor(ren, 70, 70, 70, 255);
                    draw_histogram(ren, hist_i[0], hist_q[0], hist_vp.w, hist_vp.h);
                }
            }
        }
        else
        {
            // Draw 2x2 grid
            int cell_w = win_w / 2;
            int cell_h = win_h / 2;
            for (int ch = 0; ch < MAX_CHANNELS; ++ch)
            {
                int grid_x = (ch & 1) ? 1 : 0;
                int grid_y = (ch >= 2) ? 1 : 0;
                SDL_Rect vp = {
                    grid_x * cell_w, grid_y * cell_h,
                    (grid_x==0)? cell_w : (win_w - cell_w),
                    (grid_y==0)? cell_h : (win_h - cell_h)
                };
                const int M = 14;
                SDL_Rect inner = { vp.x + M, vp.y + M, vp.w - 2*M, vp.h - 2*M };

                if (inner.w > 50 && inner.h > 50) {
                    // Split: PSD on top, histogram on bottom.
                    int hist_h = inner.h / 4;
                    int psd_h  = inner.h - hist_h - 6;
                    if (psd_h < 30) psd_h = inner.h;

                    SDL_Rect psd_vp  = { inner.x, inner.y, inner.w, psd_h };
                    SDL_Rect hist_vp = { inner.x, inner.y + psd_h + 6, inner.w, inner.h - psd_h - 6 };

                    SDL_RenderSetViewport(ren, &psd_vp);
                    SDL_SetRenderDrawColor(ren, 70, 70, 70, 255);
                    draw_axes(ren, psd_vp.w, psd_vp.h);
                    SDL_SetRenderDrawColor(ren, 230, 230, 230, 255);
                    draw_psd(ren, psd_db + ch * FFT_SIZE, FFT_SIZE, psd_vp.w, psd_vp.h, db_min, db_max);

                    if (hist_vp.h >= 24) {
                        SDL_RenderSetViewport(ren, &hist_vp);
                        SDL_SetRenderDrawColor(ren, 70, 70, 70, 255);
                        draw_histogram(ren, hist_i[ch], hist_q[ch], hist_vp.w, hist_vp.h);
                    }
                }
                SDL_RenderSetViewport(ren, NULL);
                SDL_SetRenderDrawColor(ren, 40, 40, 40, 255);
                SDL_RenderDrawRect(ren, &vp);
            }
        }

        SDL_RenderPresent(ren);
    }

    // Cleanup
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    fftwf_destroy_plan(plan);
    fftwf_free(fft_in);
    fftwf_free(fft_out);
    free(psd_db);
    munmap(ring, map_len);
    close(fd);
    return 0;
}
