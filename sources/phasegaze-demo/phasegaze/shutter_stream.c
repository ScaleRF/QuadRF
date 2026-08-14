// shutter_stream.c
//
// SPSC ring buffer (producer = main thread via sphere_shutter_add,
// consumer = background writer thread) that streams shutter-mode raw
// points to flash. See shutter_stream.h for the public API.

#define _FILE_OFFSET_BITS 64

#include "shutter_stream.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>

// Periodically force dirty pages to disk so the kernel doesn't queue
// gigabytes of writeback and stall later. ~256 MB feels reasonable for
// SD/eMMC on the Pi.
#define FSYNC_EVERY_BYTES   (256ULL * 1024ULL * 1024ULL)

typedef struct {
    pthread_mutex_t mtx;
    pthread_cond_t  cv_data;

    render_point_t *buf;
    uint64_t        cap;          // entries
    uint64_t        head;         // next write slot (monotonic)
    uint64_t        tail;         // next read slot  (monotonic)

    FILE   *fp;
    char    partial_path[700];
    char    final_path[640];
    time_t  start_time;
    double  lo_start_mhz;
    double  lo_end_mhz;
    int     camera_mode;

    pthread_t writer;
    int       writer_running;
    volatile int active;
    volatile int stop_flag;

    uint64_t  total_pushed;
    uint64_t  total_dropped;
    uint64_t  total_written;
    int       write_error;
} stream_t;

static stream_t S;
static int S_inited = 0;

static void stream_lazy_init(void)
{
    if (S_inited) return;
    memset(&S, 0, sizeof(S));
    pthread_mutex_init(&S.mtx, NULL);
    pthread_cond_init(&S.cv_data, NULL);
    S_inited = 1;
}

// ---------------------------------------------------------------
// Writer thread
// ---------------------------------------------------------------

static void *writer_thread(void *arg)
{
    (void)arg;

    const uint64_t chunk_points =
        (uint64_t)SHUTTER_STREAM_CHUNK_BYTES / sizeof(render_point_t);
    const uint64_t fsync_period_points =
        FSYNC_EVERY_BYTES / sizeof(render_point_t);
    uint64_t since_fsync = 0;

    while (1) {
        pthread_mutex_lock(&S.mtx);
        while (S.head == S.tail && !S.stop_flag) {
            pthread_cond_wait(&S.cv_data, &S.mtx);
        }
        uint64_t avail = S.head - S.tail;
        if (avail == 0 && S.stop_flag) {
            pthread_mutex_unlock(&S.mtx);
            break;
        }

        // Take the largest contiguous run to the end of the buffer,
        // capped at the configured chunk size.
        uint64_t off = S.tail % S.cap;
        uint64_t contig = S.cap - off;
        if (contig > avail)        contig = avail;
        if (contig > chunk_points) contig = chunk_points;

        render_point_t *src = &S.buf[off];
        size_t n_pts = (size_t)contig;
        pthread_mutex_unlock(&S.mtx);

        size_t wrote = 0;
        if (S.fp && !S.write_error) {
            wrote = fwrite(src, sizeof(render_point_t), n_pts, S.fp);
            if (wrote != n_pts) {
                fprintf(stderr,
                        "shutter_stream: short write (%zu/%zu): %s\n",
                        wrote, n_pts, strerror(errno));
                S.write_error = 1;
            }
        }

        pthread_mutex_lock(&S.mtx);
        S.tail += (uint64_t)wrote;
        S.total_written += (uint64_t)wrote;
        pthread_mutex_unlock(&S.mtx);

        since_fsync += (uint64_t)wrote;
        if (since_fsync >= fsync_period_points && S.fp) {
            int fd = fileno(S.fp);
            if (fd >= 0) fdatasync(fd);
            since_fsync = 0;
        }
    }

    // Final drain: any remaining bytes in the libc buffer + flush to flash.
    if (S.fp) {
        fflush(S.fp);
        int fd = fileno(S.fp);
        if (fd >= 0) fdatasync(fd);
    }
    return NULL;
}

// ---------------------------------------------------------------
// Public API
// ---------------------------------------------------------------

int shutter_stream_begin(double lo_start_mhz, double lo_end_mhz,
                          int camera_mode,
                          char *path_out, int path_out_len)
{
    stream_lazy_init();

    if (S.active) {
        fprintf(stderr, "shutter_stream: begin while already active\n");
        return -1;
    }

    if (!S.buf) {
        S.cap = (uint64_t)SHUTTER_STREAM_RING_BYTES / sizeof(render_point_t);
        if (S.cap == 0) S.cap = 1;
        S.buf = (render_point_t*)malloc((size_t)S.cap * sizeof(render_point_t));
        if (!S.buf) {
            fprintf(stderr,
                    "shutter_stream: failed to alloc ring (%llu entries)\n",
                    (unsigned long long)S.cap);
            S.cap = 0;
            return -1;
        }
    }

    const char *home = getenv("HOME");
    if (!home || home[0] == '\0') home = ".";

    char desktop[512];
    char dir[512];
    snprintf(desktop, sizeof(desktop), "%s/Desktop", home);
    snprintf(dir, sizeof(dir), "%s/Desktop/rf_pics", home);
    if (mkdir(desktop, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "shutter_stream: mkdir %s: %s\n", desktop, strerror(errno));
        return -1;
    }
    if (mkdir(dir, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "shutter_stream: mkdir %s: %s\n", dir, strerror(errno));
        return -1;
    }

    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    snprintf(S.final_path, sizeof(S.final_path),
             "%s/rfpic_%04d%02d%02d_%02d%02d%02d.rfpic",
             dir,
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec);
    snprintf(S.partial_path, sizeof(S.partial_path),
             "%s.partial", S.final_path);

    S.fp = fopen(S.partial_path, "wb+");
    if (!S.fp) {
        fprintf(stderr, "shutter_stream: fopen %s: %s\n",
                S.partial_path, strerror(errno));
        return -1;
    }
    // Unbuffered: our point chunks are already large, and bypassing the
    // libc buffer means fdatasync after fwrite has predictable meaning.
    setvbuf(S.fp, NULL, _IONBF, 0);

    S.start_time   = now;
    S.lo_start_mhz = lo_start_mhz;
    S.lo_end_mhz   = lo_end_mhz;
    S.camera_mode  = camera_mode;

    snapshot_orientation_t no_orient = {0};
    if (snapshot_write_header(S.fp, lo_start_mhz, lo_end_mhz, camera_mode,
                               now, &no_orient, 0) != 0) {
        fclose(S.fp);
        S.fp = NULL;
        return -1;
    }

    pthread_mutex_lock(&S.mtx);
    S.head = S.tail = 0;
    S.total_pushed = S.total_dropped = S.total_written = 0;
    S.write_error  = 0;
    S.stop_flag    = 0;
    S.active       = 1;
    pthread_mutex_unlock(&S.mtx);

    if (pthread_create(&S.writer, NULL, writer_thread, NULL) != 0) {
        fprintf(stderr, "shutter_stream: pthread_create: %s\n", strerror(errno));
        fclose(S.fp);
        S.fp = NULL;
        S.active = 0;
        return -1;
    }
    S.writer_running = 1;

    if (path_out && path_out_len > 0)
        snprintf(path_out, (size_t)path_out_len, "%s", S.final_path);

    fprintf(stdout, "shutter_stream: recording -> %s\n", S.partial_path);
    fflush(stdout);
    return 0;
}

void shutter_stream_push(const render_point_t *pts, int n, int *dropped_out)
{
    if (dropped_out) *dropped_out = 0;
    if (!S_inited || !S.active || n <= 0 || !pts || !S.buf) return;

    pthread_mutex_lock(&S.mtx);

    uint64_t used = S.head - S.tail;
    uint64_t room = (used >= S.cap) ? 0 : (S.cap - used);
    uint64_t take = ((uint64_t)n < room) ? (uint64_t)n : room;

    if (take > 0) {
        uint64_t off   = S.head % S.cap;
        uint64_t first = S.cap - off;
        if (first > take) first = take;
        memcpy(&S.buf[off], pts, (size_t)first * sizeof(render_point_t));
        if (take > first) {
            memcpy(&S.buf[0], pts + first,
                   (size_t)(take - first) * sizeof(render_point_t));
        }
        S.head += take;
        S.total_pushed += take;
        pthread_cond_signal(&S.cv_data);
    }

    uint64_t dropped = (uint64_t)n - take;
    if (dropped > 0) S.total_dropped += dropped;

    pthread_mutex_unlock(&S.mtx);

    if (dropped_out && dropped > 0) *dropped_out = 1;
}

int shutter_stream_active(void)
{
    return S_inited && S.active;
}

uint64_t shutter_stream_total_pushed(void)
{
    if (!S_inited) return 0;
    pthread_mutex_lock(&S.mtx);
    uint64_t v = S.total_pushed;
    pthread_mutex_unlock(&S.mtx);
    return v;
}

uint64_t shutter_stream_total_dropped(void)
{
    if (!S_inited) return 0;
    pthread_mutex_lock(&S.mtx);
    uint64_t v = S.total_dropped;
    pthread_mutex_unlock(&S.mtx);
    return v;
}

uint64_t shutter_stream_bytes_written(void)
{
    if (!S_inited) return 0;
    pthread_mutex_lock(&S.mtx);
    uint64_t v = S.total_written;
    pthread_mutex_unlock(&S.mtx);
    return v * sizeof(render_point_t);
}

int shutter_stream_end(const snapshot_orientation_t *orient,
                        uint32_t *count_out)
{
    if (!S_inited || !S.active) {
        if (count_out) *count_out = 0;
        return -1;
    }

    // Tell the writer to drain and exit, then join.
    pthread_mutex_lock(&S.mtx);
    S.stop_flag = 1;
    pthread_cond_broadcast(&S.cv_data);
    pthread_mutex_unlock(&S.mtx);

    if (S.writer_running) {
        pthread_join(S.writer, NULL);
        S.writer_running = 0;
    }

    int rc = 0;
    if (S.write_error) rc = -1;

    uint64_t total64 = S.total_written;
    if (total64 > 0xFFFFFFFFULL) total64 = 0xFFFFFFFFULL;
    uint32_t total = (uint32_t)total64;

    if (S.fp) {
        // Patch the header in place with the final point_count + IMU.
        if (fseek(S.fp, 0, SEEK_SET) != 0) {
            fprintf(stderr, "shutter_stream: fseek SET: %s\n", strerror(errno));
            rc = -1;
        } else {
            snapshot_orientation_t no_orient = {0};
            const snapshot_orientation_t *o = orient ? orient : &no_orient;
            if (snapshot_write_header(S.fp, S.lo_start_mhz, S.lo_end_mhz,
                                       S.camera_mode, S.start_time, o,
                                       total) != 0) {
                rc = -1;
            }
        }

        // Seek to end so the NOTES block is appended past the points.
        if (fseek(S.fp, 0, SEEK_END) != 0) {
            fprintf(stderr, "shutter_stream: fseek END: %s\n", strerror(errno));
            rc = -1;
        }

        int verify = snapshot_append_notes_and_close(S.fp, S.partial_path);
        S.fp = NULL;
        if (verify == -2)                rc = -2;
        else if (verify != 0 && rc == 0) rc = -1;
    } else {
        rc = -1;
    }

    if (rc == 0) {
        if (rename(S.partial_path, S.final_path) != 0) {
            fprintf(stderr, "shutter_stream: rename %s -> %s: %s\n",
                    S.partial_path, S.final_path, strerror(errno));
            rc = -1;
        } else {
            unsigned long long drops = S.total_dropped;
            if (drops) {
                fprintf(stdout,
                        "shutter_stream: saved %u points -> %s  (%llu dropped, ring overflow)\n",
                        (unsigned)total, S.final_path, drops);
            } else {
                fprintf(stdout,
                        "shutter_stream: saved %u points -> %s\n",
                        (unsigned)total, S.final_path);
            }
            fflush(stdout);
        }
    } else {
        // Leave .partial on disk for forensic recovery; do not rename.
        fprintf(stderr, "shutter_stream: ended with errors, kept %s\n",
                S.partial_path);
    }

    pthread_mutex_lock(&S.mtx);
    S.active    = 0;
    S.stop_flag = 0;
    pthread_mutex_unlock(&S.mtx);

    if (count_out) *count_out = total;
    return rc;
}
