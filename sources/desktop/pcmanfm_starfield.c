#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <malloc.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct _GdkWindow GdkWindow;
typedef struct _cairo cairo_t;
typedef struct _cairo_pattern cairo_pattern_t;

static void (*real_gdk_window_set_background_pattern)(GdkWindow *, cairo_pattern_t *);
static void (*real_cairo_fill)(cairo_t *);

static void (*fn_cairo_save)(cairo_t *);
static void (*fn_cairo_restore)(cairo_t *);
static void (*fn_cairo_clip)(cairo_t *);
static void (*fn_cairo_set_operator)(cairo_t *, int);
static void (*fn_cairo_set_source_rgb)(cairo_t *, double, double, double);
static void (*fn_cairo_set_source_rgba)(cairo_t *, double, double, double, double);
static void (*fn_cairo_paint)(cairo_t *);
static void (*fn_cairo_clip_extents)(cairo_t *, double *, double *, double *, double *);
static void (*fn_cairo_move_to)(cairo_t *, double, double);
static void (*fn_cairo_line_to)(cairo_t *, double, double);
static void (*fn_cairo_set_line_width)(cairo_t *, double);
static void (*fn_cairo_stroke)(cairo_t *);
static void (*fn_cairo_arc)(cairo_t *, double, double, double, double, double);
static cairo_pattern_t *(*fn_cairo_get_source)(cairo_t *);

static cairo_pattern_t *desktop_bg_pattern = NULL;

__attribute__((constructor))
static void init_starfield(void) {
    /* Strip LD_PRELOAD only once inside pcmanfm itself so child processes
     * launched from desktop icons do not inherit the hook, while wrapper
     * utilities (env, nohup, bash) still pass LD_PRELOAD through to pcmanfm. */
    if (program_invocation_short_name &&
        strcmp(program_invocation_short_name, "pcmanfm") == 0) {
        unsetenv("LD_PRELOAD");
    }
}

static void resolve_cairo_symbols(void) {
    if (fn_cairo_get_source)
        return;
    fn_cairo_save            = dlsym(RTLD_DEFAULT, "cairo_save");
    fn_cairo_restore         = dlsym(RTLD_DEFAULT, "cairo_restore");
    fn_cairo_clip            = dlsym(RTLD_DEFAULT, "cairo_clip");
    fn_cairo_set_operator    = dlsym(RTLD_DEFAULT, "cairo_set_operator");
    fn_cairo_set_source_rgb  = dlsym(RTLD_DEFAULT, "cairo_set_source_rgb");
    fn_cairo_set_source_rgba = dlsym(RTLD_DEFAULT, "cairo_set_source_rgba");
    fn_cairo_paint           = dlsym(RTLD_DEFAULT, "cairo_paint");
    fn_cairo_clip_extents    = dlsym(RTLD_DEFAULT, "cairo_clip_extents");
    fn_cairo_move_to         = dlsym(RTLD_DEFAULT, "cairo_move_to");
    fn_cairo_line_to         = dlsym(RTLD_DEFAULT, "cairo_line_to");
    fn_cairo_set_line_width  = dlsym(RTLD_DEFAULT, "cairo_set_line_width");
    fn_cairo_stroke          = dlsym(RTLD_DEFAULT, "cairo_stroke");
    fn_cairo_arc             = dlsym(RTLD_DEFAULT, "cairo_arc");
    fn_cairo_get_source      = dlsym(RTLD_DEFAULT, "cairo_get_source");
}

/* Stateless 2D spatial hash for procedural star placement without backing pixmaps. */
static inline uint32_t hash2d(int32_t x, int32_t y, uint32_t seed) {
    uint32_t h = (uint32_t)x * 374761393U + (uint32_t)y * 668265263U + seed * 1442695041U;
    h = (h ^ (h >> 13)) * 1274126177U;
    return h ^ (h >> 16);
}

static inline double u01(uint32_t h) {
    return (double)(h & 0xFFFFFFU) / (double)0x1000000U;
}

typedef struct {
    double x, y;
    double radius;
    double alpha;
    int has_star;
} anchor_star_t;

static inline anchor_star_t get_anchor_star(int32_t cx, int32_t cy) {
    anchor_star_t s = {0};
    uint32_t h0 = hash2d(cx, cy, 0xA17C9E3U);
    if ((h0 & 0xFFU) < 168) {
        s.has_star = 1;
        uint32_t h1 = hash2d(cx, cy, 0x19283746U);
        uint32_t h2 = hash2d(cx, cy, 0x9E3779B9U);
        s.x = ((double)cx + 0.14 + 0.72 * u01(h1)) * 96.0;
        s.y = ((double)cy + 0.14 + 0.72 * u01(h2)) * 96.0;
        s.radius = 0.75 + 0.45 * u01(h0 >> 8);
        s.alpha  = 0.50 + 0.35 * u01(h1 >> 8);
    }
    return s;
}

static void paint_procedural_starfield(cairo_t *cr) {
    fn_cairo_save(cr);

    /* GTK3's gdk_window_begin_draw_frame loads the YX-banded dirty region into
     * cr's active path via gdk_cairo_region() before calling cairo_fill().
     * Consuming that path into the clip region prevents cairo_stroke() below
     * from stroking the dirty-region band boundaries during rubberband drags. */
    fn_cairo_clip(cr);

    double x1, y1, x2, y2;
    fn_cairo_clip_extents(cr, &x1, &y1, &x2, &y2);

    fn_cairo_set_operator(cr, 1 /* CAIRO_OPERATOR_SOURCE */);
    fn_cairo_set_source_rgb(cr, 6.0 / 255.0, 10.0 / 255.0, 20.0 / 255.0);
    fn_cairo_paint(cr);

    fn_cairo_set_operator(cr, 2 /* CAIRO_OPERATOR_OVER */);

    int32_t acx0 = (int32_t)floor(x1 / 96.0) - 1;
    int32_t acy0 = (int32_t)floor(y1 / 96.0) - 1;
    int32_t acx1 = (int32_t)floor(x2 / 96.0) + 1;
    int32_t acy1 = (int32_t)floor(y2 / 96.0) + 1;

    fn_cairo_set_line_width(cr, 0.85);
    fn_cairo_set_source_rgba(cr, 185.0 / 255.0, 205.0 / 255.0, 235.0 / 255.0, 0.055);

    static const int nbr[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
    for (int32_t cy = acy0; cy <= acy1; cy++) {
        for (int32_t cx = acx0; cx <= acx1; cx++) {
            anchor_star_t a = get_anchor_star(cx, cy);
            if (!a.has_star) continue;
            for (int k = 0; k < 4; k++) {
                int32_t nx = cx + nbr[k][0];
                int32_t ny = cy + nbr[k][1];
                anchor_star_t b = get_anchor_star(nx, ny);
                if (!b.has_star) continue;
                uint32_t eh = hash2d(cx * 7 + nx * 13, cy * 11 + ny * 17, 0x55AA1234U);
                if ((eh & 0xFFU) < 42) {
                    double dx = b.x - a.x;
                    double dy = b.y - a.y;
                    double d2 = dx * dx + dy * dy;
                    if (d2 >= 45.0 * 45.0 && d2 <= 125.0 * 125.0) {
                        fn_cairo_move_to(cr, a.x, a.y);
                        fn_cairo_line_to(cr, b.x, b.y);
                    }
                }
            }
        }
    }
    fn_cairo_stroke(cr);

    for (int32_t cy = acy0; cy <= acy1; cy++) {
        for (int32_t cx = acx0; cx <= acx1; cx++) {
            anchor_star_t a = get_anchor_star(cx, cy);
            if (!a.has_star) continue;
            if (a.x < x1 - 4.0 || a.x > x2 + 4.0 || a.y < y1 - 4.0 || a.y > y2 + 4.0) continue;
            fn_cairo_set_source_rgba(cr, 215.0 / 255.0, 228.0 / 255.0, 248.0 / 255.0, a.alpha);
            fn_cairo_arc(cr, a.x, a.y, a.radius, 0.0, 6.283185307179586);
            real_cairo_fill(cr);
        }
    }

    int32_t fcx0 = (int32_t)floor(x1 / 32.0) - 1;
    int32_t fcy0 = (int32_t)floor(y1 / 32.0) - 1;
    int32_t fcx1 = (int32_t)floor(x2 / 32.0) + 1;
    int32_t fcy1 = (int32_t)floor(y2 / 32.0) + 1;

    for (int32_t cy = fcy0; cy <= fcy1; cy++) {
        for (int32_t cx = fcx0; cx <= fcx1; cx++) {
            uint32_t h0 = hash2d(cx, cy, 0x7F4A7C15U);
            if ((h0 & 0xFFU) >= 145) continue;
            uint32_t h1 = hash2d(cx, cy, 0x3C6EF372U);
            uint32_t h2 = hash2d(cx, cy, 0xA54FF53AU);
            double sx = ((double)cx + 0.10 + 0.80 * u01(h1)) * 32.0;
            double sy = ((double)cy + 0.10 + 0.80 * u01(h2)) * 32.0;
            if (sx < x1 - 2.0 || sx > x2 + 2.0 || sy < y1 - 2.0 || sy > y2 + 2.0) continue;
            double u = u01(h0 >> 8);
            double radius = (u < 0.75) ? (0.48 + 0.22 * u) : (0.68 + 0.25 * (u - 0.75) * 4.0);
            double alpha  = 0.20 + 0.42 * u01(h1 >> 8);
            fn_cairo_set_source_rgba(cr, 205.0 / 255.0, 220.0 / 255.0, 244.0 / 255.0, alpha);
            fn_cairo_arc(cr, sx, sy, radius, 0.0, 6.283185307179586);
            real_cairo_fill(cr);
        }
    }

    fn_cairo_restore(cr);
}

void gdk_window_set_background_pattern(GdkWindow *window, cairo_pattern_t *pattern) {
    if (!real_gdk_window_set_background_pattern)
        real_gdk_window_set_background_pattern = dlsym(RTLD_NEXT, "gdk_window_set_background_pattern");

    desktop_bg_pattern = pattern;
    real_gdk_window_set_background_pattern(window, pattern);
    malloc_trim(0);
}

void cairo_fill(cairo_t *cr) {
    if (!real_cairo_fill)
        real_cairo_fill = dlsym(RTLD_NEXT, "cairo_fill");
    resolve_cairo_symbols();

    if (desktop_bg_pattern && fn_cairo_get_source && fn_cairo_get_source(cr) == desktop_bg_pattern) {
        paint_procedural_starfield(cr);
        return;
    }
    real_cairo_fill(cr);
}
