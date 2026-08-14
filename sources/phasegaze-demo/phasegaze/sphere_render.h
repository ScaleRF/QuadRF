// sphere_render.h
// Hemisphere mesh, FOV tile shader, and point cloud rendering

#ifndef SPHERE_RENDER_H
#define SPHERE_RENDER_H

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

#include <stdint.h>

#include "worker.h"

typedef struct {
    float gx, gy;
    float r, g, b;
    float intensity;
} render_point_t;

// Shutter-mode binning cell. Each new point's contribution is added in;
// rebuild_display walks the grid and emits one render_point per non-empty bin.
typedef struct {
    float gx_sum, gy_sum;
    float r_sum, g_sum, b_sum;
    float intensity_sum;
    uint32_t count;
} shutter_bin_t;

// View options bundle passed to sphere_render. Lets the renderer flip
// between the orbit-camera sphere view and a fixed pinhole-from-origin
// "viewfinder" view (with two sub-modes) without growing the call signature.
typedef struct {
    int   viewfinder;   // 0 = orbit sphere, 1 = viewfinder
    int   flat;         // viewfinder only: 1 = ortho on (gx, gy), 0 = pinhole
    float ortho_sx;     // viewfinder + flat: clip-space scale for gx
    float ortho_sy;     // viewfinder + flat: clip-space scale for gy

    // Optional centroid crosshair overlay. Drawn after the points in the
    // same (gx, gy) coordinate frame. Used by phase-cal mode to show the
    // intensity-weighted centre of the live point cloud.
    int   show_centroid;
    float centroid_gx;
    float centroid_gy;

    // Hex reticle in viewfinder mode. Off during phase-cal so the boresight
    // overlay isn't competing with the green outline.
    int   show_hex_reticle;
} sphere_view_opts_t;

typedef struct {
    // Upper hemisphere
    GLuint hemi_vao, hemi_vbo, hemi_ebo;
    int hemi_index_count;

    // Lower hemisphere
    GLuint hemi_lower_vao, hemi_lower_vbo, hemi_lower_ebo;
    int hemi_lower_index_count;

    // Hemisphere shader
    GLuint hemi_shader;
    GLint hemi_u_mvp;
    GLint hemi_u_scale_factor;
    GLint hemi_u_scale_factor_low;
    GLint hemi_u_scale_factor_high;

    // Point cloud
    GLuint pt_vao, pt_vbo;
    GLuint pt_shader;
    GLint pt_u_mvp;
    GLint pt_u_scale_factor;
    GLint pt_u_radius_offset;
    GLint pt_u_point_size;
    GLint pt_u_gain;
    GLint pt_u_flip_w;
    GLint pt_u_gradient_offset;
    GLint pt_u_gauss_sigma;   // 0.0 = legacy smoothstep circle, >0 = Gaussian sigma
    GLint pt_u_min_alpha;     // alpha floor (shutter only)
    GLint pt_u_size_floor;    // floor for the (floor + (1-floor)*I) size curve
    GLint pt_u_flat;          // 1 = bypass sphere projection, draw at uOrthoScale*(gx,gy)
    GLint pt_u_ortho_scale;   // clip-space scale used when uFlat=1

    // Hex reticle (deterministic Voronoi cell outline)
    GLuint hex_vao, hex_vbo;
    GLuint hex_shader;
    GLint hex_u_mvp;
    GLint hex_u_flat;
    GLint hex_u_ortho_scale;
    GLint hex_u_scale_factor;
    GLint hex_u_radius_offset;
    GLint hex_u_color;

    // Centroid crosshair (phase-cal mode overlay). Reuses the hex shader.
    // The VBO holds 4 vertices (two GL_LINES) recomputed each draw to centre
    // on the live (gx, gy) centroid.
    GLuint cent_vao, cent_vbo;

    // Floor disk
    GLuint floor_vao, floor_vbo;
    int floor_vert_count;
    GLuint floor_shader;
    GLint floor_u_mvp;

    // Axes
    GLuint axes_vao, axes_vbo;
    GLuint axes_shader;
    GLint axes_u_mvp;

    // History buffer for decay
    render_point_t *history;
    int history_count;
    int history_capacity;

    // Shutter mode (long-exposure). Raw points are streamed straight to
    // flash by shutter_stream.c; the only in-RAM accumulator is the bin
    // grid below, which drives the live preview.
    uint64_t        shutter_total_points;    // running total of points pushed
    int             shutter_full;            // sticky 1 if stream dropped pts

    // Display bin grid (SHUTTER_GRID_RES * SHUTTER_GRID_RES cells).
    shutter_bin_t  *shutter_grid;

    int             shutter_active;
    int             shutter_dirty_frames;    // frames since last display rebuild
} sphere_state_t;

void sphere_init(sphere_state_t *s);
void sphere_destroy(sphere_state_t *s);

// decay_threshold: minimum intensity to keep after decay and for new points.
void sphere_update_points(sphere_state_t *s,
                          const point_data_t *new_pts, int n_new,
                          float decay_factor, float decay_threshold);

// mvp = column-major 4x4. show_lower toggles bottom hemisphere.
// show_mirrors draws point cloud at all reciprocal lattice offsets.
// lo_start_mhz/lo_end_mhz are the current sweep bounds for the dual boundary lines.
// view may be NULL, in which case the renderer behaves as the legacy orbit-camera
// sphere view.
void sphere_render(const sphere_state_t *s, const float *mvp,
                   float point_size, float point_gain,
                   int show_lower, int show_mirrors,
                   float lo_start_mhz, float lo_end_mhz,
                   const sphere_view_opts_t *view);

// ---- Shutter mode (long exposure) ----
//
// begin: lazy-allocate the bin grid, zero it, mark shutter active, clear the
//        live display. The actual file-streaming lifecycle is owned by
//        shutter_stream.c — main.c calls shutter_stream_begin / _end around
//        these.
// add:   push raw points into the streaming ring (writes them to flash via
//        the background writer thread) and bin each one into the display
//        grid. No GPU upload here.
// rebuild_display: walk the grid, write one render_point per non-empty cell
//        into history[], then upload to the VBO (one upload regardless of
//        how many raw points have been pushed).
// end:   clear shutter_active. Bin grid stays allocated for reuse.
void sphere_shutter_begin(sphere_state_t *s);
void sphere_shutter_add(sphere_state_t *s, const point_data_t *pts, int n);
void sphere_shutter_rebuild_display(sphere_state_t *s);
void sphere_shutter_end(sphere_state_t *s);

// Stream raw points to flash without touching the bin grid. Used by the
// viewfinder path so disk recording keeps working while the live display
// is driven by the normal decay-history buffer instead of the bin grid.
void sphere_shutter_stream_only(sphere_state_t *s, const point_data_t *pts, int n);

#endif // SPHERE_RENDER_H
