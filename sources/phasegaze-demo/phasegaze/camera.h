// camera.h
// Orbit camera for 3D hemisphere visualization

#ifndef CAMERA_H
#define CAMERA_H

typedef struct {
    // Spherical coordinates (orbit around target)
    float azimuth;      // radians, horizontal angle
    float elevation;    // radians, vertical angle (clamped above horizon)
    float distance;     // distance from target

    // Target point (orbit center)
    float target[3];

    // Mouse state
    int dragging;
    double last_mx, last_my;

    // Computed view matrix (column-major)
    float view[16];
    float proj[16];
    float mvp[16];
} camera_t;

// Initialize with default orbit position looking at hemisphere
void camera_init(camera_t *cam);

// Recompute projection matrix for the given aspect ratio
void camera_set_projection(camera_t *cam, float aspect, float fov_deg,
                           float near, float far);

// Recompute view matrix from current spherical coords
void camera_update(camera_t *cam);

// Input handlers (call from GLFW callbacks)
void camera_on_mouse_button(camera_t *cam, int button, int action, double mx, double my);
void camera_on_cursor_move(camera_t *cam, double mx, double my);
void camera_on_scroll(camera_t *cam, double yoffset);

// Utility: 4x4 matrix multiply (column-major)
void mat4_mul(float *out, const float *a, const float *b);

// Build a column-major perspective projection matrix.
void mat4_perspective(float *m, float fov_rad, float aspect,
                      float near, float far);

// Build a column-major look-at view matrix.
void mat4_lookat(float *m, const float *eye, const float *center,
                 const float *up);

#endif // CAMERA_H
