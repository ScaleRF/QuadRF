// camera.c
// Orbit camera implementation

#include "camera.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void mat4_mul(float *out, const float *a, const float *b)
{
    float tmp[16];
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
        {
            tmp[c*4+r] = 0;
            for (int k = 0; k < 4; ++k)
                tmp[c*4+r] += a[k*4+r] * b[c*4+k];
        }
    memcpy(out, tmp, sizeof(tmp));
}

static void mat4_identity(float *m)
{
    memset(m, 0, 16 * sizeof(float));
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}

void mat4_perspective(float *m, float fov_rad, float aspect,
                      float near, float far)
{
    memset(m, 0, 16 * sizeof(float));
    float f = 1.0f / tanf(fov_rad * 0.5f);
    m[0]  = f / aspect;
    m[5]  = f;
    m[10] = (far + near) / (near - far);
    m[11] = -1.0f;
    m[14] = (2.0f * far * near) / (near - far);
}

void mat4_lookat(float *m, const float *eye, const float *center,
                 const float *up)
{
    float fx = center[0] - eye[0];
    float fy = center[1] - eye[1];
    float fz = center[2] - eye[2];
    float fl = sqrtf(fx*fx + fy*fy + fz*fz);
    if (fl > 1e-9f) { fx /= fl; fy /= fl; fz /= fl; }

    // side = f x up
    float sx = fy * up[2] - fz * up[1];
    float sy = fz * up[0] - fx * up[2];
    float sz = fx * up[1] - fy * up[0];
    float sl = sqrtf(sx*sx + sy*sy + sz*sz);
    if (sl > 1e-9f) { sx /= sl; sy /= sl; sz /= sl; }

    // u = s x f
    float ux = sy * fz - sz * fy;
    float uy = sz * fx - sx * fz;
    float uz = sx * fy - sy * fx;

    mat4_identity(m);
    m[0]  = sx;  m[4]  = sy;  m[8]  = sz;
    m[1]  = ux;  m[5]  = uy;  m[9]  = uz;
    m[2]  = -fx; m[6]  = -fy; m[10] = -fz;
    m[12] = -(sx*eye[0] + sy*eye[1] + sz*eye[2]);
    m[13] = -(ux*eye[0] + uy*eye[1] + uz*eye[2]);
    m[14] = (fx*eye[0] + fy*eye[1] + fz*eye[2]);
}

void camera_init(camera_t *cam)
{
    memset(cam, 0, sizeof(*cam));
    cam->azimuth   = -0.8f;   // slightly rotated
    cam->elevation =  0.6f;   // above horizon
    cam->distance  =  3.2f;
    cam->target[0] = 0.0f;
    cam->target[1] = 0.0f;
    cam->target[2] = 0.3f;    // center on hemisphere midpoint
    cam->dragging  = 0;
}

void camera_set_projection(camera_t *cam, float aspect, float fov_deg,
                           float near, float far)
{
    mat4_perspective(cam->proj, fov_deg * (float)M_PI / 180.0f,
                     aspect, near, far);
}

void camera_update(camera_t *cam)
{
    float az = cam->azimuth;
    float el = cam->elevation;
    float d  = cam->distance;

    float ce = cosf(el), se = sinf(el);
    float ca = cosf(az), sa = sinf(az);

    float eye[3] = {
        cam->target[0] + d * ce * ca,
        cam->target[1] + d * ce * sa,
        cam->target[2] + d * se,
    };

    float up[3] = { 0.0f, 0.0f, 1.0f };

    mat4_lookat(cam->view, eye, cam->target, up);
    mat4_mul(cam->mvp, cam->proj, cam->view);
}

void camera_on_mouse_button(camera_t *cam, int button, int action,
                            double mx, double my)
{
    // button 0 = left click
    if (button == 0)
    {
        if (action == 1) // press
        {
            cam->dragging = 1;
            cam->last_mx = mx;
            cam->last_my = my;
        }
        else // release
        {
            cam->dragging = 0;
        }
    }
}

void camera_on_cursor_move(camera_t *cam, double mx, double my)
{
    if (!cam->dragging) return;

    double dx = mx - cam->last_mx;
    double dy = my - cam->last_my;
    cam->last_mx = mx;
    cam->last_my = my;

    cam->azimuth   -= (float)dx * 0.005f;
    cam->elevation += (float)dy * 0.005f;

    // Clamp elevation to avoid gimbal flip
    float limit = (float)M_PI * 0.5f - 0.01f;
    if (cam->elevation >  limit) cam->elevation =  limit;
    if (cam->elevation < -limit) cam->elevation = -limit;
}

void camera_on_scroll(camera_t *cam, double yoffset)
{
    cam->distance -= (float)yoffset * 0.2f;
    if (cam->distance < 0.5f)  cam->distance = 0.5f;
    if (cam->distance > 10.0f) cam->distance = 10.0f;
}
