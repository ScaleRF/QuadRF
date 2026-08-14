// sphere_render.c
// Hemisphere mesh, FOV tile shader (black + outlines), point cloud, full sphere

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

#include "sphere_render.h"
#include "config.h"
#include "shutter_stream.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ----------------------------------------------------------------
// Shader helpers
// ----------------------------------------------------------------

static GLuint compile_shader(GLenum type, const char *src)
{
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(s, sizeof(log), NULL, log);
        fprintf(stderr, "Shader compile error:\n%s\n", log);
    }
    return s;
}

static GLuint link_program(GLuint vs, GLuint fs)
{
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    GLint ok;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(p, sizeof(log), NULL, log);
        fprintf(stderr, "Program link error:\n%s\n", log);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return p;
}

static GLuint build_shader(const char *vs_src, const char *fs_src)
{
    GLuint vs = compile_shader(GL_VERTEX_SHADER, vs_src);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fs_src);
    return link_program(vs, fs);
}

// ----------------------------------------------------------------
// Half-sphere mesh builder (reused for upper and lower)
// ----------------------------------------------------------------

static void build_half_sphere(GLuint *vao, GLuint *vbo, GLuint *ebo,
                              int *out_index_count, int upper)
{
    const int nlat = SPHERE_LAT_SEGS;
    const int nlon = SPHERE_LON_SEGS;
    int nverts = (nlat + 1) * (nlon + 1);
    int ntris = nlat * nlon * 2;

    float *verts = (float*)malloc((size_t)nverts * 3 * sizeof(float));
    unsigned int *indices = (unsigned int*)malloc((size_t)ntris * 3 * sizeof(unsigned int));

    int vi = 0;
    for (int lat = 0; lat <= nlat; ++lat)
    {
        // upper: theta 0 (zenith) to pi/2 (horizon)
        // lower: theta pi/2 (horizon) to pi (nadir)
        float theta;
        if (upper)
            theta = (float)lat / (float)nlat * ((float)M_PI * 0.5f);
        else
            theta = (float)M_PI * 0.5f +
                    (float)lat / (float)nlat * ((float)M_PI * 0.5f);

        float st = sinf(theta);
        float ct = cosf(theta);
        for (int lon = 0; lon <= nlon; ++lon)
        {
            float phi = (float)lon / (float)nlon * (float)M_PI * 2.0f;
            verts[vi*3+0] = st * cosf(phi);   // u
            verts[vi*3+1] = st * sinf(phi);   // v
            verts[vi*3+2] = ct;               // w
            vi++;
        }
    }

    int ii = 0;
    for (int lat = 0; lat < nlat; ++lat)
    {
        for (int lon = 0; lon < nlon; ++lon)
        {
            int a = lat * (nlon + 1) + lon;
            int b = a + nlon + 1;
            indices[ii++] = a;
            indices[ii++] = b;
            indices[ii++] = a + 1;
            indices[ii++] = a + 1;
            indices[ii++] = b;
            indices[ii++] = b + 1;
        }
    }
    *out_index_count = ii;

    glGenVertexArrays(1, vao);
    glBindVertexArray(*vao);

    glGenBuffers(1, vbo);
    glBindBuffer(GL_ARRAY_BUFFER, *vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(nverts * 3 * sizeof(float)),
                 verts, GL_STATIC_DRAW);

    glGenBuffers(1, ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(ii * sizeof(unsigned int)),
                 indices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindVertexArray(0);

    free(verts);
    free(indices);
}

// ----------------------------------------------------------------
// Floor disk
// ----------------------------------------------------------------

static void build_floor(sphere_state_t *s)
{
    const int segs = 64;
    const float radius = 1.15f;
    int nverts = segs + 2;
    float *verts = (float*)malloc((size_t)nverts * 3 * sizeof(float));

    verts[0] = 0.0f; verts[1] = 0.0f; verts[2] = 0.0f;
    for (int i = 0; i <= segs; ++i)
    {
        float angle = (float)i / (float)segs * (float)M_PI * 2.0f;
        verts[(i+1)*3+0] = radius * cosf(angle);
        verts[(i+1)*3+1] = radius * sinf(angle);
        verts[(i+1)*3+2] = 0.0f;
    }
    s->floor_vert_count = nverts;

    glGenVertexArrays(1, &s->floor_vao);
    glBindVertexArray(s->floor_vao);

    glGenBuffers(1, &s->floor_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, s->floor_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(nverts * 3 * sizeof(float)),
                 verts, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindVertexArray(0);
    free(verts);
}

// ----------------------------------------------------------------
// Axes (6 verts: 3 line segments)
// ----------------------------------------------------------------

static void build_axes(sphere_state_t *s)
{
    float data[] = {
        0,0,0, 1,0,0,   1.4f,0,0, 1,0,0,
        0,0,0, 0,1,0,   0,1.4f,0, 0,1,0,
        0,0,0, 0,0,1,   0,0,1.4f, 0,0,1,
    };

    glGenVertexArrays(1, &s->axes_vao);
    glBindVertexArray(s->axes_vao);

    glGenBuffers(1, &s->axes_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, s->axes_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(data), data, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void*)(3 * sizeof(float)));

    glBindVertexArray(0);
}

// ----------------------------------------------------------------
// Shaders
// ----------------------------------------------------------------

// Hemisphere: black fill, thin light outlines at tile boundaries
static const char *hemi_vs_src =
    "#version 140\n"
    "in vec3 aPos;\n"
    "uniform mat4 uMVP;\n"
    "out vec3 vPosition;\n"
    "void main() {\n"
    "    vPosition = aPos;\n"
    "    gl_Position = uMVP * vec4(aPos, 1.0);\n"
    "}\n";

static const char *hemi_fs_src =
    "#version 140\n"
    "#define PI 3.1415926535897932\n"
    "in vec3 vPosition;\n"
    "out vec4 FragColor;\n"
    "uniform float uScaleFactor;\n"
    "uniform float uScaleFactorLow;\n"
    "uniform float uScaleFactorHigh;\n"
    "\n"
    "void main() {\n"
    "    float u = vPosition.x;\n"
    "    float v = vPosition.y;\n"
    "    float w = vPosition.z;\n"
    "\n"
    "    float gx = u * uScaleFactor;\n"
    "    float gy = v * uScaleFactor;\n"
    "\n"
    "    vec2 r1 = vec2(4.0 * PI / sqrt(3.0), 0.0);\n"
    "    vec2 r2 = vec2(2.0 * PI / sqrt(3.0), 2.0 * PI);\n"
    "\n"
    "    float minDist1 = 1e20;\n"
    "    float minDist2 = 1e20;\n"
    "    float minDistNonZero = 1e20;\n"
    "    vec2 best_n = vec2(0.0);\n"
    "\n"
    "    for (float n1 = -3.0; n1 <= 3.0; n1++) {\n"
    "        for (float n2 = -3.0; n2 <= 3.0; n2++) {\n"
    "            vec2 lattice = n1 * r1 + n2 * r2;\n"
    "            vec2 diff = vec2(gx, gy) - lattice;\n"
    "            float dist = dot(diff, diff);\n"
    "            if (!(n1 == 0.0 && n2 == 0.0)) {\n"
    "                minDistNonZero = min(minDistNonZero, dist);\n"
    "            }\n"
    "            if (dist < minDist1) {\n"
    "                minDist2 = minDist1;\n"
    "                minDist1 = dist;\n"
    "                best_n = vec2(n1, n2);\n"
    "            } else if (dist < minDist2) {\n"
    "                minDist2 = dist;\n"
    "            }\n"
    "        }\n"
    "    }\n"
    "\n"
    // Tile boundary outlines
    "    float edgeDist = sqrt(minDist2) - sqrt(minDist1);\n"
    "    float lineThickness = 0.08;\n"
    "    float lineSmooth = fwidth(edgeDist) * 1.5;\n"
    "    float lineMix = 1.0 - smoothstep(lineThickness - lineSmooth,\n"
    "                                      lineThickness + lineSmooth, edgeDist);\n"
    "\n"
    "    vec3 lineColor;\n"
    "    if (best_n.x == 0.0 && best_n.y == 0.0) {\n"
    "        lineColor = vec3(0.35, 0.65, 0.35);\n"
    "    } else {\n"
    "        lineColor = vec3(0.22, 0.22, 0.22);\n"
    "    }\n"
    "\n"
    "    float shade = 0.015 * smoothstep(0.0, 1.0, abs(w));\n"
    "    vec3 base = vec3(shade);\n"
    "    vec3 color = mix(base, lineColor, lineMix);\n"
    "\n"
    // Deterministic region boundaries: show low-freq (blue) and high-freq (orange-red)
    // bounds of the current sweep. Inside both lines = unambiguous at all sweep freqs.
    // Inside only the blue line = ambiguous at the highest frequencies.
    "    if (best_n.x == 0.0 && best_n.y == 0.0) {\n"
    "        float threshLow = uScaleFactorLow * uScaleFactorLow;\n"
    "        float edgeLow   = minDistNonZero - threshLow;\n"
    "        float smLow     = fwidth(edgeLow) * 2.0;\n"
    "        float lineLow   = 1.0 - smoothstep(-smLow, smLow, abs(edgeLow) - 0.3);\n"
    "        color = mix(color, vec3(0.15, 0.45, 0.70), lineLow * 0.9);\n"
    "        float threshHigh = uScaleFactorHigh * uScaleFactorHigh;\n"
    "        float edgeHigh   = minDistNonZero - threshHigh;\n"
    "        float smHigh     = fwidth(edgeHigh) * 2.0;\n"
    "        float lineHigh   = 1.0 - smoothstep(-smHigh, smHigh, abs(edgeHigh) - 0.3);\n"
    "        color = mix(color, vec3(0.75, 0.25, 0.10), lineHigh * 0.9);\n"
    "    }\n"
    "\n"
    "    FragColor = vec4(color, 0.85);\n"
    "}\n";

// Point cloud: positions on sphere from (gx, gy) + optional lattice offset.
// When uFlat = 1, bypass the unit-sphere projection and place the point
// directly at uOrthoScale * (gx, gy) in clip space — used by the viewfinder
// "flat fill" sub-mode so the deterministic hex maps 1:1 onto the screen.
static const char *pt_vs_src =
    "#version 140\n"
    "in vec2 aGradient;\n"
    "in vec4 aColor;\n"
    "uniform mat4 uMVP;\n"
    "uniform float uScaleFactor;\n"
    "uniform float uRadiusOffset;\n"
    "uniform float uPointSize;\n"
    "uniform float uGain;\n"
    "uniform float uFlipW;\n"
    "uniform float uSizeFloor;\n"
    "uniform vec2 uGradientOffset;\n"
    "uniform int  uFlat;\n"
    "uniform vec2 uOrthoScale;\n"
    "out vec4 vColor;\n"
    "void main() {\n"
    "    if (uFlat == 1) {\n"
    "        vec2 p = (aGradient + uGradientOffset) * uOrthoScale;\n"
    "        gl_Position = vec4(p, 0.0, 1.0);\n"
    "        float intensity = clamp(abs(aColor.w) * uGain, 0.0, 1.0);\n"
    "        gl_PointSize = uPointSize * (uSizeFloor + (1.0 - uSizeFloor) * intensity);\n"
    "        vColor = vec4(aColor.rgb, intensity);\n"
    "        return;\n"
    "    }\n"
    "    float u = (aGradient.x + uGradientOffset.x) / uScaleFactor;\n"
    "    float v = (aGradient.y + uGradientOffset.y) / uScaleFactor;\n"
    "    float r2 = u*u + v*v;\n"
    "    if (r2 > 1.0) {\n"
    "        gl_Position = vec4(0.0, 0.0, -2.0, 1.0);\n"
    "        gl_PointSize = 0.0;\n"
    "        vColor = vec4(0.0);\n"
    "        return;\n"
    "    }\n"
    "    float w = sqrt(1.0 - r2) * sign(aColor.w) * uFlipW;\n"
    "    vec3 pos = vec3(u, v, w) * uRadiusOffset;\n"
    "    gl_Position = uMVP * vec4(pos, 1.0);\n"
    "    float intensity = clamp(abs(aColor.w) * uGain, 0.0, 1.0);\n"
    "    gl_PointSize = uPointSize * (uSizeFloor + (1.0 - uSizeFloor) * intensity);\n"
    "    vColor = vec4(aColor.rgb, intensity);\n"
    "}\n";

static const char *pt_fs_src =
    "#version 140\n"
    "in vec4 vColor;\n"
    "uniform float uGaussSigma;\n"
    "uniform float uMinAlpha;\n"
    "out vec4 FragColor;\n"
    "void main() {\n"
    "    vec2 pc = gl_PointCoord - vec2(0.5);\n"
    "    float r2 = dot(pc, pc);\n"
    "    if (uGaussSigma > 0.0) {\n"
    "        float g = exp(-r2 / (2.0 * uGaussSigma * uGaussSigma));\n"
    "        float peak = clamp(vColor.w + uMinAlpha, 0.0, 1.0);\n"
    "        FragColor = vec4(vColor.rgb, peak * g);\n"
    "    } else {\n"
    "        if (r2 > 0.25) discard;\n"
    "        float a = vColor.w * smoothstep(0.25, 0.05, r2);\n"
    "        FragColor = vec4(vColor.rgb, a);\n"
    "    }\n"
    "}\n";

// Floor shader
static const char *floor_vs_src =
    "#version 140\n"
    "in vec3 aPos;\n"
    "uniform mat4 uMVP;\n"
    "void main() {\n"
    "    gl_Position = uMVP * vec4(aPos, 1.0);\n"
    "}\n";

static const char *floor_fs_src =
    "#version 140\n"
    "out vec4 FragColor;\n"
    "void main() {\n"
    "    FragColor = vec4(0.05, 0.05, 0.05, 1.0);\n"
    "}\n";

// Axes shader
static const char *axes_vs_src =
    "#version 140\n"
    "in vec3 aPos;\n"
    "in vec3 aCol;\n"
    "uniform mat4 uMVP;\n"
    "out vec3 vCol;\n"
    "void main() {\n"
    "    gl_Position = uMVP * vec4(aPos, 1.0);\n"
    "    vCol = aCol;\n"
    "}\n";

static const char *axes_fs_src =
    "#version 140\n"
    "in vec3 vCol;\n"
    "out vec4 FragColor;\n"
    "void main() {\n"
    "    FragColor = vec4(vCol, 0.5);\n"
    "}\n";

// Hex reticle: 6 vertices in gradient space (gx, gy) drawn as GL_LINE_LOOP.
// Shares the two-projection branch with the point shader so the reticle
// follows whichever sub-mode the viewfinder is in (flat ortho or pinhole).
static const char *hex_vs_src =
    "#version 140\n"
    "in vec2 aGradient;\n"
    "uniform mat4 uMVP;\n"
    "uniform int  uFlat;\n"
    "uniform vec2 uOrthoScale;\n"
    "uniform float uScaleFactor;\n"
    "uniform float uRadiusOffset;\n"
    "void main() {\n"
    "    if (uFlat == 1) {\n"
    "        vec2 p = aGradient * uOrthoScale;\n"
    "        gl_Position = vec4(p, 0.0, 1.0);\n"
    "        return;\n"
    "    }\n"
    "    float u = aGradient.x / uScaleFactor;\n"
    "    float v = aGradient.y / uScaleFactor;\n"
    "    float r2 = u*u + v*v;\n"
    "    if (r2 > 1.0) {\n"
    // Clamp to the rim so vertices outside the unit disk still land on
    // the sphere instead of being clipped to the back of the view.
    "        float s = inversesqrt(r2);\n"
    "        u *= s; v *= s; r2 = 1.0;\n"
    "    }\n"
    "    float w = sqrt(1.0 - r2);\n"
    "    vec3 pos = vec3(u, v, w) * uRadiusOffset;\n"
    "    gl_Position = uMVP * vec4(pos, 1.0);\n"
    "}\n";

static const char *hex_fs_src =
    "#version 140\n"
    "uniform vec4 uColor;\n"
    "out vec4 FragColor;\n"
    "void main() {\n"
    "    FragColor = uColor;\n"
    "}\n";

// ----------------------------------------------------------------
// Hex reticle VBO (deterministic Voronoi cell outline, gradient space)
// ----------------------------------------------------------------

static void build_hex_reticle(sphere_state_t *s)
{
    // Flat-top hex in (gx, gy) matching is_deterministic() in utils.c:
    // vertices at (+/- 2*pi/sqrt(3), 0) and (+/- pi/sqrt(3), +/- pi).
    const float pi = (float)M_PI;
    const float R  = 2.0f * pi / sqrtf(3.0f);
    const float H  = pi;
    const float Rh = pi / sqrtf(3.0f);

    // Sub-divide each edge so the line bends nicely with the spherical
    // projection in the pinhole sub-mode. 12 segments per side is plenty.
    enum { SEGS = 12 };
    const float vx[6] = {  R,   Rh,  -Rh,  -R,  -Rh,   Rh };
    const float vy[6] = {  0,    H,    H,   0,   -H,   -H };

    int n = 6 * SEGS;
    float *verts = (float*)malloc((size_t)n * 2 * sizeof(float));
    int k = 0;
    for (int i = 0; i < 6; ++i) {
        int j = (i + 1) % 6;
        for (int t = 0; t < SEGS; ++t) {
            float a = (float)t / (float)SEGS;
            verts[k*2+0] = vx[i] + (vx[j] - vx[i]) * a;
            verts[k*2+1] = vy[i] + (vy[j] - vy[i]) * a;
            k++;
        }
    }

    glGenVertexArrays(1, &s->hex_vao);
    glBindVertexArray(s->hex_vao);

    glGenBuffers(1, &s->hex_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, s->hex_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(n * 2 * sizeof(float)),
                 verts, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

    glBindVertexArray(0);
    free(verts);
}

// ----------------------------------------------------------------
// Centroid crosshair VBO (dynamic; 4 verts = 2 line segments)
// ----------------------------------------------------------------

static void build_centroid_vbo(sphere_state_t *s)
{
    glGenVertexArrays(1, &s->cent_vao);
    glBindVertexArray(s->cent_vao);

    glGenBuffers(1, &s->cent_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, s->cent_vbo);
    // 4 verts, 2 floats each
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(4 * 2 * sizeof(float)),
                 NULL, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE,
                          2 * sizeof(float), (void*)0);

    glBindVertexArray(0);
}

// ----------------------------------------------------------------
// Point cloud VBO (dynamic)
// ----------------------------------------------------------------

static void build_point_vbo(sphere_state_t *s)
{
    glGenVertexArrays(1, &s->pt_vao);
    glBindVertexArray(s->pt_vao);

    glGenBuffers(1, &s->pt_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, s->pt_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(MAX_RENDER_POINTS * 6 * sizeof(float)),
                 NULL, GL_DYNAMIC_DRAW);

    // aGradient = (gx, gy)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);

    // aColor = (r, g, b, intensity)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void*)(2 * sizeof(float)));

    glBindVertexArray(0);
}

// ----------------------------------------------------------------
// Public API
// ----------------------------------------------------------------

void sphere_init(sphere_state_t *s)
{
    memset(s, 0, sizeof(*s));

    build_half_sphere(&s->hemi_vao, &s->hemi_vbo, &s->hemi_ebo,
                      &s->hemi_index_count, 1);
    build_half_sphere(&s->hemi_lower_vao, &s->hemi_lower_vbo,
                      &s->hemi_lower_ebo, &s->hemi_lower_index_count, 0);

    build_floor(s);
    build_axes(s);
    build_point_vbo(s);
    build_hex_reticle(s);
    build_centroid_vbo(s);

    s->hemi_shader = build_shader(hemi_vs_src, hemi_fs_src);
    s->hemi_u_mvp = glGetUniformLocation(s->hemi_shader, "uMVP");
    s->hemi_u_scale_factor      = glGetUniformLocation(s->hemi_shader, "uScaleFactor");
    s->hemi_u_scale_factor_low  = glGetUniformLocation(s->hemi_shader, "uScaleFactorLow");
    s->hemi_u_scale_factor_high = glGetUniformLocation(s->hemi_shader, "uScaleFactorHigh");

    s->pt_shader = build_shader(pt_vs_src, pt_fs_src);
    s->pt_u_mvp = glGetUniformLocation(s->pt_shader, "uMVP");
    s->pt_u_scale_factor = glGetUniformLocation(s->pt_shader, "uScaleFactor");
    s->pt_u_radius_offset = glGetUniformLocation(s->pt_shader, "uRadiusOffset");
    s->pt_u_point_size = glGetUniformLocation(s->pt_shader, "uPointSize");
    s->pt_u_gain = glGetUniformLocation(s->pt_shader, "uGain");
    s->pt_u_flip_w = glGetUniformLocation(s->pt_shader, "uFlipW");
    s->pt_u_gradient_offset = glGetUniformLocation(s->pt_shader, "uGradientOffset");
    s->pt_u_gauss_sigma = glGetUniformLocation(s->pt_shader, "uGaussSigma");
    s->pt_u_min_alpha   = glGetUniformLocation(s->pt_shader, "uMinAlpha");
    s->pt_u_size_floor  = glGetUniformLocation(s->pt_shader, "uSizeFloor");
    s->pt_u_flat        = glGetUniformLocation(s->pt_shader, "uFlat");
    s->pt_u_ortho_scale = glGetUniformLocation(s->pt_shader, "uOrthoScale");

    s->floor_shader = build_shader(floor_vs_src, floor_fs_src);
    s->floor_u_mvp = glGetUniformLocation(s->floor_shader, "uMVP");

    s->axes_shader = build_shader(axes_vs_src, axes_fs_src);
    s->axes_u_mvp = glGetUniformLocation(s->axes_shader, "uMVP");

    s->hex_shader = build_shader(hex_vs_src, hex_fs_src);
    s->hex_u_mvp           = glGetUniformLocation(s->hex_shader, "uMVP");
    s->hex_u_flat          = glGetUniformLocation(s->hex_shader, "uFlat");
    s->hex_u_ortho_scale   = glGetUniformLocation(s->hex_shader, "uOrthoScale");
    s->hex_u_scale_factor  = glGetUniformLocation(s->hex_shader, "uScaleFactor");
    s->hex_u_radius_offset = glGetUniformLocation(s->hex_shader, "uRadiusOffset");
    s->hex_u_color         = glGetUniformLocation(s->hex_shader, "uColor");

    s->history_capacity = MAX_RENDER_POINTS;
    s->history = (render_point_t*)calloc((size_t)s->history_capacity,
                                         sizeof(render_point_t));
    s->history_count = 0;

    // Shutter mode: bin grid is allocated lazily on first sphere_shutter_begin.
    // Raw points stream to flash via shutter_stream.c — no big RAM buffer here.
    s->shutter_total_points = 0;
    s->shutter_full         = 0;
    s->shutter_grid         = NULL;
    s->shutter_active       = 0;
    s->shutter_dirty_frames = 0;
}

void sphere_destroy(sphere_state_t *s)
{
    glDeleteVertexArrays(1, &s->hemi_vao);
    glDeleteBuffers(1, &s->hemi_vbo);
    glDeleteBuffers(1, &s->hemi_ebo);

    glDeleteVertexArrays(1, &s->hemi_lower_vao);
    glDeleteBuffers(1, &s->hemi_lower_vbo);
    glDeleteBuffers(1, &s->hemi_lower_ebo);

    glDeleteProgram(s->hemi_shader);

    glDeleteVertexArrays(1, &s->pt_vao);
    glDeleteBuffers(1, &s->pt_vbo);
    glDeleteProgram(s->pt_shader);

    glDeleteVertexArrays(1, &s->floor_vao);
    glDeleteBuffers(1, &s->floor_vbo);
    glDeleteProgram(s->floor_shader);

    glDeleteVertexArrays(1, &s->axes_vao);
    glDeleteBuffers(1, &s->axes_vbo);
    glDeleteProgram(s->axes_shader);

    glDeleteVertexArrays(1, &s->hex_vao);
    glDeleteBuffers(1, &s->hex_vbo);
    glDeleteProgram(s->hex_shader);

    glDeleteVertexArrays(1, &s->cent_vao);
    glDeleteBuffers(1, &s->cent_vbo);

    free(s->history);
    free(s->shutter_grid);
}

void sphere_update_points(sphere_state_t *s,
                          const point_data_t *new_pts, int n_new,
                          float decay_factor, float decay_threshold)
{
    int alive = 0;
    for (int i = 0; i < s->history_count; ++i)
    {
        s->history[i].intensity *= decay_factor;
        if (fabsf(s->history[i].intensity) >= decay_threshold)
            s->history[alive++] = s->history[i];
    }
    s->history_count = alive;

    int room = s->history_capacity - s->history_count;
    int base = s->history_count;
    int added = 0;
    for (int i = 0; i < n_new && added < room; ++i)
    {
        if (fabsf(new_pts[i].intensity) < decay_threshold)
            continue;
        render_point_t *rp = &s->history[base + added];
        rp->gx = new_pts[i].gx;
        rp->gy = new_pts[i].gy;
        rp->r  = new_pts[i].r;
        rp->g  = new_pts[i].g;
        rp->b  = new_pts[i].b;
        rp->intensity = new_pts[i].intensity;
        added++;
    }
    s->history_count = base + added;

    if (s->history_count > 0)
    {
        float *buf = (float*)malloc((size_t)s->history_count * 6 * sizeof(float));
        for (int i = 0; i < s->history_count; ++i)
        {
            render_point_t *rp = &s->history[i];
            buf[i*6+0] = rp->gx;
            buf[i*6+1] = rp->gy;
            buf[i*6+2] = rp->r;
            buf[i*6+3] = rp->g;
            buf[i*6+4] = rp->b;
            buf[i*6+5] = rp->intensity;
        }
        glBindBuffer(GL_ARRAY_BUFFER, s->pt_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        (GLsizeiptr)(s->history_count * 6 * sizeof(float)), buf);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        free(buf);
    }
}

void sphere_render(const sphere_state_t *s, const float *mvp,
                   float point_size, float point_gain,
                   int show_lower, int show_mirrors,
                   float lo_start_mhz, float lo_end_mhz,
                   const sphere_view_opts_t *view)
{
    float sf_center = SCALE_FACTOR_AT_MHZ((lo_start_mhz + lo_end_mhz) * 0.5f);
    float sf_low    = SCALE_FACTOR_AT_MHZ(lo_start_mhz);
    float sf_high   = SCALE_FACTOR_AT_MHZ(lo_end_mhz);

    int  vf      = (view && view->viewfinder) ? 1 : 0;
    int  vf_flat = vf && view->flat ? 1 : 0;
    float ortho_sx = vf ? view->ortho_sx : 1.0f;
    float ortho_sy = vf ? view->ortho_sy : 1.0f;

    // In viewfinder mode we're "inside" the antenna sphere looking out, so
    // the floor disk, world axes, and hemisphere mesh would all be in front
    // of the camera and obscure the view. Skip them entirely. Depth testing
    // also stops mattering -- the only geometry is the reticle and points.
    if (vf) {
        glDisable(GL_DEPTH_TEST);
    }

    if (!vf) {
        // Floor
        glUseProgram(s->floor_shader);
        glUniformMatrix4fv(s->floor_u_mvp, 1, GL_FALSE, mvp);
        glBindVertexArray(s->floor_vao);
        glDrawArrays(GL_TRIANGLE_FAN, 0, s->floor_vert_count);

        // Axes
        glUseProgram(s->axes_shader);
        glUniformMatrix4fv(s->axes_u_mvp, 1, GL_FALSE, mvp);
        glBindVertexArray(s->axes_vao);
        glLineWidth(2.0f);
        glDrawArrays(GL_LINES, 0, 6);
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (!vf) {
        // Hemisphere (semi-transparent outlines on black)
        glUseProgram(s->hemi_shader);
        glUniformMatrix4fv(s->hemi_u_mvp, 1, GL_FALSE, mvp);
        glUniform1f(s->hemi_u_scale_factor,      sf_center);
        glUniform1f(s->hemi_u_scale_factor_low,  sf_low);
        glUniform1f(s->hemi_u_scale_factor_high, sf_high);

        // Upper hemisphere (always drawn)
        glBindVertexArray(s->hemi_vao);
        glDrawElements(GL_TRIANGLES, s->hemi_index_count, GL_UNSIGNED_INT, 0);

        // Lower hemisphere (conditional)
        if (show_lower)
        {
            glBindVertexArray(s->hemi_lower_vao);
            glDrawElements(GL_TRIANGLES, s->hemi_lower_index_count,
                           GL_UNSIGNED_INT, 0);
        }
    }

    // Hex reticle (viewfinder only). Drawn before points so points overlay it.
    if (vf && (!view || view->show_hex_reticle)) {
        glUseProgram(s->hex_shader);
        glUniformMatrix4fv(s->hex_u_mvp, 1, GL_FALSE, mvp);
        glUniform1i(s->hex_u_flat, vf_flat);
        glUniform2f(s->hex_u_ortho_scale, ortho_sx, ortho_sy);
        glUniform1f(s->hex_u_scale_factor, sf_center);
        glUniform1f(s->hex_u_radius_offset, POINT_RADIUS_OFFSET);
        glUniform4f(s->hex_u_color, 0.35f, 0.65f, 0.35f, 0.9f);
        glBindVertexArray(s->hex_vao);
        glLineWidth(2.0f);
        glDrawArrays(GL_LINE_LOOP, 0, 6 * 12);
    }

    // Points. Normal mode uses additive blending for the glow effect.
    // Shutter mode switches to standard alpha blending with depth-write off
    // so each binned splat reads as a solid blob head-on, instead of looking
    // see-through head-on and only "lighting up" at glancing angles where
    // additive overlap stacks contributions. Viewfinder mode follows the
    // additive look since it always uses the decay-history buffer (the bin
    // grid is bypassed in viewfinder mode).
    if (s->history_count > 0)
    {
        int use_shutter_look = s->shutter_active && !vf;
        if (use_shutter_look) {
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
        } else {
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        }
        glEnable(GL_PROGRAM_POINT_SIZE);

        glUseProgram(s->pt_shader);
        glUniformMatrix4fv(s->pt_u_mvp, 1, GL_FALSE, mvp);
        glUniform1f(s->pt_u_scale_factor, sf_center);
        glUniform1f(s->pt_u_radius_offset, POINT_RADIUS_OFFSET);
        glUniform1i(s->pt_u_flat, vf_flat);
        glUniform2f(s->pt_u_ortho_scale, ortho_sx, ortho_sy);

        if (use_shutter_look) {
            // intensity is already soft-saturated to [-1, 1] by
            // sphere_shutter_rebuild_display; passing gain=1 avoids re-
            // scaling and re-clamping which would collapse the dynamic
            // range we want to show as accumulation.
            glUniform1f(s->pt_u_gain,        1.0f);
            glUniform1f(s->pt_u_point_size,  SHUTTER_POINT_SIZE);
            glUniform1f(s->pt_u_gauss_sigma, SHUTTER_GAUSS_SIGMA);
            glUniform1f(s->pt_u_min_alpha,   SHUTTER_MIN_ALPHA);
            glUniform1f(s->pt_u_size_floor,  SHUTTER_POINT_SIZE_FLOOR);
        } else {
            glUniform1f(s->pt_u_gain,        point_gain);
            glUniform1f(s->pt_u_point_size,  point_size);
            glUniform1f(s->pt_u_gauss_sigma, 0.0f);
            glUniform1f(s->pt_u_min_alpha,   0.0f);
            glUniform1f(s->pt_u_size_floor,  0.3f);
        }

        glBindVertexArray(s->pt_vao);

        // Reciprocal lattice basis for mirror tile offsets
        const float r1x = 4.0f * (float)M_PI / sqrtf(3.0f);
        const float r2x = 2.0f * (float)M_PI / sqrtf(3.0f);
        const float r2y = 2.0f * (float)M_PI;

        // In viewfinder mode the mirror tiles and lower hemisphere don't
        // make sense (we're staring straight at the central cell). Force
        // a single draw of the primary cell, upper hemisphere only.
        const int R = (show_mirrors && !vf) ? MIRROR_SEARCH_RANGE : 0;
        const int draw_lower = (show_lower && !vf);

        for (int n1 = -R; n1 <= R; ++n1)
        {
            for (int n2 = -R; n2 <= R; ++n2)
            {
                float ox = (float)n1 * r1x + (float)n2 * r2x;
                float oy = (float)n2 * r2y;
                glUniform2f(s->pt_u_gradient_offset, ox, oy);

                glUniform1f(s->pt_u_flip_w, 1.0f);
                glDrawArrays(GL_POINTS, 0, s->history_count);

                if (draw_lower)
                {
                    glUniform1f(s->pt_u_flip_w, -1.0f);
                    glDrawArrays(GL_POINTS, 0, s->history_count);
                }
            }
        }

        glDisable(GL_PROGRAM_POINT_SIZE);

        if (use_shutter_look) {
            glDepthMask(GL_TRUE);
        }
    }

    // Centroid crosshair overlay. Only drawn when explicitly requested by
    // the caller (phase-calibration mode) and only in viewfinder mode --
    // outside the viewfinder the (gx, gy) coordinate doesn't map directly
    // to a stable on-screen position, so the crosshair would be misleading.
    if (view && view->show_centroid && vf) {
        float gx = view->centroid_gx;
        float gy = view->centroid_gy;
        // Crosshair half-extent in (gx, gy) units. Picked so the cross is
        // visible but small relative to the hex reticle (R = 2*pi/sqrt(3) ~ 3.6).
        float ext = 0.45f;
        float verts[8] = {
            gx - ext, gy,        gx + ext, gy,
            gx,       gy - ext,  gx,       gy + ext,
        };

        glBindBuffer(GL_ARRAY_BUFFER, s->cent_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(verts), verts);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glUseProgram(s->hex_shader);
        glUniformMatrix4fv(s->hex_u_mvp, 1, GL_FALSE, mvp);
        glUniform1i(s->hex_u_flat, vf_flat);
        glUniform2f(s->hex_u_ortho_scale, ortho_sx, ortho_sy);
        glUniform1f(s->hex_u_scale_factor, sf_center);
        glUniform1f(s->hex_u_radius_offset, POINT_RADIUS_OFFSET);
        // Yellow, slightly transparent so background points show through.
        glUniform4f(s->hex_u_color, 1.0f, 0.9f, 0.15f, 0.95f);

        glBindVertexArray(s->cent_vao);
        glLineWidth(3.0f);
        glDrawArrays(GL_LINES, 0, 4);
    }

    glDisable(GL_BLEND);
    glBindVertexArray(0);
    glUseProgram(0);

    if (vf) {
        glEnable(GL_DEPTH_TEST);
    }
}

// ----------------------------------------------------------------
// Shutter mode (long exposure)
// ----------------------------------------------------------------

// Map a gx (or gy) value to a [0, SHUTTER_GRID_RES-1] bin index.
// Returns -1 if the value is outside the grid extent.
static inline int shutter_bin_index(float v)
{
    float t = (v + SHUTTER_GRID_HALFRANGE) / (2.0f * SHUTTER_GRID_HALFRANGE);
    if (t < 0.0f || t >= 1.0f) return -1;
    int i = (int)(t * (float)SHUTTER_GRID_RES);
    if (i < 0) i = 0;
    if (i >= SHUTTER_GRID_RES) i = SHUTTER_GRID_RES - 1;
    return i;
}

void sphere_shutter_begin(sphere_state_t *s)
{
    if (!s->shutter_grid) {
        size_t ncells = (size_t)SHUTTER_GRID_RES * (size_t)SHUTTER_GRID_RES;
        s->shutter_grid = (shutter_bin_t*)malloc(ncells * sizeof(shutter_bin_t));
        if (!s->shutter_grid) {
            fprintf(stderr, "shutter: failed to alloc grid (%zu cells)\n", ncells);
        }
    }

    s->shutter_total_points = 0;
    s->shutter_full         = 0;
    s->shutter_dirty_frames = 0;
    s->shutter_active       = 1;

    if (s->shutter_grid) {
        size_t ncells = (size_t)SHUTTER_GRID_RES * (size_t)SHUTTER_GRID_RES;
        memset(s->shutter_grid, 0, ncells * sizeof(shutter_bin_t));
    }

    s->history_count = 0;
}

void sphere_shutter_add(sphere_state_t *s, const point_data_t *pts, int n)
{
    if (!s->shutter_active || n <= 0 || !pts) return;
    if (!s->shutter_grid) return;

    // Pack render_point_t records on the stack for the stream push. We need
    // a contiguous render_point_t array; the worker gives us point_data_t,
    // which has the same field layout but the struct types differ.
    enum { CHUNK = 4096 };
    render_point_t tmp[CHUNK];
    int dropped_any = 0;

    int i = 0;
    while (i < n) {
        int take = (n - i) > CHUNK ? CHUNK : (n - i);
        for (int k = 0; k < take; ++k) {
            const point_data_t *src = &pts[i + k];
            tmp[k].gx        = src->gx;
            tmp[k].gy        = src->gy;
            tmp[k].r         = src->r;
            tmp[k].g         = src->g;
            tmp[k].b         = src->b;
            tmp[k].intensity = src->intensity;

            int ix = shutter_bin_index(src->gx);
            int iy = shutter_bin_index(src->gy);
            if (ix < 0 || iy < 0) continue;
            shutter_bin_t *bin = &s->shutter_grid[iy * SHUTTER_GRID_RES + ix];
            bin->gx_sum        += src->gx;
            bin->gy_sum        += src->gy;
            bin->r_sum         += src->r;
            bin->g_sum         += src->g;
            bin->b_sum         += src->b;
            bin->intensity_sum += src->intensity;
            bin->count         += 1;
        }

        int chunk_dropped = 0;
        shutter_stream_push(tmp, take, &chunk_dropped);
        if (chunk_dropped) dropped_any = 1;

        i += take;
    }

    s->shutter_total_points += (uint64_t)n;
    if (dropped_any) s->shutter_full = 1;
}

void sphere_shutter_rebuild_display(sphere_state_t *s)
{
    if (!s->shutter_grid || !s->history) return;

    int out = 0;
    int cap = s->history_capacity;
    size_t ncells = (size_t)SHUTTER_GRID_RES * (size_t)SHUTTER_GRID_RES;
    for (size_t c = 0; c < ncells && out < cap; ++c) {
        shutter_bin_t *bin = &s->shutter_grid[c];
        if (bin->count == 0) continue;
        float inv_n = 1.0f / (float)bin->count;
        float gx_avg = bin->gx_sum * inv_n;
        float gy_avg = bin->gy_sum * inv_n;
        float r_out  = bin->r_sum  * inv_n;
        float g_out  = bin->g_sum  * inv_n;
        float b_out  = bin->b_sum  * inv_n;

        // intensity_sum is the running sum of all contributions; preserve
        // sign for lower-hemisphere handling. Use a Naka-Rushton soft
        // saturation (x / (x + K)) instead of a hard clamp so a bin that
        // keeps getting hits keeps growing visibly toward 1.0 rather than
        // slamming the ceiling on the first few samples. Cost: one divide
        // per non-empty bin per rebuild (every N frames). Free in practice.
        float i_sum  = bin->intensity_sum;
        float i_abs  = fabsf(i_sum);
        float i_norm = i_abs / (i_abs + SHUTTER_SATURATION_K);
        float i_out  = (i_sum >= 0.0f) ? i_norm : -i_norm;

        render_point_t *rp = &s->history[out++];
        rp->gx        = gx_avg;
        rp->gy        = gy_avg;
        rp->r         = r_out;
        rp->g         = g_out;
        rp->b         = b_out;
        rp->intensity = i_out;
    }
    s->history_count = out;

    if (out > 0) {
        float *buf = (float*)malloc((size_t)out * 6 * sizeof(float));
        if (buf) {
            for (int i = 0; i < out; ++i) {
                render_point_t *rp = &s->history[i];
                buf[i*6+0] = rp->gx;
                buf[i*6+1] = rp->gy;
                buf[i*6+2] = rp->r;
                buf[i*6+3] = rp->g;
                buf[i*6+4] = rp->b;
                buf[i*6+5] = rp->intensity;
            }
            glBindBuffer(GL_ARRAY_BUFFER, s->pt_vbo);
            glBufferSubData(GL_ARRAY_BUFFER, 0,
                            (GLsizeiptr)(out * 6 * sizeof(float)), buf);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            free(buf);
        }
    }
}

void sphere_shutter_end(sphere_state_t *s)
{
    s->shutter_active       = 0;
    s->shutter_dirty_frames = 0;
    // shutter_grid, shutter_full, shutter_total_points kept until the next
    // begin so the UI can show the final count and the bin grid can be
    // reused without reallocating.
}

// Stream raw points to the shutter writer without updating the bin grid.
// Used by viewfinder mode: the display is driven by the normal decay
// history buffer (sphere_update_points) but disk recording must continue.
void sphere_shutter_stream_only(sphere_state_t *s, const point_data_t *pts, int n)
{
    if (!s->shutter_active || n <= 0 || !pts) return;

    enum { CHUNK = 4096 };
    render_point_t tmp[CHUNK];
    int dropped_any = 0;

    int i = 0;
    while (i < n) {
        int take = (n - i) > CHUNK ? CHUNK : (n - i);
        for (int k = 0; k < take; ++k) {
            const point_data_t *src = &pts[i + k];
            tmp[k].gx        = src->gx;
            tmp[k].gy        = src->gy;
            tmp[k].r         = src->r;
            tmp[k].g         = src->g;
            tmp[k].b         = src->b;
            tmp[k].intensity = src->intensity;
        }

        int chunk_dropped = 0;
        shutter_stream_push(tmp, take, &chunk_dropped);
        if (chunk_dropped) dropped_any = 1;

        i += take;
    }

    s->shutter_total_points += (uint64_t)n;
    if (dropped_any) s->shutter_full = 1;
}
