/*
 * Video Card 3D Pipeline
 * Fixed-point software rasterizer with:
 *   - Z-buffer depth testing
 *   - Flat and Gouraud shading
 *   - Backface culling (signed screen-space area)
 *   - Near-plane clipping
 *   - Affine texture mapping
 */

#include "video_card.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "VIDEO3D";

// ============================================================================
// 3D pipeline state
// ============================================================================

static struct {
    bool initialized;

    uint16_t *framebuffer;   // Pointer to video card's back buffer
    uint16_t  fb_width;
    uint16_t  fb_height;

    uint16_t *zbuffer;       // 16-bit depth buffer (SPIRAM)

    video_matrix_t mvp;      // Model-View-Projection matrix

    // Viewport
    int vp_x, vp_y, vp_w, vp_h;

    // Backface culling
    bool backface_cull;

    // Texture state
    uint32_t tex_addr;
    uint16_t tex_width;
    uint16_t tex_height;
    video_emu_read_fn tex_read;
} s_3d;

// ============================================================================
// Fixed-point trig lookup (256 entries = full circle)
// sin table, values in fixed-point 16.16
// ============================================================================

static const fixed16_t sin_table[256] = {
         0,   1608,   3216,   4821,   6424,   8022,   9616,  11204,
     12785,  14359,  15924,  17479,  19024,  20557,  22078,  23586,
     25080,  26558,  28020,  29466,  30893,  32303,  33692,  35062,
     36410,  37736,  39040,  40320,  41576,  42806,  44011,  45190,
     46341,  47464,  48559,  49624,  50660,  51665,  52639,  53581,
     54491,  55368,  56212,  57022,  57798,  58538,  59244,  59914,
     60547,  61145,  61705,  62228,  62714,  63162,  63572,  63944,
     64277,  64571,  64827,  65043,  65220,  65358,  65457,  65516,
     65536,  65516,  65457,  65358,  65220,  65043,  64827,  64571,
     64277,  63944,  63572,  63162,  62714,  62228,  61705,  61145,
     60547,  59914,  59244,  58538,  57798,  57022,  56212,  55368,
     54491,  53581,  52639,  51665,  50660,  49624,  48559,  47464,
     46341,  45190,  44011,  42806,  41576,  40320,  39040,  37736,
     36410,  35062,  33692,  32303,  30893,  29466,  28020,  26558,
     25080,  23586,  22078,  20557,  19024,  17479,  15924,  14359,
     12785,  11204,   9616,   8022,   6424,   4821,   3216,   1608,
         0,  -1608,  -3216,  -4821,  -6424,  -8022,  -9616, -11204,
    -12785, -14359, -15924, -17479, -19024, -20557, -22078, -23586,
    -25080, -26558, -28020, -29466, -30893, -32303, -33692, -35062,
    -36410, -37736, -39040, -40320, -41576, -42806, -44011, -45190,
    -46341, -47464, -48559, -49624, -50660, -51665, -52639, -53581,
    -54491, -55368, -56212, -57022, -57798, -58538, -59244, -59914,
    -60547, -61145, -61705, -62228, -62714, -63162, -63572, -63944,
    -64277, -64571, -64827, -65043, -65220, -65358, -65457, -65516,
    -65536, -65516, -65457, -65358, -65220, -65043, -64827, -64571,
    -64277, -63944, -63572, -63162, -62714, -62228, -61705, -61145,
    -60547, -59914, -59244, -58538, -57798, -57022, -56212, -55368,
    -54491, -53581, -52639, -51665, -50660, -49624, -48559, -47464,
    -46341, -45190, -44011, -42806, -41576, -40320, -39040, -37736,
    -36410, -35062, -33692, -32303, -30893, -29466, -28020, -26558,
    -25080, -23586, -22078, -20557, -19024, -17479, -15924, -14359,
    -12785, -11204,  -9616,  -8022,  -6424,  -4821,  -3216,  -1608,
};

static inline fixed16_t fp_sin(fixed16_t angle_deg) {
    int deg = FP_TO_INT(angle_deg) % 360;
    if (deg < 0) deg += 360;
    int idx = (deg * 256) / 360;
    return sin_table[idx & 0xFF];
}

static inline fixed16_t fp_cos(fixed16_t angle_deg) {
    int deg = (FP_TO_INT(angle_deg) + 90) % 360;
    if (deg < 0) deg += 360;
    int idx = (deg * 256) / 360;
    return sin_table[idx & 0xFF];
}

// ============================================================================
// Matrix operations
// ============================================================================

static void mat_identity(video_matrix_t *m) {
    memset(m, 0, sizeof(*m));
    m->m[0][0] = FP_ONE;
    m->m[1][1] = FP_ONE;
    m->m[2][2] = FP_ONE;
    m->m[3][3] = FP_ONE;
}

static void mat_multiply(video_matrix_t *result, const video_matrix_t *a, const video_matrix_t *b) {
    video_matrix_t tmp;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            int64_t sum = 0;
            for (int k = 0; k < 4; k++) {
                sum += (int64_t)a->m[i][k] * b->m[k][j];
            }
            tmp.m[i][j] = (fixed16_t)(sum >> FP_SHIFT);
        }
    }
    *result = tmp;
}

static void mat_transform_point(const video_matrix_t *m, fixed16_t x, fixed16_t y, fixed16_t z,
                                fixed16_t *ox, fixed16_t *oy, fixed16_t *oz, fixed16_t *ow) {
    *ox = FP_MUL(m->m[0][0], x) + FP_MUL(m->m[0][1], y) + FP_MUL(m->m[0][2], z) + m->m[0][3];
    *oy = FP_MUL(m->m[1][0], x) + FP_MUL(m->m[1][1], y) + FP_MUL(m->m[1][2], z) + m->m[1][3];
    *oz = FP_MUL(m->m[2][0], x) + FP_MUL(m->m[2][1], y) + FP_MUL(m->m[2][2], z) + m->m[2][3];
    *ow = FP_MUL(m->m[3][0], x) + FP_MUL(m->m[3][1], y) + FP_MUL(m->m[3][2], z) + m->m[3][3];
}

// ============================================================================
// RGB565 color helpers
// ============================================================================

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

static inline void rgb565_unpack(uint16_t c, uint8_t *r, uint8_t *g, uint8_t *b) {
    *r = ((c >> 11) & 0x1F) << 3;
    *g = ((c >> 5) & 0x3F) << 2;
    *b = (c & 0x1F) << 3;
}

static inline uint16_t color_lerp(uint16_t c1, uint16_t c2, fixed16_t t) {
    uint8_t r1, g1, b1, r2, g2, b2;
    rgb565_unpack(c1, &r1, &g1, &b1);
    rgb565_unpack(c2, &r2, &g2, &b2);
    uint8_t r = r1 + FP_TO_INT(FP_MUL(INT_TO_FP(r2 - r1), t));
    uint8_t g = g1 + FP_TO_INT(FP_MUL(INT_TO_FP(g2 - g1), t));
    uint8_t b = b1 + FP_TO_INT(FP_MUL(INT_TO_FP(b2 - b1), t));
    return rgb565(r, g, b);
}

static inline uint16_t color_scale(uint16_t c, fixed16_t intensity) {
    uint8_t r, g, b;
    rgb565_unpack(c, &r, &g, &b);
    r = (uint8_t)(FP_TO_INT(FP_MUL(INT_TO_FP(r), intensity)));
    g = (uint8_t)(FP_TO_INT(FP_MUL(INT_TO_FP(g), intensity)));
    b = (uint8_t)(FP_TO_INT(FP_MUL(INT_TO_FP(b), intensity)));
    return rgb565(r, g, b);
}

static inline uint16_t color_add(uint16_t c1, uint16_t c2) {
    uint8_t r1, g1, b1, r2, g2, b2;
    rgb565_unpack(c1, &r1, &g1, &b1);
    rgb565_unpack(c2, &r2, &g2, &b2);
    int r = r1 + r2; if (r > 255) r = 255;
    int g = g1 + g2; if (g > 255) g = 255;
    int b = b1 + b2; if (b > 255) b = 255;
    return rgb565(r, g, b);
}

// ============================================================================
// Texture sampling
// ============================================================================

static inline uint16_t sample_texture(fixed16_t u, fixed16_t v) {
    if (!s_3d.tex_read || s_3d.tex_width == 0 || s_3d.tex_height == 0) return 0xFFFF;

    // Clamp UV to [0, 1)
    int ui = FP_TO_INT(FP_MUL(u, INT_TO_FP(s_3d.tex_width))) % s_3d.tex_width;
    int vi = FP_TO_INT(FP_MUL(v, INT_TO_FP(s_3d.tex_height))) % s_3d.tex_height;
    if (ui < 0) ui += s_3d.tex_width;
    if (vi < 0) vi += s_3d.tex_height;

    // Read RGB565 texel from emulator RAM (big-endian)
    uint32_t offset = (vi * s_3d.tex_width + ui) * 2;
    uint32_t addr = s_3d.tex_addr + offset;
    uint8_t hi = s_3d.tex_read(addr);
    uint8_t lo = s_3d.tex_read(addr + 1);
    return (uint16_t)((hi << 8) | lo);
}

// ============================================================================
// Scanline triangle rasterizer
// ============================================================================

typedef struct {
    int x, y;           // Screen position
    fixed16_t z;        // Depth (for z-buffer)
    uint16_t color;     // Vertex color (for Gouraud)
    fixed16_t u, v;     // Texture coordinates
} screen_vertex_t;

static inline void put_pixel_3d(int x, int y, uint16_t z_val, uint16_t color, bool z_test) {
    if (x < s_3d.vp_x || x >= s_3d.vp_x + s_3d.vp_w) return;
    if (y < s_3d.vp_y || y >= s_3d.vp_y + s_3d.vp_h) return;

    int idx = y * s_3d.fb_width + x;

    if (z_test && s_3d.zbuffer) {
        if (z_val >= s_3d.zbuffer[idx]) return;
        s_3d.zbuffer[idx] = z_val;
    }

    s_3d.framebuffer[idx] = color;
}

// Rasterize a single triangle using scanline algorithm
static void rasterize_triangle(screen_vertex_t v0, screen_vertex_t v1, screen_vertex_t v2,
                               int shade_mode, bool z_test, bool textured) {
    // Sort vertices by Y coordinate (v0.y <= v1.y <= v2.y)
    if (v0.y > v1.y) { screen_vertex_t t = v0; v0 = v1; v1 = t; }
    if (v1.y > v2.y) { screen_vertex_t t = v1; v1 = v2; v2 = t; }
    if (v0.y > v1.y) { screen_vertex_t t = v0; v0 = v1; v1 = t; }

    int total_height = v2.y - v0.y;
    if (total_height == 0) return;

    uint16_t flat_color = v0.color;

    for (int y = v0.y; y <= v2.y; y++) {
        bool second_half = (y > v1.y) || (v1.y == v0.y);
        int segment_height = second_half ? (v2.y - v1.y) : (v1.y - v0.y);
        if (segment_height == 0) continue;

        fixed16_t alpha = FP_DIV(INT_TO_FP(y - v0.y), INT_TO_FP(total_height));
        fixed16_t beta;
        if (second_half) {
            beta = FP_DIV(INT_TO_FP(y - v1.y), INT_TO_FP(segment_height));
        } else {
            beta = FP_DIV(INT_TO_FP(y - v0.y), INT_TO_FP(segment_height));
        }

        // Interpolate X, Z, color, UV along edges
        int ax = v0.x + FP_TO_INT(FP_MUL(INT_TO_FP(v2.x - v0.x), alpha));
        int bx;
        fixed16_t az = v0.z + FP_MUL(v2.z - v0.z, alpha);
        fixed16_t bz;
        uint16_t ac, bc;
        fixed16_t au = v0.u + FP_MUL(v2.u - v0.u, alpha);
        fixed16_t av_coord = v0.v + FP_MUL(v2.v - v0.v, alpha);
        fixed16_t bu, bv_coord;

        if (second_half) {
            bx = v1.x + FP_TO_INT(FP_MUL(INT_TO_FP(v2.x - v1.x), beta));
            bz = v1.z + FP_MUL(v2.z - v1.z, beta);
            ac = (shade_mode == 1) ? color_lerp(v0.color, v2.color, alpha) : 0;
            bc = (shade_mode == 1) ? color_lerp(v1.color, v2.color, beta) : 0;
            bu = v1.u + FP_MUL(v2.u - v1.u, beta);
            bv_coord = v1.v + FP_MUL(v2.v - v1.v, beta);
        } else {
            bx = v0.x + FP_TO_INT(FP_MUL(INT_TO_FP(v1.x - v0.x), beta));
            bz = v0.z + FP_MUL(v1.z - v0.z, beta);
            ac = (shade_mode == 1) ? color_lerp(v0.color, v2.color, alpha) : 0;
            bc = (shade_mode == 1) ? color_lerp(v0.color, v1.color, beta) : 0;
            bu = v0.u + FP_MUL(v1.u - v0.u, beta);
            bv_coord = v0.v + FP_MUL(v1.v - v0.v, beta);
        }

        if (ax > bx) {
            int ti = ax; ax = bx; bx = ti;
            fixed16_t tf = az; az = bz; bz = tf;
            if (shade_mode == 1) { uint16_t tc = ac; ac = bc; bc = tc; }
            tf = au; au = bu; bu = tf;
            tf = av_coord; av_coord = bv_coord; bv_coord = tf;
        }

        int span = bx - ax;
        for (int x = ax; x <= bx; x++) {
            fixed16_t t = (span > 0) ? FP_DIV(INT_TO_FP(x - ax), INT_TO_FP(span)) : 0;
            fixed16_t z = az + FP_MUL(bz - az, t);
            uint16_t z16 = (uint16_t)(FP_TO_INT(z) & 0xFFFF);

            uint16_t color;
            if (textured) {
                fixed16_t su = au + FP_MUL(bu - au, t);
                fixed16_t sv = av_coord + FP_MUL(bv_coord - av_coord, t);
                color = sample_texture(su, sv);
                // Modulate texture with vertex color for lighting
                if (shade_mode == 1) {
                    uint16_t vc = color_lerp(ac, bc, t);
                    // Simple multiply modulation
                    uint8_t tr, tg, tb, vr, vg, vb;
                    rgb565_unpack(color, &tr, &tg, &tb);
                    rgb565_unpack(vc, &vr, &vg, &vb);
                    tr = (tr * vr) >> 8;
                    tg = (tg * vg) >> 8;
                    tb = (tb * vb) >> 8;
                    color = rgb565(tr, tg, tb);
                }
            } else if (shade_mode == 1) {
                color = color_lerp(ac, bc, t);
            } else {
                color = flat_color;
            }

            put_pixel_3d(x, y, z16, color, z_test);
        }
    }
}

// ============================================================================
// Near-plane clipping (clip triangle against w=near plane)
// Returns number of output vertices (0, 3, or 4 for a quad)
// ============================================================================

typedef struct {
    fixed16_t x, y, z, w;
    uint16_t color;
    fixed16_t u, v;
    fixed16_t nx, ny, nz;
} clip_vertex_t;

static int clip_near_plane(clip_vertex_t in[3], clip_vertex_t out[4]) {
    // Clip against w > 0 (near plane in clip space)
    // We use a small positive threshold to avoid divide-by-zero
    const fixed16_t NEAR_W = FP_ONE / 64; // Small positive w threshold

    int inside[3];
    int num_inside = 0;
    for (int i = 0; i < 3; i++) {
        inside[i] = (in[i].w > NEAR_W) ? 1 : 0;
        num_inside += inside[i];
    }

    if (num_inside == 3) {
        // All inside
        out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
        return 3;
    }
    if (num_inside == 0) {
        return 0; // All behind near plane
    }

    // Clip: interpolate between inside and outside vertices
    int count = 0;
    for (int i = 0; i < 3; i++) {
        int j = (i + 1) % 3;
        if (inside[i]) {
            out[count++] = in[i];
        }
        if (inside[i] != inside[j]) {
            // Edge crosses near plane - compute intersection
            fixed16_t dw_i = in[i].w - NEAR_W;
            fixed16_t dw_j = in[j].w - NEAR_W;
            fixed16_t denom = dw_i - dw_j;
            if (denom == 0) denom = 1;
            fixed16_t t = FP_DIV(dw_i, denom);

            clip_vertex_t *c = &out[count++];
            c->x = in[i].x + FP_MUL(in[j].x - in[i].x, t);
            c->y = in[i].y + FP_MUL(in[j].y - in[i].y, t);
            c->z = in[i].z + FP_MUL(in[j].z - in[i].z, t);
            c->w = NEAR_W;
            c->u = in[i].u + FP_MUL(in[j].u - in[i].u, t);
            c->v = in[i].v + FP_MUL(in[j].v - in[i].v, t);
            c->nx = in[i].nx + FP_MUL(in[j].nx - in[i].nx, t);
            c->ny = in[i].ny + FP_MUL(in[j].ny - in[i].ny, t);
            c->nz = in[i].nz + FP_MUL(in[j].nz - in[i].nz, t);
            c->color = color_lerp(in[i].color, in[j].color, t);
        }
    }
    return count; // 3 or 4
}

// ============================================================================
// Public 3D API
// ============================================================================

esp_err_t video_3d_init(uint16_t *framebuffer, uint16_t fb_width, uint16_t fb_height) {
    s_3d.framebuffer = framebuffer;
    s_3d.fb_width = fb_width;
    s_3d.fb_height = fb_height;

    size_t zb_size = fb_width * fb_height * sizeof(uint16_t);
    s_3d.zbuffer = heap_caps_calloc(1, zb_size, MALLOC_CAP_SPIRAM);
    if (!s_3d.zbuffer) {
        ESP_LOGW(TAG, "Z-buffer alloc failed (%u bytes), 3D depth test disabled", (unsigned)zb_size);
    }

    mat_identity(&s_3d.mvp);

    s_3d.vp_x = 0;
    s_3d.vp_y = 0;
    s_3d.vp_w = fb_width;
    s_3d.vp_h = fb_height;
    s_3d.backface_cull = false;

    s_3d.initialized = true;
    ESP_LOGI(TAG, "3D pipeline initialized (%dx%d, zbuffer=%s)",
             fb_width, fb_height, s_3d.zbuffer ? "yes" : "no");
    return ESP_OK;
}

void video_3d_deinit(void) {
    if (s_3d.zbuffer) {
        heap_caps_free(s_3d.zbuffer);
        s_3d.zbuffer = NULL;
    }
    s_3d.initialized = false;
}

void video_3d_set_framebuffer(uint16_t *fb) {
    s_3d.framebuffer = fb;
}

void video_3d_clear_zbuffer(void) {
    if (s_3d.zbuffer) {
        memset(s_3d.zbuffer, 0xFF, s_3d.fb_width * s_3d.fb_height * sizeof(uint16_t));
    }
}

void video_3d_set_matrix(const video_matrix_t *mat) {
    s_3d.mvp = *mat;
}

video_matrix_t *video_3d_get_matrix(void) {
    return &s_3d.mvp;
}

void video_3d_identity(void) {
    mat_identity(&s_3d.mvp);
}

void video_3d_translate(fixed16_t tx, fixed16_t ty, fixed16_t tz) {
    video_matrix_t t;
    mat_identity(&t);
    t.m[0][3] = tx;
    t.m[1][3] = ty;
    t.m[2][3] = tz;
    video_matrix_t result;
    mat_multiply(&result, &s_3d.mvp, &t);
    s_3d.mvp = result;
}

void video_3d_rotate_x(fixed16_t angle) {
    fixed16_t c = fp_cos(angle);
    fixed16_t s = fp_sin(angle);
    video_matrix_t r;
    mat_identity(&r);
    r.m[1][1] = c;  r.m[1][2] = -s;
    r.m[2][1] = s;  r.m[2][2] = c;
    video_matrix_t result;
    mat_multiply(&result, &s_3d.mvp, &r);
    s_3d.mvp = result;
}

void video_3d_rotate_y(fixed16_t angle) {
    fixed16_t c = fp_cos(angle);
    fixed16_t s = fp_sin(angle);
    video_matrix_t r;
    mat_identity(&r);
    r.m[0][0] = c;  r.m[0][2] = s;
    r.m[2][0] = -s; r.m[2][2] = c;
    video_matrix_t result;
    mat_multiply(&result, &s_3d.mvp, &r);
    s_3d.mvp = result;
}

void video_3d_rotate_z(fixed16_t angle) {
    fixed16_t c = fp_cos(angle);
    fixed16_t s = fp_sin(angle);
    video_matrix_t r;
    mat_identity(&r);
    r.m[0][0] = c;  r.m[0][1] = -s;
    r.m[1][0] = s;  r.m[1][1] = c;
    video_matrix_t result;
    mat_multiply(&result, &s_3d.mvp, &r);
    s_3d.mvp = result;
}

void video_3d_scale(fixed16_t sx, fixed16_t sy, fixed16_t sz) {
    video_matrix_t sc;
    mat_identity(&sc);
    sc.m[0][0] = sx;
    sc.m[1][1] = sy;
    sc.m[2][2] = sz;
    video_matrix_t result;
    mat_multiply(&result, &s_3d.mvp, &sc);
    s_3d.mvp = result;
}

void video_3d_perspective(fixed16_t fov, fixed16_t aspect, fixed16_t near_val, fixed16_t far_val) {
    fixed16_t half_fov = fov / 2;
    fixed16_t s_val = fp_sin(half_fov);
    fixed16_t c_val = fp_cos(half_fov);
    if (s_val == 0) s_val = 1;
    fixed16_t f = FP_DIV(c_val, s_val);

    fixed16_t range = near_val - far_val;
    if (range == 0) range = 1;

    video_matrix_t p;
    memset(&p, 0, sizeof(p));
    p.m[0][0] = (aspect != 0) ? FP_DIV(f, aspect) : f;
    p.m[1][1] = f;
    p.m[2][2] = FP_DIV(far_val + near_val, range);
    p.m[2][3] = FP_DIV(FP_MUL(INT_TO_FP(2), FP_MUL(far_val, near_val)), range);
    p.m[3][2] = -FP_ONE;
    p.m[3][3] = 0;

    video_matrix_t result;
    mat_multiply(&result, &p, &s_3d.mvp);
    s_3d.mvp = result;
}

void video_3d_set_viewport(int x, int y, int w, int h) {
    s_3d.vp_x = x;
    s_3d.vp_y = y;
    s_3d.vp_w = w;
    s_3d.vp_h = h;
}

void video_3d_set_backface_cull(bool enabled) {
    s_3d.backface_cull = enabled;
}

void video_3d_set_texture(uint32_t addr, uint16_t width, uint16_t height,
                          video_emu_read_fn read_fn) {
    s_3d.tex_addr = addr;
    s_3d.tex_width = width;
    s_3d.tex_height = height;
    s_3d.tex_read = read_fn;
}

// Helper: convert clip-space vertex to screen vertex after perspective divide
static screen_vertex_t clip_to_screen(const clip_vertex_t *cv) {
    screen_vertex_t sv;
    fixed16_t w = cv->w;
    if (w == 0) w = 1;

    fixed16_t ndcx = FP_DIV(cv->x, w);
    fixed16_t ndcy = FP_DIV(cv->y, w);
    fixed16_t ndcz = FP_DIV(cv->z, w);

    sv.x = s_3d.vp_x + FP_TO_INT(FP_MUL(ndcx + FP_ONE, INT_TO_FP(s_3d.vp_w)) / 2);
    sv.y = s_3d.vp_y + FP_TO_INT(FP_MUL(FP_ONE - ndcy, INT_TO_FP(s_3d.vp_h)) / 2);
    sv.z = (ndcz + FP_ONE) / 2;
    sv.color = cv->color;
    sv.u = cv->u;
    sv.v = cv->v;
    return sv;
}

void video_3d_draw_triangles(
    const video_vertex_t *vertices, uint32_t num_verts,
    const uint16_t *indices, uint32_t num_indices,
    int shade_mode, bool zbuffer_enable,
    fixed16_t light_x, fixed16_t light_y, fixed16_t light_z,
    uint16_t light_color, uint16_t ambient) {

    if (!s_3d.initialized || !s_3d.framebuffer) return;

    bool textured = (s_3d.tex_read != NULL && s_3d.tex_width > 0 && s_3d.tex_height > 0);

    for (uint32_t i = 0; i + 2 < num_indices; i += 3) {
        uint16_t i0 = indices[i];
        uint16_t i1 = indices[i + 1];
        uint16_t i2 = indices[i + 2];

        if (i0 >= num_verts || i1 >= num_verts || i2 >= num_verts) continue;

        const video_vertex_t *v0 = &vertices[i0];
        const video_vertex_t *v1 = &vertices[i1];
        const video_vertex_t *v2 = &vertices[i2];

        // Transform vertices through MVP matrix
        clip_vertex_t cv[3];
        const video_vertex_t *verts[3] = {v0, v1, v2};
        for (int vi = 0; vi < 3; vi++) {
            mat_transform_point(&s_3d.mvp, verts[vi]->x, verts[vi]->y, verts[vi]->z,
                                &cv[vi].x, &cv[vi].y, &cv[vi].z, &cv[vi].w);
            cv[vi].color = verts[vi]->color;
            cv[vi].u = verts[vi]->u;
            cv[vi].v = verts[vi]->v;
            cv[vi].nx = verts[vi]->nx;
            cv[vi].ny = verts[vi]->ny;
            cv[vi].nz = verts[vi]->nz;
        }

        // Near-plane clipping
        clip_vertex_t clipped[4];
        int clip_count = clip_near_plane(cv, clipped);
        if (clip_count < 3) continue;

        // Process clipped triangles (3 or 4 vertices = 1 or 2 triangles)
        for (int tri = 0; tri < clip_count - 2; tri++) {
            clip_vertex_t *c0 = &clipped[0];
            clip_vertex_t *c1 = &clipped[tri + 1];
            clip_vertex_t *c2 = &clipped[tri + 2];

            // Perspective divide and viewport transform
            screen_vertex_t sv0 = clip_to_screen(c0);
            screen_vertex_t sv1 = clip_to_screen(c1);
            screen_vertex_t sv2 = clip_to_screen(c2);

            // Backface culling: compute signed screen-space area (2x cross product)
            if (s_3d.backface_cull) {
                int area2x = (sv1.x - sv0.x) * (sv2.y - sv0.y) -
                             (sv2.x - sv0.x) * (sv1.y - sv0.y);
                if (area2x >= 0) continue; // Clockwise = back-facing, skip
            }

            // Trivial screen reject
            int min_x = sv0.x < sv1.x ? (sv0.x < sv2.x ? sv0.x : sv2.x) : (sv1.x < sv2.x ? sv1.x : sv2.x);
            int max_x = sv0.x > sv1.x ? (sv0.x > sv2.x ? sv0.x : sv2.x) : (sv1.x > sv2.x ? sv1.x : sv2.x);
            int min_y = sv0.y < sv1.y ? (sv0.y < sv2.y ? sv0.y : sv2.y) : (sv1.y < sv2.y ? sv1.y : sv2.y);
            int max_y = sv0.y > sv1.y ? (sv0.y > sv2.y ? sv0.y : sv2.y) : (sv1.y > sv2.y ? sv1.y : sv2.y);
            if (max_x < s_3d.vp_x || min_x >= s_3d.vp_x + s_3d.vp_w) continue;
            if (max_y < s_3d.vp_y || min_y >= s_3d.vp_y + s_3d.vp_h) continue;

            // Compute face color / per-vertex lighting
            if (shade_mode == 0) {
                // Flat shading: face normal from world-space edges
                fixed16_t ex1 = v1->x - v0->x, ey1 = v1->y - v0->y, ez1 = v1->z - v0->z;
                fixed16_t ex2 = v2->x - v0->x, ey2 = v2->y - v0->y, ez2 = v2->z - v0->z;

                fixed16_t nx = FP_MUL(ey1, ez2) - FP_MUL(ez1, ey2);
                fixed16_t ny = FP_MUL(ez1, ex2) - FP_MUL(ex1, ez2);
                fixed16_t nz = FP_MUL(ex1, ey2) - FP_MUL(ey1, ex2);

                fixed16_t dot = FP_MUL(nx, light_x) + FP_MUL(ny, light_y) + FP_MUL(nz, light_z);
                if (dot < 0) dot = 0;
                if (dot > FP_ONE) dot = FP_ONE;

                uint16_t diffuse = color_scale(light_color, dot);
                uint16_t face_color = color_add(ambient, diffuse);
                sv0.color = sv1.color = sv2.color = face_color;
            } else {
                // Gouraud: per-vertex lighting
                screen_vertex_t *svs[3] = {&sv0, &sv1, &sv2};
                for (int vi = 0; vi < 3; vi++) {
                    const video_vertex_t *vert = verts[vi];
                    fixed16_t dot = FP_MUL(vert->nx, light_x) +
                                    FP_MUL(vert->ny, light_y) +
                                    FP_MUL(vert->nz, light_z);
                    if (dot < 0) dot = 0;
                    if (dot > FP_ONE) dot = FP_ONE;

                    uint16_t lit = color_scale(vert->color, dot);
                    svs[vi]->color = color_add(ambient, lit);
                }
            }

            rasterize_triangle(sv0, sv1, sv2, shade_mode,
                               zbuffer_enable && s_3d.zbuffer != NULL, textured);
        }
    }
}
