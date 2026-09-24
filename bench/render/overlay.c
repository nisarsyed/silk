#include "overlay.h"
#include <float.h>
#include <math.h>
#include <silk/assert.h>

static bool line(sl_render_overlay *out, double ax, double ay, double bx,
                 double by, float color)
{
    if (out->line_count >= out->line_capacity) {
        return false;
    }
    const double pixels[] = { out->offset_x + ax * out->scale,
                              out->offset_y - ay * out->scale,
                              out->offset_x + bx * out->scale,
                              out->offset_y - by * out->scale };
    for (uint32_t i = 0u; i < 4u; ++i) {
        if (!isfinite(pixels[i]) || fabs(pixels[i]) > (double)FLT_MAX) {
            return false;
        }
    }
    float *p = out->lines + 5u * out->line_count;
    for (uint32_t i = 0u; i < 4u; ++i) {
        p[i] = (float)pixels[i];
    }
    p[4] = color;
    const float dx = p[2] - p[0], dy = p[3] - p[1];
    if (!isfinite(p[0]) || !isfinite(p[1]) || !isfinite(p[2]) ||
        !isfinite(p[3]) || !isfinite(dx * dx + dy * dy)) {
        return false;
    }
    ++out->line_count;
    return true;
}
static bool marker(sl_render_overlay *out, double x, double y, float color)
{
    if (out->marker_count >= out->marker_capacity) {
        return false;
    }
    const double px = out->offset_x + x * out->scale;
    const double py = out->offset_y - y * out->scale;
    if (!isfinite(px) || !isfinite(py) || fabs(px) > (double)FLT_MAX ||
        fabs(py) > (double)FLT_MAX) {
        return false;
    }
    float *p = out->markers + 3u * out->marker_count;
    p[0] = (float)px;
    p[1] = (float)py;
    p[2] = color;
    if (!isfinite(p[0]) || !isfinite(p[1])) {
        return false;
    }
    ++out->marker_count;
    return true;
}
static bool normal(sl_render_overlay *out, double x, double y, double nx,
                   double ny, float color)
{
    // Only the display direction is normalized. Raw physics normals remain
    // unchanged; the shared glyph length is twelve drawing-buffer pixels.
    const double length = hypot(nx, ny);
    if (!isfinite(length) || length == 0.0) {
        return false;
    }
    const double scale = 12.0 / (length * out->scale);
    return line(out, x, y, x + nx * scale, y + ny * scale, color);
}
static bool box(sl_render_overlay *out, double lx, double ly, double ux,
                double uy, float color)
{
    return line(out, lx, ly, ux, ly, color) &&
           line(out, ux, ly, ux, uy, color) &&
           line(out, ux, uy, lx, uy, color) && line(out, lx, uy, lx, ly, color);
}
static float body_color(const sl_world *world, sl_body_handle body)
{
    if (sl_world_body_get_type(world, body) == SL_BODY_STATIC) {
        return 0.0f;
    }
    if (!sl_world_body_is_awake(world, body)) {
        return 2.0f;
    }
    sl_island_stats island;
    return sl_world_body_get_island_stats(world, body, &island) &&
                   island.id != UINT32_MAX
               ? (float)(3u + island.id % 8u)
               : 1.0f;
}
bool sl_render_overlay_prepare(sl_render_study *study, sl_render_overlay *out)
{
    if (study == NULL || out == NULL || out->lines == NULL ||
        out->markers == NULL || out->colors == NULL || !isfinite(out->scale) ||
        out->scale <= 0.0 || !isfinite(out->offset_x) ||
        !isfinite(out->offset_y)) {
        return false;
    }
    const uint32_t *status = sl_render_study_status(study);
    const uint32_t b = status[5], c = status[6], j = status[7];
    const sl_world *world = sl_render_study_world(study);
    const uint32_t count = sl_world_body_count(world);
    if (out->color_count != count ||
        out->line_capacity < 4u * b + 2u * c + j + 6u ||
        out->marker_capacity < 2u * c + 2u * j + 2u ||
        !sl_render_study_diagnostics(study)) {
        return false;
    }
    out->line_count = 0u;
    out->marker_count = 0u;
    const float *d = sl_render_study_diagnostic_f32(study);
    const uint32_t *w = sl_render_study_diagnostic_u32(study);
    for (uint32_t row = 0u; row < count; ++row) {
        const sl_body_handle body = sl_world_body_at(world, row);
        out->colors[row] = body_color(world, body);
        if ((w[body.index] & 8u) != 0u &&
            !box(out, (double)d[body.index], (double)d[b + body.index],
                 (double)d[2u * b + body.index], (double)d[3u * b + body.index],
                 (w[body.index] & 7u) != 0u ? 15.0f : 13.0f)) {
            return false;
        }
    }
    const uint32_t contact_count = sl_world_contact_count(world);
    for (uint32_t row = 0u; row < contact_count; ++row) {
        const sl_manifold *m = &sl_world_contact_at(world, row)->manifold;
        SL_ASSERT(m->point_count <= SL_MANIFOLD_POINT_COUNT_MAX);
        if (m->point_count > SL_MANIFOLD_POINT_COUNT_MAX) {
            return false;
        }
        for (uint32_t p = 0u; p < m->point_count; ++p) {
            const sl_vec2 point = m->points[p].point;
            if (!normal(out, (double)point.x, (double)point.y,
                        (double)m->normal.x, (double)m->normal.y, 12.0f) ||
                !marker(out, (double)point.x, (double)point.y, 11.0f)) {
                return false;
            }
        }
    }
    const uint32_t joint_count = sl_world_joint_count(world);
    for (uint32_t row = 0u; row < joint_count; ++row) {
        const sl_joint_desc joint =
            sl_world_joint_get_desc(world, sl_world_joint_at(world, row));
        const sl_transform a = sl_world_body_get_transform(world, joint.body_a);
        const sl_transform z = sl_world_body_get_transform(world, joint.body_b);
        const double ax =
            (double)a.position.x +
            (double)a.rotation.c * (double)joint.local_anchor_a.x -
            (double)a.rotation.s * (double)joint.local_anchor_a.y;
        const double ay =
            (double)a.position.y +
            (double)a.rotation.s * (double)joint.local_anchor_a.x +
            (double)a.rotation.c * (double)joint.local_anchor_a.y;
        const double bx =
            (double)z.position.x +
            (double)z.rotation.c * (double)joint.local_anchor_b.x -
            (double)z.rotation.s * (double)joint.local_anchor_b.y;
        const double by =
            (double)z.position.y +
            (double)z.rotation.s * (double)joint.local_anchor_b.x +
            (double)z.rotation.c * (double)joint.local_anchor_b.y;
        if (!line(out, ax, ay, bx, by, 14.0f) || !marker(out, ax, ay, 14.0f) ||
            !marker(out, bx, by, 14.0f)) {
            return false;
        }
    }
    const float *rect = sl_render_study_settings(study) + 17;
    const double x = ((double)rect[0] + (double)rect[2]) * 0.5;
    const double y = ((double)rect[1] + (double)rect[3]) * 0.5;
    if (!box(out, x - 1.0, y - 1.0, x + 1.0, y + 1.0, 15.0f) ||
        !line(out, (double)rect[0], y, (double)rect[2], y, 15.0f) ||
        !marker(out, x, y, 15.0f)) {
        return false;
    }
    if (w[b + 2u] != 0u) {
        const float *ray = d + 4u * b;
        if (!marker(out, (double)ray[1], (double)ray[2], 15.0f) ||
            !normal(out, (double)ray[1], (double)ray[2], (double)ray[3],
                    (double)ray[4], 15.0f)) {
            return false;
        }
    }
    return true;
}
