#include <string.h>

#include "energy.h"
#include "world_internal.h"

/* sl_world_cloth_add builds every derived array directly into the unused
 * tail of the shared pools. Nothing observable changes until the final
 * commit bumps the counts, so a rejected mesh leaves the world as it was. */

#define SL_COLOR_NONE 0xFFu

static bool sl_material_valid(const sl_cloth_desc *d)
{
    const float values[] = { d->density,         d->membrane_mu,
                             d->membrane_lambda, d->bending_stiffness,
                             d->damping,         d->thickness,
                             d->friction,        d->drag };
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        if (!sl_is_finite(values[i]) || !(values[i] >= 0.0f)) {
            return false;
        }
    }
    return d->density > 0.0f && d->thickness > 0.0f &&
           d->thickness <= SL_POSITION_ABS_MAX;
}

static bool sl_position_valid(sl_vec3 p)
{
    return sl_vec3_is_finite(p) && sl_abs(p.x) <= SL_POSITION_ABS_MAX &&
           sl_abs(p.y) <= SL_POSITION_ABS_MAX &&
           sl_abs(p.z) <= SL_POSITION_ABS_MAX;
}

/* Local corner of vertex v in local triangle t of desc, or 3 if absent. */
static uint32_t sl_corner_of(const uint32_t *indices, uint32_t t, uint32_t v)
{
    for (uint32_t k = 0; k < 3u; k++) {
        if (indices[3u * t + k] == v) {
            return k;
        }
    }
    return 3u;
}

/* Greedy coloring in ascending vertex order: a vertex takes the lowest
 * color unused by every vertex sharing a triangle or bending stencil with
 * it. Returns the color count, or 0 when SL_CLOTH_COLOR_COUNT_MAX is
 * exceeded. Bound: one pass over each vertex's adjacency. */
static uint32_t sl_mesh_color(sl_world_state *s, uint32_t v0, uint32_t n)
{
    uint8_t *color = s->color_scratch + v0;
    memset(color, SL_COLOR_NONE, n);
    uint32_t color_count = 0;
    for (uint32_t local = 0; local < n; local++) {
        const uint32_t v = v0 + local;
        uint64_t used = 0;
        for (uint32_t r = s->vertex_triangle_offset[v];
             r < s->vertex_triangle_offset[v + 1u]; r++) {
            const uint32_t t = s->vertex_triangle[r] / 3u;
            for (uint32_t k = 0; k < 3u; k++) {
                const uint8_t c = color[s->triangle_index[3u * t + k] - v0];
                if (c != SL_COLOR_NONE) {
                    used |= (uint64_t)1u << c;
                }
            }
        }
        for (uint32_t r = s->vertex_bend_offset[v];
             r < s->vertex_bend_offset[v + 1u]; r++) {
            const uint32_t b = s->vertex_bend[r] / 4u;
            for (uint32_t k = 0; k < 4u; k++) {
                const uint8_t c = color[s->bend_index[4u * b + k] - v0];
                if (c != SL_COLOR_NONE) {
                    used |= (uint64_t)1u << c;
                }
            }
        }
        uint32_t pick = 0;
        while (pick < SL_CLOTH_COLOR_COUNT_MAX &&
               (used & ((uint64_t)1u << pick)) != 0u) {
            pick++;
        }
        if (pick == SL_CLOTH_COLOR_COUNT_MAX) {
            return 0u;
        }
        color[local] = (uint8_t)pick;
        if (pick + 1u > color_count) {
            color_count = pick + 1u;
        }
    }
    return color_count;
}

/* sigma_max of a committed cloth: per vertex, the trace of its rest
 * elastic Hessian block over 3 m. At rest F is an isometry and G = 0, so a
 * membrane corner contributes A |w|^2 (3 mu + lambda) and a bending slot
 * 3 k scale weight^2. */
static float sl_mesh_stiffness_rate_max(const sl_world_state *s,
                                        const sl_cloth_record *rec)
{
    float rate_max = 0.0f;
    for (uint32_t i = rec->vertex_offset;
         i < rec->vertex_offset + rec->vertex_count; i++) {
        double trace = 0.0;
        for (uint32_t r = s->vertex_triangle_offset[i];
             r < s->vertex_triangle_offset[i + 1u]; r++) {
            const uint32_t t = s->vertex_triangle[r] / 3u;
            const uint32_t corner = s->vertex_triangle[r] % 3u;
            const float *rest = s->triangle_rest + 4u * t;
            float wx = -(rest[0] + rest[2]);
            float wy = -(rest[1] + rest[3]);
            if (corner == 1u) {
                wx = rest[0];
                wy = rest[1];
            } else if (corner == 2u) {
                wx = rest[2];
                wy = rest[3];
            }
            trace += (double)s->triangle_area[t] * (double)(wx * wx + wy * wy) *
                     (double)(3.0f * rec->membrane_mu + rec->membrane_lambda);
        }
        for (uint32_t r = s->vertex_bend_offset[i];
             r < s->vertex_bend_offset[i + 1u]; r++) {
            const uint32_t b = s->vertex_bend[r] / 4u;
            const float w = s->bend_weight[s->vertex_bend[r]];
            trace += 3.0 * (double)rec->bending_stiffness *
                     (double)s->bend_scale[b] * (double)(w * w);
        }
        const float rate = (float)(trace / (3.0 * (double)s->mass[i]));
        rate_max = sl_max(rate_max, rate);
    }
    return rate_max;
}

bool sl_world_cloth_add(sl_world *world, const sl_cloth_desc *desc,
                        uint32_t *cloth)
{
    SL_ASSERT(world != NULL && world->state != NULL);
    SL_ASSERT(desc != NULL);
    SL_ASSERT(cloth != NULL);
    sl_world_state *s = world->state;
    if (desc == NULL || desc->positions == NULL || desc->indices == NULL ||
        desc->vertex_count == 0u || desc->triangle_count == 0u ||
        s->cloth_count >= s->cloth_capacity ||
        desc->vertex_count > s->vertex_capacity - s->vertex_count ||
        desc->triangle_count > s->triangle_capacity - s->triangle_count ||
        !sl_material_valid(desc)) {
        return false;
    }
    const uint32_t n = desc->vertex_count;
    const uint32_t tn = desc->triangle_count;
    const uint32_t v0 = s->vertex_count;
    const uint32_t t0 = s->triangle_count;
    const uint32_t b0 = s->bend_count;
    const uint32_t e0 = s->edge_count;
    for (uint32_t i = 0; i < n; i++) {
        if (!sl_position_valid(desc->positions[i])) {
            return false;
        }
    }

    /* Triangles: indices, rest metric, per-vertex incidence counts. */
    /* Entry v0 is the previous cloth's end (3 * t0) and is never cleared,
     * so a rejected add cannot corrupt it. */
    uint32_t *tri_offset = s->vertex_triangle_offset;
    SL_ASSERT(tri_offset[v0] == 3u * t0);
    for (uint32_t i = 1; i <= n; i++) {
        tri_offset[v0 + i] = 0u;
    }
    for (uint32_t t = 0; t < tn; t++) {
        const uint32_t *idx = desc->indices + 3u * t;
        if (idx[0] >= n || idx[1] >= n || idx[2] >= n || idx[0] == idx[1] ||
            idx[1] == idx[2] || idx[0] == idx[2]) {
            return false;
        }
        float *rest = s->triangle_rest + 4u * (t0 + t);
        if (!sl_membrane_rest(desc->positions[idx[0]], desc->positions[idx[1]],
                              desc->positions[idx[2]], rest,
                              &s->triangle_area[t0 + t])) {
            return false;
        }
        for (uint32_t k = 0; k < 3u; k++) {
            s->triangle_index[3u * (t0 + t) + k] = v0 + idx[k];
            tri_offset[v0 + idx[k] + 1u]++;
        }
    }
    for (uint32_t i = 0; i < n; i++) {
        const uint32_t count = tri_offset[v0 + i + 1u];
        if (count == 0u || count > SL_VERTEX_TRIANGLE_COUNT_MAX) {
            return false;
        }
        tri_offset[v0 + i + 1u] = tri_offset[v0 + i] + count;
    }
    /* Fill using color_order as a per-vertex cursor; it is rebuilt below. */
    uint32_t *cursor = s->color_order + v0;
    for (uint32_t i = 0; i < n; i++) {
        cursor[i] = tri_offset[v0 + i];
    }
    for (uint32_t t = 0; t < tn; t++) {
        for (uint32_t k = 0; k < 3u; k++) {
            const uint32_t v = desc->indices[3u * t + k];
            s->vertex_triangle[cursor[v]++] = 3u * (t0 + t) + k;
        }
    }

    /* Interior edges: each is found from its lower-numbered triangle by
     * scanning the incident triangles of one endpoint (bounded by
     * SL_VERTEX_TRIANGLE_COUNT_MAX). */
    uint32_t bn = 0;
    uint32_t en = 0;
    for (uint32_t t = 0; t < tn; t++) {
        const uint32_t *idx = desc->indices + 3u * t;
        for (uint32_t e = 0; e < 3u; e++) {
            const uint32_t a = idx[e];
            const uint32_t b = idx[(e + 1u) % 3u];
            const uint32_t wing = idx[(e + 2u) % 3u];
            uint32_t other = UINT32_MAX;
            uint32_t share = 0;
            for (uint32_t r = tri_offset[v0 + a]; r < tri_offset[v0 + a + 1u];
                 r++) {
                const uint32_t u = s->vertex_triangle[r] / 3u - t0;
                if (u != t && sl_corner_of(desc->indices, u, b) < 3u) {
                    other = u;
                    share++;
                }
            }
            if (share > 1u) {
                return false; /* non-manifold edge */
            }
            if (other < t && share == 1u) {
                continue; /* edge already listed by the other triangle */
            }
            s->edge_index[2u * (e0 + en)] = v0 + a;
            s->edge_index[2u * (e0 + en) + 1u] = v0 + b;
            en++;
            if (share == 0u) {
                continue; /* boundary edge: no bending stencil */
            }
            uint32_t opposite = UINT32_MAX;
            for (uint32_t k = 0; k < 3u; k++) {
                const uint32_t c = desc->indices[3u * other + k];
                if (c != a && c != b) {
                    opposite = c;
                }
            }
            if (opposite == UINT32_MAX || opposite == wing) {
                return false; /* duplicate triangle */
            }
            const uint32_t stencil = b0 + bn;
            const uint32_t local[4] = { a, b, wing, opposite };
            sl_vec3 x[4];
            for (uint32_t k = 0; k < 4u; k++) {
                s->bend_index[4u * stencil + k] = v0 + local[k];
                x[k] = desc->positions[local[k]];
            }
            if (!sl_bend_rest(x, s->bend_weight + 4u * stencil,
                              &s->bend_scale[stencil])) {
                return false;
            }
            bn++;
        }
    }
    SL_ASSERT(b0 + bn <= s->bend_capacity);
    SL_ASSERT(e0 + en <= s->edge_capacity);

    /* Vertex -> bending stencil CSR. */
    uint32_t *bend_offset = s->vertex_bend_offset;
    SL_ASSERT(bend_offset[v0] == 4u * b0);
    for (uint32_t i = 1; i <= n; i++) {
        bend_offset[v0 + i] = 0u;
    }
    for (uint32_t b = 0; b < bn; b++) {
        for (uint32_t k = 0; k < 4u; k++) {
            bend_offset[s->bend_index[4u * (b0 + b) + k] + 1u]++;
        }
    }
    for (uint32_t i = 0; i < n; i++) {
        bend_offset[v0 + i + 1u] += bend_offset[v0 + i];
    }
    for (uint32_t i = 0; i < n; i++) {
        cursor[i] = bend_offset[v0 + i];
    }
    for (uint32_t b = 0; b < bn; b++) {
        for (uint32_t k = 0; k < 4u; k++) {
            const uint32_t v = s->bend_index[4u * (b0 + b) + k] - v0;
            s->vertex_bend[cursor[v]++] = 4u * (b0 + b) + k;
        }
    }

    const uint32_t color_count = sl_mesh_color(s, v0, n);
    if (color_count == 0u) {
        return false;
    }

    /* Commit: from here on nothing can fail. */
    sl_cloth_record *rec = &s->cloths[s->cloth_count];
    memset(rec, 0, sizeof(*rec));
    rec->vertex_offset = v0;
    rec->vertex_count = n;
    rec->triangle_offset = t0;
    rec->triangle_count = tn;
    rec->bend_offset = b0;
    rec->bend_count = bn;
    rec->edge_offset = e0;
    rec->edge_count = en;
    rec->color_count = color_count;
    rec->membrane_mu = desc->membrane_mu;
    rec->membrane_lambda = desc->membrane_lambda;
    rec->bending_stiffness = desc->bending_stiffness;
    rec->damping = desc->damping;
    rec->thickness = desc->thickness;
    rec->friction = desc->friction;
    rec->drag = desc->drag;
    /* Counting sort by color, ascending vertex order within a color. */
    const uint8_t *color = s->color_scratch + v0;
    for (uint32_t i = 0; i < n; i++) {
        rec->color_offsets[color[i] + 1u]++;
    }
    for (uint32_t c = 0; c < color_count; c++) {
        rec->color_offsets[c + 1u] += rec->color_offsets[c];
    }
    uint32_t fill[SL_CLOTH_COLOR_COUNT_MAX];
    memcpy(fill, rec->color_offsets, sizeof(fill));
    for (uint32_t i = 0; i < n; i++) {
        s->color_order[v0 + fill[color[i]]++] = v0 + i;
    }

    for (uint32_t i = 0; i < n; i++) {
        const uint32_t v = v0 + i;
        s->position[v] = desc->positions[i];
        s->position_start[v] = desc->positions[i];
        s->velocity[v] = sl_vec3_zero();
        s->inertia_target[v] = desc->positions[i];
        s->pin_target[v] = desc->positions[i];
        s->pin_origin[v] = desc->positions[i];
        s->mass[v] = 0.0f;
        s->vertex_cloth[v] = s->cloth_count;
        s->pinned[v] = 0u;
        s->attach_stiffness[v] = 0.0f;
        s->contact[v] = 0u;
    }
    for (uint32_t t = 0; t < tn; t++) {
        const float corner_mass =
            desc->density * s->triangle_area[t0 + t] / 3.0f;
        for (uint32_t k = 0; k < 3u; k++) {
            s->mass[s->triangle_index[3u * (t0 + t) + k]] += corner_mass;
        }
    }
    rec->stiffness_rate_max = sl_mesh_stiffness_rate_max(s, rec);
    s->stiffness_rate_max =
        sl_max(s->stiffness_rate_max, rec->stiffness_rate_max);
    s->vertex_count += n;
    s->triangle_count += tn;
    s->bend_count += bn;
    s->edge_count += en;
    *cloth = s->cloth_count;
    s->cloth_count++;
    if (sl_self_enabled(s)) {
        sl_self_build(s);
    }
    return true;
}
