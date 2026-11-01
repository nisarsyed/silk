#include "contact.h"

bool sl_vertex_block_contains(sl_vec3 x, sl_vec3 v, const sl_vec3 *neighbours,
                              uint32_t neighbour_count)
{
    const sl_vec3 d = sl_vec3_sub(x, v);
    for (uint32_t k = 0; k < neighbour_count; k++) {
        if (sl_vec3_dot(d, sl_vec3_sub(v, neighbours[k])) < 0.0f) {
            return false;
        }
    }
    return true;
}

bool sl_edge_block_contains(sl_vec3 x, sl_vec3 e0, sl_vec3 e1,
                            const sl_vec3 *opposite, uint32_t opposite_count)
{
    const sl_vec3 axis = sl_vec3_sub(e1, e0);
    if (!(sl_vec3_dot(sl_vec3_sub(x, e0), axis) > 0.0f) ||
        !(sl_vec3_dot(sl_vec3_sub(x, e1), sl_vec3_neg(axis)) > 0.0f)) {
        return false;
    }
    const float length_sq = sl_vec3_length_sq(axis);
    for (uint32_t k = 0; k < opposite_count; k++) {
        const float u =
            sl_vec3_dot(sl_vec3_sub(opposite[k], e0), axis) / length_sq;
        const sl_vec3 foot = sl_vec3_mul_add(e0, axis, u);
        if (sl_vec3_dot(sl_vec3_sub(x, foot), sl_vec3_sub(foot, opposite[k])) <
            0.0f) {
            return false;
        }
    }
    return true;
}

float sl_contact_energy(float d, float r, float k, float *slope,
                        float *curvature)
{
    SL_ASSERT(slope != NULL && curvature != NULL);
    SL_ASSERT(d > 0.0f && r > 0.0f && k >= 0.0f);
    if (d >= r) {
        *slope = 0.0f;
        *curvature = 0.0f;
        return 0.0f;
    }
    const float tau = 0.5f * r;
    if (d >= tau) {
        *slope = -k * (r - d);
        *curvature = k;
        return 0.5f * k * (r - d) * (r - d);
    }
    const float kl = k * tau * tau;
    const float b = 0.5f * k * (r - tau) * (r - tau) + kl * logf(tau);
    *slope = -kl / d;
    *curvature = kl / (d * d);
    return -kl * logf(d) + b;
}
