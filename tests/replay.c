#include "replay.h"
#include "tree.h"
#include "world_internal.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint32_t float_bits(float value)
{
    uint32_t bits = 0u;
    _Static_assert(sizeof(value) == sizeof(bits), "replay requires binary32");
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}
/* Each field is visited explicitly; pointers, padding and dead payloads never
 * enter the comparison. F also rejects two identically non-finite states. */
#define U(field)                                                               \
    do {                                                                       \
        if ((uint64_t)a->field != (uint64_t)b->field) {                        \
            out->entity = i;                                                   \
            out->field_name = #field;                                          \
            out->value_a = (uint64_t)a->field;                                 \
            out->value_b = (uint64_t)b->field;                                 \
            return false;                                                      \
        }                                                                      \
    } while (false)
#define F(field)                                                               \
    do {                                                                       \
        if (!isfinite(a->field) || !isfinite(b->field) ||                      \
            float_bits(a->field) != float_bits(b->field)) {                    \
            out->entity = i;                                                   \
            out->field_name = #field;                                          \
            out->value_a = float_bits(a->field);                               \
            out->value_b = float_bits(b->field);                               \
            return false;                                                      \
        }                                                                      \
    } while (false)
#define V(field)                                                               \
    do {                                                                       \
        F(field.x);                                                            \
        F(field.y);                                                            \
    } while (false)
#define H(field)                                                               \
    do {                                                                       \
        U(field.index);                                                        \
        U(field.generation);                                                   \
    } while (false)
static bool tree_compare(const sl_tree *a, const sl_tree *b,
                         sl_replay_mismatch *out)
{
    uint32_t i = 0u;
    U(node_capacity);
    U(proxy_capacity);
    U(node_count);
    U(proxy_count);
    U(free_list);
    for (i = 0u; i < SL_TREE_ROOT_COUNT; ++i) {
        U(roots[i]);
    }
    for (i = 0u; i < a->node_capacity; ++i) {
        U(nodes[i].height);
        U(nodes[i].parent);
        if (a->nodes[i].height == SL_TREE_HEIGHT_FREE) {
            continue;
        }
        U(nodes[i].child_1);
        U(nodes[i].child_2);
        if (a->nodes[i].height == 0u) {
            U(nodes[i].user);
        }
        V(nodes[i].aabb.lower);
        V(nodes[i].aabb.upper);
    }
    return true;
}
bool sl_replay_compare(const sl_world *owner_a, const sl_world *owner_b,
                       sl_replay_mismatch *out)
{
    const sl_world_state *a = owner_a->state;
    const sl_world_state *b = owner_b->state;
    uint32_t i = 0u;
    U(body_count);
    U(body_capacity);
    U(free_count);
    U(contact_count);
    U(contact_capacity);
    U(contact_drop_count);
    U(pair_capacity);
    U(moved_count);
    U(joint_count);
    U(joint_capacity);
    U(joint_free_count);
    U(substep_count);
    F(linear_drag);
    F(angular_drag);
    F(linear_speed_max);
    F(contact_hertz);
    F(contact_damping_ratio);
    F(contact_push_velocity_max);
    F(restitution_threshold);
    F(joint_hertz);
    F(joint_damping_ratio);
    U(work_total.tree_node_visits);
    U(step_stats.work.tree_node_visits);
    U(work_total.pair_candidates);
    U(step_stats.work.pair_candidates);
    U(work_total.pair_probes);
    U(step_stats.work.pair_probes);
    U(work_total.proxy_creates);
    U(step_stats.work.proxy_creates);
    U(work_total.proxy_destroys);
    U(step_stats.work.proxy_destroys);
    U(work_total.proxy_moves);
    U(step_stats.work.proxy_moves);
    U(work_total.contact_drops);
    U(step_stats.work.contact_drops);
    U(body_count_high);
    U(contact_count_high);
    U(joint_count_high);
    U(step_stats.dynamic_body_count);
    U(step_stats.kinematic_body_count);
    U(step_stats.contact_constraint_count);
    U(step_stats.joint_constraint_count);
    U(step_stats.substep_count);
    V(gravity);
    for (i = 0u; i < a->body_capacity; ++i) {
        U(slots[i].dense);
        U(slots[i].generation);
    }
    for (i = 0u; i < a->free_count; ++i) {
        U(free_indices[i]);
    }
    for (i = 0u; i < a->body_count; ++i) {
        U(slot_of[i]);
        U(types[i]);
        U(proxies[i]);
        U(moved[i]);
        V(positions[i]);
        V(velocities[i]);
        V(forces[i]);
        V(delta_positions[i]);
        F(masses[i]);
        F(inv_masses[i]);
        F(angular_velocities[i]);
        F(torques[i]);
        F(inertias[i]);
        F(inv_inertias[i]);
        F(frictions[i]);
        F(restitutions[i]);
        F(rotations[i].c);
        F(rotations[i].s);
        F(delta_rotations[i].c);
        F(delta_rotations[i].s);
        V(proxy_aabbs[i].lower);
        V(proxy_aabbs[i].upper);
        U(shapes[i].kind);
        if (a->shapes[i].kind == SL_SHAPE_CIRCLE) {
            F(shapes[i].circle.radius);
        }
        if (a->shapes[i].kind == SL_SHAPE_POLYGON) {
            U(shapes[i].polygon.count);
            for (uint32_t j = 0u; j < a->shapes[i].polygon.count; ++j) {
                V(shapes[i].polygon.vertices[j]);
            }
        }
    }
    for (i = 0u; i < a->moved_count; ++i) {
        U(moved_slots[i]);
    }
    for (i = 0u; i < a->pair_capacity; ++i) {
        U(pair_keys[i]);
    }
    for (i = 0u; i < a->contact_count; ++i) {
        H(contacts[i].body_a);
        H(contacts[i].body_b);
        F(contacts[i].friction);
        F(contacts[i].restitution);
        U(contacts[i].touching);
        V(contacts[i].manifold.normal);
        U(contacts[i].manifold.point_count);
        for (uint32_t j = 0u; j < a->contacts[i].manifold.point_count; ++j) {
            V(contacts[i].manifold.points[j].anchor_a);
            V(contacts[i].manifold.points[j].anchor_b);
            V(contacts[i].manifold.points[j].point);
            F(contacts[i].manifold.points[j].separation);
            F(contacts[i].manifold.points[j].normal_impulse);
            F(contacts[i].manifold.points[j].tangent_impulse);
            F(contacts[i].manifold.points[j].normal_velocity);
            U(contacts[i].manifold.points[j].id);
            U(contacts[i].manifold.points[j].persisted);
        }
    }
    for (i = 0u; i < a->joint_capacity; ++i) {
        U(joint_slots[i].dense);
        U(joint_slots[i].generation);
    }
    for (i = 0u; i < a->joint_free_count; ++i) {
        U(joint_free_indices[i]);
    }
    for (i = 0u; i < a->joint_count; ++i) {
        U(joint_slot_of[i]);
        U(joint_kinds[i]);
        U(joint_collide_connected[i]);
        H(joint_bodies_a[i]);
        H(joint_bodies_b[i]);
        V(joint_local_anchors_a[i]);
        V(joint_local_anchors_b[i]);
        V(joint_revolute_impulses[i]);
        V(joint_linear_impulses[i]);
        F(joint_distance_lengths[i]);
        F(joint_distance_impulses[i]);
        const uint32_t edge = 2u * a->joint_slot_of[i];
        U(joint_edge_prevs[edge]);
        U(joint_edge_nexts[edge]);
        U(joint_edge_prevs[edge + 1u]);
        U(joint_edge_nexts[edge + 1u]);
    }
    if (a->joint_capacity != 0u) {
        for (i = 0u; i < a->body_capacity; ++i) {
            U(body_joint_heads[i]);
            U(body_joint_counts[i]);
        }
    }
    /* Tree scratch is overwritten for every traversal. Free nodes only
     * contribute their next-free link and height marker. */
    return tree_compare(a->tree, b->tree, out);
}
bool sl_replay_check(const sl_world *a, const sl_world *b, const char *fixture,
                     uint32_t seed, uint32_t operation)
{
    sl_replay_mismatch m = { .fixture = fixture,
                             .seed = seed,
                             .operation = operation };
    if (sl_replay_compare(a, b, &m)) {
        return true;
    }
    fprintf(stderr,
            "%s seed=0x%08" PRIx32 " operation=%" PRIu32 " entity=%" PRIu32
            " field=%s a=0x%016" PRIx64 " b=0x%016" PRIx64 "\n",
            m.fixture, m.seed, m.operation, m.entity, m.field_name, m.value_a,
            m.value_b);
    return false;
}
