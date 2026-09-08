#include "world_internal.h"

#include <math.h>
#include <string.h>

#include "joint.h"
#include "solver.h"
#include "stats.h"

static bool dynamic_slot(const sl_world_state *world, uint32_t slot)
{
    return slot < world->body_capacity &&
           world->slots[slot].dense != SL_BODY_DENSE_NONE &&
           world->types[world->slots[slot].dense] == (uint8_t)SL_BODY_DYNAMIC;
}

static void quiet_clear(sl_world_state *world, uint32_t row)
{
    world->quiet_steps[row] = 0u;
    world->quiet_translation[row] = 0.0f;
    world->quiet_rotation[row] = 0.0f;
}

static void row_wake(sl_world_state *world, uint32_t row)
{
    if (world->sleeping[row] != 0u) {
        SL_WORK_ADD(world, body_wakes, 1u);
        world->sleeping[row] = 0u;
    }
    quiet_clear(world, row);
}

bool sl_island_is_sleeping(const sl_world_state *world, uint32_t id)
{
    return world->sleep_enabled &&
           world->sleeping[world->slots[world->islands[id].root_slot].dense] !=
               0u;
}

void sl_wake_begin(sl_world_state *world)
{
    if (!world->sleep_enabled) {
        return;
    }
    SL_ASSERT(!world->wake_batch);
    world->wake_batch = true;
    world->wake_count = 0u;
    /* Two bit sets share parent scratch: visited body slots and expanded
     * retained components. Size scratch is a bounded body-slot queue. */
    memset(world->island_parents, 0,
           (size_t)world->body_capacity * sizeof(uint32_t));
    SL_WORK_ADD(world, wake_visits, world->body_capacity);
}

void sl_wake_seed(sl_world_state *world, uint32_t slot)
{
    if (!world->sleep_enabled || !world->wake_batch ||
        !dynamic_slot(world, slot)) {
        return;
    }
    if ((world->island_parents[slot] & 1u) == 0u) {
        SL_ASSERT(world->wake_count < world->body_capacity);
        world->island_parents[slot] |= 1u;
        world->island_sizes[world->wake_count++] = slot;
    }
}

static void joint_neighbors(sl_world_state *world, uint32_t slot)
{
    if (world->joint_capacity == 0u) {
        return;
    }
    uint32_t edge = world->body_joint_heads[slot];
    for (uint32_t count = 0u;
         edge != UINT32_MAX && count < world->joint_capacity; ++count) {
        SL_WORK_ADD(world, wake_visits, 1u);
        const uint32_t row = world->joint_slots[edge / 2u].dense;
        const uint32_t other = (edge % 2u) == 0u
                                   ? world->joint_bodies_b[row].index
                                   : world->joint_bodies_a[row].index;
        sl_wake_seed(world, other);
        edge = world->joint_edge_nexts[edge];
    }
}

void sl_wake_finish(sl_world_state *world)
{
    if (!world->sleep_enabled) {
        return;
    }
    SL_ASSERT(world->wake_batch);
    for (uint32_t head = 0u; head < world->wake_count; ++head) {
        const uint32_t slot = world->island_sizes[head];
        SL_WORK_ADD(world, wake_visits, 1u);
        row_wake(world, world->slots[slot].dense);
        const uint32_t id = world->body_islands[slot];
        if (id != SL_BODY_DENSE_NONE &&
            (world->island_parents[id] & 2u) == 0u) {
            world->island_parents[id] |= 2u;
            const sl_island *island = &world->islands[id];
            for (uint32_t i = 0u; i < island->body_count; ++i) {
                SL_WORK_ADD(world, wake_visits, 1u);
                const uint32_t member =
                    world->island_bodies[island->body_offset + i];
                /* Destroy and reuse clear membership before a new body can
                 * inhabit this slot. Old ranges must never capture it. */
                if (world->body_islands[member] == id) {
                    sl_wake_seed(world, member);
                }
            }
        }
        joint_neighbors(world, slot);
    }
    world->wake_count = 0u;
    world->wake_batch = false;
}

void sl_sleep_body_changed(sl_world_state *world, uint32_t slot)
{
    if (!world->sleep_enabled) {
        return;
    }
    sl_wake_begin(world);
    sl_wake_seed(world, slot);
    /* A boundary is not a graph vertex: seed its partners once, without
     * walking from one dynamic component through the boundary into others. */
    if (!dynamic_slot(world, slot)) {
        for (uint32_t row = 0u; row < world->contact_count; ++row) {
            SL_WORK_ADD(world, wake_visits, 1u);
            const sl_contact *c = &world->contacts[row];
            if (c->touching &&
                (c->body_a.index == slot || c->body_b.index == slot)) {
                sl_wake_seed(world, c->body_a.index);
                sl_wake_seed(world, c->body_b.index);
            }
        }
        joint_neighbors(world, slot);
    }
    sl_wake_finish(world);
}

bool sl_world_body_is_awake(const sl_world *owner, sl_body_handle handle)
{
    if (owner == NULL || owner->state == NULL ||
        !sl_body_is_valid(owner->state, handle)) {
        return false;
    }
    const sl_world_state *world = owner->state;
    const uint32_t row = world->slots[handle.index].dense;
    return world->types[row] == (uint8_t)SL_BODY_KINEMATIC ||
           (world->types[row] == (uint8_t)SL_BODY_DYNAMIC &&
            world->sleeping[row] == 0u);
}

bool sl_world_body_wake(sl_world *owner, sl_body_handle handle)
{
    if (owner == NULL || owner->state == NULL ||
        !sl_body_is_valid(owner->state, handle) ||
        !dynamic_slot(owner->state, handle.index)) {
        return false;
    }
    sl_sleep_body_changed(owner->state, handle.index);
    return true;
}

void sl_sleep_step_begin(sl_world_state *world, float dt)
{
    if (!world->sleep_enabled) {
        return;
    }
    if (world->sleep_dt != dt) {
        const double required =
            ceil((double)world->sleep_time_min / (double)dt);
        world->sleep_steps_required =
            required <= (double)UINT32_MAX ? (uint32_t)required : 0u;
        world->sleep_dt = dt;
        for (uint32_t row = 0u; row < world->body_count; ++row) {
            if (world->types[row] == (uint8_t)SL_BODY_DYNAMIC) {
                row_wake(world, row);
            }
        }
    }
    sl_wake_begin(world);
}

static bool kinematic_moving(const sl_world_state *world, uint32_t slot)
{
    const uint32_t row = world->slots[slot].dense;
    return world->types[row] == (uint8_t)SL_BODY_KINEMATIC &&
           (world->velocities[row].x != 0.0f ||
            world->velocities[row].y != 0.0f ||
            world->angular_velocities[row] != 0.0f);
}

static void boundary_mark(sl_world_state *world, uint32_t a, uint32_t b)
{
    if (kinematic_moving(world, a) && dynamic_slot(world, b)) {
        world->island_parents[world->body_islands[b]] = 1u;
    }
    if (kinematic_moving(world, b) && dynamic_slot(world, a)) {
        world->island_parents[world->body_islands[a]] = 1u;
    }
}

void sl_sleep_graph_ready(sl_world_state *world)
{
    if (!world->sleep_enabled) {
        return;
    }
    /* The graph build has finished using union scratch. Parent entries now
     * record moving boundaries through the remainder of this step. */
    for (uint32_t id = 0u; id < world->island_count; ++id) {
        world->island_parents[id] = 0u;
    }
    for (uint32_t i = 0u; i < world->island_contact_count; ++i) {
        const sl_contact *c = &world->contacts[world->island_contacts[i]];
        boundary_mark(world, c->body_a.index, c->body_b.index);
    }
    SL_WORK_ADD(world, wake_visits,
                world->island_contact_count + world->joint_count);
    for (uint32_t row = 0u; row < world->joint_count; ++row) {
        boundary_mark(world, world->joint_bodies_a[row].index,
                      world->joint_bodies_b[row].index);
    }
    for (uint32_t id = 0u; id < world->island_count; ++id) {
        const sl_island *island = &world->islands[id];
        uint32_t sleeping = 0u;
        for (uint32_t i = 0u; i < island->body_count; ++i) {
            const uint32_t row =
                world->slots[world->island_bodies[island->body_offset + i]]
                    .dense;
            SL_WORK_ADD(world, wake_visits, 1u);
            sleeping += world->sleeping[row] != 0u ? 1u : 0u;
        }
        if (sleeping > 0u && (sleeping < island->body_count ||
                              world->island_parents[id] != 0u)) {
            for (uint32_t i = 0u; i < island->body_count; ++i) {
                SL_WORK_ADD(world, wake_visits, 1u);
                row_wake(
                    world,
                    world->slots[world->island_bodies[island->body_offset + i]]
                        .dense);
            }
        }
    }
}

static float shape_radius(const sl_shape *shape)
{
    if (shape->kind == SL_SHAPE_CIRCLE) {
        return shape->circle.radius;
    }
    float radius = 0.0f;
    if (shape->kind == SL_SHAPE_POLYGON) {
        for (uint32_t i = 0u; i < shape->polygon.count; ++i) {
            radius = sl_max(radius, sl_vec2_length(shape->polygon.vertices[i]));
        }
    }
    return radius;
}

static bool speed_quiet(const sl_world_state *world, uint32_t row)
{
    const double speed = hypot((double)world->velocities[row].x,
                               (double)world->velocities[row].y);
    const double spin = fabs((double)world->angular_velocities[row]);
    return speed + (double)shape_radius(&world->shapes[row]) * spin <=
               (double)world->sleep_speed_max &&
           spin <= (double)world->sleep_angular_speed_max;
}

void sl_sleep_travel(sl_world_state *world, uint32_t row, sl_vec2 before,
                     sl_rotation rotation_before)
{
    if (!world->sleep_enabled ||
        world->types[row] != (uint8_t)SL_BODY_DYNAMIC) {
        return;
    }
    const float travel =
        sl_vec2_length(sl_vec2_sub(world->delta_positions[row], before));
    const sl_rotation now = world->delta_rotations[row];
    const float angle =
        sl_abs(atan2f(now.s * rotation_before.c - now.c * rotation_before.s,
                      now.c * rotation_before.c + now.s * rotation_before.s));
    world->quiet_translation[row] =
        sl_min(2.0f * SL_LINEAR_SLOP, world->quiet_translation[row] + travel);
    world->quiet_rotation[row] =
        sl_min(0.01f, world->quiet_rotation[row] + angle);
    if (!speed_quiet(world, row)) {
        world->quiet_translation[row] = 2.0f * SL_LINEAR_SLOP;
    }
}

void sl_sleep_step_end(sl_world_state *world)
{
    if (!world->sleep_enabled) {
        return;
    }
    for (uint32_t id = 0u; id < world->island_count; ++id) {
        if (sl_island_is_sleeping(world, id)) {
            continue;
        }
        const sl_island *island = &world->islands[id];
        bool quiet = world->sleep_steps_required > 0u &&
                     world->island_parents[id] == 0u &&
                     sl_solver_island_quiet(world, id) &&
                     sl_joint_island_quiet(world, id);
        uint32_t steps = UINT32_MAX;
        for (uint32_t i = 0u; i < island->body_count; ++i) {
            const uint32_t row =
                world->slots[world->island_bodies[island->body_offset + i]]
                    .dense;
            quiet = quiet && speed_quiet(world, row) &&
                    world->quiet_translation[row] <= SL_LINEAR_SLOP &&
                    world->quiet_rotation[row] <= 0.005f;
            if (world->quiet_steps[row] < steps) {
                steps = world->quiet_steps[row];
            }
        }
        if (steps < UINT32_MAX) {
            steps++;
        }
        for (uint32_t i = 0u; i < island->body_count; ++i) {
            const uint32_t row =
                world->slots[world->island_bodies[island->body_offset + i]]
                    .dense;
            if (!quiet) {
                quiet_clear(world, row);
                continue;
            }
            world->quiet_steps[row] = steps;
            if (steps >= world->sleep_steps_required) {
                world->sleeping[row] = 1u;
                world->velocities[row] = sl_vec2_make(0.0f, 0.0f);
                world->angular_velocities[row] = 0.0f;
                SL_WORK_ADD(world, body_sleeps, 1u);
            }
        }
    }
}
