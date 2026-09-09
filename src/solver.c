#include "solver.h"
#include "world_internal.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "silk/assert.h"

typedef struct sl_contact_softness {
    float bias_rate;
    float mass_scale;
    float impulse_scale;
} sl_contact_softness;

typedef struct sl_contact_constraint_point {
    /* Fixed world-space offsets from each body origin at prepare time.
     * Delta rotation updates separation, but the scalar Jacobian keeps these
     * anchors fixed for every substep. */
    sl_vec2 anchor_a;
    sl_vec2 anchor_b;
    float base_separation;
    float normal_mass;
    float tangent_mass;
    float initial_normal_velocity;
    float normal_impulse;
    float tangent_impulse;
    float max_normal_impulse;
    uint32_t id;
} sl_contact_constraint_point;

typedef struct sl_contact_constraint {
    uint32_t contact_row;
    uint32_t dense_a;
    uint32_t dense_b;
    sl_vec2 normal;
    float friction;
    float restitution;
    sl_contact_softness softness;
    /* Off-diagonal K term coupling the two normal constraints. The diagonal
     * terms are the reciprocals of each point's normal_mass. */
    float normal_coupling;
    uint32_t point_count;
    sl_contact_constraint_point points[SL_MANIFOLD_POINT_COUNT_MAX];
} sl_contact_constraint;

#define SL_SOLVER_CARVE_ALIGN ((size_t)8u)

_Static_assert(_Alignof(sl_contact_constraint) <= SL_SOLVER_CARVE_ALIGN &&
                   SL_SOLVER_CARVE_ALIGN % _Alignof(sl_contact_constraint) ==
                       0u,
               "contact constraint alignment violates the arena contract");
_Static_assert(sizeof(sl_contact_constraint) == 144u,
               "constraint layout changed; re-pin the world arena budget");

size_t sl_solver_memory_bytes(uint32_t contact_capacity)
{
    if (contact_capacity < 1u || contact_capacity > SL_CONTACT_COUNT_MAX ||
        sizeof(sl_contact_constraint) > SIZE_MAX / contact_capacity) {
        return 0u;
    }
    const size_t bytes =
        (size_t)contact_capacity * sizeof(sl_contact_constraint);
    if (bytes > SIZE_MAX - (SL_SOLVER_CARVE_ALIGN - 1u)) {
        return 0u;
    }
    return (bytes + SL_SOLVER_CARVE_ALIGN - 1u) & ~(SL_SOLVER_CARVE_ALIGN - 1u);
}

bool sl_solver_init(sl_world_state *world, void *memory, size_t memory_bytes)
{
    SL_ASSERT(world != NULL);
    const size_t required = sl_solver_memory_bytes(world->contact_capacity);
    if (memory == NULL || required == 0u || memory_bytes < required ||
        ((uintptr_t)memory & (SL_SOLVER_CARVE_ALIGN - 1u)) != 0u) {
        return false;
    }
    memset(memory, 0, required);
    world->contact_constraints = memory;
    world->contact_constraint_count = 0u;
    return true;
}

static sl_contact_constraint *world_constraints(sl_world_state *world)
{
    return (sl_contact_constraint *)world->contact_constraints;
}

static sl_contact_softness softness_make(float hertz, float damping_ratio,
                                         float h)
{
    /* Implicit damped correction, rearranged as a projected impulse update.
     * With omega = 2*pi*f and a1 = 2*zeta + h*omega:
     *
     *   delta_lambda = -M * mass_scale * (v_n + bias_rate*C)
     *                  -impulse_scale * lambda
     *
     * where mass_scale = a2/(1+a2), impulse_scale = 1/(1+a2), and
     * a2 = h*omega*a1. Their sum is exactly one in real arithmetic. The
     * hertz cap in prepare bounds h*omega; initialization validates the
     * worst damping-derived terms before any state is allocated. */
    const float omega = 2.0f * SL_PI * hertz;
    const float h_omega = h * omega;
    const float a_1 = 2.0f * damping_ratio + h_omega;
    const float a_2 = h_omega * a_1;
    const float a_3 = 1.0f / (1.0f + a_2);
    const sl_contact_softness softness = {
        .bias_rate = omega / a_1,
        .mass_scale = a_2 * a_3,
        .impulse_scale = a_3,
    };
    const bool valid =
        sl_is_finite(softness.bias_rate) && softness.bias_rate >= 0.0f &&
        sl_is_finite(softness.mass_scale) && softness.mass_scale >= 0.0f &&
        softness.mass_scale <= 1.0f && sl_is_finite(softness.impulse_scale) &&
        softness.impulse_scale >= 0.0f && softness.impulse_scale <= 1.0f;
    SL_ASSERT(valid);
    if (!valid) {
        return (sl_contact_softness){ 0.0f, 1.0f, 0.0f };
    }
    return softness;
}

static float effective_mass(const sl_world_state *world, uint32_t dense_a,
                            uint32_t dense_b, sl_vec2 anchor_a,
                            sl_vec2 anchor_b, sl_vec2 axis)
{
    const float cross_a = sl_vec2_cross(anchor_a, axis);
    const float cross_b = sl_vec2_cross(anchor_b, axis);
    const float square_a = cross_a * cross_a;
    const float square_b = cross_b * cross_b;
    const float rotation_a = world->inv_inertias[dense_a] * square_a;
    const float rotation_b = world->inv_inertias[dense_b] * square_b;
    if (!sl_is_finite(square_a) || !sl_is_finite(square_b) ||
        !sl_is_finite(rotation_a) || !sl_is_finite(rotation_b)) {
        return 0.0f;
    }
    float inverse_mass =
        world->inv_masses[dense_a] + world->inv_masses[dense_b];
    if (!sl_is_finite(inverse_mass)) {
        return 0.0f;
    }
    inverse_mass += rotation_a;
    if (!sl_is_finite(inverse_mass)) {
        return 0.0f;
    }
    inverse_mass += rotation_b;
    if (!sl_is_finite(inverse_mass) || inverse_mass <= 0.0f) {
        return 0.0f;
    }
    const float mass = 1.0f / inverse_mass;
    return (sl_is_finite(mass) && mass > 0.0f) ? mass : 0.0f;
}

static float effective_mass_coupling(const sl_world_state *world,
                                     uint32_t dense_a, uint32_t dense_b,
                                     sl_vec2 anchor_a_1, sl_vec2 anchor_b_1,
                                     sl_vec2 anchor_a_2, sl_vec2 anchor_b_2,
                                     sl_vec2 axis)
{
    const float cross_a_1 = sl_vec2_cross(anchor_a_1, axis);
    const float cross_b_1 = sl_vec2_cross(anchor_b_1, axis);
    const float cross_a_2 = sl_vec2_cross(anchor_a_2, axis);
    const float cross_b_2 = sl_vec2_cross(anchor_b_2, axis);
    const float coupling =
        world->inv_masses[dense_a] + world->inv_masses[dense_b] +
        world->inv_inertias[dense_a] * cross_a_1 * cross_a_2 +
        world->inv_inertias[dense_b] * cross_b_1 * cross_b_2;
    /* FLT_MAX is a scratch-only invalid marker. An exact coupling at that
     * extreme is conservatively sent through the scalar fallback too. */
    return sl_is_finite(coupling) ? coupling : FLT_MAX;
}

/* Keep both bodies' solver state in one stack record while a complete
 * manifold is consumed. A two-point contact otherwise reloads and stores the
 * same scattered world arrays four times in one normal/friction sweep. */
typedef struct sl_velocity_pair {
    sl_vec2 linear_a;
    sl_vec2 linear_b;
    float angular_a;
    float angular_b;
    float inverse_mass_a;
    float inverse_mass_b;
    float inverse_inertia_a;
    float inverse_inertia_b;
    bool dynamic_a;
    bool dynamic_b;
} sl_velocity_pair;

static sl_velocity_pair
velocity_pair_load(const sl_world_state *world,
                   const sl_contact_constraint *constraint)
{
    const uint32_t dense_a = constraint->dense_a;
    const uint32_t dense_b = constraint->dense_b;
    const sl_velocity_pair pair = {
        .linear_a = world->velocities[dense_a],
        .linear_b = world->velocities[dense_b],
        .angular_a = world->angular_velocities[dense_a],
        .angular_b = world->angular_velocities[dense_b],
        .inverse_mass_a = world->inv_masses[dense_a],
        .inverse_mass_b = world->inv_masses[dense_b],
        .inverse_inertia_a = world->inv_inertias[dense_a],
        .inverse_inertia_b = world->inv_inertias[dense_b],
        .dynamic_a = world->types[dense_a] == (uint8_t)SL_BODY_DYNAMIC,
        .dynamic_b = world->types[dense_b] == (uint8_t)SL_BODY_DYNAMIC,
    };
    return pair;
}

static void velocity_pair_store(sl_world_state *world,
                                const sl_contact_constraint *constraint,
                                const sl_velocity_pair *pair)
{
    if (pair->dynamic_a) {
        world->velocities[constraint->dense_a] = pair->linear_a;
        world->angular_velocities[constraint->dense_a] = pair->angular_a;
    }
    if (pair->dynamic_b) {
        world->velocities[constraint->dense_b] = pair->linear_b;
        world->angular_velocities[constraint->dense_b] = pair->angular_b;
    }
}

static float relative_velocity(const sl_velocity_pair *pair, sl_vec2 anchor_a,
                               sl_vec2 anchor_b, sl_vec2 axis)
{
    const sl_vec2 velocity_a = sl_vec2_add(
        pair->linear_a, sl_scalar_cross_vec2(pair->angular_a, anchor_a));
    const sl_vec2 velocity_b = sl_vec2_add(
        pair->linear_b, sl_scalar_cross_vec2(pair->angular_b, anchor_b));
    return sl_vec2_dot(sl_vec2_sub(velocity_b, velocity_a), axis);
}

static bool impulse_apply(sl_velocity_pair *pair, sl_vec2 anchor_a,
                          sl_vec2 anchor_b, sl_vec2 impulse)
{
    if (!sl_vec2_is_finite(impulse)) {
        return false;
    }

    sl_vec2 linear_a = pair->linear_a;
    sl_vec2 linear_b = pair->linear_b;
    float angular_a = pair->angular_a;
    float angular_b = pair->angular_b;
    if (pair->dynamic_a) {
        linear_a =
            sl_vec2_sub(linear_a, sl_vec2_scale(impulse, pair->inverse_mass_a));
        angular_a -= pair->inverse_inertia_a * sl_vec2_cross(anchor_a, impulse);
    }
    if (pair->dynamic_b) {
        linear_b =
            sl_vec2_add(linear_b, sl_vec2_scale(impulse, pair->inverse_mass_b));
        angular_b += pair->inverse_inertia_b * sl_vec2_cross(anchor_b, impulse);
    }
    if (!sl_vec2_is_finite(linear_a) || !sl_vec2_is_finite(linear_b) ||
        !sl_is_finite(angular_a) || !sl_is_finite(angular_b)) {
        return false;
    }

    pair->linear_a = linear_a;
    pair->linear_b = linear_b;
    pair->angular_a = angular_a;
    pair->angular_b = angular_b;
    return true;
}

void sl_solver_prepare(sl_world_state *world, float h, float inverse_h)
{
    SL_ASSERT(world != NULL);
    SL_ASSERT(sl_is_finite(h) && h > 0.0f);
    SL_ASSERT(sl_is_finite(inverse_h) && inverse_h > 0.0f);
    sl_contact_constraint *constraints = world_constraints(world);
    uint32_t constraint_count = 0u;

    for (uint32_t index = 0u; index < world->island_contact_count; ++index) {
        const uint32_t row = world->island_contacts[index];
        const sl_contact *contact = &world->contacts[row];
        if (!contact->touching) {
            continue;
        }
        SL_ASSERT(constraint_count < world->contact_capacity);
        sl_contact_constraint *constraint = &constraints[constraint_count];
        memset(constraint, 0, sizeof(*constraint));
        constraint_count += 1u;

        constraint->contact_row = row;
        constraint->dense_a = world->slots[contact->body_a.index].dense;
        constraint->dense_b = world->slots[contact->body_b.index].dense;
        constraint->normal = contact->manifold.normal;
        constraint->friction = contact->friction;
        constraint->restitution = contact->restitution;
        constraint->point_count = contact->manifold.point_count;

        float hertz = world->contact_hertz;
        const bool static_partner =
            world->types[constraint->dense_a] == (uint8_t)SL_BODY_STATIC ||
            world->types[constraint->dense_b] == (uint8_t)SL_BODY_STATIC;
        if (static_partner) {
            hertz *= 2.0f;
        }
        hertz = sl_min(hertz, 0.25f * inverse_h);
        constraint->softness =
            softness_make(hertz, world->contact_damping_ratio, h);

        const sl_velocity_pair pair = velocity_pair_load(world, constraint);
        const sl_vec2 tangent = sl_vec2_right_perp(constraint->normal);
        for (uint32_t i = 0u; i < constraint->point_count; ++i) {
            const sl_manifold_point *manifold_point =
                &contact->manifold.points[i];
            sl_contact_constraint_point *point = &constraint->points[i];
            point->anchor_a = manifold_point->anchor_a;
            point->anchor_b = manifold_point->anchor_b;
            point->base_separation =
                manifold_point->separation -
                sl_vec2_dot(sl_vec2_sub(point->anchor_b, point->anchor_a),
                            constraint->normal);
            point->normal_mass = effective_mass(
                world, constraint->dense_a, constraint->dense_b,
                point->anchor_a, point->anchor_b, constraint->normal);
            point->tangent_mass =
                effective_mass(world, constraint->dense_a, constraint->dense_b,
                               point->anchor_a, point->anchor_b, tangent);
            point->initial_normal_velocity = relative_velocity(
                &pair, point->anchor_a, point->anchor_b, constraint->normal);
            point->id = manifold_point->id;
            if (point->normal_mass > 0.0f) {
                point->normal_impulse = manifold_point->normal_impulse;
            }
            if (point->tangent_mass > 0.0f) {
                point->tangent_impulse = manifold_point->tangent_impulse;
            }
        }
        if (constraint->point_count == 2u &&
            constraint->points[0].normal_mass > 0.0f &&
            constraint->points[1].normal_mass > 0.0f) {
            constraint->normal_coupling = effective_mass_coupling(
                world, constraint->dense_a, constraint->dense_b,
                constraint->points[0].anchor_a, constraint->points[0].anchor_b,
                constraint->points[1].anchor_a, constraint->points[1].anchor_b,
                constraint->normal);
        }
    }
    world->contact_constraint_count = constraint_count;
}

void sl_solver_warm_start(sl_world_state *world)
{
    SL_ASSERT(world != NULL);
    sl_contact_constraint *constraints = world_constraints(world);
    for (uint32_t row = 0u; row < world->contact_constraint_count; ++row) {
        sl_contact_constraint *constraint = &constraints[row];
        sl_velocity_pair pair = velocity_pair_load(world, constraint);
        const sl_vec2 tangent = sl_vec2_right_perp(constraint->normal);
        for (uint32_t i = 0u; i < constraint->point_count; ++i) {
            sl_contact_constraint_point *point = &constraint->points[i];
            const sl_vec2 impulse = sl_vec2_add(
                sl_vec2_scale(constraint->normal, point->normal_impulse),
                sl_vec2_scale(tangent, point->tangent_impulse));
            (void)impulse_apply(&pair, point->anchor_a, point->anchor_b,
                                impulse);
        }
        velocity_pair_store(world, constraint, &pair);
    }
}

static float separation_current(const sl_world_state *world,
                                const sl_contact_constraint *constraint,
                                const sl_contact_constraint_point *point)
{
    const sl_vec2 rotated_a = sl_rotation_apply(
        world->delta_rotations[constraint->dense_a], point->anchor_a);
    const sl_vec2 rotated_b = sl_rotation_apply(
        world->delta_rotations[constraint->dense_b], point->anchor_b);
    const sl_vec2 delta_a =
        sl_vec2_add(world->delta_positions[constraint->dense_a], rotated_a);
    const sl_vec2 delta_b =
        sl_vec2_add(world->delta_positions[constraint->dense_b], rotated_b);
    return point->base_separation +
           sl_vec2_dot(sl_vec2_sub(delta_b, delta_a), constraint->normal);
}

static float speculative_bias(float separation, float inverse_h)
{
    if (separation <= 0.0f) {
        return 0.0f;
    }
    if (separation > FLT_MAX / inverse_h) {
        return FLT_MAX;
    }
    return separation * inverse_h;
}

static float penetration_bias(const sl_world_state *world, float separation,
                              float bias_rate)
{
    if (separation >= 0.0f) {
        return 0.0f;
    }
    float bias = separation * bias_rate;
    if (!sl_is_finite(bias)) {
        bias = -world->contact_push_velocity_max;
    }
    return sl_max(bias, -world->contact_push_velocity_max);
}

typedef struct sl_normal_terms {
    float bias;
    float mass_scale;
    float impulse_scale;
} sl_normal_terms;

static sl_normal_terms normal_terms_make(
    const sl_world_state *world, const sl_contact_constraint *constraint,
    const sl_contact_constraint_point *point, float inverse_h, bool use_bias)
{
    sl_normal_terms terms = { 0.0f, 1.0f, 0.0f };
    if (!use_bias) {
        return terms;
    }

    const float separation = separation_current(world, constraint, point);
    if (separation > 0.0f) {
        terms.bias = speculative_bias(separation, inverse_h);
    } else {
        terms.bias =
            penetration_bias(world, separation, constraint->softness.bias_rate);
        terms.mass_scale = constraint->softness.mass_scale;
        terms.impulse_scale = constraint->softness.impulse_scale;
    }
    return terms;
}

static void normal_solve(const sl_world_state *world,
                         sl_contact_constraint *constraint,
                         sl_contact_constraint_point *point,
                         sl_velocity_pair *pair, float inverse_h, bool use_bias,
                         sl_vec2 anchor_a, sl_vec2 anchor_b)
{
    if (point->normal_mass == 0.0f) {
        return;
    }
    const float velocity =
        relative_velocity(pair, anchor_a, anchor_b, constraint->normal);
    if (!sl_is_finite(velocity)) {
        return;
    }

    const sl_normal_terms terms =
        normal_terms_make(world, constraint, point, inverse_h, use_bias);

    const float impulse =
        -point->normal_mass * terms.mass_scale * (velocity + terms.bias) -
        terms.impulse_scale * point->normal_impulse;
    const float accumulated = point->normal_impulse + impulse;
    if (!sl_is_finite(impulse) || !sl_is_finite(accumulated)) {
        return;
    }
    const float next_impulse = sl_max(accumulated, 0.0f);
    const float applied = next_impulse - point->normal_impulse;
    const sl_vec2 vector = sl_vec2_scale(constraint->normal, applied);
    if (!impulse_apply(pair, anchor_a, anchor_b, vector)) {
        return;
    }
    point->normal_impulse = next_impulse;
    point->max_normal_impulse = sl_max(point->max_normal_impulse, applied);
}

static bool normal_block_apply(sl_velocity_pair *pair,
                               const sl_contact_constraint *constraint,
                               sl_vec2 impulse_delta)
{
    sl_velocity_pair candidate = *pair;
    const sl_vec2 impulse_1 =
        sl_vec2_scale(constraint->normal, impulse_delta.x);
    const sl_vec2 impulse_2 =
        sl_vec2_scale(constraint->normal, impulse_delta.y);
    if (!impulse_apply(&candidate, constraint->points[0].anchor_a,
                       constraint->points[0].anchor_b, impulse_1) ||
        !impulse_apply(&candidate, constraint->points[1].anchor_a,
                       constraint->points[1].anchor_b, impulse_2)) {
        return false;
    }
    *pair = candidate;
    return true;
}

/* Solve the two unilateral normal constraints as one 2x2 LCP. Scalar PGS
 * makes the second point inherit the angular velocity created by the first;
 * enumerating the four possible active sets removes that artificial point
 * ordering. Soft penetration adds diagonal regularization; speculative rows
 * use the unregularized matrix. */
static bool normal_block_solve(const sl_world_state *world,
                               sl_contact_constraint *constraint,
                               sl_velocity_pair *pair, float inverse_h)
{
    if (constraint->point_count != 2u ||
        constraint->points[0].normal_mass == 0.0f ||
        constraint->points[1].normal_mass == 0.0f) {
        return false;
    }

    sl_contact_constraint_point *point_1 = &constraint->points[0];
    sl_contact_constraint_point *point_2 = &constraint->points[1];
    const float k_11 = 1.0f / point_1->normal_mass;
    const float k_22 = 1.0f / point_2->normal_mass;
    const float k_12 = constraint->normal_coupling;
    const float velocity_1 = relative_velocity(
        pair, point_1->anchor_a, point_1->anchor_b, constraint->normal);
    const float velocity_2 = relative_velocity(
        pair, point_2->anchor_a, point_2->anchor_b, constraint->normal);
    const sl_normal_terms terms_1 =
        normal_terms_make(world, constraint, point_1, inverse_h, true);
    const sl_normal_terms terms_2 =
        normal_terms_make(world, constraint, point_2, inverse_h, true);
    if (!sl_is_finite(k_11) || !sl_is_finite(k_22) || k_12 == FLT_MAX ||
        !sl_is_finite(velocity_1) || !sl_is_finite(velocity_2) ||
        terms_1.mass_scale <= 0.0f || terms_2.mass_scale <= 0.0f) {
        return false;
    }

    const sl_mat2 matrix = {
        k_11 / terms_1.mass_scale,
        k_12,
        k_12,
        k_22 / terms_2.mass_scale,
    };
    if (!sl_is_finite(matrix.m00) || !sl_is_finite(matrix.m11)) {
        return false;
    }
    /* For accumulated impulse x, solve x >= 0, A*x+b >= 0, and
     * x_i*(A*x+b)_i = 0, where b = v+bias-K*lambda_old. Dividing each K
     * diagonal by mass_scale is equivalent to adding
     * gamma_i = K_ii*impulse_scale/mass_scale, so the active-set solve
     * matches the scalar regularized update above. */
    const sl_vec2 accumulated =
        sl_vec2_make(point_1->normal_impulse, point_2->normal_impulse);
    const sl_vec2 b = sl_vec2_make(
        velocity_1 + terms_1.bias - k_11 * accumulated.x - k_12 * accumulated.y,
        velocity_2 + terms_2.bias - k_12 * accumulated.x -
            k_22 * accumulated.y);
    if (!sl_vec2_is_finite(b)) {
        return false;
    }

    sl_vec2 next = sl_vec2_make(0.0f, 0.0f);
    bool solved = sl_mat2_solve(matrix, sl_vec2_scale(b, -1.0f), &next) &&
                  next.x >= 0.0f && next.y >= 0.0f;
    if (!solved) {
        next.x = -b.x / matrix.m00;
        next.y = 0.0f;
        const float residual_2 = matrix.m10 * next.x + b.y;
        solved = sl_vec2_is_finite(next) && sl_is_finite(residual_2) &&
                 next.x >= 0.0f && residual_2 >= 0.0f;
    }
    if (!solved) {
        next.x = 0.0f;
        next.y = -b.y / matrix.m11;
        const float residual_1 = matrix.m01 * next.y + b.x;
        solved = sl_vec2_is_finite(next) && sl_is_finite(residual_1) &&
                 next.y >= 0.0f && residual_1 >= 0.0f;
    }
    if (!solved) {
        next = sl_vec2_make(0.0f, 0.0f);
        solved = b.x >= 0.0f && b.y >= 0.0f;
    }
    if (!solved) {
        return false;
    }

    const sl_vec2 applied = sl_vec2_sub(next, accumulated);
    if (!normal_block_apply(pair, constraint, applied)) {
        return false;
    }
    point_1->normal_impulse = next.x;
    point_2->normal_impulse = next.y;
    point_1->max_normal_impulse =
        sl_max(point_1->max_normal_impulse, applied.x);
    point_2->max_normal_impulse =
        sl_max(point_2->max_normal_impulse, applied.y);
    return true;
}

static void friction_solve(sl_contact_constraint *constraint,
                           sl_contact_constraint_point *point,
                           sl_velocity_pair *pair, sl_vec2 tangent,
                           sl_vec2 anchor_a, sl_vec2 anchor_b)
{
    if (point->tangent_mass == 0.0f) {
        return;
    }
    const float velocity = relative_velocity(pair, anchor_a, anchor_b, tangent);
    const float impulse = -point->tangent_mass * velocity;
    const float accumulated = point->tangent_impulse + impulse;
    if (!sl_is_finite(velocity) || !sl_is_finite(impulse) ||
        !sl_is_finite(accumulated)) {
        return;
    }
    float limit = constraint->friction * point->normal_impulse;
    if (!sl_is_finite(limit)) {
        limit = FLT_MAX;
    }
    const float next_impulse = sl_clamp(accumulated, -limit, limit);
    const float applied = next_impulse - point->tangent_impulse;
    const sl_vec2 vector = sl_vec2_scale(tangent, applied);
    if (!impulse_apply(pair, anchor_a, anchor_b, vector)) {
        return;
    }
    point->tangent_impulse = next_impulse;
}

static void constraint_solve(sl_world_state *world,
                             sl_contact_constraint *constraint, float inverse_h,
                             bool use_bias)
{
    sl_velocity_pair pair = velocity_pair_load(world, constraint);
    /* The biased pass gets the exact two-point normal LCP. The later unbiased
     * relaxation is deliberately a cheaper scalar smoothing pass; using a
     * second block solve did not improve the stress scenes enough to repay
     * its matrix work. */
    const bool block_solved =
        use_bias && normal_block_solve(world, constraint, &pair, inverse_h);
    if (!block_solved) {
        for (uint32_t i = 0u; i < constraint->point_count; ++i) {
            sl_contact_constraint_point *point = &constraint->points[i];
            normal_solve(world, constraint, point, &pair, inverse_h, use_bias,
                         point->anchor_a, point->anchor_b);
        }
    }
    const sl_vec2 tangent = sl_vec2_right_perp(constraint->normal);
    for (uint32_t i = 0u; i < constraint->point_count; ++i) {
        sl_contact_constraint_point *point = &constraint->points[i];
        friction_solve(constraint, point, &pair, tangent, point->anchor_a,
                       point->anchor_b);
    }
    velocity_pair_store(world, constraint, &pair);
}

void sl_solver_solve(sl_world_state *world, float inverse_h, bool use_bias)
{
    SL_ASSERT(world != NULL);
    sl_contact_constraint *constraints = world_constraints(world);
    for (uint32_t row = 0u; row < world->contact_constraint_count; ++row) {
        constraint_solve(world, &constraints[row], inverse_h, use_bias);
    }
}

void sl_solver_restitution(sl_world_state *world)
{
    SL_ASSERT(world != NULL);
    sl_contact_constraint *constraints = world_constraints(world);
    for (uint32_t row = 0u; row < world->contact_constraint_count; ++row) {
        sl_contact_constraint *constraint = &constraints[row];
        if (constraint->restitution == 0.0f) {
            continue;
        }
        sl_velocity_pair pair = velocity_pair_load(world, constraint);
        for (uint32_t i = 0u; i < constraint->point_count; ++i) {
            sl_contact_constraint_point *point = &constraint->points[i];
            if (point->normal_mass == 0.0f ||
                point->initial_normal_velocity >=
                    -world->restitution_threshold ||
                point->max_normal_impulse <= 0.0f) {
                continue;
            }
            const float velocity = relative_velocity(
                &pair, point->anchor_a, point->anchor_b, constraint->normal);
            const float target =
                -constraint->restitution * point->initial_normal_velocity;
            const float impulse = point->normal_mass * (target - velocity);
            const float accumulated = point->normal_impulse + impulse;
            if (!sl_is_finite(velocity) || !sl_is_finite(target) ||
                !sl_is_finite(impulse) || !sl_is_finite(accumulated)) {
                continue;
            }
            const float next_impulse = sl_max(accumulated, 0.0f);
            const float applied = next_impulse - point->normal_impulse;
            const sl_vec2 vector = sl_vec2_scale(constraint->normal, applied);
            if (!impulse_apply(&pair, point->anchor_a, point->anchor_b,
                               vector)) {
                continue;
            }
            point->normal_impulse = next_impulse;
            point->max_normal_impulse =
                sl_max(point->max_normal_impulse, applied);
        }
        velocity_pair_store(world, constraint, &pair);
    }
}

void sl_solver_store(sl_world_state *world)
{
    SL_ASSERT(world != NULL);
    sl_contact_constraint *constraints = world_constraints(world);
    for (uint32_t row = 0u; row < world->contact_constraint_count; ++row) {
        const sl_contact_constraint *constraint = &constraints[row];
        SL_ASSERT(constraint->contact_row < world->contact_count);
        sl_contact *contact = &world->contacts[constraint->contact_row];
        for (uint32_t i = 0u; i < constraint->point_count; ++i) {
            const sl_contact_constraint_point *point = &constraint->points[i];
            sl_manifold_point *manifold_point = &contact->manifold.points[i];
            SL_ASSERT(manifold_point->id == point->id);
            if (manifold_point->id != point->id) {
                continue;
            }
            manifold_point->normal_impulse = point->normal_impulse;
            manifold_point->tangent_impulse = point->tangent_impulse;
            manifold_point->normal_velocity = point->initial_normal_velocity;
        }
    }
}
