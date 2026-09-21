#include "fixtures.h"
#include "quality.h"
#include <math.h>
#include <string.h>

static const sl_vec2 k_gravity = { 0.0f, -9.81f };

static uint32_t rng_next(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    *state = x;
    return x;
}

/* Exactly representable uniform variate in [0, 1). */
static float rng_unit(uint32_t *state)
{
    return (float)(rng_next(state) >> 8u) * (1.0f / 16777216.0f);
}

static float rng_range(uint32_t *state, float low, float high)
{
    return low + (high - low) * rng_unit(state);
}

static bool scene_body_add(sl_bench_scene *scene, const sl_body_desc *desc)
{
    if (scene->body_count >= SL_BENCH_BODY_COUNT_MAX) {
        return false;
    }
    const sl_body_handle body = sl_world_body_create(scene->world, desc);
    if (sl_body_handle_is_null(body)) {
        return false;
    }
    if (scene->body_count > 0u &&
        scene->bodies[scene->body_count - 1u].index >= body.index) {
        return false;
    }
    scene->bodies[scene->body_count] = body;
    scene->body_count += 1u;
    return true;
}

static bool scene_world_init(sl_bench_scene *scene, const char *name,
                             sl_world_config config, uint32_t seed)
{
    if (!sl_world_init(scene->world, &config)) {
        return false;
    }
    scene->config = config;
    scene->name = name;
    scene->seed = seed;
    return true;
}

static bool pyramid_build(sl_bench_scene *scene, bool sleep_enabled)
{
    if (!scene_world_init(
            scene, "pyramid",
            (sl_world_config){ .sleep_enabled = sleep_enabled,
                               .body_capacity = SL_BENCH_PYRAMID_BODY_COUNT,
                               .gravity = k_gravity,
                               .substep_count = SL_BENCH_SUBSTEP_COUNT },
            0u)) {
        return false;
    }

    sl_shape ground = sl_shape_none();
    sl_shape box = sl_shape_none();
    if (!sl_shape_make_box(12.0f, 0.5f, &ground) ||
        !sl_shape_make_box(0.5f, 0.5f, &box)) {
        return false;
    }
    const sl_body_desc ground_desc = {
        .position = { 0.0f, -0.5f },
        .type = SL_BODY_STATIC,
        .shape = &ground,
        .friction = 0.6f,
    };
    if (!scene_body_add(scene, &ground_desc)) {
        return false;
    }

    for (uint32_t row = 0u; row < SL_BENCH_PYRAMID_ROW_COUNT; ++row) {
        const uint32_t row_body_count = SL_BENCH_PYRAMID_ROW_COUNT - row;
        for (uint32_t column = 0u; column < row_body_count; ++column) {
            const sl_body_desc desc = {
                .position = {
                    ((float)column - 0.5f * (float)(row_body_count - 1u)) *
                        1.01f,
                    0.5f + (float)row * 1.01f,
                },
                .mass = 1.0f,
                .shape = &box,
                .friction = 0.6f,
            };
            if (!scene_body_add(scene, &desc)) {
                return false;
            }
        }
    }
    return scene->body_count == SL_BENCH_PYRAMID_BODY_COUNT;
}

static bool rain_build(sl_bench_scene *scene, bool sleep_enabled)
{
    if (!scene_world_init(
            scene, "rain",
            (sl_world_config){ .sleep_enabled = sleep_enabled,
                               .body_capacity = SL_BENCH_RAIN_BODY_COUNT,
                               .contact_capacity =
                                   SL_BENCH_RAIN_CONTACT_CAPACITY,
                               .gravity = k_gravity,
                               .substep_count = SL_BENCH_SUBSTEP_COUNT },
            SL_BENCH_RAIN_SEED)) {
        return false;
    }

    sl_shape floor_shape = sl_shape_none();
    sl_shape wall_shape = sl_shape_none();
    sl_shape circle = sl_shape_none();
    if (!sl_shape_make_box(8.4f, 0.2f, &floor_shape) ||
        !sl_shape_make_box(0.2f, 12.0f, &wall_shape) ||
        !sl_shape_make_circle(0.15f, &circle)) {
        return false;
    }
    const sl_body_desc floor_desc = {
        .position = { 0.0f, -0.2f },
        .type = SL_BODY_STATIC,
        .shape = &floor_shape,
        .friction = 0.4f,
        .restitution = 0.1f,
    };
    const sl_body_desc wall_left_desc = {
        .position = { -8.2f, 12.0f },
        .type = SL_BODY_STATIC,
        .shape = &wall_shape,
        .friction = 0.4f,
        .restitution = 0.1f,
    };
    const sl_body_desc wall_right_desc = {
        .position = { 8.2f, 12.0f },
        .type = SL_BODY_STATIC,
        .shape = &wall_shape,
        .friction = 0.4f,
        .restitution = 0.1f,
    };
    if (!scene_body_add(scene, &floor_desc) ||
        !scene_body_add(scene, &wall_left_desc) ||
        !scene_body_add(scene, &wall_right_desc)) {
        return false;
    }

    uint32_t rng = SL_BENCH_RAIN_SEED;
    for (uint32_t i = 0u; i < SL_BENCH_RAIN_CIRCLE_COUNT; ++i) {
        const uint32_t column = i % 40u;
        const uint32_t row = i / 40u;
        const sl_body_desc desc = {
            .position = {
                -7.41f + (float)column * 0.38f +
                    rng_range(&rng, -0.025f, 0.025f),
                0.8f + (float)row * 0.38f +
                    rng_range(&rng, -0.025f, 0.025f),
            },
            .velocity = {
                rng_range(&rng, -0.1f, 0.1f),
                rng_range(&rng, -0.05f, 0.05f),
            },
            .mass = 0.1f,
            .shape = &circle,
            .friction = 0.4f,
            .restitution = 0.1f,
        };
        if (!scene_body_add(scene, &desc)) {
            return false;
        }
    }
    return scene->body_count == SL_BENCH_RAIN_BODY_COUNT;
}

static bool fixture_body(sl_bench_scene *scene, sl_shape *shape, float x,
                         float y, float mass)
{
    const sl_body_desc desc = { .position = { x, y },
                                .mass = mass,
                                .type = mass == 0.0f ? SL_BODY_STATIC
                                                     : SL_BODY_DYNAMIC,
                                .shape = shape,
                                .friction = 0.6f };
    return scene_body_add(scene, &desc);
}
static bool piles_build(sl_bench_scene *scene, bool sleep_enabled)
{
    if (!scene_world_init(
            scene, "piles",
            (sl_world_config){ .sleep_enabled = sleep_enabled,
                               .body_capacity = 161u,
                               .contact_capacity = 2048u,
                               .joint_capacity = 0u,
                               .gravity = { 0.0f, -9.81f },
                               .substep_count = SL_BENCH_SUBSTEP_COUNT },
            UINT32_C(0x59B3AC01))) {
        return false;
    }
    sl_shape ground = sl_shape_none(), box = sl_shape_none();
    if (!sl_shape_make_box(26.0f, 0.5f, &ground) ||
        !sl_shape_make_box(0.5f, 0.5f, &box) ||
        !fixture_body(scene, &ground, 0.0f, -0.5f, 0.0f)) {
        return false;
    }
    for (uint32_t pile = 0u; pile < 16u; ++pile) {
        for (uint32_t row = 0u; row < 10u; ++row) {
            if (!fixture_body(scene, &box, 3.0f * ((float)pile - 7.5f),
                              0.5f + (float)row, 1.0f)) {
                return false;
            }
        }
    }
    return true;
}
static bool chains_build(sl_bench_scene *scene, bool sleep_enabled)
{
    if (!scene_world_init(
            scene, "chains",
            (sl_world_config){ .sleep_enabled = sleep_enabled,
                               .body_capacity = 104u,
                               .contact_capacity = 1024u,
                               .joint_capacity = 96u,
                               .gravity = { 0.0f, -9.81f },
                               .substep_count = SL_BENCH_SUBSTEP_COUNT },
            UINT32_C(0x59B3AC01))) {
        return false;
    }
    sl_shape box = sl_shape_none();
    if (!sl_shape_make_box(0.25f, 0.4f, &box)) {
        return false;
    }
    for (uint32_t chain = 0u; chain < 8u; ++chain) {
        sl_body_handle previous = sl_body_handle_null();
        for (uint32_t row = 0u; row < 13u; ++row) {
            if (!fixture_body(scene, &box, 4.0f * (float)chain,
                              14.0f - (float)row, row == 0u ? 0.0f : 1.0f)) {
                return false;
            }
            const sl_body_handle body = scene->bodies[scene->body_count - 1u];
            if (row > 0u) {
                const sl_joint_desc joint = { .body_a = previous,
                                              .body_b = body,
                                              .kind = (row & 1u)
                                                          ? SL_JOINT_DISTANCE
                                                          : SL_JOINT_REVOLUTE,
                                              .local_anchor_a = { 0.0f, -0.4f },
                                              .local_anchor_b = { 0.0f, 0.6f },
                                              .distance = { .length = 0.005f },
                                              .collide_connected = false };
                if (sl_joint_handle_is_null(
                        sl_world_joint_create(scene->world, &joint))) {
                    return false;
                }
            }
            previous = body;
        }
    }
    return true;
}
static bool churn_build(sl_bench_scene *scene, bool sleep_enabled)
{
    if (!scene_world_init(
            scene, "churn",
            (sl_world_config){ .sleep_enabled = sleep_enabled,
                               .body_capacity = 128u,
                               .contact_capacity = 2048u,
                               .joint_capacity = 0u,
                               .gravity = { 0.0f, 0.0f },
                               .substep_count = SL_BENCH_SUBSTEP_COUNT },
            UINT32_C(0x59B3AC01))) {
        return false;
    }
    sl_shape circle = sl_shape_none();
    if (!sl_shape_make_circle(0.25f, &circle)) {
        return false;
    }
    for (uint32_t i = 0u; i < 128u; ++i) {
        if (!fixture_body(scene, &circle, 0.6f * (float)(i % 16u),
                          0.6f * (float)(i / 16u), 1.0f)) {
            return false;
        }
    }
    return true;
}
static bool table_build(sl_bench_scene *scene, bool sleep_enabled)
{
    if (!scene_world_init(
            scene, "table",
            (sl_world_config){ .sleep_enabled = sleep_enabled,
                               .body_capacity = 5u,
                               .contact_capacity = 64u,
                               .joint_capacity = 0u,
                               .gravity = { 0.0f, -9.81f },
                               .substep_count = SL_BENCH_SUBSTEP_COUNT },
            UINT32_C(0x59B3AC01))) {
        return false;
    }
    sl_shape ground = sl_shape_none(), leg = sl_shape_none(),
             top = sl_shape_none(), load = sl_shape_none();
    return sl_shape_make_box(8.0f, 0.5f, &ground) &&
           sl_shape_make_box(0.4f, 1.0f, &leg) &&
           sl_shape_make_box(3.0f, 0.2f, &top) &&
           sl_shape_make_box(0.5f, 0.5f, &load) &&
           fixture_body(scene, &ground, 0.0f, -0.5f, 0.0f) &&
           fixture_body(scene, &leg, -2.0f, 1.0f, 1.0f) &&
           fixture_body(scene, &leg, 2.0f, 1.0f, 1.0f) &&
           fixture_body(scene, &top, 0.0f, 2.2f, 2.0f) &&
           fixture_body(scene, &load, 0.0f, 2.9f, 5.0f);
}
static bool inverted_build(sl_bench_scene *scene, bool sleep_enabled)
{
    if (!scene_world_init(
            scene, "inverted",
            (sl_world_config){ .sleep_enabled = sleep_enabled,
                               .body_capacity = 7u,
                               .contact_capacity = 128u,
                               .joint_capacity = 0u,
                               .gravity = { 0.0f, -9.81f },
                               .substep_count = SL_BENCH_SUBSTEP_COUNT },
            UINT32_C(0x59B3AC01))) {
        return false;
    }
    sl_shape ground = sl_shape_none(), box = sl_shape_none();
    if (!sl_shape_make_box(8.0f, 0.5f, &ground) ||
        !sl_shape_make_box(0.5f, 0.5f, &box) ||
        !fixture_body(scene, &ground, 0.0f, -0.5f, 0.0f)) {
        return false;
    }
    float mass = 0.25f;
    for (uint32_t row = 0u; row < 6u; ++row) {
        if (!fixture_body(scene, &box, 0.0f, 0.5f + (float)row, mass)) {
            return false;
        }
        mass *= 2.0f;
    }
    return true;
}
typedef struct bench_fixture {
    const char *name;
    bool (*build)(sl_bench_scene *, bool);
} bench_fixture;
static const bench_fixture k_fixtures[] = {
    { "pyramid", pyramid_build },  { "rain", rain_build },
    { "piles", piles_build },      { "chains", chains_build },
    { "churn", churn_build },      { "table", table_build },
    { "inverted", inverted_build }
};
_Static_assert(sizeof(k_fixtures) / sizeof(k_fixtures[0]) ==
                   SL_BENCH_FIXTURE_COUNT,
               "Fixture inventory must match the public bound");
const char *sl_bench_fixture_name(uint32_t fixture)
{
    return fixture < SL_BENCH_FIXTURE_COUNT ? k_fixtures[fixture].name : NULL;
}
bool sl_bench_scene_build(sl_bench_scene *scene, sl_world *world,
                          uint32_t fixture, bool sleep_enabled)
{
    if (scene == NULL || world == NULL || world->state != NULL ||
        fixture >= SL_BENCH_FIXTURE_COUNT) {
        return false;
    }
    memset(scene, 0, sizeof(*scene));
    scene->world = world;
    if (!k_fixtures[fixture].build(scene, sleep_enabled)) {
        sl_world_destroy(world);
        memset(scene, 0, sizeof(*scene));
        return false;
    }
    return true;
}
bool sl_bench_scene_mutate(sl_bench_scene *scene, uint32_t step)
{
    if (scene == NULL || scene->world == NULL || scene->world->state == NULL ||
        step >= 2u * SL_BENCH_STEP_COUNT_MAX) {
        return false;
    }
    if (strcmp(scene->name, "churn") != 0) {
        return true;
    }
    sl_shape circle = sl_shape_none();
    if (!sl_shape_make_circle(0.25f, &circle)) {
        return false;
    }
    for (uint32_t j = 0u; j < 4u; ++j) {
        const uint32_t slot = (step * 4u + j) % 128u;
        sl_world_body_destroy(scene->world, scene->bodies[slot]);
        const sl_body_desc desc = { .mass = 1.0f,
                                    .shape = &circle,
                                    .friction = 0.6f,
                                    .position = { 0.6f * (float)(slot % 16u),
                                                  0.6f *
                                                      (float)(slot / 16u) } };
        const sl_body_handle body = sl_world_body_create(scene->world, &desc);
        if (sl_body_handle_is_null(body) || body.index != slot) {
            return false;
        }
        scene->bodies[slot] =
            body; /* immediate LIFO reuse preserves slot order */
        if (!sl_world_body_set_position(
                scene->world, body,
                (sl_vec2){ desc.position.x + ((step & 1u) ? 0.15f : -0.15f),
                           desc.position.y })) {
            return false;
        }
    }
    return true;
}

void sl_bench_reference_capture(sl_bench_scene *scene)
{
    for (uint32_t i = 0u; i < scene->body_count; ++i) {
        scene->reference[i] =
            sl_world_body_get_transform(scene->world, scene->bodies[i]);
    }
}
bool sl_bench_quality_sample(sl_bench_scene *scene, sl_bench_quality *q,
                             bool window)
{
    const sl_world *world = scene->world;
    double support_force = 0.0;
    for (uint32_t i = 0u; i < sl_world_contact_count(world); ++i) {
        const sl_contact *c = sl_world_contact_at(world, i);
        const sl_shape *a = sl_world_body_get_shape(world, c->body_a);
        const sl_shape *b = sl_world_body_get_shape(world, c->body_b);
        const float depth = sl_bench_penetration(
            a, sl_world_body_get_transform(world, c->body_a), b,
            sl_world_body_get_transform(world, c->body_b));
        q->penetration_max = sl_max(q->penetration_max, depth);
        for (uint32_t j = 0u; j < c->manifold.point_count; ++j) {
            const sl_manifold_point *point = &c->manifold.points[j];
            q->cached_penetration_max =
                sl_max(q->cached_penetration_max, -point->separation);
            if (sl_world_body_get_type(world, c->body_a) == SL_BODY_STATIC) {
                support_force +=
                    (double)(point->normal_impulse * c->manifold.normal.y) *
                    (double)SL_BENCH_SUBSTEP_COUNT / (double)SL_BENCH_TIMESTEP;
            } else if (sl_world_body_get_type(world, c->body_b) ==
                       SL_BODY_STATIC) {
                support_force -=
                    (double)(point->normal_impulse * c->manifold.normal.y) *
                    (double)SL_BENCH_SUBSTEP_COUNT / (double)SL_BENCH_TIMESTEP;
            }
        }
    }
    for (uint32_t i = 0u; i < sl_world_joint_count(world); ++i) {
        const sl_joint_handle handle = sl_world_joint_at(world, i);
        const sl_joint_desc d = sl_world_joint_get_desc(world, handle);
        const sl_vec2 a = sl_transform_apply(
            sl_world_body_get_transform(world, d.body_a), d.local_anchor_a);
        const sl_vec2 b = sl_transform_apply(
            sl_world_body_get_transform(world, d.body_b), d.local_anchor_b);
        float error = sl_vec2_length(sl_vec2_sub(a, b));
        if (d.kind == SL_JOINT_DISTANCE) {
            error = sl_abs(error - d.distance.length);
        }
        q->joint_error_max = sl_max(q->joint_error_max, error);
    }
    q->supported_weight = 0.0;
    for (uint32_t i = 0u; i < scene->body_count; ++i) {
        const sl_body_handle body = scene->bodies[i];
        const sl_transform t = sl_world_body_get_transform(world, body);
        const sl_vec2 v = sl_world_body_get_velocity(world, body);
        const float w = sl_world_body_get_angular_velocity(world, body);
        if (!sl_vec2_is_finite(t.position) || !sl_vec2_is_finite(v) ||
            !isfinite(w) || !isfinite(t.rotation.c) ||
            !isfinite(t.rotation.s)) {
            return false;
        }
        q->supported_weight += (double)sl_world_body_get_mass(world, body) *
                               -(double)scene->config.gravity.y;
        if (window) {
            q->translation_drift_max =
                sl_max(q->translation_drift_max,
                       sl_vec2_length(sl_vec2_sub(
                           t.position, scene->reference[i].position)));
            q->rotation_drift_max =
                sl_max(q->rotation_drift_max,
                       sl_abs(sl_angle_wrap(
                           sl_rotation_angle(t.rotation) -
                           sl_rotation_angle(scene->reference[i].rotation))));
            q->linear_speed_max =
                sl_max(q->linear_speed_max, sl_vec2_length(v));
            q->angular_speed_max = sl_max(q->angular_speed_max, sl_abs(w));
        }
    }
    if (window) {
        q->support_force_sum += support_force;
        q->window_steps += 1u;
    }
    return isfinite(q->support_force_sum) && isfinite(q->supported_weight) &&
           isfinite(q->penetration_max) && isfinite(q->joint_error_max);
}
