#include "silk_test.h"
#include "suites.h"
#include "world_internal.h"

#include <float.h>
#include <stdbool.h>
#include <stdint.h>

#include <silk/step.h>

static sl_shape circle_make(float radius)
{
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(radius, &shape));
    return shape;
}

static sl_body_handle body_make(sl_world *world, sl_body_type type,
                                sl_vec2 position, const sl_shape *shape,
                                float friction, float restitution)
{
    const sl_body_desc desc = {
        .position = position,
        .mass = (type == SL_BODY_DYNAMIC) ? 1.0f : 0.0f,
        .type = type,
        .shape = shape,
        .friction = friction,
        .restitution = restitution,
    };
    const sl_body_handle body = sl_world_body_create(world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(body));
    return body;
}

static void step_once(sl_world *world)
{
    sl_world_step(world, 1.0f / 60.0f);
}

static bool contact_has_body(const sl_contact *contact, sl_body_handle body)
{
    return (contact->body_a.index == body.index &&
            contact->body_a.generation == body.generation) ||
           (contact->body_b.index == body.index &&
            contact->body_b.generation == body.generation);
}

static void pair_lifecycle_and_speculative_touching(void)
{
    const sl_world_config config = { .body_capacity = 3u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape circle = circle_make(1.0f);
    const sl_body_handle ground = body_make(
        &world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), &circle, 0.0f, 0.0f);
    const sl_body_handle body =
        body_make(&world, SL_BODY_DYNAMIC, sl_vec2_make(2.015f, 0.0f), &circle,
                  0.0f, 0.0f);

    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    const sl_contact *contact = sl_world_contact_at(&world, 0u);
    SL_EXPECT(contact->touching);
    SL_EXPECT_INT_EQ(contact->manifold.point_count, 1u);
    SL_EXPECT(contact->manifold.points[0].separation > 0.0f);
    SL_EXPECT(contact->body_a.index < contact->body_b.index);
    SL_EXPECT(contact_has_body(contact, ground));
    SL_EXPECT(contact_has_body(contact, body));

    SL_EXPECT(
        sl_world_body_set_position(&world, body, sl_vec2_make(3.5f, 0.0f)));
    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 0u);
    SL_EXPECT_INT_EQ(sl_world_contact_drop_count(&world), 0u);
    sl_world_destroy(&world);
}

static uint32_t pair_count_for_types(sl_body_type type_a, sl_body_type type_b)
{
    const sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape circle = circle_make(1.0f);
    (void)body_make(&world, type_a, sl_vec2_make(0.0f, 0.0f), &circle, 0.0f,
                    0.0f);
    (void)body_make(&world, type_b, sl_vec2_make(1.0f, 0.0f), &circle, 0.0f,
                    0.0f);
    step_once(&world);
    const uint32_t count = sl_world_contact_count(&world);
    sl_world_destroy(&world);
    return count;
}

static void type_and_shape_filters(void)
{
    SL_EXPECT_INT_EQ(pair_count_for_types(SL_BODY_STATIC, SL_BODY_STATIC), 0u);
    SL_EXPECT_INT_EQ(pair_count_for_types(SL_BODY_KINEMATIC, SL_BODY_STATIC),
                     0u);
    SL_EXPECT_INT_EQ(pair_count_for_types(SL_BODY_KINEMATIC, SL_BODY_DYNAMIC),
                     1u);
    SL_EXPECT_INT_EQ(pair_count_for_types(SL_BODY_DYNAMIC, SL_BODY_DYNAMIC),
                     1u);

    const sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape circle = circle_make(1.0f);
    const sl_body_handle point = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(0.0f, 0.0f), NULL, 0.0f, 0.0f);
    (void)body_make(&world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), &circle,
                    0.0f, 0.0f);
    sl_aabb untouched =
        sl_aabb_make(sl_vec2_make(7.0f, 8.0f), sl_vec2_make(9.0f, 10.0f));
    SL_EXPECT(!sl_world_body_get_proxy_aabb(&world, point, &untouched));
    SL_EXPECT(untouched.lower.x == 7.0f && untouched.upper.y == 10.0f);
    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 0u);
    sl_world_destroy(&world);
}

static void proxy_fat_aabb_lifecycle_and_reset(void)
{
    const sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape circle = circle_make(1.0f);
    const sl_body_handle body = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(0.0f, 0.0f), &circle, 0.0f, 0.0f);
    sl_aabb first = { 0 };
    SL_EXPECT(sl_world_body_get_proxy_aabb(&world, body, &first));
    SL_EXPECT_NEAR(first.lower.x,
                   -1.0f - SL_SPECULATIVE_DISTANCE - SL_AABB_MARGIN, 1e-6f);
    SL_EXPECT_NEAR(first.upper.y,
                   1.0f + SL_SPECULATIVE_DISTANCE + SL_AABB_MARGIN, 1e-6f);

    SL_EXPECT(
        sl_world_body_set_position(&world, body, sl_vec2_make(0.05f, 0.0f)));
    sl_aabb inside = { 0 };
    SL_EXPECT(sl_world_body_get_proxy_aabb(&world, body, &inside));
    SL_EXPECT(inside.lower.x == first.lower.x &&
              inside.upper.x == first.upper.x);

    SL_EXPECT(
        sl_world_body_set_position(&world, body, sl_vec2_make(0.25f, 0.0f)));
    sl_aabb moved = { 0 };
    SL_EXPECT(sl_world_body_get_proxy_aabb(&world, body, &moved));
    SL_EXPECT(moved.lower.x > first.lower.x);

    SL_EXPECT(sl_world_body_set_shape(&world, body, NULL));
    SL_EXPECT(!sl_world_body_get_proxy_aabb(&world, body, &moved));
    sl_world_reset(&world);
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 0u);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 0u);
    SL_EXPECT_INT_EQ(world.state->moved_count, 0u);
    const sl_body_handle after = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(0.0f, 0.0f), &circle, 0.0f, 0.0f);
    SL_EXPECT(sl_world_body_get_proxy_aabb(&world, after, &moved));
    SL_EXPECT_INT_EQ(world.state->moved_count, 1u);
    sl_world_body_destroy(&world, after);
    SL_EXPECT_INT_EQ(world.state->moved_count, 0u);
    const sl_body_handle reused = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(4.0f, 0.0f), &circle, 0.0f, 0.0f);
    SL_EXPECT_INT_EQ(reused.index, after.index);
    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 0u);
    sl_world_destroy(&world);
}

static void stale_contact_reaps_before_slot_reuse(void)
{
    const sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape circle = circle_make(1.0f);
    const sl_body_handle ground = body_make(
        &world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), &circle, 0.0f, 0.0f);
    const sl_body_handle old_body = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(1.0f, 0.0f), &circle, 0.0f, 0.0f);
    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);

    sl_world_body_destroy(&world, old_body);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    const sl_body_handle reused = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(5.0f, 0.0f), &circle, 0.0f, 0.0f);
    SL_EXPECT_INT_EQ(reused.index, old_body.index);
    SL_EXPECT(reused.generation != old_body.generation);
    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 0u);

    SL_EXPECT(
        sl_world_body_set_position(&world, reused, sl_vec2_make(1.0f, 0.0f)));
    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    const sl_contact *contact = sl_world_contact_at(&world, 0u);
    SL_EXPECT(contact_has_body(contact, ground));
    SL_EXPECT(contact_has_body(contact, reused));
    sl_world_destroy(&world);
}

static void materials_and_feature_impulses_refresh(void)
{
    const sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape circle = circle_make(1.0f);
    const sl_body_handle a = body_make(
        &world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), &circle, 4.0f, 0.2f);
    const sl_body_handle b = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(1.5f, 0.0f), &circle, 9.0f, 0.7f);
    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    SL_EXPECT_NEAR(world.state->contacts[0].friction, 6.0f, 1e-6f);
    SL_EXPECT_NEAR(world.state->contacts[0].restitution, 0.7f, 1e-6f);
    world.state->contacts[0].manifold.points[0].normal_impulse = 3.0f;
    world.state->contacts[0].manifold.points[0].tangent_impulse = -1.0f;

    SL_EXPECT(sl_world_body_set_friction(&world, a, 0.25f));
    SL_EXPECT(sl_world_body_set_friction(&world, b, 16.0f));
    SL_EXPECT(sl_world_body_set_restitution(&world, a, 0.9f));
    SL_EXPECT(
        sl_world_body_set_position(&world, b, sl_vec2_make(1.501f, 0.0f)));
    step_once(&world);
    const sl_contact *contact = sl_world_contact_at(&world, 0u);
    SL_EXPECT_NEAR(contact->friction, 2.0f, 1e-6f);
    SL_EXPECT_NEAR(contact->restitution, 0.9f, 1e-6f);
    SL_EXPECT(contact->manifold.points[0].persisted);
    SL_EXPECT(sl_is_finite(contact->manifold.points[0].normal_impulse));
    SL_EXPECT(sl_is_finite(contact->manifold.points[0].tangent_impulse));

    SL_EXPECT(sl_world_body_set_friction(&world, a, FLT_MAX));
    SL_EXPECT(sl_world_body_set_friction(&world, b, FLT_MAX));
    step_once(&world);
    SL_EXPECT(sl_is_finite(sl_world_contact_at(&world, 0u)->friction));
    sl_world_destroy(&world);
}

static void saturation_retries_without_new_motion(void)
{
    const sl_world_config config = {
        .body_capacity = 3u,
        .contact_capacity = 1u,
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape circle = circle_make(1.0f);
    const sl_body_handle ground = body_make(
        &world, SL_BODY_STATIC, sl_vec2_make(0.0f, 0.0f), &circle, 0.0f, 0.0f);
    const sl_body_handle body_a = body_make(
        &world, SL_BODY_DYNAMIC, sl_vec2_make(0.5f, 0.0f), &circle, 0.0f, 0.0f);
    const sl_body_handle body_b =
        body_make(&world, SL_BODY_DYNAMIC, sl_vec2_make(-0.5f, 0.0f), &circle,
                  0.0f, 0.0f);
    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    SL_EXPECT(sl_world_contact_drop_count(&world) > 0u);
    SL_EXPECT(world.state->moved_count > 0u);

    const sl_contact *first = sl_world_contact_at(&world, 0u);
    const sl_body_handle victim =
        contact_has_body(first, body_a) ? body_a : body_b;
    const sl_body_handle survivor =
        (victim.index == body_a.index) ? body_b : body_a;
    sl_world_body_destroy(&world, victim);
    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
    const sl_contact *retried = sl_world_contact_at(&world, 0u);
    SL_EXPECT(contact_has_body(retried, ground));
    SL_EXPECT(contact_has_body(retried, survivor));
    SL_EXPECT_INT_EQ(sl_world_contact_drop_count(&world), 0u);
    sl_world_destroy(&world);
}

static void saturation_preserves_retry_order_and_pair_uniqueness(void)
{
    enum { PAIR_COUNT = 4 };
    const sl_world_config config = {
        .body_capacity = 2u * PAIR_COUNT,
        .contact_capacity = 1u,
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape circle = circle_make(1.0f);
    sl_body_handle bodies[2u * PAIR_COUNT];
    for (uint32_t i = 0u; i < PAIR_COUNT; ++i) {
        /* Fat proxies overlap, but the shapes are beyond speculative range.
         * No solver motion can requeue a body and mask a lost retry. */
        bodies[2u * i] =
            body_make(&world, SL_BODY_DYNAMIC,
                      sl_vec2_make(5.0f * (float)i, 0.0f), &circle, 0.0f, 0.0f);
        bodies[2u * i + 1u] = body_make(
            &world, SL_BODY_DYNAMIC, sl_vec2_make(5.0f * (float)i + 2.1f, 0.0f),
            &circle, 0.0f, 0.0f);
    }

    for (uint32_t i = 0u; i < PAIR_COUNT; ++i) {
        for (uint32_t repeat = 0u; repeat < 2u; ++repeat) {
            step_once(&world);
            SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 1u);
            SL_EXPECT_INT_EQ(sl_world_contact_drop_count(&world),
                             PAIR_COUNT - 1u - i);
            const sl_contact *contact = sl_world_contact_at(&world, 0u);
            SL_EXPECT(contact_has_body(contact, bodies[2u * i]));
            SL_EXPECT(contact_has_body(contact, bodies[2u * i + 1u]));
            SL_EXPECT(!contact->touching);
        }
        /* Body removal also swap-removes packed rows while later pairs
         * still await capacity; retries must retain stable slot ownership. */
        sl_world_body_destroy(&world, bodies[2u * i]);
        sl_world_body_destroy(&world, bodies[2u * i + 1u]);
    }
    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 0u);
    SL_EXPECT_INT_EQ(sl_world_contact_drop_count(&world), 0u);
    SL_EXPECT_INT_EQ(world.state->moved_count, 0u);
    sl_world_destroy(&world);
}

static void capacity_config_and_deterministic_snapshot(void)
{
    const sl_world_config defaults = { .body_capacity = 8u };
    const sl_world_config explicit = {
        .body_capacity = 8u,
        .contact_capacity = 3u,
    };
    const sl_world_config invalid = {
        .body_capacity = 8u,
        .contact_capacity = SL_CONTACT_COUNT_MAX + 1u,
    };
    sl_world left = { 0 };
    sl_world right = { 0 };
    SL_EXPECT(sl_world_init(&left, &defaults));
    SL_EXPECT_INT_EQ(sl_world_contact_capacity(&left), 32u);
    sl_world_destroy(&left);
    SL_EXPECT(sl_world_init(&left, &explicit));
    SL_EXPECT(sl_world_init(&right, &explicit));
    SL_EXPECT_INT_EQ(sl_world_contact_capacity(&left), 3u);
    SL_EXPECT_INT_EQ(sl_world_memory_bytes(&invalid), 0u);
    sl_world rejected = { 0 };
    SL_EXPECT(!sl_world_init(&rejected, &invalid));
    SL_EXPECT(rejected.state == NULL);

    const sl_shape circle = circle_make(0.75f);
    for (uint32_t i = 0u; i < 3u; ++i) {
        const sl_body_type type = (i == 0u) ? SL_BODY_STATIC : SL_BODY_DYNAMIC;
        const sl_vec2 position = sl_vec2_make((float)i * 0.5f, 0.0f);
        (void)body_make(&left, type, position, &circle, (float)i, 0.1f);
        (void)body_make(&right, type, position, &circle, (float)i, 0.1f);
    }
    step_once(&left);
    step_once(&right);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&left),
                     sl_world_contact_count(&right));
    SL_EXPECT_INT_EQ(left.state->contact_drop_count,
                     right.state->contact_drop_count);
    for (uint32_t i = 0u; i < sl_world_contact_count(&left); ++i) {
        const sl_contact *a = sl_world_contact_at(&left, i);
        const sl_contact *b = sl_world_contact_at(&right, i);
        SL_EXPECT_INT_EQ(a->body_a.index, b->body_a.index);
        SL_EXPECT_INT_EQ(a->body_b.index, b->body_b.index);
        SL_EXPECT_INT_EQ(a->manifold.point_count, b->manifold.point_count);
        SL_EXPECT(a->touching == b->touching);
        SL_EXPECT(a->friction == b->friction);
        for (uint32_t point = 0u; point < a->manifold.point_count; ++point) {
            SL_EXPECT_INT_EQ(a->manifold.points[point].id,
                             b->manifold.points[point].id);
            SL_EXPECT(a->manifold.points[point].separation ==
                      b->manifold.points[point].separation);
        }
    }
    sl_world_destroy(&right);
    sl_world_destroy(&left);
}

static void pair_hash_churn_preserves_all_live_pairs(void)
{
    enum {
        PAIR_COUNT = 6,
        CYCLE_COUNT = 32,
    };
    const sl_world_config config = {
        .body_capacity = 2u * PAIR_COUNT,
        .contact_capacity = PAIR_COUNT,
    };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    const sl_shape circle = circle_make(0.5f);
    sl_body_handle dynamic[PAIR_COUNT];
    for (uint32_t i = 0u; i < PAIR_COUNT; ++i) {
        (void)body_make(&world, SL_BODY_STATIC,
                        sl_vec2_make(4.0f * (float)i, 0.0f), &circle, 0.0f,
                        0.0f);
    }
    for (uint32_t i = 0u; i < PAIR_COUNT; ++i) {
        dynamic[i] =
            body_make(&world, SL_BODY_DYNAMIC,
                      sl_vec2_make(4.0f * (float)i, 0.0f), &circle, 0.0f, 0.0f);
    }
    step_once(&world);
    SL_EXPECT_INT_EQ(sl_world_contact_count(&world), PAIR_COUNT);

    for (uint32_t cycle = 0u; cycle < CYCLE_COUNT; ++cycle) {
        for (uint32_t i = 0u; i < PAIR_COUNT; ++i) {
            sl_world_body_destroy(&world, dynamic[i]);
        }
        step_once(&world);
        SL_EXPECT_INT_EQ(sl_world_contact_count(&world), 0u);

        for (uint32_t i = 0u; i < PAIR_COUNT; ++i) {
            const uint32_t position_index = PAIR_COUNT - 1u - i;
            dynamic[i] =
                body_make(&world, SL_BODY_DYNAMIC,
                          sl_vec2_make(4.0f * (float)position_index, 0.0f),
                          &circle, 0.0f, 0.0f);
        }
        step_once(&world);
        SL_EXPECT_INT_EQ(sl_world_contact_count(&world), PAIR_COUNT);
    }
    sl_world_destroy(&world);
}

static const sl_test_case k_cases[] = {
    { "pair lifecycle and speculative",
      pair_lifecycle_and_speculative_touching },
    { "type and shape filters", type_and_shape_filters },
    { "proxy lifecycle and reset", proxy_fat_aabb_lifecycle_and_reset },
    { "stale contact and slot reuse", stale_contact_reaps_before_slot_reuse },
    { "materials and persistence", materials_and_feature_impulses_refresh },
    { "saturation retry", saturation_retries_without_new_motion },
    { "saturation retry order and uniqueness",
      saturation_preserves_retry_order_and_pair_uniqueness },
    { "capacity and determinism", capacity_config_and_deterministic_snapshot },
    { "pair hash churn", pair_hash_churn_preserves_all_live_pairs },
};

int sl_contact_suite(void)
{
    return sl_run_suite("contact", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
