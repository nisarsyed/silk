#include "silk_test.h"
#include "suites.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <silk/world.h>

/* Exact equality is the property under test wherever state is stored and
 * retrieved verbatim (packing, setters, traversal); anything through a
 * division (inverse mass) compares via SL_EXPECT_NEAR. */
static const float k_eps = 1e-6f;

static void test_init_destroy_roundtrip(void)
{
    sl_world_config config = { .body_capacity = 4u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));
    SL_EXPECT_INT_EQ(world.body_capacity, 4);
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 0);
    SL_EXPECT(world.memory != NULL);

    sl_world_destroy(&world);
    SL_EXPECT(world.memory == NULL);
    SL_EXPECT_INT_EQ(world.body_capacity, 0);

    /* Destroy is idempotent on an already-zeroed world. */
    sl_world_destroy(&world);
    SL_EXPECT(world.memory == NULL);
}

static void test_init_rejects_bad_capacity(void)
{
    sl_world world = { 0 };

    sl_world_config zero = { .body_capacity = 0u };
    SL_EXPECT(!sl_world_init(&world, &zero));
    SL_EXPECT(world.memory == NULL && world.body_capacity == 0u);

    sl_world_config overflow = { .body_capacity = SL_BODY_COUNT_MAX + 1u };
    SL_EXPECT(!sl_world_init(&world, &overflow));
    SL_EXPECT(world.memory == NULL && world.body_capacity == 0u);

    /* Both bounds themselves are accepted. */
    sl_world_config one = { .body_capacity = 1u };
    SL_EXPECT(sl_world_init(&world, &one));
    sl_world_destroy(&world);

    sl_world_config full = { .body_capacity = SL_BODY_COUNT_MAX };
    SL_EXPECT(sl_world_init(&world, &full));
    SL_EXPECT_INT_EQ(world.body_capacity, SL_BODY_COUNT_MAX);
    sl_world_destroy(&world);
}

static void test_create_roundtrips_state(void)
{
    sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(1.5f, -2.5f),
                          .velocity = sl_vec2_make(10.0f, -20.0f),
                          .mass = 4.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(!sl_body_handle_is_null(h));
    SL_EXPECT(sl_world_body_is_valid(&world, h));
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 1);

    sl_vec2 position = sl_world_body_get_position(&world, h);
    SL_EXPECT(position.x == 1.5f && position.y == -2.5f);

    sl_vec2 velocity = sl_world_body_get_velocity(&world, h);
    SL_EXPECT(velocity.x == 10.0f && velocity.y == -20.0f);

    SL_EXPECT(sl_world_body_get_mass(&world, h) == 4.0f);
    SL_EXPECT_NEAR(sl_world_body_get_inv_mass(&world, h), 0.25f, k_eps);

    /* Appended descriptor fields zero-fill to: dynamic, upright, at
     * rest, point particle. */
    SL_EXPECT(sl_world_body_get_type(&world, h) == SL_BODY_DYNAMIC);
    SL_EXPECT(sl_world_body_get_angle(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_angular_velocity(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_inertia(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_inv_inertia(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_torque(&world, h) == 0.0f);
    SL_EXPECT(sl_world_body_get_shape(&world, h)->kind == SL_SHAPE_NONE);
    const sl_transform tf = sl_world_body_get_transform(&world, h);
    SL_EXPECT(tf.position.x == 1.5f && tf.position.y == -2.5f);
    SL_EXPECT(tf.rotation.c == 1.0f && tf.rotation.s == 0.0f);

    sl_world_destroy(&world);
}

/* The world's arena is bounded by contract: at the population cap it
 * stays inside a 10 MiB budget (141 bytes per body plus slice padding),
 * and out-of-range capacities report zero bytes rather than guessing. */
static void test_memory_bytes_at_capacity_max_within_budget(void)
{
    const size_t worst = sl_world_memory_bytes(SL_BODY_COUNT_MAX);
    SL_EXPECT(worst > 0u);
    SL_EXPECT(worst <= (size_t)10u << 20);

    SL_EXPECT_INT_EQ((int)sl_world_memory_bytes(0u), 0);
    SL_EXPECT_INT_EQ((int)sl_world_memory_bytes(SL_BODY_COUNT_MAX + 1u), 0);

    /* A tiny world allocates plausibly more than its raw payload but
     * nothing extravagant. */
    const size_t small = sl_world_memory_bytes(4u);
    SL_EXPECT(small >= 4u * 141u);
    SL_EXPECT(small <= 4u * 141u + 256u);
}

static void test_create_rejects_non_finite_state(void)
{
    sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc base = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1.0f };

    sl_body_desc nan_x = base;
    nan_x.position.x = NAN;
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &nan_x)));

    sl_body_desc neg_inf_y = base;
    neg_inf_y.position.y = -INFINITY;
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &neg_inf_y)));

    sl_body_desc inf_vel = base;
    inf_vel.velocity.x = INFINITY;
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &inf_vel)));

    sl_body_desc nan_vel = base;
    nan_vel.velocity.y = NAN;
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &nan_vel)));

    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 0);
    sl_world_destroy(&world);
}

static void test_create_rejects_non_positive_mass(void)
{
    sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    const float bad_masses[] = { 0.0f, -1.0f, NAN, INFINITY };
    for (uint32_t i = 0u; i < 4u; ++i) {
        sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                              .velocity = sl_vec2_make(0.0f, 0.0f),
                              .mass = bad_masses[i] };
        SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &desc)));
    }

    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 0);
    sl_world_destroy(&world);
}

static void test_create_rejects_inverse_overflow(void)
{
    sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    /* Positive and finite, but 1 / m overflows float: still rejected. */
    sl_body_desc tiny = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1e-40f };
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &tiny)));

    /* Small-but-sound masses are accepted. */
    sl_body_desc small = { .position = sl_vec2_make(0.0f, 0.0f),
                           .velocity = sl_vec2_make(0.0f, 0.0f),
                           .mass = 1e-35f };
    sl_body_handle h = sl_world_body_create(&world, &small);
    SL_EXPECT(!sl_body_handle_is_null(h));

    sl_world_destroy(&world);
}

static void test_fill_to_capacity(void)
{
    sl_world_config config = { .body_capacity = 3u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    for (uint32_t i = 0u; i < 3u; ++i) {
        sl_body_desc desc = { .position = sl_vec2_make((float)i, 0.0f),
                              .velocity = sl_vec2_make(0.0f, 0.0f),
                              .mass = 1.0f };
        sl_body_handle h = sl_world_body_create(&world, &desc);
        SL_EXPECT(!sl_body_handle_is_null(h));
        SL_EXPECT_INT_EQ(sl_world_body_count(&world), i + 1u);
    }

    sl_body_desc extra = { .position = sl_vec2_make(99.0f, 0.0f),
                           .velocity = sl_vec2_make(0.0f, 0.0f),
                           .mass = 1.0f };
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &extra)));
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 3);

    sl_world_destroy(&world);
}

static void test_destroy_invalidates_handle(void)
{
    sl_world_config config = { .body_capacity = 2u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT(sl_world_body_is_valid(&world, h));

    sl_world_body_destroy(&world, h);
    SL_EXPECT(!sl_world_body_is_valid(&world, h));
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 0);

    sl_world_destroy(&world);
}

static void test_destroy_tolerates_bad_handles(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1.0f };

    /* Null, malformed, and unknown-slot handles: silent no-ops. */
    sl_world_body_destroy(&world, sl_body_handle_null());
    sl_body_handle garbage = { UINT32_MAX, 7u };
    sl_world_body_destroy(&world, garbage);
    sl_body_handle out_of_range = { 42u, 1u };
    sl_world_body_destroy(&world, out_of_range);

    sl_body_handle h = sl_world_body_create(&world, &desc);
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 1);

    /* Double destroy acts once: the second call is a tolerated no-op, so
     * the recreated slot's generation is bumped by exactly one. */
    sl_world_body_destroy(&world, h);
    sl_world_body_destroy(&world, h);
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 0);

    sl_body_handle h2 = sl_world_body_create(&world, &desc);
    SL_EXPECT_INT_EQ(h2.index, h.index);
    SL_EXPECT_INT_EQ(h2.generation, h.generation + 1u);

    sl_world_destroy(&world);
}

static void test_slot_reuse_bumps_generation(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1.0f };

    sl_body_handle first = sl_world_body_create(&world, &desc);
    SL_EXPECT_INT_EQ(first.generation, 1u);

    sl_world_body_destroy(&world, first);

    sl_body_handle second = sl_world_body_create(&world, &desc);
    SL_EXPECT_INT_EQ(second.index, first.index);
    SL_EXPECT_INT_EQ(second.generation, 2u);
    SL_EXPECT(!sl_world_body_is_valid(&world, first));
    SL_EXPECT(sl_world_body_is_valid(&world, second));

    sl_world_destroy(&world);
}

static void test_arrays_stay_packed_on_middle_destroy(void)
{
    sl_world_config config = { .body_capacity = 4u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_handle handles[4];
    for (uint32_t i = 0u; i < 4u; ++i) {
        sl_body_desc desc = { .position = sl_vec2_make((float)(i + 1u), 0.0f),
                              .velocity = sl_vec2_make(0.0f, 0.0f),
                              .mass = 1.0f };
        handles[i] = sl_world_body_create(&world, &desc);
        SL_EXPECT(!sl_body_handle_is_null(handles[i]));
    }

    /* Removing the second body swaps the last into its hole: packed order
     * becomes bodies 0, 3, 2 with their payloads intact. */
    sl_world_body_destroy(&world, handles[1]);
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 3);
    SL_EXPECT(!sl_world_body_is_valid(&world, handles[1]));

    const float expected_x[3] = { 1.0f, 4.0f, 3.0f };
    sl_body_handle it = sl_world_body_first(&world);
    uint32_t visited = 0u;
    while (!sl_body_handle_is_null(it)) {
        SL_EXPECT(visited < 3u);
        sl_vec2 position = sl_world_body_get_position(&world, it);
        SL_EXPECT(position.x == expected_x[visited]);
        visited++;
        it = sl_world_body_next(&world, it);
    }
    SL_EXPECT_INT_EQ(visited, 3u);

    sl_world_destroy(&world);
}

static void test_traversal_visits_each_body_once(void)
{
    sl_world_config config = { .body_capacity = 8u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    /* Empty worlds yield the null handle. */
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_first(&world)));

    for (uint32_t i = 0u; i < 8u; ++i) {
        sl_body_desc desc = { .position = sl_vec2_make((float)i, (float)-i),
                              .velocity = sl_vec2_make(0.0f, 0.0f),
                              .mass = 1.0f };
        SL_EXPECT(!sl_body_handle_is_null(sl_world_body_create(&world, &desc)));
    }

    bool seen[8] = { false };
    uint32_t visits = 0u;
    sl_body_handle it = sl_world_body_first(&world);
    while (!sl_body_handle_is_null(it)) {
        SL_EXPECT(it.index < 8u);
        SL_EXPECT(!seen[it.index]);
        seen[it.index] = true;
        visits++;
        it = sl_world_body_next(&world, it);
    }
    SL_EXPECT_INT_EQ(visits, 8u);
    for (uint32_t i = 0u; i < 8u; ++i) {
        SL_EXPECT(seen[i]);
    }

    sl_world_destroy(&world);
}

static void test_destroy_during_traversal_visits_each_once(void)
{
    sl_world_config config = { .body_capacity = 4u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    for (uint32_t i = 0u; i < 4u; ++i) {
        sl_body_desc desc = { .position = sl_vec2_make((float)(i + 1u), 0.0f),
                              .velocity = sl_vec2_make(0.0f, 0.0f),
                              .mass = 1.0f };
        SL_EXPECT(!sl_body_handle_is_null(sl_world_body_create(&world, &desc)));
    }

    /* Destroying every visited body must still visit all four exactly
     * once: each removal refills the cursor row, so the walk order is
     * 1, 4, 3, 2. Capturing a successor instead would skip the last
     * body — the bug the row-cursor idiom exists to prevent. Both loop
     * guards are structural: the pool shrinks every pass and visited
     * bounds the expected-table index, so neither can run out of range
     * even while an expectation has already failed. */
    const float expected[4] = { 1.0f, 4.0f, 3.0f, 2.0f };
    uint32_t visited = 0u;
    while (sl_world_body_count(&world) > 0u && visited < 4u) {
        /* This walk always destroys row 0, so it never advances. */
        const sl_body_handle body = sl_world_body_at(&world, 0u);
        const sl_vec2 position = sl_world_body_get_position(&world, body);
        SL_EXPECT(position.x == expected[visited]);
        visited++;
        sl_world_body_destroy(&world, body);
    }
    SL_EXPECT_INT_EQ(visited, 4u);
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 0u);

    sl_world_destroy(&world);
}

static void test_partial_cull_during_traversal(void)
{
    sl_world_config config = { .body_capacity = 4u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    for (uint32_t i = 0u; i < 4u; ++i) {
        sl_body_desc desc = { .position = sl_vec2_make((float)(i + 1u), 0.0f),
                              .velocity = sl_vec2_make(0.0f, 0.0f),
                              .mass = 1.0f };
        SL_EXPECT(!sl_body_handle_is_null(sl_world_body_create(&world, &desc)));
    }

    /* Removing only x == 2 swaps x == 4 into its hole behind the
     * cursor; the retested row keeps it. Surviving order: 1, 4, 3. */
    for (uint32_t row = 0u; row < sl_world_body_count(&world);) {
        sl_body_handle body = sl_world_body_at(&world, row);
        if (sl_world_body_get_position(&world, body).x == 2.0f) {
            sl_world_body_destroy(&world, body);
        } else {
            row++;
        }
    }

    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 3u);
    const float expected_x[3] = { 1.0f, 4.0f, 3.0f };
    uint32_t visited = 0u;
    for (sl_body_handle it = sl_world_body_first(&world);
         !sl_body_handle_is_null(it); it = sl_world_body_next(&world, it)) {
        SL_EXPECT(visited < 3u);
        sl_vec2 position = sl_world_body_get_position(&world, it);
        SL_EXPECT(position.x == expected_x[visited]);
        visited++;
    }
    SL_EXPECT_INT_EQ(visited, 3u);

    sl_world_destroy(&world);
}

static void test_at_matches_handle_traversal(void)
{
    sl_world_config config = { .body_capacity = 8u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    for (uint32_t i = 0u; i < 6u; ++i) {
        sl_body_desc desc = { .position = sl_vec2_make((float)(i + 1u), 0.0f),
                              .velocity = sl_vec2_make(0.0f, 0.0f),
                              .mass = 1.0f };
        SL_EXPECT(!sl_body_handle_is_null(sl_world_body_create(&world, &desc)));
    }
    /* Remove one mid-pack so the two APIs must agree across a swap. */
    sl_body_handle middle = sl_world_body_at(&world, 3u);
    sl_world_body_destroy(&world, middle);

    float via_rows[8] = { 0.0f };
    uint32_t rows = 0u;
    for (uint32_t row = 0u; row < sl_world_body_count(&world); ++row) {
        sl_body_handle body = sl_world_body_at(&world, row);
        via_rows[rows++] = sl_world_body_get_position(&world, body).x;
    }

    float via_handles[8] = { 0.0f };
    uint32_t handles = 0u;
    for (sl_body_handle it = sl_world_body_first(&world);
         !sl_body_handle_is_null(it); it = sl_world_body_next(&world, it)) {
        via_handles[handles++] = sl_world_body_get_position(&world, it).x;
    }

    SL_EXPECT_INT_EQ(rows, handles);
    for (uint32_t i = 0u; i < rows; ++i) {
        SL_EXPECT(via_rows[i] == via_handles[i]);
    }

    sl_world_destroy(&world);
}

static void test_reset_invalidates_and_refills(void)
{
    sl_world_config config = { .body_capacity = 4u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 1.0f };
    sl_body_handle before[4];
    for (uint32_t i = 0u; i < 4u; ++i) {
        before[i] = sl_world_body_create(&world, &desc);
        SL_EXPECT(!sl_body_handle_is_null(before[i]));
    }

    sl_world_reset(&world);
    SL_EXPECT_INT_EQ(sl_world_body_count(&world), 0);
    for (uint32_t i = 0u; i < 4u; ++i) {
        SL_EXPECT(!sl_world_body_is_valid(&world, before[i]));
    }

    /* Fully refillable afterwards, on strictly later generations. */
    for (uint32_t i = 0u; i < 4u; ++i) {
        sl_body_handle h = sl_world_body_create(&world, &desc);
        SL_EXPECT(!sl_body_handle_is_null(h));
        SL_EXPECT_INT_EQ(h.generation, before[i].generation + 1u);
    }

    sl_world_destroy(&world);
}

static void test_set_mass_updates_inv_mass(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(0.0f, 0.0f),
                          .velocity = sl_vec2_make(0.0f, 0.0f),
                          .mass = 2.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);

    SL_EXPECT(sl_world_body_set_mass(&world, h, 5.0f));
    SL_EXPECT(sl_world_body_get_mass(&world, h) == 5.0f);
    SL_EXPECT_NEAR(sl_world_body_get_inv_mass(&world, h), 0.2f, k_eps);

    /* Rejected masses report false and leave both cached values untouched. */
    const float rejected[] = { -1.0f, 0.0f, NAN, INFINITY, 1e-40f };
    for (uint32_t i = 0u; i < 5u; ++i) {
        SL_EXPECT(!sl_world_body_set_mass(&world, h, rejected[i]));
        SL_EXPECT(sl_world_body_get_mass(&world, h) == 5.0f);
        SL_EXPECT_NEAR(sl_world_body_get_inv_mass(&world, h), 0.2f, k_eps);
    }

    sl_world_destroy(&world);
}

static void test_setters_roundtrip(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(1.0f, 2.0f),
                          .velocity = sl_vec2_make(3.0f, 4.0f),
                          .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);

    SL_EXPECT(
        sl_world_body_set_position(&world, h, sl_vec2_make(-7.5f, 9.25f)));
    SL_EXPECT(
        sl_world_body_set_velocity(&world, h, sl_vec2_make(100.0f, -200.0f)));

    sl_vec2 position = sl_world_body_get_position(&world, h);
    SL_EXPECT(position.x == -7.5f && position.y == 9.25f);

    sl_vec2 velocity = sl_world_body_get_velocity(&world, h);
    SL_EXPECT(velocity.x == 100.0f && velocity.y == -200.0f);

    sl_world_destroy(&world);
}

static void test_setters_report_rejected_values(void)
{
    sl_world_config config = { .body_capacity = 1u };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    sl_body_desc desc = { .position = sl_vec2_make(1.0f, 2.0f),
                          .velocity = sl_vec2_make(3.0f, 4.0f),
                          .mass = 1.0f };
    sl_body_handle h = sl_world_body_create(&world, &desc);

    SL_EXPECT(!sl_world_body_set_position(&world, h, sl_vec2_make(NAN, 0.0f)));
    SL_EXPECT(
        !sl_world_body_set_velocity(&world, h, sl_vec2_make(0.0f, INFINITY)));

    sl_vec2 position = sl_world_body_get_position(&world, h);
    SL_EXPECT(position.x == 1.0f && position.y == 2.0f);

    sl_vec2 velocity = sl_world_body_get_velocity(&world, h);
    SL_EXPECT(velocity.x == 3.0f && velocity.y == 4.0f);

    sl_world_destroy(&world);
}

/* Seeded churn stress: xorshift32 (Marsaglia) driven against a reference
 * model of the pool. The seed is committed so failures reproduce. */
#define CHURN_SEED 0x5EEDC0DEu
#define CHURN_OPS 4096u
#define CHURN_CAPACITY 64u

static uint32_t rng_state = CHURN_SEED;

static uint32_t rng_next(void)
{
    rng_state ^= rng_state << 13u;
    rng_state ^= rng_state >> 17u;
    rng_state ^= rng_state << 5u;
    return rng_state;
}

/* Mirrors the engine's generation rule so the model ages identically. */
static uint32_t model_generation_bump(uint32_t generation)
{
    generation++;
    return (generation == 0u) ? 1u : generation;
}

static void test_churn_stress_matches_model(void)
{
    sl_world_config config = { .body_capacity = CHURN_CAPACITY };
    sl_world world = { 0 };
    SL_EXPECT(sl_world_init(&world, &config));

    /* Reference model: one entry per slot. */
    bool alive[CHURN_CAPACITY] = { false };
    uint32_t generation[CHURN_CAPACITY];
    int64_t tag[CHURN_CAPACITY];
    uint32_t live = 0u;
    int64_t next_tag = 0;
    for (uint32_t i = 0u; i < CHURN_CAPACITY; ++i) {
        generation[i] = 1u;
        tag[i] = -1;
    }

    bool counts_match = true;
    bool invariant_holds = true;
    bool validity_matches = true;
    bool payloads_intact = true;

    for (uint32_t op = 0u; op < CHURN_OPS; ++op) {
        const uint32_t roll = rng_next() % 24u;
        if (roll == 0u) {
            /* Rare reset: every generation bumps, all slots free. */
            sl_world_reset(&world);
            for (uint32_t i = 0u; i < CHURN_CAPACITY; ++i) {
                generation[i] = model_generation_bump(generation[i]);
                alive[i] = false;
                tag[i] = -1;
            }
            live = 0u;
        } else if (roll <= 11u) {
            /* Create: outcome must agree with the model's occupancy. The
             * payload carries its ordinal for later verification. */
            sl_body_desc desc = { .position = sl_vec2_make((float)next_tag,
                                                           (float)(-next_tag)),
                                  .velocity = sl_vec2_make(0.0f, 0.0f),
                                  .mass = 1.0f };
            sl_body_handle h = sl_world_body_create(&world, &desc);
            if (live == CHURN_CAPACITY) {
                counts_match = counts_match && sl_body_handle_is_null(h);
            } else {
                counts_match = counts_match && !sl_body_handle_is_null(h);
                if (!sl_body_handle_is_null(h)) {
                    validity_matches = validity_matches && !alive[h.index] &&
                                       h.generation == generation[h.index];
                    alive[h.index] = true;
                    tag[h.index] = next_tag;
                    live++;
                }
            }
            next_tag++;
        } else if (roll <= 19u) {
            /* Destroy a random live body via a model-fresh handle. */
            uint32_t victim = CHURN_CAPACITY;
            for (uint32_t probe = 0u; probe < 8u && victim == CHURN_CAPACITY;
                 ++probe) {
                const uint32_t candidate = rng_next() % CHURN_CAPACITY;
                if (alive[candidate]) {
                    victim = candidate;
                }
            }
            if (victim != CHURN_CAPACITY) {
                sl_body_handle h = { victim, generation[victim] };
                validity_matches =
                    validity_matches && sl_world_body_is_valid(&world, h);
                sl_world_body_destroy(&world, h);
                alive[victim] = false;
                generation[victim] = model_generation_bump(generation[victim]);
                live--;
            }
        }

        /* Full-model verification sweep, every operation. */
        const uint32_t reported = sl_world_body_count(&world);
        counts_match = counts_match && reported == live;
        invariant_holds =
            invariant_holds && reported + world.free_count == CHURN_CAPACITY;
        for (uint32_t i = 0u; i < CHURN_CAPACITY; ++i) {
            sl_body_handle probe = { i, generation[i] };
            const bool engine_alive = sl_world_body_is_valid(&world, probe);
            validity_matches = validity_matches && engine_alive == alive[i];
            if (engine_alive && alive[i]) {
                sl_vec2 position = sl_world_body_get_position(&world, probe);
                payloads_intact = payloads_intact &&
                                  position.x == (float)tag[i] &&
                                  position.y == (float)(-tag[i]);
            }
        }
    }

    SL_EXPECT(counts_match);
    SL_EXPECT(invariant_holds);
    SL_EXPECT(validity_matches);
    SL_EXPECT(payloads_intact);
    SL_EXPECT(next_tag > 0);

    sl_world_destroy(&world);
}

static const sl_test_case k_cases[] = {
    { "init_destroy_roundtrip", test_init_destroy_roundtrip },
    { "init_rejects_bad_capacity", test_init_rejects_bad_capacity },
    { "create_roundtrips_state", test_create_roundtrips_state },
    { "memory_bytes_at_capacity_max_within_budget",
      test_memory_bytes_at_capacity_max_within_budget },
    { "create_rejects_non_finite_state", test_create_rejects_non_finite_state },
    { "create_rejects_non_positive_mass",
      test_create_rejects_non_positive_mass },
    { "create_rejects_inverse_overflow", test_create_rejects_inverse_overflow },
    { "fill_to_capacity", test_fill_to_capacity },
    { "destroy_invalidates_handle", test_destroy_invalidates_handle },
    { "destroy_tolerates_bad_handles", test_destroy_tolerates_bad_handles },
    { "slot_reuse_bumps_generation", test_slot_reuse_bumps_generation },
    { "arrays_stay_packed_on_middle_destroy",
      test_arrays_stay_packed_on_middle_destroy },
    { "traversal_visits_each_body_once", test_traversal_visits_each_body_once },
    { "destroy_during_traversal_visits_each_once",
      test_destroy_during_traversal_visits_each_once },
    { "partial_cull_during_traversal", test_partial_cull_during_traversal },
    { "at_matches_handle_traversal", test_at_matches_handle_traversal },
    { "reset_invalidates_and_refills", test_reset_invalidates_and_refills },
    { "set_mass_updates_inv_mass", test_set_mass_updates_inv_mass },
    { "setters_roundtrip", test_setters_roundtrip },
    { "setters_report_rejected_values", test_setters_report_rejected_values },
    { "churn_stress_matches_model", test_churn_stress_matches_model },
};

int sl_world_suite(void)
{
    return sl_run_suite("world", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
