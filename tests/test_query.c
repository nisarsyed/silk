#include "replay.h"
#include "silk_test.h"
#include "suites.h"
#include <float.h>
#include <math.h>
#include <silk/query.h>
#include <silk/step.h>
#include <string.h>

#define QUERY_CAPACITY 32u
#define QUERY_SEED UINT32_C(0x62A11CE5)

static void same_handle(sl_body_handle a, sl_body_handle b)
{
    SL_EXPECT_INT_EQ(a.index, b.index);
    SL_EXPECT_INT_EQ(a.generation, b.generation);
}
static uint32_t random_next(uint32_t *state)
{
    *state ^= *state << 13u;
    *state ^= *state >> 17u;
    *state ^= *state << 5u;
    return *state;
}
static float coordinate(uint32_t *state)
{
    return (float)(random_next(state) % 49u) * 0.25f - 6.0f;
}
static bool selected(const sl_world *world, sl_body_handle body, uint32_t mask)
{
    const uint32_t bit = UINT32_C(1)
                         << (uint32_t)sl_world_body_get_type(world, body);
    return mask == 0u || (mask & bit) != 0u;
}
/* Brute-force public traversal is independent of the tree and query scratch.
 * Insertion ordering is deliberately different from production heapsort. */
static uint32_t oracle(const sl_world *world, sl_aabb bounds, bool point,
                       uint32_t mask, sl_body_handle *out)
{
    uint32_t count = 0u;
    for (uint32_t row = 0u; row < sl_world_body_count(world); ++row) {
        const sl_body_handle body = sl_world_body_at(world, row);
        const sl_shape *shape = sl_world_body_get_shape(world, body);
        const sl_transform transform = sl_world_body_get_transform(world, body);
        if (!selected(world, body, mask) || shape->kind == SL_SHAPE_NONE)
            continue;
        if (point ? !sl_shape_contains_point(shape, transform, bounds.lower)
                  : !sl_aabb_overlaps(sl_shape_aabb(shape, transform), bounds))
            continue;
        uint32_t at = count;
        while (at > 0u && out[at - 1u].index > body.index) {
            out[at] = out[at - 1u];
            --at;
        }
        out[at] = body;
        ++count;
    }
    return count;
}
static sl_query_ray_result ray_oracle(const sl_world *world, sl_ray ray,
                                      uint32_t mask)
{
    sl_query_ray_result result = { 0 };
    for (uint32_t row = 0u; row < sl_world_body_count(world); ++row) {
        const sl_body_handle body = sl_world_body_at(world, row);
        sl_ray_hit hit;
        if (!selected(world, body, mask) ||
            !sl_shape_ray_cast(sl_world_body_get_shape(world, body),
                               sl_world_body_get_transform(world, body), ray,
                               &hit))
            continue;
        if (!result.hit || hit.fraction < result.geometry.fraction ||
            (hit.fraction == result.geometry.fraction &&
             body.index < result.body.index)) {
            result.hit = true;
            result.body = body;
            result.geometry = hit;
        }
    }
    return result;
}
static void check_queries(const sl_world *world, sl_vec2 point, uint32_t mask,
                          uint32_t capacity)
{
    const sl_aabb box = { { point.x - 1.5f, point.y - 1.5f },
                          { point.x + 1.5f, point.y + 1.5f } };
    for (uint32_t kind = 0u; kind < 2u; ++kind) {
        sl_body_handle expected[QUERY_CAPACITY], actual[QUERY_CAPACITY];
        const sl_body_handle sentinel = { UINT32_MAX, UINT32_MAX };
        for (uint32_t i = 0u; i < QUERY_CAPACITY; ++i)
            actual[i] = sentinel;
        const sl_aabb bounds = kind == 0u ? box : (sl_aabb){ point, point };
        const uint32_t count =
            oracle(world, bounds, kind != 0u, mask, expected);
        sl_query_result result = { UINT32_MAX, false };
        SL_EXPECT(kind == 0u ? sl_world_query_aabb(world, box, mask, actual,
                                                   capacity, &result)
                             : sl_world_query_point(world, point, mask, actual,
                                                    capacity, &result));
        SL_EXPECT_INT_EQ(result.count, count);
        SL_EXPECT(result.truncated == (count > capacity));
        const uint32_t written = count < capacity ? count : capacity;
        for (uint32_t i = 0u; i < written; ++i)
            same_handle(actual[i], expected[i]);
        for (uint32_t i = written; i < QUERY_CAPACITY; ++i)
            same_handle(actual[i], sentinel);
    }
    const sl_ray ray = { { -8.0f, point.y }, { 16.0f, 0.0f } };
    const sl_query_ray_result expected = ray_oracle(world, ray, mask);
    sl_query_ray_result actual;
    SL_EXPECT(sl_world_query_ray(world, ray, mask, &actual));
    SL_EXPECT(actual.hit == expected.hit);
    same_handle(actual.body, expected.body);
    SL_EXPECT(actual.geometry.fraction == expected.geometry.fraction);
    SL_EXPECT(actual.geometry.point.x == expected.geometry.point.x);
    SL_EXPECT(actual.geometry.point.y == expected.geometry.point.y);
    SL_EXPECT(actual.geometry.normal.x == expected.geometry.normal.x);
    SL_EXPECT(actual.geometry.normal.y == expected.geometry.normal.y);
}

static void seeded_mutations_and_replay(void)
{
    sl_world world = { 0 }, twin = { 0 };
    const sl_world_config config = { .body_capacity = QUERY_CAPACITY,
                                     .contact_capacity = 512u };
    SL_EXPECT(sl_world_init(&world, &config));
    SL_EXPECT(sl_world_init(&twin, &config));
    sl_shape shapes[3] = { sl_shape_none(), sl_shape_none(), sl_shape_none() };
    SL_EXPECT(sl_shape_make_circle(0.7f, &shapes[1]));
    SL_EXPECT(sl_shape_make_box(0.6f, 0.25f, &shapes[2]));
    uint32_t rng = QUERY_SEED;
    for (uint32_t op = 0u; op < 1024u; ++op) {
        const uint32_t roll = random_next(&rng);
        const sl_vec2 point = { coordinate(&rng), coordinate(&rng) };
        const uint32_t count = sl_world_body_count(&world);
        if (op % 211u == 210u) {
            const sl_body_handle old = count > 0u ? sl_world_body_at(&world, 0u)
                                                  : sl_body_handle_null();
            sl_world_reset(&world);
            sl_world_reset(&twin);
            SL_EXPECT(!sl_world_body_is_valid(&world, old));
        } else if (count == 0u || roll % 6u == 0u) {
            const sl_body_type type = (sl_body_type)(random_next(&rng) % 3u);
            const sl_body_desc desc = {
                .position = point,
                .mass = type == SL_BODY_DYNAMIC ? 1.0f : 0.0f,
                .type = type,
                .shape = &shapes[random_next(&rng) % 3u]
            };
            same_handle(sl_world_body_create(&world, &desc),
                        sl_world_body_create(&twin, &desc));
        } else {
            const sl_body_handle body =
                sl_world_body_at(&world, random_next(&rng) % count);
            switch (roll % 6u) {
            case 1u:
                sl_world_body_destroy(&world, body);
                sl_world_body_destroy(&twin, body);
                SL_EXPECT(!sl_world_body_is_valid(&world, body));
                break;
            case 2u:
                SL_EXPECT(sl_world_body_set_position(&world, body, point));
                SL_EXPECT(sl_world_body_set_position(&twin, body, point));
                break;
            case 3u:
                SL_EXPECT(sl_world_body_set_angle(&world, body, point.x));
                SL_EXPECT(sl_world_body_set_angle(&twin, body, point.x));
                break;
            case 4u: {
                const sl_shape *shape = &shapes[random_next(&rng) % 3u];
                SL_EXPECT(sl_world_body_set_shape(&world, body, shape));
                SL_EXPECT(sl_world_body_set_shape(&twin, body, shape));
                break;
            }
            default:
                sl_world_step(&world, 1.0f / 60.0f);
                sl_world_step(&twin, 1.0f / 60.0f);
                break;
            }
        }
        const uint32_t capacities[] = { 0u, 1u, 5u, QUERY_CAPACITY };
        check_queries(&world, point, op % 8u, capacities[op % 4u]);
        SL_EXPECT(
            sl_replay_check(&world, &twin, "query mutations", QUERY_SEED, op));
    }
    sl_world_destroy(&world);
    sl_world_destroy(&twin);
}

static void boundaries_order_and_snapshots(void)
{
    sl_world world = { 0 }, twin = { 0 };
    const sl_world_config config = { .body_capacity = QUERY_CAPACITY,
                                     .contact_capacity = 512u,
                                     .joint_capacity = 1u };
    SL_EXPECT(sl_world_init(&world, &config));
    SL_EXPECT(sl_world_init(&twin, &config));
    sl_shape shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.0f, &shape));
    sl_body_handle handles[QUERY_CAPACITY];
    for (uint32_t i = 0u; i < QUERY_CAPACITY; ++i) {
        const sl_body_type type = (sl_body_type)(i % 3u);
        const sl_body_desc desc = { .mass =
                                        type == SL_BODY_DYNAMIC ? 1.0f : 0.0f,
                                    .type = type,
                                    .shape = &shape };
        handles[i] = sl_world_body_create(&world, &desc);
        same_handle(handles[i], sl_world_body_create(&twin, &desc));
    }
    for (uint32_t mask = 0u; mask <= SL_QUERY_ALL; ++mask) {
        check_queries(&world, (sl_vec2){ 0.0f, 0.0f }, mask, QUERY_CAPACITY);
        check_queries(&world, (sl_vec2){ 1.0f, 0.0f }, mask, 1u);
    }
    sl_query_result result;
    SL_EXPECT(sl_world_query_point(&world, (sl_vec2){ 1.0f, 0.0f }, 0u, NULL,
                                   0u, &result));
    SL_EXPECT_INT_EQ(result.count, QUERY_CAPACITY);
    SL_EXPECT(result.truncated);
    const sl_aabb fat_only = { { 1.01f, 0.0f }, { 1.02f, 0.01f } };
    SL_EXPECT(sl_world_query_aabb(&world, fat_only, 0u, NULL, 0u, &result));
    SL_EXPECT_INT_EQ(result.count, 0u);
    const sl_ray ray = { { -2.0f, 0.0f }, { 4.0f, 0.0f } };
    sl_query_ray_result hit;
    SL_EXPECT(sl_world_query_ray(&world, ray, 0u, &hit));
    SL_EXPECT(hit.hit);
    same_handle(hit.body, handles[0]);
    SL_EXPECT_NEAR(hit.geometry.fraction, 0.25f, SL_EPSILON);
    SL_EXPECT_NEAR(hit.geometry.normal.x, -1.0f, SL_EPSILON);
    SL_EXPECT(sl_world_query_ray(
        &world, (sl_ray){ { 0.0f, 0.0f }, { 4.0f, 0.0f } }, 0u, &hit));
    SL_EXPECT(!hit.hit && sl_body_handle_is_null(hit.body));
    SL_EXPECT(sl_world_query_ray(
        &world, (sl_ray){ { 1.0f, 0.0f }, { 2.0f, 0.0f } }, 0u, &hit));
    SL_EXPECT(!hit.hit); /* origin on boundary */
    SL_EXPECT(sl_world_query_ray(
        &world, (sl_ray){ { -2.0f, 1.0f }, { 4.0f, 0.0f } }, 0u, &hit));
    SL_EXPECT(hit.hit); /* grazing */
    SL_EXPECT(sl_world_query_ray(
        &world,
        (sl_ray){ { -SL_QUERY_RAY_COORDINATE_MAX, 0.0f },
                  { 2.0f * SL_QUERY_RAY_COORDINATE_MAX, 0.0f } },
        0u, &hit));
    SL_EXPECT(hit.hit);
    same_handle(hit.body, handles[0]);
    SL_EXPECT_NEAR(hit.geometry.point.x, -1.0f, SL_EPSILON);
    SL_EXPECT_NEAR(hit.geometry.normal.x, -1.0f, SL_EPSILON);
    sl_world_body_destroy(&world, handles[0]);
    sl_world_body_destroy(&twin, handles[0]);
    const sl_body_desc replacement = { .mass = 1.0f, .shape = &shape };
    const sl_body_handle reused = sl_world_body_create(&world, &replacement);
    same_handle(reused, sl_world_body_create(&twin, &replacement));
    SL_EXPECT_INT_EQ(reused.index, handles[0].index);
    SL_EXPECT(reused.generation != handles[0].generation);
    SL_EXPECT(sl_world_query_ray(&world, ray, 0u, &hit));
    same_handle(hit.body, reused);
    const sl_joint_desc joint = { .kind = SL_JOINT_REVOLUTE,
                                  .body_a = reused,
                                  .body_b = handles[3],
                                  .collide_connected = true };
    const sl_joint_handle j = sl_world_joint_create(&world, &joint);
    const sl_joint_handle k = sl_world_joint_create(&twin, &joint);
    SL_EXPECT(j.generation != 0u && j.index == k.index &&
              j.generation == k.generation);
    sl_world_step(&world, 1.0f / 60.0f);
    sl_world_step(&twin, 1.0f / 60.0f);
    SL_EXPECT(sl_world_contact_count(&world) > 0u);
    const sl_contact *contact = sl_world_contact_at(&world, 0u);
    unsigned char saved[sizeof(*contact)];
    memcpy(saved, contact, sizeof(saved));
    const sl_shape *shape_pointer = sl_world_body_get_shape(&world, reused);
    check_queries(&world, (sl_vec2){ 0.0f, 0.0f }, 0u, QUERY_CAPACITY);
    SL_EXPECT(contact == sl_world_contact_at(&world, 0u));
    SL_EXPECT(memcmp(contact, saved, sizeof(saved)) == 0);
    SL_EXPECT(shape_pointer == sl_world_body_get_shape(&world, reused));
    SL_EXPECT(
        sl_replay_check(&world, &twin, "query snapshots", QUERY_SEED, 0u));
    sl_world_step(&world, 1.0f / 60.0f);
    sl_world_step(&twin, 1.0f / 60.0f);
    SL_EXPECT(sl_replay_check(&world, &twin, "after query snapshots",
                              QUERY_SEED, 1u));
    sl_world_destroy(&world);
    sl_world_destroy(&twin);
}

static void rejection_and_empty(void)
{
    sl_world world = { 0 };
    const sl_world_config config = { .body_capacity = 1u };
    SL_EXPECT(sl_world_init(&world, &config));
    sl_body_handle buffer = { 123u, 456u };
    sl_query_result result = { 789u, true };
    const sl_aabb box = { { -1.0f, -1.0f }, { 1.0f, 1.0f } };
    SL_EXPECT(!sl_world_query_aabb(&world, box, 8u, &buffer, 1u, &result));
    SL_EXPECT(!sl_world_query_aabb(&world, box, 0u, NULL, 1u, &result));
    SL_EXPECT(!sl_world_query_aabb(&world, box, 0u, &buffer,
                                   SL_BODY_COUNT_MAX + 1u, &result));
    SL_EXPECT(!sl_world_query_aabb(&world, box, 0u, &buffer, 1u, NULL));
    SL_EXPECT(!sl_world_query_aabb(NULL, box, 0u, &buffer, 1u, &result));
    SL_EXPECT(!sl_world_query_aabb(&world,
                                   (sl_aabb){ { 1.0f, 0.0f }, { -1.0f, 0.0f } },
                                   0u, &buffer, 1u, &result));
    const float bad[] = { NAN, INFINITY, -INFINITY };
    for (uint32_t i = 0u; i < 3u; ++i) {
        SL_EXPECT(!sl_world_query_point(&world, (sl_vec2){ bad[i], 0.0f }, 0u,
                                        &buffer, 1u, &result));
        SL_EXPECT(!sl_world_query_aabb(
            &world, (sl_aabb){ { bad[i], 0.0f }, { 1.0f, 1.0f } }, 0u, &buffer,
            1u, &result));
    }
    SL_EXPECT_INT_EQ(buffer.index, 123u);
    SL_EXPECT_INT_EQ(buffer.generation, 456u);
    SL_EXPECT(result.count == 789u && result.truncated);
    const sl_ray invalid[] = {
        { { 0.0f, 0.0f }, { 0.0f, 0.0f } },
        { { 0.0f, 0.0f }, { SL_EPSILON, 0.0f } },
        { { NAN, 0.0f }, { 1.0f, 0.0f } },
        { { 0.0f, 0.0f }, { INFINITY, 0.0f } },
        { { 0.0f, 0.0f }, { FLT_MAX, 0.0f } },
        { { SL_QUERY_RAY_COORDINATE_MAX, 0.0f }, { 1.0f, 0.0f } },
        { { -SL_QUERY_RAY_COORDINATE_MAX - 1.0f, 0.0f }, { 2.0f, 0.0f } }
    };
    sl_query_ray_result ray_result = { .hit = true,
                                       .body = { 12u, 34u },
                                       .geometry = { .fraction = 0.5f } };
    unsigned char saved[sizeof(ray_result)];
    memcpy(saved, &ray_result, sizeof(saved));
    for (uint32_t i = 0u; i < (uint32_t)(sizeof(invalid) / sizeof(invalid[0]));
         ++i) {
        SL_EXPECT(!sl_world_query_ray(&world, invalid[i], 0u, &ray_result));
        SL_EXPECT(memcmp(&ray_result, saved, sizeof(saved)) == 0);
    }
    const sl_ray valid = { { -SL_QUERY_RAY_COORDINATE_MAX, 0.0f },
                           { 2.0f * SL_QUERY_RAY_COORDINATE_MAX, 0.0f } };
    SL_EXPECT(!sl_world_query_ray(&world, valid, 8u, &ray_result));
    SL_EXPECT(!sl_world_query_ray(&world, valid, 0u, NULL));
    SL_EXPECT(sl_world_query_ray(&world, valid, 0u, &ray_result));
    SL_EXPECT(!ray_result.hit && ray_result.geometry.fraction == 0.0f);
    SL_EXPECT(sl_world_query_point(&world, (sl_vec2){ FLT_MAX, -FLT_MAX }, 0u,
                                   &buffer, 1u, &result));
    SL_EXPECT(result.count == 0u && !result.truncated);
    SL_EXPECT(sl_world_query_aabb(
        &world, (sl_aabb){ { -FLT_MAX, -FLT_MAX }, { FLT_MAX, FLT_MAX } }, 0u,
        &buffer, 1u, &result));
    SL_EXPECT(result.count == 0u && !result.truncated);
    sl_shape box_shape = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(1.0f, 0.25f, &box_shape));
    const sl_body_desc desc = { .mass = 1.0f,
                                .angle = SL_PI / 4.0f,
                                .shape = &box_shape };
    const sl_body_handle body = sl_world_body_create(&world, &desc);
    const sl_vec2 corner = { 0.8f, 0.8f };
    SL_EXPECT(sl_world_query_aabb(&world, (sl_aabb){ corner, corner }, 0u,
                                  &buffer, 1u, &result));
    SL_EXPECT_INT_EQ(result.count, 1u);
    same_handle(buffer, body);
    SL_EXPECT(sl_world_query_point(&world, corner, 0u, &buffer, 1u, &result));
    SL_EXPECT_INT_EQ(result.count, 0u);
    sl_world_destroy(&world);
    SL_EXPECT(!sl_world_query_aabb(&world, box, 0u, &buffer, 1u, &result));
}
static const sl_test_case k_cases[] = {
    { "seeded mutation oracle and replay", seeded_mutations_and_replay },
    { "boundaries ordering and snapshots", boundaries_order_and_snapshots },
    { "rejection and empty world", rejection_and_empty },
};
int sl_query_suite(void)
{
    return sl_run_suite("query", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
