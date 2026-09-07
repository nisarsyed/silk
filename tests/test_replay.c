#include "replay.h"
#include "silk_test.h"
#include "suites.h"
#include <math.h>
#include <silk/step.h>
#include <string.h>

#define REPLAY_SEED UINT32_C(0x57D37E12)
static const float k_dt = 1.0f / 60.0f;

static bool checkpoint(const sl_world *a, const sl_world *b, uint32_t op)
{
    sl_replay_mismatch mismatch = { .fixture = "mixed",
                                    .seed = REPLAY_SEED,
                                    .operation = op };
    const bool equal = sl_replay_compare(a, b, &mismatch);
    if (!equal) {
        sl_replay_report(&mismatch);
    }
    SL_EXPECT(equal);
    return equal;
}

static uint32_t random_next(uint32_t *state)
{
    *state ^= *state << 13u;
    *state ^= *state >> 17u;
    *state ^= *state << 5u;
    return *state;
}

static uint64_t operation_apply(sl_world *world, uint32_t kind, uint32_t value)
{
    sl_shape shape = sl_shape_none();
    const bool made = (value & 1u) != 0u
                          ? sl_shape_make_circle(0.5f, &shape)
                          : sl_shape_make_box(0.5f, 0.5f, &shape);
    SL_EXPECT(made);
    if (kind == 0u || world->body_count == 0u) {
        const sl_body_type type = (sl_body_type)(value % 3u);
        const sl_body_desc desc = {
            .position = { (float)(value % 8u), (float)((value / 8u) % 4u) },
            .type = type,
            .mass = type == SL_BODY_DYNAMIC ? 1.0f : 0.0f,
            .shape = &shape,
            .friction = 0.6f
        };
        const sl_body_handle h = sl_world_body_create(world, &desc);
        return ((uint64_t)h.index << 32u) | h.generation;
    }
    const sl_body_handle body =
        sl_world_body_at(world, value % world->body_count);
    switch (kind) {
    case 1u:
        sl_world_body_destroy(world, body);
        return 0u;
    case 2u:
        return sl_world_body_set_position(
            world, body, (sl_vec2){ (float)(value % 8u), 2.0f });
    case 3u:
        return sl_world_body_set_angle(world, body,
                                       (float)(value % 10u) * 0.1f);
    case 4u:
        return sl_world_body_set_shape(world, body,
                                       (value & 2u) ? &shape : NULL);
    case 5u:
        return sl_world_body_set_friction(world, body,
                                          0.1f * (float)(value % 8u));
    case 6u:
        return sl_world_body_set_restitution(world, body,
                                             0.1f * (float)(value % 8u));
    case 7u:
        return sl_world_body_apply_force(world, body, (sl_vec2){ 0.5f, -0.5f });
    case 8u:
        return sl_world_body_apply_torque(world, body, 0.1f);
    case 9u:
        return sl_world_body_set_velocity(world, body, (sl_vec2){ 0.1f, 0.2f });
    case 10u:
        return sl_world_body_set_angular_velocity(world, body, 0.2f);
    case 11u:
        return sl_world_body_set_mass(world, body, 0.5f + (float)(value % 4u));
    case 12u: {
        const sl_joint_desc desc = {
            .kind = (value & 1u) ? SL_JOINT_DISTANCE : SL_JOINT_REVOLUTE,
            .body_a = body,
            .body_b = sl_world_body_at(world, (value + 1u) % world->body_count),
            .distance = { .length = 1.0f },
            .collide_connected = (value & 2u) != 0u
        };
        const sl_joint_handle h = sl_world_joint_create(world, &desc);
        return ((uint64_t)h.index << 32u) | h.generation;
    }
    case 13u:
        if (world->joint_count != 0u) {
            sl_world_joint_destroy(
                world, sl_world_joint_at(world, value % world->joint_count));
        }
        return 0u;
    case 14u:
        return sl_world_body_apply_force_at_point(
            world, body, (sl_vec2){ 0.1f, 0.2f }, (sl_vec2){ 1.0f, 1.0f });
    default:
        sl_world_step(world, k_dt);
        return 0u;
    }
}

static void mixed_replay(void)
{
    const sl_world_config config = { .body_capacity = 24u,
                                     .contact_capacity = 16u,
                                     .joint_capacity = 8u,
                                     .gravity = { 0.0f, -1.0f } };
    sl_world a = { 0 }, b = { 0 };
    SL_EXPECT(sl_world_init(&a, &config));
    SL_EXPECT(sl_world_init(&b, &config));
    uint32_t seed = REPLAY_SEED;
    for (uint32_t op = 0u; op < 4096u; ++op) {
        const uint32_t value = random_next(&seed);
        const uint32_t kind = op < 32u ? 0u : random_next(&seed) % 20u;
        const uint64_t result_a = operation_apply(&a, kind, value);
        const uint64_t result_b = operation_apply(&b, kind, value);
        SL_EXPECT(result_a == result_b);
        if (!checkpoint(&a, &b, op)) {
            break;
        }
        if (op % 257u == 256u) {
            sl_world_reset(&a);
            sl_world_reset(&b);
            if (!checkpoint(&a, &b, op)) {
                break;
            }
        }
    }
    sl_world_destroy(&a);
    sl_world_destroy(&b);
}

static void rejection_and_diagnostic(void)
{
    const sl_world_config config = { .body_capacity = 4u,
                                     .joint_capacity = 2u };
    sl_world a = { 0 }, b = { 0 };
    SL_EXPECT(sl_world_init(&a, &config));
    SL_EXPECT(sl_world_init(&b, &config));
    SL_EXPECT(operation_apply(&a, 0u, 0u) == operation_apply(&b, 0u, 0u));
    const sl_body_handle body = sl_world_body_at(&a, 0u);
    const float invalid[] = { NAN, INFINITY, -INFINITY };
    for (uint32_t i = 0u; i < 3u; ++i) {
        SL_EXPECT(!sl_world_body_set_position(&a, body,
                                              (sl_vec2){ invalid[i], 0.0f }));
        SL_EXPECT(!sl_world_body_set_velocity(&a, body,
                                              (sl_vec2){ 0.0f, invalid[i] }));
        SL_EXPECT(!sl_world_body_set_mass(&a, body, invalid[i]));
        SL_EXPECT(!sl_world_body_set_angle(&a, body, invalid[i]));
        SL_EXPECT(!sl_world_body_set_friction(&a, body, invalid[i]));
        SL_EXPECT(!sl_world_body_set_restitution(&a, body, invalid[i]));
        SL_EXPECT(!sl_world_body_apply_force(&a, body,
                                             (sl_vec2){ invalid[i], 0.0f }));
        SL_EXPECT(!sl_world_body_apply_torque(&a, body, invalid[i]));
        SL_EXPECT(checkpoint(&a, &b, i));
    }
    SL_EXPECT(!sl_world_body_set_mass(&a, body, -1.0f));
    SL_EXPECT(!sl_world_body_set_position(
        &a, body, (sl_vec2){ 2.0f * SL_POSITION_ABS_MAX, 0.0f }));
#ifdef NDEBUG
    sl_world_step(&a, NAN);
    sl_world_step(&a, 0.0f);
    sl_world_step(&a, 0x1p-149f);
#endif
    SL_EXPECT(checkpoint(&a, &b, 4u));
    sl_replay_mismatch mismatch = { .fixture = "perturbation",
                                    .seed = REPLAY_SEED,
                                    .operation = 5u };
    /* Signed zero proves this is a representation comparison, not float ==. */
    b.positions[0].x = -0.0f;
    SL_EXPECT(!sl_replay_compare(&a, &b, &mismatch));
    SL_EXPECT(strcmp(mismatch.field_name, "positions[i].x") == 0);
    SL_EXPECT_INT_EQ(mismatch.entity, 0u);
    SL_EXPECT(mismatch.value_a == 0u &&
              mismatch.value_b == UINT32_C(0x80000000));
    SL_EXPECT(mismatch.seed == REPLAY_SEED && mismatch.operation == 5u);
    b.positions[0].x = 0.0f;
    sl_world_reset(&a);
    SL_EXPECT(!sl_world_body_is_valid(&a, body));
    (void)operation_apply(&a, 0u, 0u);
    const sl_body_handle rebuilt = sl_world_body_at(&a, 0u);
    SL_EXPECT(rebuilt.generation != body.generation);
    const sl_vec2 pa = sl_world_body_get_position(&a, rebuilt);
    const sl_vec2 pb = sl_world_body_get_position(&b, body);
    SL_EXPECT(pa.x == pb.x && pa.y == pb.y);
    SL_EXPECT(sl_world_body_get_mass(&a, rebuilt) ==
              sl_world_body_get_mass(&b, body));
    sl_world_destroy(&a);
    sl_world_destroy(&b);
}

static void capacity_recovery(void)
{
    const sl_world_config config = { .body_capacity = 8u,
                                     .contact_capacity = 1u,
                                     .joint_capacity = 1u };
    sl_world worlds[2] = { { 0 } };
    sl_body_handle handles[2][8];
    sl_shape circle = sl_shape_none();
    SL_EXPECT(sl_shape_make_circle(1.0f, &circle));
    for (uint32_t w = 0u; w < 2u; ++w) {
        SL_EXPECT(sl_world_init(&worlds[w], &config));
        for (uint32_t i = 0u; i < 8u; ++i) {
            const sl_body_desc desc = {
                .mass = 1.0f,
                .shape = &circle,
                .position = { 5.0f * (float)(i / 2u) + 2.1f * (float)(i % 2u),
                              0.0f }
            };
            handles[w][i] = sl_world_body_create(&worlds[w], &desc);
        }
        const sl_body_desc extra = { .mass = 1.0f };
        SL_EXPECT(
            sl_body_handle_is_null(sl_world_body_create(&worlds[w], &extra)));
        const sl_joint_desc joint = { .body_a = handles[w][0],
                                      .body_b = handles[w][1],
                                      .distance = { .length = 2.1f },
                                      .collide_connected = false };
        const sl_joint_handle h = sl_world_joint_create(&worlds[w], &joint);
        SL_EXPECT(!sl_joint_handle_is_null(h));
        SL_EXPECT(
            sl_joint_handle_is_null(sl_world_joint_create(&worlds[w], &joint)));
        sl_world_joint_destroy(&worlds[w], h);
        SL_EXPECT(!sl_world_joint_is_valid(&worlds[w], h));
    }
    for (uint32_t pair = 0u; pair < 4u; ++pair) {
        for (uint32_t w = 0u; w < 2u; ++w) {
            sl_world_step(&worlds[w], k_dt);
            SL_EXPECT_INT_EQ(sl_world_contact_count(&worlds[w]), 1u);
            SL_EXPECT_INT_EQ(sl_world_contact_drop_count(&worlds[w]),
                             3u - pair);
        }
        SL_EXPECT(checkpoint(&worlds[0], &worlds[1], pair));
        for (uint32_t w = 0u; w < 2u; ++w) {
            sl_world_body_destroy(&worlds[w], handles[w][2u * pair]);
            sl_world_body_destroy(&worlds[w], handles[w][2u * pair + 1u]);
        }
        SL_EXPECT(checkpoint(&worlds[0], &worlds[1], pair));
    }
    sl_world_destroy(&worlds[0]);
    sl_world_destroy(&worlds[1]);
}
static void coupled_long_run(void)
{
    const sl_world_config config = { .body_capacity = 12u,
                                     .joint_capacity = 8u,
                                     .gravity = { 0.0f, -9.81f } };
    sl_world worlds[2] = { { 0 } };
    sl_shape box = sl_shape_none();
    SL_EXPECT(sl_shape_make_box(0.5f, 0.5f, &box));
    for (uint32_t w = 0u; w < 2u; ++w) {
        SL_EXPECT(sl_world_init(&worlds[w], &config));
        sl_body_handle previous = sl_body_handle_null();
        for (uint32_t i = 0u; i < 9u; ++i) {
            const sl_body_desc desc = { .type = i == 0u ? SL_BODY_STATIC
                                                        : SL_BODY_DYNAMIC,
                                        .mass = i == 0u ? 0.0f : 1.0f,
                                        .shape = &box,
                                        .friction = 0.6f,
                                        .position = { 0.0f, 0.9f * (float)i } };
            const sl_body_handle body = sl_world_body_create(&worlds[w], &desc);
            if (i > 0u) {
                const sl_joint_desc joint = {
                    .body_a = previous,
                    .body_b = body,
                    .kind = (i & 1u) ? SL_JOINT_DISTANCE : SL_JOINT_REVOLUTE,
                    .local_anchor_a = { 0.0f, 0.45f },
                    .local_anchor_b = { 0.0f, -0.45f },
                    .distance = { .length = 0.01f },
                    .collide_connected = true
                };
                SL_EXPECT(!sl_joint_handle_is_null(
                    sl_world_joint_create(&worlds[w], &joint)));
            }
            previous = body;
        }
    }
    for (uint32_t op = 0u; op < 2000u; ++op) {
        for (uint32_t w = 0u; w < 2u; ++w) {
            sl_world_step(&worlds[w], k_dt);
        }
        if (!checkpoint(&worlds[0], &worlds[1], op)) {
            break;
        }
        SL_EXPECT_INT_EQ(sl_world_contact_drop_count(&worlds[0]), 0u);
    }
    sl_replay_mismatch mismatch = { 0 };
    worlds[1].joint_distance_impulses[0] += 1.0f;
    SL_EXPECT(!sl_replay_compare(&worlds[0], &worlds[1], &mismatch));
    SL_EXPECT(strcmp(mismatch.field_name, "joint_distance_impulses[i]") == 0);
    sl_world_destroy(&worlds[0]);
    sl_world_destroy(&worlds[1]);
}

static const sl_test_case k_cases[] = {
    { "mixed operations", mixed_replay },
    { "coupled long run", coupled_long_run },
    { "rejection and first divergence", rejection_and_diagnostic },
    { "capacity and retry recovery", capacity_recovery },
};
int sl_replay_suite(void)
{
    return sl_run_suite("replay", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
