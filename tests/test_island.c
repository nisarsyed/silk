#include "silk_test.h"
#include "suites.h"
#include "world_internal.h"

#include <silk/step.h>

#define ORACLE_CAPACITY 24u

/* Deliberately independent transitive closure, not another union-find. */
static void connectivity_check(const sl_world *owner)
{
    const sl_world_state *w = owner->state;
    bool connected[ORACLE_CAPACITY][ORACLE_CAPACITY] = { { false } };
    uint32_t body_seen[ORACLE_CAPACITY] = { 0u };
    uint32_t contact_seen[96] = { 0u };
    uint32_t joint_seen[48] = { 0u };
    bool dynamic[ORACLE_CAPACITY] = { false };
    for (uint32_t a = 0u; a < w->body_capacity; ++a) {
        const uint32_t row = w->slots[a].dense;
        dynamic[a] = row != SL_BODY_DENSE_NONE &&
                     w->types[row] == (uint8_t)SL_BODY_DYNAMIC;
        connected[a][a] = dynamic[a];
    }
    for (uint32_t row = 0u; row < w->contact_count + w->joint_count; ++row) {
        uint32_t a, b;
        if (row < w->contact_count) {
            if (!w->contacts[row].touching) {
                continue;
            }
            a = w->contacts[row].body_a.index;
            b = w->contacts[row].body_b.index;
        } else {
            a = w->joint_bodies_a[row - w->contact_count].index;
            b = w->joint_bodies_b[row - w->contact_count].index;
        }
        if (dynamic[a] && dynamic[b]) {
            connected[a][b] = true;
            connected[b][a] = true;
        }
    }
    for (uint32_t k = 0u; k < w->body_capacity; ++k) {
        for (uint32_t a = 0u; a < w->body_capacity; ++a) {
            for (uint32_t b = 0u; b < w->body_capacity; ++b) {
                connected[a][b] =
                    connected[a][b] || (connected[a][k] && connected[k][b]);
            }
        }
    }
    for (uint32_t a = 0u; a < w->body_capacity; ++a) {
        if (!dynamic[a]) {
            SL_EXPECT(w->body_islands[a] == SL_BODY_DENSE_NONE);
            continue;
        }
        SL_EXPECT(w->body_islands[a] < w->island_count);
        for (uint32_t b = 0u; b < w->body_capacity; ++b) {
            if (dynamic[b]) {
                SL_EXPECT(connected[a][b] ==
                          (w->body_islands[a] == w->body_islands[b]));
            }
        }
    }
    uint32_t bodies = 0u, contacts = 0u, joints = 0u;
    for (uint32_t id = 0u; id < w->island_count; ++id) {
        const sl_island *island = &w->islands[id];
        SL_EXPECT(island->body_count > 0u);
        SL_EXPECT_INT_EQ(island->body_offset, bodies);
        SL_EXPECT_INT_EQ(island->contact_offset, contacts);
        SL_EXPECT_INT_EQ(island->joint_offset, joints);
        if (id > 0u) {
            SL_EXPECT(w->islands[id - 1u].root_slot < island->root_slot);
        }
        for (uint32_t i = 0u; i < island->body_count; ++i) {
            const uint32_t slot = w->island_bodies[bodies + i];
            SL_EXPECT_INT_EQ(w->body_islands[slot], id);
            SL_EXPECT(slot >= island->root_slot);
            body_seen[slot]++;
        }
        for (uint32_t i = 0u; i < island->contact_count; ++i) {
            const uint32_t row = w->island_contacts[contacts + i];
            SL_EXPECT(row < w->contact_count && w->contacts[row].touching);
            if (i > 0u) {
                SL_EXPECT(w->island_contacts[contacts + i - 1u] < row);
            }
            const sl_contact *c = &w->contacts[row];
            SL_EXPECT(w->body_islands[c->body_a.index] == id ||
                      w->body_islands[c->body_b.index] == id);
            contact_seen[row]++;
        }
        for (uint32_t i = 0u; i < island->joint_count; ++i) {
            const uint32_t row = w->island_joints[joints + i];
            SL_EXPECT(row < w->joint_count);
            if (i > 0u) {
                SL_EXPECT(w->island_joints[joints + i - 1u] < row);
            }
            SL_EXPECT(w->body_islands[w->joint_bodies_a[row].index] == id ||
                      w->body_islands[w->joint_bodies_b[row].index] == id);
            joint_seen[row]++;
        }
        bodies += island->body_count;
        contacts += island->contact_count;
        joints += island->joint_count;
    }
    SL_EXPECT_INT_EQ(bodies, w->island_body_count);
    SL_EXPECT_INT_EQ(contacts, w->contact_constraint_count);
    SL_EXPECT_INT_EQ(joints, w->joint_constraint_count);
    for (uint32_t i = 0u; i < w->body_capacity; ++i) {
        SL_EXPECT_INT_EQ(body_seen[i], dynamic[i] ? 1u : 0u);
    }
    for (uint32_t i = 0u; i < w->contact_count; ++i) {
        SL_EXPECT_INT_EQ(contact_seen[i], w->contacts[i].touching ? 1u : 0u);
    }
    for (uint32_t i = 0u; i < w->joint_count; ++i) {
        SL_EXPECT_INT_EQ(joint_seen[i], 1u);
    }
}

static uint32_t random_next(uint32_t *state)
{
    *state = *state * 1664525u + 1013904223u;
    return *state;
}

static void seeded_churn(void)
{
    sl_world world = { 0 };
    const sl_world_config config = {
        .body_capacity = ORACLE_CAPACITY,
        .contact_capacity = 96u,
        .joint_capacity = 48u,
    };
    SL_EXPECT(sl_world_init(&world, &config));
    sl_shape shape;
    SL_EXPECT(sl_shape_make_circle(0.45f, &shape));
    uint32_t seed = 0x63c0ffeeu;
    for (uint32_t op = 0u; op < 500u; ++op) {
        const uint32_t choice = random_next(&seed) % 7u;
        const uint32_t count = sl_world_body_count(&world);
        const sl_body_handle a =
            count > 0u ? sl_world_body_at(&world, random_next(&seed) % count)
                       : sl_body_handle_null();
        if (choice < 3u || count < 2u) {
            const sl_body_type type = (sl_body_type)(random_next(&seed) % 3u);
            const sl_body_desc desc = {
                .type = type,
                .mass = type == SL_BODY_DYNAMIC ? 1.0f : 0.0f,
                .position = { (float)(random_next(&seed) % 6u),
                              (float)(random_next(&seed) % 4u) },
                .shape = choice == 0u ? NULL : &shape,
            };
            (void)sl_world_body_create(&world, &desc);
        } else if (choice == 3u) {
            sl_world_body_destroy(&world, a);
            SL_EXPECT(world.state->body_islands[a.index] == SL_BODY_DENSE_NONE);
        } else if (choice == 4u) {
            const sl_joint_desc desc = {
                .kind = SL_JOINT_DISTANCE,
                .body_a = a,
                .body_b = sl_world_body_at(&world, random_next(&seed) % count),
                .distance = { .length = 1.0f },
                .collide_connected = (op % 2u) == 0u,
            };
            (void)sl_world_joint_create(&world, &desc);
        } else if (choice == 5u && world.state->joint_count > 0u) {
            sl_world_joint_destroy(&world, sl_world_joint_at(&world, 0u));
        } else {
            SL_EXPECT(sl_world_body_set_position(
                &world, a, (sl_vec2){ (float)(op % 6u), 0.0f }));
        }
        sl_world_step(&world, 1.0f / 60.0f);
        connectivity_check(&world);
        if (op == 250u) {
            sl_world_reset(&world);
            SL_EXPECT_INT_EQ(world.state->island_count, 0u);
        }
    }
    sl_world_destroy(&world);
}

static void boundary_check(sl_body_type boundary_type)
{
    sl_world world = { 0 };
    const sl_world_config config = { .body_capacity = 8u,
                                     .contact_capacity = 16u,
                                     .joint_capacity = 8u };
    SL_EXPECT(sl_world_init(&world, &config));
    sl_shape circle;
    SL_EXPECT(sl_shape_make_circle(0.5f, &circle));
    sl_body_handle bodies[4];
    for (uint32_t i = 0u; i < 4u; ++i) {
        const sl_body_desc desc = { .type = i == 0u ? boundary_type
                                                    : SL_BODY_DYNAMIC,
                                    .mass = i == 0u ? 0.0f : 1.0f,
                                    .position = { (float)i * 1.01f, 0.0f },
                                    .shape = &circle };
        bodies[i] = sl_world_body_create(&world, &desc);
    }
    sl_world_step(&world, 1.0f / 60.0f);
    connectivity_check(&world);
    SL_EXPECT_INT_EQ(world.state->island_count, 1u);
    SL_EXPECT(world.state->contacts[0].manifold.points[0].separation > 0.0f);
    for (uint32_t i = 1u; i < 4u; ++i) {
        SL_EXPECT(sl_world_body_set_position(
            &world, bodies[i], (sl_vec2){ (float)i * 3.0f, 0.0f }));
        const sl_joint_desc desc = { .kind = SL_JOINT_DISTANCE,
                                     .body_a = bodies[0],
                                     .body_b = bodies[i],
                                     .distance = { .length =
                                                       (float)i * 3.0f } };
        SL_EXPECT(
            !sl_joint_handle_is_null(sl_world_joint_create(&world, &desc)));
    }
    sl_world_step(&world, 1.0f / 60.0f);
    connectivity_check(&world);
    SL_EXPECT_INT_EQ(world.state->island_count, 3u);
    world.state->work_total.graph_parent_probes = UINT64_MAX;
    sl_world_step(&world, 1.0f / 60.0f);
    SL_EXPECT(world.state->work_total.graph_parent_probes == UINT64_MAX);
    sl_world_destroy(&world);
}

static void boundaries_and_speculation(void)
{
    boundary_check(SL_BODY_STATIC);
    boundary_check(SL_BODY_KINEMATIC);
}

static void capacity_and_cycles(void)
{
    sl_world world = { 0 };
    const sl_world_config config = { .body_capacity = 4u,
                                     .contact_capacity = 1u,
                                     .joint_capacity = 4u };
    SL_EXPECT(sl_world_init(&world, &config));
    sl_body_handle bodies[4];
    const sl_body_desc desc = { .type = SL_BODY_DYNAMIC, .mass = 1.0f };
    for (uint32_t i = 0u; i < 4u; ++i) {
        bodies[i] = sl_world_body_create(&world, &desc);
        SL_EXPECT(!sl_body_handle_is_null(bodies[i]));
    }
    SL_EXPECT(sl_body_handle_is_null(sl_world_body_create(&world, &desc)));
    sl_world_step(&world, 1.0f / 60.0f);
    SL_EXPECT_INT_EQ(world.state->island_count, 4u);
    connectivity_check(&world);
    for (uint32_t i = 0u; i < 4u; ++i) {
        const sl_joint_desc joint = { .kind = SL_JOINT_REVOLUTE,
                                      .body_a = bodies[i],
                                      .body_b = bodies[(i + 1u) % 4u] };
        SL_EXPECT(
            !sl_joint_handle_is_null(sl_world_joint_create(&world, &joint)));
    }
    sl_world_step(&world, 1.0f / 60.0f);
    SL_EXPECT_INT_EQ(world.state->island_count, 1u);
    connectivity_check(&world);
    sl_world_body_destroy(&world, bodies[1]);
    sl_world_body_destroy(&world, bodies[3]);
    const sl_body_handle reused = sl_world_body_create(&world, &desc);
    SL_EXPECT(world.state->body_islands[reused.index] == SL_BODY_DENSE_NONE);
    sl_world_step(&world, 1.0f / 60.0f);
    SL_EXPECT_INT_EQ(world.state->island_count, 3u);
    connectivity_check(&world);
    sl_world_destroy(&world);
}

static const sl_test_case k_cases[] = {
    { "capacity, cycles and immediate reuse", capacity_and_cycles },
    { "seeded connectivity and range oracle", seeded_churn },
    { "shared boundaries and speculative edges", boundaries_and_speculation },
};
int sl_island_suite(void)
{
    return sl_run_suite("island", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
