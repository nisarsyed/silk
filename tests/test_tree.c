#include "silk_test.h"
#include "suites.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "tree.h"

#define SL_TREE_TEST_CAPACITY 64u

typedef struct sl_tree_fixture {
    sl_tree tree;
    void *memory;
    size_t memory_bytes;
} sl_tree_fixture;

typedef struct sl_proxy_model {
    sl_aabb aabb;
    uint32_t proxy;
    sl_tree_root_kind root;
    bool active;
} sl_proxy_model;

static sl_aabb aabb_make(float lower_x, float lower_y, float upper_x,
                         float upper_y)
{
    return sl_aabb_make(sl_vec2_make(lower_x, lower_y),
                        sl_vec2_make(upper_x, upper_y));
}

static sl_tree_fixture fixture_make(uint32_t capacity)
{
    sl_tree_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    fixture.memory_bytes = sl_tree_memory_bytes(capacity);
    fixture.memory = malloc(fixture.memory_bytes);
    SL_EXPECT(fixture.memory != NULL);
    if (fixture.memory != NULL) {
        SL_EXPECT(sl_tree_init(&fixture.tree, fixture.memory,
                               fixture.memory_bytes, capacity));
    }
    return fixture;
}

static void fixture_destroy(sl_tree_fixture *fixture)
{
    free(fixture->memory);
    memset(fixture, 0, sizeof(*fixture));
}

static void memory_accounting_and_init_validation(void)
{
    SL_EXPECT_INT_EQ(sl_tree_memory_bytes(0u), 0u);
    SL_EXPECT_INT_EQ(sl_tree_memory_bytes(SL_BODY_COUNT_MAX + 1u), 0u);
    SL_EXPECT_INT_EQ(sl_tree_memory_bytes(1u), 80u);
    SL_EXPECT_INT_EQ(sl_tree_memory_bytes(17u),
                     34u * sizeof(sl_tree_node) + 34u * sizeof(uint32_t));

    const size_t bytes = sl_tree_memory_bytes(4u);
    void *memory = malloc(bytes + 8u);
    SL_EXPECT(memory != NULL);
    if (memory == NULL) {
        return;
    }

    sl_tree tree;
    memset(&tree, 0xa5, sizeof(tree));
    SL_EXPECT(!sl_tree_init(&tree, memory, bytes - 1u, 4u));
    SL_EXPECT(tree.nodes == NULL && tree.memory == NULL);
    SL_EXPECT(!sl_tree_init(&tree, (unsigned char *)memory + 1u, bytes, 4u));
    SL_EXPECT(tree.nodes == NULL && tree.memory == NULL);
    SL_EXPECT(!sl_tree_init(&tree, memory, bytes, 0u));
    SL_EXPECT(tree.nodes == NULL && tree.memory == NULL);
    free(memory);
}

static void roots_and_boundaries_are_isolated(void)
{
    sl_tree_fixture fixture = fixture_make(6u);
    if (fixture.memory == NULL) {
        return;
    }
    const sl_aabb static_box = aabb_make(0.0f, 0.0f, 1.0f, 1.0f);
    const sl_aabb kinematic_box = aabb_make(1.0f, 0.0f, 2.0f, 1.0f);
    const sl_aabb dynamic_box = aabb_make(2.0f, 0.0f, 3.0f, 1.0f);
    SL_EXPECT(sl_tree_proxy_create(&fixture.tree, SL_TREE_ROOT_STATIC,
                                   static_box, 10u) != SL_TREE_NODE_NONE);
    SL_EXPECT(sl_tree_proxy_create(&fixture.tree, SL_TREE_ROOT_KINEMATIC,
                                   kinematic_box, 20u) != SL_TREE_NODE_NONE);
    SL_EXPECT(sl_tree_proxy_create(&fixture.tree, SL_TREE_ROOT_DYNAMIC,
                                   dynamic_box, 30u) != SL_TREE_NODE_NONE);

    uint32_t users[6] = { 0u };
    sl_tree_query_result result =
        sl_tree_query(&fixture.tree, SL_TREE_ROOT_STATIC,
                      aabb_make(1.0f, 0.25f, 1.0f, 0.75f), users, 6u);
    SL_EXPECT_INT_EQ(result.count, 1u);
    SL_EXPECT_INT_EQ(users[0], 10u);
    SL_EXPECT(!result.overflow);

    result = sl_tree_query(&fixture.tree, SL_TREE_ROOT_KINEMATIC,
                           aabb_make(1.0f, 0.25f, 1.0f, 0.75f), users, 6u);
    SL_EXPECT_INT_EQ(result.count, 1u);
    SL_EXPECT_INT_EQ(users[0], 20u);

    result = sl_tree_query(&fixture.tree, SL_TREE_ROOT_DYNAMIC,
                           aabb_make(-10.0f, -10.0f, 10.0f, 10.0f), users, 6u);
    SL_EXPECT_INT_EQ(result.count, 1u);
    SL_EXPECT_INT_EQ(users[0], 30u);
    fixture_destroy(&fixture);
}

static void capacity_and_query_overflow_are_explicit(void)
{
    sl_tree_fixture fixture = fixture_make(4u);
    if (fixture.memory == NULL) {
        return;
    }
    const sl_aabb box = aabb_make(-1.0f, -1.0f, 1.0f, 1.0f);
    uint32_t proxies[4];
    for (uint32_t i = 0u; i < 4u; ++i) {
        proxies[i] = sl_tree_proxy_create(&fixture.tree, SL_TREE_ROOT_DYNAMIC,
                                          box, 100u + i);
        SL_EXPECT(proxies[i] != SL_TREE_NODE_NONE);
    }
    SL_EXPECT_INT_EQ(fixture.tree.proxy_count, 4u);
    SL_EXPECT_INT_EQ(fixture.tree.node_count, 7u);
    SL_EXPECT(sl_tree_proxy_create(&fixture.tree, SL_TREE_ROOT_DYNAMIC, box,
                                   999u) == SL_TREE_NODE_NONE);
    SL_EXPECT_INT_EQ(fixture.tree.proxy_count, 4u);

    uint32_t users[2] = { UINT32_MAX, UINT32_MAX };
    const sl_tree_query_result result =
        sl_tree_query(&fixture.tree, SL_TREE_ROOT_DYNAMIC, box, users, 2u);
    SL_EXPECT_INT_EQ(result.count, 4u);
    SL_EXPECT(result.overflow);
    SL_EXPECT(users[0] != UINT32_MAX && users[1] != UINT32_MAX);

    SL_EXPECT(
        !sl_tree_proxy_destroy(&fixture.tree, SL_TREE_ROOT_STATIC, proxies[0]));
    SL_EXPECT(
        sl_tree_proxy_destroy(&fixture.tree, SL_TREE_ROOT_DYNAMIC, proxies[0]));
    const uint32_t reused =
        sl_tree_proxy_create(&fixture.tree, SL_TREE_ROOT_DYNAMIC, box, 777u);
    SL_EXPECT_INT_EQ(reused, proxies[0]);
    fixture_destroy(&fixture);
}

static uint32_t prng_next(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    *state = x;
    return x;
}

static float prng_range(uint32_t *state, float lower, float upper)
{
    const float unit = (float)(prng_next(state) & 0xffffu) / 65535.0f;
    return lower + (upper - lower) * unit;
}

static sl_aabb random_aabb(uint32_t *state)
{
    const float x = prng_range(state, -25.0f, 25.0f);
    const float y = prng_range(state, -25.0f, 25.0f);
    const float width = prng_range(state, 0.05f, 4.0f);
    const float height = prng_range(state, 0.05f, 4.0f);
    return aabb_make(x, y, x + width, y + height);
}

static uint32_t active_model_at(const sl_proxy_model *models,
                                uint32_t active_index)
{
    for (uint32_t i = 0u; i < SL_TREE_TEST_CAPACITY; ++i) {
        if (models[i].active) {
            if (active_index == 0u) {
                return i;
            }
            active_index -= 1u;
        }
    }
    return UINT32_MAX;
}

static void expect_query_matches_model(sl_tree *tree,
                                       const sl_proxy_model *models,
                                       sl_tree_root_kind root, sl_aabb query)
{
    uint32_t users[SL_TREE_TEST_CAPACITY];
    const sl_tree_query_result result =
        sl_tree_query(tree, root, query, users, SL_TREE_TEST_CAPACITY);
    uint32_t expected_count = 0u;
    bool expected[SL_TREE_TEST_CAPACITY] = { false };
    for (uint32_t i = 0u; i < SL_TREE_TEST_CAPACITY; ++i) {
        if (models[i].active && models[i].root == root &&
            sl_aabb_overlaps(models[i].aabb, query)) {
            expected[i] = true;
            expected_count += 1u;
        }
    }
    SL_EXPECT_INT_EQ(result.count, expected_count);
    SL_EXPECT(!result.overflow);
    bool seen[SL_TREE_TEST_CAPACITY] = { false };
    for (uint32_t i = 0u; i < result.count; ++i) {
        SL_EXPECT(users[i] < SL_TREE_TEST_CAPACITY);
        if (users[i] < SL_TREE_TEST_CAPACITY) {
            SL_EXPECT(expected[users[i]]);
            SL_EXPECT(!seen[users[i]]);
            seen[users[i]] = true;
        }
    }
}

static void seeded_mutations_match_brute_force(void)
{
    sl_tree_fixture fixture = fixture_make(SL_TREE_TEST_CAPACITY);
    if (fixture.memory == NULL) {
        return;
    }
    sl_proxy_model models[SL_TREE_TEST_CAPACITY];
    memset(models, 0, sizeof(models));
    uint32_t state = 0x9e3779b9u;
    uint32_t active_count = 0u;

    for (uint32_t step = 0u; step < 2000u; ++step) {
        const uint32_t choice = prng_next(&state) % 100u;
        if (active_count == 0u ||
            (active_count < SL_TREE_TEST_CAPACITY && choice < 42u)) {
            uint32_t model_id = 0u;
            while (models[model_id].active) {
                model_id += 1u;
            }
            const sl_tree_root_kind root =
                (sl_tree_root_kind)(prng_next(&state) % SL_TREE_ROOT_COUNT);
            const sl_aabb aabb = random_aabb(&state);
            const uint32_t proxy =
                sl_tree_proxy_create(&fixture.tree, root, aabb, model_id);
            SL_EXPECT(proxy != SL_TREE_NODE_NONE);
            models[model_id].active = true;
            models[model_id].root = root;
            models[model_id].aabb = aabb;
            models[model_id].proxy = proxy;
            active_count += 1u;
        } else {
            const uint32_t active_index = prng_next(&state) % active_count;
            const uint32_t model_id = active_model_at(models, active_index);
            SL_EXPECT(model_id != UINT32_MAX);
            if (choice < 67u) {
                SL_EXPECT(sl_tree_proxy_destroy(&fixture.tree,
                                                models[model_id].root,
                                                models[model_id].proxy));
                models[model_id].active = false;
                active_count -= 1u;
            } else {
                const sl_aabb aabb = random_aabb(&state);
                SL_EXPECT(sl_tree_proxy_move(&fixture.tree,
                                             models[model_id].root,
                                             models[model_id].proxy, aabb));
                models[model_id].aabb = aabb;
            }
        }

        const sl_tree_root_kind query_root =
            (sl_tree_root_kind)(prng_next(&state) % SL_TREE_ROOT_COUNT);
        expect_query_matches_model(&fixture.tree, models, query_root,
                                   random_aabb(&state));
        SL_EXPECT_INT_EQ(fixture.tree.proxy_count, active_count);
    }
    fixture_destroy(&fixture);
}

static void identical_sequences_have_identical_order(void)
{
    sl_tree_fixture first = fixture_make(32u);
    sl_tree_fixture second = fixture_make(32u);
    if (first.memory == NULL || second.memory == NULL) {
        fixture_destroy(&first);
        fixture_destroy(&second);
        return;
    }

    uint32_t first_proxies[24];
    uint32_t second_proxies[24];
    uint32_t state = 0x243f6a88u;
    for (uint32_t i = 0u; i < 24u; ++i) {
        const sl_aabb box = random_aabb(&state);
        first_proxies[i] =
            sl_tree_proxy_create(&first.tree, SL_TREE_ROOT_DYNAMIC, box, i);
        second_proxies[i] =
            sl_tree_proxy_create(&second.tree, SL_TREE_ROOT_DYNAMIC, box, i);
        SL_EXPECT_INT_EQ(first_proxies[i], second_proxies[i]);
    }
    for (uint32_t i = 0u; i < 24u; i += 3u) {
        const sl_aabb box = random_aabb(&state);
        SL_EXPECT(sl_tree_proxy_move(&first.tree, SL_TREE_ROOT_DYNAMIC,
                                     first_proxies[i], box));
        SL_EXPECT(sl_tree_proxy_move(&second.tree, SL_TREE_ROOT_DYNAMIC,
                                     second_proxies[i], box));
    }
    for (uint32_t i = 1u; i < 24u; i += 4u) {
        SL_EXPECT(sl_tree_proxy_destroy(&first.tree, SL_TREE_ROOT_DYNAMIC,
                                        first_proxies[i]));
        SL_EXPECT(sl_tree_proxy_destroy(&second.tree, SL_TREE_ROOT_DYNAMIC,
                                        second_proxies[i]));
    }

    uint32_t first_users[32] = { 0u };
    uint32_t second_users[32] = { 0u };
    const sl_aabb all = aabb_make(-100.0f, -100.0f, 100.0f, 100.0f);
    const sl_tree_query_result first_result =
        sl_tree_query(&first.tree, SL_TREE_ROOT_DYNAMIC, all, first_users, 32u);
    const sl_tree_query_result second_result = sl_tree_query(
        &second.tree, SL_TREE_ROOT_DYNAMIC, all, second_users, 32u);
    SL_EXPECT_INT_EQ(first_result.count, second_result.count);
    SL_EXPECT(memcmp(first_users, second_users,
                     first_result.count * sizeof(uint32_t)) == 0);
    SL_EXPECT_INT_EQ(first.tree.node_count, second.tree.node_count);
    SL_EXPECT(memcmp(first.memory, second.memory, first.memory_bytes) == 0);
    fixture_destroy(&first);
    fixture_destroy(&second);
}

static void tied_costs_remain_balanced(void)
{
    const uint32_t capacity = 128u;
    sl_tree_fixture fixture = fixture_make(capacity);
    if (fixture.memory == NULL) {
        return;
    }
    const sl_aabb same = aabb_make(-1.0f, -1.0f, 1.0f, 1.0f);
    for (uint32_t i = 0u; i < capacity; ++i) {
        SL_EXPECT(sl_tree_proxy_create(&fixture.tree, SL_TREE_ROOT_DYNAMIC,
                                       same, i) != SL_TREE_NODE_NONE);
    }
    SL_EXPECT(sl_tree_height(&fixture.tree, SL_TREE_ROOT_DYNAMIC) <= 10u);
    uint32_t users[128];
    const sl_tree_query_result result = sl_tree_query(
        &fixture.tree, SL_TREE_ROOT_DYNAMIC, same, users, capacity);
    SL_EXPECT_INT_EQ(result.count, capacity);
    SL_EXPECT(!result.overflow);
    fixture_destroy(&fixture);
}

static const sl_test_case k_cases[] = {
    { "memory and init validation", memory_accounting_and_init_validation },
    { "root isolation and boundaries", roots_and_boundaries_are_isolated },
    { "capacity and query overflow", capacity_and_query_overflow_are_explicit },
    { "seeded mutations vs brute force", seeded_mutations_match_brute_force },
    { "deterministic traversal order",
      identical_sequences_have_identical_order },
    { "tied insertion balance", tied_costs_remain_balanced },
};

int sl_tree_suite(void)
{
    return sl_run_suite("tree", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
