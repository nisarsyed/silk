#include "../bench/fixtures.h"
#include "../bench/render/driver.h"
#include "replay.h"
#include "silk_test.h"
#include "suites.h"
#include <math.h>

static sl_body_handle dynamic_at(const sl_world *world, uint32_t wanted)
{
    for (uint32_t row = 0u; row < sl_world_body_count(world); ++row) {
        const sl_body_handle body = sl_world_body_at(world, row);
        if (sl_world_body_get_type(world, body) == SL_BODY_DYNAMIC) {
            if (wanted == 0u) {
                return body;
            }
            --wanted;
        }
    }
    return sl_body_handle_null();
}

static void replay_manual_force(void)
{
    const uint32_t fixtures[] = { 0u, 1u, 3u };
    for (uint32_t f = 0u; f < 3u; ++f) {
        for (uint32_t sleeping = 0u; sleeping < 2u; ++sleeping) {
            sl_render_study *actual =
                sl_render_study_create(fixtures[f], 1u, sleeping, 12u);
            sl_render_study *expected =
                sl_render_study_create(fixtures[f], 1u, sleeping, 12u);
            SL_EXPECT(actual != NULL && expected != NULL);
            if (actual == NULL || expected == NULL) {
                sl_render_study_destroy(actual);
                sl_render_study_destroy(expected);
                continue;
            }
            sl_world *a = sl_render_study_world(actual);
            sl_world *b = sl_render_study_world(expected);
            const sl_body_handle body = dynamic_at(a, 0u);
            const sl_vec2 position = sl_world_body_get_position(a, body);
            const sl_vec2 press = { position.x + 0.02f, position.y };
            const sl_vec2 local = sl_transform_apply_inverse(
                sl_world_body_get_transform(b, body), press);
            SL_EXPECT(sl_render_study_pointer(actual, 0u, press.x, press.y));
            SL_EXPECT_INT_EQ(sl_render_study_pointer_status(actual)[1],
                             body.index);
            SL_EXPECT_INT_EQ(sl_render_study_pointer_status(actual)[2],
                             body.generation);
            SL_EXPECT(sl_world_body_wake(b, body));
            // Events do not accumulate force; repeated/paused rendering is
            // inert.
            SL_EXPECT(sl_vec2_length_sq(sl_world_body_get_force(a, body)) ==
                      0.0f);
            for (uint32_t step = 0u; step < 12u; ++step) {
                if (step < 8u) {
                    const sl_vec2 target = {
                        press.x + 0.1f * (float)(step + 1u), press.y + 0.5f
                    };
                    SL_EXPECT(sl_render_study_pointer(actual, 1u, target.x,
                                                      target.y));
                    const sl_vec2 point = sl_transform_apply(
                        sl_world_body_get_transform(b, body), local);
                    const sl_vec2 force =
                        sl_vec2_scale(sl_vec2_sub(target, point), 15.0f);
                    SL_EXPECT(sl_world_body_apply_force_at_point(b, body, force,
                                                                 point));
                } else {
                    SL_EXPECT(sl_render_study_pointer(
                        actual, step == 8u ? 2u : 3u, press.x, press.y));
                    SL_EXPECT_INT_EQ(sl_render_study_pointer_status(actual)[0],
                                     0u);
                    SL_EXPECT_INT_EQ(sl_render_study_pointer_status(actual)[1],
                                     UINT32_MAX);
                }
                SL_EXPECT(sl_render_study_step(actual));
                SL_EXPECT(sl_render_study_step(expected));
                SL_EXPECT(sl_replay_check(
                    a, b, "pointer tether versus explicit public force",
                    SL_BENCH_RAIN_SEED, step));
                SL_EXPECT(sl_vec2_length_sq(sl_world_body_get_force(a, body)) ==
                          0.0f);
            }
            SL_EXPECT(!sl_render_study_pointer(actual, 0u, press.x, press.y));
            sl_render_study_destroy(actual);
            sl_render_study_destroy(expected);
        }
    }
}

static void invalid_events_preserve_state(void)
{
    SL_EXPECT(!sl_render_study_pointer(NULL, 0u, 0.0f, 0.0f));
    SL_EXPECT(sl_render_study_pointer_status(NULL) == NULL);
    sl_render_study *study = sl_render_study_create(0u, 1u, 0u, 10u);
    sl_render_study *twin = sl_render_study_create(0u, 1u, 0u, 10u);
    SL_EXPECT(study != NULL && twin != NULL);
    if (study == NULL || twin == NULL) {
        sl_render_study_destroy(study);
        sl_render_study_destroy(twin);
        return;
    }
    const uint32_t *state = sl_render_study_pointer_status(study);
    SL_EXPECT_INT_EQ(state[0], 0u);
    SL_EXPECT_INT_EQ(state[1], UINT32_MAX);
    SL_EXPECT(!sl_render_study_pointer(study, 1u, 0.0f, 0.0f));
    SL_EXPECT(!sl_render_study_pointer(study, 4u, 0.0f, 0.0f));
    SL_EXPECT(!sl_render_study_pointer(study, UINT32_MAX, 0.0f, 0.0f));
    const float bad[] = { NAN, INFINITY, -INFINITY, SL_POSITION_ABS_MAX + 1.0f,
                          -SL_POSITION_ABS_MAX - 1.0f };
    for (uint32_t i = 0u; i < 5u; ++i) {
        SL_EXPECT(!sl_render_study_pointer(study, 0u, bad[i], 0.0f));
        SL_EXPECT(!sl_render_study_pointer(study, 0u, 0.0f, bad[i]));
    }
    // A miss is a held gesture, but never creates a body or applies a force.
    SL_EXPECT(sl_render_study_pointer(study, 0u, SL_POSITION_ABS_MAX,
                                      SL_POSITION_ABS_MAX));
    SL_EXPECT_INT_EQ(state[0], 1u);
    SL_EXPECT_INT_EQ(state[1], UINT32_MAX);
    SL_EXPECT(!sl_render_study_pointer(study, 0u, 0.0f, 0.0f));
    SL_EXPECT(!sl_render_study_pointer(study, 2u, NAN, 0.0f));
    SL_EXPECT_INT_EQ(state[0], 1u);
    SL_EXPECT(sl_render_study_pointer(study, 1u, 0.0f, 0.0f));
    SL_EXPECT(sl_render_study_step(study));
    SL_EXPECT(sl_render_study_step(twin));
    SL_EXPECT(sl_replay_check(sl_render_study_world(study),
                              sl_render_study_world(twin),
                              "pointer miss/invalid events", 0u, 1u));
    SL_EXPECT(sl_render_study_pointer(study, 3u, 0.0f, 0.0f));
    SL_EXPECT(sl_render_study_pointer(study, 3u, 0.0f, 0.0f));
    SL_EXPECT_INT_EQ(state[0], 0u);
    sl_render_study_destroy(twin);
    sl_render_study_destroy(study);
    study = sl_render_study_create(0u, 2u, 0u, 1u);
    SL_EXPECT(study != NULL);
    if (study != NULL) {
        SL_EXPECT(!sl_render_study_pointer(study, 0u, 0.0f, 0.0f));
        sl_render_study_destroy(study);
    }
}

static void nearest_ties_and_stale_selection(void)
{
    sl_render_study *study = sl_render_study_create(0u, 1u, 0u, 2u);
    SL_EXPECT(study != NULL);
    if (study == NULL) {
        return;
    }
    sl_world *world = sl_render_study_world(study);
    const sl_body_handle a = dynamic_at(world, 0u), b = dynamic_at(world, 1u);
    const sl_vec2 point = { 100.0f, 100.0f };
    SL_EXPECT(a.index < b.index);
    SL_EXPECT(sl_world_body_set_position(world, a, point));
    SL_EXPECT(sl_world_body_set_position(world, b, point));
    SL_EXPECT(sl_render_study_pointer(study, 0u, point.x, point.y));
    const uint32_t *state = sl_render_study_pointer_status(study);
    SL_EXPECT_INT_EQ(state[1], a.index);
    SL_EXPECT(sl_render_study_pointer(study, 2u, point.x, point.y));
    SL_EXPECT(
        sl_world_body_set_position(world, b, (sl_vec2){ 100.05f, 100.0f }));
    SL_EXPECT(sl_render_study_pointer(study, 0u, 100.04f, 100.0f));
    SL_EXPECT_INT_EQ(state[1], b.index);
    sl_world_body_destroy(world, b);
    SL_EXPECT(!sl_world_body_is_valid(world, b));
    SL_EXPECT(sl_render_study_step(study));
    SL_EXPECT_INT_EQ(state[0], 0u);
    SL_EXPECT_INT_EQ(state[1], UINT32_MAX);
    SL_EXPECT_INT_EQ(state[2], 0u);
    sl_render_study_destroy(study);
}

static void force_rejection_stops_before_step(void)
{
    sl_render_study *study = sl_render_study_create(0u, 1u, 0u, 2u);
    SL_EXPECT(study != NULL);
    if (study == NULL) {
        return;
    }
    sl_world *world = sl_render_study_world(study);
    const sl_body_handle body = dynamic_at(world, 0u);
    const sl_vec2 position = sl_world_body_get_position(world, body);
    // Valid finite inverse mass/inertia, but the requested acceleration would
    // overflow. This injects an input rejection without stepping corrupt state.
    SL_EXPECT(sl_world_body_set_mass(world, body, 1e-35f));
    SL_EXPECT(sl_render_study_pointer(study, 0u, position.x, position.y));
    SL_EXPECT(sl_render_study_pointer(study, 1u, SL_POSITION_ABS_MAX,
                                      SL_POSITION_ABS_MAX));
    SL_EXPECT(!sl_render_study_step(study));
    SL_EXPECT(sl_render_study_failed(study));
    SL_EXPECT_INT_EQ(sl_render_study_status(study)[4], 0u);
    const sl_vec2 unchanged = sl_world_body_get_position(world, body);
    SL_EXPECT(unchanged.x == position.x && unchanged.y == position.y);
    SL_EXPECT(sl_vec2_length_sq(sl_world_body_get_force(world, body)) == 0.0f);
    SL_EXPECT(!sl_render_study_pointer(study, 3u, 0.0f, 0.0f));
    SL_EXPECT(!sl_render_study_step(study));
    sl_render_study_destroy(study);
}

static const sl_test_case k_cases[] = {
    { "force rejection is latched before simulation mutation",
      force_rejection_stops_before_step },
    { "tether matches public force/torque once per step in all default "
      "fixtures",
      replay_manual_force },
    { "invalid events and misses preserve physics",
      invalid_events_preserve_state },
    { "pick nearest center, deterministic ties and clear stale selection",
      nearest_ties_and_stale_selection }
};
int sl_render_pointer_suite(void)
{
    return sl_run_suite("render pointer driver", k_cases,
                        (int)(sizeof(k_cases) / sizeof(k_cases[0])));
}
