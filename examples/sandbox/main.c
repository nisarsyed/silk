/* silk sandbox — interactive debug playground closing Milestone 1.
 *
 * World space == screen space: pixels, origin top-left, +y down.
 * Gravity is +y. 100 px = 1 m, so gravity is 981 px/s^2.
 *
 * Controls:
 *   left-drag on empty space   spawn a body, release to launch
 *   left-hold on a body        spring-tether it toward the cursor
 *   space                      pause / resume
 *   period                     execute one fixed step (while paused)
 *   r                          reset to the default scene
 *
 * Bodies are pure particles until collision lands in a later
 * milestone: they overlap freely and fall through the bottom edge,
 * where unattended ones despawn rather than accumulating invisibly.
 * Held or aimed bodies are exempt and can be reeled back on-screen.
 *
 * Timing policy lives entirely app-side: each frame feeds raylib's
 * frame time into sl_world_advance; pausing stops feeding time, and a
 * manual step calls sl_world_step directly. The engine never sees
 * wall-clock state. The xorshift32 seed below is committed to source
 * so every launch and every reset rebuilds the identical scene. */

#include <float.h>
#include <stdint.h>
#include <stdio.h>

#include <raylib.h>

#include <silk/math.h>
#include <silk/step.h>
#include <silk/world.h>

#define SB_SCREEN_WIDTH 1280
#define SB_SCREEN_HEIGHT 720
#define SB_BODY_CAPACITY 512u

/* Committed PRNG seed; identical default scene on every launch. */
#define SB_RNG_SEED 0x9E3779B9u

/* Gravity in screen space: 9.81 m/s^2 at 100 px per metre. */
static const sl_vec2 k_sb_gravity = { 0.0f, 981.0f };

static const float k_sb_timestep = 1.0f / 60.0f; /* seconds */

/* Mild damping settles tether oscillation; any drag >= 0 is stable
 * under the integrator's divide form (see include/silk/step.h). */
static const float k_sb_linear_drag = 0.1f; /* 1 / seconds */

/* Drawn body radius: 18 px per sqrt(kg); default masses span
 * [1, 5] kg, so circles span [18, 40] px. */
static const float k_sb_radius_per_sqrt_mass = 18.0f; /* px */

/* Tether spring, applied per rendered frame and topped up to exactly
 * one application's worth before the next advance consumes it (forces
 * clear per step). Top-up removes double-strength pulses when several
 * short frames bank onto one step; a long frame spanning multiple steps
 * still leaves its later steps unforced, since sl_world_advance has no
 * per-step hook — residual wobble, invisible at 60 Hz. Trail: at the
 * default-scene mean mass near 3 kg, a ~200 px offset yields ~1000
 * px/s^2, one gravity-equivalent, which tracks the cursor without
 * whipping at 60 Hz alongside the drag above. */
static const float k_sb_tether_gain = 15.0f; /* N per px of offset */

/* Slingshot: 6 px of drag buys 1 px/s of launch speed, capped so a
 * full-window fling stays well inside float-safe integration bounds. */
static const float k_sb_launch_gain = 6.0f;         /* (px/s) per px */
static const float k_sb_launch_speed_max = 1500.0f; /* px/s */

/* Extra grab distance beyond the drawn radius, for pointer slop. */
static const float k_sb_pick_slack = 12.0f; /* px */

typedef struct sb_app {
    sl_world world;
    sl_stepper stepper;

    sl_body_handle tether_body; /* null handle when not dragging */
    sl_body_handle sling_body;  /* spawned, awaiting release */
    bool sling_armed;
    sl_vec2 press_point; /* slingshot origin, px */

    sl_vec2 cursor;       /* mouse position, px */
    sl_vec2 tether_force; /* pre-consumption snapshot for the arrow */

    bool paused;
    uint32_t last_steps; /* steps executed by the most recent advance */
} sb_app;

static uint32_t sb_rng_state = SB_RNG_SEED;

static uint32_t sb_rng_next(void)
{
    uint32_t x = sb_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    sb_rng_state = x;
    return x;
}

/* Uniform in [0, 1): 24 random bits land exactly in float's mantissa. */
static float sb_rng_unit(void)
{
    return (float)(sb_rng_next() >> 8) * (1.0f / 16777216.0f);
}

static float sb_rng_range(float lo, float hi)
{
    return lo + (hi - lo) * sb_rng_unit();
}

static Vector2 sb_to_raylib(sl_vec2 v)
{
    Vector2 out = { v.x, v.y };
    return out;
}

static bool sb_handle_eq(sl_body_handle a, sl_body_handle b)
{
    return a.index == b.index && a.generation == b.generation;
}

static float sb_draw_radius(const sl_world *world, sl_body_handle body)
{
    return k_sb_radius_per_sqrt_mass *
           sqrtf(sl_world_body_get_mass(world, body));
}

static void sb_spawn_default_scene(sl_world *world)
{
    for (uint32_t i = 0; i < 32u; ++i) {
        sl_body_desc desc;
        desc.position.x = sb_rng_range(96.0f, 1184.0f);
        desc.position.y = sb_rng_range(64.0f, 320.0f);
        desc.velocity.x = sb_rng_range(-90.0f, 90.0f);
        desc.velocity.y = sb_rng_range(-30.0f, 60.0f);
        desc.mass = sb_rng_range(1.0f, 5.0f);
        const sl_body_handle body = sl_world_body_create(world, &desc);
        /* Capacity guarantees room for the whole scene. */
        SL_ASSERT(!sl_body_handle_is_null(body));
        (void)body;
    }
}

/* Reset is the only thing that invalidates handles in this app, so it
 * owns clearing interaction state to the null handle; the PRNG reseeds
 * so the rebuilt scene matches launch exactly. */
static void sb_reset(sb_app *app)
{
    sl_world_reset(&app->world);
    app->tether_body = sl_body_handle_null();
    app->sling_body = sl_body_handle_null();
    app->sling_armed = false;
    app->tether_force = sl_vec2_make(0.0f, 0.0f);
    sb_rng_state = SB_RNG_SEED;
    sb_spawn_default_scene(&app->world);
}

/* Nearest live body whose drawn radius (plus slack) covers the point;
 * null handle when nothing qualifies. */
static sl_body_handle sb_pick_body(const sl_world *world, sl_vec2 point)
{
    sl_body_handle best = sl_body_handle_null();
    float best_dist = FLT_MAX;

    for (sl_body_handle it = sl_world_body_first(world);
         !sl_body_handle_is_null(it); it = sl_world_body_next(world, it)) {
        const sl_vec2 pos = sl_world_body_get_position(world, it);
        const float dist = sl_vec2_distance(point, pos);
        const float reach = sb_draw_radius(world, it) + k_sb_pick_slack;
        if (dist <= reach && dist < best_dist) {
            best_dist = dist;
            best = it;
        }
    }
    return best;
}

static void sb_spawn_at_cursor(sb_app *app)
{
    /* Capacity is enforced client-side, but create() still tolerates a
     * full world: a null handle drops the spawn silently and the HUD
     * body count already reflects the miss. */
    sl_body_desc desc;
    desc.position = app->cursor;
    desc.velocity = sl_vec2_make(0.0f, 0.0f);
    desc.mass = sb_rng_range(1.0f, 5.0f);
    app->sling_body = sl_world_body_create(&app->world, &desc);
}

static void sb_release_slingshot(sb_app *app)
{
    if (sl_body_handle_is_null(app->sling_body)) {
        return;
    }
    sl_vec2 velocity = sl_vec2_scale(sl_vec2_sub(app->cursor, app->press_point),
                                     k_sb_launch_gain);
    const float speed = sl_vec2_length(velocity);
    if (speed > k_sb_launch_speed_max) {
        velocity =
            sl_vec2_scale(sl_vec2_normalize(velocity), k_sb_launch_speed_max);
    }
    /* Speed is clamped above, so rejection cannot happen. The call
     * stays outside SL_ASSERT: it carries the launch side effect. */
    const bool launched =
        sl_world_body_set_velocity(&app->world, app->sling_body, velocity);
    SL_ASSERT(launched);
    (void)launched;
    app->sling_body = sl_body_handle_null();
}

static void sb_handle_input(sb_app *app)
{
    const Vector2 mouse = GetMousePosition();
    app->cursor = sl_vec2_make(mouse.x, mouse.y);

    if (IsKeyPressed(KEY_SPACE)) {
        app->paused = !app->paused;
    }
    if (IsKeyPressed(KEY_R)) {
        sb_reset(app);
        return;
    }
    if (IsKeyPressed(KEY_PERIOD) && app->paused) {
        sl_world_step(&app->world, k_sb_timestep);
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        app->press_point = app->cursor;
        app->tether_body = sb_pick_body(&app->world, app->cursor);
        if (sl_body_handle_is_null(app->tether_body)) {
            app->sling_armed = true;
            sb_spawn_at_cursor(app);
        }
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        if (!sl_body_handle_is_null(app->tether_body)) {
            app->tether_body = sl_body_handle_null();
        } else if (app->sling_armed) {
            app->sling_armed = false;
            sb_release_slingshot(app);
        }
    }
}

/* Paused frames apply nothing: zero steps means zero consumption, so
 * holding the tether while paused cannot bank unbounded force. */
static void sb_apply_tether(sb_app *app)
{
    app->tether_force = sl_vec2_make(0.0f, 0.0f);
    if (app->paused || sl_body_handle_is_null(app->tether_body)) {
        return;
    }
    const sl_vec2 pos =
        sl_world_body_get_position(&app->world, app->tether_body);
    const sl_vec2 offset = sl_vec2_sub(app->cursor, pos);
    const sl_vec2 force = sl_vec2_scale(offset, k_sb_tether_gain);
    /* Top up rather than add: short frames can outnumber the steps an
     * advance executes, and plain addition would stack duplicate
     * applications onto the one step that finally fires. Differencing
     * against the accumulator lands every path on this exact force. */
    const sl_vec2 banked =
        sl_world_body_get_force(&app->world, app->tether_body);
    /* Delta stays displacement-bounded, so overflow rejection cannot
     * fire. The call stays outside SL_ASSERT: it carries the force
     * side effect. */
    const bool applied = sl_world_body_apply_force(
        &app->world, app->tether_body, sl_vec2_sub(force, banked));
    SL_ASSERT(applied);
    (void)applied;
    app->tether_force = force;
}

/* Despawn unattended bodies that fell out of view: with no collision
 * there is no floor, and off-screen bodies would silently exhaust the
 * spawn cap. Destroying mid-walk swaps another body into the hole, so
 * the successor is captured before removal (see include/silk/world.h).
 * Bodies under active interaction are exempt — a fast tethered body
 * may leave the view and be reeled back in — and releasing one
 * off-screen makes it unattended, so it despawns the next frame. */
static void sb_cull_fallen(sb_app *app)
{
    const float kill_y = (float)SB_SCREEN_HEIGHT + 64.0f; /* px */

    sl_body_handle it = sl_world_body_first(&app->world);
    while (!sl_body_handle_is_null(it)) {
        const sl_body_handle ahead = sl_world_body_next(&app->world, it);
        const sl_vec2 pos = sl_world_body_get_position(&app->world, it);
        const bool held = sb_handle_eq(it, app->tether_body) ||
                          sb_handle_eq(it, app->sling_body);
        if (pos.y > kill_y && !held) {
            sl_world_body_destroy(&app->world, it);
        }
        it = ahead;
    }
}

static void sb_draw_grid(void)
{
    for (int x = 0; x <= SB_SCREEN_WIDTH; x += 64) {
        DrawLine(x, 0, x, SB_SCREEN_HEIGHT, Fade(DARKGRAY, 0.25f));
    }
    for (int y = 0; y <= SB_SCREEN_HEIGHT; y += 64) {
        DrawLine(0, y, SB_SCREEN_WIDTH, y, Fade(DARKGRAY, 0.25f));
    }
}

static void sb_draw_bodies(const sb_app *app)
{
    const sl_world *world = &app->world;
    for (sl_body_handle it = sl_world_body_first(world);
         !sl_body_handle_is_null(it); it = sl_world_body_next(world, it)) {
        const sl_vec2 pos = sl_world_body_get_position(world, it);
        const sl_vec2 vel = sl_world_body_get_velocity(world, it);

        /* Velocity trace: one tenth of a second of travel. */
        const sl_vec2 trace = sl_vec2_add(pos, sl_vec2_scale(vel, 0.1f));
        DrawLineEx(sb_to_raylib(pos), sb_to_raylib(trace), 2.0f,
                   Fade(GREEN, 0.6f));

        const bool tethered = sb_handle_eq(it, app->tether_body);
        const Color fill = tethered ? SKYBLUE : ORANGE;
        DrawCircleV(sb_to_raylib(pos), sb_draw_radius(world, it), fill);
        /* Outline keeps overlapping circles readable — with no
         * collision, overlap is the common case. */
        DrawCircleLinesV(sb_to_raylib(pos), sb_draw_radius(world, it),
                         DARKBROWN);
    }
}

static void sb_draw_forces(const sb_app *app)
{
    const float magnitude = sl_vec2_length(app->tether_force);
    if (magnitude <= 0.0f || sl_body_handle_is_null(app->tether_body)) {
        return;
    }
    const sl_vec2 pos =
        sl_world_body_get_position(&app->world, app->tether_body);
    /* Arrow length: 0.03 px per N, capped so extreme tethers stay
     * readable. */
    const float length = sl_min(magnitude * 0.03f, 140.0f);
    const sl_vec2 tip = sl_vec2_add(
        pos, sl_vec2_scale(sl_vec2_normalize(app->tether_force), length));
    DrawLineEx(sb_to_raylib(pos), sb_to_raylib(tip), 3.0f, RED);
    DrawCircleV(sb_to_raylib(tip), 4.0f, RED);
}

static void sb_draw_interactions(const sb_app *app)
{
    if (app->sling_armed) {
        DrawLineEx(sb_to_raylib(app->press_point), sb_to_raylib(app->cursor),
                   2.0f, WHITE);
        DrawCircleV(sb_to_raylib(app->press_point), 4.0f, WHITE);
    }
    if (!sl_body_handle_is_null(app->tether_body)) {
        const sl_vec2 pos =
            sl_world_body_get_position(&app->world, app->tether_body);
        DrawLineEx(sb_to_raylib(pos), sb_to_raylib(app->cursor), 2.0f,
                   Fade(SKYBLUE, 0.8f));
    }
}

static void sb_draw_hud(const sb_app *app)
{
    const uint32_t count = sl_world_body_count(&app->world);
    const uint32_t capacity = sl_world_body_capacity(&app->world);

    DrawText(TextFormat("bodies %u/%u", count, capacity), 12, 10, 20, LIME);
    DrawText(TextFormat("steps %u", app->last_steps), 12, 34, 20, LIME);
    DrawText(
        TextFormat("bank %.1f ms", (double)app->stepper.remainder * 1000.0), 12,
        58, 20, LIME);
    DrawText(TextFormat("%d fps", GetFPS()), 12, 82, 20, LIME);

    if (app->paused) {
        DrawText("PAUSED", SB_SCREEN_WIDTH - 130, 10, 20, RED);
    }

    DrawText("drag: spawn + slingshot   hold body: tether   space: pause   "
             "period: step   r: reset",
             12, SB_SCREEN_HEIGHT - 28, 16, GRAY);
}

static void sb_draw(const sb_app *app)
{
    BeginDrawing();
    ClearBackground(BLACK);
    sb_draw_grid();
    sb_draw_interactions(app);
    sb_draw_bodies(app);
    sb_draw_forces(app);
    sb_draw_hud(app);
    EndDrawing();
}

int main(void)
{
    SetConfigFlags(FLAG_VSYNC_HINT);
    InitWindow(SB_SCREEN_WIDTH, SB_SCREEN_HEIGHT, "silk sandbox");
    SetTargetFPS(60);

    sb_app app = { 0 };

    sl_world_config config;
    config.body_capacity = SB_BODY_CAPACITY;
    config.gravity = k_sb_gravity;
    config.linear_drag = k_sb_linear_drag;
    if (!sl_world_init(&app.world, &config)) {
        fprintf(stderr, "sandbox: world init failed\n");
        CloseWindow();
        return 1;
    }
    if (!sl_stepper_init(&app.stepper, k_sb_timestep)) {
        fprintf(stderr, "sandbox: stepper init failed\n");
        sl_world_destroy(&app.world);
        CloseWindow();
        return 1;
    }

    sb_reset(&app);

    while (!WindowShouldClose()) {
        sb_handle_input(&app);
        sb_apply_tether(&app);
        if (!app.paused) {
            const float frame_time = GetFrameTime();
            app.last_steps =
                sl_world_advance(&app.world, &app.stepper, frame_time);
        }
        sb_cull_fallen(&app);
        sb_draw(&app);
    }

    sl_world_destroy(&app.world);
    CloseWindow();
    return 0;
}
