/* silk sandbox -- interactive debug playground.
 *
 * World space == screen space: pixels, origin top-left, +y down.
 * Gravity is +y. 100 px = 1 m, so gravity is 981 px/s^2. In this frame
 * a math-CCW polygon draws clockwise and positive angles turn clockwise
 * on screen; the engine is frame-agnostic, and sl_shape_make_box
 * produces canonical CCW input so hand-ordered points are never needed.
 * raylib culls back faces and calls counter-clockwise front, so filled
 * polygons are emitted in reverse -- see sb_polygon_points.
 *
 * Controls:
 *   left-drag on empty space   spawn a body, release to launch
 *   b                          toggle spawn kind (circle / box)
 *   left-hold on a body        tether it toward the cursor at the grab
 *                              point (off-center grabs spin it)
 *   space                      pause / resume
 *   period                     execute one fixed step (while paused)
 *   r                          reset to the default scene
 *
 * Bodies still overlap freely and fall through everything until Phase 3
 * collision lands: the static ground previews where a floor will be,
 * and unattended bodies despawn below the view rather than piling up
 * invisibly. Tethered bodies are exempt and can be reeled back
 * on-screen; slingshot bodies come into existence only at release.
 *
 * Timing policy lives entirely app-side: this file owns the accumulator
 * and calls sl_world_step directly, applying the tether immediately
 * before each step, so forces land on every step of a multi-step frame
 * and nothing banks between frames. Pausing stops feeding time; a
 * manual step applies the tether once and steps once. The engine never
 * sees wall-clock state. The xorshift32 seed below is committed to
 * source so every launch and every reset rebuilds the identical
 * scene. */

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <raylib.h>

#include <silk/math.h>
#include <silk/shape.h>
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

/* Mild damping settles both tether oscillation and free spin
 * (amplitude e-folds in 1/drag ~= 10 s); any drag >= 0 is stable under
 * the integrator's divide form (see include/silk/step.h). */
static const float k_sb_linear_drag = 0.1f;  /* 1 / seconds */
static const float k_sb_angular_drag = 0.1f; /* 1 / seconds */

/* Areal density: 1 kg of anything spreads over the area of an 18 px
 * circle, preserving the historical visual scale across shapes --
 * rho = 1 / (pi * 18^2) kg/px^2. Circle radius r = sqrt(m/(rho*pi)),
 * box side s = sqrt(m/rho): equal areas, s ~= 1.772 * r. Default
 * masses span [1, 5] kg: circles span [18, 40] px, boxes [32, 71]. */
static const float k_sb_areal_density =
    1.0f / (SL_PI * 18.0f * 18.0f); /* kilograms / px^2 */

static float sb_spawn_radius(float mass)
{
    return sqrtf(mass / (SL_PI * k_sb_areal_density));
}

static float sb_spawn_half_side(float mass)
{
    return 0.5f * sqrtf(mass / k_sb_areal_density);
}

/* Tether spring, applied once per executed step: at the default-scene
 * mean mass near 3 kg, a ~200 px offset yields ~1000 px/s^2, one
 * gravity-equivalent, which tracks the cursor without whipping at
 * 60 Hz alongside the drag above. Applied through
 * sl_world_body_apply_force_at_point at the stored grab point, so
 * off-center holds visibly torque the body. */
static const float k_sb_tether_gain = 15.0f; /* N per px of offset */

/* Slingshot: 1 px of drag buys 6 px/s of launch speed, capped so a
 * full-window fling stays well inside float-safe integration bounds. */
static const float k_sb_launch_gain = 6.0f;         /* (px/s) per px */
static const float k_sb_launch_speed_max = 1500.0f; /* px/s */

/* Grab radius around the body origin, for probes the shape test cannot
 * catch: shapeless point particles, and pointer slop on a body smaller
 * than the cursor is precise. */
static const float k_sb_pick_slack = 12.0f; /* px */

typedef enum sb_spawn_kind { SB_SPAWN_CIRCLE = 0, SB_SPAWN_BOX } sb_spawn_kind;

typedef struct sb_app {
    sl_world world;
    /* Unstepped frame time, seconds. The sandbox owns its accumulator
     * outright rather than carrying an sl_stepper it only half uses --
     * see sb_advance_frame. */
    float bank;

    sl_body_handle tether_body; /* null handle when not dragging */
    sl_vec2 grab_local;         /* grab offset in body-local coordinates,
                                 * frozen at press so the pull follows
                                 * the rotating body */
    bool sling_armed;
    sl_vec2 press_point; /* slingshot origin, px */
    float sling_mass;    /* kilograms, rolled at press so the ghost
                          * preview draws the real launch size */

    sb_spawn_kind spawn_kind;

    sl_vec2 cursor;       /* mouse position, px */
    sl_vec2 tether_force; /* most recent applied force, for the arrow */

    bool paused;
    uint32_t last_steps; /* steps executed by the most recent frame or
                          * manual single-step */
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

/* Fills *out with the selected kind sized to the mass under the shared
 * areal density; constructors cannot reject at these scales. */
static void sb_make_spawn_shape(sb_spawn_kind kind, float mass, sl_shape *out)
{
    bool made;
    if (kind == SB_SPAWN_CIRCLE) {
        made = sl_shape_make_circle(sb_spawn_radius(mass), out);
    } else {
        const float half = sb_spawn_half_side(mass);
        made = sl_shape_make_box(half, half, out);
    }
    /* The call stays outside SL_ASSERT because it carries the side
     * effect: SL_ASSERT compiles out under NDEBUG, and a release build
     * would hand *out to the world uninitialized. */
    SL_ASSERT(made);
    (void)made;
}

static void sb_spawn_default_scene(sl_world *world)
{
    for (uint32_t i = 0u; i < 32u; ++i) {
        sl_body_desc desc = { 0 };
        desc.position.x = sb_rng_range(96.0f, 1184.0f);
        desc.position.y = sb_rng_range(64.0f, 320.0f);
        desc.velocity.x = sb_rng_range(-90.0f, 90.0f);
        desc.velocity.y = sb_rng_range(-30.0f, 60.0f);
        desc.mass = sb_rng_range(1.0f, 5.0f);
        desc.angle = sb_rng_range(-SL_PI, SL_PI);
        desc.angular_velocity = sb_rng_range(-3.0f, 3.0f);

        sl_shape shape = sl_shape_none();
        sb_make_spawn_shape((i % 2u == 0u) ? SB_SPAWN_CIRCLE : SB_SPAWN_BOX,
                            desc.mass, &shape);
        desc.shape = &shape;

        const sl_body_handle body = sl_world_body_create(world, &desc);
        /* Capacity guarantees room for the whole scene. */
        SL_ASSERT(!sl_body_handle_is_null(body));
        (void)body;
    }
}

static void sb_spawn_ground(sl_world *world)
{
    /* Static preview floor: spans the window, top edge at y = 680, so
     * falling bodies visibly pass through it until collision lands. */
    sl_shape slab = sl_shape_none();
    const bool made = sl_shape_make_box(640.0f, 20.0f, &slab);
    SL_ASSERT(made);
    (void)made;

    sl_body_desc desc = { 0 };
    desc.position = sl_vec2_make(640.0f, 700.0f);
    desc.mass = 0.0f;
    desc.type = SL_BODY_STATIC;
    desc.shape = &slab;

    const sl_body_handle body = sl_world_body_create(world, &desc);
    SL_ASSERT(!sl_body_handle_is_null(body));
    (void)body;
}

/* Reset is the only thing that invalidates handles in this app, so it
 * owns clearing interaction state to the null handle; the PRNG reseeds
 * so the rebuilt scene matches launch exactly. */
static void sb_reset(sb_app *app)
{
    sl_world_reset(&app->world);
    app->tether_body = sl_body_handle_null();
    app->grab_local = sl_vec2_make(0.0f, 0.0f);
    app->sling_armed = false;
    app->tether_force = sl_vec2_make(0.0f, 0.0f);
    sb_rng_state = SB_RNG_SEED;
    sb_spawn_ground(&app->world);
    sb_spawn_default_scene(&app->world);
}

/* Nearest dynamic body whose shape covers the point (or whose reach
 * plus slop does); null handle when nothing qualifies. Static and
 * kinematic bodies are untetherable by contract -- they reject forces,
 * and grabbing one would fight its own movement rules. */
static sl_body_handle sb_pick_body(const sb_app *app, sl_vec2 point)
{
    sl_body_handle best = sl_body_handle_null();
    float best_dist = FLT_MAX;

    for (sl_body_handle it = sl_world_body_first(&app->world);
         !sl_body_handle_is_null(it);
         it = sl_world_body_next(&app->world, it)) {
        if (sl_world_body_get_type(&app->world, it) != SL_BODY_DYNAMIC) {
            continue;
        }
        const sl_transform tf = sl_world_body_get_transform(&app->world, it);
        const sl_shape *shape = sl_world_body_get_shape(&app->world, it);
        const float dist = sl_vec2_distance(point, tf.position);
        /* The engine's shape test is what decides; the slop disc only
         * covers what it cannot. Comparing against the shape's reach
         * instead would make the test dead weight -- containment
         * already implies dist <= reach for a centroid-centered convex
         * shape, so OR-ing the two could never change the answer. */
        const bool covered = dist <= k_sb_pick_slack ||
                             sl_shape_contains_point(shape, tf, point);
        if (covered && dist < best_dist) {
            best_dist = dist;
            best = it;
        }
    }
    return best;
}

/* Launches the aimed body: spawned at the press point with the launch
 * velocity baked into the descriptor, one create call, so the launch
 * starts exactly where the aim line starts. Nothing exists between
 * press and release, so aiming integrates nothing and culls nothing. */
static void sb_launch_slingshot(sb_app *app)
{
    sl_vec2 velocity = sl_vec2_scale(sl_vec2_sub(app->cursor, app->press_point),
                                     k_sb_launch_gain);
    const float speed = sl_vec2_length(velocity);
    if (speed > k_sb_launch_speed_max) {
        velocity =
            sl_vec2_scale(sl_vec2_normalize(velocity), k_sb_launch_speed_max);
    }
    /* Speed is clamped above, so the velocity cannot be rejected. A
     * full world returns the null handle and drops the spawn silently;
     * the HUD body count already shows it. The result is intentionally
     * discarded. */
    sl_body_desc desc = { 0 };
    desc.position = app->press_point;
    desc.velocity = velocity;
    desc.mass = app->sling_mass;

    sl_shape shape = sl_shape_none();
    sb_make_spawn_shape(app->spawn_kind, desc.mass, &shape);
    desc.shape = &shape;
    sl_world_body_create(&app->world, &desc);
}

static void sb_apply_tether_for_step(sb_app *app);

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
    if (IsKeyPressed(KEY_B)) {
        app->spawn_kind = (app->spawn_kind == SB_SPAWN_CIRCLE)
                              ? SB_SPAWN_BOX
                              : SB_SPAWN_CIRCLE;
    }
    if (IsKeyPressed(KEY_PERIOD) && app->paused) {
        /* Manual step shows the tether acting: one application, one
         * consumed step, nothing left banked behind. */
        sb_apply_tether_for_step(app);
        sl_world_step(&app->world, k_sb_timestep);
        app->last_steps = 1u;
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        app->press_point = app->cursor;
        app->tether_body = sb_pick_body(app, app->cursor);
        if (!sl_body_handle_is_null(app->tether_body)) {
            const sl_transform tf =
                sl_world_body_get_transform(&app->world, app->tether_body);
            app->grab_local = sl_transform_apply_inverse(tf, app->cursor);
        } else {
            /* Mass rolls at press: the ghost preview then draws the
             * exact size the launched body will have. */
            app->sling_armed = true;
            app->sling_mass = sb_rng_range(1.0f, 5.0f);
        }
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        if (!sl_body_handle_is_null(app->tether_body)) {
            /* Nothing to unwind: applications are consumed by their own
             * step, so release just drops the grab. */
            app->tether_body = sl_body_handle_null();
            app->tether_force = sl_vec2_make(0.0f, 0.0f);
        } else if (app->sling_armed) {
            app->sling_armed = false;
            sb_launch_slingshot(app);
        }
    }
}

/* Applies the tether spring at the frozen grab point for exactly one
 * upcoming step. Called immediately before every sl_world_step, so a
 * frame spanning several steps forces and torques each of them and
 * nothing survives a pause or a release. */
static void sb_apply_tether_for_step(sb_app *app)
{
    app->tether_force = sl_vec2_make(0.0f, 0.0f);
    if (sl_body_handle_is_null(app->tether_body)) {
        return;
    }
    const sl_transform tf =
        sl_world_body_get_transform(&app->world, app->tether_body);
    const sl_vec2 grab_world = sl_transform_apply(tf, app->grab_local);
    const sl_vec2 offset = sl_vec2_sub(app->cursor, grab_world);
    const sl_vec2 force = sl_vec2_scale(offset, k_sb_tether_gain);
    /* The body's bounded center and extent keep grab_world inside the
     * force-point domain, while screen-scale displacement keeps both
     * accumulators finite. Preserve the false path because rejection is
     * still part of the API contract. */
    if (sl_world_body_apply_force_at_point(&app->world, app->tether_body, force,
                                           grab_world)) {
        app->tether_force = force;
    }
}

/* Runs the frame's fixed steps with the tether applied per step.
 * sl_world_advance cannot be used: it offers no per-step hook, and the
 * tether has to land on every step of a multi-step frame. So this owns
 * the accumulator itself, deliberately reproducing the engine's cap and
 * drop rule (include/silk/step.h) rather than holding an sl_stepper
 * whose timestep would go unread and silently drift from
 * k_sb_timestep. */
static void sb_advance_frame(sb_app *app)
{
    float available = app->bank;
    const float frame_time = GetFrameTime();
    if (sl_is_finite(frame_time) && frame_time > 0.0f) {
        available += frame_time;
    }

    uint32_t steps = 0u;
    while (available >= k_sb_timestep && steps < SL_STEP_COUNT_MAX) {
        sb_apply_tether_for_step(app);
        sl_world_step(&app->world, k_sb_timestep);
        available -= k_sb_timestep;
        steps++;
    }

    /* At the cap, drop the rest: a slow frame sheds debt instead of
     * stepping forever on the next one. */
    app->bank = (steps == SL_STEP_COUNT_MAX) ? 0.0f : available;
    app->last_steps = steps;
}

/* Despawn unattended bodies that fell out of view: with no collision
 * there is no floor yet, and off-screen bodies would silently exhaust
 * the spawn cap. Destroy is a swap-remove -- the relocated body lands
 * behind a captured-successor cursor -- so this walks rows with at()
 * instead, retesting whatever refills a removed row (see
 * include/silk/world.h). Tethered bodies are exempt -- a fast tethered
 * body may leave the view and be reeled back in; releasing one
 * off-screen makes it unattended, so it despawns the next frame. The
 * static ground sits above the kill line and never qualifies. */
static void sb_cull_fallen(sb_app *app)
{
    /* Margin below the view edge: bodies despawn only after fully
     * leaving sight. */
    const float kill_y = (float)SB_SCREEN_HEIGHT + 64.0f; /* px */

    for (uint32_t row = 0u; row < sl_world_body_count(&app->world);) {
        const sl_body_handle body = sl_world_body_at(&app->world, row);
        const sl_vec2 pos = sl_world_body_get_position(&app->world, body);
        const bool held = sb_handle_eq(body, app->tether_body);
        if (pos.y > kill_y && !held) {
            sl_world_body_destroy(&app->world, body);
        } else {
            row++;
        }
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

/* Transformed polygon points, emitted in reverse. silk winds CCW in
 * math orientation, which is clockwise once the y-down screen frame
 * flips it -- and raylib enables backface culling with counter-
 * clockwise front faces at init, so submitting silk's order directly
 * leaves every fill back-facing and invisible. Reversing here puts the
 * loop the way raylib's own DrawRectanglePro emits one. The first
 * point is appended last so a line strip closes; that extra point
 * belongs to the strip only -- DrawTriangleFan must be passed count,
 * or its final triangle is the degenerate (p0, p[n-1], p0). */
static void sb_polygon_points(const sl_polygon *polygon, const sl_transform *tf,
                              Vector2 *out, uint32_t count_max)
{
    SL_ASSERT(polygon->count + 1u <= count_max);
    (void)count_max; /* the bound is a debug-only check */
    for (uint32_t i = 0u; i < polygon->count; ++i) {
        const uint32_t src = polygon->count - 1u - i;
        out[i] = sb_to_raylib(sl_transform_apply(*tf, polygon->vertices[src]));
    }
    out[polygon->count] = out[0];
}

static void sb_draw_bodies(const sb_app *app)
{
    const sl_world *world = &app->world;
    for (sl_body_handle it = sl_world_body_first(world);
         !sl_body_handle_is_null(it); it = sl_world_body_next(world, it)) {
        const sl_vec2 pos = sl_world_body_get_position(world, it);
        const sl_vec2 vel = sl_world_body_get_velocity(world, it);
        const sl_transform tf = sl_world_body_get_transform(world, it);
        const sl_shape *shape = sl_world_body_get_shape(world, it);

        /* Velocity trace: one tenth of a second of travel. */
        const sl_vec2 trace = sl_vec2_add(pos, sl_vec2_scale(vel, 0.1f));
        DrawLineEx(sb_to_raylib(pos), sb_to_raylib(trace), 2.0f,
                   Fade(GREEN, 0.6f));

        const bool tethered = sb_handle_eq(it, app->tether_body);
        Color fill = ORANGE;
        if (tethered) {
            fill = SKYBLUE;
        } else if (sl_world_body_get_type(world, it) == SL_BODY_STATIC) {
            fill = DARKGRAY;
        }

        if (shape->kind == SL_SHAPE_CIRCLE) {
            DrawCircleV(sb_to_raylib(pos), shape->circle.radius, fill);
            DrawCircleLinesV(sb_to_raylib(pos), shape->circle.radius,
                             DARKBROWN);
            /* Radius tick makes spin visible on a rotation-invariant
             * circle. */
            const sl_vec2 tip = sl_vec2_add(
                pos, sl_vec2_scale(sl_vec2_make(tf.rotation.c, tf.rotation.s),
                                   shape->circle.radius));
            DrawLineEx(sb_to_raylib(pos), sb_to_raylib(tip), 2.0f,
                       Fade(DARKBROWN, 0.8f));
        } else if (shape->kind == SL_SHAPE_POLYGON) {
            Vector2 points[SL_POLYGON_VERTEX_COUNT_MAX + 1u];
            sb_polygon_points(&shape->polygon, &tf, points,
                              SL_POLYGON_VERTEX_COUNT_MAX + 1u);
            DrawTriangleFan(points, (int)shape->polygon.count, fill);
            DrawLineStrip(points, (int)shape->polygon.count + 1, DARKBROWN);
        }

        /* Outline keeps overlapping bodies readable -- with no
         * collision, overlap is the common case. */
    }
}

static void sb_draw_forces(const sb_app *app)
{
    const float magnitude = sl_vec2_length(app->tether_force);
    if (magnitude <= 0.0f || sl_body_handle_is_null(app->tether_body)) {
        return;
    }
    const sl_transform tf =
        sl_world_body_get_transform(&app->world, app->tether_body);
    const sl_vec2 grab_world = sl_transform_apply(tf, app->grab_local);
    /* Arrow length: 0.03 px per N, capped so extreme tethers stay
     * readable. Drawn from the grab point, where the force lands. */
    const float length = sl_min(magnitude * 0.03f, 140.0f);
    const sl_vec2 tip = sl_vec2_add(
        grab_world,
        sl_vec2_scale(sl_vec2_normalize(app->tether_force), length));
    DrawLineEx(sb_to_raylib(grab_world), sb_to_raylib(tip), 3.0f, RED);
    DrawCircleV(sb_to_raylib(tip), 4.0f, RED);
    DrawCircleV(sb_to_raylib(grab_world), 3.0f, RED);
}

static void sb_draw_interactions(const sb_app *app)
{
    if (app->sling_armed) {
        DrawLineEx(sb_to_raylib(app->press_point), sb_to_raylib(app->cursor),
                   2.0f, WHITE);
        DrawCircleV(sb_to_raylib(app->press_point), 4.0f, WHITE);

        /* Ghost at the launched body's real size and kind: the mass
         * rolls at press, so what you see is what launches. */
        sl_shape ghost = sl_shape_none();
        sb_make_spawn_shape(app->spawn_kind, app->sling_mass, &ghost);
        if (ghost.kind == SL_SHAPE_CIRCLE) {
            DrawCircleLinesV(sb_to_raylib(app->press_point),
                             ghost.circle.radius, GRAY);
        } else {
            Vector2 points[SL_POLYGON_VERTEX_COUNT_MAX + 1u];
            const sl_transform at_press =
                sl_transform_make(app->press_point, sl_rotation_identity());
            sb_polygon_points(&ghost.polygon, &at_press, points,
                              SL_POLYGON_VERTEX_COUNT_MAX + 1u);
            DrawLineStrip(points, (int)ghost.polygon.count + 1, GRAY);
        }
    }
    if (!sl_body_handle_is_null(app->tether_body)) {
        const sl_transform tf =
            sl_world_body_get_transform(&app->world, app->tether_body);
        const sl_vec2 grab_world = sl_transform_apply(tf, app->grab_local);
        DrawLineEx(sb_to_raylib(grab_world), sb_to_raylib(app->cursor), 2.0f,
                   Fade(SKYBLUE, 0.8f));
    }
}

static void sb_draw_hud(const sb_app *app)
{
    const uint32_t count = sl_world_body_count(&app->world);
    const uint32_t capacity = sl_world_body_capacity(&app->world);

    DrawText(TextFormat("bodies %u/%u", count, capacity), 12, 10, 20, LIME);
    DrawText(TextFormat("steps %u", app->last_steps), 12, 34, 20, LIME);
    DrawText(TextFormat("bank %.1f ms", (double)app->bank * 1000.0), 12, 58, 20,
             LIME);
    DrawText(TextFormat("%d fps", GetFPS()), 12, 82, 20, LIME);
    DrawText(TextFormat("spawn: %s",
                        app->spawn_kind == SB_SPAWN_CIRCLE ? "circle" : "box"),
             12, 106, 20, LIME);

    if (!sl_body_handle_is_null(app->tether_body)) {
        const float degrees =
            sl_world_body_get_angle(&app->world, app->tether_body) *
            (180.0f / SL_PI);
        DrawText(TextFormat("angle %.0f deg", (double)degrees),
                 SB_SCREEN_WIDTH - 170, 10, 20, SKYBLUE);
    }

    if (app->paused) {
        DrawText("PAUSED", SB_SCREEN_WIDTH - 130, 40, 20, RED);
    }

    DrawText("drag: spawn + slingshot   b: shape   hold body: tether   "
             "space: pause   period: step   r: reset",
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
    config.angular_drag = k_sb_angular_drag;
    if (!sl_world_init(&app.world, &config)) {
        fprintf(stderr, "sandbox: world init failed\n");
        CloseWindow();
        return 1;
    }

    sb_reset(&app);

    while (!WindowShouldClose()) {
        sb_handle_input(&app);
        if (!app.paused) {
            sb_advance_frame(&app);
        }
        sb_cull_fallen(&app);
        sb_draw(&app);
    }

    sl_world_destroy(&app.world);
    CloseWindow();
    return 0;
}
