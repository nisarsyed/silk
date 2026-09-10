/* silk sandbox -- interactive debug playground.
 *
 * Simulation lengths are metres; rendering scales them at 100 px/m.
 * The simulation and screen both use origin top-left and +y down, so
 * gravity is +9.81 m/s^2. In this frame a math-CCW polygon draws clockwise
 * and positive angles turn clockwise on screen; the engine is frame-
 * agnostic, and sl_shape_make_box produces canonical CCW input so hand-
 * ordered points are never needed. raylib culls back faces and calls
 * counter-clockwise front, so filled polygons are emitted in reverse --
 * see sb_polygon_points.
 *
 * Controls:
 *   left-drag on empty space   spawn a body, release to launch
 *   b                          toggle spawn kind (circle / box)
 *   left-hold on a body        tether it toward the cursor at the grab
 *                              point (off-center grabs spin it)
 *   space                      pause / resume
 *   period                     execute one fixed step (while paused)
 *   c                          toggle contact points and normals
 *   a                          toggle tight/fat AABBs
 *   1-4                        select deterministic scene
 *   s                          toggle sleep policy and rebuild scene
 *   i                          toggle last-build island colors
 *   m                          start/stop the moving support
 *   q                          cycle point/ray/AABB query
 *   right-drag                 define query; release selects first hit
 *   j                          spawn a six-link revolute chain at cursor
 *   r                          rebuild the selected scene
 *
 * Shaped bodies collide with the static ground and one another.
 * The off-screen cull remains a safety net for objects that tunnel at high
 * speed before continuous collision detection lands. Tethered bodies are
 * exempt and can be reeled back on-screen; slingshot bodies come into
 * existence only at release.
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
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <raylib.h>

#include <silk/math.h>
#include <silk/query.h>
#include <silk/shape.h>
#include <silk/step.h>
#include <silk/world.h>

#define SB_SCREEN_WIDTH 1280
#define SB_SCREEN_HEIGHT 720
#define SB_BODY_CAPACITY 512u
#define SB_CONTACT_CAPACITY 2048u
#define SB_JOINT_CAPACITY 64u
#define SB_SUBSTEP_COUNT 4u
#define SB_PYRAMID_ROW_COUNT 10u
#define SB_RAIN_BODY_COUNT 24u
#define SB_CHAIN_LINK_COUNT 6u

/* Committed PRNG seed; identical default scene on every launch. */
#define SB_RNG_SEED 0x9E3779B9u

/* Simulation values stay near unity for float resolution and future
 * contact tolerances. Screen-defined lengths cross this scale at the
 * input, rendering, and window-layout boundaries. */
static const float k_sb_pixels_per_metre = 100.0f;
static const sl_vec2 k_sb_gravity = { 0.0f, 9.81f }; /* metres / second^2 */

static const float k_sb_timestep = 1.0f / 60.0f; /* seconds */
static const float k_sb_friction = 0.6f;
static const float k_sb_restitution = 0.1f;

/* Mild damping settles both tether oscillation and free spin
 * (amplitude e-folds in 1/drag ~= 10 s); any drag >= 0 is stable under
 * the integrator's divide form (see include/silk/step.h). */
static const float k_sb_linear_drag = 0.1f;  /* 1 / seconds */
static const float k_sb_angular_drag = 0.1f; /* 1 / seconds */

/* Areal density: 1 kg of anything spreads over the area of a 0.18 m
 * circle, preserving the historical visual scale across shapes --
 * rho = 1 / (pi * 0.18^2) kg/m^2. Circle radius r = sqrt(m/(rho*pi)),
 * box side s = sqrt(m/rho): equal areas, s ~= 1.772 * r. Default
 * masses span [1, 5] kg: circle radii span [0.18, 0.40] m and box sides
 * span [0.32, 0.71] m. */
static const float k_sb_areal_density =
    1.0f / (SL_PI * 0.18f * 0.18f); /* kilograms / metre^2 */

static float sb_spawn_radius(float mass)
{
    return sqrtf(mass / (SL_PI * k_sb_areal_density));
}

static float sb_spawn_half_side(float mass)
{
    return 0.5f * sqrtf(mass / k_sb_areal_density);
}

/* Tether spring, applied once per executed step: at the default-scene
 * mean mass near 3 kg, a ~2 m offset yields ~10 m/s^2, one gravity-
 * equivalent, which tracks the cursor without whipping at 60 Hz
 * alongside the drag above. Applied through
 * sl_world_body_apply_force_at_point at the stored grab point, so
 * off-center holds visibly torque the body. */
static const float k_sb_tether_gain = 15.0f; /* N per metre of offset */

/* Slingshot: 1 m of drag buys 6 m/s of launch speed. The 15 m/s cap
 * limits travel to 0.25 m per 60 Hz step, keeping fast launches easy
 * to follow in the debug view. */
static const float k_sb_launch_gain = 6.0f;       /* (m/s) per metre */
static const float k_sb_launch_speed_max = 15.0f; /* metres / second */

/* One-metre cells expose the simulation scale directly. */
static const float k_sb_grid_spacing = 1.0f; /* metres */

typedef enum sb_spawn_kind { SB_SPAWN_CIRCLE = 0, SB_SPAWN_BOX } sb_spawn_kind;

typedef struct sb_app {
    sl_world world;
    sl_world_memory_breakdown memory;
    bool failed;
    bool sleep_enabled;
    bool show_islands;
    uint32_t scene;
    uint32_t scene_step;
    sl_body_handle support;
    bool support_moving;
    sl_body_handle selected;
    sl_body_handle pick_buffer[SB_BODY_CAPACITY];
    uint32_t query_mode; /* 0 point, 1 ray, 2 AABB */
    sl_vec2 query_start;
    sl_vec2 query_end;
    bool query_dragging;
    bool query_defined;
    bool query_valid;
    sl_query_result query_result;
    sl_query_ray_result ray_result;
    sl_body_handle query_bodies[8];
    /* Unstepped frame time, seconds. The sandbox owns its accumulator
     * outright rather than carrying an sl_stepper it only half uses --
     * see sb_advance_frame. */
    float bank;

    sl_body_handle tether_body; /* null handle when not dragging */
    sl_vec2 grab_local;         /* grab offset in body-local coordinates,
                                 * frozen at press so the pull follows
                                 * the rotating body */
    bool sling_armed;
    sl_vec2 press_point; /* slingshot origin, metres */
    float sling_mass;    /* kilograms, rolled at press so the ghost
                          * preview draws the real launch size */

    sb_spawn_kind spawn_kind;

    sl_vec2 cursor;       /* mouse position, metres */
    sl_vec2 tether_force; /* most recent applied force, for the arrow */

    bool paused;
    bool show_contacts;
    bool show_aabbs;
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

static float sb_length_to_pixels(float length)
{
    return length * k_sb_pixels_per_metre;
}

static float sb_length_from_pixels(float length)
{
    return length / k_sb_pixels_per_metre;
}

static Vector2 sb_to_raylib(sl_vec2 v)
{
    Vector2 out = { sb_length_to_pixels(v.x), sb_length_to_pixels(v.y) };
    return out;
}

static sl_vec2 sb_from_raylib(Vector2 v)
{
    return sl_vec2_make(sb_length_from_pixels(v.x), sb_length_from_pixels(v.y));
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

static void sb_spawn_pyramid(sl_world *world)
{
    const float screen_width_metres =
        sb_length_from_pixels((float)SB_SCREEN_WIDTH);
    const float screen_height_metres =
        sb_length_from_pixels((float)SB_SCREEN_HEIGHT);
    const float half = 0.25f;
    const float spacing = 2.0f * half + 0.01f;
    const float ground_top = screen_height_metres - 0.4f;
    sl_shape box = sl_shape_none();
    const bool made = sl_shape_make_box(half, half, &box);
    SL_ASSERT(made);
    (void)made;

    for (uint32_t row = 0u; row < SB_PYRAMID_ROW_COUNT; ++row) {
        const uint32_t count = SB_PYRAMID_ROW_COUNT - row;
        const float y = ground_top - half - (float)row * spacing;
        for (uint32_t column = 0u; column < count; ++column) {
            const sl_body_desc desc = {
                .position = {
                    0.5f * screen_width_metres +
                        ((float)column - 0.5f * (float)(count - 1u)) * spacing,
                    y,
                },
                .mass = 1.0f,
                .shape = &box,
                .friction = k_sb_friction,
                .restitution = k_sb_restitution,
            };
            const sl_body_handle body = sl_world_body_create(world, &desc);
            SL_ASSERT(!sl_body_handle_is_null(body));
            (void)body;
        }
    }
}

static void sb_spawn_rain(sl_world *world)
{
    const float screen_width_metres =
        sb_length_from_pixels((float)SB_SCREEN_WIDTH);
    for (uint32_t i = 0u; i < SB_RAIN_BODY_COUNT; ++i) {
        const uint32_t column = i % 8u;
        const uint32_t row = i / 8u;
        const float mass = sb_rng_range(0.5f, 1.5f);
        sl_shape circle = sl_shape_none();
        sb_make_spawn_shape(SB_SPAWN_CIRCLE, mass, &circle);
        const sl_body_desc desc = {
            .position = {
                0.2f * screen_width_metres +
                    (float)column * (0.6f * screen_width_metres / 7.0f) +
                    sb_rng_range(-0.08f, 0.08f),
                0.3f + (float)row * 0.5f + sb_rng_range(-0.05f, 0.05f),
            },
            .velocity = { sb_rng_range(-0.25f, 0.25f),
                          sb_rng_range(0.0f, 0.3f) },
            .mass = mass,
            .angle = sb_rng_range(-SL_PI, SL_PI),
            .angular_velocity = sb_rng_range(-2.0f, 2.0f),
            .shape = &circle,
            .friction = k_sb_friction,
            .restitution = k_sb_restitution,
        };
        const sl_body_handle body = sl_world_body_create(world, &desc);
        SL_ASSERT(!sl_body_handle_is_null(body));
        (void)body;
    }
}

static void sb_spawn_ground(sl_world *world)
{
    const float screen_width_metres =
        sb_length_from_pixels((float)SB_SCREEN_WIDTH);
    const float screen_height_metres =
        sb_length_from_pixels((float)SB_SCREEN_HEIGHT);
    const float ground_half_width_metres = 0.5f * screen_width_metres;
    const float ground_half_height_metres = 0.2f;

    /* Static floor spans the window with its top edge 0.4 m above the
     * bottom. */
    sl_shape slab = sl_shape_none();
    const bool made = sl_shape_make_box(ground_half_width_metres,
                                        ground_half_height_metres, &slab);
    SL_ASSERT(made);
    (void)made;

    sl_body_desc desc = { 0 };
    desc.position =
        sl_vec2_make(ground_half_width_metres,
                     screen_height_metres - ground_half_height_metres);
    desc.mass = 0.0f;
    desc.type = SL_BODY_STATIC;
    desc.shape = &slab;
    desc.friction = k_sb_friction;
    desc.restitution = k_sb_restitution;

    const sl_body_handle body = sl_world_body_create(world, &desc);
    SL_ASSERT(!sl_body_handle_is_null(body));
    (void)body;
}

static void sb_spawn_joint_chain(sb_app *app, float link_angle);

static sl_body_handle sb_box(sb_app *app, sl_body_type type, float x, float y,
                             float hx, float hy)
{
    sl_shape shape = sl_shape_none();
    const bool made = sl_shape_make_box(hx, hy, &shape);
    SL_ASSERT(made);
    (void)made;
    const sl_body_desc desc = { .type = type,
                                .position = { x, y },
                                .mass = type == SL_BODY_DYNAMIC ? 1.0f : 0.0f,
                                .shape = &shape,
                                .friction = k_sb_friction };
    const sl_body_handle body = sl_world_body_create(&app->world, &desc);
    SL_ASSERT(!sl_body_handle_is_null(body));
    return body;
}

/* Rebuild with the selected initialization-only sleep policy. Clear all
 * handles before allocating: numerical handle collisions across lifetimes
 * are possible. Each fixture starts from the committed seed and fixed poses. */
static void sb_reset(sb_app *app)
{
    sl_world_destroy(&app->world);
    app->tether_body = sl_body_handle_null();
    app->selected = sl_body_handle_null();
    app->support = sl_body_handle_null();
    app->grab_local = sl_vec2_make(0.0f, 0.0f);
    app->sling_armed = false;
    app->tether_force = sl_vec2_make(0.0f, 0.0f);
    app->bank = 0.0f;
    app->scene_step = 0u;
    app->support_moving = false;
    app->query_defined = false;
    app->query_dragging = false;
    const sl_world_config config = {
        .body_capacity = SB_BODY_CAPACITY,
        .contact_capacity = SB_CONTACT_CAPACITY,
        .joint_capacity = SB_JOINT_CAPACITY,
        .gravity = k_sb_gravity,
        .linear_drag = k_sb_linear_drag,
        .angular_drag = k_sb_angular_drag,
        .substep_count = SB_SUBSTEP_COUNT,
        .sleep_enabled = app->sleep_enabled,
    };
    if (!sl_world_init(&app->world, &config) ||
        !sl_world_memory_breakdown_get(&config, &app->memory)) {
        fprintf(stderr, "sandbox: world initialization failed\n");
        app->failed = true;
        return;
    }
    sb_rng_state = SB_RNG_SEED;
    sb_spawn_ground(&app->world);
    if (app->scene == 0u) {
        sb_spawn_pyramid(&app->world);
        sb_spawn_rain(&app->world);
    } else if (app->scene == 1u) {
        for (uint32_t pile = 0u; pile < 4u; ++pile) {
            for (uint32_t row = 0u; row < 3u; ++row) {
                (void)sb_box(app, SL_BODY_DYNAMIC, 2.0f + 2.8f * (float)pile,
                             6.55f - 0.5f * (float)row, 0.25f, 0.25f);
            }
        }
    } else if (app->scene == 2u) {
        app->support = sb_box(app, SL_BODY_KINEMATIC, 6.4f, 5.8f, 2.2f, 0.15f);
        for (uint32_t column = 0u; column < 5u; ++column) {
            (void)sb_box(app, SL_BODY_DYNAMIC, 5.2f + 0.6f * (float)column,
                         5.4f, 0.25f, 0.25f);
        }
    } else {
        const sl_vec2 cursor = app->cursor;
        app->cursor = sl_vec2_make(6.4f, 2.0f);
        sb_spawn_joint_chain(app, 0.0f);
        app->cursor = cursor;
    }
}

/* Exact public containment, including sleeping bodies. Query results are
 * slot-ordered, so strict distance improvement retains the lowest-slot tie. */
static sl_body_handle sb_pick_body(sb_app *app, sl_vec2 point)
{
    sl_query_result result = { 0 };
    sl_body_handle best = sl_body_handle_null();
    float best_distance = FLT_MAX;
    if (!sl_world_query_point(&app->world, point, SL_QUERY_DYNAMIC,
                              app->pick_buffer, SB_BODY_CAPACITY, &result)) {
        return best;
    }
    for (uint32_t i = 0u; i < result.count; ++i) {
        const sl_body_handle body = app->pick_buffer[i];
        const sl_vec2 position = sl_world_body_get_position(&app->world, body);
        const float distance = sl_vec2_length_sq(sl_vec2_sub(point, position));
        if (distance < best_distance) {
            best_distance = distance;
            best = body;
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
    desc.friction = k_sb_friction;
    desc.restitution = k_sb_restitution;

    sl_shape shape = sl_shape_none();
    sb_make_spawn_shape(app->spawn_kind, desc.mass, &shape);
    desc.shape = &shape;
    sl_world_body_create(&app->world, &desc);
}

static void sb_spawn_joint_chain(sb_app *app, float link_angle)
{
    const uint32_t body_remaining =
        sl_world_body_capacity(&app->world) - sl_world_body_count(&app->world);
    const uint32_t joint_remaining = sl_world_joint_capacity(&app->world) -
                                     sl_world_joint_count(&app->world);
    if (body_remaining < SB_CHAIN_LINK_COUNT + 1u ||
        joint_remaining < SB_CHAIN_LINK_COUNT) {
        return;
    }

    const sl_body_desc anchor_desc = {
        .position = app->cursor,
        .type = SL_BODY_STATIC,
    };
    sl_body_handle previous = sl_world_body_create(&app->world, &anchor_desc);
    SL_ASSERT(!sl_body_handle_is_null(previous));

    const float half_width = 0.1f;
    const float half_height = 0.225f;
    const float link_height = 2.0f * half_height;
    const sl_vec2 link_step = sl_rotation_apply(
        sl_rotation_make(link_angle), sl_vec2_make(0.0f, link_height));
    sl_shape link_shape = sl_shape_none();
    const bool made = sl_shape_make_box(half_width, half_height, &link_shape);
    SL_ASSERT(made);
    (void)made;

    for (uint32_t i = 0u; i < SB_CHAIN_LINK_COUNT; ++i) {
        const sl_body_desc link_desc = {
            /* Adjacent anchors coincide at the supplied initial angle. */
            .position = sl_vec2_add(app->cursor,
                                    sl_vec2_scale(link_step, (float)i + 0.5f)),
            .mass = 0.75f,
            .angle = link_angle,
            .shape = &link_shape,
            .friction = k_sb_friction,
            .restitution = k_sb_restitution,
        };
        const sl_body_handle link =
            sl_world_body_create(&app->world, &link_desc);
        SL_ASSERT(!sl_body_handle_is_null(link));

        const sl_joint_desc joint_desc = {
            .kind = SL_JOINT_REVOLUTE,
            .body_a = previous,
            .body_b = link,
            .local_anchor_a = (i == 0u) ? sl_vec2_make(0.0f, 0.0f)
                                        : sl_vec2_make(0.0f, half_height),
            .local_anchor_b = sl_vec2_make(0.0f, -half_height),
            .collide_connected = false,
        };
        const sl_joint_handle joint =
            sl_world_joint_create(&app->world, &joint_desc);
        SL_ASSERT(!sl_joint_handle_is_null(joint));
        (void)joint;
        previous = link;
    }
}

static void sb_apply_tether_for_step(sb_app *app);
static void sb_step(sb_app *app)
{
    if (sl_world_body_is_valid(&app->world, app->support)) {
        const float speed =
            !app->support_moving
                ? 0.0f
                : (app->scene_step % 480u < 240u ? 0.4f : -0.4f);
        const bool accepted = sl_world_body_set_velocity(
            &app->world, app->support, sl_vec2_make(speed, 0.0f));
        SL_ASSERT(accepted);
        (void)accepted;
    }
    sb_apply_tether_for_step(app);
    sl_world_step(&app->world, k_sb_timestep);
    app->scene_step = (app->scene_step + 1u) % 480u;
}

static void sb_handle_input(sb_app *app)
{
    const Vector2 mouse = GetMousePosition();
    app->cursor = sb_from_raylib(mouse);

    /* Consume press events so a quick down/up within one render frame is not
     * lost by polling only the final held state. These controls are ASCII;
     * other keys are ignored. Bound input work independently of rendering. */
    bool pressed[128] = { false };
    for (uint32_t event = 0u; event < 32u; ++event) {
        const int key = GetKeyPressed();
        if (key == 0) {
            break;
        }
        if (key > 0 && key < 128) {
            pressed[key] = true;
        }
    }

    if (pressed[KEY_SPACE]) {
        app->paused = !app->paused;
    }
    if (pressed[KEY_R]) {
        sb_reset(app);
        return;
    }
    for (uint32_t scene = 0u; scene < 4u; ++scene) {
        if (pressed[KEY_ONE + (int)scene]) {
            app->scene = scene;
            sb_reset(app);
            return;
        }
    }
    if (pressed[KEY_S]) {
        app->sleep_enabled = !app->sleep_enabled;
        sb_reset(app);
        return;
    }
    if (pressed[KEY_I]) {
        app->show_islands = !app->show_islands;
    }
    if (pressed[KEY_M]) {
        app->support_moving = !app->support_moving;
    }
    if (pressed[KEY_Q]) {
        app->query_mode = (app->query_mode + 1u) % 3u;
        app->query_defined = false;
        app->query_dragging = false;
    }
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        app->query_start = app->cursor;
        app->query_defined = true;
        app->query_dragging = true;
    }
    if (app->query_dragging) {
        app->query_end = app->cursor;
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
        app->query_dragging = false;
    }
    if (pressed[KEY_B]) {
        app->spawn_kind = (app->spawn_kind == SB_SPAWN_CIRCLE)
                              ? SB_SPAWN_BOX
                              : SB_SPAWN_CIRCLE;
    }
    if (pressed[KEY_C]) {
        app->show_contacts = !app->show_contacts;
    }
    if (pressed[KEY_A]) {
        app->show_aabbs = !app->show_aabbs;
    }
    if (pressed[KEY_J]) {
        sb_spawn_joint_chain(app, 0.35f);
    }
    if (pressed[KEY_PERIOD] && app->paused) {
        /* Manual step shows the tether acting: one application, one
         * consumed step, nothing left banked behind. */
        sb_step(app);
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        app->press_point = app->cursor;
        app->tether_body = sb_pick_body(app, app->cursor);
        app->selected = app->tether_body;
        if (!sl_body_handle_is_null(app->tether_body)) {
            const bool woke = sl_world_body_wake(&app->world, app->tether_body);
            SL_ASSERT(woke);
            (void)woke;
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
        sb_step(app);
        available -= k_sb_timestep;
        steps++;
    }

    /* At the cap, drop the rest: a slow frame sheds debt instead of
     * stepping forever on the next one. */
    app->bank = (steps == SL_STEP_COUNT_MAX) ? 0.0f : available;
}

/* Despawn unattended bodies that fell out of view after a fast tunnel or
 * launch, which would otherwise silently exhaust the spawn cap. Destroy is a
 * swap-remove -- the relocated body lands
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
    const float kill_y_metres = sb_length_from_pixels(
        (float)SB_SCREEN_HEIGHT + 64.0f); /* 64 px below the view */

    for (uint32_t row = 0u; row < sl_world_body_count(&app->world);) {
        const sl_body_handle body = sl_world_body_at(&app->world, row);
        const sl_vec2 pos = sl_world_body_get_position(&app->world, body);
        const bool held = sb_handle_eq(body, app->tether_body);
        if (pos.y > kill_y_metres && !held) {
            sl_world_body_destroy(&app->world, body);
        } else {
            row++;
        }
    }
}

static void sb_draw_grid(void)
{
    const float screen_width_metres =
        sb_length_from_pixels((float)SB_SCREEN_WIDTH);
    const float screen_height_metres =
        sb_length_from_pixels((float)SB_SCREEN_HEIGHT);

    for (float x = 0.0f; x <= screen_width_metres; x += k_sb_grid_spacing) {
        DrawLineV(sb_to_raylib(sl_vec2_make(x, 0.0f)),
                  sb_to_raylib(sl_vec2_make(x, screen_height_metres)),
                  Fade(DARKGRAY, 0.25f));
    }
    for (float y = 0.0f; y <= screen_height_metres; y += k_sb_grid_spacing) {
        DrawLineV(sb_to_raylib(sl_vec2_make(0.0f, y)),
                  sb_to_raylib(sl_vec2_make(screen_width_metres, y)),
                  Fade(DARKGRAY, 0.25f));
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
        } else if (sl_world_body_get_type(world, it) == SL_BODY_KINEMATIC) {
            fill = PURPLE;
        } else {
            sl_island_stats island;
            if (app->show_islands &&
                sl_world_body_get_island_stats(world, it, &island)) {
                const Color colors[] = { ORANGE, GOLD,    PINK,
                                         LIME,   SKYBLUE, VIOLET };
                fill = colors[island.id % 6u];
            }
            if (!sl_world_body_is_awake(world, it)) {
                fill = Fade(fill, 0.35f);
            }
        }

        if (shape->kind == SL_SHAPE_CIRCLE) {
            const float radius_pixels =
                sb_length_to_pixels(shape->circle.radius);
            DrawCircleV(sb_to_raylib(pos), radius_pixels, fill);
            DrawCircleLinesV(sb_to_raylib(pos), radius_pixels, DARKBROWN);
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

        /* Outline keeps transient overlaps readable. */
    }
}

static void sb_draw_bounds(sl_aabb bounds, Color color, float width)
{
    const Rectangle rectangle = {
        sb_length_to_pixels(bounds.lower.x),
        sb_length_to_pixels(bounds.lower.y),
        sb_length_to_pixels(bounds.upper.x - bounds.lower.x),
        sb_length_to_pixels(bounds.upper.y - bounds.lower.y),
    };
    DrawRectangleLinesEx(rectangle, width, color);
}

static void sb_draw_aabbs(const sb_app *app)
{
    if (!app->show_aabbs) {
        return;
    }
    for (sl_body_handle body = sl_world_body_first(&app->world);
         !sl_body_handle_is_null(body);
         body = sl_world_body_next(&app->world, body)) {
        sl_aabb fat;
        if (!sl_world_body_get_proxy_aabb(&app->world, body, &fat)) {
            continue;
        }
        sb_draw_bounds(fat, Fade(SKYBLUE, 0.75f), 1.0f);
        const sl_shape *shape = sl_world_body_get_shape(&app->world, body);
        const sl_transform tf = sl_world_body_get_transform(&app->world, body);
        sb_draw_bounds(sl_shape_aabb(shape, tf), Fade(YELLOW, 0.8f), 1.0f);
    }
}

static sl_aabb sb_query_bounds(const sb_app *app)
{
    return (sl_aabb){
        .lower = { sl_min(app->query_start.x, app->query_end.x),
                   sl_min(app->query_start.y, app->query_end.y) },
        .upper = { sl_max(app->query_start.x, app->query_end.x),
                   sl_max(app->query_start.y, app->query_end.y) },
    };
}

/* Query scratch is used after all mutations and before consuming snapshots.
 * Store copied handles only; recompute every frame so destruction cannot leave
 * a displayed query result pointing at a replacement body. */
static void sb_update_query(sb_app *app)
{
    app->query_result = (sl_query_result){ 0 };
    app->ray_result = (sl_query_ray_result){ 0 };
    app->query_valid = false;
    if (app->query_mode == 0u) {
        app->query_valid =
            sl_world_query_point(&app->world, app->cursor, SL_QUERY_ALL,
                                 app->query_bodies, 8u, &app->query_result);
    } else if (app->query_defined && app->query_mode == 1u) {
        const sl_ray ray = { app->query_start,
                             sl_vec2_sub(app->query_end, app->query_start) };
        app->query_valid = sl_world_query_ray(&app->world, ray, SL_QUERY_ALL,
                                              &app->ray_result);
        if (app->query_valid && app->ray_result.hit) {
            app->query_result.count = 1u;
            app->query_bodies[0] = app->ray_result.body;
        }
    } else if (app->query_defined) {
        app->query_valid =
            sl_world_query_aabb(&app->world, sb_query_bounds(app), SL_QUERY_ALL,
                                app->query_bodies, 8u, &app->query_result);
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
        app->selected = app->query_result.count > 0u ? app->query_bodies[0]
                                                     : sl_body_handle_null();
    }
}

static void sb_draw_query(const sb_app *app)
{
    if (app->query_mode == 0u) {
        DrawCircleLinesV(sb_to_raylib(app->cursor), 5.0f, WHITE);
    } else if (app->query_defined && app->query_mode == 1u) {
        DrawLineEx(sb_to_raylib(app->query_start), sb_to_raylib(app->query_end),
                   2.0f, WHITE);
        if (app->ray_result.hit) {
            const sl_ray_hit hit = app->ray_result.geometry;
            DrawCircleV(sb_to_raylib(hit.point), 5.0f, RED);
            DrawLineEx(sb_to_raylib(hit.point),
                       sb_to_raylib(sl_vec2_add(
                           hit.point, sl_vec2_scale(hit.normal, 0.3f))),
                       2.0f, RED);
        }
    } else if (app->query_defined) {
        sb_draw_bounds(sb_query_bounds(app), WHITE, 2.0f);
    }
    const uint32_t shown =
        app->query_result.count < 8u ? app->query_result.count : 8u;
    for (uint32_t i = 0u; i < shown; ++i) {
        const sl_body_handle body = app->query_bodies[i];
        const sl_shape *shape = sl_world_body_get_shape(&app->world, body);
        const sl_transform tf = sl_world_body_get_transform(&app->world, body);
        sb_draw_bounds(sl_shape_aabb(shape, tf), WHITE, 2.0f);
    }
}

static void sb_draw_contacts(const sb_app *app)
{
    if (!app->show_contacts) {
        return;
    }
    const sl_world *world = &app->world;
    for (uint32_t row = 0u; row < sl_world_contact_count(world); ++row) {
        /* Consume the snapshot immediately; no pointer survives this draw or
         * any later world mutation. */
        const sl_contact *contact = sl_world_contact_at(world, row);
        for (uint32_t i = 0u; i < contact->manifold.point_count; ++i) {
            const sl_manifold_point *point = &contact->manifold.points[i];
            const Color color = (point->separation > 0.0f) ? GOLD : RED;
            const sl_vec2 tip = sl_vec2_add(
                point->point, sl_vec2_scale(contact->manifold.normal, 0.25f));
            DrawCircleV(sb_to_raylib(point->point), 4.0f, color);
            DrawLineEx(sb_to_raylib(point->point), sb_to_raylib(tip), 2.0f,
                       color);
        }
    }
}

static void sb_draw_joints(const sb_app *app)
{
    const sl_world *world = &app->world;
    for (uint32_t row = 0u; row < sl_world_joint_count(world); ++row) {
        const sl_joint_handle joint = sl_world_joint_at(world, row);
        const sl_joint_desc desc = sl_world_joint_get_desc(world, joint);
        const sl_vec2 anchor_a =
            sl_transform_apply(sl_world_body_get_transform(world, desc.body_a),
                               desc.local_anchor_a);
        const sl_vec2 anchor_b =
            sl_transform_apply(sl_world_body_get_transform(world, desc.body_b),
                               desc.local_anchor_b);
        const Color color = (desc.kind == SL_JOINT_DISTANCE) ? VIOLET : MAGENTA;
        DrawLineEx(sb_to_raylib(anchor_a), sb_to_raylib(anchor_b), 2.0f,
                   Fade(color, 0.9f));
        DrawCircleV(sb_to_raylib(anchor_a), 3.0f, color);
        DrawCircleV(sb_to_raylib(anchor_b), 3.0f, color);
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
    /* Arrow length: 0.03 m per N, capped at 1.4 m so extreme tethers stay
     * readable. Drawn from the grab point, where the force lands. */
    const float length = sl_min(magnitude * 0.03f, 1.4f);
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
                             sb_length_to_pixels(ghost.circle.radius), GRAY);
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
    const char *scenes[] = { "playground", "settled piles", "moving support",
                             "joint chain" };
    const char *queries[] = { "point", "closest ray", "AABB" };
    const sl_world_stats stats = sl_world_get_stats(&app->world);
    DrawRectangle(4, 4, 630, 222, Fade(BLACK, 0.86f));
    DrawText(TextFormat("%u: %s | sleep %s | %s | %s", app->scene + 1u,
                        scenes[app->scene], app->sleep_enabled ? "on" : "off",
                        app->paused ? "PAUSED" : "running",
                        app->spawn_kind == SB_SPAWN_CIRCLE ? "circle" : "box"),
             12, 10, 20, WHITE);
    DrawText(
        TextFormat("Current: bodies %u/%u  awake %u  asleep %u  joints %u/%u",
                   stats.body_count, stats.body_capacity,
                   stats.awake_dynamic_count, stats.sleeping_dynamic_count,
                   stats.joint_count, stats.joint_capacity),
        12, 38, 16, LIME);
    DrawText(TextFormat("Contacts %u/%u  pairs %u/%u  drops %" PRIu64,
                        stats.contact_count, stats.contact_capacity,
                        stats.pair_count, stats.pair_capacity,
                        stats.cumulative.contact_drops),
             12, 60, 16, LIME);
    DrawText(
        TextFormat("Last step: islands %u  executed %u  skipped %u  largest %u",
                   stats.step.island_count, stats.step.island_executed_count,
                   stats.step.island_skipped_count,
                   stats.step.island_body_count_max),
        12, 82, 16, SKYBLUE);
    DrawText(TextFormat("Prepared contacts/joints %u/%u  tree visits %" PRIu64
                        "  probes %" PRIu64,
                        stats.step.contact_constraint_count,
                        stats.step.joint_constraint_count,
                        stats.step.work.tree_node_visits,
                        stats.step.work.pair_probes),
             12, 104, 16, SKYBLUE);
    DrawText(TextFormat("Cumulative: graph bodies %" PRIu64
                        "  wake visits %" PRIu64 "  wakes %" PRIu64,
                        stats.cumulative.graph_body_visits,
                        stats.cumulative.wake_visits,
                        stats.cumulative.body_wakes),
             12, 126, 16, GRAY);
    DrawText(TextFormat(
                 "Memory %zu B (arena %zu + shell %zu)  island %zu  sleep %zu",
                 app->memory.arena_bytes + app->memory.world_bytes,
                 app->memory.arena_bytes, app->memory.world_bytes,
                 app->memory.island_bytes, app->memory.sleep_bytes),
             12, 148, 16, GRAY);
    DrawText(
        TextFormat("Query %s: %s  matches %u  shown %u%s",
                   queries[app->query_mode],
                   app->query_valid
                       ? (app->query_result.count > 0u ? "HIT" : "MISS")
                       : (app->query_defined ? "INVALID" : "drag to define"),
                   app->query_result.count,
                   app->query_result.count < 8u ? app->query_result.count : 8u,
                   app->query_result.truncated ? "  TRUNCATED" : ""),
        12, 170, 18, WHITE);
    DrawText(TextFormat("%s colors | asleep = dim | bounds: tight yellow / fat "
                        "blue | %d fps",
                        app->show_islands ? "Last-build island" : "Activation",
                        GetFPS()),
             12, 196, 16, GRAY);
    if (sl_world_body_is_valid(&app->world, app->selected)) {
        const sl_body_handle body = app->selected;
        const sl_vec2 position = sl_world_body_get_position(&app->world, body);
        const sl_vec2 velocity = sl_world_body_get_velocity(&app->world, body);
        DrawRectangle(930, 4, 346, 144, Fade(BLACK, 0.86f));
        DrawText(TextFormat("Selected %u:%u  %s", body.index, body.generation,
                            sl_world_body_is_awake(&app->world, body)
                                ? "awake"
                                : "static/asleep"),
                 940, 12, 18, WHITE);
        DrawText(TextFormat("position %.3f, %.3f m", (double)position.x,
                            (double)position.y),
                 940, 38, 16, GRAY);
        DrawText(TextFormat("velocity %.3f, %.3f m/s", (double)velocity.x,
                            (double)velocity.y),
                 940, 60, 16, GRAY);
        DrawText(TextFormat("angle %.3f rad  spin %.3f rad/s",
                            (double)sl_world_body_get_angle(&app->world, body),
                            (double)sl_world_body_get_angular_velocity(
                                &app->world, body)),
                 940, 82, 16, GRAY);
        sl_island_stats island;
        if (sl_world_body_get_island_stats(&app->world, body, &island)) {
            DrawText(TextFormat("Last build: island %u, bodies %u", island.id,
                                island.dynamic_body_count),
                     940, 104, 16, SKYBLUE);
            DrawText(TextFormat("contacts %u, joints %u", island.contact_count,
                                island.joint_count),
                     940, 126, 16, SKYBLUE);
        }
    }
    DrawRectangle(0, SB_SCREEN_HEIGHT - 52, SB_SCREEN_WIDTH, 52,
                  Fade(BLACK, 0.9f));
    DrawText("1-4: scene | S: sleep + rebuild | R: reset | space: pause | .: "
             "step | M: move support | I: island colors",
             12, SB_SCREEN_HEIGHT - 48, 16, WHITE);
    DrawText("left hold: wake/tether | empty drag: spawn | B: shape | J: chain "
             "| Q: query mode | right drag: query | A: bounds | C: contacts",
             12, SB_SCREEN_HEIGHT - 24, 16, GRAY);
}

static void sb_draw(const sb_app *app)
{
    BeginDrawing();
    ClearBackground(BLACK);
    sb_draw_grid();
    sb_draw_interactions(app);
    sb_draw_bodies(app);
    sb_draw_aabbs(app);
    sb_draw_contacts(app);
    sb_draw_joints(app);
    sb_draw_forces(app);
    sb_draw_query(app);
    sb_draw_hud(app);
    EndDrawing();
}

int main(void)
{
    SetConfigFlags(FLAG_VSYNC_HINT);
    InitWindow(SB_SCREEN_WIDTH, SB_SCREEN_HEIGHT, "silk sandbox");
    if (!IsWindowReady()) {
        fprintf(stderr, "sandbox: display initialization failed\n");
        return 1;
    }
    SetTargetFPS(60);

    sb_app app = { 0 };

    app.sleep_enabled = true;
    app.show_contacts = true;
    sb_reset(&app);

    while (!app.failed && !WindowShouldClose()) {
        sb_handle_input(&app);
        if (app.failed) {
            break;
        }
        if (!app.paused) {
            sb_advance_frame(&app);
        }
        sb_cull_fallen(&app);
        sb_update_query(&app);
        sb_draw(&app);
    }

    sl_world_destroy(&app.world);
    CloseWindow();
    return app.failed ? 1 : 0;
}
