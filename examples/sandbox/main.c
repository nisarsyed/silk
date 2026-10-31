/* silk cloth sandbox: interactive raylib front end over the public API.
 *
 *   1-5    switch scene          left drag   grab and pull cloth
 *   space  pause / resume        right drag  orbit camera
 *   S      single step (paused)  wheel       zoom
 *   R      reset scene           W           toggle wind
 *   C      toggle self contact (rebuilds the scene)
 *
 * The simulation advances at most one fixed step per rendered frame, so a
 * scene that is costlier than real time runs in slow motion (shown in the
 * HUD) instead of freezing the window.
 *
 * --threads N runs the solver on N threads (default 4) through the example
 * pool (examples/pool); use the number of performance cores at most, since
 * a step waits for the slowest thread at every color.
 *
 * For visual checks without interaction:
 *   sandbox [--scene 1-5] [--wind] [--threads N]
 *           [--frames N --screenshot FILE.png]
 * runs N frames, saves a screenshot (raylib writes it relative to the
 * working directory) and exits.
 *
 * Palette (Box2D v3 conventions on graphite): dynamic cloth carries the
 * saturation, pinned (kinematic) vertices are royal blue, the held vertex
 * is a white-smoke highlight, colliders are static grey-green with the fill
 * at 0.6x the outline, forces (wind) magenta, red only for PAUSED. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "rlgl.h"
#include "silk/mesh.h"
#include "silk/silk.h"
#include "silk/step.h"
#include "silk/world.h"

#include "pool.h"

#define SB_WIDTH 1280
#define SB_HEIGHT 800
#define SB_DT (1.0f / 60.0f)
#define SB_TEXT 20

static const Color k_background = { 0x2B, 0x2D, 0x31, 255 };
static const Color k_ground = { 0x33, 0x35, 0x3A, 255 };
/* Dynamic cloths, one saturated hue each; the back face is darker. */
#define SB_CLOTH_COUNT_MAX 3u
static const Color k_cloth_front[SB_CLOTH_COUNT_MAX] = {
    { 0xE8, 0x7A, 0x3D, 255 },
    { 0x2F, 0xB8, 0xA6, 255 },
    { 0xC9, 0x5B, 0xD6, 255 }
};
static const Color k_collider = { 0x8B, 0xB4, 0x8B, 255 };
static const Color k_pinned = { 0x41, 0x69, 0xE1, 255 }; /* RoyalBlue */
static const Color k_held = { 0xF5, 0xF5, 0xF5, 255 };   /* WhiteSmoke */
static const Color k_wind = { 0xFF, 0x00, 0xFF, 255 };   /* Magenta */

typedef enum sb_scene_id {
    SB_SCENE_CURTAIN = 0,
    SB_SCENE_DRAPE = 1,
    SB_SCENE_FLAG = 2,
    SB_SCENE_RAIL = 3,
    SB_SCENE_PILE = 4,
    SB_SCENE_COUNT = 5
} sb_scene_id;

static const char *const k_scene_names[SB_SCENE_COUNT] = {
    "curtain", "sphere drape", "flag", "towel on a rail", "pile of sheets"
};

typedef struct sb_app {
    sl_world world;
    sl_stepper stepper;
    sb_scene_id scene;
    uint32_t cloth_count;    /* every cloth shares one grid topology */
    uint32_t vertex_count;   /* per cloth */
    uint32_t triangle_count; /* per cloth */
    uint32_t *indices;
    Vector3 *normals; /* vertex_count per cloth */
    sl_vec3 wind;
    bool wind_on;
    bool paused;
    bool self_contact;
    double sim_time; /* seconds simulated since the scene loaded */
    /* Curtain: ring indices and their starting x, gathered over time. */
    uint32_t ring_count;
    uint32_t ring_vertex[16];
    float ring_x[16];
    /* Grab state; held == UINT32_MAX when nothing is held. */
    uint32_t held;
    uint32_t held_cloth;
    bool held_was_pinned;
    Vector3 held_plane_point;
    /* Orbit camera. */
    float yaw;
    float pitch;
    float distance;
    Vector3 focus;
    double step_ms;
    sp_pool *pool; /* shared by every scene */
} sb_app;

static Vector3 sb_v(sl_vec3 v)
{
    Vector3 r = { v.x, v.y, v.z };
    return r;
}

static sl_vec3 sb_sl(Vector3 v)
{
    return sl_vec3_make(v.x, v.y, v.z);
}

static sl_cloth_desc sb_cotton(void)
{
    sl_cloth_desc d;
    memset(&d, 0, sizeof(d));
    d.density = 0.2f;
    d.membrane_mu = 2.0e3f;
    d.membrane_lambda = 2.0e3f;
    d.bending_stiffness = 5.0e-4f;
    d.damping = 20.0f;
    d.thickness = 0.004f; /* self-contact gap 8 mm, under 0.25 edge */
    d.friction = 0.6f;
    d.drag = 0.3f;
    return d;
}

static void sb_scene_free(sb_app *app)
{
    sl_world_destroy(&app->world);
    free(app->indices);
    free(app->normals);
    app->indices = NULL;
    app->normals = NULL;
    app->cloth_count = 0;
    app->vertex_count = 0;
    app->triangle_count = 0;
    app->held = UINT32_MAX;
}

/* Rotates v about the y axis by angle radians. */
static sl_vec3 sb_turn(sl_vec3 v, float angle)
{
    const float c = cosf(angle);
    const float n = sinf(angle);
    return sl_vec3_make(c * v.x - n * v.z, v.y, n * v.x + c * v.z);
}

/* Builds `layers` grid cloths spanning origin + s u + t v, each raised by
 * `rise` and turned about the vertical by `turn` radians from the last,
 * with self contact unless toggled off, and offset along its normal by
 * `pleat` * sin(pi * column / 4) (m). Returns false on any allocation or
 * validation failure, leaving the app empty. */
static bool sb_scene_layers(sb_app *app, sl_vec3 origin, sl_vec3 u, sl_vec3 v,
                            uint32_t cells_u, uint32_t cells_v, float drag,
                            uint32_t layers, float rise, float turn,
                            float pleat)
{
    const uint32_t n = sl_mesh_grid_vertex_count(cells_u, cells_v);
    const uint32_t tn = sl_mesh_grid_triangle_count(cells_u, cells_v);
    sl_vec3 *positions = malloc(sizeof(sl_vec3) * n);
    app->indices = malloc(sizeof(uint32_t) * 3u * tn);
    app->normals = malloc(sizeof(Vector3) * n * layers);
    sl_world_config config;
    memset(&config, 0, sizeof(config));
    config.vertex_capacity = n * layers;
    config.triangle_capacity = tn * layers;
    config.cloth_capacity = layers;
    config.gravity = sl_vec3_make(0.0f, -9.81f, 0.0f);
    config.self_contact_capacity = app->self_contact ? 8u * n * layers : 0u;
    config.parallel_for = (app->pool != NULL) ? sp_pool_parallel_for : NULL;
    config.parallel_context = app->pool;
    bool ok = positions != NULL && app->indices != NULL &&
              app->normals != NULL && sl_world_init(&app->world, &config);
    const sl_vec3 center =
        sl_vec3_add(origin, sl_vec3_scale(sl_vec3_add(u, v), 0.5f));
    for (uint32_t k = 0; ok && k < layers; k++) {
        const sl_vec3 uk = sb_turn(u, turn * (float)k);
        const sl_vec3 vk = sb_turn(v, turn * (float)k);
        const sl_vec3 ck = sl_vec3_mul_add(
            center, sl_vec3_make(0.0f, 1.0f, 0.0f), rise * (float)k);
        const sl_vec3 ok_origin =
            sl_vec3_sub(ck, sl_vec3_scale(sl_vec3_add(uk, vk), 0.5f));
        sl_cloth_desc desc = sb_cotton();
        desc.drag = drag;
        desc.positions = positions;
        desc.vertex_count = n;
        desc.indices = app->indices;
        desc.triangle_count = tn;
        uint32_t cloth = 0;
        ok = sl_mesh_grid(ok_origin, uk, vk, cells_u, cells_v, positions, n,
                          app->indices, 3u * tn);
        /* Pleats: offset along the normal, pleat * sin(pi * column / 4). */
        const sl_vec3 normal = sl_vec3_normalize(sl_vec3_cross(uk, vk));
        for (uint32_t j = 0; ok && pleat != 0.0f && j < n; j++) {
            const float column = (float)(j % (cells_u + 1u));
            positions[j] = sl_vec3_mul_add(positions[j], normal,
                                           pleat * sinf(PI * column / 4.0f));
        }
        ok = ok && sl_world_cloth_add(&app->world, &desc, &cloth);
    }
    free(positions);
    if (!ok) {
        sb_scene_free(app);
        return false;
    }
    app->cloth_count = layers;
    app->vertex_count = n;
    app->triangle_count = tn;
    return true;
}

static bool sb_scene_grid(sb_app *app, sl_vec3 origin, sl_vec3 u, sl_vec3 v,
                          uint32_t cells_u, uint32_t cells_v, float drag)
{
    return sb_scene_layers(app, origin, u, v, cells_u, cells_v, drag, 1u, 0.0f,
                           0.0f, 0.0f);
}

static bool sb_add_collider(sb_app *app, sl_collider_kind kind, sl_vec3 a,
                            sl_vec3 b, float radius)
{
    sl_collider c;
    memset(&c, 0, sizeof(c));
    c.kind = kind;
    c.point_a = a;
    c.point_b = b;
    c.radius = radius;
    c.friction = 0.6f;
    uint32_t id = 0;
    return sl_world_collider_add(&app->world, &c, &id);
}

static bool sb_scene_load(sb_app *app, sb_scene_id scene)
{
    sb_scene_free(app);
    app->scene = scene;
    app->sim_time = 0.0;
    app->ring_count = 0u;
    app->wind = sl_vec3_zero();
    (void)sl_stepper_init(&app->stepper, SB_DT);
    const sl_vec3 up = sl_vec3_make(0.0f, 1.0f, 0.0f);
    bool ok = false;
    switch (scene) {
    case SB_SCENE_CURTAIN: {
        /* 1.6 m x 1.4 m, hung from rings at every 4th top vertex, with
         * 1 cm pleats between the rings. A perfectly flat curtain has no
         * reason to leave its plane: gathering it crumpled the sheet within
         * the plane, coplanar triangles overlapped at distance 0, and the
         * vertices there froze at zero self-contact bounds. */
        const uint32_t cu = 40u;
        ok = sb_scene_layers(app, sl_vec3_make(-0.8f, 2.0f, 0.0f),
                             sl_vec3_make(1.6f, 0.0f, 0.0f),
                             sl_vec3_make(0.0f, -1.4f, 0.0f), cu, 35u, 0.3f, 1u,
                             0.0f, 0.0f, 0.01f);
        /* Rings slide together to 65% of the width over SB_GATHER_TIME
         * (sb_gather_rings), gathering the curtain into folds. Doing it in
         * one step crushed the top into pleats instantly; pinned vertices
         * are not bounded by self contact, and that one scene cost up to
         * 3 s per step. */
        const sl_vec3 *p =
            ok ? sl_world_cloth_positions(&app->world, 0u) : NULL;
        for (uint32_t i = 0; ok && i <= cu && app->ring_count < 16u; i += 4u) {
            app->ring_vertex[app->ring_count] = i;
            app->ring_x[app->ring_count] = p[i].x;
            app->ring_count++;
            ok = sl_world_vertex_pin_set(&app->world, 0u, i, true);
        }
        ok = ok &&
             sb_add_collider(app, SL_COLLIDER_PLANE, sl_vec3_zero(), up, 0.0f);
        app->wind = sl_vec3_make(0.0f, 0.0f, 2.5f);
        app->focus = (Vector3){ 0.0f, 1.2f, 0.0f };
        break;
    }
    case SB_SCENE_DRAPE:
        ok = sb_scene_grid(app, sl_vec3_make(-0.8f, 1.4f, -0.8f),
                           sl_vec3_make(1.6f, 0.0f, 0.0f),
                           sl_vec3_make(0.0f, 0.0f, 1.6f), 40u, 40u, 0.3f);
        ok = ok &&
             sb_add_collider(app, SL_COLLIDER_SPHERE,
                             sl_vec3_make(0.0f, 0.7f, 0.0f), sl_vec3_zero(),
                             0.45f) &&
             sb_add_collider(app, SL_COLLIDER_PLANE, sl_vec3_zero(), up, 0.0f);
        app->wind = sl_vec3_make(1.5f, 0.0f, 0.5f);
        app->focus = (Vector3){ 0.0f, 0.6f, 0.0f };
        break;
    case SB_SCENE_FLAG: {
        /* 1.5 m x 1 m, pinned along the pole edge. */
        const uint32_t cu = 36u;
        const uint32_t cv = 24u;
        ok = sb_scene_grid(app, sl_vec3_make(0.0f, 2.4f, 0.0f),
                           sl_vec3_make(1.5f, 0.0f, 0.0f),
                           sl_vec3_make(0.0f, -1.0f, 0.0f), cu, cv, 0.6f);
        for (uint32_t j = 0; ok && j <= cv; j++) {
            ok = sl_world_vertex_pin_set(&app->world, 0u, j * (cu + 1u), true);
        }
        ok = ok && sb_add_collider(app, SL_COLLIDER_CAPSULE,
                                   sl_vec3_make(-0.03f, 0.0f, 0.0f),
                                   sl_vec3_make(-0.03f, 2.5f, 0.0f), 0.025f);
        app->wind = sl_vec3_make(6.0f, 0.0f, 1.5f);
        app->focus = (Vector3){ 0.8f, 1.8f, 0.0f };
        break;
    }
    case SB_SCENE_RAIL:
        ok = sb_scene_grid(app, sl_vec3_make(-0.5f, 1.3f, -0.6f),
                           sl_vec3_make(1.0f, 0.0f, 0.0f),
                           sl_vec3_make(0.0f, 0.0f, 1.2f), 32u, 38u, 0.3f);
        ok = ok &&
             sb_add_collider(app, SL_COLLIDER_CAPSULE,
                             sl_vec3_make(-0.8f, 1.0f, 0.1f),
                             sl_vec3_make(0.8f, 1.0f, 0.1f), 0.05f) &&
             sb_add_collider(app, SL_COLLIDER_PLANE, sl_vec3_zero(), up, 0.0f);
        app->wind = sl_vec3_make(0.0f, 0.0f, 3.0f);
        app->focus = (Vector3){ 0.0f, 0.8f, 0.0f };
        break;
    case SB_SCENE_PILE:
        /* Three 1.2 m sheets, each 15 cm above and turned 30 degrees from
         * the last, dropped on a sphere: cloth-cloth contact. */
        ok = sb_scene_layers(app, sl_vec3_make(-0.6f, 1.1f, -0.6f),
                             sl_vec3_make(1.2f, 0.0f, 0.0f),
                             sl_vec3_make(0.0f, 0.0f, 1.2f), 30u, 30u, 0.3f, 3u,
                             0.15f, 0.52f, 0.0f);
        ok = ok &&
             sb_add_collider(app, SL_COLLIDER_SPHERE,
                             sl_vec3_make(0.0f, 0.55f, 0.0f), sl_vec3_zero(),
                             0.3f) &&
             sb_add_collider(app, SL_COLLIDER_PLANE, sl_vec3_zero(), up, 0.0f);
        app->wind = sl_vec3_make(1.5f, 0.0f, 0.5f);
        app->focus = (Vector3){ 0.0f, 0.6f, 0.0f };
        break;
    case SB_SCENE_COUNT:
        break;
    }
    if (ok) {
        ok = sl_world_wind_set(&app->world,
                               app->wind_on ? app->wind : sl_vec3_zero());
    }
    if (!ok) {
        sb_scene_free(app);
    }
    return ok;
}

#define SB_GATHER_TIME 1.5f

/* Moves the curtain rings toward 65% of their spread, smoothly, reaching
 * it SB_GATHER_TIME after the scene loads. Targets are for the end of the
 * next step. A held ring is left to the mouse. */
static void sb_gather_rings(sb_app *app)
{
    const float t = sl_clamp(
        (float)(app->sim_time + (double)SB_DT) / SB_GATHER_TIME, 0.0f, 1.0f);
    const float ease = t * t * (3.0f - 2.0f * t);
    const sl_vec3 *p = sl_world_cloth_positions(&app->world, 0u);
    for (uint32_t r = 0; r < app->ring_count; r++) {
        const uint32_t i = app->ring_vertex[r];
        if (app->held == i && app->held_cloth == 0u) {
            continue;
        }
        const float x = app->ring_x[r] * (1.0f - 0.35f * ease);
        (void)sl_world_vertex_target_set(&app->world, 0u, i,
                                         sl_vec3_make(x, p[i].y, p[i].z));
    }
}

static Camera3D sb_camera(const sb_app *app)
{
    Camera3D camera;
    memset(&camera, 0, sizeof(camera));
    const float cp = cosf(app->pitch);
    camera.position = (Vector3){
        app->focus.x + app->distance * cp * sinf(app->yaw),
        app->focus.y + app->distance * sinf(app->pitch),
        app->focus.z + app->distance * cp * cosf(app->yaw),
    };
    camera.target = app->focus;
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    return camera;
}

static Vector3 sb_camera_forward(Camera3D camera)
{
    return sb_v(sl_vec3_normalize(
        sl_vec3_sub(sb_sl(camera.target), sb_sl(camera.position))));
}

/* Nearest cloth vertex under the mouse, or UINT32_MAX; *cloth gets its
 * cloth. */
static uint32_t sb_pick(const sb_app *app, Ray ray, Vector3 *hit_point,
                        uint32_t *cloth)
{
    float best = INFINITY;
    uint32_t picked = UINT32_MAX;
    for (uint32_t c = 0; c < app->cloth_count; c++) {
        const sl_vec3 *p = sl_world_cloth_positions(&app->world, c);
        for (uint32_t t = 0; t < app->triangle_count; t++) {
            const uint32_t *idx = app->indices + 3u * t;
            const Vector3 a = sb_v(p[idx[0]]);
            const Vector3 b = sb_v(p[idx[1]]);
            const Vector3 d = sb_v(p[idx[2]]);
            RayCollision hit = GetRayCollisionTriangle(ray, a, b, d);
            if (!hit.hit) {
                hit = GetRayCollisionTriangle(ray, a, d, b); /* back side */
            }
            if (!hit.hit || hit.distance >= best) {
                continue;
            }
            best = hit.distance;
            *hit_point = hit.point;
            *cloth = c;
            float nearest = INFINITY;
            for (uint32_t k = 0; k < 3u; k++) {
                const float dist =
                    sl_vec3_length_sq(sl_vec3_sub(p[idx[k]], sb_sl(hit.point)));
                if (dist < nearest) {
                    nearest = dist;
                    picked = idx[k];
                }
            }
        }
    }
    return picked;
}

static void sb_update_grab(sb_app *app, Camera3D camera)
{
    const Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && app->vertex_count > 0u) {
        Vector3 hit = { 0.0f, 0.0f, 0.0f };
        uint32_t cloth = 0;
        const uint32_t vertex = sb_pick(app, ray, &hit, &cloth);
        if (vertex != UINT32_MAX) {
            app->held = vertex;
            app->held_cloth = cloth;
            app->held_was_pinned =
                sl_world_vertex_is_pinned(&app->world, cloth, vertex);
            app->held_plane_point = hit;
            (void)sl_world_vertex_pin_set(&app->world, cloth, vertex, true);
        }
    }
    if (app->held == UINT32_MAX) {
        return;
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        if (!app->held_was_pinned) {
            (void)sl_world_vertex_pin_set(&app->world, app->held_cloth,
                                          app->held, false);
        }
        app->held = UINT32_MAX;
        return;
    }
    /* Drag in the plane through the grab point facing the camera. */
    const sl_vec3 forward = sb_sl(sb_camera_forward(camera));
    const float denom = sl_vec3_dot(sb_sl(ray.direction), forward);
    if (sl_abs(denom) < 1e-4f) {
        return;
    }
    const float t = sl_vec3_dot(sl_vec3_sub(sb_sl(app->held_plane_point),
                                            sb_sl(ray.position)),
                                forward) /
                    denom;
    const sl_vec3 target =
        sl_vec3_mul_add(sb_sl(ray.position), sb_sl(ray.direction), t);
    (void)sl_world_vertex_target_set(&app->world, app->held_cloth, app->held,
                                     target);
}

static void sb_update_camera(sb_app *app)
{
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        const Vector2 delta = GetMouseDelta();
        app->yaw -= delta.x * 0.005f;
        app->pitch = sl_clamp(app->pitch + delta.y * 0.005f, -0.2f, 1.4f);
    }
    const float wheel = GetMouseWheelMove();
    app->distance =
        sl_clamp(app->distance * (1.0f - 0.1f * wheel), 1.0f, 12.0f);
}

/* Area-weighted vertex normals for smooth two-sided shading. */
static void sb_compute_normals(const sb_app *app, const sl_vec3 *p,
                               Vector3 *normals)
{
    for (uint32_t i = 0; i < app->vertex_count; i++) {
        normals[i] = (Vector3){ 0.0f, 0.0f, 0.0f };
    }
    for (uint32_t t = 0; t < app->triangle_count; t++) {
        const uint32_t *idx = app->indices + 3u * t;
        const sl_vec3 n = sl_vec3_cross(sl_vec3_sub(p[idx[1]], p[idx[0]]),
                                        sl_vec3_sub(p[idx[2]], p[idx[0]]));
        for (uint32_t k = 0; k < 3u; k++) {
            normals[idx[k]] = sb_v(sl_vec3_add(sb_sl(normals[idx[k]]), n));
        }
    }
    for (uint32_t i = 0; i < app->vertex_count; i++) {
        normals[i] = sb_v(sl_vec3_normalize(sb_sl(normals[i])));
    }
}

static unsigned char sb_channel(unsigned char base, float light)
{
    return (unsigned char)sl_clamp((float)base * light, 0.0f, 255.0f);
}

static void sb_shaded_vertex(sl_vec3 p, sl_vec3 n, Color base, sl_vec3 light)
{
    const float lambert = sl_max(sl_vec3_dot(n, light), 0.0f);
    const float k = 0.35f + 0.75f * lambert;
    rlColor4ub(sb_channel(base.r, k), sb_channel(base.g, k),
               sb_channel(base.b, k), 255);
    rlVertex3f(p.x, p.y, p.z);
}

static void sb_draw_cloth(sb_app *app)
{
    const sl_vec3 light = sl_vec3_normalize(sl_vec3_make(0.4f, 1.0f, 0.6f));
    for (uint32_t c = 0; c < app->cloth_count; c++) {
        const sl_vec3 *p = sl_world_cloth_positions(&app->world, c);
        Vector3 *normals = app->normals + c * app->vertex_count;
        sb_compute_normals(app, p, normals);
        const Color front = k_cloth_front[c % SB_CLOTH_COUNT_MAX];
        const Color back = ColorBrightness(front, -0.35f);
        rlBegin(RL_TRIANGLES);
        for (uint32_t t = 0; t < app->triangle_count; t++) {
            const uint32_t *idx = app->indices + 3u * t;
            for (uint32_t k = 0; k < 3u; k++) {
                sb_shaded_vertex(p[idx[k]], sb_sl(normals[idx[k]]), front,
                                 light);
            }
            /* Back face: reversed winding, flipped normal. */
            for (uint32_t k = 3u; k > 0u; k--) {
                sb_shaded_vertex(p[idx[k - 1u]],
                                 sl_vec3_neg(sb_sl(normals[idx[k - 1u]])), back,
                                 light);
            }
        }
        rlEnd();
        for (uint32_t i = 0; i < app->vertex_count; i++) {
            const bool held = app->held == i && app->held_cloth == c;
            if (!held && sl_world_vertex_is_pinned(&app->world, c, i)) {
                DrawSphere(sb_v(p[i]), 0.012f, k_pinned);
            }
        }
        if (app->held != UINT32_MAX && app->held_cloth == c) {
            DrawSphere(sb_v(p[app->held]), 0.02f, k_held);
        }
    }
}

static void sb_draw_colliders(const sb_app *app)
{
    const Color fill = ColorBrightness(k_collider, -0.4f); /* 0.6x */
    for (uint32_t c = 0;; c++) {
        sl_collider collider;
        if (!sl_world_collider_get(&app->world, c, &collider)) {
            break;
        }
        switch (collider.kind) {
        case SL_COLLIDER_PLANE:
            DrawPlane(sb_v(collider.point_a), (Vector2){ 8.0f, 8.0f },
                      k_ground);
            break;
        case SL_COLLIDER_SPHERE:
            /* Drawn slightly inside so the cloth's thickness shell shows. */
            DrawSphere(sb_v(collider.point_a), collider.radius * 0.99f, fill);
            DrawSphereWires(sb_v(collider.point_a), collider.radius, 12, 18,
                            k_collider);
            break;
        case SL_COLLIDER_CAPSULE:
            DrawCapsule(sb_v(collider.point_a), sb_v(collider.point_b),
                        collider.radius * 0.99f, 12, 6, fill);
            DrawCapsuleWires(sb_v(collider.point_a), sb_v(collider.point_b),
                             collider.radius, 12, 6, k_collider);
            break;
        }
    }
}

static void sb_draw_wind(const sb_app *app)
{
    if (!app->wind_on || sl_vec3_length(app->wind) <= 0.0f) {
        return;
    }
    const sl_vec3 from =
        sl_vec3_make(app->focus.x - 1.4f, app->focus.y + 0.9f, app->focus.z);
    const sl_vec3 to = sl_vec3_mul_add(from, app->wind, 0.08f);
    DrawLine3D(sb_v(from), sb_v(to), k_wind);
    DrawSphere(sb_v(to), 0.02f, k_wind);
}

static void sb_draw_hud(const sb_app *app)
{
    const sl_world_stats stats = sl_world_stats_get(&app->world);
    const int version = sl_version();
    DrawText(TextFormat("silk %d.%d.%d  |  %d  %s", (version >> 16) & 0xff,
                        (version >> 8) & 0xff, version & 0xff,
                        (int)app->scene + 1, k_scene_names[app->scene]),
             12, 10, SB_TEXT, k_held);
    DrawText(TextFormat("%d vertices  %d triangles  %d substeps%s  %.2f "
                        "ms/step  %d thread(s)",
                        (int)stats.vertex_count, (int)stats.triangle_count,
                        (int)stats.substep_count,
                        stats.substep_limited ? " (capped)" : "", app->step_ms,
                        (int)sp_pool_thread_count(app->pool)),
             12, 36, SB_TEXT, GRAY);
    DrawText(TextFormat("%d collider contacts  %d self-contact pairs  %d "
                        "detections/step",
                        (int)stats.contact_count, (int)stats.self_contact_count,
                        (int)stats.self_detection_count),
             12, 62, SB_TEXT, GRAY);
    DrawText("1-5 scene  space pause  S step  R reset  W wind  C contact  "
             "LMB grab  RMB orbit  wheel zoom",
             12, SB_HEIGHT - 30, SB_TEXT, GRAY);
    if (app->wind_on) {
        DrawText("WIND", SB_WIDTH - 90, 10, SB_TEXT, k_wind);
    }
    if (app->paused) {
        DrawText("PAUSED", SB_WIDTH - 90, 36, SB_TEXT, RED);
    } else if (app->step_ms > 1000.0 * (double)SB_DT) {
        /* One step per frame, and the step costs more than it simulates. */
        DrawText("SLOW MOTION", SB_WIDTH - 160, 36, SB_TEXT, RED);
    }
    if (!app->self_contact) {
        DrawText("NO SELF CONTACT", SB_WIDTH - 210, 62, SB_TEXT, GRAY);
    }
}

static void sb_handle_keys(sb_app *app)
{
    for (int k = 0; k < (int)SB_SCENE_COUNT; k++) {
        if (IsKeyPressed(KEY_ONE + k)) {
            (void)sb_scene_load(app, (sb_scene_id)k);
        }
    }
    if (IsKeyPressed(KEY_R)) {
        (void)sb_scene_load(app, app->scene);
    }
    if (IsKeyPressed(KEY_C)) {
        app->self_contact = !app->self_contact;
        (void)sb_scene_load(app, app->scene);
    }
    if (IsKeyPressed(KEY_SPACE)) {
        app->paused = !app->paused;
    }
    if (IsKeyPressed(KEY_W)) {
        app->wind_on = !app->wind_on;
        if (app->vertex_count > 0u) {
            (void)sl_world_wind_set(&app->world,
                                    app->wind_on ? app->wind : sl_vec3_zero());
        }
    }
}

static uint32_t sb_arg_count(const char *text)
{
    char *end = NULL;
    const unsigned long value = strtoul(text, &end, 10);
    return (end != text && *end == '\0' && value <= 100000ul) ? (uint32_t)value
                                                              : 0u;
}

int main(int argc, char **argv)
{
    sb_scene_id first = SB_SCENE_CURTAIN;
    bool wind = false;
    uint32_t frames = 0;
    const char *screenshot = NULL;
    uint32_t threads = 4u;
    for (int a = 1; a < argc; a++) {
        if (strcmp(argv[a], "--wind") == 0) {
            wind = true;
        } else if (a + 1 < argc && strcmp(argv[a], "--scene") == 0) {
            const uint32_t n = sb_arg_count(argv[++a]);
            if (n >= 1u && n <= (uint32_t)SB_SCENE_COUNT) {
                first = (sb_scene_id)(n - 1u);
            }
        } else if (a + 1 < argc && strcmp(argv[a], "--frames") == 0) {
            frames = sb_arg_count(argv[++a]);
        } else if (a + 1 < argc && strcmp(argv[a], "--screenshot") == 0) {
            screenshot = argv[++a];
        } else if (a + 1 < argc && strcmp(argv[a], "--threads") == 0) {
            threads = sb_arg_count(argv[++a]);
        }
    }
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(SB_WIDTH, SB_HEIGHT, "silk cloth sandbox");
    SetTargetFPS(60);
    sb_app app;
    memset(&app, 0, sizeof(app));
    app.held = UINT32_MAX;
    app.yaw = 0.6f;
    app.pitch = 0.35f;
    app.distance = 4.0f;
    app.wind_on = wind;
    app.self_contact = true;
    app.pool = sp_pool_create(threads);
    if (app.pool == NULL) {
        CloseWindow();
        return 1;
    }
    (void)sb_scene_load(&app, first);

    uint32_t frame = 0;
    double step_ms_sum = 0.0;
    double step_ms_max = 0.0;
    while (!WindowShouldClose()) {
        sb_handle_keys(&app);
        sb_update_camera(&app);
        const Camera3D camera = sb_camera(&app);
        if (app.vertex_count > 0u) {
            sb_update_grab(&app, camera);
            const double start = GetTime();
            uint32_t steps = 0;
            if (app.ring_count > 0u) {
                sb_gather_rings(&app);
            }
            if (frames > 0u) {
                /* Capture mode: exactly one fixed step per frame. */
                sl_world_step(&app.world, SB_DT);
                steps = 1u;
            } else if (!app.paused) {
                /* At most one step per frame: frame time beyond one step is
                 * dropped, so a slow scene runs in slow motion and input is
                 * still read every frame. */
                steps = sl_world_advance(&app.world, &app.stepper,
                                         sl_min(GetFrameTime(), SB_DT));
            } else if (IsKeyPressed(KEY_S)) {
                sl_world_step(&app.world, SB_DT);
                steps = 1u;
            }
            app.sim_time += (double)steps * (double)SB_DT;
            if (steps > 0u) {
                app.step_ms = 1000.0 * (GetTime() - start) / (double)steps;
                step_ms_sum += app.step_ms;
                step_ms_max =
                    (app.step_ms > step_ms_max) ? app.step_ms : step_ms_max;
            }
        }

        BeginDrawing();
        ClearBackground(k_background);
        BeginMode3D(camera);
        rlDisableBackfaceCulling();
        if (app.vertex_count > 0u) {
            sb_draw_colliders(&app);
            sb_draw_cloth(&app);
            sb_draw_wind(&app);
        }
        EndMode3D();
        if (app.vertex_count > 0u) {
            sb_draw_hud(&app);
        } else {
            DrawText("scene failed to build", 12, 10, SB_TEXT, RED);
        }
        EndDrawing();
        frame++;
        if (frames > 0u && frame >= frames) {
            if (screenshot != NULL) {
                TakeScreenshot(screenshot);
            }
            printf("scene %d: %.2f ms/step mean, %.2f ms max over %u steps\n",
                   (int)app.scene + 1, step_ms_sum / (double)frames,
                   step_ms_max, frames);
            break;
        }
    }
    sb_scene_free(&app);
    sp_pool_destroy(app.pool);
    CloseWindow();
    return 0;
}
