#include "context.h"

#include <stdlib.h>
#include <string.h>

#include <silk/step.h>

sl_wasm_context *sl_wasm_context_create(void)
{
    return calloc(1u, sizeof(sl_wasm_context));
}
void sl_wasm_context_destroy(sl_wasm_context *context)
{
    if (context != NULL) {
        sl_wasm_world_dispose(context);
        free(context);
    }
}
size_t sl_wasm_context_bytes(void)
{
    return sizeof(sl_wasm_context);
}
float *sl_wasm_input_f32(sl_wasm_context *context)
{
    return context->input_f32;
}
uint32_t *sl_wasm_input_u32(sl_wasm_context *context)
{
    return context->input_u32;
}
const float *sl_wasm_output_f32(const sl_wasm_context *context)
{
    return context->output_f32;
}
const uint32_t *sl_wasm_output_u32(const sl_wasm_context *context)
{
    return context->output_u32;
}

static sl_world_config config_read(const sl_wasm_context *c)
{
    const float *f = c->input_f32;
    const uint32_t *u = c->input_u32;
    return (sl_world_config){ .body_capacity = u[0],
                              .contact_capacity = u[1],
                              .joint_capacity = u[2],
                              .substep_count = u[3],
                              .sleep_enabled = u[4] == 1u,
                              .gravity = { f[0], f[1] },
                              .linear_drag = f[2],
                              .angular_drag = f[3],
                              .linear_speed_max = f[4],
                              .contact_hertz = f[5],
                              .contact_damping_ratio = f[6],
                              .contact_push_velocity_max = f[7],
                              .restitution_threshold = f[8],
                              .joint_hertz = f[9],
                              .joint_damping_ratio = f[10],
                              .sleep_speed_max = f[11],
                              .sleep_angular_speed_max = f[12],
                              .sleep_time_min = f[13] };
}
size_t sl_wasm_world_bytes(const sl_wasm_context *context)
{
    if (context == NULL || context->input_u32[4] > 1u) {
        return 0u;
    }
    const sl_world_config config = config_read(context);
    return sl_world_memory_bytes(&config);
}
bool sl_wasm_world_init(sl_wasm_context *context)
{
    if (context == NULL || context->world.state != NULL ||
        sl_wasm_world_bytes(context) == 0u) {
        return false;
    }
    const sl_world_config config = config_read(context);
    if (!sl_world_init(&context->world, &config)) {
        return false;
    }
    context->config = config;
    context->config.contact_capacity =
        sl_world_contact_capacity(&context->world);
    if (!sl_wasm_storage_init(context)) {
        sl_world_destroy(&context->world);
        return false;
    }
    return true;
}
void sl_wasm_world_reset(sl_wasm_context *context)
{
    if (context != NULL && context->world.state != NULL) {
        sl_world_reset(&context->world);
        memset(context->geometry_generation, 0,
               (size_t)context->config.body_capacity * sizeof(uint32_t));
    }
}
void sl_wasm_world_dispose(sl_wasm_context *context)
{
    if (context != NULL) {
        sl_wasm_storage_dispose(context);
        sl_world_destroy(&context->world);
    }
}
bool sl_wasm_timestep_valid(const sl_wasm_context *context, float dt)
{
    if (context == NULL || context->world.state == NULL || !sl_is_finite(dt) ||
        dt <= 0.0f) {
        return false;
    }
    /* Match the public step contract before entering its assertion paths. */
    const float h = dt / (float)sl_world_get_substep_count(&context->world);
    if (!sl_is_finite(h) || h <= 0.0f) {
        return false;
    }
    const float inverse_dt = 1.0f / dt;
    const float inverse_h = 1.0f / h;
    if (!sl_is_finite(inverse_dt) || inverse_dt <= 0.0f ||
        !sl_is_finite(inverse_h) || inverse_h <= 0.0f) {
        return false;
    }
    return true;
}
bool sl_wasm_world_step(sl_wasm_context *c, float dt)
{
    if (!sl_wasm_timestep_valid(c, dt)) {
        return false;
    }
    sl_world_step(&c->world, dt);
    return true;
}
bool sl_wasm_world_advance(sl_wasm_context *c, float dt, float remainder,
                           float frame_time)
{
    if (!sl_wasm_timestep_valid(c, dt) || !sl_is_finite(remainder) ||
        remainder < 0.0f || remainder >= dt || !sl_is_finite(frame_time)) {
        return false;
    }
    sl_stepper stepper;
    if (!sl_stepper_init(&stepper, dt)) {
        return false;
    }
    stepper.remainder = remainder;
    const uint32_t steps = sl_world_advance(&c->world, &stepper, frame_time);
    /* Use double solely for host debt accounting, never simulation state.
     * Two finite binary32 inputs cannot overflow this reporting accumulator. */
    const double available =
        (double)remainder + (frame_time > 0.0f ? (double)frame_time : 0.0);
    const double dropped =
        available - (double)steps * (double)dt - (double)stepper.remainder;
    c->dropped_time =
        steps == SL_STEP_COUNT_MAX && dropped > 0.0 ? dropped : 0.0;
    c->output_u32[0] = steps;
    c->output_f32[0] = stepper.remainder;
    return true;
}
double sl_wasm_dropped_time(const sl_wasm_context *c)
{
    return c->dropped_time;
}

uint32_t sl_wasm_body_count(const sl_wasm_context *context)
{
    return context != NULL && context->world.state != NULL
               ? sl_world_body_count(&context->world)
               : 0u;
}
bool sl_wasm_body_valid(const sl_wasm_context *c, uint32_t index,
                        uint32_t generation)
{
    return c != NULL && c->world.state != NULL &&
           sl_world_body_is_valid(&c->world,
                                  (sl_body_handle){ index, generation });
}
static void handle_write(sl_wasm_context *c, sl_body_handle h)
{
    c->output_u32[0] = h.index;
    c->output_u32[1] = h.generation;
}
bool sl_wasm_body_create(sl_wasm_context *c)
{
    if (c == NULL || c->world.state == NULL ||
        c->input_u32[0] > SL_BODY_STATIC) {
        return false;
    }
    const float *f = c->input_f32;
    const sl_body_desc desc = { .type = (sl_body_type)c->input_u32[0],
                                .position = { f[0], f[1] },
                                .velocity = { f[2], f[3] },
                                .mass = f[4],
                                .angle = f[5],
                                .angular_velocity = f[6],
                                .friction = f[7],
                                .restitution = f[8],
                                .shape = &c->shape };
    const sl_body_handle h = sl_world_body_create(&c->world, &desc);
    if (sl_body_handle_is_null(h)) {
        return false;
    }
    handle_write(c, h);
    return true;
}
bool sl_wasm_body_at(sl_wasm_context *c, uint32_t row)
{
    if (c == NULL || c->world.state == NULL ||
        row >= sl_world_body_count(&c->world)) {
        return false;
    }
    handle_write(c, sl_world_body_at(&c->world, row));
    return true;
}
static void shape_write(sl_wasm_context *c, const sl_shape *shape)
{
    c->output_u32[8] = (uint32_t)shape->kind;
    c->output_u32[9] =
        shape->kind == SL_SHAPE_POLYGON ? shape->polygon.count : 0u;
    c->output_f32[32] =
        shape->kind == SL_SHAPE_CIRCLE ? shape->circle.radius : 0.0f;
    for (uint32_t i = 0u; i < SL_POLYGON_VERTEX_COUNT_MAX; ++i) {
        const sl_vec2 v =
            shape->kind == SL_SHAPE_POLYGON && i < shape->polygon.count
                ? shape->polygon.vertices[i]
                : (sl_vec2){ 0.0f, 0.0f };
        c->output_f32[33u + 2u * i] = v.x;
        c->output_f32[34u + 2u * i] = v.y;
    }
}
bool sl_wasm_body_next(sl_wasm_context *c, uint32_t index, uint32_t generation)
{
    if (!sl_wasm_body_valid(c, index, generation)) {
        return false;
    }
    const sl_body_handle next =
        sl_world_body_next(&c->world, (sl_body_handle){ index, generation });
    if (sl_body_handle_is_null(next)) {
        return false;
    }
    handle_write(c, next);
    return true;
}
bool sl_wasm_body_read(sl_wasm_context *c, uint32_t index, uint32_t generation)
{
    if (!sl_wasm_body_valid(c, index, generation)) {
        return false;
    }
    const sl_body_handle h = { index, generation };
    const sl_transform t = sl_world_body_get_transform(&c->world, h);
    const sl_vec2 v = sl_world_body_get_velocity(&c->world, h);
    const sl_vec2 f = sl_world_body_get_force(&c->world, h);
    const float values[] = { t.position.x,
                             t.position.y,
                             t.rotation.c,
                             t.rotation.s,
                             v.x,
                             v.y,
                             sl_world_body_get_angle(&c->world, h),
                             sl_world_body_get_angular_velocity(&c->world, h),
                             sl_world_body_get_mass(&c->world, h),
                             sl_world_body_get_inv_mass(&c->world, h),
                             sl_world_body_get_inertia(&c->world, h),
                             sl_world_body_get_inv_inertia(&c->world, h),
                             f.x,
                             f.y,
                             sl_world_body_get_torque(&c->world, h),
                             sl_world_body_get_friction(&c->world, h),
                             sl_world_body_get_restitution(&c->world, h) };
    memcpy(c->output_f32, values, sizeof(values));
    handle_write(c, h);
    c->output_u32[2] = (uint32_t)sl_world_body_get_type(&c->world, h);
    c->output_u32[3] = sl_world_body_is_awake(&c->world, h) ? 1u : 0u;
    sl_aabb aabb = { 0 };
    c->output_u32[4] =
        sl_world_body_get_proxy_aabb(&c->world, h, &aabb) ? 1u : 0u;
    c->output_f32[17] = aabb.lower.x;
    c->output_f32[18] = aabb.lower.y;
    c->output_f32[19] = aabb.upper.x;
    c->output_f32[20] = aabb.upper.y;
    shape_write(c, sl_world_body_get_shape(&c->world, h));
    return true;
}
bool sl_wasm_body_destroy(sl_wasm_context *c, uint32_t index,
                          uint32_t generation)
{
    if (!sl_wasm_body_valid(c, index, generation)) {
        return false;
    }
    sl_world_body_destroy(&c->world, (sl_body_handle){ index, generation });
    c->geometry_generation[index] = 0u;
    return true;
}
bool sl_wasm_body_set_shape(sl_wasm_context *c, uint32_t index,
                            uint32_t generation)
{
    if (!sl_wasm_body_valid(c, index, generation) ||
        !sl_world_body_set_shape(
            &c->world, (sl_body_handle){ index, generation }, &c->shape)) {
        return false;
    }
    c->geometry_generation[index] = 0u;
    return true;
}

#define BODY_VECTOR(name)                                                      \
    bool sl_wasm_body_##name(sl_wasm_context *c, uint32_t index,               \
                             uint32_t generation, float x, float y)            \
    {                                                                          \
        return sl_wasm_body_valid(c, index, generation) &&                     \
               sl_world_body_##name(&c->world,                                 \
                                    (sl_body_handle){ index, generation },     \
                                    (sl_vec2){ x, y });                        \
    }
BODY_VECTOR(set_position)
BODY_VECTOR(set_velocity)
BODY_VECTOR(apply_force)
#undef BODY_VECTOR
#define BODY_SCALAR(name)                                                      \
    bool sl_wasm_body_##name(sl_wasm_context *c, uint32_t index,               \
                             uint32_t generation, float value)                 \
    {                                                                          \
        return sl_wasm_body_valid(c, index, generation) &&                     \
               sl_world_body_##name(                                           \
                   &c->world, (sl_body_handle){ index, generation }, value);   \
    }
BODY_SCALAR(set_angle)
BODY_SCALAR(set_angular_velocity)
BODY_SCALAR(set_mass)
BODY_SCALAR(set_friction)
BODY_SCALAR(set_restitution)
BODY_SCALAR(apply_torque)
#undef BODY_SCALAR
bool sl_wasm_body_apply_force_at_point(sl_wasm_context *c, uint32_t index,
                                       uint32_t generation, float fx, float fy,
                                       float px, float py)
{
    return sl_wasm_body_valid(c, index, generation) &&
           sl_world_body_apply_force_at_point(
               &c->world, (sl_body_handle){ index, generation },
               (sl_vec2){ fx, fy }, (sl_vec2){ px, py });
}
bool sl_wasm_body_wake(sl_wasm_context *c, uint32_t index, uint32_t generation)
{
    return sl_wasm_body_valid(c, index, generation) &&
           sl_world_body_wake(&c->world, (sl_body_handle){ index, generation });
}
void sl_wasm_shape_none(sl_wasm_context *c)
{
    c->shape = sl_shape_none();
}
bool sl_wasm_shape_circle(sl_wasm_context *c, float radius)
{
    return sl_shape_make_circle(radius, &c->shape);
}
bool sl_wasm_shape_box(sl_wasm_context *c, float x, float y)
{
    return sl_shape_make_box(x, y, &c->shape);
}
bool sl_wasm_shape_polygon(sl_wasm_context *c, uint32_t count)
{
    if (count < 3u || count > SL_POLYGON_VERTEX_COUNT_MAX) {
        return false;
    }
    sl_vec2 points[SL_POLYGON_VERTEX_COUNT_MAX] = { 0 };
    for (uint32_t i = 0u; i < count; ++i) {
        points[i] =
            (sl_vec2){ c->input_f32[2u * i], c->input_f32[2u * i + 1u] };
    }
    return sl_shape_make_polygon(points, count, &c->shape);
}
void sl_wasm_shape_read(sl_wasm_context *c)
{
    shape_write(c, &c->shape);
}

bool sl_wasm_shape_load(sl_wasm_context *c)
{
    sl_shape shape = sl_shape_none();
    if (c->input_u32[0] > SL_SHAPE_POLYGON) {
        return false;
    }
    shape.kind = (sl_shape_kind)c->input_u32[0];
    if (shape.kind == SL_SHAPE_CIRCLE) {
        shape.circle.radius = c->input_f32[0];
    } else if (shape.kind == SL_SHAPE_POLYGON) {
        shape.polygon.count = c->input_u32[1];
        if (shape.polygon.count > SL_POLYGON_VERTEX_COUNT_MAX) {
            return false;
        }
        for (uint32_t i = 0u; i < shape.polygon.count; ++i) {
            shape.polygon.vertices[i] = (sl_vec2){ c->input_f32[1u + 2u * i],
                                                   c->input_f32[2u + 2u * i] };
        }
    }
    if (!sl_shape_is_valid(&shape)) {
        return false;
    }
    c->shape = shape;
    return true;
}

void sl_wasm_shape_mass(sl_wasm_context *c)
{
    const sl_mass_data data = sl_shape_mass_data(&c->shape);
    c->output_f32[0] = data.area;
    c->output_f32[1] = data.centroid.x;
    c->output_f32[2] = data.centroid.y;
    c->output_f32[3] = data.inertia_per_unit_mass;
}
static bool transform_read(sl_wasm_context *c, sl_transform *out)
{
    const float *f = c->input_f32;
    if (!sl_is_finite(f[0]) || !sl_is_finite(f[1]) || !sl_is_finite(f[2])) {
        return false;
    }
    *out = (sl_transform){ .position = { f[0], f[1] },
                           .rotation = sl_rotation_make(f[2]) };
    return true;
}
bool sl_wasm_shape_aabb(sl_wasm_context *c)
{
    sl_transform transform;
    if (!transform_read(c, &transform)) {
        return false;
    }
    const sl_aabb bounds = sl_shape_aabb(&c->shape, transform);
    if (!sl_aabb_is_valid(bounds)) {
        return false;
    }
    c->output_f32[0] = bounds.lower.x;
    c->output_f32[1] = bounds.lower.y;
    c->output_f32[2] = bounds.upper.x;
    c->output_f32[3] = bounds.upper.y;
    return true;
}
bool sl_wasm_shape_contains(sl_wasm_context *c)
{
    sl_transform transform;
    return transform_read(c, &transform) &&
           sl_shape_contains_point(
               &c->shape, transform,
               (sl_vec2){ c->input_f32[3], c->input_f32[4] });
}
bool sl_wasm_shape_ray(sl_wasm_context *c)
{
    sl_transform transform;
    if (!transform_read(c, &transform)) {
        return false;
    }
    const sl_ray ray = { .origin = { c->input_f32[3], c->input_f32[4] },
                         .translation = { c->input_f32[5], c->input_f32[6] } };
    sl_ray_hit hit;
    if (!sl_shape_ray_cast(&c->shape, transform, ray, &hit)) {
        return false;
    }
    c->output_f32[0] = hit.fraction;
    c->output_f32[1] = hit.point.x;
    c->output_f32[2] = hit.point.y;
    c->output_f32[3] = hit.normal.x;
    c->output_f32[4] = hit.normal.y;
    return true;
}

size_t sl_wasm_adapter_bytes(const sl_wasm_context *c)
{
    if (c == NULL) {
        return 0u;
    }
    if (c->world.state != NULL) {
        return sizeof(*c) + c->buffer_bytes;
    }
    const sl_world_config config = config_read(c);
    const size_t bytes =
        sl_wasm_world_bytes(c) != 0u ? sl_wasm_storage_bytes(&config) : 0u;
    return bytes != 0u ? sizeof(*c) + bytes : 0u;
}
