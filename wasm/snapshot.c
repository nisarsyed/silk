#include "context.h"
#include <stdlib.h>
#include <string.h>

_Static_assert(sizeof(float) == 4u && sizeof(uint32_t) == 4u,
               "Serialized snapshot columns require four-byte words");
_Static_assert(sizeof(sl_body_handle) == 8u &&
                   _Alignof(sl_body_handle) <= _Alignof(uint32_t),
               "Query handle slices require two four-byte-aligned words");

/* Per body: 7 float + 5 word transform/state columns; 17 float + 2 word
 * geometry columns, one private generation, query handles and two query words.
 * Contacts: 24 float + 10 word columns. Joints: 7 float + 8 word columns.
 * wasm32 totals are exactly 144 B/body + 136 B/contact + 60 B/joint.
 */
size_t sl_wasm_storage_bytes(const sl_world_config *config)
{
    if (sl_world_memory_bytes(config) == 0u) {
        return 0u;
    }
    const size_t body_stride =
        24u * sizeof(float) + 10u * sizeof(uint32_t) + sizeof(sl_body_handle);
    const size_t contact_stride = 24u * sizeof(float) + 10u * sizeof(uint32_t);
    const size_t joint_stride = 7u * sizeof(float) + 8u * sizeof(uint32_t);
    const size_t contacts = config->contact_capacity != 0u
                                ? config->contact_capacity
                                : 4u * config->body_capacity;
    if (config->body_capacity > SIZE_MAX / body_stride) {
        return 0u;
    }
    size_t total = (size_t)config->body_capacity * body_stride;
    if (contacts > (SIZE_MAX - total) / contact_stride) {
        return 0u;
    }
    total += contacts * contact_stride;
    if (config->joint_capacity > (SIZE_MAX - total) / joint_stride) {
        return 0u;
    }
    return total + (size_t)config->joint_capacity * joint_stride;
}
bool sl_wasm_storage_init(sl_wasm_context *c)
{
    const size_t bytes = sl_wasm_storage_bytes(&c->config);
    if (bytes == 0u || c->buffers != NULL) {
        return false;
    }
    unsigned char *base = calloc(1u, bytes);
    if (base == NULL) {
        return false;
    }
    unsigned char *cursor = base;
    const size_t bodies = c->config.body_capacity,
                 contacts = c->config.contact_capacity;
    const size_t joints = c->config.joint_capacity;
#define SL_BUFFER(member, count, type)                                         \
    c->member = (type *)cursor;                                                \
    cursor += (count) * sizeof(type)
    SL_BUFFER(body_f32, 7u * bodies, float);
    SL_BUFFER(body_u32, 5u * bodies, uint32_t);
    SL_BUFFER(geometry_f32, 17u * bodies, float);
    SL_BUFFER(geometry_u32, 2u * bodies, uint32_t);
    SL_BUFFER(geometry_generation, bodies, uint32_t);
    SL_BUFFER(query_handles, bodies, sl_body_handle);
    SL_BUFFER(query_u32, 2u * bodies, uint32_t);
    SL_BUFFER(contact_f32, 24u * contacts, float);
    SL_BUFFER(contact_u32, 10u * contacts, uint32_t);
    SL_BUFFER(joint_f32, 7u * joints, float);
    SL_BUFFER(joint_u32, 8u * joints, uint32_t);
#undef SL_BUFFER
    SL_ASSERT((size_t)(cursor - base) == bytes);
    c->buffers = base;
    c->buffer_bytes = bytes;
    return true;
}
void sl_wasm_storage_dispose(sl_wasm_context *c)
{
    free(c->buffers);
    c->buffers = NULL;
    c->buffer_bytes = 0u;
}
#define VIEW(name, member, type)                                               \
    const type *sl_wasm_##name(const sl_wasm_context *c)                       \
    {                                                                          \
        return c != NULL && c->buffers != NULL ? c->member : NULL;             \
    }
VIEW(snapshot_f32, body_f32, float)
VIEW(snapshot_u32, body_u32, uint32_t)
VIEW(geometry_f32, geometry_f32, float)
VIEW(geometry_u32, geometry_u32, uint32_t)
VIEW(contacts_f32, contact_f32, float)
VIEW(contacts_u32, contact_u32, uint32_t)
VIEW(joints_f32, joint_f32, float)
VIEW(joints_u32, joint_u32, uint32_t)
VIEW(query_u32, query_u32, uint32_t)
#undef VIEW
const uint64_t *sl_wasm_work_data(const sl_wasm_context *c)
{
    return c->work;
}

static void geometry_refresh(sl_wasm_context *c, sl_body_handle body)
{
    const uint32_t n = c->config.body_capacity, row = body.index;
    const sl_shape *shape = sl_world_body_get_shape(&c->world, body);
    const uint32_t count =
        shape->kind == SL_SHAPE_POLYGON ? shape->polygon.count : 0u;
    c->geometry_u32[row] = (uint32_t)shape->kind;
    c->geometry_u32[n + row] = count;
    c->geometry_f32[row] =
        shape->kind == SL_SHAPE_CIRCLE ? shape->circle.radius : 0.0f;
    for (uint32_t i = 0u; i < SL_POLYGON_VERTEX_COUNT_MAX; ++i) {
        const sl_vec2 v =
            i < count ? shape->polygon.vertices[i] : (sl_vec2){ 0.0f, 0.0f };
        c->geometry_f32[(1u + 2u * i) * n + row] = v.x;
        c->geometry_f32[(2u + 2u * i) * n + row] = v.y;
    }
    c->geometry_generation[row] = body.generation;
}
bool sl_wasm_snapshot_refresh(sl_wasm_context *c, uint32_t diagnostics)
{
    if (c == NULL || c->world.state == NULL || diagnostics > 1u) {
        return false;
    }
    const uint32_t n = c->config.body_capacity,
                   count = sl_world_body_count(&c->world);
    uint32_t geometry_updates = 0u;
    for (uint32_t row = 0u; row < count; ++row) {
        const sl_body_handle body = sl_world_body_at(&c->world, row);
        const sl_transform t = sl_world_body_get_transform(&c->world, body);
        const sl_vec2 v = sl_world_body_get_velocity(&c->world, body);
        c->body_f32[row] = t.position.x;
        c->body_f32[n + row] = t.position.y;
        c->body_f32[2u * n + row] = t.rotation.c;
        c->body_f32[3u * n + row] = t.rotation.s;
        c->body_f32[4u * n + row] = v.x;
        c->body_f32[5u * n + row] = v.y;
        c->body_f32[6u * n + row] =
            sl_world_body_get_angular_velocity(&c->world, body);
        c->body_u32[row] = body.index;
        c->body_u32[n + row] = body.generation;
        c->body_u32[2u * n + row] =
            (uint32_t)sl_world_body_get_type(&c->world, body);
        c->body_u32[3u * n + row] =
            sl_world_body_is_awake(&c->world, body) ? 1u : 0u;
        sl_island_stats island;
        c->body_u32[4u * n + row] =
            diagnostics != 0u &&
                    sl_world_body_get_island_stats(&c->world, body, &island)
                ? island.id
                : UINT32_MAX;
        if (c->geometry_generation[body.index] != body.generation) {
            geometry_refresh(c, body);
            ++geometry_updates;
        }
    }
    const uint32_t contacts =
        diagnostics != 0u ? sl_world_contact_count(&c->world) : 0u;
    const uint32_t joints =
        diagnostics != 0u ? sl_world_joint_count(&c->world) : 0u;
    for (uint32_t row = 0u; row < contacts; ++row) {
        sl_wasm_contact_pack(c->contact_f32, c->contact_u32,
                             c->config.contact_capacity, row,
                             sl_world_contact_at(&c->world, row));
    }
    for (uint32_t row = 0u; row < joints; ++row) {
        sl_wasm_joint_pack(c->joint_f32, c->joint_u32, c->config.joint_capacity,
                           row, &c->world, sl_world_joint_at(&c->world, row));
    }
    c->output_u32[0] = count;
    c->output_u32[1] = contacts;
    c->output_u32[2] = joints;
    c->output_u32[3] = geometry_updates;
    return true;
}
