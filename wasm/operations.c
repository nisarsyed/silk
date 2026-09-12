#include "context.h"

static bool live(const sl_wasm_context *c)
{
    return c != NULL && c->world.state != NULL;
}
bool sl_wasm_joint_valid(const sl_wasm_context *c, uint32_t index,
                         uint32_t generation)
{
    return live(c) && sl_world_joint_is_valid(
                          &c->world, (sl_joint_handle){ index, generation });
}
uint32_t sl_wasm_joint_count(const sl_wasm_context *c)
{
    return live(c) ? sl_world_joint_count(&c->world) : 0u;
}
bool sl_wasm_joint_create(sl_wasm_context *c)
{
    if (!live(c) || c->input_u32[0] > SL_JOINT_REVOLUTE ||
        c->input_u32[5] > 1u) {
        return false;
    }
    const uint32_t *u = c->input_u32;
    const float *f = c->input_f32;
    const sl_joint_desc desc = { .kind = (sl_joint_kind)u[0],
                                 .body_a = { u[1], u[2] },
                                 .body_b = { u[3], u[4] },
                                 .local_anchor_a = { f[0], f[1] },
                                 .local_anchor_b = { f[2], f[3] },
                                 .collide_connected = u[5] != 0u,
                                 .distance = { .length = f[4] } };
    const sl_joint_handle h = sl_world_joint_create(&c->world, &desc);
    if (sl_joint_handle_is_null(h)) {
        return false;
    }
    c->output_u32[0] = h.index;
    c->output_u32[1] = h.generation;
    return true;
}
bool sl_wasm_joint_at(sl_wasm_context *c, uint32_t row)
{
    if (!live(c) || row >= sl_world_joint_count(&c->world)) {
        return false;
    }
    const sl_joint_handle h = sl_world_joint_at(&c->world, row);
    c->output_u32[0] = h.index;
    c->output_u32[1] = h.generation;
    return true;
}
bool sl_wasm_joint_destroy(sl_wasm_context *c, uint32_t index,
                           uint32_t generation)
{
    if (!sl_wasm_joint_valid(c, index, generation)) {
        return false;
    }
    sl_world_joint_destroy(&c->world, (sl_joint_handle){ index, generation });
    return true;
}
void sl_wasm_joint_pack(float *f, uint32_t *u, uint32_t n, uint32_t row,
                        const sl_world *world, sl_joint_handle h)
{
    const sl_joint_desc d = sl_world_joint_get_desc(world, h);
    const sl_vec2 impulse = sl_world_joint_get_linear_impulse(world, h);
    const float floats[] = { d.local_anchor_a.x,
                             d.local_anchor_a.y,
                             d.local_anchor_b.x,
                             d.local_anchor_b.y,
                             d.kind == SL_JOINT_DISTANCE ? d.distance.length
                                                         : 0.0f,
                             impulse.x,
                             impulse.y };
    const uint32_t words[] = { h.index,
                               h.generation,
                               (uint32_t)d.kind,
                               d.body_a.index,
                               d.body_a.generation,
                               d.body_b.index,
                               d.body_b.generation,
                               d.collide_connected ? 1u : 0u };
    for (uint32_t i = 0u; i < 7u; ++i) {
        f[i * n + row] = floats[i];
    }
    for (uint32_t i = 0u; i < 8u; ++i) {
        u[i * n + row] = words[i];
    }
}
bool sl_wasm_joint_read(sl_wasm_context *c, uint32_t index, uint32_t generation)
{
    if (!sl_wasm_joint_valid(c, index, generation)) {
        return false;
    }
    sl_wasm_joint_pack(c->output_f32, c->output_u32, 1u, 0u, &c->world,
                       (sl_joint_handle){ index, generation });
    return true;
}
uint32_t sl_wasm_contact_count(const sl_wasm_context *c)
{
    return live(c) ? sl_world_contact_count(&c->world) : 0u;
}
void sl_wasm_contact_pack(float *f, uint32_t *u, uint32_t n, uint32_t row,
                          const sl_contact *c)
{
    f[row] = c->friction;
    f[n + row] = c->restitution;
    f[2u * n + row] = c->manifold.normal.x;
    f[3u * n + row] = c->manifold.normal.y;
    const uint32_t words[] = { c->body_a.index,       c->body_a.generation,
                               c->body_b.index,       c->body_b.generation,
                               c->touching ? 1u : 0u, c->manifold.point_count };
    for (uint32_t i = 0u; i < 6u; ++i) {
        u[i * n + row] = words[i];
    }
    for (uint32_t i = 0u; i < SL_MANIFOLD_POINT_COUNT_MAX; ++i) {
        const sl_manifold_point p = i < c->manifold.point_count
                                        ? c->manifold.points[i]
                                        : (sl_manifold_point){ 0 };
        const float values[] = { p.anchor_a.x,      p.anchor_a.y,
                                 p.anchor_b.x,      p.anchor_b.y,
                                 p.point.x,         p.point.y,
                                 p.separation,      p.normal_impulse,
                                 p.tangent_impulse, p.normal_velocity };
        for (uint32_t j = 0u; j < 10u; ++j) {
            f[(4u + 10u * i + j) * n + row] = values[j];
        }
        u[(6u + 2u * i) * n + row] = p.id;
        u[(7u + 2u * i) * n + row] = p.persisted ? 1u : 0u;
    }
}
bool sl_wasm_contact_read(sl_wasm_context *c, uint32_t row)
{
    if (!live(c) || row >= sl_world_contact_count(&c->world)) {
        return false;
    }
    sl_wasm_contact_pack(c->output_f32, c->output_u32, 1u, 0u,
                         sl_world_contact_at(&c->world, row));
    return true;
}
static void query_write(sl_wasm_context *c, sl_query_result result,
                        uint32_t capacity)
{
    const uint32_t written = result.count < capacity ? result.count : capacity;
    for (uint32_t i = 0u; i < written; ++i) {
        c->query_u32[i] = c->query_handles[i].index;
        c->query_u32[c->config.body_capacity + i] =
            c->query_handles[i].generation;
    }
    c->output_u32[0] = result.count;
    c->output_u32[1] = result.truncated ? 1u : 0u;
    c->output_u32[2] = written;
}
bool sl_wasm_query_point(sl_wasm_context *c, uint32_t mask, uint32_t capacity)
{
    if (!live(c) || capacity > c->config.body_capacity) {
        return false;
    }
    sl_query_result result;
    if (!sl_world_query_point(&c->world,
                              (sl_vec2){ c->input_f32[0], c->input_f32[1] },
                              mask, c->query_handles, capacity, &result)) {
        return false;
    }
    query_write(c, result, capacity);
    return true;
}
bool sl_wasm_query_aabb(sl_wasm_context *c, uint32_t mask, uint32_t capacity)
{
    if (!live(c) || capacity > c->config.body_capacity) {
        return false;
    }
    const sl_aabb bounds = { .lower = { c->input_f32[0], c->input_f32[1] },
                             .upper = { c->input_f32[2], c->input_f32[3] } };
    sl_query_result result;
    if (!sl_world_query_aabb(&c->world, bounds, mask, c->query_handles,
                             capacity, &result)) {
        return false;
    }
    query_write(c, result, capacity);
    return true;
}
bool sl_wasm_query_ray(sl_wasm_context *c, uint32_t mask)
{
    if (!live(c)) {
        return false;
    }
    const sl_ray ray = { .origin = { c->input_f32[0], c->input_f32[1] },
                         .translation = { c->input_f32[2], c->input_f32[3] } };
    sl_query_ray_result result;
    if (!sl_world_query_ray(&c->world, ray, mask, &result)) {
        return false;
    }
    c->output_u32[0] = result.hit ? 1u : 0u;
    c->output_u32[1] = result.body.index;
    c->output_u32[2] = result.body.generation;
    c->output_f32[0] = result.geometry.fraction;
    c->output_f32[1] = result.geometry.point.x;
    c->output_f32[2] = result.geometry.point.y;
    c->output_f32[3] = result.geometry.normal.x;
    c->output_f32[4] = result.geometry.normal.y;
    return true;
}
bool sl_wasm_island_read(sl_wasm_context *c, uint32_t index,
                         uint32_t generation)
{
    if (!sl_wasm_body_valid(c, index, generation)) {
        return false;
    }
    sl_island_stats result;
    if (!sl_world_body_get_island_stats(
            &c->world, (sl_body_handle){ index, generation }, &result)) {
        return false;
    }
    c->output_u32[0] = result.id;
    c->output_u32[1] = result.dynamic_body_count;
    c->output_u32[2] = result.contact_count;
    c->output_u32[3] = result.joint_count;
    return true;
}
void sl_wasm_work_pack(uint64_t *out, const sl_world_work *w)
{
    out[0] = w->tree_node_visits;
    out[1] = w->pair_candidates;
    out[2] = w->pair_probes;
    out[3] = w->wake_visits;
    out[4] = w->body_wakes;
    out[5] = w->body_sleeps;
    out[6] = w->graph_body_visits;
    out[7] = w->graph_constraint_visits;
    out[8] = w->graph_parent_probes;
    out[9] = w->proxy_creates;
    out[10] = w->proxy_destroys;
    out[11] = w->proxy_moves;
    out[12] = w->contact_drops;
}
bool sl_wasm_stats_read(sl_wasm_context *c)
{
    if (!live(c)) {
        return false;
    }
    const sl_world_stats s = sl_world_get_stats(&c->world);
    sl_wasm_work_pack(c->work, &s.step.work);
    sl_wasm_work_pack(c->work + 13, &s.cumulative);
    const uint32_t words[] = { s.awake_dynamic_count,
                               s.sleeping_dynamic_count,
                               s.body_count,
                               s.body_capacity,
                               s.contact_count,
                               s.contact_capacity,
                               s.joint_count,
                               s.joint_capacity,
                               s.pair_count,
                               s.pair_capacity,
                               s.body_count_high,
                               s.contact_count_high,
                               s.joint_count_high,
                               s.step.dynamic_body_count,
                               s.step.kinematic_body_count,
                               s.step.contact_constraint_count,
                               s.step.joint_constraint_count,
                               s.step.island_executed_count,
                               s.step.island_skipped_count,
                               s.step.island_count,
                               s.step.island_body_count_max,
                               s.step.substep_count,
                               sl_world_contact_drop_count(&c->world) };
    for (uint32_t i = 0u; i < sizeof(words) / sizeof(words[0]); ++i) {
        c->output_u32[i] = words[i];
    }
    return true;
}
bool sl_wasm_memory_read(sl_wasm_context *c)
{
    if (!live(c)) {
        return false;
    }
    sl_world_memory_breakdown m;
    if (!sl_world_memory_breakdown_get(&c->config, &m)) {
        return false;
    }
    /* Configured maximum storage is below 2^32 on every supported ABI. */
    const size_t bytes[] = { m.sleep_bytes,       m.island_bytes,
                             m.world_state_bytes, m.body_bytes,
                             m.broadphase_bytes,  m.contact_bytes,
                             m.pair_bytes,        m.contact_solver_bytes,
                             m.joint_bytes,       m.padding_bytes,
                             m.arena_bytes,       m.world_bytes };
    for (uint32_t i = 0u; i < sizeof(bytes) / sizeof(bytes[0]); ++i) {
        SL_ASSERT(bytes[i] <= UINT32_MAX);
        c->output_u32[i] = (uint32_t)bytes[i];
    }
    return true;
}
