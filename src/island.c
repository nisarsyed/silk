#include "world_internal.h"

#include <string.h>

#include "stats.h"

/* Union by size bounds depth; ties use the lower root slot. Compression is
 * iterative. Labels are assigned separately by minimum member slot, so a
 * larger component's root need not be its label. */
static uint32_t root_find(sl_world_state *world, uint32_t slot,
                          uint64_t *probes)
{
    uint32_t root = slot;
    for (uint32_t depth = 0u; depth < world->body_capacity; ++depth) {
        const uint32_t parent = world->island_parents[root];
        *probes += 1u;
        if (parent == root) {
            break;
        }
        root = parent;
    }
    for (uint32_t depth = 0u; depth < world->body_capacity; ++depth) {
        const uint32_t parent = world->island_parents[slot];
        *probes += 1u;
        world->island_parents[slot] = root;
        if (parent == slot) {
            break;
        }
        slot = parent;
    }
    return root;
}

static bool dynamic_slot(const sl_world_state *world, uint32_t slot)
{
    const uint32_t row = world->slots[slot].dense;
    return row != SL_BODY_DENSE_NONE &&
           world->types[row] == (uint8_t)SL_BODY_DYNAMIC;
}

static void edge_join(sl_world_state *world, uint32_t a, uint32_t b,
                      uint64_t *probes)
{
    if (!dynamic_slot(world, a) || !dynamic_slot(world, b)) {
        return;
    }
    a = root_find(world, a, probes);
    b = root_find(world, b, probes);
    if (a == b) {
        return;
    }
    if (world->island_sizes[a] < world->island_sizes[b] ||
        (world->island_sizes[a] == world->island_sizes[b] && a > b)) {
        const uint32_t swap = a;
        a = b;
        b = swap;
    }
    world->island_parents[b] = a;
    world->island_sizes[a] += world->island_sizes[b];
}

static uint32_t edge_island(const sl_world_state *world, uint32_t a, uint32_t b)
{
    const uint32_t id = dynamic_slot(world, a) ? world->body_islands[a]
                                               : world->body_islands[b];
    SL_ASSERT(id < world->island_count);
    return id;
}

void sl_islands_build(sl_world_state *world)
{
    uint64_t bodies = 0u;
    uint64_t edges = 0u;
    uint64_t probes = 0u;
    world->island_count = 0u;
    world->island_body_count = 0u;
    world->island_contact_count = 0u;
    world->island_joint_count = 0u;
    for (uint32_t slot = 0u; slot < world->body_capacity; ++slot) {
        bodies += 1u;
        world->island_parents[slot] = slot;
        world->island_sizes[slot] = 1u;
        world->body_islands[slot] = SL_BODY_DENSE_NONE;
    }
    for (uint32_t row = 0u; row < world->contact_count; ++row) {
        edges += 1u;
        const sl_contact *contact = &world->contacts[row];
        if (contact->touching) {
            edge_join(world, contact->body_a.index, contact->body_b.index,
                      &probes);
        }
    }
    for (uint32_t row = 0u; row < world->joint_count; ++row) {
        edges += 1u;
        edge_join(world, world->joint_bodies_a[row].index,
                  world->joint_bodies_b[row].index, &probes);
    }
    /* Sizes are no longer needed: root -> compact ID scratch. */
    for (uint32_t slot = 0u; slot < world->body_capacity; ++slot) {
        bodies += 1u;
        world->island_sizes[slot] = SL_BODY_DENSE_NONE;
    }
    for (uint32_t slot = 0u; slot < world->body_capacity; ++slot) {
        bodies += 1u;
        if (!dynamic_slot(world, slot)) {
            continue;
        }
        const uint32_t root = root_find(world, slot, &probes);
        uint32_t id = world->island_sizes[root];
        if (id == SL_BODY_DENSE_NONE) {
            id = world->island_count++;
            world->island_sizes[root] = id;
            memset(&world->islands[id], 0, sizeof(world->islands[id]));
            world->islands[id].root_slot = slot;
        }
        world->body_islands[slot] = id;
        world->islands[id].body_count += 1u;
    }
    for (uint32_t row = 0u; row < world->contact_count; ++row) {
        edges += 1u;
        const sl_contact *contact = &world->contacts[row];
        if (contact->touching) {
            const uint32_t id = edge_island(world, contact->body_a.index,
                                            contact->body_b.index);
            world->islands[id].contact_count += 1u;
        }
    }
    for (uint32_t row = 0u; row < world->joint_count; ++row) {
        edges += 1u;
        const uint32_t id = edge_island(world, world->joint_bodies_a[row].index,
                                        world->joint_bodies_b[row].index);
        world->islands[id].joint_count += 1u;
    }
    /* Parent and size scratch become range write cursors after membership is
     * complete. Stable source scans preserve constraint order within islands.
     */
    for (uint32_t id = 0u; id < world->island_count; ++id) {
        sl_island *island = &world->islands[id];
        island->body_offset = world->island_body_count;
        island->contact_offset = world->island_contact_count;
        island->joint_offset = world->island_joint_count;
        world->island_body_count += island->body_count;
        world->island_contact_count += island->contact_count;
        world->island_joint_count += island->joint_count;
        world->island_parents[id] = island->body_offset;
        world->island_sizes[id] = island->contact_offset;
    }
    for (uint32_t slot = 0u; slot < world->body_capacity; ++slot) {
        bodies += 1u;
        const uint32_t id = world->body_islands[slot];
        if (id != SL_BODY_DENSE_NONE) {
            world->island_bodies[world->island_parents[id]++] = slot;
        }
    }
    for (uint32_t row = 0u; row < world->contact_count; ++row) {
        edges += 1u;
        const sl_contact *contact = &world->contacts[row];
        if (contact->touching) {
            const uint32_t id = edge_island(world, contact->body_a.index,
                                            contact->body_b.index);
            world->island_contacts[world->island_sizes[id]++] = row;
        }
    }
    for (uint32_t id = 0u; id < world->island_count; ++id) {
        world->island_parents[id] = world->islands[id].joint_offset;
    }
    for (uint32_t row = 0u; row < world->joint_count; ++row) {
        edges += 1u;
        const uint32_t id = edge_island(world, world->joint_bodies_a[row].index,
                                        world->joint_bodies_b[row].index);
        world->island_joints[world->island_parents[id]++] = row;
    }
    SL_WORK_ADD(world, graph_body_visits, bodies);
    SL_WORK_ADD(world, graph_constraint_visits, edges);
    SL_WORK_ADD(world, graph_parent_probes, probes);
}
