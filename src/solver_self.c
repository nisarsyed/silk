#include "vertex_block.h"

/* The sweep with self-contact terms; see vertex_block.h for why it has its
 * own translation unit. Same order as the contact-free sweep: cloths, then
 * colors.
 *
 * A contact pair can join two vertices of the same color, which the mesh
 * coloring cannot prevent. Solving a color in place would let one block
 * read a partner's position while another thread writes it. Each color is
 * therefore solved into position_next and committed afterwards: within a
 * color, contact partners see each other's start-of-color positions, as in
 * a Jacobi step, while mesh neighbours (other colors) see the latest
 * positions as before. The result is the same for every partition and
 * thread count. */
static void sl_sweep_color_self(uint32_t begin, uint32_t end, void *context)
{
    const sl_sweep_task *task = context;
    sl_world_state *s = task->state;
    for (uint32_t k = begin; k < end; k++) {
        const uint32_t i = task->order[k];
        if (s->pinned[i] == 0u) {
            s->position_next[i] =
                sl_vertex_block(s, task->rec, i, task->h, true);
        }
    }
}

void sl_solver_sweep_self(sl_world_state *s, float h)
{
    SL_ASSERT(h > 0.0f);
    for (uint32_t c = 0; c < s->cloth_count; c++) {
        const sl_cloth_record *rec = &s->cloths[c];
        for (uint32_t color = 0; color < rec->color_count; color++) {
            const uint32_t begin =
                rec->vertex_offset + rec->color_offsets[color];
            const uint32_t end =
                rec->vertex_offset + rec->color_offsets[color + 1u];
            sl_sweep_task task = { s, rec, s->color_order + begin, h };
            sl_parallel(s, sl_sweep_color_self, end - begin,
                        SL_PARALLEL_GRAIN_BLOCK, &task);
            for (uint32_t k = begin; k < end; k++) {
                const uint32_t i = s->color_order[k];
                if (s->pinned[i] == 0u) {
                    s->position[i] = s->position_next[i];
                }
            }
        }
    }
}
