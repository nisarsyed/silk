#include "vertex_block.h"

/* The sweep with self-contact terms; see vertex_block.h for why it has its
 * own translation unit. Same order as the contact-free sweep: cloths,
 * colors, then ascending vertices. */
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
            for (uint32_t k = begin; k < end; k++) {
                const uint32_t i = s->color_order[k];
                if (s->pinned[i] == 0u) {
                    sl_vertex_block(s, rec, i, h, true);
                }
            }
        }
    }
}
