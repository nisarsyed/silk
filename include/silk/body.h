#ifndef SILK_BODY_H
#define SILK_BODY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* index means nothing unless generation != 0; generations are never
 * issued as 0, so any zeroed handle is the null handle. */
/* Scoped to the originating world lifetime. Equal values can exist in other
 * worlds or after destroy/reinitialize; cross-world handles are invalid usage.
 */
typedef struct sl_body_handle {
    uint32_t index;
    uint32_t generation;
} sl_body_handle;

static inline sl_body_handle sl_body_handle_null(void)
{
    sl_body_handle h = { UINT32_MAX, 0 };
    return h;
}

static inline bool sl_body_handle_is_null(sl_body_handle h)
{
    return h.generation == 0;
}

#ifdef __cplusplus
}
#endif

#endif /* SILK_BODY_H */
