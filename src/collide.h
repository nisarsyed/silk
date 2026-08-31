#ifndef SILK_COLLIDE_H
#define SILK_COLLIDE_H

#include "silk/contact.h"
#include "silk/shape.h"

/* Internal narrow-phase dispatch. Inputs must be valid shapes and finite
 * rigid transforms. NONE returns an empty manifold. */
sl_manifold sl_collide_shapes(const sl_shape *shape_a, sl_transform transform_a,
                              const sl_shape *shape_b,
                              sl_transform transform_b);

#endif /* SILK_COLLIDE_H */
