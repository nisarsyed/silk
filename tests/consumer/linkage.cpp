/* Verify the existing extern-C boundary; this is not a C++ engine wrapper. */
#include <silk/assert.h>
#include <silk/body.h>
#include <silk/contact.h>
#include <silk/joint.h>
#include <silk/math.h>
#include <silk/query.h>
#include <silk/shape.h>
#include <silk/silk.h>
#include <silk/step.h>
#include <silk/world.h>

int main()
{
    sl_world world = {};
    sl_world_config config = {};
    config.body_capacity = 1u;
    if (sl_version() != ((SL_VERSION_MAJOR << 16) | (SL_VERSION_MINOR << 8) |
                         SL_VERSION_PATCH) ||
        !sl_world_init(&world, &config)) {
        return 1;
    }
    sl_shape shape = sl_shape_none();
    const bool made = sl_shape_make_circle(0.25f, &shape);
    sl_body_desc desc = {};
    desc.mass = 1.0f;
    desc.shape = &shape;
    const sl_body_handle body = sl_world_body_create(&world, &desc);
    sl_world_step(&world, 1.0f / 60.0f);
    sl_query_result matches = {};
    sl_body_handle found = sl_body_handle_null();
    const bool valid =
        made && !sl_body_handle_is_null(body) &&
        sl_world_query_point(&world, sl_vec2_make(0.0f, 0.0f), SL_QUERY_DYNAMIC,
                             &found, 1u, &matches) &&
        matches.count == 1u;
    sl_world_destroy(&world);
    return valid ? 0 : 1;
}
