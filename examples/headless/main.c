/* Public, allocation-free stepping example. The application chooses metres,
 * kilograms and seconds here; Silk itself does not impose a unit system. */
#include <silk/query.h>
#include <silk/silk.h>
#include <silk/step.h>

#include <math.h>
#include <stdio.h>

static bool run_example(sl_world *world)
{
    const sl_world_config invalid = { 0 };
    if (sl_world_init(world, &invalid)) {
        return false;
    }
    /* Zero settings select documented defaults; opt in to joints and sleep. */
    const sl_world_config config = { .body_capacity = 3u,
                                     .contact_capacity = 8u,
                                     .joint_capacity = 1u,
                                     .gravity = { 0.0f, -9.81f },
                                     .sleep_enabled = true };
    if (!sl_world_init(world, &config)) {
        return false;
    }
    sl_shape floor = sl_shape_none(), ball = sl_shape_none();
    if (!sl_shape_make_box(4.0f, 0.5f, &floor) ||
        !sl_shape_make_circle(0.25f, &ball)) {
        return false;
    }
    const sl_body_desc ground_desc = { .type = SL_BODY_STATIC,
                                       .position = { 0.0f, -0.5f },
                                       .shape = &floor,
                                       .friction = 0.6f };
    const sl_body_handle ground = sl_world_body_create(world, &ground_desc);
    const sl_body_desc ball_desc = { .mass = 1.0f,
                                     .position = { -0.5f, 2.0f },
                                     .shape = &ball,
                                     .friction = 0.6f,
                                     .restitution = 0.0f };
    const sl_body_handle a = sl_world_body_create(world, &ball_desc);
    sl_body_desc other_desc = ball_desc;
    other_desc.position.x = 0.5f;
    const sl_body_handle b = sl_world_body_create(world, &other_desc);
    if (sl_body_handle_is_null(ground) || sl_body_handle_is_null(a) ||
        sl_body_handle_is_null(b)) {
        return false;
    }
    /* Pool exhaustion and invalid input are recoverable failures. */
    if (!sl_body_handle_is_null(sl_world_body_create(world, &ball_desc)) ||
        sl_world_body_set_velocity(world, a, sl_vec2_make(NAN, 0.0f))) {
        return false;
    }
    const sl_joint_desc joint_desc = { .kind = SL_JOINT_DISTANCE,
                                       .body_a = a,
                                       .body_b = b,
                                       .distance = { .length = 1.0f },
                                       .collide_connected = true };
    const sl_joint_handle joint = sl_world_joint_create(world, &joint_desc);
    if (sl_joint_handle_is_null(joint) ||
        !sl_joint_handle_is_null(sl_world_joint_create(world, &joint_desc))) {
        return false;
    }
    const float dt = 1.0f / 60.0f;
    for (uint32_t step = 0u; step < 600u; ++step) {
        sl_world_step(world, dt);
    }
    const sl_world_stats stats = sl_world_get_stats(world);
    if (stats.cumulative.contact_drops != 0u ||
        sl_world_body_is_awake(world, a) || sl_world_body_is_awake(world, b)) {
        return false;
    }
    /* Contact and shape pointers are consumed before mutation. Contact
     * impulses remain cached values while the bodies sleep. */
    uint32_t points = 0u;
    for (uint32_t row = 0u; row < sl_world_contact_count(world); ++row) {
        const sl_contact *contact = sl_world_contact_at(world, row);
        points += contact->manifold.point_count;
    }
    const sl_vec2 position = sl_world_body_get_position(world, a);
    if (!sl_vec2_is_finite(position) ||
        sl_world_body_get_shape(world, a)->kind != SL_SHAPE_CIRCLE) {
        return false;
    }
    sl_body_handle found[1];
    sl_query_result matches = { 0 };
    const sl_aabb bounds = { { -4.0f, -1.0f }, { 4.0f, 3.0f } };
    if (!sl_world_query_aabb(world, bounds, SL_QUERY_DYNAMIC, found, 1u,
                             &matches) ||
        matches.count != 2u || !matches.truncated) {
        return false;
    }
    /* A zero-capacity query counts all matches without a result buffer. */
    if (!sl_world_query_aabb(world, bounds, SL_QUERY_DYNAMIC, NULL, 0u,
                             &matches) ||
        matches.count != 2u || !matches.truncated) {
        return false;
    }
    if (!sl_world_query_point(world, position, SL_QUERY_DYNAMIC, found, 1u,
                              &matches) ||
        matches.count != 1u || matches.truncated) {
        return false;
    }
    const sl_ray ray = { .origin = { -3.0f, -0.25f },
                         .translation = { 6.0f, 0.0f } };
    sl_query_ray_result hit = { 0 };
    /* This ray starts inside the floor: that shape is deliberately missed. */
    if (!sl_world_query_ray(world, ray, SL_QUERY_ALL, &hit) || hit.hit) {
        return false;
    }
    if (!sl_world_body_wake(world, a) || !sl_world_body_is_awake(world, b)) {
        return false;
    }
    printf("Silk %d.%d.%d: %u bodies, %u contact points; query truncation and "
           "component wake verified\n",
           SL_VERSION_MAJOR, SL_VERSION_MINOR, SL_VERSION_PATCH,
           stats.body_count, points);
    sl_world_reset(world);
    if (sl_world_body_is_valid(world, a) ||
        sl_world_joint_is_valid(world, joint) ||
        sl_world_body_count(world) != 0u) {
        return false;
    }
    return true;
}

int main(void)
{
    sl_world world = { 0 }; /* Never copy a live owning shell. */
    const bool ok = run_example(&world);
    sl_world_destroy(&world); /* One cleanup path, including partial setup. */
    sl_world_destroy(&world); /* Repeated destruction is safe. */
    if (!ok) {
        fprintf(stderr, "Silk example: initialization, capacity or simulation "
                        "check failed\n");
    }
    return ok ? 0 : 1;
}
