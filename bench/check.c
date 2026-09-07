/* Developer-tool geometry checks, separate from the engine CTest runner. */
#include "quality.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static bool near(float actual, float expected)
{
    if (!isfinite(actual) || sl_abs(actual - expected) > 1e-5f) {
        fprintf(stderr, "quality geometry: expected %.9g, got %.9g\n",
                (double)expected, (double)actual);
        return false;
    }
    return true;
}
int main(void)
{
    sl_shape circle = sl_shape_none(), box = sl_shape_none(),
             small = sl_shape_none();
    if (!sl_shape_make_circle(1.0f, &circle) ||
        !sl_shape_make_box(1.0f, 1.0f, &box) ||
        !sl_shape_make_box(0.5f, 0.5f, &small)) {
        return EXIT_FAILURE;
    }
    const sl_transform origin =
        sl_transform_make((sl_vec2){ 0.0f, 0.0f }, sl_rotation_identity());
    const sl_transform shifted =
        sl_transform_make((sl_vec2){ 1.5f, 0.0f }, sl_rotation_identity());
    const sl_transform corner =
        sl_transform_make((sl_vec2){ 2.0f, 2.0f }, sl_rotation_identity());
    const sl_transform turned = sl_transform_make(
        (sl_vec2){ 0.0f, 0.0f }, sl_rotation_make(0.25f * SL_PI));
    bool valid =
        near(sl_bench_penetration(&circle, origin, &circle, shifted), 0.5f);
    valid =
        near(sl_bench_penetration(&box, origin, &box, shifted), 0.5f) && valid;
    valid = near(sl_bench_penetration(&circle, shifted, &box, origin), 0.5f) &&
            valid;
    valid = near(sl_bench_penetration(&box, origin, &circle, shifted), 0.5f) &&
            valid;
    valid = near(sl_bench_penetration(&box, origin, &circle, corner), 0.0f) &&
            valid;
    valid =
        near(sl_bench_penetration(&box, origin, &small, origin), 1.5f) && valid;
    valid = near(sl_bench_penetration(&box, origin, &small, turned),
                 1.0f + sqrtf(0.5f)) &&
            valid;
    return valid ? EXIT_SUCCESS : EXIT_FAILURE;
}
