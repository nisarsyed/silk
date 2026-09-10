/* Fixed public-query workloads, timed separately from simulation steps.
 * Build against either measured library; identical outputs pin query order. */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <silk/query.h>

#define BODY_COUNT 1024u
#define QUERY_COUNT 65536u
#define RESULT_CAPACITY 8u

typedef struct query_sample {
    sl_query_result result;
    sl_body_handle bodies[RESULT_CAPACITY];
} query_sample;

static bool queries_run(const sl_world *world, bool broad,
                        query_sample *samples)
{
    for (uint32_t i = 0u; i < QUERY_COUNT; ++i) {
        /* Binary-exact grid gaps overlap fat proxies but miss tight circles.
         * Alternate with body centers so the workload also returns handles. */
        const sl_vec2 point = { (float)(i % 32u) * 0.125f +
                                    ((i & 1u) != 0u ? 0.0625f : 0.0f),
                                (float)((i / 32u) % 32u) * 0.125f };
        const bool valid =
            broad
                ? sl_world_query_aabb(
                      world, (sl_aabb){ { -1.0f, -1.0f }, { 5.0f, 5.0f } }, 0u,
                      samples[i].bodies, RESULT_CAPACITY, &samples[i].result)
                : sl_world_query_point(world, point, 0u, samples[i].bodies,
                                       RESULT_CAPACITY, &samples[i].result);
        if (!valid) {
            return false;
        }
    }
    return true;
}

static bool measure(const sl_world *world, bool broad, query_sample *samples)
{
    if (!queries_run(world, broad, samples)) {
        return false;
    }
    struct timespec start, end;
    if (timespec_get(&start, TIME_UTC) != TIME_UTC ||
        !queries_run(world, broad, samples) ||
        timespec_get(&end, TIME_UTC) != TIME_UTC) {
        return false;
    }
    const double elapsed = difftime(end.tv_sec, start.tv_sec) * 1000.0 +
                           ((double)end.tv_nsec - (double)start.tv_nsec) * 1e-6;
    if (!isfinite(elapsed) || elapsed < 0.0 || start.tv_nsec < 0L ||
        start.tv_nsec >= 1000000000L || end.tv_nsec < 0L ||
        end.tv_nsec >= 1000000000L) {
        return false;
    }
    uint64_t digest = UINT64_C(14695981039346656037);
    for (uint32_t i = 0u; i < QUERY_COUNT; ++i) {
        const query_sample *sample = &samples[i];
        const uint32_t expected =
            broad ? BODY_COUNT : ((i & 1u) == 0u ? 1u : 0u);
        if (sample->result.count != expected ||
            sample->result.truncated != broad) {
            return false;
        }
        digest = (digest ^ expected) * UINT64_C(1099511628211);
        const uint32_t count =
            expected < RESULT_CAPACITY ? expected : RESULT_CAPACITY;
        for (uint32_t j = 0u; j < count; ++j) {
            /* Creation permutes grid positions, while result slots stay sorted.
             */
            const uint32_t slot =
                broad ? j : ((i % BODY_COUNT) * 721u) % BODY_COUNT;
            if (sample->bodies[j].index != slot ||
                sample->bodies[j].generation != 1u) {
                return false;
            }
            digest = (digest ^ slot) * UINT64_C(1099511628211);
        }
    }
    printf("%s{\"scene\":\"%s\",\"queries\":%u,\"batch_ms\":%.9g,"
           "\"digest\":\"%016" PRIx64 "\"}",
           broad ? "," : "", broad ? "query-broad" : "query-point", QUERY_COUNT,
           elapsed, digest);
    return true;
}

int main(void)
{
    sl_world world = { 0 };
    const sl_world_config config = { .body_capacity = BODY_COUNT,
                                     .contact_capacity = 1u };
    query_sample *samples = calloc(QUERY_COUNT, sizeof(*samples));
    bool valid = samples != NULL && sl_world_init(&world, &config);
    sl_shape shape = sl_shape_none();
    valid = valid && sl_shape_make_circle(0.03125f, &shape);
    for (uint32_t slot = 0u; valid && slot < BODY_COUNT; ++slot) {
        const uint32_t cell = (slot * 561u) % BODY_COUNT;
        const sl_body_desc body = { .type = SL_BODY_STATIC,
                                    .position = { (float)(cell % 32u) * 0.125f,
                                                  (float)(cell / 32u) *
                                                      0.125f },
                                    .shape = &shape };
        valid = !sl_body_handle_is_null(sl_world_body_create(&world, &body));
    }
    printf("[");
    valid = valid && measure(&world, false, samples) &&
            measure(&world, true, samples);
    printf("]\n");
    sl_world_destroy(&world);
    free(samples);
    return valid ? 0 : 1;
}
