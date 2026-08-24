#ifndef SILK_TEST_H
#define SILK_TEST_H

/*
 * Minimal test framework for the silk engine test suite.
 *
 * Usage:
 *   - Suites are arrays of sl_test_case, run via sl_run_suite().
 *   - Assertions inside a case increment sl_test_failures on failure.
 *   - Each executable defines SILK_TEST_IMPLEMENTATION before including
 *     this header exactly once (in main.c) to provide the globals.
 */

#include <stdio.h>

typedef struct {
    const char *name;
    void (*fn)(void);
} sl_test_case;

/* Total failed assertions across all suites. Defined by the TU that sets
 * SILK_TEST_IMPLEMENTATION before including this header. */
extern int sl_test_failures;

/* Runs each case, printing pass/fail lines. Returns the suite's failure count. */
int sl_run_suite(const char *suite, const sl_test_case *cases, int count);

#ifdef SILK_TEST_IMPLEMENTATION

int sl_test_failures = 0;

int sl_run_suite(const char *suite, const sl_test_case *cases, int count)
{
    int start = sl_test_failures;
    for (int i = 0; i < count; i++) {
        printf("[%s] %-32s ", suite, cases[i].name);
        fflush(stdout);
        int before = sl_test_failures;
        cases[i].fn();
        printf("%s\n", (sl_test_failures == before) ? "ok" : "FAILED");
    }
    return sl_test_failures - start;
}

#endif /* SILK_TEST_IMPLEMENTATION */

#define SL_EXPECT(cond)                                                       \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("\n    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            sl_test_failures++;                                               \
        }                                                                     \
    } while (0)

#define SL_EXPECT_INT_EQ(a, b)                                                \
    do {                                                                      \
        long long sl_a_ = (long long)(a);                                     \
        long long sl_b_ = (long long)(b);                                     \
        if (sl_a_ != sl_b_) {                                                 \
            printf("\n    FAIL %s:%d: %s == %s (%lld != %lld)\n",             \
                   __FILE__, __LINE__, #a, #b, sl_a_, sl_b_);                 \
            sl_test_failures++;                                               \
        }                                                                     \
    } while (0)

#define SL_EXPECT_NEAR(a, b, eps)                                             \
    do {                                                                      \
        double sl_a_ = (double)(a);                                           \
        double sl_b_ = (double)(b);                                           \
        double sl_d_ = sl_a_ - sl_b_;                                         \
        if (sl_d_ < 0.0)                                                      \
            sl_d_ = -sl_d_;                                                   \
        if (!(sl_d_ <= (double)(eps))) {                                      \
            printf("\n    FAIL %s:%d: |%s - %s| = %g > %g\n",                 \
                   __FILE__, __LINE__, #a, #b, sl_d_, (double)(eps));         \
            sl_test_failures++;                                               \
        }                                                                     \
    } while (0)

#endif /* SILK_TEST_H */
