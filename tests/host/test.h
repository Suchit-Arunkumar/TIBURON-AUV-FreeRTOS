/*
 * Minimal test helpers for the host tests. Each test is one C file built
 * with the PC's gcc against the real driver source; see run.sh.
 */
#ifndef TEST_H
#define TEST_H

#include <stdio.h>
#include <math.h>

static int test_failures = 0;
static int test_checks   = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        test_checks++;                                                  \
        if (!(cond)) {                                                  \
            test_failures++;                                            \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                               \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                           \
    do {                                                                \
        test_checks++;                                                  \
        double a_ = (double)(a), b_ = (double)(b);                      \
        if (fabs(a_ - b_) > (tol)) {                                    \
            test_failures++;                                            \
            printf("  FAIL %s:%d: %s = %g, expected %g\n",              \
                   __FILE__, __LINE__, #a, a_, b_);                     \
        }                                                               \
    } while (0)

static int test_summary(const char *name)
{
    printf("%-12s %d checks, %d failed\n", name, test_checks, test_failures);
    return test_failures ? 1 : 0;
}

#endif
