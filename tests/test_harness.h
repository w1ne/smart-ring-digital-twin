/*
 * test_harness.h - ~60 lines of test framework.
 *
 * Deliberately not Unity/CMock/GoogleTest. A dependency-free harness means
 * `cmake && ctest` works on a clean machine with no network, which is what
 * you want both for a reviewer and for a CI runner. If this codebase grew a
 * need for mocking or parameterised fixtures, Unity would be the move.
 */
#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int         g_checks;
static int         g_failures;
static const char *g_current_test;

#define TEST(name) static void name(void)

#define RUN(fn)                                                                          \
    do {                                                                                 \
        g_current_test = #fn;                                                            \
        int before     = g_failures;                                                     \
        fn();                                                                            \
        printf("  %s %s\n", g_failures == before ? "PASS" : "FAIL", #fn);                \
    } while (0)

#define CHECK(cond)                                                                      \
    do {                                                                                 \
        g_checks++;                                                                      \
        if (!(cond)) {                                                                   \
            g_failures++;                                                                \
            printf("    ASSERT %s:%d in %s: %s\n", __FILE__, __LINE__, g_current_test,   \
                   #cond);                                                               \
        }                                                                                \
    } while (0)

/* Long long everywhere so any integer type prints without a format-string
 * zoo at every call site. */
#define CHECK_EQ(actual, expected)                                                       \
    do {                                                                                 \
        long long a_ = (long long)(actual);                                              \
        long long e_ = (long long)(expected);                                            \
        g_checks++;                                                                      \
        if (a_ != e_) {                                                                  \
            g_failures++;                                                                \
            printf("    ASSERT %s:%d in %s: %s == %s (got %lld, want %lld)\n", __FILE__, \
                   __LINE__, g_current_test, #actual, #expected, a_, e_);                \
        }                                                                                \
    } while (0)

#define TEST_MAIN_BEGIN(suite)                                                           \
    int main(void)                                                                       \
    {                                                                                    \
        g_checks = g_failures = 0;                                                       \
        printf("[%s]\n", suite);

#define TEST_MAIN_END()                                                                  \
    printf("  %d checks, %d failures\n", g_checks, g_failures);                          \
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;                                \
    }

#endif /* TEST_HARNESS_H */
