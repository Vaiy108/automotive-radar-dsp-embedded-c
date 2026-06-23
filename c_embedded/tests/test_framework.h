/*
 * test_framework.h
 *
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <math.h>

static int g_tf_pass_count = 0;
static int g_tf_fail_count = 0;

#define TF_CHECK(cond, msg) \
    do { \
        if (cond) { \
            g_tf_pass_count++; \
        } else { \
            g_tf_fail_count++; \
            printf("  [FAIL] %s (line %d): %s\n", __func__, __LINE__, msg); \
        } \
    } while (0)

#define TF_ASSERT_TRUE(cond)  TF_CHECK((cond), #cond)

#define TF_ASSERT_EQ_INT(a, b) \
    TF_CHECK((a) == (b), #a " == " #b)

#define TF_ASSERT_NEAR(a, b, tol) \
    TF_CHECK(fabsf((float)(a) - (float)(b)) <= (float)(tol), #a " ~= " #b)

#define TF_RUN(test_fn) \
    do { \
        printf("RUN  %s\n", #test_fn); \
        test_fn(); \
    } while (0)

/* Call at the end of main(). Prints a summary and returns the process
 * exit code (0 = all passed). */
static inline int tf_summary(void) {
    printf("---- %d passed, %d failed ----\n", g_tf_pass_count, g_tf_fail_count);
    return g_tf_fail_count == 0 ? 0 : 1;
}

#endif /* TEST_FRAMEWORK_H */
