#pragma once

#define PRECISION 2
#define MAX_LINE_LENGTH 9046

#ifndef REAL_TYPE
#define REAL_TYPE
#if PRECISION == 1
typedef float real_t;
#elif PRECISION == 2
typedef double real_t;
#else
#error "Floating Point Precision not defined!"
#endif
#endif

#ifndef status_t
#define status_t __attribute__((warn_unused_result)) int
#endif


/** Terminal ANSI colors */
#define KRED "\x1B[1;31m"
#define KGRN "\x1B[1;32m"
#define KYEL "\x1B[1;33m"
#define KBLU "\x1B[1;34m"
#define KMAG "\x1B[1;35m"
#define KCYN "\x1B[1;36m"
#define KWHT "\x1B[1;37m"
#define KNRM "\x1B[1;0m"

/** Macro function that returns the caller's filename */
#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

/** Macro that adds the ability to switch between C / C++ style mallocs */
#ifdef __cplusplus

#ifndef MALLOC
#define MALLOC(TYPE, N) (TYPE *) malloc(sizeof(TYPE) * (N));
#endif

#ifndef REALLOC
#define REALLOC(PTR, TYPE, N) (TYPE *) realloc(PTR, sizeof(TYPE) * (N));
#endif

#ifndef CALLOC
#define CALLOC(TYPE, N) (TYPE *) calloc((N), sizeof(TYPE));
#endif

#else

#ifndef MALLOC
#define MALLOC(TYPE, N) malloc(sizeof(TYPE) * (N));
#endif

#ifndef REALLOC
#define REALLOC(PTR, TYPE, N) realloc(PTR, sizeof(TYPE) * (N));
#endif

#ifndef CALLOC
#define CALLOC(TYPE, N) calloc((N), sizeof(TYPE));
#endif

#endif

/**
 * Free macro
 */
#ifndef FREE
#define FREE(X) free(X);
#endif

/**
 * Free memory
 */
#ifndef FREE_MEM
#define FREE_MEM(TARGET, FREE_FUNC)                                            \
  if (TARGET) {                                                                \
    FREE_FUNC((void *) TARGET);                                                \
  }
#endif

/**
 * Assert if condition is true
 */
#ifndef ASSERT_IF
#define ASSERT_IF(COND, ASSERT_COND)                                           \
  if (COND) {                                                                  \
    assert(ASSERT_COND);                                                       \
  }
#endif

/**
 * Mark variable unused.
 * @param[in] expr Variable to mark as unused
 */
#ifndef UNUSED
#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)
#endif

/**
 * Check if condition is satisfied.
 *
 * If the condition is not satisfied a message M will be logged and a goto
 * error is called.
 *
 * @param[in] A Condition to be checked
 */
#ifndef CHECK
#define CHECK(A)                                                               \
  if (!(A)) {                                                                  \
    LOG_ERROR(#A " Failed!\n");                                                \
    goto error;                                                                \
  }
#endif

/**
 * Array copy
 */
#define ARRAY_COPY(SRC, N, DST)                                                \
  for (int i = 0; i < N; i++) {                                                \
    DST[i] = SRC[i];                                                           \
  }

/**
 * Median value in buffer
 */
#define MEDIAN_VALUE(DATA_TYPE, DATA_CMP, BUF, BUF_SIZE, MEDIAN_VAR)           \
  {                                                                            \
    DATA_TYPE VALUES[BUF_SIZE] = {0};                                          \
    for (size_t i = 0; i < BUF_SIZE; i++) {                                    \
      VALUES[i] = BUF[i];                                                      \
    }                                                                          \
                                                                               \
    qsort(VALUES, BUF_SIZE, sizeof(DATA_TYPE), DATA_CMP);                      \
    if ((BUF_SIZE % 2) == 0) {                                                 \
      const size_t bwd_idx = (size_t) (BUF_SIZE - 1) / 2.0;                    \
      const size_t fwd_idx = (size_t) (BUF_SIZE + 1) / 2.0;                    \
      MEDIAN_VAR = (VALUES[bwd_idx] + VALUES[fwd_idx]) / 2.0;                  \
    } else {                                                                   \
      const size_t mid_idx = (BUF_SIZE - 1) / 2;                               \
      MEDIAN_VAR = VALUES[mid_idx];                                            \
    }                                                                          \
  }

/**
 * Mean value in buffer
 */
#define MEAN_VALUE(DATA_TYPE, BUF, BUF_SIZE, MEAN_VAR)                         \
  {                                                                            \
    DATA_TYPE VALUE = 0;                                                       \
    for (size_t i = 0; i < BUF_SIZE; i++) {                                    \
      VALUE += BUF[i];                                                         \
    }                                                                          \
    MEAN_VAR = VALUE / (real_t) BUF_SIZE;                                      \
  }
