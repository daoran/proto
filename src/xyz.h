#pragma once

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <errno.h>

#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>

#include <cblas.h>
#include <suitesparse/cholmod.h>

#define PRECISION 2
#define MAX_LINE_LENGTH 9046
#define USE_LAPACK

/******************************************************************************
 * MACROS
 *****************************************************************************/

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


/*******************************************************************************
 * LOGGING
 ******************************************************************************/

/**
 * Debug
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef DEBUG
#define DEBUG(...)                                                             \
  do {                                                                         \
    fprintf(stderr, "[DEBUG] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);   \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);
#endif

/**
 * Log info
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef LOG_INFO
#define LOG_INFO(...)                                                          \
  do {                                                                         \
    fprintf(stderr, "[INFO] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);    \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0)
#endif

/**
 * Log error
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef LOG_ERROR
#define LOG_ERROR(...)                                                         \
  do {                                                                         \
    fprintf(stderr, "[ERROR] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);   \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0)
#endif

/**
 * Log warn
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef LOG_WARN
#define LOG_WARN(...)                                                          \
  do {                                                                         \
    fprintf(stderr, "[WARN] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);    \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0)
#endif

/**
 * Fatal
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef FATAL
#define FATAL(...)                                                             \
  do {                                                                         \
    fprintf(stderr, "[FATAL] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);   \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);                                                                 \
  exit(-1)
#endif

/*******************************************************************************
 * DATA
 ******************************************************************************/

size_t string_copy(char *dst, const char *src);
void string_subcopy(char *dst, const char *src, const int s, const int n);
void string_cat(char *dst, const char *src);
char *string_malloc(const char *s);
char *string_strip(char *s);
char *string_strip_char(char *s, const char c);
char **string_split(char *s, const char d, size_t *n);

int **load_iarrays(const char *csv_path, int *num_arrays);
double **load_darrays(const char *csv_path, int *num_arrays);

int *int_malloc(const int val);
float *float_malloc(const float val);
double *double_malloc(const double val);
double *vector_malloc(const double *vec, const double N);

int dsv_rows(const char *fp);
int dsv_cols(const char *fp, const char delim);
char **dsv_fields(const char *fp, const char delim, int *num_fields);
double **
dsv_data(const char *fp, const char delim, int *num_rows, int *num_cols);
void dsv_free(double **data, const int num_rows);

double **csv_data(const char *fp, int *num_rows, int *num_cols);
void csv_free(double **data, const int num_rows);

void path_file_name(const char *path, char *fname);
void path_file_ext(const char *path, char *fext);
void path_dir_name(const char *path, char *dir_name);
char *path_join(const char *x, const char *y);
char **list_files(const char *path, int *num_files);
void list_files_free(char **data, const int n);

char *file_read(const char *fp);
void skip_line(FILE *fp);
status_t file_exists(const char *fp);
status_t file_rows(const char *fp);
status_t file_copy(const char *src, const char *dest);

/*******************************************************************************
 * TIME
 ******************************************************************************/

// Timestamp Type
#ifndef timestamp_t
typedef int64_t timestamp_t;
#endif

// Tic toc macros
#define TIC(X) struct timespec X = tic()
#define TOC(X) toc(&X)
#define MTOC(X) mtoc(&X)
#define PRINT_TOC(PREFIX, X) printf("[%s]: %.4fs\n", PREFIX, toc(&X))
#define PRINT_MTOC(PREFIX, X) printf("[%s]: %.4fms\n", PREFIX, mtoc(&X))

struct timespec tic(void);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
timestamp_t time_now(void);

timestamp_t str2ts(const char *ts_str);
double ts2sec(const timestamp_t ts);
timestamp_t sec2ts(const double time_s);

/*******************************************************************************
 * NETWORK
 ******************************************************************************/

/**
 * TCP server
 */
typedef struct tcp_server_t {
  int port;
  int sockfd;
  int conn;
  void *(*conn_handler)(void *);
} tcp_server_t;

/**
 * TCP client
 */
typedef struct tcp_client_t {
  char server_ip[1024];
  int server_port;
  int sockfd;
  int (*loop_cb)(struct tcp_client_t *);
} tcp_client_t;

status_t ip_port_info(const int sockfd, char *ip, int *port);

status_t tcp_server_setup(tcp_server_t *server, const int port);
status_t tcp_server_loop(tcp_server_t *server);

status_t tcp_client_setup(tcp_client_t *client,
                        const char *server_ip,
                        const int server_port);
status_t tcp_client_loop(tcp_client_t *client);

/*******************************************************************************
 * MATH
 ******************************************************************************/

/** Mathematical Pi constant (i.e. 3.1415..) */
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

/** Real number comparison tolerance */
#ifndef CMP_TOL
#define CMP_TOL 1e-6
#endif

/** Min of two numbers, X or Y. */
#define MIN(x, y) ((x) < (y) ? (x) : (y))

/** Max of two numbers, X or Y. */
#define MAX(x, y) ((x) > (y) ? (x) : (y))

/** Based on sign of b, return +ve or -ve a. */
#define SIGN2(a, b) ((b) > 0.0 ? fabs(a) : -fabs(a))

float randf(float a, float b);
void randvec(const real_t a, const real_t b, const size_t n, real_t *v);
real_t deg2rad(const real_t d);
real_t rad2deg(const real_t r);
real_t wrap_180(const real_t d);
real_t wrap_360(const real_t d);
real_t wrap_pi(const real_t r);
real_t wrap_2pi(const real_t r);
int intcmp(const int x, const int y);
int intcmp2(const void *x, const void *y);
int fltcmp(const real_t x, const real_t y);
int fltcmp2(const void *x, const void *y);
int strcmp2(const void *x, const void *y);
int flteqs(const real_t x, const real_t y);
int streqs(const char *x, const char *y);
void cumsum(const real_t *x, const size_t n, real_t *s);
void logspace(const real_t a, const real_t b, const size_t n, real_t *x);
real_t pythag(const real_t a, const real_t b);
real_t clip_value(const real_t x, const real_t vmin, const real_t vmax);
void clip(real_t *x, const size_t n, const real_t vmin, const real_t vmax);
real_t lerp(const real_t a, const real_t b, const real_t t);
void lerp3(const real_t a[3], const real_t b[3], const real_t t, real_t x[3]);
real_t sinc(const real_t x);
real_t mean(const real_t *x, const size_t length);
real_t median(const real_t *x, const size_t length);
real_t var(const real_t *x, const size_t length);
real_t stddev(const real_t *x, const size_t length);

/*******************************************************************************
 * LINEAR ALGEBRA
 ******************************************************************************/

void print_matrix(const char *prefix,
                  const real_t *A,
                  const size_t m,
                  const size_t n);
void print_vector(const char *prefix, const real_t *v, const size_t n);
void print_float_array(const char *prefix, const float *arr, const size_t n);
void print_double_array(const char *prefix, const double *arr, const size_t n);
void vec2str(const real_t *v, const int n, char *s);
void vec2csv(const real_t *v, const int n, char *s);

void eye(real_t *A, const size_t m, const size_t n);
void ones(real_t *A, const size_t m, const size_t n);
void zeros(real_t *A, const size_t m, const size_t n);
void hat(const real_t x[3], real_t A[3 * 3]);
void vee(const real_t A[3 * 3], real_t x[3]);
void fwdsubs(const real_t *L, const real_t *b, real_t *y, const size_t n);
void bwdsubs(const real_t *U, const real_t *y, real_t *x, const size_t n);
void enforce_spd(real_t *A, const int m, const int n);

real_t *mat_malloc(const size_t m, const size_t n);
int mat_cmp(const real_t *A, const real_t *B, const size_t m, const size_t n);
int mat_equals(const real_t *A,
               const real_t *B,
               const size_t m,
               const size_t n,
               const real_t tol);
// int mat_save(const char *save_path, const real_t *A, const int m, const int n);
// real_t *mat_load(const char *save_path, int *num_rows, int *num_cols);
void mat_set(real_t *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const real_t val);
real_t
mat_val(const real_t *A, const size_t stride, const size_t i, const size_t j);
void mat_copy(const real_t *src, const int m, const int n, real_t *dest);
void mat_row_set(real_t *A,
                 const size_t stride,
                 const int row_idx,
                 const real_t *x);
void mat_col_set(real_t *A,
                 const size_t stride,
                 const int num_rows,
                 const int col_idx,
                 const real_t *x);
void mat_col_get(const real_t *A,
                 const int m,
                 const int n,
                 const int col_idx,
                 real_t *x);
void mat_block_get(const real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t re,
                   const size_t cs,
                   const size_t ce,
                   real_t *block);
void mat_block_set(real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t re,
                   const size_t cs,
                   const size_t ce,
                   const real_t *block);
void mat_block_add(real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t re,
                   const size_t cs,
                   const size_t ce,
                   const real_t *block);
void mat_block_sub(real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t re,
                   const size_t cs,
                   const size_t ce,
                   const real_t *block);
void mat_diag_get(const real_t *A, const int m, const int n, real_t *d);
void mat_diag_set(real_t *A, const int m, const int n, const real_t *d);
void mat_triu(const real_t *A, const size_t n, real_t *U);
void mat_tril(const real_t *A, const size_t n, real_t *L);
real_t mat_trace(const real_t *A, const size_t m, const size_t n);
void mat_transpose(const real_t *A, size_t m, size_t n, real_t *A_t);
void mat_add(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n);
void mat_sub(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n);
void mat_scale(real_t *A, const size_t m, const size_t n, const real_t scale);

void mat3_copy(const real_t src[3 * 3], real_t dst[3 * 3]);
void mat3_add(const real_t A[3 * 3], const real_t B[3 * 3], real_t C[3 * 3]);
void mat3_sub(const real_t A[3 * 3], const real_t B[3 * 3], real_t C[3 * 3]);

real_t *vec_malloc(const real_t *x, const size_t n);
void vec_copy(const real_t *src, const size_t n, real_t *dest);
int vec_equals(const real_t *x, const real_t *y, const size_t n);
real_t vec_min(const real_t *x, const size_t n);
real_t vec_max(const real_t *x, const size_t n);
void vec_range(const real_t *x,
               const size_t n,
               real_t *vmin,
               real_t *vmax,
               real_t *r);
// real_t *vec_load(const char *save_path, int *num_rows, int *num_cols);
void vec_add(const real_t *x, const real_t *y, real_t *z, size_t n);
void vec_sub(const real_t *x, const real_t *y, real_t *z, size_t n);
void vec_scale(real_t *x, const size_t n, const real_t scale);
real_t vec_norm(const real_t *x, const size_t n);
void vec_normalize(real_t *x, const size_t n);

void vec3_copy(const real_t src[3], real_t dst[3]);
void vec3_add(const real_t a[3], const real_t b[3], real_t c[3]);
void vec3_sub(const real_t a[3], const real_t b[3], real_t c[3]);
void vec3_cross(const real_t a[3], const real_t b[3], real_t c[3]);
real_t vec3_norm(const real_t x[3]);
void vec3_normalize(real_t x[3]);

void dot(const real_t *A,
         const size_t A_m,
         const size_t A_n,
         const real_t *B,
         const size_t B_m,
         const size_t B_n,
         real_t *C);
void dot3(const real_t *A,
          const size_t A_m,
          const size_t A_n,
          const real_t *B,
          const size_t B_m,
          const size_t B_n,
          const real_t *C,
          const size_t C_m,
          const size_t C_n,
          real_t *D);
void dot_XtAX(const real_t *X,
              const size_t X_m,
              const size_t X_n,
              const real_t *A,
              const size_t A_m,
              const size_t A_n,
              real_t *Y);
void dot_XAXt(const real_t *X,
              const size_t X_m,
              const size_t X_n,
              const real_t *A,
              const size_t A_m,
              const size_t A_n,
              real_t *Y);

void bdiag_inv(const real_t *A, const int m, const int bs, real_t *A_inv);
void bdiag_inv_sub(const real_t *A,
                   const int stride,
                   const int m,
                   const int bs,
                   real_t *A_inv);
void bdiag_dot(const real_t *A,
               const int m,
               const int n,
               const int bs,
               const real_t *x,
               real_t *b);

#define MAT_TRANSPOSE(A, M, N, B)                                              \
  real_t B[N * M] = {0};                                                       \
  mat_transpose(A, M, N, B);

#define DOT(A, AM, AN, B, BM, BN, C)                                           \
  real_t C[AM * BN] = {0};                                                     \
  dot(A, AM, AN, B, BM, BN, C);

#define DOT3(A, AM, AN, B, BM, BN, C, CM, CN, D)                               \
  real_t D[AM * CN] = {0};                                                     \
  dot3(A, AM, AN, B, BM, BN, C, CM, CN, D);

#define DOT_XTAX(X, XM, XN, A, AM, AN, Y)                                      \
  real_t Y[XN * XN] = {0};                                                     \
  dot_XtAX(X, XM, XN, A, AM, AN, Y);

#define DOT_XAXt(X, XM, XN, A, AM, AN, Y)                                      \
  real_t Y[XM * XM] = {0};                                                     \
  dot_XAXt(X, XM, XN, A, AM, AN, Y);

#define HAT(X, X_HAT)                                                          \
  real_t X_HAT[3 * 3] = {0};                                                   \
  hat(X, X_HAT);

#define VEE(A, X)                                                              \
  real_t X[3] = {0};                                                           \
  vee(A, X);

int check_inv(const real_t *A, const real_t *A_inv, const int m);
real_t check_Axb(const real_t *A,
                 const real_t *x,
                 const real_t *b,
                 const int m,
                 const int n);
int check_jacobian(const char *jac_name,
                   const real_t *fdiff,
                   const real_t *jac,
                   const size_t m,
                   const size_t n,
                   const real_t tol,
                   const int verbose);

#define CHECK_JACOBIAN(JAC_IDX, FACTOR, FACTOR_EVAL, STEP_SIZE, TOL, VERBOSE)  \
  {                                                                            \
    const int r_size = FACTOR.r_size;                                          \
    const int p_size = param_global_size(FACTOR.param_types[JAC_IDX]);         \
    const int J_cols = param_local_size(FACTOR.param_types[JAC_IDX]);          \
                                                                               \
    real_t *param_copy = MALLOC(real_t, p_size);                               \
    real_t *r_fwd = MALLOC(real_t, r_size);                                    \
    real_t *r_bwd = MALLOC(real_t, r_size);                                    \
    real_t *r_diff = MALLOC(real_t, r_size);                                   \
    real_t *J_fdiff = MALLOC(real_t, r_size * J_cols);                         \
    real_t *J = MALLOC(real_t, r_size * J_cols);                               \
                                                                               \
    /* Evaluate factor to get analytical Jacobian */                           \
    FACTOR_EVAL((void *) &FACTOR);                                             \
    mat_copy(factor.jacs[JAC_IDX], r_size, J_cols, J);                         \
                                                                               \
    /* Calculate numerical differerntiated Jacobian */                         \
    for (int i = 0; i < J_cols; i++) {                                         \
      vec_copy(FACTOR.params[JAC_IDX], p_size, param_copy);                    \
                                                                               \
      FACTOR.params[JAC_IDX][i] += 0.5 * STEP_SIZE;                            \
      FACTOR_EVAL((void *) &FACTOR);                                           \
      vec_copy(FACTOR.r, r_size, r_fwd);                                       \
      vec_copy(param_copy, p_size, FACTOR.params[JAC_IDX]);                    \
                                                                               \
      FACTOR.params[JAC_IDX][i] -= 0.5 * STEP_SIZE;                            \
      FACTOR_EVAL((void *) &FACTOR);                                           \
      vec_copy(FACTOR.r, r_size, r_bwd);                                       \
      vec_copy(param_copy, p_size, FACTOR.params[JAC_IDX]);                    \
                                                                               \
      vec_sub(r_fwd, r_bwd, r_diff, r_size);                                   \
      vec_scale(r_diff, r_size, 1.0 / STEP_SIZE);                              \
      mat_col_set(J_fdiff, J_cols, r_size, i, r_diff);                         \
    }                                                                          \
                                                                               \
    char s[100] = {0};                                                         \
    sprintf(s, "J%d", JAC_IDX);                                                \
    int retval = check_jacobian(s, J_fdiff, J, r_size, J_cols, TOL, VERBOSE);  \
                                                                               \
    free(param_copy);                                                          \
    free(r_fwd);                                                               \
    free(r_bwd);                                                               \
    free(r_diff);                                                              \
    free(J_fdiff);                                                             \
    free(J);                                                                   \
                                                                               \
    MU_ASSERT(retval == 0);                                                    \
  }

#define CHECK_POSE_JACOBIAN(JAC_IDX,                                           \
                            FACTOR,                                            \
                            FACTOR_EVAL,                                       \
                            STEP_SIZE,                                         \
                            TOL,                                               \
                            VERBOSE)                                           \
  {                                                                            \
    const int r_size = FACTOR.r_size;                                          \
    const int J_cols = param_local_size(FACTOR.param_types[JAC_IDX]);          \
                                                                               \
    real_t *r = MALLOC(real_t, r_size);                                        \
    real_t *r_fwd = MALLOC(real_t, r_size);                                    \
    real_t *r_diff = MALLOC(real_t, r_size);                                   \
    real_t *J_fdiff = MALLOC(real_t, r_size * J_cols);                         \
    real_t *J = MALLOC(real_t, r_size * J_cols);                               \
                                                                               \
    /* Eval */                                                                 \
    FACTOR_EVAL(&FACTOR);                                                      \
    vec_copy(FACTOR.r, r_size, r);                                             \
    mat_copy(FACTOR.jacs[JAC_IDX], r_size, J_cols, J);                         \
                                                                               \
    /* Check pose position jacobian */                                         \
    for (int i = 0; i < 3; i++) {                                              \
      FACTOR.params[JAC_IDX][i] += STEP_SIZE;                                  \
      FACTOR_EVAL((void *) &FACTOR);                                           \
      vec_copy(FACTOR.r, r_size, r_fwd);                                       \
      FACTOR.params[JAC_IDX][i] -= STEP_SIZE;                                  \
                                                                               \
      vec_sub(r_fwd, r, r_diff, r_size);                                       \
      vec_scale(r_diff, r_size, 1.0 / STEP_SIZE);                              \
      mat_col_set(J_fdiff, J_cols, r_size, i, r_diff);                         \
    }                                                                          \
    for (int i = 0; i < 3; i++) {                                              \
      quat_perturb(FACTOR.params[JAC_IDX] + 3, i, STEP_SIZE);                  \
      FACTOR_EVAL((void *) &FACTOR);                                           \
      vec_copy(FACTOR.r, r_size, r_fwd);                                       \
      quat_perturb(FACTOR.params[JAC_IDX] + 3, i, -STEP_SIZE);                 \
                                                                               \
      vec_sub(r_fwd, r, r_diff, r_size);                                       \
      vec_scale(r_diff, r_size, 1.0 / STEP_SIZE);                              \
      mat_col_set(J_fdiff, J_cols, r_size, i + 3, r_diff);                     \
    }                                                                          \
                                                                               \
    char s[100] = {0};                                                         \
    sprintf(s, "J%d", JAC_IDX);                                                \
    int retval = check_jacobian(s, J_fdiff, J, r_size, J_cols, TOL, VERBOSE);  \
                                                                               \
    free(r);                                                                   \
    free(r_fwd);                                                               \
    free(r_diff);                                                              \
    free(J_fdiff);                                                             \
    free(J);                                                                   \
                                                                               \
    MU_ASSERT(retval == 0);                                                    \
  }

#define CHECK_FACTOR_J(PARAM_IDX,                                              \
                       FACTOR,                                                 \
                       FACTOR_EVAL,                                            \
                       STEP_SIZE,                                              \
                       TOL,                                                    \
                       VERBOSE)                                                \
  {                                                                            \
    int param_type = FACTOR.param_types[PARAM_IDX];                            \
    switch (param_type) {                                                      \
      case POSE_PARAM:                                                         \
      case EXTRINSIC_PARAM:                                                    \
      case FIDUCIAL_PARAM:                                                     \
        CHECK_POSE_JACOBIAN(PARAM_IDX,                                         \
                            FACTOR,                                            \
                            FACTOR_EVAL,                                       \
                            STEP_SIZE,                                         \
                            TOL,                                               \
                            VERBOSE)                                           \
        break;                                                                 \
      default:                                                                 \
        CHECK_JACOBIAN(PARAM_IDX,                                              \
                       FACTOR,                                                 \
                       FACTOR_EVAL,                                            \
                       STEP_SIZE,                                              \
                       TOL,                                                    \
                       VERBOSE)                                                \
        break;                                                                 \
    }                                                                          \
  }

/////////
// SVD //
/////////

int svd(const real_t *A,
        const int m,
        const int n,
        real_t *U,
        real_t *s,
        real_t *V);
void pinv(const real_t *A, const int m, const int n, real_t *A_inv);
int svd_det(const real_t *A, const int m, const int n, real_t *det);
int svd_rank(const real_t *A, const int m, const int n, real_t tol);

//////////
// CHOL //
//////////

void chol(const real_t *A, const size_t n, real_t *L);
void chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n);

////////
// QR //
////////

void qr(real_t *A, const int m, const int n, real_t *R);

/////////
// EIG //
/////////

#define EIG_V_SIZE(Am, An) (Am * An)
#define EIG_W_SIZE(Am, An) (An)

int eig_sym(const real_t *A, const int m, const int n, real_t *V, real_t *w);
int eig_inv(real_t *A, const int m, const int n, const int c, real_t *A_inv);
int eig_rank(const real_t *A, const int m, const int n, const real_t tol);

/*******************************************************************************
 * SUITE-SPARSE
 ******************************************************************************/

cholmod_sparse *cholmod_sparse_malloc(cholmod_common *c,
                                      const real_t *A,
                                      const int m,
                                      const int n,
                                      const int stype);
cholmod_dense *cholmod_dense_malloc(cholmod_common *c,
                                    const real_t *x,
                                    const int n);
void cholmod_dense_raw(const cholmod_dense *src, real_t *dst, const int n);
real_t suitesparse_chol_solve(cholmod_common *c,
                              const real_t *A,
                              const int A_m,
                              const int A_n,
                              const real_t *b,
                              const int b_m,
                              real_t *x);

/*******************************************************************************
 * TRANSFORMS
 ******************************************************************************/

#define TF(PARAMS, T)                                                          \
  real_t T[4 * 4] = {0};                                                       \
  tf(PARAMS, T);

#define TF_TRANS(T, TRANS) real_t TRANS[3] = {T[3], T[7], T[11]};

#define TF_ROT(T, ROT)                                                         \
  real_t ROT[3 * 3] = {0};                                                     \
  ROT[0] = T[0];                                                               \
  ROT[1] = T[1];                                                               \
  ROT[2] = T[2];                                                               \
  ROT[3] = T[4];                                                               \
  ROT[4] = T[5];                                                               \
  ROT[5] = T[6];                                                               \
  ROT[6] = T[8];                                                               \
  ROT[7] = T[9];                                                               \
  ROT[8] = T[10];

#define TF_QUAT(T, QUAT)                                                       \
  real_t QUAT[4] = {0};                                                        \
  tf_quat_get(T, QUAT);

#define TF_CR(C, R, T)                                                         \
  real_t T[4 * 4] = {0};                                                       \
  tf_cr(C, R, T);

#define TF_ER(E, R, T)                                                         \
  real_t T[4 * 4] = {0};                                                       \
  tf_er(E, R, T);

#define TF_VECTOR(T, V)                                                        \
  real_t V[7] = {0};                                                           \
  tf_vector(T, V);

#define TF_DECOMPOSE(T, ROT, TRANS)                                            \
  real_t ROT[3 * 3] = {0};                                                     \
  real_t TRANS[3] = {0};                                                       \
  tf_decompose(T, ROT, TRANS);

#define TF_QR(Q, R, T)                                                         \
  real_t T[4 * 4] = {0};                                                       \
  tf_qr(Q, R, T);

#define TF_INV(T, T_INV)                                                       \
  real_t T_INV[4 * 4] = {0};                                                   \
  tf_inv(T, T_INV);

#define TF_POINT(T, P_IN, P_OUT)                                               \
  real_t P_OUT[3] = {0};                                                       \
  tf_point(T, P_IN, P_OUT);

#define TF_CHAIN(T, N, ...)                                                    \
  real_t T[4 * 4] = {0};                                                       \
  tf_chain2(N, __VA_ARGS__, T);

#define EULER321(YPR, C)                                                       \
  real_t C[3 * 3] = {0};                                                       \
  euler321(YPR, C);

#define EULER2QUAT(YPR, Q)                                                     \
  real_t Q[4] = {0};                                                           \
  euler2quat(YPR, Q);

#define ROT2QUAT(C, Q)                                                         \
  real_t Q[4] = {0};                                                           \
  rot2quat(C, Q);

#define QUAT2ROT(Q, C)                                                         \
  real_t C[3 * 3] = {0};                                                       \
  quat2rot(Q, C);

// clang-format off
#define TF_IDENTITY(T)                                                         \
  real_t T[4 * 4] = {                                                          \
    1.0, 0.0, 0.0, 0.0,                                                        \
    0.0, 1.0, 0.0, 0.0,                                                        \
    0.0, 0.0, 1.0, 0.0,                                                        \
    0.0, 0.0, 0.0, 1.0                                                         \
  };
// clang-format on

#define POSE_ER(YPR, POS, POSE)                                                \
  real_t POSE[7] = {0};                                                        \
  POSE[0] = POS[0];                                                            \
  POSE[1] = POS[1];                                                            \
  POSE[2] = POS[2];                                                            \
  euler2quat(YPR, POSE + 3);

#define POSE2TF(POSE, TF)                                                      \
  real_t TF[4 * 4] = {0};                                                      \
  tf(POSE, TF);

void rotx(const real_t theta, real_t C[3 * 3]);
void roty(const real_t theta, real_t C[3 * 3]);
void rotz(const real_t theta, real_t C[3 * 3]);
void tf(const real_t params[7], real_t T[4 * 4]);
void tf_cr(const real_t C[3 * 3], const real_t r[3], real_t T[4 * 4]);
void tf_qr(const real_t q[4], const real_t r[3], real_t T[4 * 4]);
void tf_er(const real_t ypr[3], const real_t r[3], real_t T[4 * 4]);
void tf_vector(const real_t T[4 * 4], real_t params[7]);
void tf_decompose(const real_t T[4 * 4], real_t C[3 * 3], real_t r[3]);
void tf_rot_set(real_t T[4 * 4], const real_t C[3 * 3]);
void tf_rot_get(const real_t T[4 * 4], real_t C[3 * 3]);
void tf_quat_set(real_t T[4 * 4], const real_t q[4]);
void tf_quat_get(const real_t T[4 * 4], real_t q[4]);
void tf_euler_set(real_t T[4 * 4], const real_t ypr[3]);
void tf_euler_get(const real_t T[4 * 4], real_t ypr[3]);
void tf_trans_set(real_t T[4 * 4], const real_t r[3]);
void tf_trans_get(const real_t T[4 * 4], real_t r[3]);
void tf_inv(const real_t T[4 * 4], real_t T_inv[4 * 4]);
void tf_point(const real_t T[4 * 4], const real_t p[3], real_t retval[3]);
void tf_hpoint(const real_t T[4 * 4], const real_t p[4], real_t retval[4]);
void tf_perturb_rot(real_t T[4 * 4], const real_t step_size, const int i);
void tf_perturb_trans(real_t T[4 * 4], const real_t step_size, const int i);
void tf_chain(const real_t **tfs, const int num_tfs, real_t T_out[4 * 4]);
void tf_chain2(const int num_tfs, ...);
void tf_diff(const real_t Ti[4 * 4], const real_t Tj[4 * 4], real_t diff[6]);
void tf_diff2(const real_t Ti[4 * 4],
              const real_t Tj[4 * 4],
              real_t dr[3],
              real_t *dtheta);
void pose_get_trans(const real_t pose[7], real_t r[3]);
void pose_get_quat(const real_t pose[7], real_t q[4]);
void pose_get_rot(const real_t p[7], real_t C[3 * 3]);
void pose_diff(const real_t pose0[7], const real_t pose1[7], real_t diff[6]);
void pose_diff2(const real_t pose0[7],
                const real_t pose1[7],
                real_t dr[3],
                real_t *dangle);
void pose_update(real_t pose[7], const real_t dx[6]);
void pose_random_perturb(real_t pose[7],
                         const real_t dtrans,
                         const real_t drot);
void print_pose(const char *prefix, const real_t pose[7]);
void vecs2rot(const real_t acc[3], const real_t g[3], real_t *C);
void rvec2rot(const real_t *rvec, const real_t eps, real_t *R);
void euler321(const real_t ypr[3], real_t C[3 * 3]);
void euler2quat(const real_t ypr[3], real_t q[4]);
void rot2quat(const real_t C[3 * 3], real_t q[4]);
void rot2euler(const real_t C[3 * 3], real_t ypr[3]);
void quat2euler(const real_t q[4], real_t ypr[3]);
void quat2rot(const real_t q[4], real_t C[3 * 3]);
void print_quat(const char *prefix, const real_t q[4]);
real_t quat_norm(const real_t q[4]);
void quat_setup(real_t q[4]);
void quat_normalize(real_t q[4]);
void quat_normalize_copy(const real_t q[4], real_t q_normalized[4]);
void quat_inv(const real_t q[4], real_t q_inv[4]);
void quat_left(const real_t q[4], real_t left[4 * 4]);
void quat_left_xyz(const real_t q[4], real_t left_xyz[3 * 3]);
void quat_right(const real_t q[4], real_t right[4 * 4]);
void quat_lmul(const real_t p[4], const real_t q[4], real_t r[4]);
void quat_rmul(const real_t p[4], const real_t q[4], real_t r[4]);
void quat_mul(const real_t p[4], const real_t q[4], real_t r[4]);
void quat_delta(const real_t dalpha[3], real_t dq[4]);
void quat_update(real_t q[4], const real_t dalpha[3]);
void quat_update_dt(real_t q[4], const real_t w[3], const real_t dt);
void quat_perturb(real_t q[4], const int i, const real_t h);
void quat_transform(const real_t q[4], const real_t x[3], real_t y[3]);

/*******************************************************************************
 * LIE
 ******************************************************************************/

void lie_Exp(const real_t phi[3], real_t C[3 * 3]);
void lie_Log(const real_t C[3 * 3], real_t rvec[3]);
void box_plus(const real_t C[3 * 3],
              const real_t alpha[3],
              real_t C_new[3 * 3]);
void box_minus(const real_t Ca[3 * 3], const real_t Cb[3 * 3], real_t alpha[3]);
