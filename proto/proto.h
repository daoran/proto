#ifndef PROTO_H
#define PROTO_H

// PROTO SETTINGS
#define PRECISION 2
#define MAX_LINE_LENGTH 9046

#define USE_CBLAS
#define USE_LAPACK
#define USE_SUITESPARSE
// #define USE_CERES
#define USE_STB

// #define USE_GUI
#define USE_APRILGRID
// #define USE_TIS
// #define USE_SBGC

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>
#include <libgen.h>
#include <assert.h>
#include <sys/time.h>

#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>

#ifdef USE_CBLAS
#include <cblas.h>
#endif

#ifdef USE_LAPACK
#include <lapacke.h>
#endif

#ifdef USE_SUITESPARSE
#include <suitesparse/cholmod.h>
#endif

#ifdef USE_CERES
#include "ceres_bridge.h"
#endif

#ifdef USE_STB
#include <stb_image.h>
#include <stb_ds.h>
#endif

#ifdef USE_GUI
#include "gui.h"
#endif

#ifdef USE_APRILGRID
#include "aprilgrid.h"
#endif

#ifdef USE_TIS
#include "tis.h"
#endif

#ifdef USE_SBGC
#include "sbgc.h"
#endif

/******************************************************************************
 * LOGGING / MACROS
 ******************************************************************************/

#ifndef STATUS
#define STATUS __attribute__((warn_unused_result)) int
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
 * @param[in] M Error message
 * @param[in] ... Varadic arguments for error message
 */
#ifndef CHECK
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    LOG_ERROR(M, ##__VA_ARGS__);                                               \
    goto error;                                                                \
  }
#endif

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
      const size_t bwd_idx = (size_t)(BUF_SIZE - 1) / 2.0;                     \
      const size_t fwd_idx = (size_t)(BUF_SIZE + 1) / 2.0;                     \
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

/******************************************************************************
 * FILESYSTEM
 ******************************************************************************/

void path_file_name(const char *path, char *fname);
void path_file_ext(const char *path, char *fext);
void path_dir_name(const char *path, char *dir_name);
char *path_join(const char *x, const char *y);
char **list_files(const char *path, int *nb_files);
void list_files_free(char **data, const int n);
char *file_read(const char *fp);
void skip_line(FILE *fp);
STATUS file_exists(const char *fp);
STATUS file_rows(const char *fp);
STATUS file_copy(const char *src, const char *dest);

/******************************************************************************
 * DATA
 ******************************************************************************/

#if PRECISION == 1
typedef float real_t;
#elif PRECISION == 2
typedef double real_t;
#else
#error "Floating Point Precision not defined!"
#endif

size_t string_copy(char *dst, const char *src);
void string_cat(char *dst, const char *src);
char *string_malloc(const char *s);
char *string_strip(char *s);
char *string_strip_char(char *s, const char c);
char **string_split(char *s, const char d, size_t *n);

int **load_iarrays(const char *csv_path, int *nb_arrays);
real_t **load_darrays(const char *csv_path, int *nb_arrays);

int *int_malloc(const int val);
float *float_malloc(const float val);
double *double_malloc(const double val);
real_t *vector_malloc(const real_t *vec, const real_t N);

int dsv_rows(const char *fp);
int dsv_cols(const char *fp, const char delim);
char **dsv_fields(const char *fp, const char delim, int *nb_fields);
real_t **dsv_data(const char *fp, const char delim, int *nb_rows, int *nb_cols);
void dsv_free(real_t **data, const int nb_rows);

real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols);
void csv_free(real_t **data, const int nb_rows);

/******************************************************************************
 * DATA-STRUCTURES
 ******************************************************************************/

////////////
// DARRAY //
////////////

#ifndef DEFAULT_EXPAND_RATE
#define DEFAULT_EXPAND_RATE 300
#endif

typedef struct darray_t {
  int end;
  int max;
  size_t element_size;
  size_t expand_rate;
  void **contents;
} darray_t;

darray_t *darray_new(size_t element_size, size_t initial_max);
void darray_destroy(darray_t *array);
void darray_clear(darray_t *array);
void darray_clear_destroy(darray_t *array);
int darray_push(darray_t *array, void *el);
void *darray_pop(darray_t *array);
int darray_contains(darray_t *array,
                    void *el,
                    int (*cmp)(const void *, const void *));
darray_t *darray_copy(darray_t *array);
void *darray_new_element(darray_t *array);
void *darray_first(darray_t *array);
void *darray_last(darray_t *array);
void darray_set(darray_t *array, int i, void *el);
void *darray_get(darray_t *array, int i);
void *darray_update(darray_t *array, int i, void *el);
void *darray_remove(darray_t *array, int i);
int darray_expand(darray_t *array);
int darray_contract(darray_t *array);

//////////
// LIST //
//////////

typedef struct list_node_t list_node_t;
struct list_node_t {
  list_node_t *next;
  list_node_t *prev;
  void *value;
};

typedef struct list_t {
  int length;
  list_node_t *first;
  list_node_t *last;
} list_t;

list_t *list_new();
void list_destroy(list_t *list);
void list_clear(list_t *list);
void list_clear_destroy(list_t *list);
void list_push(list_t *list, void *value);
void *list_pop(list_t *list);
void *list_pop_front(list_t *list);
void *list_shift(list_t *list);
void list_unshift(list_t *list, void *value);
void *list_remove(list_t *list,
                  void *target,
                  int (*cmp)(const void *, const void *));
int list_remove_destroy(list_t *list,
                        void *value,
                        int (*cmp)(const void *, const void *),
                        void (*free_func)(void *));

///////////
// STACK //
///////////

typedef struct mstack_node_t mstack_node_t;
struct mstack_node_t {
  void *value;
  mstack_node_t *next;
  mstack_node_t *prev;
};

typedef struct mstack_t {
  int size;
  mstack_node_t *root;
  mstack_node_t *end;
} mstack_t;

mstack_t *stack_new();
void mstack_destroy_traverse(mstack_node_t *n, void (*free_func)(void *));
void mstack_clear_destroy(mstack_t *s, void (*free_func)(void *));
void mstack_destroy(mstack_t *s);
int mstack_push(mstack_t *s, void *value);
void *mstack_pop(mstack_t *s);

///////////
// QUEUE //
///////////

typedef struct queue_t {
  int count;
  list_t *queue;
} queue_t;

queue_t *queue_new();
void queue_destroy(queue_t *q);
int queue_enqueue(queue_t *q, void *data);
void *queue_dequeue(queue_t *q);
int queue_count(queue_t *q);
int queue_empty(queue_t *q);
int queue_full(queue_t *q);
void *queue_first(queue_t *q);
void *queue_last(queue_t *q);

/////////////
// HASHMAP //
/////////////

#ifndef DEFEAULT_NUMBER_OF_BUCKETS
#define DEFAULT_NUMBER_OF_BUCKETS 10000
#endif

typedef struct hashmap_node_t {
  uint32_t hash;
  void *key;
  void *value;
} hashmap_node_t;

typedef struct hashmap_t {
  darray_t *buckets;
  int (*cmp)(void *, void *);
  uint32_t (*hash)(void *);

  int copy_kv;
  void *(*k_copy)(void *);
  void *(*v_copy)(void *);
  void (*k_free)(void *);
  void (*v_free)(void *);
} hashmap_t;

hashmap_t *hashmap_new();
void hashmap_clear_destroy(hashmap_t *map);
void hashmap_destroy(hashmap_t *map);
int hashmap_set(hashmap_t *map, void *key, void *data);
void *hashmap_get(hashmap_t *map, void *key);
int hashmap_traverse(hashmap_t *map,
                     int (*hashmap_traverse_cb)(hashmap_node_t *node));
void *hashmap_delete(hashmap_t *map, void *key);

/******************************************************************************
 * TIME
 ******************************************************************************/

/** Timestamp Type */
#ifndef timestamp_t
typedef int64_t timestamp_t;
#endif

struct timespec tic();
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
timestamp_t time_now();

real_t ts2sec(const timestamp_t ts);
timestamp_t sec2ts(const real_t time_s);

/******************************************************************************
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

int ip_port_info(const int sockfd, char *ip, int *port);

int tcp_server_setup(tcp_server_t *server, const int port);
int tcp_server_loop(tcp_server_t *server);

int tcp_client_setup(tcp_client_t *client,
                     const char *server_ip,
                     const int server_port);
int tcp_client_loop(tcp_client_t *client);

/******************************************************************************
 * MATHS
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
#define PMIN(x, y) ((x) < (y) ? (x) : (y))

/** Max of two numbers, X or Y. */
#define PMAX(x, y) ((x) > (y) ? (x) : (y))

/** Based on sign of b, return +ve or -ve a. */
#define SIGN2(a, b) ((b) > 0.0 ? fabs(a) : -fabs(a))

float randf(float a, float b);
real_t deg2rad(const real_t d);
real_t rad2deg(const real_t r);
int intcmp(const int x, const int y);
int intcmp2(const void *x, const void *y);
int fltcmp(const real_t x, const real_t y);
int fltcmp2(const void *x, const void *y);
int strcmp2(const void *x, const void *y);
real_t pythag(const real_t a, const real_t b);
real_t lerp(const real_t a, const real_t b, const real_t t);
void lerp3(const real_t a[3], const real_t b[3], const real_t t, real_t x[3]);
real_t sinc(const real_t x);
real_t mean(const real_t *x, const size_t length);
real_t median(const real_t *x, const size_t length);
real_t var(const real_t *x, const size_t length);
real_t stddev(const real_t *x, const size_t length);

/******************************************************************************
 * LINEAR ALGEBRA
 ******************************************************************************/

void print_matrix(const char *prefix,
                  const real_t *A,
                  const size_t m,
                  const size_t n);
void print_vector(const char *prefix, const real_t *v, const size_t n);
void vec2str(const real_t *v, const int n, char *s);

void eye(real_t *A, const size_t m, const size_t n);
void ones(real_t *A, const size_t m, const size_t n);
void zeros(real_t *A, const size_t m, const size_t n);

real_t *mat_malloc(const size_t m, const size_t n);
int mat_cmp(const real_t *A, const real_t *B, const size_t m, const size_t n);
int mat_equals(const real_t *A,
               const real_t *B,
               const size_t m,
               const size_t n,
               const real_t tol);
int mat_save(const char *save_path, const real_t *A, const int m, const int n);
real_t *mat_load(const char *save_path, int *nb_rows, int *nb_cols);
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
                 const int nb_rows,
                 const int col_idx,
                 const real_t *x);
void mat_col_get(
    const real_t *A, const int m, const int n, const int col_idx, real_t *x);
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

real_t *vec_malloc(const real_t *x, const size_t n);
void vec_copy(const real_t *src, const size_t n, real_t *dest);
int vec_equals(const real_t *x, const real_t *y, const size_t n);
real_t *vec_load(const char *save_path, int *nb_rows, int *nb_cols);
void vec_add(const real_t *x, const real_t *y, real_t *z, size_t n);
void vec_sub(const real_t *x, const real_t *y, real_t *z, size_t n);
void vec_scale(real_t *x, const size_t n, const real_t scale);
real_t vec_norm(const real_t *x, const size_t n);
void vec_normalize(real_t *x, const size_t n);

#define DOT(A, AM, AN, B, BM, BN, C)                                           \
  real_t C[AM * BN] = {0};                                                     \
  dot(A, AM, AN, B, BM, BN, C);

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

void hat(const real_t x[3], real_t A[3 * 3]);
void vee(const real_t A[3 * 3], real_t x[3]);
void fwdsubs(const real_t *L, const real_t *b, real_t *y, const size_t n);
void bwdsubs(const real_t *U, const real_t *y, real_t *x, const size_t n);
void enforce_spd(real_t *A, const int m, const int n);

int check_inv(const real_t *A, const real_t *A_inv, const int m);
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

int svd(
    const real_t *A, const int m, const int n, real_t *U, real_t *s, real_t *V);
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

/******************************************************************************
 * SUITE-SPARSE
 *****************************************************************************/

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

/******************************************************************************
 * TRANSFORMS
 ******************************************************************************/

#define TF(PARAMS, T)                                                          \
  real_t T[4 * 4] = {0};                                                       \
  tf(PARAMS, T);

#define TF_CR(C, R, T)                                                         \
  real_t T[4 * 4] = {0};                                                       \
  tf_cr(C, R, T);

#define TF_ER(E, R, T)                                                         \
  real_t T[4 * 4] = {0};                                                       \
  tf_er(E, R, T);

#define TF_VECTOR(T, V)                                                        \
  real_t V[7] = {0};                                                           \
  tf_vector(T, V);

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
void tf_chain(const real_t **tfs, const int nb_tfs, real_t T_out[4 * 4]);
void tf_chain2(const int nb_tfs, ...);
void pose_get_trans(const real_t pose[7], real_t r[3]);
void pose_get_quat(const real_t pose[7], real_t q[4]);
void pose_diff(const real_t pose0[7], const real_t pose1[7], real_t diff[6]);
void pose_diff2(const real_t pose0[7],
                const real_t pose1[7],
                real_t *dr,
                real_t *dangle);
void pose_vector_update(real_t pose[7], const real_t dx[6]);
void print_pose_vector(const char *prefix, const real_t pose[7]);
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

/******************************************************************************
 * Lie
 ******************************************************************************/

void lie_Exp(const real_t phi[3], real_t C[3 * 3]);
void lie_Log(const real_t C[3 * 3], real_t rvec[3]);
void box_plus(const real_t C[3 * 3],
              const real_t alpha[3],
              real_t C_new[3 * 3]);
void box_minus(const real_t Ca[3 * 3], const real_t Cb[3 * 3], real_t alpha[3]);

/******************************************************************************
 * CV
 ******************************************************************************/

///////////
// IMAGE //
///////////

typedef struct image_t {
  int width;
  int height;
  int channels;
  uint8_t *data;
} image_t;

void image_setup(image_t *img,
                 const int width,
                 const int height,
                 uint8_t *data);
image_t *image_load(const char *file_path);
void image_print_properties(const image_t *img);
void image_free(image_t *img);

//////////////
// GEOMETRY //
//////////////

void linear_triangulation(const real_t P_i[3 * 4],
                          const real_t P_j[3 * 4],
                          const real_t z_i[2],
                          const real_t z_j[2],
                          real_t p[3]);

////////////
// RADTAN //
////////////

void radtan4_distort(const real_t params[4],
                     const real_t p_in[2],
                     real_t p_out[2]);
void radtan4_undistort(const real_t params[4],
                       const real_t p_in[2],
                       real_t p_out[2]);
void radtan4_point_jacobian(const real_t params[4],
                            const real_t p[2],
                            real_t J_point[2 * 2]);
void radtan4_params_jacobian(const real_t params[4],
                             const real_t p[2],
                             real_t J_param[2 * 4]);

//////////
// EQUI //
//////////

void equi4_distort(const real_t params[4],
                   const real_t p_in[2],
                   real_t p_out[2]);
void equi4_undistort(const real_t params[4],
                     const real_t p_in[2],
                     real_t p_out[2]);
void equi4_point_jacobian(const real_t params[4],
                          const real_t p[2],
                          real_t J_point[2 * 2]);
void equi4_params_jacobian(const real_t params[4],
                           const real_t p[2],
                           real_t J_param[2 * 4]);

/////////////
// PINHOLE //
/////////////

typedef void (*undistort_func_t)(const real_t *params,
                                 const real_t z_in[2],
                                 real_t z_out[2]);

real_t pinhole_focal(const int image_width, const real_t fov);
void pinhole_K(const real_t params[4], real_t K[3 * 3]);
void pinhole_projection_matrix(const real_t params[4],
                               const real_t T[4 * 4],
                               real_t P[3 * 4]);
void pinhole_project(const real_t params[4], const real_t p_C[3], real_t z[2]);
void pinhole_point_jacobian(const real_t params[4], real_t J_point[2 * 2]);
void pinhole_params_jacobian(const real_t params[4],
                             const real_t x[2],
                             real_t J[2 * 4]);

/////////////////////
// PINHOLE-RADTAN4 //
/////////////////////

void pinhole_radtan4_project(const real_t params[8],
                             const real_t p_C[3],
                             real_t z[2]);
void pinhole_radtan4_undistort(const real_t params[8],
                               const real_t z_in[2],
                               real_t z_out[2]);
void pinhole_radtan4_back_project(const real_t params[8],
                                  const real_t z[2],
                                  real_t ray[3]);
void pinhole_radtan4_project_jacobian(const real_t params[8],
                                      const real_t p_C[3],
                                      real_t J[2 * 3]);
void pinhole_radtan4_params_jacobian(const real_t params[8],
                                     const real_t p_C[3],
                                     real_t J[2 * 8]);

///////////////////
// PINHOLE-EQUI4 //
///////////////////

void pinhole_equi4_project(const real_t params[8],
                           const real_t p_C[3],
                           real_t z[2]);
void pinhole_equi4_undistort(const real_t params[8],
                             const real_t z_in[2],
                             real_t z_out[2]);
void pinhole_equi4_back_project(const real_t params[8],
                                const real_t z[2],
                                real_t ray[3]);
void pinhole_equi4_project_jacobian(const real_t params[8],
                                    const real_t p_C[3],
                                    real_t J[2 * 3]);
void pinhole_equi4_params_jacobian(const real_t params[8],
                                   const real_t p_C[3],
                                   real_t J[2 * 8]);

/******************************************************************************
 * SENSOR FUSION
 ******************************************************************************/

#define POSE_PARAM 1
#define EXTRINSIC_PARAM 2
#define FIDUCIAL_PARAM 3
#define VELOCITY_PARAM 4
#define IMU_BIASES_PARAM 5
#define FEATURE_PARAM 6
#define JOINT_PARAM 7
#define CAMERA_PARAM 8

///////////
// UTILS //
///////////

int schurs_complement(const real_t *H,
                      const int H_m,
                      const int H_n,
                      const real_t *b,
                      const int b_m,
                      const int m,
                      const int r,
                      real_t *H_marg,
                      real_t *b_marg);
int shannon_entropy(const real_t *covar, const int m, real_t *entropy);

//////////
// POSE //
//////////

typedef struct pose_t {
  timestamp_t ts;
  real_t data[7];
} pose_t;

void pose_setup(pose_t *pose, const timestamp_t ts, const real_t *param);
void pose_print(const char *prefix, const pose_t *pose);

//////////////
// VELOCITY //
//////////////

typedef struct velocity_t {
  timestamp_t ts;
  real_t data[3];
} velocity_t;

void velocity_setup(velocity_t *vel, const timestamp_t ts, const real_t v[3]);

////////////////
// IMU-BIASES //
////////////////

typedef struct imu_biases_t {
  timestamp_t ts;
  real_t data[6];
} imu_biases_t;

void imu_biases_setup(imu_biases_t *sb,
                      const timestamp_t ts,
                      const real_t ba[3],
                      const real_t bg[3]);
void imu_biases_get_accel_bias(const imu_biases_t *biases, real_t ba[3]);
void imu_biases_get_gyro_bias(const imu_biases_t *biases, real_t bg[3]);

/////////////
// FEATURE //
/////////////

#define MAX_FEATURES 10000

typedef struct feature_t {
  real_t data[3];
} feature_t;

void feature_setup(feature_t *p, const real_t *param);
void feature_print(const feature_t *feature);

typedef struct features_t {
  feature_t data[MAX_FEATURES];
  int nb_features;
  int status[MAX_FEATURES];
} features_t;

void features_setup(features_t *features);
int features_exists(const features_t *features, const int feature_id);
feature_t *features_get(features_t *features, const int feature_id);
feature_t *features_add(features_t *features,
                        const int feature_id,
                        const real_t *param);
void features_remove(features_t *features, const int feature_id);

////////////////
// EXTRINSIC //
////////////////

typedef struct extrinsic_t {
  real_t data[7];
} extrinsic_t;

void extrinsic_setup(extrinsic_t *extrinsic, const real_t *param);
void extrinsic_print(const char *prefix, const extrinsic_t *exts);

//////////////////
// JOINT-ANGLES //
//////////////////

typedef struct joint_t {
  timestamp_t ts;
  int joint_idx;
  real_t data[1];
} joint_t;

void joint_setup(joint_t *joint,
                 const timestamp_t ts,
                 const int joint_idx,
                 const real_t theta);
void joint_print(const char *prefix, const joint_t *joint);

///////////////////////
// CAMERA-PARAMETERS //
///////////////////////

typedef struct camera_params_t {
  int cam_idx;
  int resolution[2];
  char proj_model[30];
  char dist_model[30];
  real_t data[8];
} camera_params_t;

void camera_params_setup(camera_params_t *camera,
                         const int cam_idx,
                         const int cam_res[2],
                         const char *proj_model,
                         const char *dist_model,
                         const real_t *data);
void camera_params_print(const camera_params_t *camera);

////////////
// FACTOR //
////////////

#define FACTOR_EVAL_PTR                                                        \
  int (*factor_eval)(const void *factor,                                       \
                     real_t **params,                                          \
                     real_t *residuals,                                        \
                     real_t **jacobians)

#define CERES_FACTOR_EVAL(FACTOR_TYPE,                                         \
                          FACTOR,                                              \
                          FACTOR_EVAL,                                         \
                          PARAMS,                                              \
                          R_OUT,                                               \
                          J_OUT)                                               \
  {                                                                            \
    assert(FACTOR);                                                            \
    assert(PARAMS);                                                            \
    assert(R_OUT);                                                             \
                                                                               \
    /* Copy parameters */                                                      \
    for (int i = 0; i < FACTOR->num_params; i++) {                             \
      const int global_size = param_global_size(FACTOR->param_types[i]);       \
      vec_copy(PARAMS[i], global_size, FACTOR->params[i]);                     \
    }                                                                          \
                                                                               \
    /* Evaluate factor */                                                      \
    FACTOR_EVAL(factor_ptr);                                                   \
                                                                               \
    /* Residuals */                                                            \
    vec_copy(FACTOR->r, FACTOR->r_size, r_out);                                \
                                                                               \
    /* Jacobians */                                                            \
    if (J_OUT == NULL) {                                                       \
      return 1;                                                                \
    }                                                                          \
                                                                               \
    const int r_size = FACTOR->r_size;                                         \
    for (int jac_idx = 0; jac_idx < FACTOR->num_params; jac_idx++) {           \
      if (J_OUT[jac_idx]) {                                                    \
        const int gs = param_global_size(FACTOR->param_types[jac_idx]);        \
        const int ls = param_local_size(FACTOR->param_types[jac_idx]);         \
        const int rs = 0;                                                      \
        const int re = r_size - 1;                                             \
        const int cs = 0;                                                      \
        const int ce = ls - 1;                                                 \
        zeros(J_OUT[jac_idx], r_size, gs);                                     \
        mat_block_set(J_OUT[jac_idx],                                          \
                      gs,                                                      \
                      rs,                                                      \
                      re,                                                      \
                      cs,                                                      \
                      ce,                                                      \
                      FACTOR->jacs[jac_idx]);                                  \
      }                                                                        \
    }                                                                          \
    return 1;                                                                  \
  }

/////////////////
// POSE-FACTOR //
/////////////////

int check_factor_jacobian(const void *factor,
                          FACTOR_EVAL_PTR,
                          real_t **params,
                          real_t **jacobians,
                          const int r_size,
                          const int param_size,
                          const int param_idx,
                          const real_t step_size,
                          const real_t tol,
                          const int verbose);

int check_factor_so3_jacobian(const void *factor,
                              FACTOR_EVAL_PTR,
                              real_t **params,
                              real_t **jacobians,
                              const int r_size,
                              const int param_idx,
                              const real_t step_size,
                              const real_t tol,
                              const int verbose);

typedef struct pose_factor_t {
  real_t pos_meas[3];
  real_t quat_meas[4];
  pose_t *pose_est;

  real_t covar[6 * 6];
  real_t sqrt_info[6 * 6];

  int r_size;
  int num_params;
  int param_types[1];

  real_t *params[1];
  real_t r[6];
  real_t *jacs[1];
  real_t J_pose[6 * 6];
} pose_factor_t;

void pose_factor_setup(pose_factor_t *factor,
                       pose_t *pose,
                       const real_t var[6]);
int pose_factor_eval(void *factor);

///////////////
// BA-FACTOR //
///////////////

typedef struct ba_factor_t {
  pose_t *pose;
  feature_t *feature;
  camera_params_t *camera;

  real_t covar[2 * 2];
  real_t sqrt_info[2 * 2];
  real_t z[2];

  int r_size;
  int num_params;
  int param_types[3];

  real_t *params[3];
  real_t r[2];
  real_t *jacs[3];
  real_t J_pose[2 * 6];
  real_t J_feature[2 * 3];
  real_t J_camera[2 * 8];
} ba_factor_t;

void ba_factor_setup(ba_factor_t *factor,
                     pose_t *pose,
                     feature_t *feature,
                     camera_params_t *camera,
                     const real_t z[2],
                     const real_t var[2]);
int ba_factor_eval(void *factor_ptr);

///////////////////
// VISION-FACTOR //
///////////////////

typedef struct vision_factor_t {
  pose_t *pose;
  extrinsic_t *extrinsic;
  camera_params_t *camera;
  feature_t *feature;

  real_t covar[2 * 2];
  real_t sqrt_info[2 * 2];
  real_t z[2];

  int r_size;
  int num_params;
  int param_types[4];

  real_t *params[4];
  real_t r[2];
  real_t *jacs[4];
  real_t J_pose[2 * 6];
  real_t J_extrinsic[2 * 6];
  real_t J_feature[2 * 3];
  real_t J_camera[2 * 8];
} vision_factor_t;

void vision_factor_setup(vision_factor_t *factor,
                         pose_t *pose,
                         extrinsic_t *extrinsic,
                         feature_t *feature,
                         camera_params_t *camera,
                         const real_t z[2],
                         const real_t var[2]);
int vision_factor_eval(void *factor_ptr);

////////////////////////
// JOINT-ANGLE FACTOR //
////////////////////////

typedef struct joint_factor_t {
  joint_t *joint;

  real_t z[1];
  real_t covar[1];
  real_t sqrt_info[1];

  int r_size;
  int num_params;
  int param_types[1];

  real_t *params[1];
  real_t r[1];
  real_t *jacs[1];
  real_t J_joint[1 * 1];
} joint_factor_t;

void joint_factor_setup(joint_factor_t *factor,
                        joint_t *joint0,
                        const real_t z,
                        const real_t var);
int joint_factor_eval(void *factor_ptr);

/////////////////////////
// CALIB-GIMBAL-FACTOR //
/////////////////////////

typedef struct calib_gimbal_factor_t {
  extrinsic_t *fiducial_exts;
  extrinsic_t *gimbal_exts;
  pose_t *pose;
  extrinsic_t *link0;
  extrinsic_t *link1;
  joint_t *joint0;
  joint_t *joint1;
  joint_t *joint2;
  extrinsic_t *cam_exts;
  camera_params_t *cam;

  timestamp_t ts;
  int cam_idx;
  int tag_id;
  int corner_idx;
  real_t p_FFi[3];
  real_t z[2];

  real_t covar[2 * 2];
  real_t sqrt_info[2 * 2];

  int r_size;
  int num_params;
  int param_types[10];

  real_t *params[10];
  real_t r[2];
  real_t *jacs[10];
  real_t J_fiducial_exts[2 * 6];
  real_t J_gimbal_exts[2 * 6];
  real_t J_pose[2 * 6];
  real_t J_link0[2 * 6];
  real_t J_link1[2 * 6];
  real_t J_joint0[2 * 1];
  real_t J_joint1[2 * 1];
  real_t J_joint2[2 * 1];
  real_t J_cam_exts[2 * 6];
  real_t J_cam_params[2 * 8];
} calib_gimbal_factor_t;

void gimbal_setup_extrinsic(const real_t ypr[3],
                            const real_t r[3],
                            real_t T[4 * 4],
                            extrinsic_t *link);
void gimbal_setup_joint(const timestamp_t ts,
                        const int joint_idx,
                        const real_t theta,
                        real_t T_joint[4 * 4],
                        joint_t *joint);

void calib_gimbal_factor_setup(calib_gimbal_factor_t *factor,
                               extrinsic_t *fiducial_exts,
                               extrinsic_t *gimbal_exts,
                               pose_t *pose,
                               extrinsic_t *link0,
                               extrinsic_t *link1,
                               joint_t *joint0,
                               joint_t *joint1,
                               joint_t *joint2,
                               extrinsic_t *cam_exts,
                               camera_params_t *cam,
                               const timestamp_t ts,
                               const int cam_idx,
                               const int tag_id,
                               const int corner_idx,
                               const real_t p_FFi[3],
                               const real_t z[2],
                               const real_t var[2]);
int calib_gimbal_factor_eval(void *factor);
int calib_gimbal_factor_ceres_eval(void *factor_ptr,
                                   real_t **params,
                                   real_t *r_out,
                                   real_t **J_out);

////////////////
// IMU-FACTOR //
////////////////

#define IMU_BUF_MAX_SIZE 1000

typedef struct imu_params_t {
  int imu_idx;
  real_t rate;

  real_t sigma_aw;
  real_t sigma_gw;
  real_t sigma_a;
  real_t sigma_g;
  real_t g;
} imu_params_t;

typedef struct imu_buf_t {
  timestamp_t ts[IMU_BUF_MAX_SIZE];
  real_t acc[IMU_BUF_MAX_SIZE][3];
  real_t gyr[IMU_BUF_MAX_SIZE][3];
  int size;
} imu_buf_t;

typedef struct imu_factor_t {
  // IMU buffer and parameters
  imu_buf_t imu_buf;
  imu_params_t *imu_params;

  // Parameters
  int num_params;

  pose_t *pose_i;
  velocity_t *vel_i;
  imu_biases_t *biases_i;
  pose_t *pose_j;
  velocity_t *vel_j;
  imu_biases_t *biases_j;

  real_t *params[6];
  int param_types[6];

  // Preintegration variables
  real_t Dt;         // Time difference between pose_i and pose_j in seconds
  real_t F[15 * 15]; // State jacobian
  real_t P[15 * 15]; // State covariance
  real_t Q[12 * 12]; // Noise matrix
  real_t dr[3];      // Relative position
  real_t dv[3];      // Relative velocity
  real_t dq[4];      // Relative rotation
  real_t ba[3];      // Accel biase
  real_t bg[3];      // Gyro biase

  // Covariance and square-root info
  real_t covar[15 * 15];
  real_t sqrt_info[15 * 15];

  // Residuals
  int r_size;
  real_t r[15];

  // Jacobians
  real_t *jacs[6];
  real_t J_pose_i[15 * 6];
  real_t J_vel_i[15 * 3];
  real_t J_biases_i[15 * 6];
  real_t J_pose_j[15 * 6];
  real_t J_vel_j[15 * 3];
  real_t J_biases_j[15 * 6];
} imu_factor_t;

void imu_buf_setup(imu_buf_t *imu_buf);
void imu_buf_add(imu_buf_t *imu_buf,
                 const timestamp_t ts,
                 const real_t acc[3],
                 const real_t gyr[3]);
void imu_buf_clear(imu_buf_t *imu_buf);
void imu_buf_copy(const imu_buf_t *from, imu_buf_t *to);
void imu_buf_print(const imu_buf_t *imu_buf);

void imu_factor_propagate_step(real_t r[3],
                               real_t v[3],
                               real_t q[4],
                               real_t ba[3],
                               real_t bg[3],
                               const real_t a[3],
                               const real_t w[3],
                               const real_t dt);
void imu_factor_setup(imu_factor_t *factor,
                      imu_params_t *imu_params,
                      imu_buf_t *imu_buf,
                      pose_t *pose_i,
                      velocity_t *v_i,
                      imu_biases_t *biases_i,
                      pose_t *pose_j,
                      velocity_t *v_j,
                      imu_biases_t *biases_j);
void imu_factor_reset(imu_factor_t *factor);
int imu_factor_residuals(imu_factor_t *factor, real_t **params, real_t *r_out);
int imu_factor_eval(void *factor_ptr);

////////////
// SOLVER //
////////////

#define SOLVER_USE_SUITESPARSE

typedef struct param_order_t {
  void *key;
  int idx;
  int type;
  int fix;
} param_order_t;

size_t param_global_size(const int param_type);
size_t param_local_size(const int param_type);

typedef struct solver_t {
  int max_iter;
  real_t lambda;

  param_order_t *(*param_order_func)(const void *data,
                                     int *sv_size,
                                     int *r_size);
  void (*linearize_func)(const void *data,
                         const int sv_size,
                         param_order_t *hash,
                         real_t *H,
                         real_t *g,
                         real_t *r);
} solver_t;

void solver_setup(solver_t *solver);
real_t solver_cost(const real_t *r, const int r_size);
void solver_fill_jacobian(param_order_t *hash,
                          int num_params,
                          real_t **params,
                          real_t **jacs,
                          real_t *r,
                          int r_size,
                          int sv_size,
                          int J_row_idx,
                          real_t *J,
                          real_t *g);
void solver_fill_hessian(param_order_t *hash,
                         int num_params,
                         real_t **params,
                         real_t **jacs,
                         real_t *r,
                         int r_size,
                         int sv_size,
                         real_t *H,
                         real_t *g);
real_t **solver_params_copy(const param_order_t *hash);
void solver_params_restore(param_order_t *hash, real_t **x);
void solver_params_free(const param_order_t *hash, real_t **x);
void solver_update(param_order_t *hash, real_t *dx, int sv_size);
int solver_solve(solver_t *solver, void *data);

/////////////////
// CALIBRATION //
/////////////////

typedef struct calib_gimbal_view_t {
  timestamp_t ts;
  int view_idx;
  int cam_idx;
  int num_corners;

  int *tag_ids;
  int *corner_indices;
  real_t *object_points;
  real_t *keypoints;
  calib_gimbal_factor_t *calib_factors;
} calib_gimbal_view_t;

typedef struct calib_gimbal_t {
  // Settings
  int fix_fiducial_exts;
  int fix_gimbal_exts;
  int fix_poses;
  int fix_cam_params;
  int fix_cam_exts;
  int fix_links;
  int fix_joints;
  aprilgrid_t calib_target;

  // Flags
  int fiducial_ext_ok;
  int gimbal_ext_ok;
  int poses_ok;
  int cams_ok;
  int links_ok;
  int joints_ok;

  // Counters
  int num_cams;
  int num_views;
  int num_poses;
  int num_links;
  int num_joints;
  int num_calib_factors;
  int num_joint_factors;

  // Variables
  timestamp_t *timestamps;
  extrinsic_t fiducial_exts;
  extrinsic_t gimbal_exts;
  extrinsic_t *cam_exts;
  camera_params_t *cam_params;
  extrinsic_t *links;
  joint_t **joints;
  pose_t *poses;

  // Factors
  calib_gimbal_view_t ***views;
  joint_factor_t **joint_factors;
} calib_gimbal_t;

void calib_gimbal_view_setup(calib_gimbal_view_t *calib);
calib_gimbal_view_t *calib_gimbal_view_malloc(const timestamp_t ts,
                                              const int view_idx,
                                              const int cam_idx,
                                              const int *tag_ids,
                                              const int *corner_indices,
                                              const real_t *object_points,
                                              const real_t *keypoints,
                                              const int N);
void calib_gimbal_view_free(calib_gimbal_view_t *calib);

void calib_gimbal_setup(calib_gimbal_t *calib);
calib_gimbal_t *calib_gimbal_malloc();
void calib_gimbal_free(calib_gimbal_t *calib);
void calib_gimbal_print(calib_gimbal_t *calib);
void calib_gimbal_add_fiducial(calib_gimbal_t *calib,
                               const real_t fiducial_pose[7]);
void calib_gimbal_add_pose(calib_gimbal_t *calib,
                           const timestamp_t ts,
                           const real_t pose[7]);
void calib_gimbal_add_gimbal_extrinsic(calib_gimbal_t *calib,
                                       const real_t gimbal_ext[7]);
void calib_gimbal_add_gimbal_link(calib_gimbal_t *calib,
                                  const int link_idx,
                                  const real_t link[7]);
void calib_gimbal_add_camera(calib_gimbal_t *calib,
                             const int cam_idx,
                             const int cam_res[2],
                             const char *proj_model,
                             const char *dist_model,
                             const real_t *cam_params,
                             const real_t *cam_ext);
void calib_gimbal_add_view(calib_gimbal_t *calib,
                           const int pose_idx,
                           const int view_idx,
                           const timestamp_t ts,
                           const int cam_idx,
                           const int num_corners,
                           const int *tag_ids,
                           const int *corner_indices,
                           const real_t *object_points,
                           const real_t *keypoints,
                           const real_t *joints,
                           const int num_joints);
int calib_gimbal_remove_view(calib_gimbal_t *calib, const int view_idx);
calib_gimbal_t *calib_gimbal_load(const char *data_path);
int calib_gimbal_validate(calib_gimbal_t *calib);
void calib_gimbal_nbv(calib_gimbal_t *calib, real_t nbv_joints[3]);
param_order_t *calib_gimbal_param_order(const void *data,
                                        int *sv_size,
                                        int *r_size);
void calib_gimbal_linearize(const void *data,
                            const int J_rows,
                            const int J_cols,
                            param_order_t *hash,
                            real_t *J,
                            real_t *g,
                            real_t *r);
void calib_gimbal_linearize_compact(const void *data,
                                    const int sv_size,
                                    param_order_t *hash,
                                    real_t *H,
                                    real_t *g,
                                    real_t *r);

///////////////////////
// INERTIAL ODOMETRY //
///////////////////////

typedef struct inertial_odometry_t {
  // IMU Parameters
  imu_params_t imu_params;

  // Factors
  int num_factors;
  imu_factor_t *factors;

  // Variables
  pose_t *poses;
  velocity_t *vels;
  imu_biases_t *biases;
} inertial_odometry_t;

void inertial_odometry_free(inertial_odometry_t *odom);
void inertial_odometry_save(const inertial_odometry_t *odom,
                            const char *save_path);
param_order_t *inertial_odometry_param_order(const void *data,
                                             int *sv_size,
                                             int *r_size);
void inertial_odometry_linearize_compact(const void *data,
                                         const int sv_size,
                                         param_order_t *hash,
                                         real_t *H,
                                         real_t *g,
                                         real_t *r);

//////////////////////////////
// VISUAL INERTIAL ODOMETRY //
//////////////////////////////

// typedef struct vio_t {
//   // IMU Parameters
//   imu_params_t imu_params;

//   // Factors
//   int num_factors;
//   imu_factor_t *factors;

//   // Variables
//   pose_t *poses;
//   velocity_t *vels;
//   imu_biases_t *biases;
// } vio_t;

/******************************************************************************
 * DATASET
 ******************************************************************************/

pose_t *load_poses(const char *fp, int *nb_poses);
int **assoc_pose_data(pose_t *gnd_poses,
                      size_t nb_gnd_poses,
                      pose_t *est_poses,
                      size_t nb_est_poses,
                      double threshold,
                      size_t *nb_matches);

/******************************************************************************
 * SIMULATION
 ******************************************************************************/

//////////////////
// SIM FEATURES //
//////////////////

typedef struct sim_features_t {
  real_t **features;
  int nb_features;
} sim_features_t;

sim_features_t *sim_features_load(const char *csv_path);
void sim_features_free(sim_features_t *features_data);

//////////////////
// SIM IMU DATA //
//////////////////

typedef struct sim_imu_data_t {
  real_t **data;
  int nb_measurements;
} sim_imu_data_t;

sim_imu_data_t *sim_imu_data_load(const char *csv_path);
void sim_imu_data_free(sim_imu_data_t *imu_data);

/////////////////////
// SIM CAMERA DATA //
/////////////////////

typedef struct sim_camera_frame_t {
  timestamp_t ts;
  int *feature_ids;
  real_t **keypoints;
  int nb_measurements;
} sim_camera_frame_t;

typedef struct sim_camera_data_t {
  sim_camera_frame_t **frames;
  int nb_frames;

  timestamp_t *ts;
  real_t **poses;
} sim_camera_data_t;

sim_camera_frame_t *sim_camera_frame_load(const char *csv_path);
void sim_camera_frame_print(sim_camera_frame_t *frame_data);
void sim_camera_frame_free(sim_camera_frame_t *frame_data);

sim_camera_data_t *sim_camera_data_load(const char *dir_path);
void sim_camera_data_free(sim_camera_data_t *cam_data);

real_t **sim_create_features(const real_t origin[3],
                             const real_t dim[3],
                             const int nb_features);

/////////////////////
// SIM GIMBAL DATA //
/////////////////////

typedef struct sim_gimbal_t {
  aprilgrid_t grid;

  int num_links;
  int num_joints;
  int num_cams;

  extrinsic_t fiducial_ext;
  pose_t gimbal_pose;
  extrinsic_t gimbal_ext;
  extrinsic_t *gimbal_links;
  joint_t *gimbal_joints;
  extrinsic_t *cam_exts;
  camera_params_t *cam_params;
} sim_gimbal_t;

sim_gimbal_t *sim_gimbal_malloc();
void sim_gimbal_free(sim_gimbal_t *sim);
void sim_gimbal_print(const sim_gimbal_t *sim);
void sim_gimbal_set_joint(sim_gimbal_t *sim,
                          const int joint_idx,
                          const real_t angle);
void sim_gimbal_get_joints(sim_gimbal_t *sim,
                           const int num_joints,
                           real_t *angles);
calib_gimbal_view_t *sim_gimbal3_view(const aprilgrid_t *grid,
                                      const timestamp_t ts,
                                      const int view_idx,
                                      const real_t fiducial_pose[7],
                                      const real_t body_pose[7],
                                      const real_t gimbal_ext[7],
                                      const real_t gimbal_link0[7],
                                      const real_t gimbal_link1[7],
                                      const real_t gimbal_joint0,
                                      const real_t gimbal_joint1,
                                      const real_t gimbal_joint2,
                                      const int cam_idx,
                                      const int cam_res[2],
                                      const real_t cam_params[8],
                                      const real_t cam_ext[7]);
calib_gimbal_view_t *sim_gimbal_view(const sim_gimbal_t *sim,
                                     const timestamp_t ts,
                                     const int view_idx,
                                     const int cam_idx,
                                     const real_t T_WB[4 * 4]);

#endif // PROTO_H
