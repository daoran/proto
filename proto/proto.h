#ifndef PROTO_H
#define PROTO_H

// PROTO SETTINGS
#define PRECISION 2
#define MAX_LINE_LENGTH 9046
// #define USE_DATA_STRUCTURES
#define USE_CBLAS
#define USE_LAPACK
// #define USE_CERES
#define USE_STB_IMAGE
// #define USE_GUI

#ifndef STATUS
#define STATUS __attribute__((warn_unused_result)) int
#endif

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

#include <openssl/sha.h>

#ifdef USE_CBLAS
#include <cblas.h>
#endif

#ifdef USE_LAPACK
#include <lapacke.h>
#endif

#ifdef USE_CERES
#include <ceres/c_api.h>
#endif

#ifdef USE_GUI
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#define SDL_DISABLE_IMMINTRIN_H 1
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#endif

#include "sbgc.h"

/******************************************************************************
 * LOGGING / MACROS
 ******************************************************************************/

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
#define MALLOC(TYPE, N) (TYPE *) malloc(sizeof(TYPE) * (N));
#define CALLOC(TYPE, N) (TYPE *) calloc((N), sizeof(TYPE));
#else
#define MALLOC(TYPE, N) malloc(sizeof(TYPE) * (N));
#define CALLOC(TYPE, N) calloc((N), sizeof(TYPE));
#endif

/**
 * Debug
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#define DEBUG(...)                                                             \
  do {                                                                         \
    fprintf(stderr, "[DEBUG] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);   \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);

/**
 * Log info
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#define LOG_INFO(...)                                                          \
  do {                                                                         \
    fprintf(stderr, "[INFO] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);    \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0)

/**
 * Log error
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#define LOG_ERROR(...)                                                         \
  do {                                                                         \
    fprintf(stderr, "[ERROR] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);   \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0)

/**
 * Log warn
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#define LOG_WARN(...)                                                          \
  do {                                                                         \
    fprintf(stderr, "[WARN] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);    \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0)

/**
 * Fatal
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#define FATAL(...)                                                             \
  do {                                                                         \
    fprintf(stderr, "[FATAL] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);   \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);                                                                 \
  exit(-1)

/**
 * Mark variable unused.
 * @param[in] expr Variable to mark as unused
 */
#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

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
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    LOG_ERROR(M, ##__VA_ARGS__);                                               \
    goto error;                                                                \
  }

/**
 * Free memory
 */
#define FREE_MEM(TARGET, FREE_FUNC)                                            \
  if (TARGET) {                                                                \
    FREE_FUNC((void *) TARGET);                                                \
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

// DARRAY //////////////////////////////////////////////////////////////////////

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

// LIST //////////////////////////////////////////////////////////////////////

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

// STACK /////////////////////////////////////////////////////////////////////

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

// QUEUE /////////////////////////////////////////////////////////////////////

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

// HASHMAP /////////////////////////////////////////////////////////////////////

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
typedef int64_t timestamp_t;

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

// HTTP //////////////////////////////////////////////////////////////////////

/**
 * HTTP Status Code
 */
#define HTTP_STATUS_100 "100 Continue"
#define HTTP_STATUS_101 "101 Switching Protocols"
#define HTTP_STATUS_200 "200 OK"
#define HTTP_STATUS_201 "201 Created"
#define HTTP_STATUS_202 "202 Accepted"
#define HTTP_STATUS_203 "203 Non-Authoritative Information"
#define HTTP_STATUS_204 "204 No Content"
#define HTTP_STATUS_205 "205 Reset Content"
#define HTTP_STATUS_206 "206 Partial Content"
#define HTTP_STATUS_300 "300 Multiple Choices"
#define HTTP_STATUS_301 "301 Moved Permanently"
#define HTTP_STATUS_302 "302 Found"
#define HTTP_STATUS_303 "303 See Other"
#define HTTP_STATUS_304 "304 Not Modified"
#define HTTP_STATUS_305 "305 Use Proxy"
#define HTTP_STATUS_307 "307 Temporary Redirect"
#define HTTP_STATUS_400 "400 Bad Request"
#define HTTP_STATUS_401 "401 Unauthorized"
#define HTTP_STATUS_402 "402 Payment Required"
#define HTTP_STATUS_403 "403 Forbidden"
#define HTTP_STATUS_404 "404 Not Found"
#define HTTP_STATUS_405 "405 Method Not Allowed"
#define HTTP_STATUS_406 "406 Not Acceptable"
#define HTTP_STATUS_407 "407 Proxy Authentication Required"
#define HTTP_STATUS_408 "408 Request Time-out"
#define HTTP_STATUS_409 "409 Conflict"
#define HTTP_STATUS_410 "410 Gone"
#define HTTP_STATUS_411 "411 Length Required"
#define HTTP_STATUS_412 "412 Precondition Failed"
#define HTTP_STATUS_413 "413 Request Entity Too Large"
#define HTTP_STATUS_414 "414 Request-URI Too Large"
#define HTTP_STATUS_415 "415 Unsupported Media Type"
#define HTTP_STATUS_416 "416 Requested range not satisfiable"
#define HTTP_STATUS_417 "417 Expectation Failed"
#define HTTP_STATUS_500 "500 Internal Server Error"
#define HTTP_STATUS_501 "501 Not Implemented"
#define HTTP_STATUS_502 "502 Bad Gateway"
#define HTTP_STATUS_503 "503 Service Unavailable"
#define HTTP_STATUS_504 "504 Gateway Time-out"
#define HTTP_STATUS_505 "505 HTTP Version not supported"

#define WS_FIN 0x80
#define WS_CONT 0x00
#define WS_TEXT 0x01
#define WS_BIN 0x02
#define WS_CLOSE 0x08
#define WS_PING 0x09
#define WS_PONG 0xA
#define WS_MASK_ON 0x80
#define WS_MASK_OFF 0x00

#define WEBSOCKET_HANDSHAKE_RESPONSE                                           \
  "HTTP/1.1 101 Switching Protocols\r\n"                                       \
  "Upgrade: websocket\r\n"                                                     \
  "Connection: Upgrade\r\n"                                                    \
  "Sec-WebSocket-Accept: %s\r\n"                                               \
  "\r\n"

typedef struct http_msg_t {
  // Protocol version
  char *protocol;

  // Request
  char *method;
  char *path;

  // Response
  char *status;

  // Headers
  char *user_agent;
  char *host;
  char *upgrade;
  char *connection;
  char *sec_websocket_key;
  char *sec_websocket_version;
} http_msg_t;

typedef struct ws_frame_t {
  uint8_t header;
  uint8_t mask[4];

  size_t payload_size;
  uint8_t *payload_data;
} ws_frame_t;

char *base64_encode(const uint8_t *data, size_t in_len, size_t *out_len);
uint8_t *base64_decode(const char *data, size_t in_len, size_t *out_len);

void http_msg_setup(http_msg_t *msg);
void http_msg_free(http_msg_t *msg);
void http_msg_print(http_msg_t *msg);
int http_parse_request(char *msg_str, http_msg_t *msg);

ws_frame_t *ws_frame_malloc();
void ws_frame_free(ws_frame_t *frame);
void ws_frame_print(ws_frame_t *frame);
uint8_t *ws_frame_serialize(ws_frame_t *frame);
int ws_frame_fin_bit(uint8_t *data_frame);
int ws_frame_rsv_bit(uint8_t *data_frame);
int ws_frame_op_code(uint8_t *data_frame);
int ws_frame_mask_enabled(uint8_t *data_frame);
ws_frame_t *ws_frame_parse(int connfd);

char *ws_recv(int connfd);
void ws_send(int connfd, const uint8_t *msg);
char *ws_read(ws_frame_t *ws_frame);
char *ws_hash(const char *ws_key);
int ws_handshake(const int connfd);
int ws_server();

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
#define MIN(x, y) ((x) < (y) ? (x) : (y))

/** Max of two numbers, X or Y. */
#define MAX(x, y) ((x) > (y) ? (x) : (y))

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
void mat_block_get(const real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   real_t *block);
void mat_block_set(real_t *A,
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

real_t *vec_malloc(const size_t n);
void vec_copy(const real_t *src, const size_t n, real_t *dest);
int vec_equals(const real_t *x, const real_t *y, const size_t n);
void vec_add(const real_t *x, const real_t *y, real_t *z, size_t n);
void vec_sub(const real_t *x, const real_t *y, real_t *z, size_t n);
void vec_scale(real_t *x, const size_t n, const real_t scale);
real_t vec_norm(const real_t *x, const size_t n);
void vec_normalize(real_t *x, const size_t n);

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

int check_inv(const real_t *A, const real_t *A_inv, const int m);
int check_jacobian(const char *jac_name,
                   const real_t *fdiff,
                   const real_t *jac,
                   const size_t m,
                   const size_t n,
                   const real_t tol,
                   const int verbose);

#define CHECK_POSE_JACOBIAN(JAC_NAME,                                          \
                            JAC_IDX,                                           \
                            R,                                                 \
                            R_SIZE,                                            \
                            PARAMS,                                            \
                            JACS,                                              \
                            FACTOR,                                            \
                            FACTOR_EVAL,                                       \
                            STEP_SIZE,                                         \
                            TOL,                                               \
                            VERBOSE)                                           \
  {                                                                            \
    real_t *r_fwd = MALLOC(real_t, R_SIZE);                                    \
    real_t *r_diff = MALLOC(real_t, R_SIZE);                                   \
    real_t *J = JACS[JAC_IDX];                                                 \
                                                                               \
    /* Check pose position jacobian */                                         \
    char J_name[100] = {'\0'};                                                 \
    strcpy(J_name, JAC_NAME);                                                  \
                                                                               \
    real_t J_fdiff[2 * 6] = {0};                                               \
    for (int i = 0; i < 3; i++) {                                              \
      PARAMS[JAC_IDX][i] += STEP_SIZE;                                         \
      FACTOR_EVAL(FACTOR, PARAMS, r_fwd, NULL);                                \
      PARAMS[JAC_IDX][i] -= STEP_SIZE;                                         \
                                                                               \
      vec_sub(r_fwd, R, r_diff, 2);                                            \
      vec_scale(r_diff, 2, 1.0 / STEP_SIZE);                                   \
      mat_col_set(J_fdiff, 6, 2, i, r_diff);                                   \
    }                                                                          \
    for (int i = 0; i < 3; i++) {                                              \
      quat_perturb(PARAMS[JAC_IDX] + 3, i, STEP_SIZE);                         \
      FACTOR_EVAL(FACTOR, PARAMS, r_fwd, NULL);                                \
      quat_perturb(PARAMS[JAC_IDX] + 3, i, -STEP_SIZE);                        \
                                                                               \
      vec_sub(r_fwd, R, r_diff, 2);                                            \
      vec_scale(r_diff, 2, 1.0 / STEP_SIZE);                                   \
      mat_col_set(J_fdiff, 6, 2, i + 3, r_diff);                               \
    }                                                                          \
    MU_ASSERT(check_jacobian(J_name, J_fdiff, J, 2, 6, TOL, VERBOSE) == 0);    \
                                                                               \
    free(r_fwd);                                                               \
    free(r_diff);                                                              \
  }

/******************************************************************************
 * SVD
 ******************************************************************************/

int svd(real_t *A, const int m, const int n, real_t *U, real_t *s, real_t *V);
void svd_inv(real_t *A, const int m, const int n, real_t *A_inv);

/******************************************************************************
 * CHOL
 ******************************************************************************/

void chol(const real_t *A, const size_t n, real_t *L);
void chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n);

/******************************************************************************
 * TRANSFORMS
 ******************************************************************************/

void rotx(const real_t theta, real_t C[3 * 3]);
void roty(const real_t theta, real_t C[3 * 3]);
void rotz(const real_t theta, real_t C[3 * 3]);
void tf(const real_t params[7], real_t T[4 * 4]);
void tf_cr(const real_t C[3 * 3], const real_t r[3], real_t T[4 * 4]);
void tf_qr(const real_t q[4], const real_t r[3], real_t T[4 * 4]);
void tf_er(const real_t C[3 * 3], const real_t r[3], real_t T[4 * 4]);
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

/******************************************************************************
 * CV
 ******************************************************************************/

// IMAGE ///////////////////////////////////////////////////////////////////////

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

// GEOMETRY //////////////////////////////////////////////////////////////////

void linear_triangulation(const real_t P_i[3 * 4],
                          const real_t P_j[3 * 4],
                          const real_t z_i[2],
                          const real_t z_j[2],
                          real_t p[3]);

// RADTAN ////////////////////////////////////////////////////////////////////

void radtan4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]);
void radtan4_point_jacobian(const real_t params[4],
                            const real_t p[2],
                            real_t J_point[2 * 2]);
void radtan4_params_jacobian(const real_t params[4],
                             const real_t p[2],
                             real_t J_param[2 * 4]);

// EQUI //////////////////////////////////////////////////////////////////////

void equi4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]);
void equi4_point_jacobian(const real_t params[4],
                          const real_t p[2],
                          real_t J_point[2 * 2]);
void equi4_params_jacobian(const real_t params[4],
                           const real_t p[2],
                           real_t J_param[2 * 4]);

// PINHOLE ///////////////////////////////////////////////////////////////////

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

// PINHOLE-RADTAN4 ///////////////////////////////////////////////////////////

void pinhole_radtan4_project(const real_t params[8],
                             const real_t p_C[3],
                             real_t x[2]);
void pinhole_radtan4_project_jacobian(const real_t params[8],
                                      const real_t p_C[3],
                                      real_t J[2 * 3]);
void pinhole_radtan4_params_jacobian(const real_t params[8],
                                     const real_t p_C[3],
                                     real_t J[2 * 8]);

// PINHOLE-EQUI4 /////////////////////////////////////////////////////////////

void pinhole_equi4_project(const real_t params[8],
                           const real_t p_C[3],
                           real_t x[2]);
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
#define SB_PARAM 2
#define FEATURE_PARAM 3
#define EXTRINSICS_PARAM 4
#define JOINT_PARAM 5
#define CAMERA_PARAM 6

// POSE //////////////////////////////////////////////////////////////////////

typedef struct pose_t {
  timestamp_t ts;
  real_t data[7];
} pose_t;

void pose_setup(pose_t *pose, const timestamp_t ts, const real_t *param);
void pose_print(const char *prefix, const pose_t *pose);

// VELOCITY //////////////////////////////////////////////////////////////////

typedef struct velocity_t {
  timestamp_t ts;
  real_t v[3];
} velocity_t;

void velocity_setup(velocity_t *vel, const timestamp_t ts, const real_t v[3]);

// IMU BIASES ////////////////////////////////////////////////////////////////

typedef struct imu_biases_t {
  timestamp_t ts;
  real_t ba[3];
  real_t bg[3];
} imu_biases_t;

void imu_biases_setup(imu_biases_t *sb,
                      const timestamp_t ts,
                      const real_t ba[3],
                      const real_t bg[3]);

// FEATURE ///////////////////////////////////////////////////////////////////

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

// EXTRINSICS ////////////////////////////////////////////////////////////////

typedef struct extrinsics_t {
  real_t data[7];
} extrinsics_t;

void extrinsics_setup(extrinsics_t *extrinsics, const real_t *param);
void extrinsics_print(const char *prefix, const extrinsics_t *exts);

// JOINT ANGLE ///////////////////////////////////////////////////////////////

typedef struct joint_angle_t {
  int joint_idx;
  real_t angle[1];
} joint_angle_t;

void joint_angle_setup(joint_angle_t *joint,
                       const int joint_idx,
                       const real_t theta);
void joint_angle_print(const char *prefix, const joint_angle_t *joint);

// CAMERA PARAMS /////////////////////////////////////////////////////////////

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

// POSE FACTOR ///////////////////////////////////////////////////////////////

#define FACTOR_EVAL_PTR                                                        \
  int (*factor_eval)(const void *factor,                                       \
                     real_t **params,                                          \
                     real_t *residuals,                                        \
                     real_t **jacobians)

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
  int num_params;

  real_t covar[6 * 6];
  real_t sqrt_info[6 * 6];
} pose_factor_t;

void pose_factor_setup(pose_factor_t *factor,
                       pose_t *pose,
                       const real_t var[6]);
int pose_factor_eval(const void *factor,
                     real_t **params,
                     real_t *residuals,
                     real_t **jacobians);

// BA FACTOR /////////////////////////////////////////////////////////////////

typedef struct ba_factor_t {
  const pose_t *pose;
  const camera_params_t *camera;
  const feature_t *feature;
  int num_params;

  real_t covar[2 * 2];
  real_t sqrt_info[2 * 2];
  real_t z[2];
} ba_factor_t;

void ba_factor_setup(ba_factor_t *factor,
                     const pose_t *pose,
                     const feature_t *feature,
                     const camera_params_t *camera,
                     const real_t z[2],
                     const real_t var[2]);
int ba_factor_eval(ba_factor_t *factor,
                   real_t **params,
                   real_t *residuals,
                   real_t **jacobians);

// CAMERA FACTOR /////////////////////////////////////////////////////////////

typedef struct vision_factor_t {
  const pose_t *pose;
  const extrinsics_t *extrinsics;
  const camera_params_t *camera;
  const feature_t *feature;
  int num_params;

  real_t covar[2 * 2];
  real_t sqrt_info[2 * 2];
  real_t z[2];
} vision_factor_t;

void vision_factor_setup(vision_factor_t *factor,
                         const pose_t *pose,
                         const extrinsics_t *extrinsics,
                         const feature_t *feature,
                         const camera_params_t *camera,
                         const real_t z[2],
                         const real_t var[2]);
int vision_factor_eval(vision_factor_t *factor,
                       real_t **params,
                       real_t *residuals,
                       real_t **Jacobians);

// CALIB GIMBAL FACTOR ///////////////////////////////////////////////////////

typedef struct calib_gimbal_factor_t {
  const extrinsics_t *fiducial;
  const extrinsics_t *link0;
  const extrinsics_t *link1;
  const extrinsics_t *link2;
  const joint_angle_t *joint0;
  const joint_angle_t *joint1;
  const joint_angle_t *joint2;
  const extrinsics_t *cam_exts;
  const camera_params_t *cam;
  const feature_t *feature;
  int num_params;

  int tag_id;
  int corner_idx;
  real_t p_FFi[3];
  real_t z[2];

  real_t covar[2 * 2];
  real_t sqrt_info[2 * 2];
} calib_gimbal_factor_t;

void calib_gimbal_factor_setup(calib_gimbal_factor_t *factor,
                               const extrinsics_t *fiducial,
                               const extrinsics_t *link0,
                               const extrinsics_t *link1,
                               const extrinsics_t *link2,
                               const joint_angle_t *joint0,
                               const joint_angle_t *joint1,
                               const joint_angle_t *joint2,
                               const extrinsics_t *cam_exts,
                               const camera_params_t *cam,
                               const int tag_id,
                               const int corner_idx,
                               const real_t p_FFi[3],
                               const real_t z[2],
                               const real_t var[2]);
int calib_gimbal_factor_eval(calib_gimbal_factor_t *factor,
                             real_t **params,
                             real_t *residuals,
                             real_t **jacobians);

// IMU FACTOR ////////////////////////////////////////////////////////////////

#define MAX_IMU_BUF_SIZE 1000

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
  timestamp_t ts[MAX_IMU_BUF_SIZE];
  real_t acc[MAX_IMU_BUF_SIZE][3];
  real_t gyr[MAX_IMU_BUF_SIZE][3];
  int size;
} imu_buf_t;

typedef struct imu_factor_t {
  imu_params_t *imu_params;
  imu_buf_t imu_buf;
  pose_t *pose_i;
  pose_t *pose_j;
  velocity_t *vel_i;
  velocity_t *vel_j;
  imu_biases_t *biases_i;
  imu_biases_t *biases_j;
  int num_params;

  real_t covar[15 * 15];
  real_t sqrt_info[15 * 15];
  real_t r[15];
  int r_size;

  // Preintegration variables
  real_t Dt;
  real_t F[15 * 15]; // State jacobian
  real_t P[15 * 15]; // State covariance
  real_t Q[12 * 12]; // Noise matrix

  real_t dr[3]; // Relative position
  real_t dv[3]; // Relative velocity
  real_t dq[4]; // Relative rotation
  real_t ba[3]; // Accel biase
  real_t bg[3]; // Gyro biase
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
int imu_factor_eval(imu_factor_t *factor,
                    real_t **params,
                    real_t *residuals,
                    real_t **jacobians);

// SOLVER ////////////////////////////////////////////////////////////////////

#define MAX_NB_FACTORS 1000

#define POSE_FACTOR 1
#define BA_FACTOR 2
#define CAM_FACTOR 3
#define IMU_FACTOR 4

typedef struct keyframe_t {
  vision_factor_t *vision_factors;
  int num_vision_factors;

  imu_factor_t *imu_factors;
  int num_imu_factors;

  pose_t *pose;
} keyframe_t;

typedef struct solver_t {
  void *factors[MAX_NB_FACTORS];
  int nb_factors;
  int *factor_types;

  pose_t **poses;
  int nb_poses;

  const extrinsics_t *extrinsics;
  int nb_exts;

  camera_params_t **cam_params;
  int nb_cams;

  feature_t **features;
  int nb_features;

  real_t *H;
  real_t *g;
  real_t *x;
  int x_size;
  int r_size;
} solver_t;

void solver_setup(solver_t *solver);
void solver_print(solver_t *solver);
int solver_add_factor(solver_t *solver, void *factor, int factor_type);
int solver_eval(solver_t *solver);
void solver_solve(solver_t *solver);

// CALIBRATION ///////////////////////////////////////////////////////////////

#define CALIB_GIMBAL_EXPAND_SIZE 1000

typedef struct calib_gimbal_view_t {
  timestamp_t ts;
  int cam_idx;
  int view_idx;
  int num_corners;

  int *tag_ids;
  int *corner_indices;
  real_t **object_points;
  real_t **keypoints;
} calib_gimbal_view_t;

typedef struct calib_gimbal_t {
  extrinsics_t *fiducial;
  extrinsics_t **cam_exts;
  camera_params_t **cam_params;
  int num_cams;

  extrinsics_t **links;
  int num_links;

  joint_angle_t **joints;
  int num_joints;

  calib_gimbal_view_t ***views;
  int num_views;

  int idx_counter;
  void **params;
  int *param_types;
  int *param_indices;
  int *param_sizes;
  int num_params;
} calib_gimbal_t;

void calib_gimbal_view_setup(calib_gimbal_view_t *calib);
void calib_gimbal_view_free(calib_gimbal_view_t *calib);

void calib_gimbal_setup(calib_gimbal_t *calib);
void calib_gimbal_print(calib_gimbal_t *calib);
void calib_gimbal_free(calib_gimbal_t *calib);
calib_gimbal_t *calib_gimbal_load(const char *data_path);
void calib_gimbal_add_param(calib_gimbal_t *calib, void *param, int param_type);
void calib_gimbal_linearize(const calib_gimbal_t *calib,
                            int **param_orders,
                            int *param_sizes,
                            int num_params,
                            real_t *H,
                            int H_m,
                            int H_n,
                            real_t *r,
                            int r_size);

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
 * SIM
 ******************************************************************************/

// SIM FEATURES //////////////////////////////////////////////////////////////

typedef struct sim_features_t {
  real_t **features;
  int nb_features;
} sim_features_t;

sim_features_t *sim_features_load(const char *csv_path);
void sim_features_free(sim_features_t *features_data);

// SIM IMU DATA //////////////////////////////////////////////////////////////

typedef struct sim_imu_data_t {
  real_t **data;
  int nb_measurements;
} sim_imu_data_t;

sim_imu_data_t *sim_imu_data_load(const char *csv_path);
void sim_imu_data_free(sim_imu_data_t *imu_data);

// SIM CAM DATA //////////////////////////////////////////////////////////////

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

/******************************************************************************
 * GUI
 *****************************************************************************/
#ifdef USE_GUI

// OPENGL UTILS //////////////////////////////////////////////////////////////

GLfloat gl_deg2rad(const GLfloat d);
GLfloat gl_rad2deg(const GLfloat r);
void gl_print_vector(const char *prefix, const GLfloat *x, const int length);
void gl_print_matrix(const char *prefix,
                     const GLfloat *A,
                     const int nb_rows,
                     const int nb_cols);
int gl_equals(const GLfloat *A,
              const GLfloat *B,
              const int nb_rows,
              const int nb_cols,
              const GLfloat tol);
void gl_matf_set(GLfloat *A,
                 const int m,
                 const int n,
                 const int i,
                 const int j,
                 const GLfloat val);
GLfloat gl_matf_val(
    const GLfloat *A, const int m, const int n, const int i, const int j);
void gl_copy(const GLfloat *src, const int m, const int n, GLfloat *dest);
void gl_transpose(const GLfloat *A, size_t m, size_t n, GLfloat *A_t);
void gl_zeros(GLfloat *A, const int nb_rows, const int nb_cols);
void gl_ones(GLfloat *A, const int nb_rows, const int nb_cols);
void gl_eye(GLfloat *A, const int nb_rows, const int nb_cols);
void gl_vec2f(GLfloat *v, const GLfloat x, const GLfloat y);
void gl_vec3f(GLfloat *v, const GLfloat x, const GLfloat y, const GLfloat z);
void gl_vec4f(GLfloat *v,
              const GLfloat x,
              const GLfloat y,
              const GLfloat z,
              const GLfloat w);
void gl_vec3f_cross(const GLfloat u[3], const GLfloat v[3], GLfloat n[3]);

void gl_add(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C);
void gl_sub(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C);
void gl_dot(const GLfloat *A,
            const int A_m,
            const int A_n,
            const GLfloat *B,
            const int B_m,
            const int B_n,
            GLfloat *C);
void gl_scale(GLfloat factor, GLfloat *A, const int nb_rows, const int nb_cols);
GLfloat gl_norm(const GLfloat *x, const int size);
void gl_normalize(GLfloat *x, const int size);

void gl_perspective(const GLfloat fov,
                    const GLfloat aspect,
                    const GLfloat near,
                    const GLfloat far,
                    GLfloat P[4 * 4]);
void gl_lookat(const GLfloat eye[3],
               const GLfloat at[3],
               const GLfloat up[3],
               GLfloat V[4 * 4]);

// SHADER ////////////////////////////////////////////////////////////////////

GLuint gl_shader_compile(const char *shader_src, const int type);
GLuint gl_shaders_link(const GLuint vertex_shader,
                       const GLuint fragment_shader,
                       const GLuint geometry_shader);

// GL PROGRAM ////////////////////////////////////////////////////////////////

typedef struct gl_entity_t {
  GLfloat T[4 * 4];

  GLint program_id;
  GLuint vao;
  GLuint vbo;
  GLuint ebo;
} gl_entity_t;

GLuint gl_prog_setup(const char *vs_src,
                     const char *fs_src,
                     const char *gs_src);

int gl_prog_set_int(const GLint id, const char *k, const GLint v);
int gl_prog_set_vec2i(const GLint id, const char *k, const GLint v[2]);
int gl_prog_set_vec3i(const GLint id, const char *k, const GLint v[3]);
int gl_prog_set_vec4i(const GLint id, const char *k, const GLint v[4]);

int gl_prog_set_float(const GLint id, const char *k, const GLfloat v);
int gl_prog_set_vec2f(const GLint id, const char *k, const GLfloat v[2]);
int gl_prog_set_vec3f(const GLint id, const char *k, const GLfloat v[3]);
int gl_prog_set_vec4f(const GLint id, const char *k, const GLfloat v[4]);
int gl_prog_set_mat2f(const GLint id, const char *k, const GLfloat v[2 * 2]);
int gl_prog_set_mat3f(const GLint id, const char *k, const GLfloat v[3 * 3]);
int gl_prog_set_mat4f(const GLint id, const char *k, const GLfloat v[4 * 4]);

// GL-CAMERA /////////////////////////////////////////////////////////////////

typedef struct gl_camera_t {
  int *window_width;
  int *window_height;

  GLfloat focal[3];
  GLfloat world_up[3];
  GLfloat position[3];
  GLfloat right[3];
  GLfloat up[3];
  GLfloat front[3];
  GLfloat yaw;
  GLfloat pitch;
  GLfloat radius;

  GLfloat fov;
  GLfloat near;
  GLfloat far;

  GLfloat P[4 * 4]; // Projection matrix
  GLfloat V[4 * 4]; // View matrix
} gl_camera_t;

void gl_camera_setup(gl_camera_t *camera,
                     int *screen_width,
                     int *screen_height);
void gl_camera_update(gl_camera_t *camera);
void gl_camera_rotate(gl_camera_t *camera,
                      const float factor,
                      const float dx,
                      const float dy);
void gl_camera_pan(gl_camera_t *camera,
                   const float factor,
                   const float dx,
                   const float dy);
void gl_camera_zoom(gl_camera_t *camera,
                    const float factor,
                    const float dx,
                    const float dy);

// GL-PRIMITIVES /////////////////////////////////////////////////////////////

void gl_cube_setup(gl_entity_t *entity, GLfloat pos[3]);
void gl_cube_cleanup(const gl_entity_t *entity);
void gl_cube_draw(const gl_entity_t *entity, const gl_camera_t *camera);

void gl_camera_frame_setup(gl_entity_t *entity);
void gl_camera_frame_cleanup(const gl_entity_t *entity);
void gl_camera_frame_draw(const gl_entity_t *entity, const gl_camera_t *camera);

void gl_axis_frame_setup(gl_entity_t *entity);
void gl_axis_frame_cleanup(const gl_entity_t *entity);
void gl_axis_frame_draw(const gl_entity_t *entity, const gl_camera_t *camera);

void gl_grid_setup(gl_entity_t *entity);
void gl_grid_cleanup(const gl_entity_t *entity);
void gl_grid_draw(const gl_entity_t *entity, const gl_camera_t *camera);

// GUI ///////////////////////////////////////////////////////////////////////

typedef struct gui_t {
  int screen_width;
  int screen_height;

  SDL_Window *window;
  char *window_title;
  int window_width;
  int window_height;
  int loop;

  gl_camera_t camera;
  GLfloat movement_speed;
  GLfloat mouse_sensitivity;

  int left_click;
  int right_click;
  int last_cursor_set;
  float last_cursor_x;
  float last_cursor_y;
} gui_t;

void gui_window_callback(gui_t *gui, const SDL_Event event);
void gui_keyboard_callback(gui_t *gui, const SDL_Event event);
void gui_mouse_callback(gui_t *gui, const SDL_Event event);
void gui_event_handler(gui_t *gui);
void gui_setup(gui_t *gui);
void gui_reset(gui_t *gui);
void gui_loop(gui_t *gui);

// IMSHOW ////////////////////////////////////////////////////////////////////

typedef struct imshow_t {
  SDL_Window *window;
  SDL_Renderer *renderer;

  char *window_title;
  int window_width;
  int window_height;
  int loop;

  SDL_Surface *image_surface;

  gl_camera_t camera;
  GLfloat movement_speed;
  GLfloat mouse_sensitivity;

  int left_click;
  int right_click;
  int last_cursor_set;
  float last_cursor_x;
  float last_cursor_y;
} imshow_t;

void imshow_window_callback(imshow_t *imshow, const SDL_Event event);
void imshow_keyboard_callback(imshow_t *imshow, const SDL_Event event);
void imshow_event_handler(imshow_t *gui);
void imshow_setup(imshow_t *imshow, const char *fp);
void imshow_reset(imshow_t *imshow);
void imshow_loop(imshow_t *imshow);
#endif // USE_GUI

#endif // PROTO_H

//////////////////////////////////////////////////////////////////////////////
//                             IMPLEMENTATION                               //
//////////////////////////////////////////////////////////////////////////////

#ifdef PROTO_IMPLEMENTATION

#ifdef USE_STB_IMAGE
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#endif

#ifdef USE_SBGC
#define SBGC_IMPLEMENTATION
#include "sbgc.h"
#endif

/******************************************************************************
 * FILE SYSTEM
 ******************************************************************************/

/**
 * Extract filename from `path` to `fname`.
 */
void path_file_name(const char *path, char *fname) {
  assert(path != NULL);
  assert(fname != NULL);

  char path_copy[9046] = {0};
  memcpy(path_copy, path, strlen(path));

  char *base = strrchr(path_copy, '/');
  base = base ? base + 1 : path_copy;

  memcpy(fname, base, strlen(base));
}

/**
 * Extract file extension from `path` to `fext`.
 */
void path_file_ext(const char *path, char *fext) {
  assert(path != NULL);
  assert(fext != NULL);

  char path_copy[9046] = {0};
  memcpy(path_copy, path, strlen(path));

  char *base = strrchr(path_copy, '.');
  if (base) {
    base = base ? base + 1 : path_copy;
    memcpy(fext, base, strlen(base));
  } else {
    fext[0] = '\0';
  }
}

/**
 * Extract dir name from `path` to `dirname`.
 */
void path_dir_name(const char *path, char *dir_name) {
  assert(path != NULL);
  assert(dir_name != NULL);

  char path_copy[9046] = {0};
  memcpy(path_copy, path, strlen(path));

  char *base = strrchr(path_copy, '/');
  memcpy(dir_name, path_copy, base - path_copy);
}

/**
 * Join two paths `x` and `y`
 */
char *path_join(const char *x, const char *y) {
  assert(x != NULL && y != NULL);

  char *retval = NULL;
  if (x[strlen(x) - 1] == '/') {
    retval = MALLOC(char, (strlen(x) + strlen(y)) + 1);
    string_copy(retval, x);
    string_copy(retval + strlen(retval), (y[0] == '/') ? y + 1 : y);
  } else {
    retval = MALLOC(char, (strlen(x) + strlen(y)) + 2);
    string_copy(retval, x);
    string_cat(retval + strlen(retval), "/");
    string_copy(retval + strlen(retval), (y[0] == '/') ? y + 1 : y);
  }

  return retval;
}

/**
 * List files in `path`.
 * @returns List of files in directory and number of files `n`.
 */
char **list_files(const char *path, int *n) {
  assert(path != NULL);
  assert(n != NULL);

  struct dirent **namelist;
  int nb_files = scandir(path, &namelist, 0, alphasort);
  if (nb_files < 0) {
    return NULL;
  }

  // The first two are '.' and '..'
  free(namelist[0]);
  free(namelist[1]);

  // Allocate memory for list of files
  char **files = MALLOC(char *, nb_files - 2);
  *n = 0;

  // Create list of files
  for (int i = 2; i < nb_files; i++) {
    char fp[9046] = {0};
    const char *c = (path[strlen(path) - 1] == '/') ? "" : "/";
    string_cat(fp, path);
    string_cat(fp, c);
    string_cat(fp, namelist[i]->d_name);

    files[*n] = MALLOC(char, strlen(fp) + 1);
    memcpy(files[*n], fp, strlen(fp));
    files[*n][strlen(fp)] = '\0'; // strncpy does not null terminate
    (*n)++;

    free(namelist[i]);
  }
  free(namelist);

  return files;
}

/**
 * Free list of `files` of length `n`.
 */
void list_files_free(char **data, const int n) {
  assert(data != NULL);
  for (int i = 0; i < n; i++) {
    free(data[i]);
  }
  free(data);
}

/**
 * Read file contents in file path `fp`.
 * @returns
 * - Success: File contents
 * - Failure: NULL
 */
char *file_read(const char *fp) {
  assert(fp != NULL);
  FILE *f = fopen(fp, "rb");
  if (f == NULL) {
    return NULL;
  }

  fseek(f, 0, SEEK_END);
  long int len = ftell(f);
  fseek(f, 0, SEEK_SET);

  char *buf = MALLOC(char, len + 1);
  if (buf == NULL) {
    return NULL;
  }
  const size_t read = fread(buf, 1, len, f);
  if (read != len) {
    FATAL("Failed to read file [%s]\n", fp);
  }
  buf[len] = '\0';
  fclose(f);

  return buf;
}

/**
 * Skip line in file.
 */
void skip_line(FILE *fp) {
  assert(fp != NULL);

  char header[BUFSIZ];
  char *retval = fgets(header, BUFSIZ, fp);
  if (retval == NULL) {
    FATAL("Failed to skip line!");
  }
}

/**
 * Check if file exists.
 * @returns
 * - 1 File exists
 * - 0 File does not exist
 */
int file_exists(const char *fp) {
  return (access(fp, F_OK) == 0) ? 1 : 0;
}

/**
 * Get number of rows in file `fp`.
 * @returns
 * - Number of rows in file
 * - -1 for failure.
 */
int file_rows(const char *fp) {
  assert(fp != NULL);

  FILE *file = fopen(fp, "rb");
  if (file == NULL) {
    fclose(file);
    return -1;
  }

  // Obtain number of lines
  int nb_rows = 0;
  char *line = NULL;
  size_t len = 0;
  while (getline(&line, &len, file) != -1) {
    nb_rows++;
  }
  free(line);

  // Clean up
  fclose(file);

  return nb_rows;
}

/**
 * Copy file from path `src` to path `dst`.
 * @returns
 * - 0 for success
 * - -1 if src file could not be opend
 * - -2 if dst file could not be opened
 */
int file_copy(const char *src, const char *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  FILE *src_file = fopen(src, "rb");
  if (src_file == NULL) {
    fclose(src_file);
    return -1;
  }

  FILE *dst_file = fopen(dst, "wb");
  if (dst_file == NULL) {
    fclose(src_file);
    fclose(dst_file);
    return -2;
  }

  char *line = NULL;
  size_t len = 0;
  ssize_t read = 0;
  while ((read = getline(&line, &len, src_file)) != -1) {
    fwrite(line, sizeof(char), read, dst_file);
  }
  if (line) {
    free(line);
  }

  // Clean up
  fclose(src_file);
  fclose(dst_file);

  return 0;
}

/******************************************************************************
 * DATA
 ******************************************************************************/

/**
 * String copy from `src` to `dst`.
 */
size_t string_copy(char *dst, const char *src) {
  dst[0] = '\0';
  memcpy(dst, src, strlen(src));
  dst[strlen(src)] = '\0'; // Null terminate
  return strlen(dst);
}

/**
 * Concatenate string from `src` to `dst`.
 */
void string_cat(char *dst, const char *src) {
  size_t dst_len = strlen(dst);
  strcat(dst + dst_len, src);
  dst[dst_len + strlen(src)] = '\0'; // strncat does not null terminate
}

/**
 * Allocate heap memory for string `s`.
 */
char *string_malloc(const char *s) {
  assert(s != NULL);
  char *retval = MALLOC(char, strlen(s) + 1);
  memcpy(retval, s, strlen(s));
  retval[strlen(s)] = '\0'; // Null terminate
  return retval;
}

/**
 * Strip whitespace from string `s`.
 */
char *string_strip(char *s) {
  char *end;

  // Trim leading space
  while (*s == ' ') {
    s++;
  }

  if (*s == 0) { // All spaces?
    return s;
  }

  // Trim trailing space
  end = s + strlen(s) - 1;
  while (end > s && (*end == ' ' || *end == '\n')) {
    end--;
  }

  // Write new null terminator character
  end[1] = '\0';

  return s;
}

/**
 * Strip specific character `c` from string `s`.
 */
char *string_strip_char(char *s, const char c) {
  char *end;

  // Trim leading space
  while (*s == c) {
    s++;
  }

  if (*s == 0) { // All spaces?
    return s;
  }

  // Trim trailing space
  end = s + strlen(s) - 1;
  while (end > s && *end == c) {
    end--;
  }

  // Write new null terminator character
  end[1] = '\0';

  return s;
}

/**
 * Parse integer array line.
 * @returns
 * - 1D vector of integers
 * - NULL for failure
 */
static int *parse_iarray_line(char *line) {
  assert(line != NULL);
  char entry[MAX_LINE_LENGTH] = {0};
  int index = 0;
  int *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = CALLOC(int, array_size + 1);
      }
      data[index] = strtod(entry, NULL);
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return data;
}

/**
 * Parse 2D integer arrays from csv file.
 * @returns
 * - List of 1D vector of integers
 * - NULL for failure
 */
int **load_iarrays(const char *csv_path, int *nb_arrays) {
  assert(csv_path != NULL);
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = dsv_rows(csv_path);
  int **array = CALLOC(int *, *nb_arrays);

  char line[MAX_LINE_LENGTH] = {0};
  int frame_idx = 0;
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    array[frame_idx] = parse_iarray_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return array;
}

/**
 * Parse real array line.
 * @returns
 * - 1D vector of real
 * - NULL for failure
 */
static real_t *parse_darray_line(char *line) {
  assert(line != NULL);
  char entry[MAX_LINE_LENGTH] = {0};
  int index = 0;
  real_t *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = CALLOC(real_t, array_size);
      }
      data[index] = strtod(entry, NULL);
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return data;
}

/**
 * Parse 2D real arrays from csv file at `csv_path`, on success `nb_arrays`
 * will return number of arrays.
 * @returns
 * - List of 1D vector of reals
 * - NULL for failure
 */
real_t **load_darrays(const char *csv_path, int *nb_arrays) {
  assert(csv_path != NULL);
  assert(nb_arrays != NULL);
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = dsv_rows(csv_path);
  real_t **array = CALLOC(real_t *, *nb_arrays);

  char line[MAX_LINE_LENGTH] = {0};
  int frame_idx = 0;
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    array[frame_idx] = parse_darray_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return array;
}

/**
 * Allocate heap memory for integer `val`.
 */
int *int_malloc(const int val) {
  int *i = MALLOC(int, 1);
  *i = val;
  return i;
}

/**
 * Allocate heap memory for float `val`.
 */
float *float_malloc(const float val) {
  float *f = MALLOC(float, 1);
  *f = val;
  return f;
}

/**
 * Allocate heap memory for double `val`.
 */
double *double_malloc(const double val) {
  double *d = MALLOC(double, 1);
  *d = val;
  return d;
}

/**
 * Allocate heap memory for vector `vec` with length `N`.
 */
real_t *vector_malloc(const real_t *vec, const real_t N) {
  real_t *retval = MALLOC(real_t, N);
  for (int i = 0; i < N; i++) {
    retval[i] = vec[i];
  }
  return retval;
}

/**
 * Get number of rows in a delimited file at `fp`.
 * @returns
 * - Number of rows
 * - -1 for failure
 */
int dsv_rows(const char *fp) {
  assert(fp != NULL);

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return -1;
  }

  // Loop through lines
  int nb_rows = 0;
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      nb_rows++;
    }
  }

  // Cleanup
  fclose(infile);

  return nb_rows;
}

/**
 * Get number of columns in a delimited file at `fp`.
 * @returns
 * - Number of columns
 * - -1 for failure
 */
int dsv_cols(const char *fp, const char delim) {
  assert(fp != NULL);

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return -1;
  }

  // Get line that isn't the header
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      break;
    }
  }

  // Parse line to obtain number of elements
  int nb_elements = 1;
  int found_separator = 0;
  for (size_t i = 0; i < MAX_LINE_LENGTH; i++) {
    if (line[i] == delim) {
      found_separator = 1;
      nb_elements++;
    }
  }

  // Cleanup
  fclose(infile);

  return (found_separator) ? nb_elements : -1;
}

/**
 * Get the fields of the delimited file at `fp`, where `delim` is the value
 * separated symbol and `nb_fields` returns the length of the fields returned.
 * @returns
 * - List of field strings
 * - NULL for failure
 */
char **dsv_fields(const char *fp, const char delim, int *nb_fields) {
  assert(fp != NULL);

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  // Get last header line
  char field_line[MAX_LINE_LENGTH] = {0};
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      break;
    } else {
      memcpy(field_line, line, strlen(line));
    }
  }

  // Parse fields
  *nb_fields = dsv_cols(fp, delim);
  char **fields = MALLOC(char *, *nb_fields);
  int field_idx = 0;
  char field_name[100] = {0};

  for (size_t i = 0; i < strlen(field_line); i++) {
    char c = field_line[i];

    // Ignore # and ' '
    if (c == '#' || c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      // Add field name to fields
      fields[field_idx] = string_malloc(field_name);
      memset(field_name, '\0', 100);
      field_idx++;
    } else {
      // Append field name
      field_name[strlen(field_name)] = c;
    }
  }

  // Cleanup
  fclose(infile);

  return fields;
}

/**
 * Load delimited separated value data as a matrix.
 * @returns
 * - Matrix of DSV data
 * - NULL for failure
 */
real_t **
dsv_data(const char *fp, const char delim, int *nb_rows, int *nb_cols) {
  assert(fp != NULL);

  // Obtain number of rows and columns in dsv data
  *nb_rows = dsv_rows(fp);
  *nb_cols = dsv_cols(fp, delim);
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  // Loop through data
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;

  // Loop through data line by line
  real_t **data = MALLOC(real_t *, *nb_rows);
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    // Ignore if comment line
    if (line[0] == '#') {
      continue;
    }

    // Iterate through values in line separated by commas
    data[row_idx] = MALLOC(real_t, *nb_cols);
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        data[row_idx][col_idx] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  // Clean up
  fclose(infile);

  return data;
}

/**
 * Free DSV data.
 */
void dsv_free(real_t **data, const int nb_rows) {
  assert(data != NULL);
  for (int i = 0; i < nb_rows; i++) {
    free(data[i]);
  }
  free(data);
}

/**
 * Load comma separated data as a matrix, where `fp` is the csv file path, on
 * success `nb_rows` and `nb_cols` will be filled.
 * @returns
 * - Matrix of CSV data
 * - NULL for failure
 */
real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols) {
  assert(fp != NULL);
  return dsv_data(fp, ',', nb_rows, nb_cols);
}

/**
 * Free CSV data.
 */
void csv_free(real_t **data, const int nb_rows) {
  for (int i = 0; i < nb_rows; i++) {
    free(data[i]);
  }
  free(data);
}

/******************************************************************************
 * DATA-STRUCTURES
 ******************************************************************************/

#ifdef USE_DATA_STRUCTURES

// DARRAY ////////////////////////////////////////////////////////////////////

darray_t *darray_new(size_t element_size, size_t initial_max) {
  assert(element_size > 0);
  assert(initial_max > 0);

  darray_t *array = MALLOC(darray_t, 1);
  if (array == NULL) {
    return NULL;
  }

  array->end = 0;
  array->max = (int) initial_max;
  array->element_size = element_size;
  array->expand_rate = DEFAULT_EXPAND_RATE;
  array->contents = calloc(initial_max, sizeof(void *));
  if (array->contents == NULL) {
    free(array);
    return NULL;
  }

  return array;
}

void darray_clear(darray_t *array) {
  assert(array != NULL);
  for (int i = 0; i < array->max; i++) {
    FREE_MEM(array->contents[i], free);
  }
}

void darray_destroy(darray_t *array) {
  if (array) {
    if (array->contents) {
      free(array->contents);
    }
    free(array);
  }
}

void darray_clear_destroy(darray_t *array) {
  if (array) {
    darray_clear(array);
    darray_destroy(array);
  }
}

int darray_push(darray_t *array, void *el) {
  assert(array != NULL);

  // Push
  array->contents[array->end] = el;
  array->end++;

  // Expand darray if necessary
  if (array->end >= array->max) {
    return darray_expand(array);
  }

  return 0;
}

void *darray_pop(darray_t *array) {
  assert(array != NULL);

  // pop
  void *el = darray_remove(array, array->end - 1);
  array->end--;

  // contract
  int expanded = array->end > (int) array->expand_rate;
  int trailing_memory = array->end % (int) array->expand_rate;
  if (expanded && trailing_memory) {
    darray_contract(array);
  }

  return el;
}

int darray_contains(darray_t *array,
                    void *el,
                    int (*cmp)(const void *, const void *)) {
  assert(array != NULL);
  assert(el != NULL);
  assert(cmp != NULL);

  // Check first element
  void *element = darray_get(array, 0);
  if (element != NULL && cmp(element, el) == 0) {
    return 1;
  }

  // Rest of element
  for (int i = 0; i < array->end; i++) {
    element = darray_get(array, i);
    if (element != NULL && cmp(element, el) == 0) {
      return 1;
    }
  }

  return 0;
}

darray_t *darray_copy(darray_t *array) {
  assert(array != NULL);

  // Copy first element
  darray_t *array_copy = darray_new(array->element_size, (size_t) array->max);
  void *el = darray_get(array, 0);
  void *el_copy = NULL;

  if (el != NULL) {
    el_copy = darray_new_element(array_copy);
    memcpy(el_copy, el, array->element_size);
    darray_set(array_copy, 0, el_copy);
  }

  // Copy the rest of the elements
  for (int i = 1; i < array->end; i++) {
    el = darray_get(array, i);
    // el_copy = NULL;

    if (el != NULL) {
      memcpy(el_copy, el, array->element_size);
      darray_set(array_copy, i, el);
    }
  }

  return array_copy;
}

void *darray_new_element(darray_t *array) {
  assert(array != NULL);
  assert(array->element_size > 0);
  return calloc(1, array->element_size);
}

void *darray_first(darray_t *array) {
  assert(array != NULL);
  return array->contents[0];
}

void *darray_last(darray_t *array) {
  assert(array != NULL);
  return array->contents[array->end - 1];
}

void darray_set(darray_t *array, int i, void *el) {
  assert(array != NULL);
  assert(i < array->max);

  // Set
  array->contents[i] = el;

  // Update end
  if (i > array->end) {
    array->end = i;
  }
}

void *darray_get(darray_t *array, int i) {
  assert(array != NULL);
  assert(i < array->max);
  return array->contents[i];
}

void *darray_update(darray_t *array, int i, void *el) {
  assert(array != NULL);
  assert(i < array->max);
  void *old_el;

  // Update
  old_el = darray_get(array, i);
  darray_set(array, i, el);

  return old_el;
}

void *darray_remove(darray_t *array, int i) {
  assert(array != NULL);
  void *el = array->contents[i];
  array->contents[i] = NULL;
  return el;
}

static inline int darray_resize(darray_t *array, size_t new_max) {
  assert(array != NULL);

  // Calculate new max and size
  int old_max = (int) array->max;
  array->max = (int) new_max;

  // Reallocate new memory
  void *contents = realloc(array->contents, new_max * sizeof(void *));
  if (contents == NULL) {
    return -1;
  }
  array->contents = contents;

  // Initialize new memory to NULL
  for (int i = old_max; i < (int) new_max; i++) {
    array->contents[i] = NULL;
  }

  return 0;
}

int darray_expand(darray_t *array) {
  assert(array != NULL);
  assert(array->max > 0);

  size_t old_max = (size_t) array->max;
  size_t new_max = (size_t) array->max + array->expand_rate;
  int res = darray_resize(array, new_max);
  if (res != 0) {
    return -1;
  }
  memset(array->contents + old_max, 0, array->expand_rate + 1);

  return 0;
}

int darray_contract(darray_t *array) {
  assert(array != NULL);
  assert(array->max > 0);

  // Contract
  int new_size = 0;
  if (array->end < (int) array->expand_rate) {
    new_size = (int) array->expand_rate;
  } else {
    new_size = array->end;
  }

  return darray_resize(array, (size_t) new_size + 1);
}

// LIST //////////////////////////////////////////////////////////////////////

list_t *list_new() {
  list_t *list = calloc(1, sizeof(list_t));
  list->length = 0;
  list->first = NULL;
  list->last = NULL;
  return list;
}

void list_destroy(list_t *list) {
  assert(list != NULL);

  list_node_t *node;
  list_node_t *next_node;

  // Destroy
  node = list->first;
  while (node != NULL) {
    next_node = node->next;
    FREE_MEM(node, free);
    node = next_node;
  }

  free(list);
}

void list_clear(list_t *list) {
  assert(list != NULL);

  list_node_t *node;
  list_node_t *next_node;

  node = list->first;
  while (node != NULL) {
    next_node = node->next;
    free(node->value);
    node = next_node;
  }
}

void list_clear_destroy(list_t *list) {
  assert(list != NULL);

  list_node_t *node = list->first;
  while (node != NULL) {
    list_node_t *next_node = node->next;
    free(node->value);
    free(node);
    node = next_node;
  }
  free(list);
}

void list_push(list_t *list, void *value) {
  assert(list != NULL);
  assert(value != NULL);

  // Initialize node
  list_node_t *node = calloc(1, sizeof(list_node_t));
  if (node == NULL) {
    return;
  }
  node->value = value;

  // Push node
  if (list->last == NULL) {
    list->first = node;
    list->last = node;
  } else {
    list->last->next = node;
    node->prev = list->last;
    list->last = node;
  }

  list->length++;
}

void *list_pop(list_t *list) {
  assert(list != NULL);

  // Get last
  list_node_t *last = list->last;
  if (last == NULL) {
    return NULL;
  }
  void *value = last->value;
  list_node_t *before_last = last->prev;
  free(last);

  // Pop
  if (before_last == NULL && list->length == 1) {
    list->last = NULL;
    list->first = NULL;
  } else {
    list->last = before_last;
  }
  list->length--;

  if (before_last == NULL) {
    return NULL;
  }
  before_last->next = NULL;

  return value;
}

void *list_pop_front(list_t *list) {
  assert(list != NULL);
  assert(list->first != NULL);

  // pop front
  list_node_t *first_node = list->first;
  void *data = first_node->value;
  list_node_t *next_node = first_node->next;

  if (next_node != NULL) {
    list->first = next_node;
  } else {
    list->first = NULL;
  }
  list->length--;

  // clean up
  free(first_node);

  return data;
}

void *list_shift(list_t *list) {
  assert(list != NULL);

  list_node_t *first = list->first;
  void *value = first->value;
  list_node_t *second = list->first->next;

  list->first = second;
  list->length--;
  free(first);

  return value;
}

void list_unshift(list_t *list, void *value) {
  assert(list != NULL);

  list_node_t *node = calloc(1, sizeof(list_node_t));
  if (node == NULL) {
    return;
  }
  node->value = value;

  if (list->first == NULL) {
    list->first = node;
    list->last = node;
  } else {
    node->next = list->first;
    list->first->prev = node;
    list->first = node;
  }

  list->length++;
}

void *list_remove(list_t *list,
                  void *value,
                  int (*cmp)(const void *, const void *)) {
  assert(list != NULL);
  assert(value != NULL);
  assert(cmp != NULL);

  // Iterate list
  list_node_t *node = list->first;
  while (node != NULL) {

    // Compare target with node value
    if (cmp(node->value, value) == 0) {
      value = node->value;

      if (list->length == 1) {
        // Last node in list
        list->first = NULL;
        list->last = NULL;

      } else if (node == list->first) {
        // First node in list
        list->first = node->next;
        node->next->prev = NULL;

      } else if (node == list->last) {
        // In the case of removing last node in list
        list->last = node->prev;
        node->prev->next = NULL;

      } else {
        // Remove others
        node->prev->next = node->next;
        node->next->prev = node->prev;
      }
      list->length--;
      free(node);

      return value;
    }

    node = node->next;
  }

  return NULL;
}

int list_remove_destroy(list_t *list,
                        void *value,
                        int (*cmp)(const void *, const void *),
                        void (*free_func)(void *)) {
  assert(list != NULL);
  void *result = list_remove(list, value, cmp);
  free_func(result);
  return 0;
}

// STACK /////////////////////////////////////////////////////////////////////

mstack_t *stack_new() {
  mstack_t *s = MALLOC(mstack_t, 1);
  s->size = 0;
  s->root = NULL;
  s->end = NULL;
  return s;
}

void mstack_destroy_traverse(mstack_node_t *n, void (*free_func)(void *)) {
  if (n->next) {
    mstack_destroy_traverse(n->next, free_func);
  }
  if (free_func) {
    free_func(n->value);
  }
  free(n);
  n = NULL;
}

void mstack_clear_destroy(mstack_t *s, void (*free_func)(void *)) {
  if (s->root) {
    mstack_destroy_traverse(s->root, free_func);
  }
  free(s);
  s = NULL;
}

void mstack_destroy(mstack_t *s) {
  if (s->root) {
    mstack_destroy_traverse(s->root, NULL);
  }
  free(s);
  s = NULL;
}

int mstack_push(mstack_t *s, void *value) {
  mstack_node_t *n = MALLOC(mstack_node_t, 1);
  if (n == NULL) {
    return -1;
  }

  mstack_node_t *prev_end = s->end;
  n->value = value;
  n->next = NULL;
  n->prev = prev_end;

  if (s->size == 0) {
    s->root = n;
    s->end = n;
  } else {
    prev_end->next = n;
    s->end = n;
  }
  s->size++;

  return 0;
}

void *mstack_pop(mstack_t *s) {
  void *value = s->end->value;
  mstack_node_t *previous = s->end->prev;

  free(s->end);
  if (s->size > 1) {
    previous->next = NULL;
    s->end = previous;
  } else {
    s->root = NULL;
    s->end = NULL;
  }
  s->size--;

  return value;
}

// QUEUE /////////////////////////////////////////////////////////////////////

queue_t *queue_new() {
  queue_t *q = calloc(1, sizeof(queue_t));
  q->queue = list_new();
  q->count = 0;
  return q;
}

void queue_destroy(queue_t *q) {
  assert(q != NULL);
  list_destroy(q->queue);
  free(q);
  q = NULL;
}

int queue_enqueue(queue_t *q, void *data) {
  assert(q != NULL);
  list_push(q->queue, data);
  q->count++;
  return 0;
}

void *queue_dequeue(queue_t *q) {
  assert(q != NULL);
  void *data = list_pop_front(q->queue);
  q->count--;

  return data;
}

int queue_empty(queue_t *q) {
  assert(q != NULL);
  return (q->count == 0) ? 1 : 0;
}

void *queue_first(queue_t *q) {
  assert(q != NULL);
  if (q->count != 0) {
    return q->queue->first->value;
  }
  return NULL;
}

void *queue_last(queue_t *q) {
  assert(q != NULL);
  if (q->count != 0) {
    return q->queue->last->value;
  }
  return NULL;
}

// HASHMAP ///////////////////////////////////////////////////////////////////

static inline int default_cmp(void *a, void *b) {
  return strcmp(a, b);
}

static uint32_t default_hash(void *a) {
  // Simple bob jenkins's hash algorithm
  char *k = a;
  uint32_t hash = 0;
  for (uint32_t i = 0; i < strlen(a); i++) {
    hash += (uint32_t) k[i];
    hash += (hash << 10);
    hash ^= (hash >> 6);
  }

  hash += (hash << 3);
  hash ^= (hash >> 11);
  hash += (hash << 15);

  return hash;
}

static inline void *default_key_copy(void *target) {
  return string_malloc(target);
}

static inline void *default_value_copy(void *target) {
  return string_malloc(target);
}

hashmap_t *hashmap_new() {
  hashmap_t *map = MALLOC(hashmap_t, 1);
  if (map == NULL) {
    return NULL;
  }

  // Create bucket
  map->buckets = darray_new(sizeof(darray_t *), DEFAULT_NUMBER_OF_BUCKETS);
  map->buckets->end = map->buckets->max; // fake out expanding it
  if (map->buckets == NULL) {
    free(map);
    return NULL;
  }

  // Set comparator and hash functions
  map->cmp = default_cmp;
  map->hash = default_hash;

  // Set key and value copy functions
  map->copy_kv = 1;
  map->k_copy = default_key_copy;
  map->v_copy = default_value_copy;
  map->k_free = free;
  map->v_free = free;

  return map;
}

static void free_bucket(darray_t *bucket) {
  assert(bucket != NULL);

  for (int i = 0; i < bucket->end; i++) {
    hashmap_node_t *n = darray_get(bucket, i);
    free(n);
  }

  darray_destroy(bucket);
}

static void clear_free_bucket(hashmap_t *map, darray_t *bucket) {
  assert(map != NULL);
  assert(bucket != NULL);

  // Clear free bucket
  for (int i = 0; i < bucket->end; i++) {
    hashmap_node_t *n = darray_get(bucket, i);
    map->k_free(n->key);
    map->k_free(n->value);
    free(n);
  }

  darray_destroy(bucket);
}

static void free_buckets(hashmap_t *map) {
  assert(map != NULL);

  // Free buckets
  for (int i = 0; i < map->buckets->end; i++) {
    darray_t *bucket = darray_get(map->buckets, i);

    if (bucket) {
      if (map->copy_kv) {
        clear_free_bucket(map, bucket);
      } else {
        free_bucket(bucket);
      }
    }
  }

  darray_destroy(map->buckets);
}

void hashmap_clear_destroy(hashmap_t *map) {
  if (map) {
    if (map->buckets) {
      free_buckets(map);
    }
    free(map);
  }
}

void hashmap_destroy(hashmap_t *map) {
  if (map) {
    if (map->buckets) {
      free_buckets(map);
    }
    free(map);
  }
}

static hashmap_node_t *hashmap_node_new(uint32_t h, void *k, void *v) {
  assert(k != NULL);
  assert(v != NULL);

  // Setup
  hashmap_node_t *node = calloc(1, sizeof(hashmap_node_t));
  if (node == NULL) {
    return NULL;
  }

  // Create hashmap node
  node->key = k;
  node->value = v;
  node->hash = h;

  return node;
}

static darray_t *
hashmap_find_bucket(hashmap_t *map, void *k, int create, uint32_t *hash_out) {
  assert(map != NULL);
  assert(k != NULL);
  assert(hash_out != NULL);

  // Pre-check
  uint32_t hash = map->hash(k);
  int bucket_n = hash % DEFAULT_NUMBER_OF_BUCKETS;
  if ((bucket_n >= 0) == 0) {
    return NULL;
  }
  *hash_out = hash; // Store it for return so caller can use it

  // Find bucket
  darray_t *bucket = darray_get(map->buckets, bucket_n);

  // Coundn't find bucket, create one instead
  if (!bucket && create) {
    // New bucket, set it up
    bucket = darray_new(sizeof(void *), DEFAULT_NUMBER_OF_BUCKETS);
    if (bucket == NULL) {
      return NULL;
    }
    darray_set(map->buckets, bucket_n, bucket);
  }

  return bucket;
}

int hashmap_set(hashmap_t *map, void *k, void *v) {
  assert(map != NULL);
  assert(map->k_copy != NULL);
  assert(map->v_copy != NULL);
  assert(k != NULL);
  assert(v != NULL);

  // Pre-check
  uint32_t hash = 0;
  darray_t *bucket = hashmap_find_bucket(map, k, 1, &hash);
  if (bucket == NULL) {
    return -1;
  }

  // Set hashmap
  hashmap_node_t *node = hashmap_node_new(hash, map->k_copy(k), map->v_copy(v));
  if (node == NULL) {
    return -1;
  }
  darray_push(bucket, node);

  return 0;
}

static inline int
hashmap_get_node(hashmap_t *map, uint32_t hash, darray_t *bucket, void *k) {
  assert(map != NULL);
  assert(bucket != NULL);
  assert(k != NULL);

  for (int i = 0; i < bucket->end; i++) {
    hashmap_node_t *node = darray_get(bucket, i);
    if (node->hash == hash && map->cmp(node->key, k) == 0) {
      return i;
    }
  }

  return -1;
}

void *hashmap_get(hashmap_t *map, void *k) {
  assert(map != NULL);
  assert(k != NULL);

  // Find bucket
  uint32_t hash = 0;
  darray_t *bucket = hashmap_find_bucket(map, k, 0, &hash);
  if (bucket == NULL) {
    return NULL;
  }

  // Find hashmap node
  int i = hashmap_get_node(map, hash, bucket, k);
  if (i == -1) {
    return NULL;
  }

  // Get value
  hashmap_node_t *node = darray_get(bucket, i);
  if (node == NULL) {
    return NULL;
  }

  return node->value;
}

int hashmap_traverse(hashmap_t *map,
                     int (*hashmap_traverse_cb)(hashmap_node_t *)) {
  assert(map != NULL);
  assert(hashmap_traverse_cb != NULL);

  // Traverse
  int rc = 0;
  for (int i = 0; i < map->buckets->end; i++) {
    darray_t *bucket = darray_get(map->buckets, i);

    if (bucket) {
      for (int j = 0; j < bucket->end; j++) {
        hashmap_node_t *node = darray_get(bucket, j);
        rc = hashmap_traverse_cb(node);

        if (rc != 0) {
          return rc;
        }
      }
    }
  }

  return 0;
}

void *hashmap_delete(hashmap_t *map, void *k) {
  assert(map != NULL);
  assert(k != NULL);

  // Find bucket containing hashmap node
  uint32_t hash = 0;
  darray_t *bucket = hashmap_find_bucket(map, k, 0, &hash);
  if (bucket == NULL) {
    return NULL;
  }

  // From bucket get hashmap node and free it
  int i = hashmap_get_node(map, hash, bucket, k);
  if (i == -1) {
    return NULL;
  }

  // Get node
  hashmap_node_t *node = darray_get(bucket, i);
  void *v = node->value;
  if (map->copy_kv) {
    map->k_free(node->key);
  }
  free(node);

  // Check to see if last element in bucket is a node
  hashmap_node_t *ending = darray_pop(bucket);
  if (ending != node) {
    // Alright looks like it's not the last one, swap it
    darray_set(bucket, i, ending);
  }

  return v;
}

#endif // USE_DATA_STRUCTURES

/******************************************************************************
 * TIME
 ******************************************************************************/

/**
 * Tic, start timer.
 * @returns A timespec encapsulating the time instance when tic() is called
 */
struct timespec tic() {
  struct timespec time_start;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  return time_start;
}

/**
 * Toc, stop timer.
 * @returns Time elapsed in seconds
 */
float toc(struct timespec *tic) {
  assert(tic != NULL);
  struct timespec toc;
  float time_elasped;

  clock_gettime(CLOCK_MONOTONIC, &toc);
  time_elasped = (toc.tv_sec - tic->tv_sec);
  time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

  return time_elasped;
}

/**
 * Toc, stop timer.
 * @returns Time elapsed in milli-seconds
 */
float mtoc(struct timespec *tic) {
  assert(tic != NULL);
  return toc(tic) * 1000.0;
}

/**
 * Get time now since epoch.
 * @return Time now in nano-seconds since epoch
 */
timestamp_t time_now() {
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);

  const time_t sec = spec.tv_sec;
  const long int ns = spec.tv_nsec;
  const uint64_t BILLION = 1000000000L;

  return (uint64_t) sec * BILLION + (uint64_t) ns;
}

/**
 * Convert timestamp to seconds
 */
real_t ts2sec(const timestamp_t ts) {
  return ts * 1e-9;
}

/**
 * Convert seconds to timestamp
 */
timestamp_t sec2ts(const real_t time_s) {
  return time_s * 1e9;
}

/******************************************************************************
 * NETWORK
 ******************************************************************************/

/**
 * Return IP and Port info from socket file descriptor `sockfd` to `ip` and
 * `port`. Returns `0` for success and `-1` for failure.
 * @returns
 * - 0 for success
 * - -1 for failure
 */
int ip_port_info(const int sockfd, char *ip, int *port) {
  assert(ip != NULL);
  assert(port != NULL);

  struct sockaddr_storage addr;
  socklen_t len = sizeof addr;
  if (getpeername(sockfd, (struct sockaddr *) &addr, &len) != 0) {
    return -1;
  }

  // Deal with both IPv4 and IPv6:
  char ipstr[INET6_ADDRSTRLEN];

  if (addr.ss_family == AF_INET) {
    // IPV4
    struct sockaddr_in *s = (struct sockaddr_in *) &addr;
    *port = ntohs(s->sin_port);
    inet_ntop(AF_INET, &s->sin_addr, ipstr, sizeof(ipstr));
  } else {
    // IPV6
    struct sockaddr_in6 *s = (struct sockaddr_in6 *) &addr;
    *port = ntohs(s->sin6_port);
    inet_ntop(AF_INET6, &s->sin6_addr, ipstr, sizeof(ipstr));
  }
  memcpy(ip, ipstr, strlen(ipstr));
  ip[strlen(ip)] = '\0';

  return 0;
}

/**
 * Configure TCP server
 */
int tcp_server_setup(tcp_server_t *server, const int port) {
  assert(server != NULL);

  // Setup server struct
  server->port = port;
  server->sockfd = -1;
  server->conn = -1;

  // Create socket
  server->sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (server->sockfd == -1) {
    LOG_ERROR("Socket creation failed...");
    return -1;
  }

  // Socket options
  const int en = 1;
  const size_t int_sz = sizeof(int);
  if (setsockopt(server->sockfd, SOL_SOCKET, SO_REUSEADDR, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEADDR) failed");
  }
  if (setsockopt(server->sockfd, SOL_SOCKET, SO_REUSEPORT, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEPORT) failed");
  }

  // Assign IP, PORT
  struct sockaddr_in addr;
  bzero(&addr, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(server->port);

  // Bind newly created socket to given IP
  int retval = bind(server->sockfd, (struct sockaddr *) &addr, sizeof(addr));
  if (retval != 0) {
    LOG_ERROR("Socket bind failed: %s", strerror(errno));
    return -1;
  }

  return 0;
}

/**
 * Loop TCP server
 * @returns `0` for success, `-1` for failure
 */
int tcp_server_loop(tcp_server_t *server) {
  assert(server != NULL);

  // Server is ready to listen
  if ((listen(server->sockfd, 5)) != 0) {
    LOG_ERROR("Listen failed...");
    return -1;
  }

  // Accept the data packet from client and verification
  DEBUG("Server ready!");
  while (1) {
    // Accept incomming connections
    struct sockaddr_in sockaddr;
    socklen_t len = sizeof(sockaddr);
    int connfd = accept(server->sockfd, (struct sockaddr *) &sockaddr, &len);
    if (connfd < 0) {
      LOG_ERROR("Server acccept failed!");
      return -1;
    } else {
      server->conn = connfd;
      server->conn_handler(&server);
    }
  }
  DEBUG("Server shutting down ...");

  return 0;
}

/**
 * Configure TCP client
 */
int tcp_client_setup(tcp_client_t *client,
                     const char *server_ip,
                     const int server_port) {
  assert(client != NULL);
  assert(server_ip != NULL);

  // Setup client struct
  string_copy(client->server_ip, server_ip);
  client->server_port = server_port;
  client->sockfd = -1;

  // Create socket
  client->sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (client->sockfd == -1) {
    LOG_ERROR("Socket creation failed!");
    return -1;
  }

  // Assign IP, PORT
  struct sockaddr_in server;
  size_t server_size = sizeof(server);
  bzero(&server, server_size);
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(client->server_ip);
  server.sin_port = htons(client->server_port);

  // Connect to server
  if (connect(client->sockfd, (struct sockaddr *) &server, server_size) != 0) {
    LOG_ERROR("Failed to connect to server!");
    return -1;
  }
  DEBUG("Connected to the server!");

  return 0;
}

/**
 * Loop TCP client
 */
int tcp_client_loop(tcp_client_t *client) {
  while (1) {
    if (client->loop_cb) {
      int retval = client->loop_cb(client);
      switch (retval) {
        case -1:
          return -1;
        case 1:
          break;
      }
    }
  }

  return 0;
}

// HTTP //////////////////////////////////////////////////////////////////////

static char b64_encode_table[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
                                  'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
                                  'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                                  'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
                                  'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
                                  'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
                                  'w', 'x', 'y', 'z', '0', '1', '2', '3',
                                  '4', '5', '6', '7', '8', '9', '+', '/'};

char *base64_encode(const uint8_t *data, size_t in_len, size_t *out_len) {
  int mod_table[3] = {0, 2, 1};
  unsigned long i;
  unsigned long j;
  uint32_t octet_a;
  uint32_t octet_b;
  uint32_t octet_c;
  uint32_t triple;
  char *encoded_data;

  *out_len = (4 * ((in_len + 2) / 3));       // length of the encoding string
  encoded_data = CALLOC(char, *out_len + 1); // +1 for the null char

  if (encoded_data == NULL) {
    return NULL;
  }

  for (i = 0, j = 0; i < in_len;) {
    octet_a = i < in_len ? (uint8_t) data[i++] : 0;
    octet_b = i < in_len ? (uint8_t) data[i++] : 0;
    octet_c = i < in_len ? (uint8_t) data[i++] : 0;
    triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

    encoded_data[j++] = b64_encode_table[(triple >> 3 * 6) & 0x3F];
    encoded_data[j++] = b64_encode_table[(triple >> 2 * 6) & 0x3F];
    encoded_data[j++] = b64_encode_table[(triple >> 1 * 6) & 0x3F];
    encoded_data[j++] = b64_encode_table[(triple >> 0 * 6) & 0x3F];
  }

  for (i = 0; i < (unsigned long) mod_table[in_len % 3]; i++) {
    encoded_data[*out_len - 1 - i] = '=';
  }

  return encoded_data;
}

uint8_t *base64_decode(const char *data, size_t in_len, size_t *out_len) {
  unsigned long i;
  unsigned long j;
  uint32_t sextet_a;
  uint32_t sextet_b;
  uint32_t sextet_c;
  uint32_t sextet_d;
  uint32_t triple;
  uint8_t *decoded_data;

  char *decode_table = MALLOC(char, 256);
  for (int i = 0; i < 64; i++) {
    decode_table[(uint8_t) b64_encode_table[i]] = (char) i;
  }

  if (in_len % 4 != 0) {
    return NULL;
  }

  *out_len = in_len / 4 * 3;
  if (data[in_len - 1] == '=') {
    (*out_len)--;
  }

  if (data[in_len - 2] == '=') {
    (*out_len)--;
  }

  decoded_data = CALLOC(uint8_t, *out_len + 1);
  if (decoded_data == NULL) {
    return NULL;
  }

  for (i = 0, j = 0; i < in_len;) {
    sextet_a =
        (data[i] == '=') ? 0 & i++ : (uint32_t) decode_table[(int) data[i++]];
    sextet_b =
        (data[i] == '=') ? 0 & i++ : (uint32_t) decode_table[(int) data[i++]];
    sextet_c =
        (data[i] == '=') ? 0 & i++ : (uint32_t) decode_table[(int) data[i++]];
    sextet_d =
        (data[i] == '=') ? 0 & i++ : (uint32_t) decode_table[(int) data[i++]];

    triple = ((sextet_a << 3 * 6) + (sextet_b << 2 * 6) + (sextet_c << 1 * 6) +
              (sextet_d << 0 * 6));

    if (j < *out_len) {
      decoded_data[j++] = (triple >> 2 * 8) & 0xFF;
    }
    if (j < *out_len) {
      decoded_data[j++] = (triple >> 1 * 8) & 0xFF;
    }
    if (j < *out_len) {
      decoded_data[j++] = (triple >> 0 * 8) & 0xFF;
    }
  }

  free(decode_table);
  return decoded_data;
}

void http_msg_setup(http_msg_t *msg) {
  // Protocol
  msg->protocol = NULL;

  // Request
  msg->method = NULL;
  msg->path = NULL;

  // Response
  msg->status = NULL;

  // Headers
  msg->user_agent = NULL;
  msg->host = NULL;
  msg->upgrade = NULL;
  msg->connection = NULL;
  msg->sec_websocket_key = NULL;
  msg->sec_websocket_version = NULL;
}

void http_msg_free(http_msg_t *msg) {
  // Protocol
  FREE_MEM(msg->protocol, free);

  // Request
  FREE_MEM(msg->method, free);
  FREE_MEM(msg->path, free);

  // Response
  FREE_MEM(msg->status, free);

  // Headers
  FREE_MEM(msg->user_agent, free);
  FREE_MEM(msg->host, free);
  FREE_MEM(msg->upgrade, free);
  FREE_MEM(msg->connection, free);
  FREE_MEM(msg->sec_websocket_key, free);
  FREE_MEM(msg->sec_websocket_version, free);
}

void http_msg_print(http_msg_t *msg) {
  if (msg->method) {
    printf("%s %s %s\r\n", msg->method, msg->path, msg->protocol);
  }

  if (msg->status) {
    printf("%s %s\r\n", msg->status, msg->protocol);
  }

  if (msg->user_agent) {
    printf("User-Agent: %s\r\n", msg->user_agent);
  }

  if (msg->host) {
    printf("Host: %s\r\n", msg->host);
  }

  if (msg->upgrade) {
    printf("Upgrade: %s\r\n", msg->upgrade);
  }

  if (msg->connection) {
    printf("Connection: %s\r\n", msg->connection);
  }

  if (msg->sec_websocket_key) {
    printf("Sec-WebSocket-Key: %s\r\n", msg->sec_websocket_key);
  }

  if (msg->sec_websocket_version) {
    printf("Sec-WebSocket-Version: %s\r\n", msg->sec_websocket_version);
  }
}

int http_parse_request(char *msg_str, http_msg_t *msg) {
  int line_idx = 0;
  char line[1024] = {0};
  char *line_end = NULL;
  char *line_tok = __strtok_r(msg_str, "\r\n", &line_end);

  while (line_tok != NULL) {
    string_copy(line, line_tok);

    if (line_idx == 0) {
      // Parse request line
      // Method
      char *tok_end = NULL;
      char *tok = __strtok_r(line, " ", &tok_end);
      msg->method = string_malloc(tok);

      // Path
      tok = __strtok_r(NULL, " ", &tok_end);
      msg->path = string_malloc(tok);

      // Protocol
      tok = __strtok_r(NULL, " ", &tok_end);
      msg->protocol = string_malloc(tok);

    } else {
      // Parse headers
      char *tok_end = NULL;
      char *tok = __strtok_r(line, " ", &tok_end);

      if (strcmp(tok, "User-Agent:") == 0) {
        msg->user_agent = string_malloc(tok_end);
      } else if (strcmp(tok, "Host:") == 0) {
        msg->host = string_malloc(tok_end);
      } else if (strcmp(tok, "Upgrade:") == 0) {
        msg->upgrade = string_malloc(tok_end);
      } else if (strcmp(tok, "Connection:") == 0) {
        msg->connection = string_malloc(tok_end);
      } else if (strcmp(tok, "Sec-WebSocket-Key:") == 0) {
        msg->sec_websocket_key = string_malloc(tok_end);
      } else if (strcmp(tok, "Sec-WebSocket-Version:") == 0) {
        msg->sec_websocket_version = string_malloc(tok_end);
      }
    }

    line_tok = __strtok_r(NULL, "\r\n", &line_end);
    line_idx++;
  }

  return 0;
}

int http_request_websocket_handshake(const http_msg_t *msg) {
  const int upgrade_ok = strcmp(msg->upgrade, "websocket") == 0;
  const int connection_ok = strcmp(msg->connection, "Upgrade") == 0;
  if (upgrade_ok && connection_ok) {
    return 1;
  }
  return 0;
}

ws_frame_t *ws_frame_malloc() {
  ws_frame_t *frame;

  frame = MALLOC(ws_frame_t, 1);
  frame->header = 0x0;
  frame->mask[0] = 0x0;
  frame->mask[1] = 0x0;
  frame->mask[2] = 0x0;
  frame->mask[3] = 0x0;
  frame->payload_size = 0x0;
  frame->payload_data = NULL;

  return frame;
}

void ws_frame_free(ws_frame_t *frame) {
  // FREE_MEM(frame->payload_data, free);
  FREE_MEM(frame, free);
}

void ws_frame_print(ws_frame_t *frame) {
  printf("ws frame [size: %ld, type: 0x%x]: ",
         frame->payload_size,
         frame->header);
  for (int i = 0; i < (int) frame->payload_size; i++) {
    printf("%c", ((char *) frame->payload_data)[i]);
  }
  printf("\n");
}

uint8_t *ws_frame_serialize(ws_frame_t *frame) {
  // Setup
  uint8_t header[10];
  bzero(header, 10);
  header[0] = frame->header;

  // Payload details
  size_t header_size = 0;
  size_t payload_size = frame->payload_size;

  if (payload_size <= 126) {
    header[1] = (uint8_t)(payload_size & 0x00000000000000FFU);
    header_size = 2;

  } else if (payload_size >= 126 && payload_size <= 65535) {
    header[1] = WS_MASK_OFF | 0x7E;
    header[2] = (payload_size >> 8) & 0xFF;
    header[3] = payload_size & 0xFF;
    header_size = 4;

  } else {
    header[1] = WS_MASK_OFF | 0x7F;
    header[2] = (payload_size >> 56) & 0xFF;
    header[3] = (payload_size >> 48) & 0xFF;
    header[4] = (payload_size >> 40) & 0xFF;
    header[5] = (payload_size >> 32) & 0xFF;
    header[6] = (payload_size >> 24) & 0xFF;
    header[7] = (payload_size >> 16) & 0xFF;
    header[8] = (payload_size >> 8) & 0xFF;
    header[9] = payload_size & 0xFF;
    header_size = 10;
  }

  // Serialize ws frame
  size_t frame_size = header_size + payload_size;
  uint8_t *frame_bytes = CALLOC(uint8_t, frame_size);
  memcpy(frame_bytes, header, header_size);
  memcpy(frame_bytes + header_size, frame->payload_data, payload_size);
  return frame_bytes;
}

int ws_frame_fin_bit(uint8_t *data_frame) {
  return data_frame[0] >> 7;
}

int ws_frame_rsv_bit(uint8_t *data_frame) {
  return (data_frame[0] ^ 0x80) >> 4;
}

int ws_frame_op_code(uint8_t *data_frame) {
  return data_frame[0] & 0x0F;
}

int ws_frame_mask_enabled(uint8_t *data_frame) {
  return data_frame[1] >> 7;
}

ws_frame_t *ws_frame_parse(int connfd) {
  // Parse header
  uint8_t header[2] = {0};
  int retval = (int) recv(connfd, header, 2, 0);
  if (retval != 0) {
    return NULL;
  }
  ws_frame_t *ws_frame = ws_frame_malloc();
  ws_frame->header = header[0];
  ws_frame->payload_size = header[1] & 0x7F;

  // Additional payload size
  if (ws_frame->payload_size == 126) {
    // Obtain extended data size - 2 bytes
    uint8_t buf_2bytes[2] = {0};
    retval = (int) recv(connfd, buf_2bytes, 2, 0);
    if (retval != 0) {
      return NULL;
    }

    // Parse payload size
    ws_frame->payload_size = (((unsigned long long) buf_2bytes[0] << 8) |
                              ((unsigned long long) buf_2bytes[1]));

  } else if (ws_frame->payload_size == 127) {
    // Obtain extended data size - 8 bytes
    uint8_t buf_8bytes[8] = {0};
    retval = (int) recv(connfd, buf_8bytes, 8, 0);
    if (retval != 0) {
      return NULL;
    }

    // Parse payload size
    ws_frame->payload_size =
        ((((unsigned long long) buf_8bytes[0] << 56) & 0xFF00000000000000U) |
         (((unsigned long long) buf_8bytes[1] << 48) & 0x00FF000000000000U) |
         (((unsigned long long) buf_8bytes[2] << 40) & 0x0000FF0000000000U) |
         (((unsigned long long) buf_8bytes[3] << 32) & 0x000000FF00000000U) |
         (((unsigned long long) buf_8bytes[4] << 24) & 0x00000000FF000000U) |
         (((unsigned long long) buf_8bytes[5] << 16) & 0x0000000000FF0000U) |
         (((unsigned long long) buf_8bytes[6] << 8) & 0x000000000000FF00U) |
         (((unsigned long long) buf_8bytes[7]) & 0x00000000000000FFU));
  }

  // Recv mask
  uint8_t mask[4] = {0};
  if (ws_frame_mask_enabled(header)) {
    retval = (int) recv(connfd, mask, 4, 0);
    if (retval != 0) {
      return NULL;
    }
  }

  // Recv payload
  if (ws_frame->payload_size) {
    uint8_t *payload_data = CALLOC(uint8_t, ws_frame->payload_size);
    retval = (int) recv(connfd, payload_data, ws_frame->payload_size, 0);
    if (retval != 0) {
      return NULL;
    }

    // Decode payload data with mask
    if (ws_frame_mask_enabled(header)) {
      for (size_t i = 0; i < ws_frame->payload_size; i++) {
        payload_data[i] = payload_data[i] ^ mask[i % 4];
      }
    }
    ws_frame->payload_data = payload_data;
  }

  return ws_frame;
}

char *ws_recv(int connfd) {
  ws_frame_t *frame = ws_frame_parse(connfd);
  if (frame->header != WS_TEXT || frame->header != (WS_FIN | WS_TEXT)) {
    ws_frame_free(frame);
    return NULL;
  } else {
    return (char *) frame->payload_data;
  }
}

void ws_send(int connfd, const uint8_t *msg) {
  // Setup
  ws_frame_t *frame = ws_frame_malloc();
  frame->header = WS_FIN | WS_TEXT;
  frame->payload_size = strlen((char *) msg);
  frame->payload_data = MALLOC(uint8_t, strlen((const char *) msg));
  memcpy(frame->payload_data, (const char *) msg, strlen((const char *) msg));

  // Write
  uint8_t *frame_bytes = ws_frame_serialize(frame);
  const size_t wrote = write(connfd, frame_bytes, frame->payload_size + 2);
  UNUSED(wrote);

  // Clean up
  free(frame_bytes);
  free(frame->payload_data);
  ws_frame_free(frame);
}

char *ws_read(ws_frame_t *ws_frame) {
  int i;
  char *message;

  message = CALLOC(char, ws_frame->payload_size + 1);
  for (i = 0; i < (int) ws_frame->payload_size; i++) {
    message[i] = ((char *) ws_frame->payload_data)[i];
  }

  return message;
}

char *ws_hash(const char *ws_key) {
  // Concatenate websocket key and guid
  const char *WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  char key[1024] = {0};
  string_copy(key, ws_key);
  string_cat(key, WS_GUID);

  // Perform SHA1 hash on key
  uint8_t hash[SHA_DIGEST_LENGTH];
  memset(hash, '\0', SHA_DIGEST_LENGTH);
  SHA1((const uint8_t *) key, strlen(key), hash);

  // Encode SHA1 hash with base64 encoding
  size_t length = 0;
  return base64_encode(hash, SHA_DIGEST_LENGTH, &length);
}

int ws_handshake(const int connfd) {
  // Get incoming websocket handshake request
  char buf[9046] = {0};
  recv(connfd, buf, 9046, 0);

  // Parse HTTP Request
  http_msg_t req;
  http_msg_setup(&req);
  http_parse_request(buf, &req);
  if (req.sec_websocket_key == NULL) {
    http_msg_free(&req);
    return -1;
  }

  // Get WebSocket Key
  char ws_key[128] = {0};
  string_copy(ws_key, req.sec_websocket_key);
  http_msg_free(&req);

  // Respond websocket handshake and establish connection
  char *hash = ws_hash(ws_key);
  char resp[1024] = {0};
  snprintf(resp, sizeof(resp), WEBSOCKET_HANDSHAKE_RESPONSE, hash);
  const size_t wrote = write(connfd, resp, strlen(resp));
  UNUSED(wrote);
  free(hash);

  return 0;
}

int ws_server() {
  // Setup server
  tcp_server_t server;
  const int port = 5000;
  if (tcp_server_setup(&server, port) != 0) {
    return -1;
  }

  // Server is ready to listen
  if ((listen(server.sockfd, 5)) != 0) {
    LOG_ERROR("Listen failed...");
    return -1;
  }

  // Accept incomming connections
  struct sockaddr_in sockaddr;
  socklen_t len = sizeof(sockaddr);
  int connfd = accept(server.sockfd, (struct sockaddr *) &sockaddr, &len);
  if (connfd < 0) {
    LOG_ERROR("Server acccept failed!");
    return -1;
  }

  // Perform websocket handshake
  ws_handshake(connfd);

  while (1) {
    const char *msg = "Hello World!";
    ws_send(connfd, (const uint8_t *) msg);
  }

  return 0;
}

/******************************************************************************
 *                                 MATHS
 ******************************************************************************/

/**
 * Generate random number between a and b from a uniform distribution.
 * @returns Random number
 */
float randf(const float a, const float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

/**
 * Degrees to radians.
 * @returns Radians
 */
real_t deg2rad(const real_t d) {
  return d * (M_PI / 180.0);
}

/**
 * Radians to degrees.
 * @returns Degrees
 */
real_t rad2deg(const real_t r) {
  return r * (180.0 / M_PI);
}

/**
 * Compare ints.
 * @returns
 * - 0 if v1 == v2
 * - 1 if v1 > v2
 * - -1 if v1 < v2
 */
int intcmp(const int x, int y) {
  if (x > y) {
    return 1;
  } else if (x < y) {
    return -1;
  }
  return 0;
}

/**
 Compare ints.
 * @returns
 * - 0 if v1 == v2
 * - 1 if v1 > v2
 * - -1 if v1 < v2
 */
int intcmp2(const void *x, const void *y) {
  return intcmp(*(int *) x, *(int *) y);
}

/**
 * Compare reals.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int fltcmp(const real_t x, const real_t y) {
  if (fabs(x - y) < CMP_TOL) {
    return 0;
  } else if (x > y) {
    return 1;
  }

  return -1;
}

/**
 * Compare reals.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int fltcmp2(const void *x, const void *y) {
  assert(x != NULL);
  assert(y != NULL);
  return fltcmp(*(real_t *) x, *(real_t *) y);
}

/**
 * Compare strings.
 */
int strcmp2(const void *x, const void *y) {
  return strcmp((char *) x, (char *) y);
}

/**
 * Pythagoras
 *
 *   c = sqrt(a^2 + b^2)
 *
 * @returns Hypotenuse of a and b
 */
real_t pythag(const real_t a, const real_t b) {
  real_t at = fabs(a);
  real_t bt = fabs(b);
  real_t ct = 0.0;
  real_t result = 0.0;

  if (at > bt) {
    ct = bt / at;
    result = at * sqrt(1.0 + ct * ct);
  } else if (bt > 0.0) {
    ct = at / bt;
    result = bt * sqrt(1.0 + ct * ct);
  } else {
    result = 0.0;
  }

  return result;
}

/**
 * Perform 1D Linear interpolation between `a` and `b` with `t` as the
 * interpolation hyper-parameter.
 * @returns Linear interpolated value between a and b
 */
real_t lerp(const real_t a, const real_t b, const real_t t) {
  return a * (1.0 - t) + b * t;
}

/**
 * Perform 3D Linear interpolation between `a` and `b` with `t` as the
 * interpolation hyper-parameter.
 */
void lerp3(const real_t a[3], const real_t b[3], const real_t t, real_t x[3]) {
  assert(a != NULL);
  assert(b != NULL);
  assert(x != NULL);

  x[0] = lerp(a[0], b[0], t);
  x[1] = lerp(a[1], b[1], t);
  x[2] = lerp(a[2], b[2], t);
}

/**
 * Sinc.
 * @return Result of sinc
 */
real_t sinc(const real_t x) {
  if (fabs(x) > 1e-6) {
    return sin(x) / x;
  } else {
    const real_t c2 = 1.0 / 6.0;
    const real_t c4 = 1.0 / 120.0;
    const real_t c6 = 1.0 / 5040.0;
    const real_t x2 = x * x;
    const real_t x4 = x2 * x2;
    const real_t x6 = x2 * x2 * x2;
    return 1.0 - c2 * x2 + c4 * x4 - c6 * x6;
  }
}

/**
 * Calculate mean from vector `x` of length `n`.
 * @returns Mean of x
 */
real_t mean(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  real_t sum = 0.0;
  for (size_t i = 0; i < n; i++) {
    sum += x[i];
  }
  return sum / n;
}

/**
 * Calculate median from vector `x` of length `n`.
 * @returns Median of x
 */
real_t median(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  // Make a copy of the original input vector x
  real_t *vals = MALLOC(real_t, n);
  for (size_t i = 0; i < n; i++) {
    vals[i] = x[i];
  }

  // Sort the values
  qsort(vals, n, sizeof(real_t), fltcmp2);

  // Get median value
  real_t median_value = 0.0;
  if ((n % 2) == 0) {
    const int bwd_idx = (int) (n - 1) / 2.0;
    const int fwd_idx = (int) (n + 1) / 2.0;
    median_value = (vals[bwd_idx] + vals[fwd_idx]) / 2.0;
  } else {
    const int midpoint_idx = n / 2.0;
    median_value = vals[midpoint_idx];
  }

  // Clean up
  free(vals);

  return median_value;
}

/**
 * Calculate variance from vector `x` of length `n`.
 * @returns Variance of x
 */
real_t var(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  const real_t mu = mean(x, n);
  real_t sse = 0.0;
  for (size_t i = 0; i < n; i++) {
    sse += (x[i] - mu) * (x[i] - mu);
  }

  return sse / (n - 1);
}

/**
 * Calculate standard deviation from vector `x` of length `n`.
 * @returns Standard deviation of x
 */
real_t stddev(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);
  return sqrt(var(x, n));
}

/******************************************************************************
 * LINEAR ALGEBRA
 ******************************************************************************/

/**
 * Print matrix `A` of size `m x n`.
 */
void print_matrix(const char *prefix,
                  const real_t *A,
                  const size_t m,
                  const size_t n) {
  assert(prefix != NULL);
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0;
  printf("%s:\n", prefix);
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      printf("%f  ", A[idx]);
      idx++;
    }
    printf("\n");
  }
  printf("\n");
}

/**
 * Print vector `v` of length `n`.
 */
void print_vector(const char *prefix, const real_t *v, const size_t n) {
  assert(prefix != NULL);
  assert(v != NULL);
  assert(n != 0);

  size_t idx = 0;
  printf("%s: ", prefix);
  for (size_t i = 0; i < n; i++) {
    // printf("%.4f ", v[idx]);
    printf("%.10f ", v[idx]);
    idx++;
  }
  printf("\n");
}

/**
 * Convert vector string
 */
void vec2str(const real_t *v, const int n, char *s) {
  s[0] = '[';
  for (int i = 0; i < n; i++) {
    sprintf(s + strlen(s), "%f", v[i]);
    if (i < (n - 1)) {
      strcat(s + strlen(s), ", ");
    }
  }
  strcat(s + strlen(s), "]");
}

/**
 * Form identity matrix `A` of size `m x n`.
 */
void eye(real_t *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = (i == j) ? 1.0 : 0.0;
      idx++;
    }
  }
}

/**
 * Form ones matrix `A` of size `m x n`.
 */
void ones(real_t *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = 1.0;
      idx++;
    }
  }
}

/**
 * Form zeros matrix `A` of size `m x n`.
 */
void zeros(real_t *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = 0.0;
      idx++;
    }
  }
}

/**
 * Malloc matrix of size `m x n`.
 */
real_t *mat_malloc(const size_t m, const size_t n) {
  assert(m > 0);
  assert(n > 0);
  return CALLOC(real_t, m * n);
}

/**
 * Compare two matrices `A` and `B` of size `m x n`.
 *
 * @returns
 * - 0 if A == B
 * - 1 if A > B
 * - -1 if A < B
 */
int mat_cmp(const real_t *A, const real_t *B, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(B != NULL);
  assert(m > 0);
  assert(n > 0);

  size_t index = 0;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      int retval = fltcmp(A[index], B[index]);
      if (retval != 0) {
        printf("Failed at index[%zu]\n", index);
        return retval;
      }
      index++;
    }
  }

  return 0;
}

/**
 * Check to see if two matrices `A` and `B` of size `m x n` are equal to a
 * tolerance.
 * @returns 0 if A == B or -1 if A != B
 */
int mat_equals(const real_t *A,
               const real_t *B,
               const size_t m,
               const size_t n,
               const real_t tol) {
  assert(A != NULL);
  assert(B != NULL);
  assert(m > 0);
  assert(n > 0);
  assert(tol > 0);

  size_t index = 0;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      if (fabs(A[index] - B[index]) > tol) {
        printf("Failed at index[%zu]\n", index);
        return -1;
      }
      index++;
    }
  }

  return 0;
}

/**
 * Save matrix `A` of size `m x n` to `save_path`.
 * @returns `0` for success, `-1` for failure
 */
int mat_save(const char *save_path, const real_t *A, const int m, const int n) {
  assert(save_path != NULL);
  assert(A != NULL);
  assert(m > 0);
  assert(n > 0);

  FILE *csv_file = fopen(save_path, "w");
  if (csv_file == NULL) {
    return -1;
  }

  int idx = 0;
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      fprintf(csv_file, "%e", A[idx]);
      idx++;
      if ((j + 1) != n) {
        fprintf(csv_file, ",");
      }
    }
    fprintf(csv_file, "\n");
  }
  fclose(csv_file);

  return 0;
}

/**
 * Load matrix from file in `mat_path`, on success `nb_rows` and `nb_cols` will
 * be set respectively.
 */
real_t *mat_load(const char *mat_path, int *nb_rows, int *nb_cols) {
  assert(mat_path != NULL);
  assert(nb_rows != NULL);
  assert(nb_cols != NULL);

  // Obtain number of rows and columns in csv data
  *nb_rows = dsv_rows(mat_path);
  *nb_cols = dsv_cols(mat_path, ',');
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  // Initialize memory for csv data
  real_t *A = MALLOC(real_t, *nb_rows * *nb_cols);

  // Load file
  FILE *infile = fopen(mat_path, "r");
  if (infile == NULL) {
    fclose(infile);
    free(A);
    return NULL;
  }

  // Loop through data
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;
  int idx = 0;

  // Loop through data line by line
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    // Ignore if comment line
    if (line[0] == '#') {
      continue;
    }

    // Iterate through values in line separated by commas
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        A[idx] = strtod(entry, NULL);
        idx++;

        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  // Clean up
  fclose(infile);

  return A;
}

/**
 * Set matrix `A` with value `val` at `(i, j)`.
 */
void mat_set(real_t *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const real_t val) {
  assert(A != NULL);
  assert(stride != 0);
  A[(i * stride) + j] = val;
}

/**
 * Get value from matrix `A` with `stride` at `(i, j)`.
 */
real_t
mat_val(const real_t *A, const size_t stride, const size_t i, const size_t j) {
  assert(A != NULL);
  assert(stride != 0);
  return A[(i * stride) + j];
}

/**
 * Copy matrix `src` of size `m x n` to `dest`.
 */
void mat_copy(const real_t *src, const int m, const int n, real_t *dest) {
  assert(src != NULL);
  assert(m > 0);
  assert(n > 0);
  assert(dest != NULL);

  for (int i = 0; i < (m * n); i++) {
    dest[i] = src[i];
  }
}

/**
 * Set matrix row.
 */
void mat_row_set(real_t *A,
                 const size_t stride,
                 const int row_idx,
                 const real_t *x) {
  int vec_idx = 0;
  for (size_t i = 0; i < stride; i++) {
    A[(stride * row_idx) + i] = x[vec_idx++];
  }
}

/**
 * Set matrix column.
 */
void mat_col_set(real_t *A,
                 const size_t stride,
                 const int nb_rows,
                 const int col_idx,
                 const real_t *x) {
  int vec_idx = 0;
  for (int i = 0; i < nb_rows; i++) {
    A[i * stride + col_idx] = x[vec_idx++];
  }
}

/**
 * Get matrix sub-block from `A` with `stride` from row and column start `rs`
 * and `cs`, to row and column end `re` and `ce`. The sub-block is written to
 * `block`.
 */
void mat_block_get(const real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   real_t *block) {
  assert(A != NULL);
  assert(block != NULL);
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      // block[idx] = mat_val(A, stride, i, j);
      block[idx] = A[(i * stride) + j];
      idx++;
    }
  }
}

/**
 * Set matrix sub-block `block` to `A` with `stride` from row and column start
 * `rs` and `cs`, to row and column end `re` and `ce`.
 */
void mat_block_set(real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t re,
                   const size_t cs,
                   const size_t ce,
                   const real_t *block) {
  assert(A != NULL);
  assert(block != NULL);
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      // mat_set(A, stride, i, j, block[idx]);
      A[(i * stride) + j] = block[idx];
      idx++;
    }
  }
}

/**
 * Get diagonal vector `d` from matrix `A` of size `m x n`.
 */
void mat_diag_get(const real_t *A, const int m, const int n, real_t *d) {
  int mat_index = 0;
  int vec_index = 0;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) {
        d[vec_index] = A[mat_index];
        vec_index++;
      }
      mat_index++;
    }
  }
}

/**
 * Set the diagonal of matrix `A` of size `m x n` with vector `d`.
 */
void mat_diag_set(real_t *A, const int m, const int n, const real_t *d) {
  assert(A != NULL);
  assert(m > 0);
  assert(n > 0);
  assert(d != NULL);

  int mat_index = 0;
  int vec_index = 0;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) {
        A[mat_index] = d[vec_index];
        vec_index++;
      } else {
        A[mat_index] = 0.0;
      }
      mat_index++;
    }
  }
}

/**
 * Get upper triangular square matrix of `A` of size `m x m`, results are
 * outputted to `U`.
 */
void mat_triu(const real_t *A, const size_t m, real_t *U) {
  assert(A != NULL);
  assert(m > 0);
  assert(U != NULL);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < m; j++) {
      U[i * m + j] = (j >= i) ? A[i * m + j] : 0.0;
    }
  }
}

/**
 * Get lower triangular square matrix of `A` of size `m x m`, results are
 * outputted to `L`.
 */
void mat_tril(const real_t *A, const size_t m, real_t *L) {
  assert(A != NULL);
  assert(m > 0);
  assert(L != NULL);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < m; j++) {
      L[i * m + j] = (j <= i) ? A[i * m + j] : 0.0;
    }
  }
}

/**
 * Get the trace matrix of `A` of size `m x n`.
 */
real_t mat_trace(const real_t *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m > 0);
  assert(n > 0);

  real_t tr = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      tr += (i == j) ? A[i * n + j] : 0.0;
    }
  }
  return tr;
}

/**
 * Transpose of matrix `A` of size `m x n`, results are outputted to `A_t`.
 */
void mat_transpose(const real_t *A, size_t m, size_t n, real_t *A_t) {
  assert(A != NULL && A != A_t);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(A_t, m, j, i, mat_val(A, n, i, j));
    }
  }
}

/**
 * Add two matrices `A` and `B` of size `m x n`, results are outputted to `C`.
 */
void mat_add(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n) {
  assert(A != NULL && B != NULL && C != NULL);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < (m * n); i++) {
    C[i] = A[i] + B[i];
  }
}

/**
 * Subtract two matrices `A` and `B` of size `m x n`, results are outputted to
 * matrix `C`.
 */
void mat_sub(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n) {
  assert(A != NULL && B != NULL && C != NULL && B != C && A != C);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < (m * n); i++) {
    C[i] = A[i] - B[i];
  }
}

/**
 * Scale matrix `A` of size `m x n` inplace with `scale`.
 */
void mat_scale(real_t *A, const size_t m, const size_t n, const real_t scale) {
  assert(A != NULL);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < (m * n); i++) {
    A[i] = A[i] * scale;
  }
}

/**
 * Create new vector of length `n` in heap memory.
 * @returns Heap allocated vector
 */
real_t *vec_malloc(const size_t n) {
  assert(n > 0);
  return CALLOC(real_t, n);
}

/**
 * Copy vector `src` of length `n` to `dest`.
 */
void vec_copy(const real_t *src, const size_t n, real_t *dest) {
  assert(src != NULL);
  assert(n > 0);
  assert(dest != NULL);

  for (size_t i = 0; i < n; i++) {
    dest[i] = src[i];
  }
}

/**
 * Check if vectors `x` and `y` of length `n` are equal.
 * @returns
 * - 1 for x == y
 * - 0 for x != y
 */
int vec_equals(const real_t *x, const real_t *y, const size_t n) {
  assert(x != NULL);
  assert(y != NULL);
  assert(n > 0);

  for (size_t i = 0; i < n; i++) {
    if (fltcmp(x[i], y[i]) != 0) {
      return 0;
    }
  }

  return 1;
}

/**
 * Sum two vectors `x` and `y` of length `n` to `z`.
 */
void vec_add(const real_t *x, const real_t *y, real_t *z, size_t n) {
  assert(x != NULL && y != NULL && z != NULL && x != y && x != z);
  assert(n > 0);

  for (size_t i = 0; i < n; i++) {
    z[i] = x[i] + y[i];
  }
}

/**
 * Subtract two vectors `x` and `y` of length `n` to `z`.
 */
void vec_sub(const real_t *x, const real_t *y, real_t *z, size_t n) {
  assert(x != NULL && y != NULL && z != NULL && x != y && x != z);
  assert(n > 0);

  for (size_t i = 0; i < n; i++) {
    z[i] = x[i] - y[i];
  }
}

/**
 * Scale a vector `x` of length `n` with `scale` in-place.
 */
void vec_scale(real_t *x, const size_t n, const real_t scale) {
  assert(x != NULL);
  assert(n > 0);

  for (size_t i = 0; i < n; i++) {
    x[i] = x[i] * scale;
  }
}

/**
 * Calculate vector norm of `x` of length `n`.
 * @returns Norm of vector x
 */
real_t vec_norm(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  real_t sum = 0.0;
  for (size_t i = 0; i < n; i++) {
    sum += x[i] * x[i];
  }
  return sqrt(sum);
}

/**
 * Normalize vector `x` of length `n` in place.
 */
void vec_normalize(real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  const real_t norm = vec_norm(x, n);
  for (size_t i = 0; i < n; i++) {
    x[i] = x[i] / norm;
  }
}

/**
 * Dot product of two matrices or vectors `A` and `B` of size `A_m x A_n` and
 * `B_m x B_n`. Results are written to `C`.
 */
void dot(const real_t *A,
         const size_t A_m,
         const size_t A_n,
         const real_t *B,
         const size_t B_m,
         const size_t B_n,
         real_t *C) {
  assert(A != NULL && B != NULL && A != C && B != C);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

#ifdef USE_CBLAS
#if PRECISION == 1
  cblas_sgemm(CblasRowMajor, // Matrix data arrangement
              CblasNoTrans,  // Transpose A
              CblasNoTrans,  // Transpose B
              A_m,           // Number of rows in A and C
              B_n,           // Number of cols in B and C
              A_n,           // Number of cols in A
              1.0,           // Scaling factor for the product of A and B
              A,             // Matrix A
              A_n,           // First dimension of A
              B,             // Matrix B
              B_n,           // First dimension of B
              0.0,           // Scale factor for C
              C,             // Output
              B_n);          // First dimension of C
#elif PRECISION == 2
  cblas_dgemm(CblasRowMajor, // Matrix data arrangement
              CblasNoTrans,  // Transpose A
              CblasNoTrans,  // Transpose B
              A_m,           // Number of rows in A and C
              B_n,           // Number of cols in B and C
              A_n,           // Number of cols in A
              1.0,           // Scaling factor for the product of A and B
              A,             // Matrix A
              A_n,           // First dimension of A
              B,             // Matrix B
              B_n,           // First dimension of B
              0.0,           // Scale factor for C
              C,             // Output
              B_n);          // First dimension of C
#endif
#else
  size_t m = A_m;
  size_t n = B_n;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      for (size_t k = 0; k < A_n; k++) {
        C[(i * n) + j] += A[(i * A_n) + k] * B[(k * B_n) + j];
      }
    }
  }

#endif
}

/**
 * Dot product of two matrices or vectors `A` and `B` of size `A_m x A_n` and
 * `B_m x B_n`. Results are written to `C`.
 */
void dot3(const real_t *A,
          const size_t A_m,
          const size_t A_n,
          const real_t *B,
          const size_t B_m,
          const size_t B_n,
          const real_t *C,
          const size_t C_m,
          const size_t C_n,
          real_t *D) {
  real_t *AB = MALLOC(real_t, A_m * B_n);
  dot(A, A_m, A_n, B, B_m, B_n, AB);
  dot(AB, A_m, B_m, C, C_m, C_n, D);
  free(AB);
}

/**
 * Mulitply Y = X' * A * X
 */
void dot_XtAX(const real_t *X,
              const size_t X_m,
              const size_t X_n,
              const real_t *A,
              const size_t A_m,
              const size_t A_n,
              real_t *Y) {
  assert(X != NULL);
  assert(A != NULL);
  assert(Y != NULL);
  assert(X_m == A_m);

  real_t *XtA = MALLOC(real_t, (X_m * A_m));
  real_t *Xt = MALLOC(real_t, (X_m * X_n));

  mat_transpose(X, X_m, X_n, Xt);
  dot(Xt, X_n, X_m, A, A_m, A_n, XtA);
  dot(XtA, X_m, A_m, Xt, X_n, X_m, Y);

  free(Xt);
  free(XtA);
}

/**
 * Mulitply Y = X * A * X'
 */
void dot_XAXt(const real_t *X,
              const size_t X_m,
              const size_t X_n,
              const real_t *A,
              const size_t A_m,
              const size_t A_n,
              real_t *Y) {
  assert(X != NULL);
  assert(A != NULL);
  assert(Y != NULL);
  assert(X_n == A_m);

  real_t *XA = MALLOC(real_t, (X_m * A_n));
  real_t *Xt = MALLOC(real_t, (X_m * X_n));

  dot(X, X_m, X_n, A, A_m, A_n, XA);
  mat_transpose(X, X_m, X_n, Xt);
  dot(XA, X_n, A_m, Xt, X_n, X_m, Y);

  free(XA);
  free(Xt);
}

/**
 * Create skew-symmetric matrix `A` from a 3x1 vector `x`.
 */
void hat(const real_t x[3], real_t A[3 * 3]) {
  assert(x != NULL);
  assert(A != NULL);

  // First row
  A[0] = 0.0;
  A[1] = -x[2];
  A[2] = x[1];

  // Second row
  A[3] = x[2];
  A[4] = 0.0;
  A[5] = -x[0];

  // Third row
  A[6] = -x[1];
  A[7] = x[0];
  A[8] = 0.0;
}

/**
 * Opposite of the skew-symmetric matrix
 */
void vee(const real_t A[3 * 3], real_t x[3]) {
  assert(A != NULL);
  assert(x != NULL);

  const real_t A02 = A[2];
  const real_t A10 = A[3];
  const real_t A21 = A[7];

  x[0] = A21;
  x[1] = A02;
  x[2] = A10;
}

/**
 * Perform forward substitution with a lower triangular matrix `L`, column
 * vector `b` and solve for vector `y` of size `n`.
 */
void fwdsubs(const real_t *L, const real_t *b, real_t *y, const size_t n) {
  assert(L != NULL);
  assert(b != NULL);
  assert(y != NULL);
  assert(n > 0);

  for (size_t i = 0; i < n; i++) {
    real_t alpha = b[i];
    for (size_t j = 0; j < i; j++) {
      alpha -= L[i * n + j] * y[j];
    }
    y[i] = alpha / L[i * n + i];
  }
}

/**
 * Perform backward substitution with a upper triangular matrix `U`, column
 * vector `y` and solve for vector `x` of size `n`.
 */
void bwdsubs(const real_t *U, const real_t *y, real_t *x, const size_t n) {
  assert(U != NULL);
  assert(y != NULL);
  assert(x != NULL);
  assert(n > 0);

  for (int i = n - 1; i >= 0; i--) {
    real_t alpha = y[i];
    for (int j = i; j < (int) n; j++) {
      alpha -= U[i * n + j] * x[j];
    }
    x[i] = alpha / U[i * n + i];
  }
}

/**
 * Check inverted matrix A by multiplying by its inverse.
 */
int check_inv(const real_t *A, const real_t *A_inv, const int m) {
  real_t *inv_check = MALLOC(real_t, m * m);
  dot(A, m, m, A_inv, m, m, inv_check);

  for (int i = 0; i < m; i++) {
    if (fltcmp(inv_check[i * m + i], 1.0) != 0) {
      free(inv_check);
      return -1;
    }
  }

  free(inv_check);
  return 0;
}

/**
 * Check analytical jacobian `jac` with name `jac_name` and compare it with a
 * numerically differentiated jacobian `fdiff` (finite diff). Where both
 * matrices are of size `m x n`, `tol` denotes the tolerance to consider both
 * `jac` and `fdiff` to be close enough. For debugging purposes use `verbose`
 * to show the matrices and differences.
 *
 * @returns
 * - 0 for success
 * - -1 for failure
 */
int check_jacobian(const char *jac_name,
                   const real_t *fdiff,
                   const real_t *jac,
                   const size_t m,
                   const size_t n,
                   const real_t tol,
                   const int verbose) {
  assert(jac_name != NULL);
  assert(fdiff != NULL);
  assert(jac != NULL);
  assert(m > 0);
  assert(n > 0);
  assert(tol > 0);

  int retval = 0;
  int ok = 1;
  real_t *delta = mat_malloc(m, n);
  mat_sub(fdiff, jac, delta, m, n);

  // Check if any of the values are beyond the tol
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      if (fabs(mat_val(delta, n, i, j)) >= tol) {
        ok = 0;
      }
    }
  }

  // Print result
  if (ok == 0) {
    if (verbose) {
      LOG_ERROR("Bad jacobian [%s]!\n", jac_name);
      print_matrix("analytical jac", jac, m, n);
      print_matrix("num diff jac", fdiff, m, n);
      print_matrix("difference matrix", delta, m, n);
    }
    retval = -1;
  } else {
    if (verbose) {
      printf("Check [%s] ok!\n", jac_name);
      // print_matrix("analytical jac", jac, m, n);
      // print_matrix("num diff jac", fdiff, m, n);
      // print_matrix("difference matrix", delta, m, n);
    }
    retval = 0;
  }

  // Clean up
  free(delta);

  return retval;
}

/******************************************************************************
 * SVD
 ******************************************************************************/

#ifdef USE_LAPACK
/**
 * Decompose matrix A with SVD
 */
int __lapack_svd(
    real_t *A, const int m, const int n, real_t *s, real_t *U, real_t *Vt) {
  const int lda = n;
#if PRECISION == 1
  const int info =
      LAPACKE_sgesdd(LAPACK_ROW_MAJOR, 'S', m, n, A, lda, s, U, n, Vt, n);
  return (info == 0) ? 0 : -1;
#elif PRECISION == 2
  const int info =
      LAPACKE_dgesdd(LAPACK_ROW_MAJOR, 'S', m, n, A, lda, s, U, n, Vt, n);
  return (info == 0) ? 0 : -1;
#endif
}
#endif // USE_LAPACK

/**
 * Singular Value Decomposition
 *
 * Given a matrix A of size m x n, compute the singular value decomposition of
 * A = U * W * Vt, where the input A is replaced by U, the diagonal matrix W is
 * output as a vector w of size n, and the matrix V (not V transpose) is of
 * size n x n.
 *
 * Source (Singular-Value-Decomposition: page 59-70):
 *
 *   Press, William H., et al. "Numerical recipes in C++." The art of
 *   scientific computing 2 (2007): 1002.
 *
 * @returns 0 for success, -1 for failure
 */
int __svd(real_t *A, const int m, const int n, real_t *w, real_t *V) {
  int flag, i, its, j, jj, k, l, nm;
  double anorm, c, f, g, h, s, scale, x, y, z, *rv1;
  l = 0;

  rv1 = MALLOC(double, n);
  if (rv1 == NULL) {
    printf("svd(): Unable to allocate vector\n");
    return (-1);
  }

  g = scale = anorm = 0.0;
  for (i = 0; i < n; i++) {
    l = i + 1;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m) {
      for (k = i; k < m; k++)
        scale += fabs(A[k * n + i]);
      if (scale) {
        for (k = i; k < m; k++) {
          A[k * n + i] /= scale;
          s += A[k * n + i] * A[k * n + i];
        }
        f = A[i * n + i];
        g = -SIGN2(sqrt(s), f);
        h = f * g - s;
        A[i * n + i] = f - g;
        for (j = l; j < n; j++) {
          for (s = 0.0, k = i; k < m; k++)
            s += A[k * n + i] * A[k * n + j];
          f = s / h;
          for (k = i; k < m; k++)
            A[k * n + j] += f * A[k * n + i];
        }
        for (k = i; k < m; k++)
          A[k * n + i] *= scale;
      }
    }
    w[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m && i != n - 1) {
      for (k = l; k < n; k++)
        scale += fabs(A[i * n + k]);
      if (scale) {
        for (k = l; k < n; k++) {
          A[i * n + k] /= scale;
          s += A[i * n + k] * A[i * n + k];
        }
        f = A[i * n + l];
        g = -SIGN2(sqrt(s), f);
        h = f * g - s;
        A[i * n + l] = f - g;
        for (k = l; k < n; k++)
          rv1[k] = A[i * n + k] / h;
        for (j = l; j < m; j++) {
          for (s = 0.0, k = l; k < n; k++)
            s += A[j * n + k] * A[i * n + k];
          for (k = l; k < n; k++)
            A[j * n + k] += s * rv1[k];
        }
        for (k = l; k < n; k++)
          A[i * n + k] *= scale;
      }
    }
    anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
  }

  for (i = n - 1; i >= 0; i--) {
    if (i < n - 1) {
      if (g) {
        for (j = l; j < n; j++)
          V[j * n + i] = (A[i * n + j] / A[i * n + l]) / g;
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < n; k++)
            s += A[i * n + k] * V[k * n + j];
          for (k = l; k < n; k++)
            V[k * n + j] += s * V[k * n + i];
        }
      }
      for (j = l; j < n; j++)
        V[i * n + j] = V[j * n + i] = 0.0;
    }
    V[i * n + i] = 1.0;
    g = rv1[i];
    l = i;
  }

  for (i = MIN(m, n) - 1; i >= 0; i--) {
    l = i + 1;
    g = w[i];
    for (j = l; j < n; j++)
      A[i * n + j] = 0.0;
    if (g) {
      g = 1.0 / g;
      for (j = l; j < n; j++) {
        for (s = 0.0, k = l; k < m; k++)
          s += A[k * n + i] * A[k * n + j];
        f = (s / A[i * n + i]) * g;
        for (k = i; k < m; k++)
          A[k * n + j] += f * A[k * n + i];
      }
      for (j = i; j < m; j++)
        A[j * n + i] *= g;
    } else
      for (j = i; j < m; j++)
        A[j * n + i] = 0.0;
    ++A[i * n + i];
  }

  for (k = n - 1; k >= 0; k--) {
    for (its = 0; its < 30; its++) {
      flag = 1;
      for (l = k; l >= 0; l--) {
        nm = l - 1;
        if ((fabs(rv1[l]) + anorm) == anorm) {
          flag = 0;
          break;
        }
        if ((fabs(w[nm]) + anorm) == anorm)
          break;
      }
      if (flag) {
        c = 0.0;
        s = 1.0;
        for (i = l; i <= k; i++) {
          f = s * rv1[i];
          rv1[i] = c * rv1[i];
          if ((fabs(f) + anorm) == anorm)
            break;
          g = w[i];
          h = pythag(f, g);
          w[i] = h;
          h = 1.0 / h;
          c = g * h;
          s = -f * h;
          for (j = 0; j < m; j++) {
            y = A[j * n + nm];
            z = A[j * n + i];
            A[j * n + nm] = y * c + z * s;
            A[j * n + i] = z * c - y * s;
          }
        }
      }
      z = w[k];
      if (l == k) {
        if (z < 0.0) {
          w[k] = -z;
          for (j = 0; j < n; j++)
            V[j * n + k] = -V[j * n + k];
        }
        break;
      }
      if (its == 29)
        printf("no convergence in 30 svd iterations\n");
      x = w[l];
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = pythag(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + SIGN2(g, f))) - h)) / x;
      c = s = 1.0;
      for (j = l; j <= nm; j++) {
        i = j + 1;
        g = rv1[i];
        y = w[i];
        h = s * g;
        g = c * g;
        z = pythag(f, h);
        rv1[j] = z;
        c = f / z;
        s = h / z;
        f = x * c + g * s;
        g = g * c - x * s;
        h = y * s;
        y *= c;
        for (jj = 0; jj < n; jj++) {
          x = V[jj * n + j];
          z = V[jj * n + i];
          V[jj * n + j] = x * c + z * s;
          V[jj * n + i] = z * c - x * s;
        }
        z = pythag(f, h);
        w[j] = z;
        if (z) {
          z = 1.0 / z;
          c = f * z;
          s = h * z;
        }
        f = c * g + s * y;
        x = c * y - s * g;
        for (jj = 0; jj < m; jj++) {
          y = A[jj * n + j];
          z = A[jj * n + i];
          A[jj * n + j] = y * c + z * s;
          A[jj * n + i] = z * c - y * s;
        }
      }
      rv1[l] = 0.0;
      rv1[k] = f;
      w[k] = x;
    }
  }
  free(rv1);

  return 0;
}

/**
 * Decompose matrix A with SVD
 */
int svd(real_t *A, const int m, const int n, real_t *U, real_t *s, real_t *V) {
#ifdef USE_LAPACK
  real_t *Vt = MALLOC(real_t, n * n);
  const int retval = __lapack_svd(A, m, n, s, U, Vt);
  mat_transpose(Vt, n, n, V);
  free(Vt);
  return retval;
#else
  mat_copy(A, m, n, U);
  return __svd(U, m, n, s, V);
#endif // USE_LAPACK
}

/**
 * Pseudo inverse of matrix A with SVD
 */
void svd_inv(real_t *A, const int m, const int n, real_t *A_inv) {
  assert(m == n);

  // Decompose A = U * S * V_t
  const int diag_size = (m < n) ? m : n;
  real_t *s = MALLOC(real_t, diag_size);
  real_t *U = MALLOC(real_t, m * n);
  real_t *Ut = MALLOC(real_t, m * n);
  real_t *V = MALLOC(real_t, n * n);
  svd(A, m, n, U, s, V);
  mat_transpose(U, m, n, Ut);

  // Form Sinv diagonal matrix
  real_t *S_inv = MALLOC(real_t, m * n);
  zeros(S_inv, n, m);
  for (int idx = 0; idx < m; idx++) {
    const int diag_idx = idx * n + idx;
    if (s[idx] > 1e-8) {
      S_inv[diag_idx] = 1.0 / s[idx];
    } else {
      S_inv[diag_idx] = 0.0;
    }
  }

  // A_inv = Vt * S_inv * U
  real_t *V_S_inv = MALLOC(real_t, m * m);
  dot(V, m, n, S_inv, n, m, V_S_inv);
  dot(V_S_inv, m, m, Ut, m, n, A_inv);

  // Clean up
  free(s);
  free(U);
  free(Ut);
  free(V);
  free(S_inv);
  free(V_S_inv);
}

/******************************************************************************
 * CHOL
 ******************************************************************************/

#ifdef USE_LAPACK
/**
 * Decompose matrix A to lower triangular matrix L
 */
void __lapack_chol(const real_t *A, const size_t m, real_t *L) {
  assert(A != NULL);
  assert(m > 0);
  assert(L != NULL);

  // Cholesky Decomposition
  int info = 0;
  int lda = m;
  int n = m;
  char uplo = 'L';
  mat_copy(A, m, m, L);
#if PRECISION == 1
  spotrf_(&uplo, &n, L, &lda, &info);
#elif PRECISION == 2
  dpotrf_(&uplo, &n, L, &lda, &info);
#endif
  if (info != 0) {
    fprintf(stderr, "Failed to decompose A using Cholesky Decomposition!\n");
  }

  // Transpose and zero upper triangular result
  for (size_t i = 0; i < m; i++) {
    for (size_t j = i; j < m; j++) {
      if (i != j) {
        L[(j * m) + i] = L[(i * m) + j];
        L[(i * m) + j] = 0.0;
      }
    }
  }
}

/**
 * Solve Ax = b using LAPACK's implementation of Cholesky decomposition, where
 * `A` is a square matrix, `b` is a vector and `x` is the solution vector of
 * size `n`.
 */
void __lapack_chol_solve(const real_t *A,
                         const real_t *b,
                         real_t *x,
                         const size_t m) {
  assert(A != NULL);
  assert(b != NULL);
  assert(x != NULL);
  assert(m > 0);

  // Cholesky Decomposition
  int info = 0;
  int lda = m;
  int n = m;
  char uplo = 'L';
  real_t *a = mat_malloc(m, m);
  mat_copy(A, m, m, a);
#if PRECISION == 1
  spotrf_(&uplo, &n, a, &lda, &info);
#elif PRECISION == 2
  dpotrf_(&uplo, &n, a, &lda, &info);
#endif
  if (info != 0) {
    fprintf(stderr, "Failed to decompose A using Cholesky Decomposition!\n");
  }

  // Solve Ax = b using Cholesky decomposed A from above
  vec_copy(b, m, x);
  int nhrs = 1;
  int ldb = m;
#if PRECISION == 1
  spotrs_(&uplo, &n, &nhrs, a, &lda, x, &ldb, &info);
#elif PRECISION == 2
  dpotrs_(&uplo, &n, &nhrs, a, &lda, x, &ldb, &info);
#endif
  if (info != 0) {
    fprintf(stderr, "Failed to solve Ax = b!\n");
  }

  free(a);
}
#endif // USE_LAPACK

/**
 * Cholesky decomposition. Takes a `m x m` matrix `A` and decomposes it into a
 * lower and upper triangular matrix `L` and `U` with Cholesky decomposition.
 * This function only returns the `L` triangular matrix.
 */
void __chol(const real_t *A, const size_t m, real_t *L) {
  assert(A != NULL);
  assert(m > 0);
  assert(L != NULL);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < (i + 1); j++) {

      if (i == j) {
        real_t s = 0.0;
        for (size_t k = 0; k < j; k++) {
          s += L[j * m + k] * L[j * m + k];
        }
        L[i * m + j] = sqrt(A[i * m + i] - s);

      } else {
        real_t s = 0.0;
        for (size_t k = 0; k < j; k++) {
          s += L[i * m + k] * L[j * m + k];
        }
        L[i * m + j] = (1.0 / L[j * m + j] * (A[i * m + j] - s));
      }
    }
  }
}

/**
 * Solve `Ax = b` using Cholesky decomposition, where `A` is a square matrix,
 * `b` is a vector and `x` is the solution vector of size `n`.
 */
void __chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n) {
  assert(A != NULL);
  assert(b != NULL);
  assert(x != NULL);
  assert(n > 0);

  // Allocate memory
  real_t *L = CALLOC(real_t, n * n);
  real_t *Lt = CALLOC(real_t, n * n);
  real_t *y = CALLOC(real_t, n);

  // Cholesky decomposition
  chol(A, n, L);
  mat_transpose(L, n, n, Lt);

  // Forward substitution
  // Ax = b -> LLt x = b.
  // Let y = Lt x, L y = b (Solve for y)
  for (int i = 0; i < (int) n; i++) {
    real_t alpha = b[i];

    if (fltcmp(L[i * n + i], 0.0) == 0) {
      y[i] = 0.0;

    } else {
      for (int j = 0; j < i; j++) {
        alpha -= L[i * n + j] * y[j];
      }
      y[i] = alpha / L[i * n + i];
    }
  }

  // Backward substitution
  // Now we have y, we can go back to (Lt x = y) and solve for x
  for (int i = n - 1; i >= 0; i--) {
    real_t alpha = y[i];

    if (fltcmp(Lt[i * n + i], 0.0) == 0) {
      x[i] = 0.0;

    } else {
      for (int j = i; j < (int) n; j++) {
        alpha -= Lt[i * n + j] * x[j];
      }
      x[i] = alpha / Lt[i * n + i];
    }
  }

  // Clean up
  free(y);
  free(L);
  free(Lt);
}

/**
 * Cholesky decomposition. Takes a `m x m` matrix `A` and decomposes it into a
 * lower and upper triangular matrix `L` and `U` with Cholesky decomposition.
 * This function only returns the `L` triangular matrix.
 */
void chol(const real_t *A, const size_t m, real_t *L) {
#ifdef USE_LAPACK
  __lapack_chol(A, m, L);
#else
  __chol(A, m, L);
#endif // USE_LAPACK
}

/**
 * Solve `Ax = b` using Cholesky decomposition, where `A` is a square matrix,
 * `b` is a vector and `x` is the solution vector of size `n`.
 */
void chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n) {
#ifdef USE_LAPACK
  __lapack_chol_solve(A, b, x, n);
#else
  __chol_solve(A, b, x, n);
#endif // USE_LAPACK
}

/******************************************************************************
 * Lie
 ******************************************************************************/

/**
 * Exponential Map
 */
void lie_Exp(const real_t phi[3], real_t C[3 * 3]) {
  assert(phi != NULL);
  assert(C != NULL);

  real_t phi_norm = vec_norm(phi, 3);
  real_t phi_skew[3 * 3] = {0};
  real_t phi_skew_sq[3 * 3] = {0};

  hat(phi, phi_skew);
  dot(phi_skew, 3, 3, phi_skew, 3, 3, phi_skew_sq);

  if (phi_norm < 1e-3) {
    // C = eye(3) + hat(phi);
    eye(C, 3, 3);
    mat_add(C, phi_skew, C, 3, 3);
  } else {
    // C = eye(3);
    // C += (sin(phi_norm) / phi_norm) * phi_skew;
    // C += ((1 - cos(phi_norm)) / phi_norm ^ 2) * phi_skew_sq;
    real_t A[3 * 3] = {0};
    mat_copy(phi_skew, 3, 3, A);
    mat_scale(A, 3, 3, (sin(phi_norm) / phi_norm));

    real_t B[3 * 3] = {0};
    mat_copy(phi_skew_sq, 3, 3, B);
    mat_scale(B, 3, 3, (1.0 - cos(phi_norm)) / (phi_norm * phi_norm));

    eye(C, 3, 3);
    mat_add(C, A, C, 3, 3);
    mat_add(C, B, C, 3, 3);
  }
}

/**
 * Logarithmic Map
 */
void lie_Log(const real_t C[3 * 3], real_t rvec[3]) {
  assert(C != NULL);
  assert(rvec != NULL);

  /**
   * phi = acos((trace(C) - 1) / 2);
   * vec = vee(C - C') / (2 * sin(phi));
   * rvec = phi * vec;
   */
  const real_t tr = C[0] + C[4] + C[8];
  const real_t phi = acos((tr - 1.0) / 2.0);

  real_t C_t[3 * 3] = {0};
  real_t dC[3 * 3] = {0};
  mat_transpose(C, 3, 3, C_t);
  mat_sub(C, C_t, dC, 3, 3);
  real_t u[3] = {0};
  vee(dC, u);
  const real_t s = 1.0 / (2 * sin(phi));
  const real_t vec[3] = {s * u[0], s * u[1], s * u[2]};

  rvec[0] = phi * vec[0];
  rvec[1] = phi * vec[1];
  rvec[2] = phi * vec[2];
}

/******************************************************************************
 * TRANSFORMS
 ******************************************************************************/

/** Form rotation matrix around x axis **/
void rotx(const real_t theta, real_t C[3 * 3]) {
  C[0] = 1.0;
  C[1] = 0.0;
  C[2] = 0.0;

  C[3] = 0.0;
  C[4] = cos(theta);
  C[5] = -sin(theta);

  C[6] = 0.0;
  C[7] = sin(theta);
  C[8] = cos(theta);
}

/** Form rotation matrix around y axis */
void roty(const real_t theta, real_t C[3 * 3]) {
  C[0] = cos(theta);
  C[1] = 0.0;
  C[2] = sin(theta);

  C[3] = 0.0;
  C[4] = 1.0;
  C[5] = 0.0;

  C[6] = -sin(theta);
  C[7] = 0.0;
  C[8] = cos(theta);
}

/** Form rotation matrix around z axis */
void rotz(const real_t theta, real_t C[3 * 3]) {
  C[0] = cos(theta);
  C[1] = -sin(theta);
  C[2] = 0.0;

  C[3] = sin(theta);
  C[4] = cos(theta);
  C[5] = 0.0;

  C[6] = 0.0;
  C[7] = 0.0;
  C[8] = 1.0;
}

/**
 * Form 4x4 homogeneous transformation matrix `T` from a 7x1 pose vector
 * `params`.
 *
 *    pose = (translation, rotation)
 *    pose = (rx, ry, rz, qx, qy, qz, qw)
 */
void tf(const real_t params[7], real_t T[4 * 4]) {
  assert(params != NULL);
  assert(T != NULL);

  const real_t r[3] = {params[0], params[1], params[2]};
  const real_t q[4] = {params[3], params[4], params[5], params[6]};

  real_t C[3 * 3] = {0};
  quat2rot(q, C);

  T[0] = C[0];
  T[1] = C[1];
  T[2] = C[2];
  T[3] = r[0];

  T[4] = C[3];
  T[5] = C[4];
  T[6] = C[5];
  T[7] = r[1];

  T[8] = C[6];
  T[9] = C[7];
  T[10] = C[8];
  T[11] = r[2];

  T[12] = 0.0;
  T[13] = 0.0;
  T[14] = 0.0;
  T[15] = 1.0;
}

/**
 * Form 4x4 homogeneous transformation matrix `T` from a rotation matrix `C` and
 * translation vector `r`.
 */
void tf_cr(const real_t C[3 * 3], const real_t r[3], real_t T[4 * 4]) {
  T[0] = C[0];
  T[1] = C[1];
  T[2] = C[2];
  T[3] = r[0];

  T[4] = C[3];
  T[5] = C[4];
  T[6] = C[5];
  T[7] = r[1];

  T[8] = C[6];
  T[9] = C[7];
  T[10] = C[8];
  T[11] = r[2];

  T[12] = 0.0;
  T[13] = 0.0;
  T[14] = 0.0;
  T[15] = 1.0;
}

/**
 * Form 4x4 homogeneous transformation matrix `T` from a quaternion `q` and
 * translation vector `r`.
 */
void tf_qr(const real_t q[4], const real_t r[3], real_t T[4 * 4]) {
  real_t C[3 * 3] = {0};
  quat2rot(q, C);
  tf_cr(C, r, T);
}

/**
 * Form 4x4 homogeneous transformation matrix `T` from a euler-angles `ypr`
 * (yaw-pitch-roll) and translation vector `r`.
 */
void tf_er(const real_t ypr[3], const real_t r[3], real_t T[4 * 4]) {
  real_t C[3 * 3] = {0};
  euler321(ypr, C);
  tf_cr(C, r, T);
}

/**
 * Form 7x1 pose parameter vector `params` from 4x4 homogeneous transformation
 * matrix `T`.
 */
void tf_vector(const real_t T[4 * 4], real_t params[7]) {
  assert(T != NULL);
  assert(params != NULL);

  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);

  real_t r[3] = {0};
  tf_trans_get(T, r);

  real_t q[4] = {0};
  rot2quat(C, q);

  params[0] = r[0];
  params[1] = r[1];
  params[2] = r[2];

  params[3] = q[0];
  params[4] = q[1];
  params[5] = q[2];
  params[6] = q[3];
}

/**
 * Decompose transform `T` into the rotation `C` and translation `r`
 * components.
 */
void tf_decompose(const real_t T[4 * 4], real_t C[3 * 3], real_t r[3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(r != NULL);

  C[0] = T[0];
  C[1] = T[1];
  C[2] = T[2];
  r[0] = T[3];

  C[3] = T[4];
  C[4] = T[5];
  C[5] = T[6];
  r[1] = T[7];

  C[6] = T[8];
  C[7] = T[9];
  C[8] = T[10];
  r[1] = T[11];
}

/**
 * Set the rotational component in the 4x4 transformation matrix `T` using a
 * 3x3 rotation matrix `C`.
 */
void tf_rot_set(real_t T[4 * 4], const real_t C[3 * 3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(T != C);

  T[0] = C[0];
  T[1] = C[1];
  T[2] = C[2];

  T[4] = C[3];
  T[5] = C[4];
  T[6] = C[5];

  T[8] = C[6];
  T[9] = C[7];
  T[10] = C[8];
}

/**
 * Get the rotation matrix `C` from the 4x4 transformation matrix `T`.
 */
void tf_rot_get(const real_t T[4 * 4], real_t C[3 * 3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(T != C);

  C[0] = T[0];
  C[1] = T[1];
  C[2] = T[2];

  C[3] = T[4];
  C[4] = T[5];
  C[5] = T[6];

  C[6] = T[8];
  C[7] = T[9];
  C[8] = T[10];
}

/**
 * Set the rotation component in the 4x4 transformation matrix `T` using a 4x1
 * quaternion `q`.
 */
void tf_quat_set(real_t T[4 * 4], const real_t q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  real_t C[3 * 3] = {0};
  quat2rot(q, C);
  tf_rot_set(T, C);
}

/**
 * Get the quaternion `q` from the 4x4 transformation matrix `T`.
 */
void tf_quat_get(const real_t T[4 * 4], real_t q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);
  rot2quat(C, q);
}

/**
 * Set the rotational component in the 4x4 transformation matrix `T` using a
 * 3x1 euler angle vector `euler`.
 */
void tf_euler_set(real_t T[4 * 4], const real_t ypr[3]) {
  assert(T != NULL);
  assert(ypr != NULL);
  assert(T != ypr);

  real_t C[3 * 3] = {0};
  euler321(ypr, C);
  tf_rot_set(T, C);
}

/**
 * Get the rotational component in the 4x4 transformation matrix `T` in the
 * form of a 3x1 euler angle vector `ypr`.
 */
void tf_euler_get(const real_t T[4 * 4], real_t ypr[3]) {
  assert(T != NULL);
  assert(ypr != NULL);
  assert(T != ypr);

  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);
  rot2euler(C, ypr);
}

/**
 * Set the translational component in the 4x4 transformation matrix `T` using
 * a 3x1 translation vector `r`.
 */
void tf_trans_set(real_t T[4 * 4], const real_t r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  T[3] = r[0];
  T[7] = r[1];
  T[11] = r[2];
}

/**
 * Get the translational vector `r` from the 4x4 transformation matrix `T`.
 */
void tf_trans_get(const real_t T[4 * 4], real_t r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

/**
 * Invert the 4x4 homogeneous transformation matrix `T` where results are
 * written to `T_inv`.
 */
void tf_inv(const real_t T[4 * 4], real_t T_inv[4 * 4]) {
  assert(T != NULL);
  assert(T_inv != NULL);
  assert(T != T_inv);

  /**
   * Transformation T comprises of rotation C and translation r:
   *
   *   T = [C0, C1, C2, r0]
   *       [C3, C4, C5, r1]
   *       [C6, C7, C8, r2]
   *       [0, 0, 0, 1]
   *
   * The inverse is therefore:
   *
   *   C_inv = C^T
   *   r_inv = -C^T * r
   *
   *   T_inv = [C0, C3, C6, -C0*r0 - C3*r1 - C6*r2]
   *           [C1, C4, C7, -C1*r0 - C4*r1 - C7*r2]
   *           [C2, C5, C8, -C2*r0 - C5*r1 - C8*r2]
   *           [0, 0, 0, 1]
   */

  // Get rotation and translation components
  real_t C[3 * 3] = {0};
  real_t r[3] = {0};
  tf_rot_get(T, C);
  tf_trans_get(T, r);

  // Invert translation
  real_t r_out[3] = {0};
  r_out[0] = -C[0] * r[0] - C[3] * r[1] - C[6] * r[2];
  r_out[1] = -C[1] * r[0] - C[4] * r[1] - C[7] * r[2];
  r_out[2] = -C[2] * r[0] - C[5] * r[1] - C[8] * r[2];

  // First row
  T_inv[0] = C[0];
  T_inv[1] = C[3];
  T_inv[2] = C[6];
  T_inv[3] = r_out[0];

  // Second row
  T_inv[4] = C[1];
  T_inv[5] = C[4];
  T_inv[6] = C[7];
  T_inv[7] = r_out[1];

  // Third row
  T_inv[8] = C[2];
  T_inv[9] = C[5];
  T_inv[10] = C[8];
  T_inv[11] = r_out[2];

  // Fourth row
  T_inv[12] = 0.0;
  T_inv[13] = 0.0;
  T_inv[14] = 0.0;
  T_inv[15] = 1.0;
}

/**
 * Transform 3x1 point `p` using 4x4 homogeneous transformation matrix `T` and
 * output to 3x1 `retval`.
 */
void tf_point(const real_t T[4 * 4], const real_t p[3], real_t retval[3]) {
  assert(T != NULL);
  assert(p != NULL);
  assert(retval != NULL);
  assert(p != retval);

  const real_t hp_a[4] = {p[0], p[1], p[2], 1.0};
  real_t hp_b[4] = {0.0, 0.0, 0.0, 0.0};
  dot(T, 4, 4, hp_a, 4, 1, hp_b);

  retval[0] = hp_b[0];
  retval[1] = hp_b[1];
  retval[2] = hp_b[2];
}

/**
 * Transform 4x1 homogeneous point `hp` using 4x4 homogeneous transformation
 * matrix `T` and output to 4x1 `retval`.
 */
void tf_hpoint(const real_t T[4 * 4], const real_t hp[4], real_t retval[4]) {
  assert(T != NULL);
  assert(hp != retval);
  assert(retval != NULL);
  dot(T, 4, 4, hp, 4, 1, retval);
}

/**
 * Perturb the `i`-th rotational component of a 4x4 homogeneous transformation
 * matrix `T` with `step_size`.
 */
void tf_perturb_rot(real_t T[4 * 4], const real_t step_size, const int i) {
  assert(T != NULL);
  assert(i >= 0 && i <= 2);

  // Build perturb drvec
  real_t drvec[3] = {0};
  drvec[i] = step_size;

  // Decompose transform to rotation and translation
  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);

  // Perturb rotation
  real_t C_rvec[3 * 3] = {0};
  real_t C_diff[3 * 3] = {0};
  rvec2rot(drvec, 1e-8, C_rvec);
  dot(C, 3, 3, C_rvec, 3, 3, C_diff);
  tf_rot_set(T, C_diff);
}

/**
 * Perturb the `i`-th translation component of a 4x4 homogeneous
 * transformation matrix with `step_size`.
 */
void tf_perturb_trans(real_t T[4 * 4], const real_t step_size, const int i) {
  assert(T != NULL);
  assert(i >= 0 && i <= 2);

  // Build perturb dr
  real_t dr[3] = {0};
  dr[i] = step_size;

  // Decompose transform get translation
  real_t r[3] = {0};
  tf_trans_get(T, r);

  // Perturb translation
  const real_t r_diff[3] = {r[0] + dr[0], r[1] + dr[1], r[2] + dr[2]};
  tf_trans_set(T, r_diff);
}

/**
 * Chain `N` homogeneous transformations `tfs`.
 */
void tf_chain(const real_t **tfs, const int N, real_t T_out[4 * 4]) {
  assert(tfs != NULL);
  assert(T_out != NULL);

  // Initialize T_out with the first transform in tfs
  mat_copy(tfs[0], 4, 4, T_out);

  // Chain transforms
  for (int i = 1; i < N; i++) {
    real_t T_from[4 * 4] = {0};
    mat_copy(T_out, 4, 4, T_from);
    dot(T_from, 4, 4, tfs[i], 4, 4, T_out);
  }
}

/**
 * Print pose vector
 */
void print_pose_vector(const char *prefix, const real_t pose[7]) {
  if (prefix) {
    printf("%s: ", prefix);
  }

  for (int i = 0; i < 7; i++) {
    printf("%.2f", pose[i]);
    if ((i + 1) < 7) {
      printf(", ");
    }
  }
  printf("\n");
}

/**
 * Convert rotation vector `rvec` to 3x3 rotation matrix `R`, where `eps` is
 * the tolerance to determine if the rotation is too small.
 */
void rvec2rot(const real_t *rvec, const real_t eps, real_t *R) {
  assert(rvec != NULL);
  assert(eps > 0);
  assert(R != NULL);

  // Magnitude of rvec
  const real_t theta = sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1]);
  // ^ basically norm(rvec), but faster

  // Check if rotation is too small
  if (theta < eps) {
    R[0] = 1.0;
    R[1] = -rvec[2];
    R[2] = rvec[1];

    R[3] = rvec[2];
    R[4] = 1.0;
    R[5] = -rvec[0];

    R[6] = -rvec[1];
    R[7] = rvec[0], R[8] = 1.0;
    return;
  }

  // Convert rvec to rotation matrix
  real_t rvec_normed[3] = {rvec[0], rvec[1], rvec[2]};
  vec_scale(rvec_normed, 3, 1 / theta);
  const real_t x = rvec_normed[0];
  const real_t y = rvec_normed[1];
  const real_t z = rvec_normed[2];

  const real_t c = cos(theta);
  const real_t s = sin(theta);
  const real_t C = 1 - c;

  const real_t xs = x * s;
  const real_t ys = y * s;
  const real_t zs = z * s;

  const real_t xC = x * C;
  const real_t yC = y * C;
  const real_t zC = z * C;

  const real_t xyC = x * yC;
  const real_t yzC = y * zC;
  const real_t zxC = z * xC;

  R[0] = x * xC + c;
  R[1] = xyC - zs;
  R[2] = zxC + ys;

  R[3] = xyC + zs;
  R[4] = y * yC + c;
  R[5] = yzC - xs;

  R[6] = zxC - ys;
  R[7] = yzC + xs;
  R[8] = z * zC + c;
}

/**
 * Convert Euler angles `ypr` (yaw, pitch, roll) in degrees to a 3x3 rotation
 * matrix `C`.
 */
void euler321(const real_t ypr[3], real_t C[3 * 3]) {
  assert(ypr != NULL);
  assert(C != NULL);

  const real_t psi = ypr[0];
  const real_t theta = ypr[1];
  const real_t phi = ypr[2];

  const real_t cpsi = cos(psi);
  const real_t spsi = sin(psi);

  const real_t ctheta = cos(theta);
  const real_t stheta = sin(theta);

  const real_t cphi = cos(phi);
  const real_t sphi = sin(phi);

  // 1st row
  C[0] = cpsi * ctheta;
  C[1] = cpsi * stheta * sphi - spsi * cphi;
  C[2] = cpsi * stheta * cphi + spsi * sphi;

  // 2nd row
  C[3] = spsi * ctheta;
  C[4] = spsi * stheta * sphi + cpsi * cphi;
  C[5] = spsi * stheta * cphi - cpsi * sphi;

  // 3rd row
  C[6] = -stheta;
  C[7] = ctheta * sphi;
  C[8] = ctheta * cphi;
}

/**
 * Convert Euler angles `ypr` in radians to a Hamiltonian Quaternion.
 */
void euler2quat(const real_t ypr[3], real_t q[4]) {
  const real_t psi = ypr[0];
  const real_t theta = ypr[1];
  const real_t phi = ypr[2];

  const real_t cphi = cos(phi / 2.0);
  const real_t ctheta = cos(theta / 2.0);
  const real_t cpsi = cos(psi / 2.0);
  const real_t sphi = sin(phi / 2.0);
  const real_t stheta = sin(theta / 2.0);
  const real_t spsi = sin(psi / 2.0);

  const real_t qx = sphi * ctheta * cpsi - cphi * stheta * spsi;
  const real_t qy = cphi * stheta * cpsi + sphi * ctheta * spsi;
  const real_t qz = cphi * ctheta * spsi - sphi * stheta * cpsi;
  const real_t qw = cphi * ctheta * cpsi + sphi * stheta * spsi;

  const real_t mag = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  q[0] = qw / mag;
  q[1] = qx / mag;
  q[2] = qy / mag;
  q[3] = qz / mag;
}

/**
 * Convert 3x3 rotation matrix `C` to Quaternion `q`.
 */
void rot2quat(const real_t C[3 * 3], real_t q[4]) {
  assert(C != NULL);
  assert(q != NULL);

  const real_t C00 = C[0];
  const real_t C01 = C[1];
  const real_t C02 = C[2];
  const real_t C10 = C[3];
  const real_t C11 = C[4];
  const real_t C12 = C[5];
  const real_t C20 = C[6];
  const real_t C21 = C[7];
  const real_t C22 = C[8];

  const real_t tr = C00 + C11 + C22;
  real_t S = 0.0f;
  real_t qw = 0.0f;
  real_t qx = 0.0f;
  real_t qy = 0.0f;
  real_t qz = 0.0f;

  if (tr > 0) {
    S = sqrt(tr + 1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (C21 - C12) / S;
    qy = (C02 - C20) / S;
    qz = (C10 - C01) / S;
  } else if ((C00 > C11) && (C[0] > C22)) {
    S = sqrt(1.0 + C[0] - C11 - C22) * 2; // S=4*qx
    qw = (C21 - C12) / S;
    qx = 0.25 * S;
    qy = (C01 + C10) / S;
    qz = (C02 + C20) / S;
  } else if (C11 > C22) {
    S = sqrt(1.0 + C11 - C[0] - C22) * 2; // S=4*qy
    qw = (C02 - C20) / S;
    qx = (C01 + C10) / S;
    qy = 0.25 * S;
    qz = (C12 + C21) / S;
  } else {
    S = sqrt(1.0 + C22 - C[0] - C11) * 2; // S=4*qz
    qw = (C10 - C01) / S;
    qx = (C02 + C20) / S;
    qy = (C12 + C21) / S;
    qz = 0.25 * S;
  }

  q[0] = qw;
  q[1] = qx;
  q[2] = qy;
  q[3] = qz;
}

/**
 * Convert 3 x 3 rotation matrix `C` to euler angles `euler`.
 */
void rot2euler(const real_t C[3 * 3], real_t ypr[3]) {
  assert(C != NULL);
  assert(ypr != NULL);

  real_t q[4] = {0};
  rot2quat(C, q);
  quat2euler(q, ypr);
}

/**
 * Convert Quaternion `q` to Euler angles 3x1 vector `euler`.
 */
void quat2euler(const real_t q[4], real_t ypr[3]) {
  assert(q != NULL);
  assert(ypr != NULL);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  const real_t qw2 = qw * qw;
  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;

  const real_t t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const real_t t2 = asin(2 * (qy * qw - qx * qz));
  const real_t t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  ypr[0] = t3;
  ypr[1] = t2;
  ypr[2] = t1;
}

/**
 * Print Quaternion
 */
void print_quat(const char *prefix, const real_t q[4]) {
  printf("%s: [w: %.10f, x: %.10f, y: %.10f, z: %.10f]\n",
         prefix,
         q[0],
         q[1],
         q[2],
         q[3]);
}

/**
 * Return Quaternion norm
 */
real_t quat_norm(const real_t q[4]) {
  return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

/**
 * Setup Quaternion
 */
void quat_setup(real_t q[4]) {
  q[0] = 1.0;
  q[1] = 0.0;
  q[2] = 0.0;
  q[3] = 0.0;
}

/**
 * Normalize Quaternion
 */
void quat_normalize(real_t q[4]) {
  const real_t n = quat_norm(q);
  q[0] = q[0] / n;
  q[1] = q[1] / n;
  q[2] = q[2] / n;
  q[3] = q[3] / n;
}

/**
 * Normalize Quaternion
 */
void quat_normalize_copy(const real_t q[4], real_t q_normalized[4]) {
  const real_t n = quat_norm(q);
  q_normalized[0] = q[0] / n;
  q_normalized[1] = q[1] / n;
  q_normalized[2] = q[2] / n;
  q_normalized[3] = q[3] / n;
}

/**
 * Convert Quaternion `q` to 3x3 rotation matrix `C`.
 */
void quat2rot(const real_t q[4], real_t C[3 * 3]) {
  assert(q != NULL);
  assert(C != NULL);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;
  const real_t qw2 = qw * qw;

  // Homogeneous form
  // -- 1st row
  C[0] = qw2 + qx2 - qy2 - qz2;
  C[1] = 2 * (qx * qy - qw * qz);
  C[2] = 2 * (qx * qz + qw * qy);
  // -- 2nd row
  C[3] = 2 * (qx * qy + qw * qz);
  C[4] = qw2 - qx2 + qy2 - qz2;
  C[5] = 2 * (qy * qz - qw * qx);
  // -- 3rd row
  C[6] = 2 * (qx * qz - qw * qy);
  C[7] = 2 * (qy * qz + qw * qx);
  C[8] = qw2 - qx2 - qy2 + qz2;
}

/**
 * Inverse Quaternion `q`.
 */
void quat_inv(const real_t q[4], real_t q_inv[4]) {
  q_inv[0] = q[0];
  q_inv[1] = -q[1];
  q_inv[2] = -q[2];
  q_inv[3] = -q[3];
}

/**
 * Form Quaternion left multiplication matrix.
 */
void quat_left(const real_t q[4], real_t left[4 * 4]) {
  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  // clang-format off
  left[0]  = qw; left[1]  = -qx; left[2]  = -qy; left[3]  = -qz;
  left[4]  = qx; left[5]  = qw;  left[6]  = -qz; left[7]  = qy;
  left[8]  = qy; left[9]  = qz;  left[10] = qw;  left[11] = -qx;
  left[12] = qz; left[13] = -qy; left[14] = qx;  left[15] = qw;
  // clang-format on
}

/**
 * Form Quaternion left multiplication matrix.
 */
void quat_left_xyz(const real_t q[4], real_t left_xyz[3 * 3]) {
  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  // clang-format off
  left_xyz[0]  = qw; left_xyz[1] = -qz; left_xyz[2]  = qy;
  left_xyz[3]  = qz; left_xyz[4] = qw;  left_xyz[5] = -qx;
  left_xyz[6] = -qy; left_xyz[7] = qx;  left_xyz[8] = qw;
  // clang-format on
}

/**
 * Form Quaternion right multiplication matrix.
 */
void quat_right(const real_t q[4], real_t right[4 * 4]) {
  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  // clang-format off
  right[0] = qw;
  right[1] = -qx;
  right[2] = -qy;
  right[3] = -qz;
  right[4] = qx;
  right[5] = qw;
  right[6] = qz;
  right[7] = -qy;
  right[8] = qy;
  right[9] = -qz;
  right[10] = qw;
  right[11] = qx;
  right[12] = qz;
  right[13] = qy;
  right[14] = -qx;
  right[15] = qw;
  // clang-format on
}

/**
 * Quaternion left-multiply `p` with `q`, results are outputted to `r`.
 */
void quat_lmul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL);
  assert(q != NULL);
  assert(r != NULL);

  const real_t pw = p[0];
  const real_t px = p[1];
  const real_t py = p[2];
  const real_t pz = p[3];

  r[0] = pw * q[0] - px * q[1] - py * q[2] - pz * q[3];
  r[1] = px * q[0] + pw * q[1] - pz * q[2] + py * q[3];
  r[2] = py * q[0] + pz * q[1] + pw * q[2] - px * q[3];
  r[3] = pz * q[0] - py * q[1] + px * q[2] + pw * q[3];
}

/**
 * Quaternion right-multiply `p` with `q`, results are outputted to `r`.
 */
void quat_rmul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL);
  assert(q != NULL);
  assert(r != NULL);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  r[0] = qw * q[0] - qx * q[1] - qy * q[2] - qz * q[3];
  r[1] = qx * q[0] + qw * q[1] + qz * q[2] - qy * q[3];
  r[2] = qy * q[0] - qz * q[1] + qw * q[2] + qx * q[3];
  r[3] = qz * q[0] + qy * q[1] - qx * q[2] + qw * q[3];
}

/**
 * Quaternion multiply `p` with `q`, results are outputted to `r`.
 */
void quat_mul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL);
  assert(q != NULL);
  assert(r != NULL);
  quat_lmul(p, q, r);
}

/**
 * Form delta quaternion `dq` from a small rotation vector `dalpha`.
 */
void quat_delta(const real_t dalpha[3], real_t dq[4]) {
  assert(dalpha != NULL);
  assert(dq != NULL);

  const real_t half_norm = 0.5 * vec_norm(dalpha, 3);
  const real_t k = sinc(half_norm) * 0.5;
  const real_t vector[3] = {k * dalpha[0], k * dalpha[1], k * dalpha[2]};
  real_t scalar = cos(half_norm);

  dq[0] = scalar;
  dq[1] = vector[0];
  dq[2] = vector[1];
  dq[3] = vector[2];
}

/**
 * Update quaternion with small update dalpha.
 */
void quat_update(real_t q[4], const real_t dalpha[3]) {
  const real_t dq[4] = {1.0, 0.5 * dalpha[0], 0.5 * dalpha[1], 0.5 * dalpha[2]};
  real_t q_new[4] = {0};
  quat_mul(q, dq, q_new);
  q[0] = q_new[0];
  q[1] = q_new[1];
  q[2] = q_new[2];
  q[3] = q_new[3];
}

/**
 * Update quaternion with angular velocity and dt.
 */
void quat_update_dt(real_t q[4], const real_t w[3], const real_t dt) {
  real_t dalpha[3] = {w[0] * dt, w[1] * dt, w[2] * dt};
  quat_update(q, dalpha);
}

/**
 * Perturb quaternion
 */
void quat_perturb(real_t q[4], const int i, const real_t h) {
  assert(i >= 0 && i <= 2);

  // Form small pertubation quaternion dq
  real_t dalpha[3] = {0};
  real_t dq[4] = {0};
  dalpha[i] = h;
  quat_delta(dalpha, dq);

  // Perturb quaternion
  real_t q_[4] = {q[0], q[1], q[2], q[3]};
  quat_mul(q_, dq, q);
  quat_normalize(q);
}

/*****************************************************************************
 * CV
 *****************************************************************************/

// IMAGE ///////////////////////////////////////////////////////////////////////

/**
 * Setup image `img` with `width`, `height` and `data`.
 */
void image_setup(image_t *img,
                 const int width,
                 const int height,
                 uint8_t *data) {
  assert(img != NULL);
  img->width = width;
  img->height = height;
  img->data = data;
}

/**
 * Load image at `file_path`.
 * @returns Heap allocated image
 */
image_t *image_load(const char *file_path) {
  assert(file_path != NULL);

#ifdef USE_STB_IMAGE
  int img_w = 0;
  int img_h = 0;
  int img_c = 0;
  stbi_set_flip_vertically_on_load(1);
  uint8_t *data = stbi_load(file_path, &img_w, &img_h, &img_c, 0);
  if (!data) {
    FATAL("Failed to load image file: [%s]", file_path);
  }

  image_t *img = MALLOC(image_t, 1);
  img->width = img_w;
  img->height = img_h;
  img->channels = img_c;
  img->data = data;
  return img;
#else
  FATAL("Not Implemented!");
#endif
}

/**
 * Print image properties.
 */
void image_print_properties(const image_t *img) {
  assert(img != NULL);
  printf("img.width: %d\n", img->width);
  printf("img.height: %d\n", img->height);
  printf("img.channels: %d\n", img->channels);
}

/**
 * Free image.
 */
void image_free(image_t *img) {
  assert(img != NULL);
  free(img->data);
  free(img);
}

// GEOMETRY ////////////////////////////////////////////////////////////////////

/**
 * Triangulate a single 3D point `p` observed by two different camera frames
 * represented by two 3x4 camera projection matrices `P_i` and `P_j`, and the
 * 2D image point correspondance `z_i` and `z_j`.
 */
void linear_triangulation(const real_t P_i[3 * 4],
                          const real_t P_j[3 * 4],
                          const real_t z_i[2],
                          const real_t z_j[2],
                          real_t p[3]) {
  // Form A matrix
  real_t A[4 * 4] = {0};
  // -- ROW 1
  A[0] = -P_i[4] + P_i[8] * z_i[1];
  A[1] = -P_i[5] + P_i[9] * z_i[1];
  A[2] = P_i[10] * z_i[1] - P_i[6];
  A[3] = P_i[11] * z_i[1] - P_i[7];
  // -- ROW 2
  A[4] = -P_i[0] + P_i[8] * z_i[0];
  A[5] = -P_i[1] + P_i[9] * z_i[0];
  A[6] = P_i[10] * z_i[0] - P_i[2];
  A[7] = P_i[11] * z_i[0] - P_i[3];
  // -- ROW 3
  A[8] = -P_j[4] + P_j[8] * z_j[1];
  A[9] = -P_j[5] + P_j[9] * z_j[1];
  A[10] = P_j[10] * z_j[1] - P_j[6];
  A[11] = P_j[11] * z_j[1] - P_j[7];
  // -- ROW 4
  A[12] = -P_j[0] + P_j[8] * z_j[0];
  A[13] = -P_j[1] + P_j[9] * z_j[0];
  A[14] = P_j[10] * z_j[0] - P_j[2];
  A[15] = P_j[11] * z_j[0] - P_j[3];

  // Form A_t
  real_t A_t[4 * 4] = {0};
  mat_transpose(A, 4, 4, A_t);

  // SVD
  real_t A2[4 * 4] = {0};
  real_t s[4] = {0};
  real_t U[4 * 4] = {0};
  real_t V[4 * 4] = {0};
  dot(A_t, 4, 4, A, 4, 4, A2);
  svd(A2, 4, 4, U, s, V);

  // Get last row of V_t and normalize the scale to obtain the 3D point
  const real_t x = V[3];
  const real_t y = V[7];
  const real_t z = V[11];
  const real_t w = V[15];
  p[0] = x / w;
  p[1] = y / w;
  p[2] = z / w;
}

// RADTAN //////////////////////////////////////////////////////////////////////

/**
 * Distort 2x1 point `p` using Radial-Tangential distortion, where the
 * distortion params are stored in `params` (k1, k2, p1, p2), results are
 * written to 2x1 vector `p_d`.
 */
void radtan4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(p_d != NULL);

  // Distortion parameters
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t p1 = params[2];
  const real_t p2 = params[3];

  // Point
  const real_t x = p[0];
  const real_t y = p[1];

  // Apply radial distortion
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

  // Apply tangential distortion
  const real_t xy = x * y;
  const real_t x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const real_t y_ddash = y_dash + (2.0 * p2 * xy + p1 * (r2 + 2.0 * y2));

  // Distorted point
  p_d[0] = x_ddash;
  p_d[1] = y_ddash;
}

/**
 * Form Radial-Tangential point jacobian, using distortion `params` (k1, k2,
 * p1, p2), 2x1 image point `p`, the jacobian is written to 2x2 `J_point`.
 */
void radtan4_point_jacobian(const real_t params[4],
                            const real_t p[2],
                            real_t J_point[2 * 2]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_point != NULL);

  // Distortion parameters
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t p1 = params[2];
  const real_t p2 = params[3];

  // Point
  const real_t x = p[0];
  const real_t y = p[1];

  // Apply radial distortion
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  // Point Jacobian is 2x2
  J_point[0] = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x;
  J_point[0] += x * (2 * k1 * x + 4 * k2 * x * r2) + 1;
  J_point[1] = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point[2] = J_point[1];
  J_point[3] = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x;
  J_point[3] += y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
}

/**
 * Form Radial-Tangential parameter jacobian, using distortion `params` (k1,
 * k2, p1, p2), 2x1 image point `p`, the jacobian is written to 2x4 `J_param`.
 */
void radtan4_params_jacobian(const real_t params[4],
                             const real_t p[2],
                             real_t J_param[2 * 4]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_param != NULL);
  UNUSED(params);

  // Point
  const real_t x = p[0];
  const real_t y = p[1];

  // Setup
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t xy = x * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  // Param Jacobian is 2x4
  J_param[0] = x * r2;
  J_param[1] = x * r4;
  J_param[2] = 2 * xy;
  J_param[3] = 3 * x2 + y2;

  J_param[4] = y * r2;
  J_param[5] = y * r4;
  J_param[6] = x2 + 3 * y2;
  J_param[7] = 2 * xy;
}

// EQUI ////////////////////////////////////////////////////////////////////////

/**
 * Distort 2x1 point `p` using Equi-Distant distortion, where the
 * distortion params are stored in `params` (k1, k2, k3, k4), results are
 * written to 2x1 vector `p_d`.
 */
void equi4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(p_d != NULL);

  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t k3 = params[2];
  const real_t k4 = params[3];

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const real_t s = thd / r;

  p_d[0] = s * x;
  p_d[1] = s * y;
}

/**
 * Form Equi-Distant point jacobian, using distortion `params` (k1, k2, k3,
 * k4), 2x1 image point `p`, the jacobian is written to 2x2 `J_point`.
 */
void equi4_point_jacobian(const real_t params[4],
                          const real_t p[2],
                          real_t J_point[2 * 2]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_point != NULL);

  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t k3 = params[2];
  const real_t k4 = params[3];

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);

  const real_t th_r = 1.0 / (r * r + 1.0);
  const real_t thd_th =
      1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  const real_t s = thd / r;
  const real_t s_r = thd_th * th_r / r - thd / (r * r);
  const real_t r_x = 1.0 / r * x;
  const real_t r_y = 1.0 / r * y;

  // Point Jacobian is 2x2
  J_point[0] = s + x * s_r * r_x;
  J_point[1] = x * s_r * r_y;
  J_point[2] = y * s_r * r_x;
  J_point[3] = s + y * s_r * r_y;
}

/**
 * Form Equi-Distant parameter jacobian, using distortion `params` (k1, k2,
 * k3, k4), 2x1 image point `p`, the jacobian is written to 2x4 `J_param`.
 */
void equi4_params_jacobian(const real_t params[4],
                           const real_t p[2],
                           real_t J_param[2 * 4]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_param != NULL);
  UNUSED(params);

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th3 = th2 * th;
  const real_t th5 = th3 * th2;
  const real_t th7 = th5 * th2;
  const real_t th9 = th7 * th2;

  // Param Jacobian is 2x4
  J_param[0] = x * th3 / r;
  J_param[1] = x * th5 / r;
  J_param[2] = x * th7 / r;
  J_param[3] = x * th9 / r;

  J_param[4] = y * th3 / r;
  J_param[5] = y * th5 / r;
  J_param[6] = y * th7 / r;
  J_param[7] = y * th9 / r;
}

// PINHOLE /////////////////////////////////////////////////////////////////////

/**
 * Estimate pinhole focal length. The focal length is estimated using
 * `image_width` [pixels], and `fov` (Field of view of the camera) [rad].
 */
real_t pinhole_focal(const int image_width, const real_t fov) {
  return ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
}

/**
 * From 3x3 camera matrix K using pinhole camera parameters.
 *
 *   K = [fx,  0,  cx,
 *         0  fy,  cy,
 *         0   0,   1];
 *
 * where `params` is assumed to contain the fx, fy, cx, cy in that order.
 */
void pinhole_K(const real_t params[4], real_t K[3 * 3]) {
  K[0] = params[0];
  K[1] = 0.0;
  K[2] = params[2];

  K[3] = 0.0;
  K[4] = params[1];
  K[5] = params[3];

  K[6] = 0.0;
  K[7] = 0.0;
  K[8] = 1.0;
}

/**
 * Form 3x4 pinhole projection matrix `P`:
 *
 *   P = K * [-C | -C * r];
 *
 * Where K is the pinhole camera matrix formed using the camera parameters
 * `params` (fx, fy, cx, cy), C and r is the rotation and translation component
 * of the camera pose represented as a 4x4 homogenous transform `T`.
 */
void pinhole_projection_matrix(const real_t params[4],
                               const real_t T[4 * 4],
                               real_t P[3 * 4]) {
  assert(params != NULL);
  assert(T != NULL);
  assert(P != NULL);

  // Form K matrix
  real_t K[3 * 3] = {0};
  pinhole_K(params, K);

  // Invert camera pose
  real_t T_inv[4 * 4] = {0};
  tf_inv(T, T_inv);

  // Extract rotation and translation component
  real_t C[3 * 3] = {0};
  real_t r[3] = {0};
  tf_rot_get(T_inv, C);
  tf_trans_get(T_inv, r);

  // Form [C | r] matrix
  real_t Cr[3 * 4] = {0};
  Cr[0] = C[0];
  Cr[1] = C[1];
  Cr[2] = C[2];
  Cr[3] = r[0];

  Cr[4] = C[3];
  Cr[5] = C[4];
  Cr[6] = C[5];
  Cr[7] = r[1];

  Cr[8] = C[6];
  Cr[9] = C[7];
  Cr[10] = C[8];
  Cr[11] = r[2];

  // Form projection matrix P = K * [C | r]
  dot(K, 3, 3, Cr, 3, 4, P);
}

/**
 * Project 3D point `p_C` observed from the camera to the image plane `z`
 * using pinhole parameters `params` (fx, fy, cx, cy).
 */
void pinhole_project(const real_t params[4], const real_t p_C[3], real_t z[2]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(z != NULL);

  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  const real_t px = p_C[0] / p_C[2];
  const real_t py = p_C[1] / p_C[2];

  z[0] = px * fx + cx;
  z[1] = py * fy + cy;
}

/**
 * Form Pinhole point jacobian `J` using pinhole parameters `params`.
 */
void pinhole_point_jacobian(const real_t params[4], real_t J[2 * 2]) {
  assert(params != NULL);
  assert(J != NULL);

  J[0] = params[0];
  J[1] = 0.0;
  J[2] = 0.0;
  J[3] = params[1];
}

/**
 * Form Pinhole parameter jacobian `J` using pinhole parameters `params` and
 * 2x1 image point `x`.
 */
void pinhole_params_jacobian(const real_t params[4],
                             const real_t x[2],
                             real_t J[2 * 4]) {
  assert(params != NULL);
  assert(x != NULL);
  assert(J != NULL);
  UNUSED(params);

  J[0] = x[0];
  J[1] = 0.0;
  J[2] = 1.0;
  J[3] = 0.0;

  J[4] = 0.0;
  J[5] = x[1];
  J[6] = 0.0;
  J[7] = 1.0;
}

// PINHOLE-RADTAN4 /////////////////////////////////////////////////////////////

/**
 * Projection of 3D point to image plane using Pinhole + Radial-Tangential.
 */
void pinhole_radtan4_project(const real_t params[8],
                             const real_t p_C[3],
                             real_t x[2]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(x != NULL);

  // Project
  const real_t p[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};

  // Distort
  const real_t d[4] = {params[4], params[5], params[6], params[7]};
  real_t p_d[2] = {0};
  radtan4_distort(d, p, p_d);

  // Scale and center
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  x[0] = p_d[0] * fx + cx;
  x[1] = p_d[1] * fy + cy;
}

/**
 * Projection Jacobian of Pinhole + Radial-Tangential.
 */
void pinhole_radtan4_project_jacobian(const real_t params[8],
                                      const real_t p_C[3],
                                      real_t J[2 * 3]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  // Project
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  // Projection Jacobian
  real_t J_p[2 * 3] = {0};
  J_p[0] = 1.0 / z;
  J_p[1] = 0.0;
  J_p[2] = -x / (z * z);
  J_p[3] = 0.0;
  J_p[4] = 1.0 / z;
  J_p[5] = -y / (z * z);

  // Distortion Point Jacobian
  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};
  real_t J_d[2 * 2] = {0};
  radtan4_point_jacobian(d, p, J_d);

  // Project Point Jacobian
  real_t J_k[2 * 3] = {0};
  pinhole_point_jacobian(params, J_k);

  // J = J_k * J_d * J_p;
  real_t J_dp[2 * 3] = {0};
  dot(J_d, 2, 2, J_p, 2, 3, J_dp);
  dot(J_k, 2, 2, J_dp, 2, 3, J);
}

/**
 * Parameter Jacobian of Pinhole + Radial-Tangential.
 */
void pinhole_radtan4_params_jacobian(const real_t params[8],
                                     const real_t p_C[3],
                                     real_t J[2 * 8]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t k[4] = {fx, fy, cx, cy};

  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};

  // Project
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  // Distort
  real_t p_d[2] = {0};
  radtan4_distort(d, p, p_d);

  // Project params Jacobian: J_proj_params
  real_t J_proj_params[2 * 4] = {0};
  pinhole_params_jacobian(k, p_d, J_proj_params);

  // Project point Jacobian: J_proj_point
  real_t J_proj_point[2 * 2] = {0};
  pinhole_point_jacobian(k, J_proj_point);

  // Distortion point Jacobian: J_dist_params
  real_t J_dist_params[2 * 4] = {0};
  radtan4_params_jacobian(d, p, J_dist_params);

  // Radtan4 params Jacobian: J_radtan4
  real_t J_radtan4[2 * 4] = {0};
  dot(J_proj_point, 2, 2, J_dist_params, 2, 4, J_radtan4);

  // J = [J_proj_params, J_proj_point * J_dist_params]
  J[0] = J_proj_params[0];
  J[1] = J_proj_params[1];
  J[2] = J_proj_params[2];
  J[3] = J_proj_params[3];

  J[8] = J_proj_params[4];
  J[9] = J_proj_params[5];
  J[10] = J_proj_params[6];
  J[11] = J_proj_params[7];

  J[4] = J_radtan4[0];
  J[5] = J_radtan4[1];
  J[6] = J_radtan4[2];
  J[7] = J_radtan4[3];

  J[12] = J_radtan4[4];
  J[13] = J_radtan4[5];
  J[14] = J_radtan4[6];
  J[15] = J_radtan4[7];
}

// PINHOLE-EQUI4 ///////////////////////////////////////////////////////////////

/**
 * Projection of 3D point to image plane using Pinhole + Equi-Distant.
 */
void pinhole_equi4_project(const real_t params[8],
                           const real_t p_C[3],
                           real_t x[2]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(x != NULL);

  // Project
  const real_t p[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};

  // Distort
  const real_t d[4] = {params[4], params[5], params[6], params[7]};
  real_t p_d[2] = {0};
  equi4_distort(d, p, p_d);

  // Scale and center
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  x[0] = p_d[0] * fx + cx;
  x[1] = p_d[1] * fy + cy;
}

/**
 * Projection Jacobian of Pinhole + Equi-Distant.
 */
void pinhole_equi4_project_jacobian(const real_t params[8],
                                    const real_t p_C[3],
                                    real_t J[2 * 3]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  // Project
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  // Projection Jacobian
  real_t J_p[2 * 3] = {0};
  J_p[0] = 1.0 / z;
  J_p[1] = 0.0;
  J_p[2] = -x / (z * z);
  J_p[3] = 0.0;
  J_p[4] = 1.0 / z;
  J_p[5] = -y / (z * z);

  // Distortion Point Jacobian
  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t k3 = params[6];
  const real_t k4 = params[7];
  const real_t d[4] = {k1, k2, k3, k4};
  real_t J_d[2 * 2] = {0};
  equi4_point_jacobian(d, p, J_d);

  // Project Point Jacobian
  real_t J_k[2 * 3] = {0};
  pinhole_point_jacobian(params, J_k);

  // J = J_k * J_d * J_p;
  real_t J_dp[2 * 3] = {0};
  dot(J_d, 2, 2, J_p, 2, 3, J_dp);
  dot(J_k, 2, 2, J_dp, 2, 3, J);
}

void pinhole_equi4_params_jacobian(const real_t params[8],
                                   const real_t p_C[3],
                                   real_t J[2 * 8]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t k[4] = {fx, fy, cx, cy};

  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};

  // Project
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  // Distort
  real_t p_d[2] = {0};
  equi4_distort(d, p, p_d);

  // Project params Jacobian: J_proj_params
  real_t J_proj_params[2 * 4] = {0};
  pinhole_params_jacobian(k, p_d, J_proj_params);

  // Project point Jacobian: J_proj_point
  real_t J_proj_point[2 * 2] = {0};
  pinhole_point_jacobian(k, J_proj_point);

  // Distortion point Jacobian: J_dist_params
  real_t J_dist_params[2 * 4] = {0};
  equi4_params_jacobian(d, p, J_dist_params);

  // Radtan4 params Jacobian: J_equi4
  real_t J_equi4[2 * 4] = {0};
  dot(J_proj_point, 2, 2, J_dist_params, 2, 4, J_equi4);

  // J = [J_proj_params, J_proj_point * J_dist_params]
  J[0] = J_proj_params[0];
  J[1] = J_proj_params[1];
  J[2] = J_proj_params[2];
  J[3] = J_proj_params[3];

  J[8] = J_proj_params[4];
  J[9] = J_proj_params[5];
  J[10] = J_proj_params[6];
  J[11] = J_proj_params[7];

  J[4] = J_equi4[0];
  J[5] = J_equi4[1];
  J[6] = J_equi4[2];
  J[7] = J_equi4[3];

  J[12] = J_equi4[4];
  J[13] = J_equi4[5];
  J[14] = J_equi4[6];
  J[15] = J_equi4[7];
}

/******************************************************************************
 * SENSOR FUSION
 ******************************************************************************/

// POSE //////////////////////////////////////////////////////////////////////

void pose_setup(pose_t *pose, const timestamp_t ts, const real_t *data) {
  assert(pose != NULL);
  assert(data != NULL);

  // Timestamp
  pose->ts = ts;

  // Translation
  pose->data[0] = data[0]; // rx
  pose->data[1] = data[1]; // ry
  pose->data[2] = data[2]; // rz

  // Rotation (Quaternion)
  pose->data[3] = data[3]; // qw
  pose->data[4] = data[4]; // qx
  pose->data[5] = data[5]; // qy
  pose->data[6] = data[6]; // qz
}

/**
 * Print pose
 */
void pose_print(const char *prefix, const pose_t *pose) {
  const timestamp_t ts = pose->ts;

  const real_t x = pose->data[0];
  const real_t y = pose->data[1];
  const real_t z = pose->data[2];

  const real_t qw = pose->data[3];
  const real_t qx = pose->data[4];
  const real_t qy = pose->data[5];
  const real_t qz = pose->data[6];

  printf("[%s] ", prefix);
  printf("ts: %19ld, ", ts);
  printf("pos: (%f, %f, %f), ", x, y, z);
  printf("quat: (%f, %f, %f, %f)\n", qw, qx, qy, qz);
}

// VELOCITY //////////////////////////////////////////////////////////////////

/**
 * Setup velocity
 */
void velocity_setup(velocity_t *vel, const timestamp_t ts, const real_t v[3]) {
  assert(vel != NULL);
  assert(v != NULL);

  // Timestamp
  vel->ts = ts;

  // Accel biases
  vel->v[0] = v[0];
  vel->v[1] = v[1];
  vel->v[2] = v[2];
}

void velocity_print(const velocity_t *vel);

// IMU BIASES /////////////////////////////////////////////////////////////////

/**
 * Setup speed and biases
 */
void imu_biases_setup(imu_biases_t *sb,
                      const timestamp_t ts,
                      const real_t ba[3],
                      const real_t bg[3]) {
  assert(sb != NULL);
  assert(ba != NULL);
  assert(bg != NULL);

  // Timestamp
  sb->ts = ts;

  // Accel biases
  sb->ba[0] = ba[0];
  sb->ba[1] = ba[1];
  sb->ba[2] = ba[2];

  // Gyro biases
  sb->bg[0] = bg[0];
  sb->bg[1] = bg[1];
  sb->bg[2] = bg[2];
}

// FEATURE /////////////////////////////////////////////////////////////////////

/**
 * Setup feature
 */
void feature_setup(feature_t *feature, const real_t *data) {
  assert(feature != NULL);
  assert(data != NULL);

  feature->data[0] = data[0];
  feature->data[1] = data[1];
  feature->data[2] = data[2];
}

/**
 * Setup features
 */
void features_setup(features_t *features) {
  assert(features);
  features->nb_features = 0;
  for (int i = 0; i < MAX_FEATURES; i++) {
    features->status[i] = 0;
  }
}

/**
 * Check whether feature with `feature_id` exists
 * @returns 1 for yes, 0 for no
 */
int features_exists(const features_t *features, const int feature_id) {
  return features->status[feature_id];
}

/**
 * @returns pointer to feature with `feature_id`
 */
feature_t *features_get(features_t *features, const int feature_id) {
  return &features->data[feature_id];
}

/**
 * Add feature
 * @returns Pointer to feature
 */
feature_t *features_add(features_t *features,
                        const int feature_id,
                        const real_t *param) {
  feature_setup(&features->data[feature_id], param);
  features->status[feature_id] = 1;
  return &features->data[feature_id];
}

/**
 * Remove feature with `feature_id`
 */
void features_remove(features_t *features, const int feature_id) {
  const real_t param[3] = {0};
  feature_setup(&features->data[feature_id], param);
  features->status[feature_id] = 0;
}

// EXTRINSICS //////////////////////////////////////////////////////////////////

/**
 * Setup extrinsics
 */
void extrinsics_setup(extrinsics_t *exts, const real_t *data) {
  assert(exts != NULL);
  assert(data != NULL);

  // Translation
  exts->data[0] = data[0]; // rx
  exts->data[1] = data[1]; // ry
  exts->data[2] = data[2]; // rz

  // Rotation (Quaternion)
  exts->data[3] = data[3]; // qw
  exts->data[4] = data[4]; // qx
  exts->data[5] = data[5]; // qy
  exts->data[6] = data[6]; // qz
}

/**
 * Print extrinsics
 */
void extrinsics_print(const char *prefix, const extrinsics_t *exts) {
  const real_t x = exts->data[0];
  const real_t y = exts->data[1];
  const real_t z = exts->data[2];

  const real_t qw = exts->data[3];
  const real_t qx = exts->data[4];
  const real_t qy = exts->data[5];
  const real_t qz = exts->data[6];

  printf("[%s] ", prefix);
  printf("pos: (%.2f, %.2f, %.2f), ", x, y, z);
  printf("quat: (%.2f, %.2f, %.2f, %.2f)\n", qw, qx, qy, qz);
}

// JOINT ANGLES ////////////////////////////////////////////////////////////////

/**
 * Joint Angle Setup
 */
void joint_angle_setup(joint_angle_t *joint,
                       const int joint_idx,
                       const real_t theta) {
  assert(joint != NULL);
  joint->joint_idx = joint_idx;
  joint->angle[0] = theta;
}

/**
 * Print Joint Angle
 */
void joint_angle_print(const char *prefix, const joint_angle_t *joint) {
  printf("[%s] ", prefix);
  printf("%f\n", joint->angle[0]);
}

// CAMERA PARAMS ///////////////////////////////////////////////////////////////

/**
 * Setup camera parameters
 */
void camera_params_setup(camera_params_t *camera,
                         const int cam_idx,
                         const int cam_res[2],
                         const char *proj_model,
                         const char *dist_model,
                         const real_t data[8]) {
  assert(camera != NULL);
  assert(cam_res != NULL);
  assert(proj_model != NULL);
  assert(dist_model != NULL);
  assert(data != NULL);

  camera->cam_idx = cam_idx;
  camera->resolution[0] = cam_res[0];
  camera->resolution[1] = cam_res[1];

  string_copy(camera->proj_model, proj_model);
  string_copy(camera->dist_model, dist_model);

  camera->data[0] = data[0];
  camera->data[1] = data[1];
  camera->data[2] = data[2];
  camera->data[3] = data[3];
  camera->data[4] = data[4];
  camera->data[5] = data[5];
  camera->data[6] = data[6];
  camera->data[7] = data[7];
}

/**
 * Print camera parameters
 */
void camera_params_print(const camera_params_t *camera) {
  assert(camera != NULL);

  printf("cam_idx: %d\n", camera->cam_idx);
  printf("cam_res: [%d, %d]\n", camera->resolution[0], camera->resolution[1]);
  printf("proj_model: %s\n", camera->proj_model);
  printf("dist_model: %s\n", camera->dist_model);
  printf("data: [");
  for (int i = 0; i < 8; i++) {
    if ((i + 1) < 8) {
      printf("%f, ", camera->data[i]);
    } else {
      printf("%f", camera->data[i]);
    }
  }
  printf("]\n");
}

// POSE FACTOR /////////////////////////////////////////////////////////////////

int check_factor_jacobian(const void *factor,
                          FACTOR_EVAL_PTR,
                          real_t **params,
                          real_t **jacobians,
                          const int r_size,
                          const int param_size,
                          const int param_idx,
                          const real_t step_size,
                          const real_t tol,
                          const int verbose) {
  // Form jacobian name
  char J_name[10] = {0};
  if (snprintf(J_name, 10, "J%d", param_idx) <= 0) {
    return -1;
  }

  // Setup
  real_t *r = CALLOC(real_t, r_size);
  real_t *J_numdiff = CALLOC(real_t, r_size * param_size);

  // Evaluate factor
  if (factor_eval(factor, params, r, NULL) != 0) {
    return -2;
  }

  // Numerical diff - forward finite difference
  for (int i = 0; i < param_size; i++) {
    real_t *r_fwd = CALLOC(real_t, r_size);
    real_t *r_diff = CALLOC(real_t, r_size);

    params[param_idx][i] += step_size;
    factor_eval(factor, params, r_fwd, NULL);
    params[param_idx][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, r_size);
    vec_scale(r_diff, r_size, 1.0 / step_size);
    mat_col_set(J_numdiff, param_size, r_size, i, r_diff);

    free(r_fwd);
    free(r_diff);
  }

  // Check jacobian
  const int retval = check_jacobian(J_name,
                                    J_numdiff,
                                    jacobians[param_idx],
                                    r_size,
                                    param_size,
                                    tol,
                                    verbose);
  free(r);
  free(J_numdiff);

  return retval;
}

int check_factor_so3_jacobian(const void *factor,
                              FACTOR_EVAL_PTR,
                              real_t **params,
                              real_t **jacobians,
                              const int r_size,
                              const int param_idx,
                              const real_t step_size,
                              const real_t tol,
                              const int verbose) {
  // Form jacobian name
  char J_name[10] = {0};
  if (snprintf(J_name, 10, "J%d", param_idx) <= 0) {
    return -1;
  }

  // Setup
  const int param_size = 3;
  real_t *r = CALLOC(real_t, r_size);
  real_t *J_numdiff = CALLOC(real_t, r_size * param_size);

  // Evaluate factor
  if (factor_eval(factor, params, r, NULL) != 0) {
    return -2;
  }

  for (int i = 0; i < param_size; i++) {
    real_t *r_fwd = CALLOC(real_t, r_size);
    real_t *r_diff = CALLOC(real_t, r_size);

    quat_perturb(params[param_idx], i, step_size);
    factor_eval(factor, params, r_fwd, NULL);
    quat_perturb(params[param_idx], i, -step_size);

    vec_sub(r_fwd, r, r_diff, r_size);
    vec_scale(r_diff, r_size, 1.0 / step_size);
    mat_col_set(J_numdiff, param_size, r_size, i, r_diff);

    free(r_fwd);
    free(r_diff);
  }

  // Check Jacobian
  const int retval = check_jacobian(J_name,
                                    J_numdiff,
                                    jacobians[param_idx],
                                    r_size,
                                    param_size,
                                    tol,
                                    verbose);
  free(r);
  free(J_numdiff);

  return retval;
}

/**
 * Setup pose factor
 */
void pose_factor_setup(pose_factor_t *factor,
                       pose_t *pose,
                       const real_t var[6]) {
  assert(factor != NULL);
  assert(pose != NULL);
  assert(var != NULL);

  // Parameters
  factor->pose_est = pose;
  factor->num_params = 1;

  // Measurement
  factor->pos_meas[0] = pose->data[0];
  factor->pos_meas[1] = pose->data[1];
  factor->pos_meas[2] = pose->data[2];
  factor->quat_meas[0] = pose->data[3];
  factor->quat_meas[1] = pose->data[4];
  factor->quat_meas[2] = pose->data[5];
  factor->quat_meas[3] = pose->data[6];

  // Measurement covariance matrix
  zeros(factor->covar, 6, 6);
  factor->covar[0] = 1.0 / (var[0] * var[0]);
  factor->covar[7] = 1.0 / (var[1] * var[1]);
  factor->covar[14] = 1.0 / (var[2] * var[2]);
  factor->covar[21] = 1.0 / (var[3] * var[3]);
  factor->covar[28] = 1.0 / (var[4] * var[4]);
  factor->covar[35] = 1.0 / (var[5] * var[5]);

  // Square root information matrix
  zeros(factor->sqrt_info, 6, 6);
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[7] = sqrt(1.0 / factor->covar[7]);
  factor->sqrt_info[14] = sqrt(1.0 / factor->covar[14]);
  factor->sqrt_info[21] = sqrt(1.0 / factor->covar[21]);
  factor->sqrt_info[28] = sqrt(1.0 / factor->covar[28]);
  factor->sqrt_info[35] = sqrt(1.0 / factor->covar[35]);
}

/**
 * Evaluate pose factor
 *
 * Parameter `params`:
 * 1. Position r_est
 * 2. Quaternion q_est
 *
 * Residuals `r_out` represents the pose error [6x1]
 * Jacobians `J_out` with respect to `params`
 *
 * @returns `0` for success, `-1` for failure
 */
int pose_factor_eval(const void *factor,
                     real_t **params,
                     real_t *r_out,
                     real_t **J_out) {
  assert(factor != NULL);
  const pose_factor_t *pose_factor = (const pose_factor_t *) factor;

  // Map params
  const real_t r_est[3] = {params[0][0], params[0][1], params[0][2]};
  const real_t q_est[4] = {params[0][3],
                           params[0][4],
                           params[0][5],
                           params[0][6]};
  const real_t *r_meas = pose_factor->pos_meas;
  const real_t *q_meas = pose_factor->quat_meas;

  // Calculate pose error
  // -- Translation error
  // dr = r_meas - r_est;
  real_t dr[3] = {0};
  dr[0] = r_meas[0] - r_est[0];
  dr[1] = r_meas[1] - r_est[1];
  dr[2] = r_meas[2] - r_est[2];

  // -- Rotation error
  // dq = quat_mul(quat_inv(q_meas), q_est);
  real_t dq[4] = {0};
  real_t q_meas_inv[4] = {0};
  quat_inv(q_meas, q_meas_inv);
  quat_mul(q_meas_inv, q_est, dq);

  // dtheta = 2 * dq;
  real_t dtheta[3] = {0};
  dtheta[0] = 2 * dq[1];
  dtheta[1] = 2 * dq[2];
  dtheta[2] = 2 * dq[3];

  // -- Set residuals
  // r = factor.sqrt_info * [dr; dtheta];
  real_t r[6] = {0};
  r[0] = dr[0];
  r[1] = dr[1];
  r[2] = dr[2];
  r[3] = dtheta[0];
  r[4] = dtheta[1];
  r[5] = dtheta[2];
  dot(pose_factor->sqrt_info, 6, 6, r, 6, 1, r_out);

  // Calculate Jacobians
  if (J_out == NULL) {
    return 0;
  }

  if (J_out[0]) {
    const real_t dqw = dq[0];
    const real_t dqx = dq[1];
    const real_t dqy = dq[2];
    const real_t dqz = dq[3];

    real_t J[6 * 6] = {0};

    J[0] = -1.0;
    J[1] = 0.0;
    J[2] = 0.0;
    J[6] = 0.0;
    J[7] = -1.0;
    J[8] = 0.0;
    J[12] = 0.0;
    J[13] = 0.0;
    J[14] = -1.0;

    J[21] = dqw;
    J[22] = -dqz;
    J[23] = dqy;
    J[27] = dqz;
    J[28] = dqw;
    J[29] = -dqx;
    J[33] = -dqy;
    J[34] = dqx;
    J[35] = dqw;

    dot(pose_factor->sqrt_info, 6, 6, J, 6, 6, J_out[0]);
  }

  return 0;
}

// BA FACTOR ///////////////////////////////////////////////////////////////////

/**
 * Setup bundle adjustment factor
 */
void ba_factor_setup(ba_factor_t *factor,
                     const pose_t *pose,
                     const feature_t *feature,
                     const camera_params_t *camera,
                     const real_t z[2],
                     const real_t var[2]) {
  assert(factor != NULL);
  assert(pose != NULL);
  assert(feature != NULL);
  assert(camera != NULL);
  assert(var != NULL);

  // Parameters
  factor->pose = pose;
  factor->feature = feature;
  factor->camera = camera;
  factor->num_params = 3;

  // Measurement covariance
  factor->covar[0] = 1.0 / (var[0] * var[0]);
  factor->covar[1] = 0.0;
  factor->covar[2] = 0.0;
  factor->covar[3] = 1.0 / (var[1] * var[1]);

  // Square-root information matrix
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[1] = 0.0;
  factor->sqrt_info[2] = 0.0;
  factor->sqrt_info[3] = sqrt(1.0 / factor->covar[3]);

  // Measurement
  factor->z[0] = z[0];
  factor->z[1] = z[1];
}

/**
 * Camera pose jacobian
 */
static void ba_factor_pose_jacobian(const real_t Jh_weighted[2 * 3],
                                    const real_t T_WC[4 * 4],
                                    const real_t p_W[3],
                                    real_t *J) {
  // Pre-check
  if (J == NULL) {
    return;
  }

  // Jh_weighted = -1 * sqrt_info * Jh;
  // J_pos = Jh_weighted * -C_CW;
  // J_rot = Jh_weighted * -C_CW * hat(p_W - r_WC) * -C_WC;
  // J = [J_pos, J_rot]

  // Setup
  real_t C_WC[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};
  real_t r_WC[3] = {0};
  tf_rot_get(T_WC, C_WC);
  tf_trans_get(T_WC, r_WC);
  mat_transpose(C_WC, 3, 3, C_CW);

  // J_pos = -1 * sqrt_info * Jh * -C_CW;
  real_t J_pos[2 * 3] = {0};
  real_t neg_C_CW[3 * 3] = {0};
  mat_copy(C_CW, 3, 3, neg_C_CW);
  mat_scale(neg_C_CW, 3, 3, -1.0);
  dot(Jh_weighted, 2, 3, neg_C_CW, 3, 3, J_pos);

  J[0] = J_pos[0];
  J[1] = J_pos[1];
  J[2] = J_pos[2];

  J[6] = J_pos[3];
  J[7] = J_pos[4];
  J[8] = J_pos[5];

  /**
   * Jh_weighted = -1 * sqrt_info * Jh;
   * J_rot = Jh_weighted * -C_CW * hat(p_W - r_WC) * -C_WC;
   * where:
   *
   *   A = -C_CW;
   *   B = hat(p_W - r_WC);
   *   C = -C_WC;
   */
  real_t J_rot[2 * 3] = {0};
  real_t A[3 * 3] = {0};
  mat_copy(neg_C_CW, 3, 3, A);

  real_t B[3 * 3] = {0};
  real_t dp[3] = {0};
  dp[0] = p_W[0] - r_WC[0];
  dp[1] = p_W[1] - r_WC[1];
  dp[2] = p_W[2] - r_WC[2];
  hat(dp, B);

  real_t C[3 * 3] = {0};
  mat_copy(C_WC, 3, 3, C);
  mat_scale(C, 3, 3, -1.0);

  real_t AB[3 * 3] = {0};
  real_t ABC[3 * 3] = {0};
  dot(A, 3, 3, B, 3, 3, AB);
  dot(AB, 3, 3, C, 3, 3, ABC);
  dot(Jh_weighted, 2, 3, ABC, 3, 3, J_rot);

  J[3] = J_rot[0];
  J[4] = J_rot[1];
  J[5] = J_rot[2];

  J[9] = J_rot[3];
  J[10] = J_rot[4];
  J[11] = J_rot[5];
}

/**
 * Feature jacobian
 */
static void ba_factor_feature_jacobian(const real_t Jh_weighted[2 * 3],
                                       const real_t T_WC[4 * 4],
                                       real_t *J) {
  // Pre-check
  if (J == NULL) {
    return;
  }

  // Jh_weighted = -1 * sqrt_info * Jh;
  // J = Jh_weighted * C_CW;
  real_t C_WC[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};
  tf_rot_get(T_WC, C_WC);
  mat_transpose(C_WC, 3, 3, C_CW);
  dot(Jh_weighted, 2, 3, C_CW, 3, 3, J);
}

/**
 * Camera parameters jacobian
 */
static void ba_factor_camera_jacobian(const real_t neg_sqrt_info[2 * 2],
                                      const real_t J_cam_params[2 * 8],
                                      real_t *J) {
  // Pre-check
  if (J == NULL) {
    return;
  }

  // J = -1 * sqrt_info * J_cam_params;
  dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, J);
}

/**
 * Evaluate bundle adjustment factor
 *
 * Parameter `params`:
 * 1. Position r_WCi
 * 2. Quaternion q_WCi
 * 3. Feature p_W
 * 4. Camera i parameters
 *
 * Residuals `r_out` represents the reprojection error [2x1]
 * Jacobians `J_out` with respect to `params`
 *
 * @returns `0` for success, `-1` for failure
 */
int ba_factor_eval(ba_factor_t *factor,
                   real_t **params,
                   real_t *r_out,
                   real_t **J_out) {
  assert(factor != NULL);
  assert(params != NULL);
  assert(r_out != NULL);

  // Map params
  real_t T_WCi[4 * 4] = {0};
  tf(params[0], T_WCi);
  const real_t *p_W = params[1];
  const real_t *cam_params = params[2];

  // Calculate residuals
  // -- Project point from world to image plane
  real_t T_CiW[4 * 4] = {0};
  real_t p_Ci[3] = {0};
  real_t z_hat[2];
  tf_inv(T_WCi, T_CiW);
  tf_point(T_CiW, p_W, p_Ci);
  pinhole_radtan4_project(cam_params, p_Ci, z_hat);
  // -- Residual
  real_t r[2] = {0};
  r[0] = factor->z[0] - z_hat[0];
  r[1] = factor->z[1] - z_hat[1];
  // -- Weighted residual
  dot(factor->sqrt_info, 2, 2, r, 2, 1, r_out);

  // Calculate jacobians
  if (J_out == NULL) {
    return 0;
  }
  // -- Form: -1 * sqrt_info
  real_t neg_sqrt_info[2 * 2] = {0};
  mat_copy(factor->sqrt_info, 2, 2, neg_sqrt_info);
  mat_scale(neg_sqrt_info, 2, 2, -1.0);
  // -- Form: Jh_weighted = -1 * sqrt_info * Jh
  real_t Jh[2 * 3] = {0};
  real_t Jh_w[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(cam_params, p_Ci, Jh);
  dot(neg_sqrt_info, 2, 2, Jh, 2, 3, Jh_w);
  // -- Form: J_cam_params
  real_t J_cam_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(cam_params, p_Ci, J_cam_params);
  // -- Fill jacobians
  ba_factor_pose_jacobian(Jh_w, T_WCi, p_W, J_out[0]);
  ba_factor_feature_jacobian(Jh_w, T_WCi, J_out[1]);
  ba_factor_camera_jacobian(neg_sqrt_info, J_cam_params, J_out[2]);

  return 0;
}

// CAMERA FACTOR ///////////////////////////////////////////////////////////////

/**
 * Setup vision factor
 */
void vision_factor_setup(vision_factor_t *factor,
                         const pose_t *pose,
                         const extrinsics_t *extrinsics,
                         const feature_t *feature,
                         const camera_params_t *camera,
                         const real_t z[2],
                         const real_t var[2]) {
  assert(factor != NULL);
  assert(pose != NULL);
  assert(extrinsics != NULL);
  assert(feature != NULL);
  assert(camera != NULL);
  assert(z != NULL);
  assert(var != NULL);

  // Parameters
  factor->pose = pose;
  factor->extrinsics = extrinsics;
  factor->feature = feature;
  factor->camera = camera;
  factor->num_params = 4;

  // Measurement covariance matrix
  factor->covar[0] = 1.0 / (var[0] * var[0]);
  factor->covar[1] = 0.0;
  factor->covar[2] = 0.0;
  factor->covar[3] = 1.0 / (var[1] * var[1]);

  // Square-root information matrix
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[1] = 0.0;
  factor->sqrt_info[2] = 0.0;
  factor->sqrt_info[3] = sqrt(1.0 / factor->covar[3]);

  // Measurement
  factor->z[0] = z[0];
  factor->z[1] = z[1];
}

/**
 * Pose jacobian
 */
static void vision_factor_pose_jacobian(const real_t Jh_weighted[2 * 3],
                                        const real_t T_WB[3 * 3],
                                        const real_t T_BC[3 * 3],
                                        const real_t p_W[3],
                                        real_t J[2 * 6]) {
  assert(Jh_weighted != NULL);
  assert(T_BC != NULL);
  assert(T_WB != NULL);
  assert(p_W != NULL);
  assert(J != NULL);

  // Jh_weighted = -1 * sqrt_info * Jh;
  // J_pos = Jh_weighted * C_CB * -C_BW;
  // J_rot = Jh_weighted * C_CB * C_BW * hat(p_W - r_WB) * -C_WB;
  // J = [J_pos, J_rot];

  // Setup
  real_t C_WB[3 * 3] = {0};
  real_t C_BC[3 * 3] = {0};
  real_t C_BW[3 * 3] = {0};
  real_t C_CB[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};

  tf_rot_get(T_WB, C_WB);
  tf_rot_get(T_BC, C_BC);
  mat_transpose(C_WB, 3, 3, C_BW);
  mat_transpose(C_BC, 3, 3, C_CB);
  dot(C_CB, 3, 3, C_BW, 3, 3, C_CW);

  // Form: -C_BW
  real_t neg_C_BW[3 * 3] = {0};
  mat_copy(C_BW, 3, 3, neg_C_BW);
  mat_scale(neg_C_BW, 3, 3, -1.0);

  // Form: -C_CW
  real_t neg_C_CW[3 * 3] = {0};
  dot(C_CB, 3, 3, neg_C_BW, 3, 3, neg_C_CW);

  // Form: -C_WB
  real_t neg_C_WB[3 * 3] = {0};
  mat_copy(C_WB, 3, 3, neg_C_WB);
  mat_scale(neg_C_WB, 3, 3, -1.0);

  // Form: C_CB * -C_BW * hat(p_W - r_WB) * -C_WB
  real_t r_WB[3] = {0};
  real_t p[3] = {0};
  real_t S[3 * 3] = {0};
  tf_trans_get(T_WB, r_WB);
  vec_sub(p_W, r_WB, p, 3);
  hat(p, S);

  real_t A[3 * 3] = {0};
  real_t B[3 * 3] = {0};
  dot(neg_C_CW, 3, 3, S, 3, 3, A);
  dot(A, 3, 3, neg_C_WB, 3, 3, B);

  // Form: J_pos = Jh_weighted * C_CB * -C_BW;
  real_t J_pos[2 * 3] = {0};
  dot(Jh_weighted, 2, 3, neg_C_CW, 3, 3, J_pos);

  J[0] = J_pos[0];
  J[1] = J_pos[1];
  J[2] = J_pos[2];

  J[6] = J_pos[3];
  J[7] = J_pos[4];
  J[8] = J_pos[5];

  // Form: J_rot = Jh_weighted * C_CB * -C_BW * hat(p_W - r_WB) * -C_WB;
  real_t J_rot[2 * 3] = {0};
  dot(Jh_weighted, 2, 3, B, 3, 3, J_rot);

  J[3] = J_rot[0];
  J[4] = J_rot[1];
  J[5] = J_rot[2];

  J[9] = J_rot[3];
  J[10] = J_rot[4];
  J[11] = J_rot[5];
}

/**
 * Body-camera extrinsics jacobian
 */
static void vision_factor_extrinsics_jacobian(const real_t Jh_weighted[2 * 3],
                                              const real_t T_BC[3 * 3],
                                              const real_t p_C[3],
                                              real_t J[2 * 6]) {
  assert(Jh_weighted != NULL);
  assert(T_BC != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  // Jh_weighted = -1 * sqrt_info * Jh;
  // J_pos = Jh_weighted * -C_CB;
  // J_rot = Jh_weighted * C_CB * hat(C_BC * p_C);

  // Setup
  real_t C_BC[3 * 3] = {0};
  real_t C_CB[3 * 3] = {0};
  real_t C_BW[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};

  tf_rot_get(T_BC, C_BC);
  mat_transpose(C_BC, 3, 3, C_CB);
  dot(C_CB, 3, 3, C_BW, 3, 3, C_CW);

  // Form: -C_CB
  real_t neg_C_CB[3 * 3] = {0};
  mat_copy(C_CB, 3, 3, neg_C_CB);
  mat_scale(neg_C_CB, 3, 3, -1.0);

  // Form: -C_BC
  real_t neg_C_BC[3 * 3] = {0};
  mat_copy(C_BC, 3, 3, neg_C_BC);
  mat_scale(neg_C_BC, 3, 3, -1.0);

  // Form: -C_CB * hat(C_BC * p_C) * -C_BC
  real_t p[3] = {0};
  real_t S[3 * 3] = {0};
  dot(C_BC, 3, 3, p_C, 3, 1, p);
  hat(p, S);

  real_t A[3 * 3] = {0};
  real_t B[3 * 3] = {0};
  dot(neg_C_CB, 3, 3, S, 3, 3, A);
  dot(A, 3, 3, neg_C_BC, 3, 3, B);

  // Form: J_rot = Jh_weighted * -C_CB;
  real_t J_pos[2 * 3] = {0};
  dot(Jh_weighted, 2, 3, neg_C_CB, 3, 3, J_pos);

  J[0] = J_pos[0];
  J[1] = J_pos[1];
  J[2] = J_pos[2];

  J[6] = J_pos[3];
  J[7] = J_pos[4];
  J[8] = J_pos[5];

  // Form: J_rot = Jh_weighted * -C_CB * hat(C_BC * p_C) * -C_BC;
  real_t J_rot[2 * 3] = {0};
  dot(Jh_weighted, 2, 3, B, 3, 3, J_rot);

  J[3] = J_rot[0];
  J[4] = J_rot[1];
  J[5] = J_rot[2];

  J[9] = J_rot[3];
  J[10] = J_rot[4];
  J[11] = J_rot[5];
}

/**
 * Camera parameters jacobian
 */
static void vision_factor_camera_jacobian(const real_t neg_sqrt_info[2 * 2],
                                          const real_t J_cam_params[2 * 8],
                                          real_t J[2 * 8]) {
  assert(neg_sqrt_info != NULL);
  assert(J_cam_params != NULL);
  assert(J != NULL);

  // J = -1 * sqrt_info * J_cam_params;
  dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, J);
}

/**
 * Feature jacobian
 */
static void vision_factor_feature_jacobian(const real_t Jh_weighted[2 * 3],
                                           const real_t T_WB[4 * 4],
                                           const real_t T_BC[4 * 4],
                                           real_t J[2 * 3]) {
  if (J == NULL) {
    return;
  }
  assert(Jh_weighted != NULL);
  assert(T_WB != NULL);
  assert(T_BC != NULL);
  assert(J != NULL);

  // Jh_weighted = -1 * sqrt_info * Jh;
  // J = Jh_weighted * C_CW;

  // Setup
  real_t T_WC[4 * 4] = {0};
  real_t C_WC[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};
  dot(T_WB, 4, 4, T_BC, 4, 4, T_WC);
  tf_rot_get(T_WC, C_WC);
  mat_transpose(C_WC, 3, 3, C_CW);

  // Form: J = -1 * sqrt_info * Jh * C_CW;
  dot(Jh_weighted, 2, 3, C_CW, 3, 3, J);
}

/**
 * Evaluate vision factor
 *
 * Parameter `params`:
 * 1. Position r_WB
 * 2. Quaternion q_WB
 * 3. Body-Camera i Extrinsics - Translation r_BCi
 * 4. Body-Camera i Extrinsics - Quaternion q_BCi
 * 5. Camera i parameters
 * 6. Feature p_W
 *
 * Residuals `r_out` camera reprojection error
 * Jacobians `J_out` with respect to `params`
 *
 * @returns `0` for success, `-1` for failure
 */
int vision_factor_eval(vision_factor_t *factor,
                       real_t **params,
                       real_t *r_out,
                       real_t **J_out) {
  assert(factor != NULL);
  assert(params != NULL);
  assert(r_out != NULL);
  assert(factor->pose);
  assert(factor->extrinsics);
  assert(factor->feature);
  assert(factor->camera);

  // Map params
  real_t T_WB[4 * 4] = {0};
  real_t T_BCi[4 * 4] = {0};
  tf(params[0], T_WB);
  tf(params[1], T_BCi);
  const real_t *cam_params = params[2];
  const real_t *p_W = params[3];

  // Form camera pose
  real_t T_WCi[4 * 4] = {0};
  real_t T_CiW[4 * 4] = {0};
  dot(T_WB, 4, 4, T_BCi, 4, 4, T_WCi);
  tf_inv(T_WCi, T_CiW);

  // Transform feature from world to camera frame
  real_t p_Ci[3] = {0};
  tf_point(T_CiW, p_W, p_Ci);

  // Calculate residuals
  // -- Project point from world to image plane
  real_t z_hat[2];
  pinhole_radtan4_project(cam_params, p_Ci, z_hat);
  // -- Residual
  real_t r[2] = {0};
  r[0] = factor->z[0] - z_hat[0];
  r[1] = factor->z[1] - z_hat[1];
  // -- Weighted residual
  dot(factor->sqrt_info, 2, 2, r, 2, 1, r_out);

  // Calculate jacobians
  if (J_out == NULL) {
    return 0;
  }
  // -- Form: -1 * sqrt_info
  real_t neg_sqrt_info[2 * 2] = {0};
  mat_copy(factor->sqrt_info, 2, 2, neg_sqrt_info);
  mat_scale(neg_sqrt_info, 2, 2, -1.0);
  // -- Form: Jh_ = -1 * sqrt_info * Jh
  real_t Jh[2 * 3] = {0};
  real_t Jh_[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(cam_params, p_Ci, Jh);
  dot(neg_sqrt_info, 2, 2, Jh, 2, 3, Jh_);
  // -- Form: J_cam_params
  real_t J_cam_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(cam_params, p_Ci, J_cam_params);
  // -- Fill Jacobians
  vision_factor_pose_jacobian(Jh_, T_WB, T_BCi, p_W, J_out[0]);
  vision_factor_extrinsics_jacobian(Jh_, T_BCi, p_Ci, J_out[1]);
  vision_factor_camera_jacobian(neg_sqrt_info, J_cam_params, J_out[2]);
  vision_factor_feature_jacobian(Jh_, T_WB, T_BCi, J_out[3]);

  return 0;
}

// CALIB GIMBAL FACTOR /////////////////////////////////////////////////////////

void gimbal_setup_extrinsics(const real_t ypr[3],
                             const real_t r[3],
                             real_t T[4 * 4],
                             extrinsics_t *link) {
  real_t C[3 * 3] = {0};
  euler321(ypr, C);
  tf_cr(C, r, T);

  real_t q[4] = {0};
  rot2quat(C, q);

  real_t x[7] = {0};
  x[0] = r[0];
  x[1] = r[1];
  x[2] = r[2];
  x[3] = q[0];
  x[4] = q[1];
  x[5] = q[2];
  x[6] = q[3];

  extrinsics_setup(link, x);
}

void gimbal_setup_joint(const int joint_idx,
                        const real_t theta,
                        real_t T_joint[4 * 4],
                        joint_angle_t *joint) {
  real_t C_joint[3 * 3] = {0};
  rotz(theta, C_joint);

  const real_t r_joint[3] = {0.0, 0.0, 0.0};
  tf_cr(C_joint, r_joint, T_joint);

  joint_angle_setup(joint, joint_idx, theta);
}

void calib_gimbal_factor_setup(calib_gimbal_factor_t *factor,
                               const extrinsics_t *fiducial,
                               const extrinsics_t *link0,
                               const extrinsics_t *link1,
                               const extrinsics_t *link2,
                               const joint_angle_t *joint0,
                               const joint_angle_t *joint1,
                               const joint_angle_t *joint2,
                               const extrinsics_t *cam_exts,
                               const camera_params_t *cam,
                               const int tag_id,
                               const int corner_idx,
                               const real_t p_FFi[3],
                               const real_t z[2],
                               const real_t var[2]) {
  assert(factor != NULL);
  assert(fiducial != NULL);
  assert(link0 != NULL && link1 != NULL && link2 != NULL);
  assert(joint0 != NULL && joint1 != NULL && joint2 != NULL);
  assert(cam_exts != NULL);
  assert(cam != NULL);
  assert(z != NULL);
  assert(var != NULL);

  // Parameters
  factor->fiducial = fiducial;
  factor->link0 = link0;
  factor->link1 = link1;
  factor->link2 = link2;
  factor->joint0 = joint0;
  factor->joint1 = joint1;
  factor->joint2 = joint2;
  factor->cam_exts = cam_exts;
  factor->cam = cam;
  factor->num_params = 9;

  // Measurement
  factor->tag_id = tag_id;
  factor->corner_idx = corner_idx;
  factor->p_FFi[0] = p_FFi[0];
  factor->p_FFi[1] = p_FFi[1];
  factor->p_FFi[2] = p_FFi[2];
  factor->z[0] = z[0];
  factor->z[1] = z[1];

  // Measurement covariance matrix
  factor->covar[0] = 1.0 / (var[0] * var[0]);
  factor->covar[1] = 0.0;
  factor->covar[2] = 0.0;
  factor->covar[3] = 1.0 / (var[1] * var[1]);

  // Square-root information matrix
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[1] = 0.0;
  factor->sqrt_info[2] = 0.0;
  factor->sqrt_info[3] = sqrt(1.0 / factor->covar[3]);
}

static void gimbal_factor_joint_tf(const real_t theta, real_t T[4 * 4]) {
  real_t C[3 * 3] = {0};
  real_t r[3] = {0.0, 0.0, 0.0};
  rotz(theta, C);
  tf_cr(C, r, T);
}

static void gimbal_factor_fiducial_jac(const real_t Jh_w[2 * 3],
                                       const real_t T_CiB[4 * 4],
                                       const real_t T_BF[4 * 4],
                                       const real_t p_FFi[3],
                                       real_t J[2 * 6]) {
  // J_pos = Jh * C_CiB
  real_t J_pos[2 * 3] = {0};
  real_t C_CiB[3 * 3] = {0};
  tf_rot_get(T_CiB, C_CiB);
  dot(Jh_w, 2, 3, C_CiB, 3, 3, J_pos);

  J[0] = J_pos[0];
  J[1] = J_pos[1];
  J[2] = J_pos[2];

  J[6] = J_pos[3];
  J[7] = J_pos[4];
  J[8] = J_pos[5];

  // J_rot = Jh * C_CiB @ -C_BF @ hat(self.p_FFi)
  real_t J_rot[2 * 3] = {0};
  real_t C_BF[3 * 3] = {0};
  real_t C_CiF[3 * 3] = {0};
  tf_rot_get(T_BF, C_BF);
  dot(C_CiB, 3, 3, C_BF, 3, 3, C_CiF);
  mat_scale(C_CiF, 3, 3, -1);

  real_t p_FFi_x[3 * 3] = {0};
  hat(p_FFi, p_FFi_x);
  dot3(Jh_w, 2, 3, C_CiF, 3, 3, p_FFi_x, 3, 3, J_rot);

  J[3] = J_rot[0];
  J[4] = J_rot[1];
  J[5] = J_rot[2];

  J[9] = J_rot[3];
  J[10] = J_rot[4];
  J[11] = J_rot[5];
}

static void gimbal_factor_link_jac(const real_t Jh_w[2 * 3],
                                   const real_t T_LaLb[4 * 4],
                                   const real_t T_CiLa[4 * 4],
                                   const real_t p_La[3],
                                   real_t J[2 * 6]) {
  // Form: -C_LaLb
  real_t nC_LaLb[3 * 3] = {0};
  tf_rot_get(T_LaLb, nC_LaLb);
  mat_scale(nC_LaLb, 3, 3, -1.0);

  // J_pos = Jh * -C_CiLa.T
  real_t J_pos[2 * 3] = {0};
  real_t nC_CiLa[3 * 3] = {0};
  tf_rot_get(T_CiLa, nC_CiLa);
  mat_scale(nC_CiLa, 3, 3, -1.0);
  dot(Jh_w, 2, 3, nC_CiLa, 3, 3, J_pos);

  J[0] = J_pos[0];
  J[1] = J_pos[1];
  J[2] = J_pos[2];

  J[6] = J_pos[3];
  J[7] = J_pos[4];
  J[8] = J_pos[5];

  // J_rot = Jh * -C_CiLa.T * hat(p_BFi - r_BM0b) * -C_BM0b
  real_t J_rot[2 * 3] = {0};
  real_t r_LaLb[3] = {0};
  real_t dp[3] = {0};
  real_t dp_x[3 * 3] = {0};
  tf_trans_get(T_LaLb, r_LaLb);
  dp[0] = p_La[0] - r_LaLb[0];
  dp[1] = p_La[1] - r_LaLb[1];
  dp[2] = p_La[2] - r_LaLb[2];
  hat(dp, dp_x);
  dot3(J_pos, 2, 3, dp_x, 3, 3, nC_LaLb, 3, 3, J_rot);

  J[3] = J_rot[0];
  J[4] = J_rot[1];
  J[5] = J_rot[2];

  J[9] = J_rot[3];
  J[10] = J_rot[4];
  J[11] = J_rot[5];
}

static void gimbal_factor_joint_jac(const real_t Jh_w[2 * 3],
                                    const real_t T_CiMe[4 * 4],
                                    const real_t p_MbFi[3],
                                    const real_t theta,
                                    real_t J_joint[2 * 1]) {
  assert(Jh_w != NULL);
  assert(T_CiMe != NULL);
  assert(p_MbFi != NULL);
  assert(J_joint != NULL);

  // C_CiMe = tf_rot(T_CiMe)
  real_t C_CiMe[3 * 3] = {0};
  tf_rot_get(T_CiMe, C_CiMe);

  // p = [-p_M0bFi[0] * sin(joints[0]) + p_M0bFi[1] * cos(joints[0]),
  //      -p_M0bFi[0] * cos(joints[0]) - p_M0bFi[1] * sin(joints[0]),
  //      0.0]
  // J_joint = Jh * C_CiMe @ p
  const real_t p[3] = {-p_MbFi[0] * sin(theta) + p_MbFi[1] * cos(theta),
                       -p_MbFi[0] * cos(theta) - p_MbFi[1] * sin(theta),
                       0.0};
  dot3(Jh_w, 2, 3, C_CiMe, 3, 3, p, 3, 1, J_joint);
}

static void gimbal_factor_cam_ext_jac(const real_t Jh_w[2 * 3],
                                      const real_t T_M2eCi[4 * 4],
                                      const real_t p_M2eFi[3],
                                      real_t J[2 * 6]) {
  assert(Jh_w != NULL);
  assert(T_M2eCi != NULL);
  assert(p_M2eFi != NULL);
  assert(J != NULL);

  // Form: -C_CiM2e
  real_t nC_M2eCi[3 * 3] = {0};
  real_t nC_CiM2e[3 * 3] = {0};
  tf_rot_get(T_M2eCi, nC_M2eCi);
  mat_scale(nC_M2eCi, 3, 3, -1.0);
  mat_transpose(nC_M2eCi, 3, 3, nC_CiM2e);

  // J_pos = Jh * -C_M2eCi.T
  real_t J_pos[2 * 6] = {0};
  dot(Jh_w, 2, 3, nC_CiM2e, 3, 3, J_pos);

  J[0] = J_pos[0];
  J[1] = J_pos[1];
  J[2] = J_pos[2];

  J[6] = J_pos[3];
  J[7] = J_pos[4];
  J[8] = J_pos[5];

  // J_rot = Jh * -C_M2eCi.T * hat(p_M2eFi - r_M2eCi) * -C_M2eCi
  real_t J_rot[2 * 6] = {0};
  real_t r_M2eCi[3] = {0};
  real_t dr[3] = {0};
  real_t dr_x[3 * 3] = {0};
  tf_trans_get(T_M2eCi, r_M2eCi);
  dr[0] = p_M2eFi[0] - r_M2eCi[0];
  dr[1] = p_M2eFi[1] - r_M2eCi[1];
  dr[2] = p_M2eFi[2] - r_M2eCi[2];
  hat(dr, dr_x);
  dot3(J_pos, 2, 3, dr_x, 3, 3, nC_M2eCi, 3, 3, J_rot);

  J[3] = J_rot[0];
  J[4] = J_rot[1];
  J[5] = J_rot[2];

  J[9] = J_rot[3];
  J[10] = J_rot[4];
  J[11] = J_rot[5];
}

static void gimbal_factor_camera_jac(const real_t neg_sqrt_info[2 * 2],
                                     const real_t J_cam_params[2 * 8],
                                     real_t J[2 * 8]) {
  assert(neg_sqrt_info != NULL);
  assert(J_cam_params != NULL);
  assert(J != NULL);
  dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, J);
}

/**
 * Evaluate gimbal calibration factor
 *
 * Parameter `params`:
 * 1. Fiducial pose T_BF
 * 2. Link0 pose T_BM0b
 * 3. Link1 pose T_M0eM1b
 * 4. Link2 pose T_M1eM2b
 * 5. Joint0 angle
 * 6. Joint1 angle
 * 7. Joint2 angle
 * 8. Camera extrinsics pose T_M2eCi
 * 9. Camera parameters K_Ci
 *
 * Residuals `r_out` camera reprojection error
 * Jacobians `J_out` with respect to `params`
 *
 * @returns `0` for success, `-1` for failure
 */
int calib_gimbal_factor_eval(calib_gimbal_factor_t *factor,
                             real_t **params,
                             real_t *r_out,
                             real_t **J_out) {
  assert(factor != NULL);
  assert(params != NULL);
  assert(r_out != NULL);

  // Map params
  const real_t *p_FFi = factor->p_FFi;
  // -- Fiducial pose
  real_t T_BF[4 * 4] = {0};
  tf(params[0], T_BF);
  // -- Links
  real_t T_BM0b[4 * 4] = {0};
  real_t T_M0eM1b[4 * 4] = {0};
  real_t T_M1eM2b[4 * 4] = {0};
  tf(params[1], T_BM0b);
  tf(params[2], T_M0eM1b);
  tf(params[3], T_M1eM2b);
  // -- Joint angles
  real_t T_M0bM0e[4 * 4] = {0};
  real_t T_M1bM1e[4 * 4] = {0};
  real_t T_M2bM2e[4 * 4] = {0};
  const real_t th0 = params[4][0];
  const real_t th1 = params[5][0];
  const real_t th2 = params[6][0];
  gimbal_factor_joint_tf(th0, T_M0bM0e);
  gimbal_factor_joint_tf(th1, T_M1bM1e);
  gimbal_factor_joint_tf(th2, T_M2bM2e);
  // -- Camera extrinsics
  real_t T_M2eCi[4 * 4] = {0};
  tf(params[7], T_M2eCi);
  // -- Camera parameters
  const real_t *cam_params = params[8];

  // Form T_CiF
  const real_t *T_chain[7] = {
      T_BM0b,   // Link0
      T_M0bM0e, // Joint0
      T_M0eM1b, // Link1
      T_M1bM1e, // Joint1
      T_M1eM2b, // Link2
      T_M2bM2e, // Joint2
      T_M2eCi,  // Camera extrinsics
  };
  real_t T_BCi[4 * 4] = {0};
  real_t T_CiB[4 * 4] = {0};
  real_t T_CiF[4 * 4] = {0};
  tf_chain(T_chain, 7, T_BCi);
  tf_inv(T_BCi, T_CiB);
  dot(T_CiB, 4, 4, T_BF, 4, 4, T_CiF);

  // Project to image plane
  real_t p_CiFi[3] = {0};
  real_t z_hat[2];
  tf_point(T_CiF, factor->p_FFi, p_CiFi);
  pinhole_radtan4_project(cam_params, p_CiFi, z_hat);

  // Calculate residuals
  real_t r[2] = {0};
  r[0] = factor->z[0] - z_hat[0];
  r[1] = factor->z[1] - z_hat[1];
  dot(factor->sqrt_info, 2, 2, r, 2, 1, r_out);

  // Calculate Jacobians
  if (J_out == NULL) {
    return 0;
  }

  // Form: -1 * sqrt_info
  real_t neg_sqrt_info[2 * 2] = {0};
  mat_copy(factor->sqrt_info, 2, 2, neg_sqrt_info);
  mat_scale(neg_sqrt_info, 2, 2, -1.0);
  // Form: Jh_w = -1 * sqrt_info * Jh
  real_t Jh[2 * 3] = {0};
  real_t Jh_w[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(cam_params, p_CiFi, Jh);
  dot(neg_sqrt_info, 2, 2, Jh, 2, 3, Jh_w);
  // Form: J_cam_params
  real_t J_cam_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(cam_params, p_CiFi, J_cam_params);

  // -- Fill Jacobians
  real_t T_CiM0b[4 * 4] = {0};
  real_t T_CiM0e[4 * 4] = {0};
  real_t T_CiM1b[4 * 4] = {0};
  real_t T_CiM1e[4 * 4] = {0};
  real_t T_CiM2b[4 * 4] = {0};
  real_t T_CiM2e[4 * 4] = {0};
  dot(T_CiB, 4, 4, T_BM0b, 4, 4, T_CiM0b);
  dot(T_CiM0b, 4, 4, T_M0bM0e, 4, 4, T_CiM0e);
  dot(T_CiM0e, 4, 4, T_M0eM1b, 4, 4, T_CiM1b);
  dot(T_CiM1b, 4, 4, T_M1bM1e, 4, 4, T_CiM1e);
  dot(T_CiM1e, 4, 4, T_M1eM2b, 4, 4, T_CiM2b);
  dot(T_CiM2b, 4, 4, T_M2bM2e, 4, 4, T_CiM2e);

  real_t T_M0bCi[4 * 4] = {0};
  real_t T_M1bCi[4 * 4] = {0};
  real_t T_M2bCi[4 * 4] = {0};
  tf_inv(T_CiM0b, T_M0bCi);
  tf_inv(T_CiM1b, T_M1bCi);
  tf_inv(T_CiM2b, T_M2bCi);

  real_t p_BFi[3] = {0};
  real_t p_M0bFi[3] = {0};
  real_t p_M1bFi[3] = {0};
  real_t p_M2bFi[3] = {0};
  real_t p_M2eFi[3] = {0};
  tf_point(T_BF, p_FFi, p_BFi);
  tf_point(T_M0bCi, p_CiFi, p_M0bFi);
  tf_point(T_M1bCi, p_CiFi, p_M1bFi);
  tf_point(T_M2bCi, p_CiFi, p_M2bFi);
  tf_point(T_M2eCi, p_CiFi, p_M2eFi);

  gimbal_factor_fiducial_jac(Jh_w, T_CiB, T_BF, p_FFi, J_out[0]);
  gimbal_factor_link_jac(Jh_w, T_BM0b, T_CiB, p_BFi, J_out[1]);
  gimbal_factor_link_jac(Jh_w, T_M0eM1b, T_CiM0b, p_M0bFi, J_out[2]);
  gimbal_factor_link_jac(Jh_w, T_M1eM2b, T_CiM1b, p_M1bFi, J_out[3]);
  gimbal_factor_joint_jac(Jh_w, T_CiM0e, p_M0bFi, th0, J_out[4]);
  gimbal_factor_joint_jac(Jh_w, T_CiM1e, p_M1bFi, th1, J_out[5]);
  gimbal_factor_joint_jac(Jh_w, T_CiM2e, p_M2bFi, th2, J_out[6]);
  gimbal_factor_cam_ext_jac(Jh_w, T_M2eCi, p_M2eFi, J_out[7]);
  gimbal_factor_camera_jac(neg_sqrt_info, J_cam_params, J_out[8]);

  return 0;
}

// IMU FACTOR //////////////////////////////////////////////////////////////////

/**
 * Setup IMU buffer
 */
void imu_buf_setup(imu_buf_t *imu_buf) {
  for (int k = 0; k < MAX_IMU_BUF_SIZE; k++) {
    imu_buf->ts[k] = 0.0;

    imu_buf->acc[k][0] = 0.0;
    imu_buf->acc[k][1] = 0.0;
    imu_buf->acc[k][2] = 0.0;

    imu_buf->gyr[k][0] = 0.0;
    imu_buf->gyr[k][1] = 0.0;
    imu_buf->gyr[k][2] = 0.0;
  }

  imu_buf->size = 0;
}

/**
 * Print IMU buffer
 */
void imu_buf_print(const imu_buf_t *imu_buf) {
  for (int k = 0; k < imu_buf->size; k++) {
    const real_t *acc = imu_buf->acc[k];
    const real_t *gyr = imu_buf->gyr[k];

    printf("ts: %ld ", imu_buf->ts[k]);
    printf("acc: [%.2f, %.2f, %.2f] ", acc[0], acc[1], acc[2]);
    printf("gyr: [%.2f, %.2f, %.2f] ", gyr[0], gyr[1], gyr[2]);
    printf("\n");
  }
}

/**
 * Add measurement to IMU buffer
 */
void imu_buf_add(imu_buf_t *imu_buf,
                 const timestamp_t ts,
                 const real_t acc[3],
                 const real_t gyr[3]) {
  assert(imu_buf->size < MAX_IMU_BUF_SIZE);
  const int k = imu_buf->size;
  imu_buf->ts[k] = ts;
  imu_buf->acc[k][0] = acc[0];
  imu_buf->acc[k][1] = acc[1];
  imu_buf->acc[k][2] = acc[2];
  imu_buf->gyr[k][0] = gyr[0];
  imu_buf->gyr[k][1] = gyr[1];
  imu_buf->gyr[k][2] = gyr[2];
  imu_buf->size++;
}

/**
 * Clear IMU buffer
 */
void imu_buf_clear(imu_buf_t *imu_buf) {
  for (int k = 0; k < imu_buf->size; k++) {
    timestamp_t *ts = &imu_buf->ts[k];
    real_t *acc = imu_buf->acc[k];
    real_t *gyr = imu_buf->gyr[k];

    *ts = 0;

    acc[0] = 0.0;
    acc[1] = 0.0;
    acc[2] = 0.0;

    gyr[0] = 0.0;
    gyr[1] = 0.0;
    gyr[2] = 0.0;
  }
  imu_buf->size = 0;
}

/**
 * Copy IMU buffer
 */
void imu_buf_copy(const imu_buf_t *src, imu_buf_t *dst) {
  dst->size = 0;
  for (int k = 0; k < src->size; k++) {
    dst->ts[k] = src->ts[k];

    dst->acc[k][0] = src->acc[k][0];
    dst->acc[k][1] = src->acc[k][1];
    dst->acc[k][2] = src->acc[k][2];

    dst->gyr[k][0] = src->gyr[k][0];
    dst->gyr[k][1] = src->gyr[k][1];
    dst->gyr[k][2] = src->gyr[k][2];
  }
  dst->size = src->size;
}

/**
 * Propagate IMU measurement
 */
void imu_factor_propagate_step(real_t r[3],
                               real_t v[3],
                               real_t q[4],
                               real_t ba[3],
                               real_t bg[3],
                               const real_t a[3],
                               const real_t w[3],
                               const real_t dt) {
  // Compensate accelerometer and gyroscope measurements
  const real_t a_t[3] = {a[0] - ba[0], a[1] - ba[1], a[2] - ba[2]};
  const real_t w_t[3] = {w[0] - bg[0], w[1] - bg[1], w[2] - bg[2]};

  // Update position:
  // dr = dr + (dv * dt) + (0.5 * dC * a_t * dt_sq);
  real_t acc_dint[3] = {0};
  real_t C[3 * 3] = {0};
  quat2rot(q, C);
  dot(C, 3, 3, a_t, 3, 1, acc_dint);
  r[0] += (v[0] * dt) + (0.5 * acc_dint[0] * dt * dt);
  r[1] += (v[1] * dt) + (0.5 * acc_dint[1] * dt * dt);
  r[2] += (v[2] * dt) + (0.5 * acc_dint[2] * dt * dt);

  // Update velocity
  // dv = dv + dC * a_t * dt;
  real_t dv[3] = {0};
  real_t acc_int[3] = {a_t[0] * dt, a_t[1] * dt, a_t[2] * dt};
  dot(C, 3, 3, acc_int, 3, 1, dv);
  v[0] += dv[0];
  v[1] += dv[1];
  v[2] += dv[2];

  // Update rotation
  quat_update_dt(q, w_t, dt);

  // Update accelerometer biases
  // ba = ba;

  // Update gyroscope biases
  // bg = bg;
}

/**
 * Form IMU Noise Matrix Q
 */
static void imu_factor_form_Q_matrix(const imu_params_t *imu_params,
                                     real_t Q[12 * 12]) {
  assert(imu_params != NULL);
  assert(Q != NULL);

  const real_t sigma_a_sq = imu_params->sigma_a * imu_params->sigma_a;
  const real_t sigma_g_sq = imu_params->sigma_g * imu_params->sigma_g;
  const real_t sigma_ba_sq = imu_params->sigma_aw * imu_params->sigma_aw;
  const real_t sigma_bg_sq = imu_params->sigma_gw * imu_params->sigma_gw;

  zeros(Q, 12, 12);
  Q[0] = sigma_a_sq;
  Q[13] = sigma_a_sq;
  Q[26] = sigma_a_sq;
  Q[39] = sigma_g_sq;
  Q[52] = sigma_g_sq;
  Q[65] = sigma_g_sq;
  Q[78] = sigma_ba_sq;
  Q[91] = sigma_ba_sq;
  Q[104] = sigma_ba_sq;
  Q[117] = sigma_bg_sq;
  Q[130] = sigma_bg_sq;
  Q[143] = sigma_bg_sq;
}

/**
 * Form IMU Transition Matrix F
 */
static void imu_factor_form_F_matrix(const real_t dq[4],
                                     const real_t ba[3],
                                     const real_t bg[3],
                                     const real_t a[3],
                                     const real_t w[3],
                                     const real_t dt,
                                     real_t I_F_dt[15 * 15]) {
  // Convert quaternion to rotation matrix
  real_t dC[3 * 3] = {0};
  quat2rot(dq, dC);

  // Compensate accelerometer and gyroscope measurements
  const real_t a_t[3] = {a[0] - ba[0], a[1] - ba[1], a[2] - ba[2]};
  const real_t w_t[3] = {w[0] - bg[0], w[1] - bg[1], w[2] - bg[2]};

  // Form continuous time transition matrix F
  // -- Initialize memory for F
  zeros(I_F_dt, 15, 15);
  // -- F[0:3, 3:6] = eye(3);
  I_F_dt[3] = 1.0;
  I_F_dt[19] = 1.0;
  I_F_dt[35] = 1.0;
  // -- F[4:6, 7:9] = -dC * hat(a_t);
  real_t F1[3 * 3] = {0};
  real_t skew_a_t[3 * 3] = {0};
  hat(a_t, skew_a_t);
  dot(dC, 3, 3, skew_a_t, 3, 3, F1);
  mat_block_set(I_F_dt, 15, 3, 5, 6, 8, F1);
  // -- F[4:6, 10:12] = -dC;
  real_t F2[3 * 3] = {0};
  for (int idx = 0; idx < 9; idx++) {
    F2[idx] = -1.0 * dC[idx];
  }
  mat_block_set(I_F_dt, 15, 3, 5, 9, 11, F2);
  // -- F[7:9, 7:9] = -hat(w_t);
  real_t F3[3 * 3] = {0};
  F3[1] = w_t[2];
  F3[2] = -w_t[1];
  F3[3] = -w_t[2];
  F3[5] = w_t[0];
  F3[6] = w_t[1];
  F3[7] = -w_t[0];
  mat_block_set(I_F_dt, 15, 6, 8, 6, 8, F3);
  // -- F[7:9, 13:15] = -eye(3);
  real_t F4[3 * 3] = {-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0};
  mat_block_set(I_F_dt, 15, 6, 8, 12, 14, F4);

  // Discretize continuous time transition matrix
  // I_F_dt = eye(15) + F * dt;
  for (int idx = 0; idx < (15 * 15); idx++) {
    if (idx == 0 || idx % 16 == 0) {
      I_F_dt[idx] = 1.0 + I_F_dt[idx] * dt;
    } else {
      I_F_dt[idx] = I_F_dt[idx] * dt;
    }
  }
}

/**
 * Form IMU Input Matrix G
 */
static void imu_factor_form_G_matrix(const real_t dq[4],
                                     const real_t dt,
                                     real_t G_dt[15 * 12]) {
  // Setup
  real_t dC[3 * 3] = {0};
  quat2rot(dq, dC);
  zeros(G_dt, 15, 12);

  real_t neg_eye[3 * 3] = {0};
  real_t pos_eye[3 * 3] = {0};
  for (int idx = 0; idx < 9; idx += 4) {
    neg_eye[idx] = -1.0;
    pos_eye[idx] = 1.0;
  }

  // Form continuous input matrix G
  // -- G[4:6, 1:3] = -dC;
  real_t G0[3 * 3] = {0};
  for (int idx = 0; idx < 9; idx++) {
    G0[idx] = -1.0 * dC[idx];
  }
  mat_block_set(G_dt, 12, 3, 5, 0, 2, G0);
  // -- G[7:9, 4:6] = -eye(3);
  mat_block_set(G_dt, 12, 6, 8, 3, 5, neg_eye);
  // -- G[10:12, 7:9] = eye(3);
  mat_block_set(G_dt, 12, 9, 11, 6, 8, pos_eye);
  // -- G[13:15, 10:12] = eye(3);
  mat_block_set(G_dt, 12, 12, 14, 9, 11, pos_eye);

  // Discretize G
  // G_dt = G * dt;
  for (int idx = 0; idx < (15 * 12); idx++) {
    G_dt[idx] = G_dt[idx] * dt;
  }
}

/**
 * IMU Factor setup
 */
void imu_factor_setup(imu_factor_t *factor,
                      imu_params_t *imu_params,
                      imu_buf_t *imu_buf,
                      pose_t *pose_i,
                      velocity_t *vel_i,
                      imu_biases_t *biases_i,
                      pose_t *pose_j,
                      velocity_t *vel_j,
                      imu_biases_t *biases_j) {
  // Parameters
  factor->imu_params = imu_params;
  imu_buf_copy(imu_buf, &factor->imu_buf);
  factor->pose_i = pose_i;
  factor->pose_j = pose_j;
  factor->vel_i = vel_i;
  factor->vel_j = vel_j;
  factor->biases_i = biases_i;
  factor->biases_j = biases_j;
  factor->num_params = 10;

  // Covariance and residuals
  zeros(factor->covar, 15, 15);
  zeros(factor->r, 15, 1);
  factor->r_size = 15;

  // Pre-integration variables
  factor->Dt = 0.0;
  eye(factor->F, 15, 15);   // State jacobian
  zeros(factor->P, 15, 15); // State covariance

  // Noise matrix
  imu_factor_form_Q_matrix(imu_params, factor->Q);

  // Setup
  zeros(factor->dr, 3, 1);               // Relative position
  zeros(factor->dv, 3, 1);               // Relative velocity
  quat_setup(factor->dq);                // Relative rotation
  vec_copy(biases_i->ba, 3, factor->ba); // Accelerometer bias
  vec_copy(biases_i->bg, 3, factor->bg); // Gyroscope bias

  // Pre-integrate imu measuremenets
  real_t dt = 0.0;
  for (int k = 0; k < (imu_buf->size - 1); k++) {
    if (k + 1 < imu_buf->size) {
      const timestamp_t ts_i = imu_buf->ts[k];
      const timestamp_t ts_j = imu_buf->ts[k + 1];
      dt = ts2sec(ts_j - ts_i);
    }
    const real_t *a = imu_buf->acc[k];
    const real_t *w = imu_buf->gyr[k];

    // Propagate
    imu_factor_propagate_step(factor->dr,
                              factor->dv,
                              factor->dq,
                              factor->ba,
                              factor->bg,
                              a,
                              w,
                              dt);

    // Transition Matrix F
    real_t I_F_dt[15 * 15] = {0};
    imu_factor_form_F_matrix(factor->dq,
                             factor->ba,
                             factor->bg,
                             a,
                             w,
                             dt,
                             I_F_dt);

    // Input jacobian G
    real_t G_dt[15 * 12] = {0};
    imu_factor_form_G_matrix(factor->dq, dt, G_dt);

    // Update state matrix F and P
    // F = I_F_dt * F;
    real_t F_prev[15 * 15] = {0};
    mat_copy(factor->F, 15, 15, F_prev);
    dot(I_F_dt, 15, 15, F_prev, 15, 15, factor->F);
    // P = I_F_dt * P * I_F_dt' + G_dt * Q * G_dt';
    real_t P_prev[15 * 15] = {0};
    real_t A[15 * 15] = {0};
    real_t B[15 * 15] = {0};
    mat_copy(factor->P, 15, 15, P_prev);
    dot_XAXt(I_F_dt, 15, 15, factor->P, 15, 15, A);
    dot_XAXt(G_dt, 15, 12, factor->Q, 12, 12, B);
    mat_add(A, B, factor->P, 15, 15);

    // Update overall dt
    factor->Dt += dt;
  }

  // Covariance
  mat_copy(factor->P, 15, 15, factor->covar);

  // Square root information
  // real_t info[15 * 15] = {0};
  // svd_inv(factor->covar, 15, 15, info);
  // chol(info, 15, factor->sqrt_info);
}

/**
 * Reset IMU Factor
 */
void imu_factor_reset(imu_factor_t *factor) {
  zeros(factor->r, 15, 1); // Residuals
  zeros(factor->dr, 3, 1); // Relative position
  zeros(factor->dv, 3, 1); // Relative velocity
  quat_setup(factor->dq);  // Relative rotation
  zeros(factor->ba, 3, 1); // Accel bias
  zeros(factor->bg, 3, 1); // Gyro bias
}

/**
 * Evaluate IMU factor
 *
 * Parameter `params`:
 * 1. Position i
 * 2. Quaternion i
 * 3. Velocity i
 * 4. Accelerometer Bias i
 * 5. Gyroscope Bias i
 * 6. Position_j
 * 7. Quaternion_j
 * 8. Velocity_j
 * 9. Accelerometer Bias_j
 * 10. Gyroscope Bias_j
 *
 * Residuals `r_out` represents the IMU error [15x1]
 * Jacobians `J_out` with respect to `params`
 *
 * @returns `0` for success, `-1` for failure
 */
int imu_factor_eval(imu_factor_t *factor,
                    real_t **params,
                    real_t *r_out,
                    real_t **J_out) {
  assert(factor != NULL);
  assert(params != NULL);
  assert(r_out != NULL);
  assert(factor->pose_i);
  assert(factor->pose_j);
  assert(factor->vel_i);
  assert(factor->vel_j);
  assert(factor->biases_i);
  assert(factor->biases_j);

  // Map params
  const real_t *r_i = params[0];
  const real_t *q_i = params[1];
  const real_t *v_i = params[2];
  const real_t *ba_i = params[3];
  const real_t *bg_i = params[4];
  const real_t *r_j = params[5];
  const real_t *q_j = params[6];
  const real_t *v_j = params[7];
  // const real_t *ba_j = params[8];
  // const real_t *bg_j = params[9];

  // Correct the relative position, velocity and rotation
  // -- Extract Jacobians from error-state jacobian
  real_t dr_dba[3 * 3] = {0};
  real_t dr_dbg[3 * 3] = {0};
  real_t dv_dba[3 * 3] = {0};
  real_t dv_dbg[3 * 3] = {0};
  real_t dq_dbg[3 * 3] = {0};
  real_t dba[3] = {0};
  real_t dbg[3] = {0};
  // mat_block_get(factor->F, 15, 0, 9, 3, 12, dr_dba);
  // mat_block_get(factor->F, 15, 0, 12, 3, 15, dr_dbg);
  // mat_block_get(factor->F, 15, 3, 9, 6, 12, dv_dba);
  // mat_block_get(factor->F, 15, 3, 12, 6, 15, dv_dbg);
  // mat_block_get(factor->F, 15, 6, 12, 9, 15, dq_dbg);
  dba[0] = ba_i[0] - factor->ba[0];
  dba[1] = ba_i[1] - factor->ba[1];
  dba[2] = ba_i[2] - factor->ba[2];
  dbg[0] = bg_i[0] - factor->bg[0];
  dbg[1] = bg_i[1] - factor->bg[1];
  dbg[2] = bg_i[2] - factor->bg[2];
  // -- Correct relative position
  // dr = dr + dr_dba * dba + dr_dbg * dbg
  real_t dr[3] = {0};
  {
    real_t ba_correction[3] = {0};
    real_t bg_correction[3] = {0};
    dot(dr_dba, 3, 3, dba, 3, 1, ba_correction);
    dot(dr_dbg, 3, 3, dbg, 3, 1, bg_correction);
    dr[0] = factor->dr[0] + ba_correction[0] + bg_correction[0];
    dr[1] = factor->dr[1] + ba_correction[1] + bg_correction[1];
    dr[2] = factor->dr[2] + ba_correction[2] + bg_correction[2];
  }
  // -- Correct relative velocity
  // dv = dv + dv_dba * dba + dv_dbg * dbg
  real_t dv[3] = {0};
  {
    real_t ba_correction[3] = {0};
    real_t bg_correction[3] = {0};
    dot(dv_dba, 3, 3, dba, 3, 1, ba_correction);
    dot(dv_dbg, 3, 3, dbg, 3, 1, bg_correction);
    dv[0] = factor->dv[0] + ba_correction[0] + bg_correction[0];
    dv[1] = factor->dv[1] + ba_correction[1] + bg_correction[1];
    dv[2] = factor->dv[2] + ba_correction[2] + bg_correction[2];
  }
  // -- Correct relative rotation
  // dq = quat_mul(dq, [1.0, 0.5 * dq_dbg * dbg])
  real_t dq[4] = {0};
  {
    real_t theta[3] = {0};
    dot(dq_dbg, 3, 3, dbg, 3, 1, theta);

    real_t q_correction[4] = {0};
    q_correction[0] = 1.0;
    q_correction[1] = 0.5 * theta[0];
    q_correction[2] = 0.5 * theta[1];
    q_correction[3] = 0.5 * theta[2];

    quat_mul(factor->dq, q_correction, dq);
    quat_normalize(dq);
  }

  // Form residuals
  // sqrt_info = self.sqrt_info
  const real_t g_W[3] = {0.0, 0.0, 10.0};
  const real_t Dt = factor->Dt;
  const real_t Dt_sq = Dt * Dt;
  real_t C_i[3 * 3] = {0};
  real_t C_it[3 * 3] = {0};
  quat2rot(q_i, C_i);
  mat_transpose(C_i, 3, 3, C_it);

  // dr_est = C_i.T @ ((r_j - r_i) - (v_i * Dt) + (0.5 * g_W * Dt_sq))
  real_t dr_est[3] = {0};
  {
    real_t dr_tmp[3] = {0};
    dr_tmp[0] = (r_j[0] - r_i[0]) - (v_i[0] * Dt) + (0.5 * g_W[0] * Dt_sq);
    dr_tmp[1] = (r_j[1] - r_i[1]) - (v_i[1] * Dt) + (0.5 * g_W[1] * Dt_sq);
    dr_tmp[2] = (r_j[2] - r_i[2]) - (v_i[2] * Dt) + (0.5 * g_W[2] * Dt_sq);

    dot(C_it, 3, 3, dr_tmp, 3, 1, dr_est);
  }

  // dv_est = C_i.T @ ((v_j - v_i) + (g_W * Dt))
  real_t dv_est[3] = {0};
  {
    real_t dv_tmp[3] = {0};
    dv_tmp[0] = (v_j[0] - v_i[0]) + (g_W[0] * Dt);
    dv_tmp[1] = (v_j[1] - v_i[1]) + (g_W[1] * Dt);
    dv_tmp[2] = (v_j[2] - v_i[2]) + (g_W[2] * Dt);

    dot(C_it, 3, 3, dv_tmp, 3, 1, dv_est);
  }

  // err_pos = dr_est - dr
  real_t err_pos[3] = {0.0, 0.0, 0.0};
  err_pos[0] = dr_est[0] - dr[0];
  err_pos[1] = dr_est[1] - dr[1];
  err_pos[2] = dr_est[2] - dr[2];

  // err_vel = dv_est - dv
  real_t err_vel[3] = {0.0, 0.0, 0.0};
  err_vel[0] = dv_est[0] - dv[0];
  err_vel[1] = dv_est[1] - dv[1];
  err_vel[2] = dv_est[2] - dv[2];

  // err_rot = (2.0 * quat_mul(quat_inv(dq), quat_mul(quat_inv(q_i),
  // q_j)))[1:4]
  real_t err_rot[3] = {0.0, 0.0, 0.0};
  {
    real_t dq_inv[4] = {0};
    real_t q_i_inv[4] = {0};
    real_t q_i_inv_j[4] = {0};
    real_t err_quat[4] = {0};

    quat_inv(factor->dq, dq_inv);
    quat_inv(q_i, q_i_inv);
    quat_mul(q_i_inv, q_j, q_i_inv_j);
    quat_mul(dq_inv, q_i_inv_j, err_quat);

    err_rot[0] = 2.0 * err_quat[1];
    err_rot[1] = 2.0 * err_quat[2];
    err_rot[2] = 2.0 * err_quat[3];
  }

  // err_ba = [0.0, 0.0, 0.0]
  real_t err_ba[3] = {0.0, 0.0, 0.0};

  // err_bg = [0.0, 0.0, 0.0]
  real_t err_bg[3] = {0.0, 0.0, 0.0};

  // Residual vector
  // r = sqrt_info * [err_pos; err_vel; err_rot; err_ba; err_bg]
  {
    real_t r_raw[15] = {0};
    r_raw[0] = err_pos[0];
    r_raw[1] = err_pos[1];
    r_raw[2] = err_pos[2];

    r_raw[3] = err_vel[0];
    r_raw[4] = err_vel[1];
    r_raw[5] = err_vel[2];

    r_raw[6] = err_rot[0];
    r_raw[7] = err_rot[1];
    r_raw[8] = err_rot[2];

    r_raw[9] = err_ba[0];
    r_raw[10] = err_ba[1];
    r_raw[11] = err_ba[2];

    r_raw[12] = err_bg[0];
    r_raw[13] = err_bg[1];
    r_raw[14] = err_bg[2];
    // print_vector("r_raw", r_raw, 15);

    dot(factor->sqrt_info, 15, 15, r_raw, 15, 1, factor->r);
  }

  // Form Jacobians
  if (J_out == NULL) {
    return 0;
  }
  // -- Setup
  real_t dC[3 * 3] = {0};
  real_t C_j[3 * 3] = {0};
  real_t C_jt[3 * 3] = {0};
  real_t C_ji[3 * 3] = {0};
  real_t C_ji_dC[3 * 3] = {0};
  real_t qji_dC[4] = {0};
  real_t left_xyz[3 * 3] = {0};
  quat2rot(factor->dq, dC);
  quat2rot(q_j, C_j);
  mat_transpose(C_j, 3, 3, C_jt);
  dot(C_jt, 3, 3, C_i, 3, 3, C_ji);
  dot(C_ji, 3, 3, dC, 3, 3, C_ji_dC);
  rot2quat(C_ji_dC, qji_dC);
  quat_left_xyz(qji_dC, left_xyz);
  // -- Jacobian w.r.t. r_i
  if (J_out[0]) {
    real_t J0[15 * 3] = {0};
    real_t drij_dri[3 * 3] = {0};
    for (int idx = 0; idx < 9; idx++) {
      drij_dri[idx] = -1.0 * C_it[idx];
    }
    mat_block_set(J0, 3, 0, 2, 0, 2, drij_dri);
    dot(factor->sqrt_info, 15, 15, J0, 15, 3, J_out[0]);
  }

  // -- Jacobian w.r.t. q_i
  if (J_out[1]) {
    real_t drij_dCi[3 * 3] = {0};
    real_t dvij_dCi[3 * 3] = {0};
    real_t dtheta_dCi[3 * 3] = {0};

    hat(dr_est, drij_dCi);
    hat(dv_est, dvij_dCi);
    mat_copy(left_xyz, 3, 3, dtheta_dCi);
    for (int i = 0; i < 9; i++) {
      dtheta_dCi[i] *= -1.0;
    }

    real_t J1[15 * 3] = {0};
    mat_block_set(J1, 3, 0, 2, 0, 2, drij_dCi);
    mat_block_set(J1, 3, 3, 5, 0, 2, dvij_dCi);
    mat_block_set(J1, 3, 6, 8, 0, 2, dtheta_dCi);
    dot(factor->sqrt_info, 15, 15, J1, 15, 3, J_out[1]);
  }

  // -- Jacobian w.r.t. v_i
  if (J_out[2]) {
    real_t drij_dvi[3 * 3] = {0};
    real_t dvij_dvi[3 * 3] = {0};
    for (int idx = 0; idx < 9; idx++) {
      drij_dvi[idx] = -1.0 * C_it[idx] * factor->Dt;
      dvij_dvi[idx] = -1.0 * C_it[idx];
    }

    real_t J2[15 * 3] = {0};
    mat_block_set(J2, 3, 0, 2, 0, 2, drij_dvi);
    mat_block_set(J2, 3, 3, 5, 0, 2, dvij_dvi);
    dot(factor->sqrt_info, 15, 15, J2, 15, 3, J_out[2]);
  }

  // -- Jacobian w.r.t ba_i
  if (J_out[3]) {
    real_t drij_dbai[3 * 3] = {0};
    real_t dvij_dbai[3 * 3] = {0};
    for (int idx = 0; idx < 9; idx++) {
      drij_dbai[idx] = -1.0 * dr_dba[idx];
      dvij_dbai[idx] = -1.0 * dr_dba[idx];
    }

    real_t J3[15 * 3] = {0};
    mat_block_set(J3, 3, 0, 2, 0, 2, drij_dbai);
    mat_block_set(J3, 3, 3, 5, 0, 2, dvij_dbai);
    dot(factor->sqrt_info, 15, 15, J3, 15, 3, J_out[3]);
  }

  // -- Jacobian w.r.t bg_i
  if (J_out[4]) {
    real_t drij_dbgi[3 * 3] = {0};
    real_t dvij_dbgi[3 * 3] = {0};
    real_t dtheta_dbgi[3 * 3] = {0};

    for (int idx = 0; idx < 9; idx++) {
      drij_dbgi[idx] = -1.0 * dr_dbg[idx];
      dvij_dbgi[idx] = -1.0 * dr_dbg[idx];
    }

    dot(left_xyz, 3, 3, dq_dbg, 3, 3, dtheta_dbgi);
    for (int i = 0; i < 9; i++) {
      dtheta_dbgi[i] *= -1.0;
    }

    real_t J4[15 * 3] = {0};
    mat_block_set(J4, 3, 0, 2, 0, 2, drij_dbgi);
    mat_block_set(J4, 3, 3, 5, 0, 2, dvij_dbgi);
    mat_block_set(J4, 3, 6, 8, 0, 2, dtheta_dbgi);
    dot(factor->sqrt_info, 15, 15, J4, 15, 3, J_out[4]);
  }

  // -- Jacobian w.r.t. r_j
  if (J_out[5]) {
    real_t drij_drj[3 * 3] = {0};
    mat_copy(C_it, 3, 3, drij_drj);

    real_t J5[15 * 3] = {0};
    mat_block_set(J5, 3, 0, 2, 0, 2, drij_drj);
    dot(factor->sqrt_info, 15, 15, J5, 15, 3, J_out[5]);
  }

  // -- Jacobian w.r.t. q_j
  if (J_out[6]) {
    real_t dtheta_dqj[3 * 3] = {0};
    quat_left_xyz(qji_dC, dtheta_dqj);

    real_t J6[15 * 3] = {0};
    mat_block_set(J6, 3, 6, 8, 0, 2, dtheta_dqj);
    dot(factor->sqrt_info, 15, 15, J6, 15, 3, J_out[6]);
  }

  // -- Jacobian w.r.t. v_j
  if (J_out[7]) {
    real_t dv_dvj[3 * 3] = {0};
    mat_copy(C_it, 3, 3, dv_dvj);

    real_t J7[15 * 3] = {0};
    mat_block_set(J7, 3, 3, 5, 0, 2, dv_dvj);
    dot(factor->sqrt_info, 15, 15, J7, 15, 3, J_out[6]);
  }

  // -- Jacobian w.r.t. ba_j
  if (J_out[8]) {
    zeros(J_out[8], 15, 3);
  }

  // -- Jacobian w.r.t. bg_j
  if (J_out[9]) {
    zeros(J_out[9], 15, 3);
  }

  return 0;
}

// SOLVER ////////////////////////////////////////////////////////////////////

void solver_setup(solver_t *solver) {
  assert(solver);
  solver->H = NULL;
  solver->g = NULL;
  solver->x = NULL;
  solver->x_size = 0;
  solver->r_size = 0;
}

void solver_print(solver_t *solver) {
  printf("solver:\n");
  printf("r_size: %d\n", solver->r_size);
  printf("x_size: %d\n", solver->x_size);
}

void solver_evaluator(solver_t *solver,
                      int **param_orders,
                      int *param_sizes,
                      int num_params,
                      real_t *r,
                      int r_size,
                      real_t **jacs) {
  real_t *H = solver->H;
  int H_size = solver->x_size;
  real_t *g = solver->g;

  for (int i = 0; i < num_params; i++) {
    int *idx_i = param_orders[i];
    int size_i = param_sizes[i];
    const real_t *J_i = jacs[i];

    real_t *J_i_trans = {0};
    mat_transpose(J_i, r_size, size_i, J_i_trans);

    for (int j = i; j < num_params; j++) {
      int *idx_j = param_orders[j];
      int size_j = param_sizes[i];
      const real_t *J_j = jacs[j];

      real_t *H_ij = {0};
      dot(J_i_trans, size_i, r_size, J_j, r_size, size_j, H_ij);

      // Fill Hessian H
      // H_ij = J_i' * J_j
      // H_ji = H_ij'
      int stride = H_size;
      int rs = *idx_i;
      int cs = *idx_j;
      int re = rs + size_i;
      int ce = cs + size_j;
      if (i == j) {
        mat_block_set(H, stride, rs, re, cs, ce, H_ij);
      } else {
        real_t *H_ji = {0};
        mat_transpose(H_ij, size_i, size_j, H_ji);
        mat_block_set(H, stride, rs, re, cs, ce, H_ij);
        mat_block_set(H, stride, cs, ce, rs, re, H_ij);
      }

      // Fill in the R.H.S of H dx = g
      // g = -J_i * r
      mat_scale(J_i_trans, H_size, r_size, -1);
      dot(J_i_trans, H_size, r_size, r, r_size, 1, g);
    }
  }

  // Update parameter order
  for (int i = 0; i < num_params; i++) {
    param_orders[i] = param_orders[i] + param_sizes[i];
  }
}

int solver_eval(solver_t *solver) {
  assert(solver != NULL);

  // int pose_idx = 0;
  // int lmks_idx = solver->nb_poses * 6;
  // int exts_idx = lmks_idx + solver->nb_features * 3;
  // int cams_idx = exts_idx + solver->nb_exts * 6;

  // // Evaluate camera factors
  // for (int i = 0; i < solver->num_vision_factors; i++) {
  //   vision_factor_t *factor = &solver->vision_factors[i];
  //   vision_factor_eval(factor);

  //   int *param_orders[4] = {&pose_idx, &exts_idx, &cams_idx, &lmks_idx};
  //   int param_sizes[4] = {6, 6, 8, 3};
  //   int num_params = 4;

  //   solver_evaluator(solver,
  //                    param_orders,
  //                    param_sizes,
  //                    num_params,
  //                    factor->r,
  //                    factor->r_size,
  //                    factor->jacs);
  // }

  return 0;
}

// int solver_solve(solver_t *solver) {
//   struct timespec solve_tic = tic();
//   real_t lambda_k = 1e-4;
//
//   int iter = 0;
//   int max_iter = 10;
//   int verbose = 1;
//
//   for (iter = 0; iter < max_iter; iter++) {
//     #<{(| Cost k |)}>#
//     #<{(| x = solver_get_state(solver); |)}>#
//     #<{(| solver_eval(solver, H, g, &marg_size, &remain_size); |)}>#
//     #<{(| const matx_t H_diag = (H.diagonal().asDiagonal()); |)}>#
//     #<{(| H = H + lambda_k * H_diag; |)}>#
//     #<{(| dx = H.ldlt().solve(g); |)}>#
//     #<{(| e = solver_residuals(solver); |)}>#
//     #<{(| cost = 0.5 * e.transpose() * e; |)}>#
//
//     #<{(| Cost k+1 |)}>#
//     #<{(| solver_update(solver, dx); |)}>#
//     #<{(| e = solver_residuals(solver); |)}>#
//     const real_t cost_k = 0.5 * e.transpose() * e;
//
//     const real_t cost_delta = cost_k - cost;
//     const real_t solve_time = toc(&solve_tic);
//     const real_t iter_time = (iter == 0) ? 0 : (solve_time / iter);
//
//     if (verbose) {
//       printf("iter[%d] ", iter);
//       printf("cost[%.2e] ", cost);
//       printf("cost_k[%.2e] ", cost_k);
//       printf("cost_delta[%.2e] ", cost_delta);
//       printf("lambda[%.2e] ", lambda_k);
//       printf("iter_time[%.4f] ", iter_time);
//       printf("solve_time[%.4f]  ", solve_time);
//       printf("\n");
//
//       // // Calculate reprojection error
//       // size_t nb_keypoints = e.size() / 2.0;
//       // real_t sse = 0.0;
//       // for (size_t i = 0; i < nb_keypoints; i++) {
//       //   sse += pow(e.segment(i * 2, 2).norm(), 2);
//       // }
//       // const real_t rmse = sqrt(sse / nb_keypoints);
//       // printf("rmse reproj error: %.2f\n", rmse);
//     }
//
//     #<{(| Determine whether to accept update |)}>#
//     if (cost_k < cost) {
//       #<{(| Accept update |)}>#
//       lambda_k /= update_factor;
//       cost = cost_k;
//     } else {
//       #<{(| Reject update |)}>#
//       #<{(| solver_set_state(solver, x); // Restore state |)}>#
//       lambda_k *= update_factor;
//     }
//
//     #<{(| Termination criterias |)}>#
//     if (fabs(cost_delta) < cost_change_threshold) {
//       break;
//     } else if ((solve_time + iter_time) > time_limit) {
//       break;
//     }
//   }
//
//   #<{(| solve_time = toc(&solve_tic); |)}>#
//   #<{(| if (verbose) { |)}>#
//   #<{(|   printf("cost: %.2e\t", cost); |)}>#
//   #<{(|   printf("solver took: %.4fs\n", solve_time); |)}>#
//   #<{(| } |)}>#
// }

// CALIBRATION ///////////////////////////////////////////////////////////////

/**
 * Setup gimbal calibration view
 */
void calib_gimbal_view_setup(calib_gimbal_view_t *view) {
  view->ts = 0;
  view->cam_idx = 0;
  view->view_idx = 0;

  view->tag_ids = NULL;
  view->corner_indices = NULL;
  view->object_points = NULL;
  view->keypoints = NULL;
  view->num_corners = 0;
}

/**
 * Free gimbal calibration view
 */
void calib_gimbal_view_free(calib_gimbal_view_t *view) {
  if (view->num_corners) {
    free(view->tag_ids);
    free(view->corner_indices);

    for (int i = 0; i < view->num_corners; i++) {
      free(view->object_points[i]);
      free(view->keypoints[i]);
    }
    free(view->object_points);
    free(view->keypoints);
  }
  free(view);
}

/**
 * Print gimbal calibration view
 */
void calib_gimbal_view_print(calib_gimbal_view_t *view) {
  printf("ts: %ld\n", view->ts);
  printf("cam_idx: %d\n", view->cam_idx);
  printf("view_idx: %d\n", view->view_idx);
  printf("num_corners: %d\n", view->num_corners);
  printf("\n");

  printf("tag_id  corner_idx  object_point          keypoint\n");
  for (int i = 0; i < view->num_corners; i++) {
    printf("%d       ", view->tag_ids[i]);
    printf("%d           ", view->corner_indices[i]);
    printf("(%.2f, %.2f, %.2f)  ",
           view->object_points[i][0],
           view->object_points[i][1],
           view->object_points[i][2]);
    printf("(%.2f, %.2f) ", view->keypoints[i][0], view->keypoints[i][1]);
    printf("\n");
  }
}

/**
 * Setup gimbal calibration data
 */
void calib_gimbal_setup(calib_gimbal_t *calib) {
  calib->fiducial = NULL;
  calib->cam_params = NULL;
  calib->cam_exts = NULL;
  calib->num_cams = 0;

  calib->links = NULL;
  calib->num_links = 0;

  calib->joints = NULL;
  calib->num_joints = 0;

  calib->views = NULL;
  calib->num_views = 0;
}

/**
 * Free gimbal calibration data
 */
void calib_gimbal_free(calib_gimbal_t *calib) {
  if (calib->fiducial) {
    free(calib->fiducial);
  }
  if (calib->cam_exts) {
    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      free(calib->cam_exts[cam_idx]);
    }
    free(calib->cam_exts);
  }
  if (calib->cam_params) {
    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      free(calib->cam_params[cam_idx]);
    }
    free(calib->cam_params);
  }

  if (calib->links) {
    for (int link_idx = 0; link_idx < calib->num_links; link_idx++) {
      free(calib->links[link_idx]);
    }
    free(calib->links);
  }

  if (calib->joints) {
    for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
      free(calib->joints[view_idx]);
    }
    free(calib->joints);
  }

  if (calib->views) {
    for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
      for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
        calib_gimbal_view_free(calib->views[view_idx][cam_idx]);
      }
      free(calib->views[view_idx]);
    }
    free(calib->views);
  }

  if (calib->num_params) {
    free(calib->params);
    free(calib->param_types);
    free(calib->param_indices);
    free(calib->param_sizes);
  }

  free(calib);
}

void calib_gimbal_print_params(calib_gimbal_t *calib) {
  for (int i = 0; i < calib->num_params; i++) {
    char type[20] = {0};
    switch (calib->param_types[i]) {
      case POSE_PARAM:
        string_copy(type, "POSE_PARAM");
        break;
      case SB_PARAM:
        string_copy(type, "SB_PARAM");
        break;
      case FEATURE_PARAM:
        string_copy(type, "FEATURE_PARAM");
        break;
      case EXTRINSICS_PARAM:
        string_copy(type, "EXTRINSICS_PARAM");
        break;
      case JOINT_PARAM:
        string_copy(type, "JOINT_PARAM");
        break;
      case CAMERA_PARAM:
        string_copy(type, "CAMERA_PARAM");
        break;
    }
    printf("param[%d], param_idx: %d, type: %s\n",
           i,
           calib->param_indices[i],
           type);
  }
}

/**
 * Print gimbal calibration data
 */
void calib_gimbal_print(calib_gimbal_t *calib) {
  // Print
  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    char cam_str[20] = {0};
    sprintf(cam_str, "cam%d_exts", cam_idx);
    extrinsics_print(cam_str, calib->cam_exts[cam_idx]);
  }

  for (int link_idx = 0; link_idx < calib->num_links; link_idx++) {
    char link_str[20] = {0};
    sprintf(link_str, "link%d_exts", link_idx);
    extrinsics_print("link0_exts", calib->links[link_idx]);
  }

  extrinsics_print("fiducial_exts", calib->fiducial);
}

/**
 * Skip line
 */
static void parse_skip_line(FILE *fp) {
  assert(fp != NULL);
  const size_t buf_len = 9046;
  char buf[9046] = {0};
  const char *read = fgets(buf, buf_len, fp);
  UNUSED(read);
}

/**
 * Parse integer vector from string line.
 * @returns `0` for success or `-1` for failure
 */
static int parse_vector_line(char *line, const char *type, void *data, int n) {
  assert(line != NULL);
  assert(data != NULL);
  char entry[MAX_LINE_LENGTH] = {0};
  int index = 0;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == '[' || c == ' ') {
      continue;
    }

    if (c == ',' || c == ']' || c == '\n') {
      if (strcmp(type, "int") == 0) {
        ((int *) data)[index] = strtod(entry, NULL);
      } else if (strcmp(type, "double") == 0) {
        ((double *) data)[index] = strtod(entry, NULL);
      } else {
        FATAL("Invalid type [%s]\n", type);
      }
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  if (index != n) {
    return -1;
  }

  return 0;
}

/** Parse key-value pair from string line **/
static void parse_key_value(FILE *fp,
                            const char *key,
                            const char *value_type,
                            void *value) {
  assert(fp != NULL);
  assert(key != NULL);
  assert(value_type != NULL);
  assert(value != NULL);

  // Parse line
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  if (fgets(buf, buf_len, fp) == NULL) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Split key-value
  char delim[2] = ":";
  char *key_str = strtok(buf, delim);
  char *value_str = strtok(NULL, delim);
  key_str = string_strip(key_str);
  value_str = string_strip(value_str);

  // Check key matches
  if (strcmp(key_str, key) != 0) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Typecase value
  if (value_type == NULL) {
    FATAL("Value type not set!\n");
  }

  // Parse value
  if (strcmp(value_type, "int") == 0) {
    *(int *) value = atoi(value_str);
  } else if (strcmp(value_type, "double") == 0) {
    *(double *) value = atof(value_str);
  } else if (strcmp(value_type, "uint64_t") == 0) {
    *(uint64_t *) value = atol(value_str);
  } else if (strcmp(value_type, "string") == 0) {
    value_str = string_strip_char(value_str, '"');
    string_copy(value, value_str);
  } else if (strcmp(value_type, "vec2i") == 0) {
    parse_vector_line(value_str, "int", value, 2);
  } else if (strcmp(value_type, "vec3i") == 0) {
    parse_vector_line(value_str, "int", value, 3);
  } else if (strcmp(value_type, "vec2d") == 0) {
    parse_vector_line(value_str, "double", value, 2);
  } else if (strcmp(value_type, "vec3d") == 0) {
    parse_vector_line(value_str, "double", value, 3);
  } else if (strcmp(value_type, "vec4d") == 0) {
    parse_vector_line(value_str, "double", value, 4);
  } else if (strcmp(value_type, "vec7d") == 0) {
    parse_vector_line(value_str, "double", value, 7);
  } else if (strcmp(value_type, "pose") == 0) {
    parse_vector_line(value_str, "double", value, 6);
  } else {
    FATAL("Invalid value type [%s]\n", value_type);
  }
}

/**
 * Load gimbal calibration data
 */
calib_gimbal_t *calib_gimbal_load(const char *data_path) {
  // Setup calib data
  calib_gimbal_t *calib = MALLOC(calib_gimbal_t, 1);
  calib_gimbal_setup(calib);

  // Load configuration
  // -- Open config file
  char conf_path[100] = {0};
  string_cat(conf_path, data_path);
  string_cat(conf_path, "/calib.config");
  FILE *conf = fopen(conf_path, "r");
  if (conf == NULL) {
    LOG_ERROR("Failed to open [%s]!\n", conf_path);
    return NULL;
  }
  // -- Parse general
  parse_key_value(conf, "num_cams", "int", &calib->num_cams);
  parse_key_value(conf, "num_links", "int", &calib->num_links);
  parse_skip_line(conf);
  // -- Parse camera parameters
  calib->cam_params = MALLOC(camera_params_t *, calib->num_cams);
  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    real_t proj_params[4] = {0};
    real_t dist_params[4] = {0};

    camera_params_t *cam_params = MALLOC(camera_params_t, 1);
    cam_params->cam_idx = cam_idx;

    parse_skip_line(conf);
    parse_key_value(conf, "resolution", "vec2i", cam_params->resolution);
    parse_key_value(conf, "proj_model", "string", cam_params->proj_model);
    parse_key_value(conf, "dist_model", "string", cam_params->dist_model);
    parse_key_value(conf, "proj_params", "vec4d", proj_params);
    parse_key_value(conf, "dist_params", "vec4d", dist_params);
    parse_skip_line(conf);

    cam_params->data[0] = proj_params[0];
    cam_params->data[1] = proj_params[1];
    cam_params->data[2] = proj_params[2];
    cam_params->data[3] = proj_params[3];

    cam_params->data[4] = dist_params[0];
    cam_params->data[5] = dist_params[1];
    cam_params->data[6] = dist_params[2];
    cam_params->data[7] = dist_params[3];

    calib->cam_params[cam_idx] = cam_params;
  }
  // -- Parse camera extrinsics
  calib->cam_exts = MALLOC(extrinsics_t *, calib->num_cams);
  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    char key[30] = {0};
    real_t pose[7] = {0};
    sprintf(key, "cam%d_exts", cam_idx);
    parse_key_value(conf, key, "pose", pose);

    calib->cam_exts[cam_idx] = MALLOC(extrinsics_t, 1);
    extrinsics_setup(calib->cam_exts[cam_idx], pose);
  }
  // -- Parse links
  calib->links = MALLOC(extrinsics_t *, calib->num_links);
  for (int link_idx = 0; link_idx < calib->num_links; link_idx++) {
    char key[30] = {0};
    real_t pose[7] = {0};
    sprintf(key, "link%d_exts", link_idx);
    parse_key_value(conf, key, "pose", pose);

    calib->links[link_idx] = MALLOC(extrinsics_t, 1);
    extrinsics_setup(calib->links[link_idx], pose);
  }
  // -- Parse fiducial
  {
    real_t pose[7] = {0};
    parse_key_value(conf, "fiducial_exts", "pose", pose);
    calib->fiducial = MALLOC(extrinsics_t, 1);
    extrinsics_setup(calib->fiducial, pose);
  }
  // -- Clean up
  fclose(conf);

  // Load joint angles
  // -- Open joint angles file
  char joints_path[100] = {0};
  string_cat(joints_path, data_path);
  string_cat(joints_path, "/joint_angles.dat");
  FILE *joints_file = fopen(joints_path, "r");
  if (joints_file == NULL) {
    LOG_ERROR("Failed to open [%s]!\n", joints_path);
    return NULL;
  }
  // -- Parse
  parse_key_value(joints_file, "num_views", "int", &calib->num_views);
  parse_key_value(joints_file, "num_joints", "int", &calib->num_joints);
  parse_skip_line(joints_file);
  parse_skip_line(joints_file);

  calib->joints = MALLOC(joint_angle_t *, calib->num_views);
  for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
    // Get line
    const size_t buf_len = 1024;
    char buf[1024] = {0};
    if (fgets(buf, buf_len, joints_file) == NULL) {
      FATAL("Failed to view parse data!\n");
    }

    // Parse line
    calib->joints[view_idx] = MALLOC(joint_angle_t, calib->num_joints);
    real_t *data = MALLOC(real_t, calib->num_joints + 1);
    parse_vector_line(buf, "double", data, calib->num_joints + 1);
    for (int joint_idx = 0; joint_idx < calib->num_joints; joint_idx++) {
      joint_angle_setup(&calib->joints[view_idx][joint_idx],
                        joint_idx,
                        data[joint_idx + 1]);
    }
    free(data);
  }
  // -- Clean up
  fclose(joints_file);

  // Load views
  char ***view_files = MALLOC(char *, calib->num_cams);
  // -- List view files
  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    // Camera directory
    char cam_path[100] = {0};
    char cam_str[100] = {0};
    sprintf(cam_str, "/cam%d", cam_idx);
    string_cat(cam_path, data_path);
    string_cat(cam_path, cam_str);

    // List files
    int num_files = 0;
    view_files[cam_idx] = list_files(cam_path, &num_files);
  }

  calib->views = MALLOC(calib_gimbal_view_t **, calib->num_views);
  for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
    calib->views[view_idx] = MALLOC(calib_gimbal_view_t *, calib->num_cams);

    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      // Get file name
      char fname[1024] = {0};
      char *view_fpath = view_files[cam_idx][view_idx];
      path_file_name(view_fpath, fname);

      // Extract timestamp from filename
      char delim[2] = ".";
      char *ts_str = strtok(fname, delim);
      timestamp_t ts = atol(ts_str);

      // Load view data
      FILE *view_file = fopen(view_fpath, "r");
      if (view_file == NULL) {
        FATAL("Failed to open file [%s]\n", view_fpath);
      }
      // -- Parse data
      int num_corners = 0;
      parse_key_value(view_file, "num_corners", "int", &num_corners);
      parse_skip_line(view_file);

      calib_gimbal_view_t *view = MALLOC(calib_gimbal_view_t, 1);
      calib_gimbal_view_setup(view);

      view->ts = ts;
      view->cam_idx = cam_idx;
      view->view_idx = view_idx;
      view->num_corners = num_corners;

      view->tag_ids = MALLOC(int, view->num_corners);
      view->corner_indices = MALLOC(int, view->num_corners);
      view->object_points = MALLOC(real_t *, view->num_corners);
      view->keypoints = MALLOC(real_t *, view->num_corners);

      parse_skip_line(view_file);
      for (int i = 0; i < view->num_corners; i++) {
        // Get line
        const size_t buf_len = 1024;
        char buf[1024] = {0};
        if (fgets(buf, buf_len, view_file) == NULL) {
          FATAL("Failed to view parse data!\n");
        }

        // Parse line
        real_t data[7] = {0};
        parse_vector_line(buf, "double", data, 7);

        // Add to view
        view->tag_ids[i] = (int) data[0];
        view->corner_indices[i] = (int) data[1];
        view->object_points[i] = vector_malloc(&data[2], 3);
        view->keypoints[i] = vector_malloc(&data[5], 2);
      }

      // Clean up
      calib->views[view_idx][cam_idx] = view;
      fclose(view_file);
    }
  }

  // Parameters
  calib->idx_counter = 0;
  calib->params = MALLOC(void *, CALIB_GIMBAL_EXPAND_SIZE);
  calib->param_types = MALLOC(int, CALIB_GIMBAL_EXPAND_SIZE);
  calib->param_indices = MALLOC(int, CALIB_GIMBAL_EXPAND_SIZE);
  calib->param_sizes = MALLOC(int, CALIB_GIMBAL_EXPAND_SIZE);
  calib->num_params = 0;

  // Clean up
  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
      free(view_files[cam_idx][view_idx]);
    }
    free(view_files[cam_idx]);
  }
  free(view_files);

  return calib;
}

void calib_gimbal_add_param(calib_gimbal_t *calib,
                            void *param,
                            int param_type) {
  size_t param_size = 0;
  switch (param_type) {
    case POSE_PARAM:
      param_size = 6;
      break;
    case SB_PARAM:
      param_size = 9;
      break;
    case FEATURE_PARAM:
      param_size = 3;
      break;
    case EXTRINSICS_PARAM:
      param_size = 6;
      break;
    case JOINT_PARAM:
      param_size = 1;
      break;
    case CAMERA_PARAM:
      param_size = 8;
      break;
  }

  calib->params[calib->num_params] = param;
  calib->param_types[calib->num_params] = param_type;
  calib->param_indices[calib->num_params] = calib->idx_counter;
  calib->param_sizes[calib->num_params] = param_size;
  calib->num_params++;
  calib->idx_counter += param_size;
}

// void calib_gimbal_linearize(const calib_gimbal_t *calib,
//                             int **param_orders,
//                             int *param_sizes,
//                             int num_params,
//                             real_t *H,
//                             int H_m,
//                             int H_n,
//                             real_t *r,
//                             int r_size) {
//   for (int i = 0; i < num_params; i++) {
//     int *idx_i = param_orders[i];
//     int size_i = param_sizes[i];
//     const real_t *J_i = jacs[i];

//     real_t *J_i_trans = {0};
//     mat_transpose(J_i, r_size, size_i, J_i_trans);

//     for (int j = i; j < num_params; j++) {
//       int *idx_j = param_orders[j];
//       int size_j = param_sizes[i];
//       const real_t *J_j = jacs[j];

//       real_t *H_ij = {0};
//       dot(J_i_trans, size_i, r_size, J_j, r_size, size_j, H_ij);

//       // Fill Hessian H
//       // H_ij = J_i' * J_j
//       // H_ji = H_ij'
//       int stride = H_size;
//       int rs = *idx_i;
//       int cs = *idx_j;
//       int re = rs + size_i;
//       int ce = cs + size_j;
//       if (i == j) {
//         mat_block_set(H, stride, rs, re, cs, ce, H_ij);
//       } else {
//         real_t *H_ji = {0};
//         mat_transpose(H_ij, size_i, size_j, H_ji);
//         mat_block_set(H, stride, rs, re, cs, ce, H_ij);
//         mat_block_set(H, stride, cs, ce, rs, re, H_ij);
//       }

//       // Fill in the R.H.S of H dx = g
//       // g = -J_i * r
//       mat_scale(J_i_trans, H_size, r_size, -1);
//       dot(J_i_trans, H_size, r_size, r, r_size, 1, g);
//     }
//   }

//   // Update parameter order
//   for (int i = 0; i < num_params; i++) {
//     param_orders[i] = param_orders[i] + param_sizes[i];
//   }
// }

/******************************************************************************
 * DATASET
 ******************************************************************************/

static int
parse_pose_data(const int i, const int j, const char *entry, pose_t *poses) {
  switch (j) {
    case 0:
      poses[i].ts = strtol(entry, NULL, 0);
      break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      poses[i].data[j - 1] = strtod(entry, NULL);
      break;
    default:
      return -1;
  }

  return 0;
}

/**
 * Load poses from file `fp`. The number of poses in file will be outputted to
 * `nb_poses`.
 */
pose_t *load_poses(const char *fp, int *nb_poses) {
  assert(fp != NULL);
  assert(nb_poses != NULL);

  // Obtain number of rows and columns in dsv data
  int nb_rows = dsv_rows(fp);
  int nb_cols = dsv_cols(fp, ',');
  if (nb_rows == -1 || nb_cols == -1) {
    return NULL;
  }

  // Initialize memory for pose data
  *nb_poses = nb_rows;
  pose_t *poses = MALLOC(pose_t, nb_rows);

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    free(poses);
    return NULL;
  }

  // Loop through data
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;

  // Loop through data line by line
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    // Ignore if comment line
    if (line[0] == '#') {
      continue;
    }

    // Iterate through values in line separated by commas
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        if (parse_pose_data(row_idx, col_idx, entry, poses) != 0) {
          return NULL;
        }
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;

      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  // Clean up
  fclose(infile);

  return poses;
}

/**
 * Associate pose data
 */
int **assoc_pose_data(pose_t *gnd_poses,
                      size_t nb_gnd_poses,
                      pose_t *est_poses,
                      size_t nb_est_poses,
                      double threshold,
                      size_t *nb_matches) {
  assert(gnd_poses != NULL);
  assert(est_poses != NULL);
  assert(nb_gnd_poses != 0);
  assert(nb_est_poses != 0);

  size_t gnd_idx = 0;
  size_t est_idx = 0;
  size_t k_end = (nb_gnd_poses > nb_est_poses) ? nb_est_poses : nb_gnd_poses;

  size_t match_idx = 0;
  int **matches = MALLOC(int *, k_end);

  while ((gnd_idx + 1) < nb_gnd_poses && (est_idx + 1) < nb_est_poses) {
    // Calculate time difference between ground truth and estimate
    double gnd_k_time = ts2sec(gnd_poses[gnd_idx].ts);
    double est_k_time = ts2sec(est_poses[est_idx].ts);
    double t_k_diff = fabs(gnd_k_time - est_k_time);

    // Check to see if next ground truth timestamp forms a smaller time diff
    double t_kp1_diff = threshold;
    if ((gnd_idx + 1) < nb_gnd_poses) {
      double gnd_kp1_time = ts2sec(gnd_poses[gnd_idx + 1].ts);
      t_kp1_diff = fabs(gnd_kp1_time - est_k_time);
    }

    // Conditions to call this pair (ground truth and estimate) a match
    int threshold_met = t_k_diff < threshold;
    int smallest_diff = t_k_diff < t_kp1_diff;

    // Mark pairs as a match or increment appropriate indicies
    if (threshold_met && smallest_diff) {
      matches[match_idx] = MALLOC(int, 2);
      matches[match_idx][0] = gnd_idx;
      matches[match_idx][1] = est_idx;
      match_idx++;

      gnd_idx++;
      est_idx++;

    } else if (gnd_k_time > est_k_time) {
      est_idx++;

    } else if (gnd_k_time < est_k_time) {
      gnd_idx++;
    }
  }

  // Clean up
  if (match_idx == 0) {
    free(matches);
    matches = NULL;
  }

  *nb_matches = match_idx;
  return matches;
}

/******************************************************************************
 * SIM
 ******************************************************************************/

// SIM FEATURES //////////////////////////////////////////////////////////////

/**
 * Load simulation feature data
 */
sim_features_t *sim_features_load(const char *csv_path) {
  sim_features_t *features_data = MALLOC(sim_features_t, 1);
  int nb_rows = 0;
  int nb_cols = 0;
  features_data->features = csv_data(csv_path, &nb_rows, &nb_cols);
  features_data->nb_features = nb_rows;
  return features_data;
}

/**
 * Free simulation feature data
 */
void sim_features_free(sim_features_t *feature_data) {
  // Pre-check
  if (feature_data == NULL) {
    return;
  }

  // Free data
  for (int i = 0; i < feature_data->nb_features; i++) {
    free(feature_data->features[i]);
  }
  free(feature_data->features);
  free(feature_data);
}

// SIM IMU DATA //////////////////////////////////////////////////////////////

/**
 * Load simulation imu data
 */
sim_imu_data_t *sim_imu_data_load(const char *csv_path) {
  sim_imu_data_t *imu_data = MALLOC(sim_imu_data_t, 1);

  int nb_rows = 0;
  int nb_cols = 0;
  imu_data->data = csv_data(csv_path, &nb_rows, &nb_cols);
  imu_data->nb_measurements = nb_rows;

  return imu_data;
}

/**
 * Free simulation imu data
 */
void sim_imu_data_free(sim_imu_data_t *imu_data) {
  // Pre-check
  if (imu_data == NULL) {
    return;
  }

  // Free data
  for (int i = 0; i < imu_data->nb_measurements; i++) {
    free(imu_data->data[i]);
  }
  free(imu_data->data);
  free(imu_data);
}

// SIM CAMERA DATA ///////////////////////////////////////////////////////////

/**
 * Extract timestamp from path
 */
static timestamp_t ts_from_path(const char *path) {
  char fname[128] = {0};
  char fext[128] = {0};
  path_file_name(path, fname);
  path_file_ext(path, fext);

  char ts_str[128] = {0};
  memcpy(ts_str, fname, strlen(fname) - strlen(fext) - 1);

  char *ptr;
  return strtol(ts_str, &ptr, 10);
}

/**
 * Load simulated camera frame
 */
sim_camera_frame_t *sim_camera_frame_load(const char *csv_path) {
  // Check if file exists
  if (file_exists(csv_path) == 0) {
    return NULL;
  }

  // Load csv data
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **data = csv_data(csv_path, &nb_rows, &nb_cols);

  // Create sim_camera_frame_t
  sim_camera_frame_t *frame_data = MALLOC(sim_camera_frame_t, 1);
  frame_data->ts = ts_from_path(csv_path);
  frame_data->feature_ids = MALLOC(int, nb_rows);
  frame_data->keypoints = MALLOC(real_t *, nb_rows);
  frame_data->nb_measurements = nb_rows;
  for (int i = 0; i < nb_rows; i++) {
    frame_data->feature_ids[i] = (int) data[i][0];
    frame_data->keypoints[i] = MALLOC(real_t, 2);
    frame_data->keypoints[i][0] = data[i][1];
    frame_data->keypoints[i][1] = data[i][2];
  }

  // Clean up
  csv_free(data, nb_rows);

  return frame_data;
}

/**
 * Print camera frame
 */
void sim_camera_frame_print(sim_camera_frame_t *frame_data) {
  printf("ts: %ld\n", frame_data->ts);
  printf("nb_frames: %d\n", frame_data->nb_measurements);
  for (int i = 0; i < frame_data->nb_measurements; i++) {
    const int feature_id = frame_data->feature_ids[i];
    const real_t *kp = frame_data->keypoints[i];
    printf("- ");
    printf("feature_id: [%d], ", feature_id);
    printf("kp: [%.2f, %.2f]\n", kp[0], kp[1]);
  }
  printf("\n");
}

/**
 * Free simulated camera frame
 */
void sim_camera_frame_free(sim_camera_frame_t *frame_data) {
  // Pre-check
  if (frame_data == NULL) {
    return;
  }

  // Free data
  free(frame_data->feature_ids);
  for (int i = 0; i < frame_data->nb_measurements; i++) {
    free(frame_data->keypoints[i]);
  }
  free(frame_data->keypoints);
  free(frame_data);
}

/**
 * Load simulated camera data
 */
sim_camera_data_t *sim_camera_data_load(const char *dir_path) {
  assert(dir_path != NULL);

  // Form csv file path
  char *csv_path = path_join(dir_path, "/data.csv");
  if (file_exists(csv_path) == 0) {
    free(csv_path);
    return NULL;
  }

  // Check number of rows
  const int nb_rows = dsv_rows(csv_path);
  if (nb_rows == 0) {
    free(csv_path);
    return NULL;
  }

  // Open csv file
  FILE *csv_file = fopen(csv_path, "r");
  if (csv_file == NULL) {
    free(csv_path);
    return NULL;
  }

  // Form sim_camera_frame_t
  sim_camera_data_t *cam_data = MALLOC(sim_camera_data_t, 1);
  cam_data->frames = MALLOC(sim_camera_frame_t *, nb_rows);
  cam_data->nb_frames = nb_rows;
  cam_data->ts = MALLOC(timestamp_t, nb_rows);
  cam_data->poses = MALLOC(real_t *, nb_rows);

  int line_idx = 0;
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
    // Skip line if its a comment
    if (line[0] == '#') {
      continue;
    }

    // Parse line
    timestamp_t ts;
    double r[3] = {0};
    double q[4] = {0};
    sscanf(line,
           "%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
           &ts,
           &r[0],
           &r[1],
           &r[2],
           &q[0],
           &q[1],
           &q[2],
           &q[3]);

    // Add camera frame to sim_camera_frame_t
    char fname[128] = {0};
    sprintf(fname, "/data/%ld.csv", ts);
    char *frame_csv = path_join(dir_path, fname);
    cam_data->frames[line_idx] = sim_camera_frame_load(frame_csv);
    free(frame_csv);

    // Add pose to sim_camera_frame_t
    cam_data->ts[line_idx] = ts;
    cam_data->poses[line_idx] = MALLOC(real_t, 7);
    cam_data->poses[line_idx][0] = r[0];
    cam_data->poses[line_idx][1] = r[1];
    cam_data->poses[line_idx][2] = r[2];
    cam_data->poses[line_idx][3] = q[0];
    cam_data->poses[line_idx][4] = q[1];
    cam_data->poses[line_idx][5] = q[2];
    cam_data->poses[line_idx][6] = q[3];

    // Update
    line_idx++;
  }

  // Clean up
  free(csv_path);
  fclose(csv_file);

  return cam_data;
}

/**
 * Free simulated camera data
 */
void sim_camera_data_free(sim_camera_data_t *cam_data) {
  // Pre-check
  if (cam_data == NULL) {
    return;
  }

  // Free data
  for (int k = 0; k < cam_data->nb_frames; k++) {
    sim_camera_frame_free(cam_data->frames[k]);
    free(cam_data->poses[k]);
  }
  free(cam_data->frames);
  free(cam_data->ts);
  free(cam_data->poses);
  free(cam_data);
}

/**
 * Simulate 3D features
 */
real_t **sim_create_features(const real_t origin[3],
                             const real_t dim[3],
                             const int nb_features) {
  assert(origin != NULL);
  assert(dim != NULL);
  assert(nb_features > 0);

  // Setup
  const real_t w = dim[0];
  const real_t l = dim[1];
  const real_t h = dim[2];
  const int features_per_side = nb_features / 4.0;
  int feature_idx = 0;
  real_t **features = MALLOC(real_t *, nb_features);

  // Features in the east side
  {
    const real_t x_bounds[2] = {origin[0] - w, origin[0] + w};
    const real_t y_bounds[2] = {origin[1] + l, origin[1] + l};
    const real_t z_bounds[2] = {origin[2] - h, origin[2] + h};
    for (int i = 0; i < features_per_side; i++) {
      features[feature_idx] = MALLOC(real_t, 3);
      features[feature_idx][0] = randf(x_bounds[0], x_bounds[1]);
      features[feature_idx][1] = randf(y_bounds[0], y_bounds[1]);
      features[feature_idx][2] = randf(z_bounds[0], z_bounds[1]);
      feature_idx++;
    }
  }

  // Features in the north side
  {
    const real_t x_bounds[2] = {origin[0] + w, origin[0] + w};
    const real_t y_bounds[2] = {origin[1] - l, origin[1] + l};
    const real_t z_bounds[2] = {origin[2] - h, origin[2] + h};
    for (int i = 0; i < features_per_side; i++) {
      features[feature_idx] = MALLOC(real_t, 3);
      features[feature_idx][0] = randf(x_bounds[0], x_bounds[1]);
      features[feature_idx][1] = randf(y_bounds[0], y_bounds[1]);
      features[feature_idx][2] = randf(z_bounds[0], z_bounds[1]);
      feature_idx++;
    }
  }

  // Features in the west side
  {
    const real_t x_bounds[2] = {origin[0] - w, origin[0] + w};
    const real_t y_bounds[2] = {origin[1] - l, origin[1] - l};
    const real_t z_bounds[2] = {origin[2] - h, origin[2] + h};
    for (int i = 0; i < features_per_side; i++) {
      features[feature_idx] = MALLOC(real_t, 3);
      features[feature_idx][0] = randf(x_bounds[0], x_bounds[1]);
      features[feature_idx][1] = randf(y_bounds[0], y_bounds[1]);
      features[feature_idx][2] = randf(z_bounds[0], z_bounds[1]);
      feature_idx++;
    }
  }

  // Features in the south side
  {
    const real_t x_bounds[2] = {origin[0] - w, origin[0] - w};
    const real_t y_bounds[2] = {origin[1] - l, origin[1] + l};
    const real_t z_bounds[2] = {origin[2] - h, origin[2] + h};
    for (int i = 0; i < features_per_side; i++) {
      features[feature_idx] = MALLOC(real_t, 3);
      features[feature_idx][0] = randf(x_bounds[0], x_bounds[1]);
      features[feature_idx][1] = randf(y_bounds[0], y_bounds[1]);
      features[feature_idx][2] = randf(z_bounds[0], z_bounds[1]);
      feature_idx++;
    }
  }

  return features;
}

// void sim_circle_trajectory() {
//   real_t circle_r = 1.0;
//   real_t circle_v = 1.0;
//   real_t cam_rate = 10.0;
//   real_t imu_rate = 200.0;
//   int nb_features = 1000;

//   // Trajectory data
//   real_t g[3] = {0.0, 0.0, 9.81};
//   real_t circle_dist = 2.0 * M_PI * circle_r;
//   real_t time_taken = circle_dist / circle_v;
//   real_t w = -2.0 * M_PI * (1.0 / time_taken);
//   real_t theta_init = M_PI;
//   real_t yaw_init = M_PI / 2.0;
// }

/******************************************************************************
 * GUI
 *****************************************************************************/
#ifdef USE_GUI

// OPENGL UTILS //////////////////////////////////////////////////////////////

GLfloat gl_deg2rad(const GLfloat d) {
  return d * M_PI / 180.0f;
}

GLfloat gl_rad2deg(const GLfloat r) {
  return r * 180.0f / M_PI;
}

void gl_print_vector(const char *prefix, const GLfloat *x, const int length) {
  printf("%s: [", prefix);
  for (int i = 0; i < length; i++) {
    printf("%f", x[i]);
    if ((i + 1) != length) {
      printf(", ");
    }
  }
  printf("]\n");
}

void gl_print_matrix(const char *prefix,
                     const GLfloat *A,
                     const int nb_rows,
                     const int nb_cols) {
  printf("%s:\n", prefix);
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_cols; j++) {
      printf("%f", A[i + (j * nb_rows)]);
      if ((j + 1) != nb_cols) {
        printf(", ");
      }
    }
    printf("\n");
  }
  printf("\n");
}

void gl_zeros(GLfloat *A, const int nb_rows, const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] = 0.0f;
  }
}

void gl_ones(GLfloat *A, const int nb_rows, const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] = 1.0f;
  }
}

void gl_eye(GLfloat *A, const int nb_rows, const int nb_cols) {
  int idx = 0;
  for (int j = 0; j < nb_cols; j++) {
    for (int i = 0; i < nb_rows; i++) {
      A[idx++] = (i == j) ? 1.0f : 0.0f;
    }
  }
}

void gl_vec2f(GLfloat *v, const GLfloat x, const GLfloat y) {
  v[0] = x;
  v[1] = y;
}

void gl_vec3f(GLfloat *v, const GLfloat x, const GLfloat y, const GLfloat z) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
}

void gl_vec4f(GLfloat *v,
              const GLfloat x,
              const GLfloat y,
              const GLfloat z,
              const GLfloat w) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
  v[3] = w;
}

int gl_equals(const GLfloat *A,
              const GLfloat *B,
              const int nb_rows,
              const int nb_cols,
              const GLfloat tol) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    if (fabs(A[i] - B[i]) > tol) {
      return 0;
    }
  }

  return 1;
}

void gl_matf_set(GLfloat *A,
                 const int m,
                 const int n,
                 const int i,
                 const int j,
                 const GLfloat val) {
  UNUSED(n);
  A[i + (j * m)] = val;
}

GLfloat gl_matf_val(
    const GLfloat *A, const int m, const int n, const int i, const int j) {
  UNUSED(n);
  return A[i + (j * m)];
}

void gl_copy(const GLfloat *src, const int m, const int n, GLfloat *dest) {
  for (int i = 0; i < (m * n); i++) {
    dest[i] = src[i];
  }
}

void gl_transpose(const GLfloat *A, size_t m, size_t n, GLfloat *A_t) {
  assert(A != NULL && A != A_t);
  assert(m > 0 && n > 0);

  int idx = 0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A_t[idx++] = gl_matf_val(A, m, n, i, j);
    }
  }
}

void gl_vec3f_cross(const GLfloat u[3], const GLfloat v[3], GLfloat n[3]) {
  assert(u);
  assert(v);
  assert(n);

  n[0] = u[1] * v[2] - u[2] * v[1];
  n[1] = u[2] * v[0] - u[0] * v[2];
  n[2] = u[0] * v[1] - u[1] * v[0];
}

void gl_add(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    C[i] = A[i] + B[i];
  }
}

void gl_sub(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    C[i] = A[i] - B[i];
  }
}

void gl_dot(const GLfloat *A,
            const int A_m,
            const int A_n,
            const GLfloat *B,
            const int B_m,
            const int B_n,
            GLfloat *C) {
  assert(A != C && B != C);
  assert(A_n == B_m);

  int m = A_m;
  int n = B_n;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      for (int k = 0; k < A_n; k++) {
        C[i + (j * n)] += A[i + (k * A_n)] * B[k + (j * B_n)];
      }
    }
  }
}

void gl_scale(GLfloat factor,
              GLfloat *A,
              const int nb_rows,
              const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] *= factor;
  }
}

GLfloat gl_norm(const GLfloat *x, const int size) {
  GLfloat sum_sq = 0.0f;
  for (int i = 0; i < size; i++) {
    sum_sq += x[i] * x[i];
  }

  return sqrt(sum_sq);
}

void gl_normalize(GLfloat *x, const int size) {
  const GLfloat n = gl_norm(x, size);
  for (int i = 0; i < size; i++) {
    x[i] /= n;
  }
}

void gl_perspective(const GLfloat fov,
                    const GLfloat aspect,
                    const GLfloat near,
                    const GLfloat far,
                    GLfloat P[4 * 4]) {
  const GLfloat f = 1.0f / tan(fov * 0.5f);

  gl_zeros(P, 4, 4);
  P[0] = f / aspect;
  P[1] = 0.0f;
  P[2] = 0.0f;
  P[3] = 0.0f;

  P[4] = 0.0f;
  P[5] = f;
  P[6] = 0.0f;
  P[7] = 0.0f;

  P[8] = 0.0f;
  P[9] = 0.0f;
  P[10] = (far + near) / (near - far);
  P[11] = -1;

  P[12] = 0.0f;
  P[13] = 0.0f;
  P[14] = (2 * far * near) / (near - far);
  P[15] = 0.0f;
}

void gl_lookat(const GLfloat eye[3],
               const GLfloat at[3],
               const GLfloat up[3],
               GLfloat V[4 * 4]) {
  // Z-axis: Camera forward
  GLfloat z[3] = {0};
  gl_sub(at, eye, 3, 1, z);
  gl_normalize(z, 3);

  // X-axis: Camera right
  GLfloat x[3] = {0};
  gl_vec3f_cross(z, up, x);
  gl_normalize(x, 3);

  // Y-axis: Camera up
  GLfloat y[3] = {0};
  gl_vec3f_cross(x, z, y);

  // Negate z-axis
  gl_scale(-1.0f, z, 3, 1);

  // Form rotation component
  GLfloat R[4 * 4] = {0};
  R[0] = x[0];
  R[1] = y[0];
  R[2] = z[0];
  R[3] = 0.0f;

  R[4] = x[1];
  R[5] = y[1];
  R[6] = z[1];
  R[7] = 0.0f;

  R[8] = x[2];
  R[9] = y[2];
  R[10] = z[2];
  R[11] = 0.0f;

  R[12] = 0.0f;
  R[13] = 0.0f;
  R[14] = 0.0f;
  R[15] = 1.0f;

  // Form translation component
  GLfloat T[4 * 4] = {0};
  T[0] = 1.0f;
  T[1] = 0.0f;
  T[2] = 0.0f;
  T[3] = 0.0f;

  T[4] = 0.0f;
  T[5] = 1.0f;
  T[6] = 0.0f;
  T[7] = 0.0f;

  T[8] = 0.0f;
  T[9] = 0.0f;
  T[10] = 1.0f;
  T[11] = 0.0f;

  T[12] = -eye[0];
  T[13] = -eye[1];
  T[14] = -eye[2];
  T[15] = 1.0f;

  // Form view matrix
  gl_zeros(V, 4, 4);
  gl_dot(R, 4, 4, T, 4, 4, V);
}

// SHADER ////////////////////////////////////////////////////////////////////

GLuint gl_shader_compile(const char *shader_src, const int type) {
  if (shader_src == NULL) {
    LOG_ERROR("Shader source is NULL!");
    return GL_FALSE;
  }

  const GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &shader_src, NULL);
  glCompileShader(shader);

  GLint retval = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &retval);
  if (retval == GL_FALSE) {
    char log[9046] = {0};
    glGetShaderInfoLog(shader, 9046, NULL, log);
    LOG_ERROR("Failed to compile shader:\n%s", log);
    return retval;
  }

  return shader;
}

GLuint gl_shaders_link(const GLuint vertex_shader,
                       const GLuint fragment_shader,
                       const GLuint geometry_shader) {
  // Attach shaders to link
  GLuint program = glCreateProgram();
  glAttachShader(program, vertex_shader);
  glAttachShader(program, fragment_shader);
  if (geometry_shader != GL_FALSE) {
    glAttachShader(program, geometry_shader);
  }
  glLinkProgram(program);

  // Link program
  GLint success = 0;
  char log[9046];
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (success == GL_FALSE) {
    glGetProgramInfoLog(program, 9046, NULL, log);
    LOG_ERROR("Failed to link shaders:\nReason: %s\n", log);
    return GL_FALSE;
  }

  // Delete shaders
  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);
  if (geometry_shader == GL_FALSE) {
    glDeleteShader(geometry_shader);
  }

  return program;
}

// GL PROGRAM ////////////////////////////////////////////////////////////////

GLuint gl_prog_setup(const char *vs_src,
                     const char *fs_src,
                     const char *gs_src) {
  GLuint vs = GL_FALSE;
  GLuint fs = GL_FALSE;
  GLuint gs = GL_FALSE;

  if (vs_src) {
    vs = gl_shader_compile(vs_src, GL_VERTEX_SHADER);
  }

  if (fs_src) {
    fs = gl_shader_compile(fs_src, GL_FRAGMENT_SHADER);
  }

  if (gs_src) {
    gs = gl_shader_compile(gs_src, GL_GEOMETRY_SHADER);
  }

  const GLuint program_id = gl_shaders_link(vs, fs, gs);
  return program_id;
}

int gl_prog_set_int(const GLint id, const char *k, const GLint v) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform1i(location, v);
  return 0;
}

int gl_prog_set_vec2i(const GLint id, const char *k, const GLint v[2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform2i(location, v[0], v[1]);
  return 0;
}

int gl_prog_set_vec3i(const GLint id, const char *k, const GLint v[3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform3i(location, v[0], v[1], v[2]);
  return 0;
}

int gl_prog_set_vec4i(const GLint id, const char *k, const GLint v[4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform4i(location, v[0], v[1], v[2], v[3]);
  return 0;
}

int gl_prog_set_float(const GLint id, const char *k, const GLfloat v) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform1f(location, v);
  return 0;
}

int gl_prog_set_vec2f(const GLint id, const char *k, const GLfloat v[2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform2f(location, v[0], v[1]);
  return 0;
}

int gl_prog_set_vec3f(const GLint id, const char *k, const GLfloat v[3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform3f(location, v[0], v[1], v[2]);
  return 0;
}

int gl_prog_set_vec4f(const GLint id, const char *k, const GLfloat v[4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform4f(location, v[0], v[1], v[2], v[3]);
  return 0;
}

int gl_prog_set_mat2f(const GLint id, const char *k, const GLfloat v[2 * 2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix2fv(location, 1, GL_FALSE, v);
  return 0;
}

int gl_prog_set_mat3f(const GLint id, const char *k, const GLfloat v[3 * 3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix3fv(location, 1, GL_FALSE, v);
  return 0;
}

int gl_prog_set_mat4f(const GLint id, const char *k, const GLfloat v[4 * 4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix4fv(location, 1, GL_FALSE, v);
  return 0;
}

// GL-CAMERA /////////////////////////////////////////////////////////////////

void gl_camera_setup(gl_camera_t *camera,
                     int *window_width,
                     int *window_height) {
  camera->window_width = window_width;
  camera->window_height = window_height;

  gl_zeros(camera->focal, 3, 1);
  gl_vec3f(camera->world_up, 0.0f, 1.0f, 0.0f);
  gl_vec3f(camera->position, 0.0f, 0.0f, 0.0f);
  gl_vec3f(camera->right, -1.0f, 0.0f, 0.0f);
  gl_vec3f(camera->up, 0.0f, 1.0f, 0.0f);
  gl_vec3f(camera->front, 0.0f, 0.0f, -1.0f);
  camera->yaw = gl_deg2rad(0.0f);
  camera->pitch = gl_deg2rad(0.0f);
  camera->radius = 10.0f;

  camera->fov = gl_deg2rad(60.0f);
  camera->near = 0.1f;
  camera->far = 100.0f;

  gl_camera_update(camera);
}

void gl_camera_update(gl_camera_t *camera) {
  // Front vector
  camera->front[0] = sin(camera->yaw) * cos(camera->pitch);
  camera->front[1] = sin(camera->pitch);
  camera->front[2] = cos(camera->yaw) * cos(camera->pitch);
  gl_normalize(camera->front, 3);

  // Right vector
  gl_vec3f_cross(camera->front, camera->world_up, camera->right);
  gl_normalize(camera->right, 3);

  // Up vector
  gl_vec3f_cross(camera->right, camera->front, camera->up);
  gl_normalize(camera->up, 3);

  // Projection matrix
  const float width = (float) *(camera->window_width);
  const float height = (float) *(camera->window_height);
  const float aspect = width / height;
  gl_perspective(camera->fov, aspect, camera->near, camera->far, camera->P);

  // View matrix
  GLfloat eye[3] = {0};
  eye[0] = camera->focal[0] + camera->radius * sin(camera->yaw);
  eye[1] = camera->focal[1] + camera->radius * cos(camera->pitch);
  eye[2] = camera->focal[2] + camera->radius * cos(camera->yaw);
  gl_lookat(eye, camera->focal, camera->world_up, camera->V);
}

void gl_camera_rotate(gl_camera_t *camera,
                      const float factor,
                      const float dx,
                      const float dy) {
  // Update yaw and pitch
  float pitch = camera->pitch;
  float yaw = camera->yaw;
  yaw -= dx * factor;
  pitch += dy * factor;

  // Constrain pitch and yaw
  pitch = (pitch <= (-M_PI / 2.0) + 1e-5) ? (-M_PI / 2.0) + 1e-5 : pitch;
  pitch = (pitch > 0.0) ? 0.0 : pitch;
  yaw = (yaw > M_PI) ? yaw - 2 * M_PI : yaw;
  yaw = (yaw < -M_PI) ? yaw + 2 * M_PI : yaw;

  // Update camera attitude
  camera->pitch = pitch;
  camera->yaw = yaw;
  gl_camera_update(camera);
}

void gl_camera_pan(gl_camera_t *camera,
                   const float factor,
                   const float dx,
                   const float dy) {
  // camera->focal -= (dy * mouse_sensitivity) * camera->front;
  // camera->focal += (dx * mouse_sensitivity) * camera->right;
  const GLfloat dx_scaled = dx * factor;
  const GLfloat dy_scaled = dy * factor;
  GLfloat front[3] = {camera->front[0], camera->front[1], camera->front[2]};
  GLfloat right[3] = {camera->right[0], camera->right[1], camera->right[2]};
  gl_scale(dy_scaled, front, 3, 1);
  gl_scale(dx_scaled, right, 3, 1);
  gl_sub(camera->focal, front, 3, 1, camera->focal);
  gl_add(camera->focal, right, 3, 1, camera->focal);

  // limit focal point y-axis
  camera->focal[1] = (camera->focal[1] < 0) ? 0 : camera->focal[1];
  gl_camera_update(camera);
}

void gl_camera_zoom(gl_camera_t *camera,
                    const float factor,
                    const float dx,
                    const float dy) {
  UNUSED(factor);
  UNUSED(dx);

  if (camera->fov >= gl_deg2rad(0.5f) && camera->fov <= gl_deg2rad(90.0f)) {
    camera->fov -= dy * 0.1;
  }

  if (camera->fov <= gl_deg2rad(0.5f)) {
    camera->fov = gl_deg2rad(5.0f);
  } else if (camera->fov >= gl_deg2rad(90.0f)) {
    camera->fov = gl_deg2rad(90.0f);
  }

  gl_camera_update(camera);
}

// GL-PRIMITIVES /////////////////////////////////////////////////////////////

// GL CUBE ///////////////////////////////////////////////////////////////////

void gl_cube_setup(gl_entity_t *entity, GLfloat pos[3]) {
  // Entity transform
  gl_eye(entity->T, 4, 4);
  entity->T[12] = pos[0];
  entity->T[13] = pos[1];
  entity->T[14] = pos[2];

  // Shader program
  char *vs = file_read("./shaders/cube.vert");
  char *fs = file_read("./shaders/cube.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Vertices
  // clang-format off
  const float color[3] = {0.9, 0.4, 0.2};
  const float cube_size = 0.5;
  const float r = color[0];
  const float g = color[1];
  const float b = color[2];
  GLfloat vertices[12 * 3 * 6] = {
    // Triangle 1
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    // Triangle 2
    cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 3
    cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 4
    cube_size, cube_size, -cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 5
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 6
    cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 7
    -cube_size, cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b,
    // Triangle 8
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 9
    cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b,
    // Triangle 10
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 11
    cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    // Triangle 12
    cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b
    // Triangle 12 : end
  };
  const size_t nb_triangles = 12;
  const size_t vertices_per_triangle = 3;
  const size_t nb_vertices = vertices_per_triangle * nb_triangles;
  const size_t vertex_buffer_size = sizeof(float) * 6 * nb_vertices;
  // clang-format on

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, vertex_buffer_size, vertices, GL_STATIC_DRAW);
  // -- Position attribute
  size_t vertex_size = 6 * sizeof(float);
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *) (3 * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, color_offset);
  glEnableVertexAttribArray(1);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

void gl_cube_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_cube_draw(const gl_entity_t *entity, const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  // 12 x 3 indices starting at 0 -> 12 triangles -> 6 squares
  glBindVertexArray(entity->vao);
  glDrawArrays(GL_TRIANGLES, 0, 36);
  glBindVertexArray(0); // Unbind VAO
}

// GL CAMERA FRAME ///////////////////////////////////////////////////////////

void gl_camera_frame_setup(gl_entity_t *entity) {
  // Entity transform
  gl_eye(entity->T, 4, 4);

  // Shader program
  char *vs = file_read("./shaders/camera_frame.vert");
  char *fs = file_read("./shaders/camera_frame.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Form the camera fov frame
  GLfloat fov = gl_deg2rad(60.0);
  GLfloat hfov = fov / 2.0f;
  GLfloat scale = 1.0f;
  GLfloat z = scale;
  GLfloat hwidth = z * tan(hfov);
  const GLfloat lb[3] = {-hwidth, hwidth, z};  // Left bottom
  const GLfloat lt[3] = {-hwidth, -hwidth, z}; // Left top
  const GLfloat rt[3] = {hwidth, -hwidth, z};  // Right top
  const GLfloat rb[3] = {hwidth, hwidth, z};   // Right bottom

  // Rectangle frame
  // clang-format off
  const GLfloat vertices[] = {
    // -- Left bottom to left top
    lb[0], lb[1], lb[2], lt[0], lt[1], lt[2],
    // -- Left top to right top
    lt[0], lt[1], lt[2], rt[0], rt[1], rt[2],
    // -- Right top to right bottom
    rt[0], rt[1], rt[2], rb[0], rb[1], rb[2],
    // -- Right bottom to left bottom
    rb[0], rb[1], rb[2], lb[0], lb[1], lb[2],
    // Rectangle frame to origin
    // -- Origin to left bottom
    0.0f, 0.0f, 0.0f, lb[0], lb[1], lb[2],
    // -- Origin to left top
    0.0f, 0.0f, 0.0f, lt[0], lt[1], lt[2],
    // -- Origin to right top
    0.0f, 0.0f, 0.0f, rt[0], rt[1], rt[2],
    // -- Origin to right bottom
    0.0f, 0.0f, 0.0f, rb[0], rb[1], rb[2]
  };
  // clang-format on
  const size_t nb_lines = 8;
  const size_t nb_vertices = nb_lines * 2;
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        3 * sizeof(float),
                        (void *) 0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

void gl_camera_frame_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_camera_frame_draw(const gl_entity_t *entity,
                          const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  // Store original line width
  GLfloat original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  GLfloat line_width = 2.0f;
  glLineWidth(line_width);

  // Draw frame
  const size_t nb_lines = 8;
  const size_t nb_vertices = nb_lines * 2;
  glBindVertexArray(entity->vao);
  glDrawArrays(GL_LINES, 0, nb_vertices);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

// GL AXIS FRAME /////////////////////////////////////////////////////////////

void gl_axis_frame_setup(gl_entity_t *entity) {
  // Entity transform
  gl_eye(entity->T, 4, 4);

  // Shader program
  char *vs = file_read("./shaders/axis_frame.vert");
  char *fs = file_read("./shaders/axis_frame.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Vertices
  // clang-format off
  static const GLfloat vertices[] = {
    // Line 1 : x-axis + color
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    // Line 2 : y-axis + color
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    // Line 3 : z-axis + color
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f
  };
  const size_t nb_vertices = 6;
  const size_t buffer_size = sizeof(GLfloat) * 6 * nb_vertices;
  // clang-format on

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  // -- Position attribute
  size_t vertex_size = 6 * sizeof(float);
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *) (3 * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, color_offset);
  glEnableVertexAttribArray(1);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

void gl_axis_frame_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_axis_frame_draw(const gl_entity_t *entity, const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  // Store original line width
  GLfloat original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  GLfloat line_width = 6.0f;
  glLineWidth(line_width);

  // Draw frame
  glBindVertexArray(entity->vao);
  glDrawArrays(GL_LINES, 0, 6);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

static GLfloat *glgrid_create_vertices(int grid_size) {
  // Allocate memory for vertices
  int nb_lines = (grid_size + 1) * 2;
  int nb_vertices = nb_lines * 2;
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;
  GLfloat *vertices = (GLfloat *) malloc(buffer_size);

  // Setup
  const float min_x = -1.0f * (float) grid_size / 2.0f;
  const float max_x = (float) grid_size / 2.0f;
  const float min_z = -1.0f * (float) grid_size / 2.0f;
  const float max_z = (float) grid_size / 2.0f;

  // Row vertices
  float z = min_z;
  int vert_idx = 0;
  for (int i = 0; i < ((grid_size + 1) * 2); i++) {
    if ((i % 2) == 0) {
      vertices[(vert_idx * 3)] = min_x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = z;
    } else {
      vertices[(vert_idx * 3)] = max_x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = z;
      z += 1.0f;
    }
    vert_idx++;
  }

  // Column vertices
  float x = min_x;
  for (int j = 0; j < ((grid_size + 1) * 2); j++) {
    if ((j % 2) == 0) {
      vertices[(vert_idx * 3)] = x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = min_z;
    } else {
      vertices[(vert_idx * 3)] = x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = max_z;
      x += 1.0f;
    }
    vert_idx++;
  }

  return vertices;
}

// GL GRID ///////////////////////////////////////////////////////////////////

void gl_grid_setup(gl_entity_t *entity) {
  // Entity transform
  gl_eye(entity->T, 4, 4);

  // Shader program
  char *vs = file_read("./shaders/grid.vert");
  char *fs = file_read("./shaders/grid.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Create vertices
  const int grid_size = 10;
  const int nb_lines = (grid_size + 1) * 2;
  const int nb_vertices = nb_lines * 2;
  GLfloat *vertices = glgrid_create_vertices(grid_size);
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        3 * sizeof(float),
                        (void *) 0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
  free(vertices);
}

void gl_grid_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_grid_draw(const gl_entity_t *entity, const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  const int grid_size = 10;
  const int nb_lines = (grid_size + 1) * 2;
  const int nb_vertices = nb_lines * 2;

  glBindVertexArray(entity->vao);
  glDrawArrays(GL_LINES, 0, nb_vertices);
  glBindVertexArray(0); // Unbind VAO
}

// GUI ///////////////////////////////////////////////////////////////////////

void gui_window_callback(gui_t *gui, const SDL_Event event) {
  if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
    const int width = event.window.data1;
    const int height = event.window.data2;
    gui->screen_width = width;
    gui->screen_height = (height > 0) ? height : 1;
    glViewport(0, 0, (GLsizei) gui->screen_width, (GLsizei) gui->screen_height);
  }
}

void gui_keyboard_callback(gui_t *gui, const SDL_Event event) {
  if (event.type == SDL_KEYDOWN) {
    switch (event.key.keysym.sym) {
      case SDLK_ESCAPE:
        gui->loop = 0;
        break;
      case SDLK_q:
        gui->loop = 0;
        break;
    }
  }
}

void gui_mouse_callback(gui_t *gui, const SDL_Event event) {
  const float x = event.motion.x;
  const float y = event.motion.y;
  const float dx = x - gui->last_cursor_x;
  const float dy = y - gui->last_cursor_y;
  gui->last_cursor_x = x;
  gui->last_cursor_y = y;

  gui->left_click = (event.button.button == SDL_BUTTON_LEFT);
  gui->right_click = (event.button.button == SDL_BUTTON_RIGHT ||
                      event.button.button == SDL_BUTTON_X1);

  if (gui->left_click) {
    // Rotate camera
    if (gui->last_cursor_set == 0) {
      gui->last_cursor_set = 1;
    } else if (gui->last_cursor_set) {
      gl_camera_rotate(&gui->camera, gui->mouse_sensitivity, dx, dy);
    }
  } else if (gui->right_click) {
    // Pan camera
    if (gui->last_cursor_set == 0) {
      gui->last_cursor_set = 1;
    } else if (gui->last_cursor_set) {
      gl_camera_pan(&gui->camera, gui->mouse_sensitivity, dx, dy);
    }
  } else if (event.wheel.type == SDL_MOUSEWHEEL && event.wheel.y) {
    gl_camera_zoom(&gui->camera, gui->mouse_sensitivity, 0, event.wheel.y);
  } else {
    // Reset cursor
    gui->left_click = 0;
    gui->right_click = 0;
    gui->last_cursor_set = 0;
    gui->last_cursor_x = 0.0;
    gui->last_cursor_y = 0.0;
  }
}

void gui_event_handler(gui_t *gui) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_WINDOWEVENT:
        gui_window_callback(gui, event);
        break;
      case SDL_KEYUP:
      case SDL_KEYDOWN:
        gui_keyboard_callback(gui, event);
        break;
      case SDL_MOUSEMOTION:
      case SDL_MOUSEBUTTONDOWN:
      case SDL_MOUSEBUTTONUP:
      case SDL_MOUSEWHEEL:
        gui_mouse_callback(gui, event);
        break;
    }
  }
}

void gui_setup(gui_t *gui) {
  // SDL init
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    FATAL("SDL_Init Error: %s/n", SDL_GetError());
  }

  // Get display size
  SDL_DisplayMode disp_mode;
  SDL_GetCurrentDisplayMode(0, &disp_mode);
  const int disp_w = disp_mode.w;
  const int disp_h = disp_mode.h;

  // Window
  const char *title = "Hello World!";
  const int w = 640;
  const int h = 480;
  const int x = disp_w / 2 - w / 2;
  const int y = disp_h / 2 - h / 2;
  const uint32_t flags = SDL_WINDOW_OPENGL;
  gui->window = SDL_CreateWindow(title, x, y, w, h, flags);
  if (gui->window == NULL) {
    FATAL("SDL_CreateWindow Error: %s/n", SDL_GetError());
  }
  SDL_SetWindowResizable(gui->window, 1);

  // OpenGL context
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_GLContext context = SDL_GL_CreateContext(gui->window);
  SDL_GL_SetSwapInterval(1);
  UNUSED(context);

  // GLEW
  GLenum err = glewInit();
  if (err != GLEW_OK) {
    FATAL("glewInit failed: %s", glewGetErrorString(err));
  }

  // Camera
  gl_camera_setup(&gui->camera, &gui->window_width, &gui->window_height);
  gui->movement_speed = 50.0f;
  gui->mouse_sensitivity = 0.02f;

  // Cursor
  gui->left_click = 0;
  gui->right_click = 0;
  gui->last_cursor_set = 0;
  gui->last_cursor_x = 0.0f;
  gui->last_cursor_y = 0.0f;
}

void gui_reset(gui_t *gui) {
  // Camera
  gui->movement_speed = 50.0f;
  gui->mouse_sensitivity = 0.02f;

  // Cursor
  gui->left_click = 0;
  gui->right_click = 0;
  gui->last_cursor_set = 0;
  gui->last_cursor_x = 0.0f;
  gui->last_cursor_y = 0.0f;
}

void gui_loop(gui_t *gui) {
  gl_entity_t cube;
  GLfloat cube_pos[3] = {0.0, 0.0, 0.0};
  gl_cube_setup(&cube, cube_pos);

  gl_entity_t cube2;
  GLfloat cube2_pos[3] = {2.0, 0.0, 0.0};
  gl_cube_setup(&cube2, cube2_pos);

  gl_entity_t cube3;
  GLfloat cube3_pos[3] = {-2.0, 0.0, 0.0};
  gl_cube_setup(&cube3, cube3_pos);

  gl_entity_t cf;
  gl_camera_frame_setup(&cf);

  gl_entity_t frame;
  gl_axis_frame_setup(&frame);

  gl_entity_t grid;
  gl_grid_setup(&grid);

  gui->loop = 1;
  while (gui->loop) {
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    gl_cube_draw(&cube, &gui->camera);
    gl_cube_draw(&cube2, &gui->camera);
    gl_cube_draw(&cube3, &gui->camera);

    gl_camera_frame_draw(&cf, &gui->camera);
    gl_axis_frame_draw(&frame, &gui->camera);
    gl_grid_draw(&grid, &gui->camera);

    gui_event_handler(gui);
    SDL_GL_SwapWindow(gui->window);
    SDL_Delay(1);
  }

  gl_cube_cleanup(&cube);
  gl_cube_cleanup(&cube2);
  gl_cube_cleanup(&cube3);
  gl_camera_frame_cleanup(&cf);
  gl_grid_cleanup(&grid);

  SDL_DestroyWindow(gui->window);
  SDL_Quit();
}

// IMSHOW ////////////////////////////////////////////////////////////////////

void imshow_window_callback(imshow_t *imshow, const SDL_Event event) {
  if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
    SDL_Surface *screen_surface = SDL_GetWindowSurface(imshow->window);

    Uint32 color = SDL_MapRGB(screen_surface->format, 0, 0, 0);
    SDL_FillRect(screen_surface, NULL, color);

    SDL_Rect stretch;
    stretch.x = 0;
    stretch.y = 0;
    stretch.w = event.window.data1;
    stretch.h = event.window.data2;
    SDL_BlitScaled(imshow->image_surface, NULL, screen_surface, &stretch);
  }
}

void imshow_keyboard_callback(imshow_t *imshow, const SDL_Event event) {
  if (event.type == SDL_KEYDOWN) {
    switch (event.key.keysym.sym) {
      case SDLK_ESCAPE:
        imshow->loop = 0;
        break;
      case SDLK_q:
        imshow->loop = 0;
        break;
    }
  }
}

void imshow_event_handler(imshow_t *imshow) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_WINDOWEVENT:
        imshow_window_callback(imshow, event);
        break;
      case SDL_KEYUP:
      case SDL_KEYDOWN:
        imshow_keyboard_callback(imshow, event);
        break;
    }
  }
}

void draw_circle(SDL_Renderer *renderer,
                 const int cx,
                 const int cy,
                 const int radius,
                 const SDL_Color color) {
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
  for (int x = cx - radius; x <= cx + radius; x++) {
    for (int y = cy - radius; y <= cy + radius; y++) {
      if ((pow(cy - y, 2) + pow(cx - x, 2)) <= pow(radius, 2)) {
        SDL_RenderDrawPoint(renderer, x, y);
      }
    }
  }
}

// void draw_circle(SDL_Renderer *renderer,
//                  int32_t centreX,
//                  int32_t centreY,
//                  int32_t radius,
//                  const SDL_Color color) {
//   SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
//   const int32_t diameter = (radius * 2);
//
//   int32_t x = (radius - 1);
//   int32_t y = 0;
//   int32_t tx = 1;
//   int32_t ty = 1;
//   int32_t error = (tx - diameter);
//
//   while (x >= y) {
//     //  Each of the following renders an octant of the circle
//     SDL_RenderDrawPoint(renderer, centreX + x, centreY - y);
//     SDL_RenderDrawPoint(renderer, centreX + x, centreY + y);
//     SDL_RenderDrawPoint(renderer, centreX - x, centreY - y);
//     SDL_RenderDrawPoint(renderer, centreX - x, centreY + y);
//     SDL_RenderDrawPoint(renderer, centreX + y, centreY - x);
//     SDL_RenderDrawPoint(renderer, centreX + y, centreY + x);
//     SDL_RenderDrawPoint(renderer, centreX - y, centreY - x);
//     SDL_RenderDrawPoint(renderer, centreX - y, centreY + x);
//
//     if (error <= 0) {
//       ++y;
//       error += ty;
//       ty += 2;
//     }
//
//     if (error > 0) {
//       --x;
//       tx += 2;
//       error += (tx - diameter);
//     }
//   }
// }

void imshow_setup(imshow_t *im, const char *fp) {
  // SDL init
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    FATAL("SDL_Init Error: %s/n", SDL_GetError());
  }

  // Load image
  im->image_surface = IMG_Load(fp);
  if (im->image_surface == NULL) {
    FATAL("Failed to load image [%s]!", fp);
  }
  const int img_w = im->image_surface->w;
  const int img_h = im->image_surface->h;

  // Get display size
  SDL_DisplayMode disp_mode;
  SDL_GetCurrentDisplayMode(0, &disp_mode);
  const int disp_w = disp_mode.w;
  const int disp_h = disp_mode.h;

  // Create window
  const int x = disp_w / 2 - img_w / 2;
  const int y = disp_h / 2 - img_h / 2;
  const int w = img_w;
  const int h = img_h;
  if (SDL_CreateWindowAndRenderer(w, h, 0, &im->window, &im->renderer) != 0) {
    FATAL("Failed to create window: %s\n", SDL_GetError());
  }
  SDL_SetWindowTitle(im->window, im->window_title);
  SDL_SetWindowPosition(im->window, x, y);
  SDL_SetWindowResizable(im->window, 1);

  // Clear render
  SDL_SetRenderDrawColor(im->renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(im->renderer);

  // Show image
  SDL_Texture *texture =
      SDL_CreateTextureFromSurface(im->renderer, im->image_surface);
  SDL_RenderCopy(im->renderer, texture, NULL, NULL);
  SDL_RenderPresent(im->renderer);

  // Draw circles
  const int x_min = 0;
  const int y_min = 0;
  const int x_max = img_w;
  const int y_max = img_h;
  for (int i = 0; i < 200; i++) {
    const int x = (rand() % (x_max + 1 - x_min)) + x_min;
    const int y = (rand() % (y_max + 1 - y_min)) + y_min;
    const int radius = 5;
    SDL_Color color;
    color.r = 255;
    color.g = 0;
    color.b = 0;
    color.a = 255;
    draw_circle(im->renderer, x, y, radius, color);
  }
  SDL_RenderPresent(im->renderer);

  // Cursor
  im->left_click = 0;
  im->right_click = 0;
  im->last_cursor_set = 0;
  im->last_cursor_x = 0.0f;
  im->last_cursor_y = 0.0f;
}

void imshow_reset(imshow_t *imshow) {
  // Camera
  imshow->movement_speed = 50.0f;
  imshow->mouse_sensitivity = 0.02f;

  // Cursor
  imshow->left_click = 0;
  imshow->right_click = 0;
  imshow->last_cursor_set = 0;
  imshow->last_cursor_x = 0.0f;
  imshow->last_cursor_y = 0.0f;
}

void imshow_loop(imshow_t *imshow) {
  imshow->loop = 1;
  while (imshow->loop) {
    imshow_event_handler(imshow);
    SDL_UpdateWindowSurface(imshow->window);
    SDL_Delay(1);
  }

  SDL_DestroyWindow(imshow->window);
  SDL_Quit();
}
#endif // USE_GUI

#endif // PROTO_IMPLEMENTATION

//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

#ifdef PROTO_UNITTEST

#include "munit.h"

/* TEST PARAMS */
#define TEST_DATA_PATH "./test_data/"
#define TEST_CSV TEST_DATA_PATH "test_csv.csv"
#define TEST_POSES_CSV TEST_DATA_PATH "poses.csv"
#define TEST_SIM_DATA TEST_DATA_PATH "sim_data"

/******************************************************************************
 * TEST MACROS
 ******************************************************************************/

int test_median_value() {
  real_t median = 0.0f;
  real_t buf[5] = {4.0, 1.0, 0.0, 3.0, 2.0};
  MEDIAN_VALUE(real_t, fltcmp2, buf, 5, median);
  MU_ASSERT(fltcmp(median, 2.0) == 0);

  return 0;
}

int test_mean_value() {
  real_t mean = 0.0f;
  real_t buf[5] = {0.0, 1.0, 2.0, 3.0, 4.0};
  MEAN_VALUE(real_t, buf, 5, mean);
  MU_ASSERT(fltcmp(mean, 2.0) == 0);

  return 0;
}

/******************************************************************************
 * TEST FILESYSTEM
 ******************************************************************************/

int test_path_file_name() {
  const char *path = "/tmp/hello_world.csv";
  char fname[128] = {0};
  path_file_name(path, fname);
  MU_ASSERT(strcmp(fname, "hello_world.csv") == 0);

  return 0;
}

int test_path_file_ext() {
  const char *path = "/tmp/hello_world.csv";
  char fext[128] = {0};
  path_file_ext(path, fext);
  MU_ASSERT(strcmp(fext, "csv") == 0);

  return 0;
}

int test_path_dir_name() {
  const char *path = "/tmp/hello_world.csv";
  char dir_name[128] = {0};
  path_dir_name(path, dir_name);
  MU_ASSERT(strcmp(dir_name, "/tmp") == 0);

  return 0;
}

int test_path_join() {
  // Case A
  {
    const char *path_a = "/tmp/A";
    const char *path_b = "B.csv";
    char *c = path_join(path_a, path_b);
    MU_ASSERT(strcmp(c, "/tmp/A/B.csv") == 0);
    free(c);
  }

  // Case B
  {
    const char *path_a = "/tmp/A/";
    const char *path_b = "B.csv";
    char *c = path_join(path_a, path_b);
    MU_ASSERT(strcmp(c, "/tmp/A/B.csv") == 0);
    free(c);
  }

  return 0;
}

int test_list_files() {
  int nb_files = 0;
  char **files = list_files("/tmp", &nb_files);
  MU_ASSERT(files != NULL);
  MU_ASSERT(nb_files != 0);

  /* printf("nb_files: %d\n", nb_files); */
  for (int i = 0; i < nb_files; i++) {
    /* printf("file: %s\n", files[i]); */
    free(files[i]);
  }
  free(files);

  return 0;
}

int test_list_files_free() {
  int nb_files = 0;
  char **files = list_files("/tmp", &nb_files);
  list_files_free(files, nb_files);

  return 0;
}

int test_file_read() {
  char *text = file_read("test_data/poses.csv");
  /* printf("%s\n", text); */
  MU_ASSERT(text != NULL);
  free(text);

  return 0;
}

int test_skip_line() {
  FILE *fp = fopen("test_data/poses.csv", "r");
  skip_line(fp);
  fclose(fp);

  return 0;
}

int test_file_rows() {
  int nb_rows = file_rows("test_data/poses.csv");
  MU_ASSERT(nb_rows > 0);
  return 0;
}

int test_file_copy() {
  int retval = file_copy("test_data/poses.csv", "/tmp/poses.csv");
  char *text0 = file_read("test_data/poses.csv");
  char *text1 = file_read("/tmp/poses.csv");
  MU_ASSERT(retval == 0);
  MU_ASSERT(strcmp(text0, text1) == 0);
  free(text0);
  free(text1);

  return 0;
}

/******************************************************************************
 * TEST DATA
 ******************************************************************************/

int test_string_malloc() {
  char *s = string_malloc("hello world!");
  MU_ASSERT(strcmp(s, "hello world!") == 0);
  free(s);
  return 0;
}

int test_dsv_rows() {
  int nb_rows = dsv_rows(TEST_CSV);
  MU_ASSERT(nb_rows == 10);
  return 0;
}

int test_dsv_cols() {
  int nb_cols = dsv_cols(TEST_CSV, ',');
  MU_ASSERT(nb_cols == 10);
  return 0;
}

int test_dsv_fields() {
  int nb_fields = 0;
  char **fields = dsv_fields(TEST_CSV, ',', &nb_fields);
  const char *expected[10] = {"a", "b", "c", "d", "e", "f", "g", "h", "i", "j"};
  if (fields == NULL) {
    printf("File not found [%s]\n", TEST_CSV);
    return -1;
  }

  MU_ASSERT(nb_fields == 10);
  for (int i = 0; i < nb_fields; i++) {
    MU_ASSERT(strcmp(fields[i], expected[i]) == 0);
    free(fields[i]);
  }
  free(fields);

  return 0;
}

int test_dsv_data() {
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **data = dsv_data(TEST_CSV, ',', &nb_rows, &nb_cols);

  int index = 0;
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_rows; j++) {
      MU_ASSERT(fltcmp(data[i][j], index + 1) == 0);
      index++;
    }
  }
  dsv_free(data, nb_rows);

  return 0;
}

int test_dsv_free() {
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **data = dsv_data(TEST_CSV, ',', &nb_rows, &nb_cols);
  dsv_free(data, nb_rows);

  return 0;
}

int test_csv_data() {
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **data = csv_data(TEST_CSV, &nb_rows, &nb_cols);
  csv_free(data, nb_rows);

  return 0;
}

/******************************************************************************
 * TEST DATA-STRUCTURE
 ******************************************************************************/

#ifdef USE_DATA_STRUCTURES

// DARRAY
// //////////////////////////////////////////////////////////////////////

int test_darray_new_and_destroy(void) {
  darray_t *array = darray_new(sizeof(int), 100);

  MU_ASSERT(array != NULL);
  MU_ASSERT(array->contents != NULL);
  MU_ASSERT(array->end == 0);
  MU_ASSERT(array->element_size == sizeof(int));
  MU_ASSERT(array->max == 100);

  darray_destroy(array);
  return 0;
}

int test_darray_push_pop(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* test push */
  for (int i = 0; i < 1000; i++) {
    int *val = darray_new_element(test_darray);
    *val = i * 333;
    darray_push(test_darray, val);
  }
  MU_ASSERT(test_darray->max == 1300);

  /* test pop */
  for (int i = 999; i >= 0; i--) {
    int *val = darray_pop(test_darray);
    MU_ASSERT(val != NULL);
    MU_ASSERT(*val == i * 333);
    free(val);
  }

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_contains(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* set element in array */
  int *val = darray_new_element(test_darray);
  *val = 99;
  darray_set(test_darray, 0, val);

  /* test contains */
  int res = darray_contains(test_darray, val, intcmp2);
  MU_ASSERT(res == 1);

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_copy(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* set element in array */
  int *val = darray_new_element(test_darray);
  *val = 99;
  darray_set(test_darray, 0, val);

  /* test copy */
  darray_t *array_copy = darray_copy(test_darray);
  int *val_copy = darray_get(array_copy, 0);
  MU_ASSERT(val != val_copy);
  MU_ASSERT(intcmp2(val, val_copy) == 0);

  darray_clear_destroy(test_darray);
  darray_clear_destroy(array_copy);
  return 0;
}

int test_darray_new_element(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* test new */
  int *val1 = darray_new_element(test_darray);
  int *val2 = darray_new_element(test_darray);

  MU_ASSERT(val1 != NULL);
  MU_ASSERT(val2 != NULL);

  free(val1);
  free(val2);

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_set_and_get(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* test set element */
  int *val1 = darray_new_element(test_darray);
  int *val2 = darray_new_element(test_darray);
  darray_set(test_darray, 0, val1);
  darray_set(test_darray, 1, val2);

  /* test get element */
  MU_ASSERT(darray_get(test_darray, 0) == val1);
  MU_ASSERT(darray_get(test_darray, 1) == val2);

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_update(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* set element */
  int *new_val1 = darray_new_element(test_darray);
  int *new_val2 = darray_new_element(test_darray);
  *new_val1 = 123;
  *new_val2 = 987;

  /* update */
  darray_update(test_darray, 0, new_val1);
  darray_update(test_darray, 1, new_val2);

  /* assert */
  MU_ASSERT(darray_get(test_darray, 0) == new_val1);
  MU_ASSERT(darray_get(test_darray, 1) == new_val2);

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_remove(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* set elements */
  int *val_1 = darray_new_element(test_darray);
  int *val_2 = darray_new_element(test_darray);
  *val_1 = 123;
  *val_2 = 987;
  darray_set(test_darray, 0, val_1);
  darray_set(test_darray, 1, val_2);

  /* remove element at index = 0 */
  int *result = darray_remove(test_darray, 0);
  MU_ASSERT(result != NULL);
  MU_ASSERT(*result == *val_1);
  MU_ASSERT(darray_get(test_darray, 0) == NULL);
  free(result);

  /* remove element at index = 1 */
  result = darray_remove(test_darray, 1);
  MU_ASSERT(result != NULL);
  MU_ASSERT(*result == *val_2);
  MU_ASSERT(darray_get(test_darray, 1) == NULL);
  free(result);

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_expand_and_contract(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* test expand */
  size_t old_max = (unsigned int) test_darray->max;
  darray_expand(test_darray);
  MU_ASSERT((unsigned int) test_darray->max ==
            old_max + test_darray->expand_rate);

  /* test contract */
  darray_contract(test_darray);
  MU_ASSERT((unsigned int) test_darray->max == test_darray->expand_rate + 1);

  darray_clear_destroy(test_darray);
  return 0;
}

// LIST
// ////////////////////////////////////////////////////////////////////////

int test_list_new_and_destroy(void) {
  list_t *list = list_new();
  MU_ASSERT(list != NULL);
  list_clear_destroy(list);
  return 0;
}

int test_list_push_pop(void) {
  /* Setup */
  list_t *list = list_new();
  char *t1 = string_malloc("test1 data");
  char *t2 = string_malloc("test2 data");
  char *t3 = string_malloc("test3 data");

  /* Push tests */
  list_push(list, t1);
  MU_ASSERT(strcmp(list->last->value, t1) == 0);

  list_push(list, t2);
  MU_ASSERT(strcmp(list->last->value, t2) == 0);

  list_push(list, t3);
  MU_ASSERT(strcmp(list->last->value, t3) == 0);
  MU_ASSERT(list->length == 3);

  /* Pop tests */
  char *val = list_pop(list);
  MU_ASSERT(val == t3);
  MU_ASSERT(list->first->value == t1);
  MU_ASSERT(list->last->value == t2);
  MU_ASSERT(list->length == 2);
  free(val);

  val = list_pop(list);
  MU_ASSERT(val == t2);
  MU_ASSERT(list->first->value == t1);
  MU_ASSERT(list->last->value == t1);
  MU_ASSERT(list->length == 1);
  free(val);

  val = list_pop(list);
  MU_ASSERT(val == t1);
  MU_ASSERT(list->first == NULL);
  MU_ASSERT(list->last == NULL);
  MU_ASSERT(list->length == 0);
  free(val);

  list_clear_destroy(list);
  return 0;
}

int test_list_shift(void) {
  /* Setup */
  list_t *list = list_new();
  char *t1 = string_malloc("test1 data");
  char *t2 = string_malloc("test2 data");

  /* Push elements */
  list_push(list, t1);
  list_push(list, t2);

  /* Shift */
  char *val = list_shift(list);
  MU_ASSERT(val == t1);
  MU_ASSERT(list->length == 1);
  free(val);

  val = list_shift(list);
  MU_ASSERT(val == t2);
  MU_ASSERT(list->length == 0);
  free(val);

  list_clear_destroy(list);
  return 0;
}

int test_list_unshift(void) {
  /* Setup */
  list_t *list = list_new();
  char *t1 = string_malloc("test1 data");
  char *t2 = string_malloc("test2 data");
  char *t3 = string_malloc("test3 data");

  /* Unshift */
  list_unshift(list, t1);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);
  MU_ASSERT(list->length == 1);

  list_unshift(list, t2);
  MU_ASSERT(strcmp(list->first->value, t2) == 0);
  MU_ASSERT(strcmp(list->first->value, t2) == 0);
  MU_ASSERT(list->length == 2);

  list_unshift(list, t3);
  MU_ASSERT(strcmp(list->first->value, t3) == 0);
  MU_ASSERT(strcmp(list->first->value, t3) == 0);
  MU_ASSERT(list->length == 3);
  list_clear_destroy(list);

  return 0;
}

int test_list_remove(void) {
  /* Push elements */
  list_t *list = list_new();
  char *t1 = string_malloc("test1 data");
  char *t2 = string_malloc("test2 data");
  char *t3 = string_malloc("test3 data");
  list_push(list, t1);
  list_push(list, t2);
  list_push(list, t3);

  /* Remove 2nd value */
  void *value = list_remove(list, t2, strcmp2);
  free(value);

  /* Assert */
  MU_ASSERT(list->length == 2);
  MU_ASSERT(strcmp(list->first->next->value, t3) == 0);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);

  /* Remove 2nd value */
  value = list_remove(list, t3, strcmp2);
  free(value);

  /* Assert */
  MU_ASSERT(list->length == 1);
  MU_ASSERT(list->first->next == NULL);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);
  list_clear_destroy(list);

  return 0;
}

int test_list_remove_destroy(void) {
  /* Setup */
  list_t *list = list_new();
  char *t1 = string_malloc("test1 data");
  char *t2 = string_malloc("test2 data");
  char *t3 = string_malloc("test3 data");

  /* Push elements */
  list_push(list, t1);
  list_push(list, t2);
  list_push(list, t3);

  /* Remove 2nd value */
  int result = list_remove_destroy(list, t2, strcmp2, free);

  /* Assert */
  MU_ASSERT(result == 0);
  MU_ASSERT(list->length == 2);
  MU_ASSERT(strcmp(list->first->next->value, t3) == 0);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);

  /* Remove 2nd value */
  result = list_remove_destroy(list, t3, strcmp2, free);

  /* Assert */
  MU_ASSERT(result == 0);
  MU_ASSERT(list->length == 1);
  MU_ASSERT(list->first->next == NULL);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);
  list_clear_destroy(list);

  return 0;
}

// STACK
// ///////////////////////////////////////////////////////////////////////

int test_mstack_new_and_destroy(void) {
  mstack_t *s = stack_new();

  MU_ASSERT(s->size == 0);
  MU_ASSERT(s->root == NULL);
  MU_ASSERT(s->end == NULL);

  mstack_destroy(s);
  return 0;
}

int test_mstack_push(void) {
  mstack_t *s = stack_new();
  float f1 = 2.0;
  float f2 = 4.0;
  float f3 = 8.0;

  /* push float 1 */
  mstack_push(s, &f1);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f1) == 0);
  MU_ASSERT(s->size == 1);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev == NULL);

  /* push float 2 */
  mstack_push(s, &f2);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f2) == 0);
  MU_ASSERT(s->size == 2);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev->value == &f1);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) &f1) == 0);

  /* push float 3 */
  mstack_push(s, &f3);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f3) == 0);
  MU_ASSERT(s->size == 3);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev->value == &f2);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) &f2) == 0);

  mstack_destroy(s);
  return 0;
}

int test_mstack_pop(void) {
  mstack_t *s = stack_new();
  float f1 = 2.0;
  float f2 = 4.0;
  float f3 = 8.0;

  /* push float 1 */
  mstack_push(s, &f1);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f1) == 0);
  MU_ASSERT(s->size == 1);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev == NULL);

  /* push float 2 */
  mstack_push(s, &f2);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f2) == 0);
  MU_ASSERT(s->size == 2);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev->value == &f1);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) &f1) == 0);

  /* push float 3 */
  mstack_push(s, &f3);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f3) == 0);
  MU_ASSERT(s->size == 3);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev->value == &f2);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) &f2) == 0);

  /* pop float 3 */
  float *flt_ptr = mstack_pop(s);
  MU_ASSERT(fltcmp(*(float *) flt_ptr, *(float *) &f3) == 0);
  MU_ASSERT(s->size == 2);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(fltcmp(*(float *) s->root->value, *(float *) &f1) == 0);

  /* pop float 2 */
  flt_ptr = mstack_pop(s);
  MU_ASSERT(fltcmp(*(float *) flt_ptr, *(float *) &f2) == 0);
  MU_ASSERT(s->size == 1);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(fltcmp(*(float *) s->root->value, *(float *) &f1) == 0);

  /* pop float 1 */
  flt_ptr = mstack_pop(s);
  MU_ASSERT(fltcmp(*(float *) flt_ptr, *(float *) &f1) == 0);
  MU_ASSERT(s->size == 0);
  MU_ASSERT(s->root == NULL);
  MU_ASSERT(s->end == NULL);

  mstack_destroy(s);
  return 0;
}

// QUEUE /////////////////////////////////////////////////////////////////////

int test_queue_new_and_destroy(void) {
  queue_t *q = queue_new();
  MU_ASSERT(q != NULL);
  MU_ASSERT(q->count == 0);
  queue_destroy(q);

  return 0;
}

int test_queue_enqueue_dequeue(void) {
  queue_t *q = queue_new();
  char *t1 = "test1 data";
  char *t2 = "test2 data";
  char *t3 = "test3 data";

  /* Enqueue tests */
  queue_enqueue(q, t1);
  MU_ASSERT(queue_first(q) == t1);
  MU_ASSERT(queue_last(q) == t1);
  MU_ASSERT(q->count == 1);

  queue_enqueue(q, t2);
  MU_ASSERT(queue_first(q) == t1);
  MU_ASSERT(queue_last(q) == t2);
  MU_ASSERT(q->count == 2);

  queue_enqueue(q, t3);
  MU_ASSERT(queue_first(q) == t1);
  MU_ASSERT(queue_last(q) == t3);
  MU_ASSERT(q->count == 3);

  /* Dequeue tests */
  char *val = queue_dequeue(q);
  MU_ASSERT(val == t1);
  MU_ASSERT(queue_first(q) == t2);
  MU_ASSERT(queue_last(q) == t3);
  MU_ASSERT(q->count == 2);

  val = queue_dequeue(q);
  MU_ASSERT(val == t2);
  MU_ASSERT(queue_first(q) == t3);
  MU_ASSERT(queue_last(q) == t3);
  MU_ASSERT(q->count == 1);

  val = queue_dequeue(q);
  MU_ASSERT(val == t3);
  MU_ASSERT(queue_first(q) == NULL);
  MU_ASSERT(queue_last(q) == NULL);
  MU_ASSERT(q->count == 0);

  return 0;
}

// HASHMAP ///////////////////////////////////////////////////////////////////

static int traverse_called;

static int traverse_good_cb(hashmap_node_t *node) {
  UNUSED(node);
  traverse_called++;
  return 0;
}

static int traverse_fail_cb(hashmap_node_t *node) {
  UNUSED(node);
  traverse_called++;
  if (traverse_called == 2) {
    return 1;
  } else {
    return 0;
  }
}

hashmap_t *hashmap_test_setup(void) {
  hashmap_t *map;
  char *test1;
  char *test2;
  char *test3;
  char *expect1;
  char *expect2;
  char *expect3;

  /* setup */
  map = hashmap_new();

  /* key and values */
  test1 = "test data 1";
  test2 = "test data 2";
  test3 = "xest data 3";
  expect1 = "THE VALUE 1";
  expect2 = "THE VALUE 2";
  expect3 = "THE VALUE 3";

  /* set */
  hashmap_set(map, test1, expect1);
  hashmap_set(map, test2, expect2);
  hashmap_set(map, test3, expect3);

  return map;
}

void hashmap_test_teardown(hashmap_t *map) {
  hashmap_destroy(map);
}

int test_hashmap_new_destroy(void) {
  hashmap_t *map;

  map = hashmap_new();
  MU_ASSERT(map != NULL);
  hashmap_destroy(map);

  return 0;
}

int test_hashmap_clear_destroy(void) {
  hashmap_t *map;

  map = hashmap_new();
  hashmap_set(map, "test", "hello");
  hashmap_clear_destroy(map);

  return 0;
}

int test_hashmap_get_set(void) {
  /* Setup */
  int rc;
  char *result;

  hashmap_t *map = hashmap_test_setup();
  char *test1 = "test data 1";
  char *test2 = "test data 2";
  char *test3 = "xest data 3";
  char *expect1 = "THE VALUE 1";
  char *expect2 = "THE VALUE 2";
  char *expect3 = "THE VALUE 3";

  /* Set and get test1 */
  rc = hashmap_set(map, test1, expect1);
  MU_ASSERT(rc == 0);
  result = hashmap_get(map, test1);
  MU_ASSERT(strcmp(result, expect1) == 0);

  /* Set and get test2 */
  rc = hashmap_set(map, test2, expect2);
  MU_ASSERT(rc == 0);
  result = hashmap_get(map, test2);
  MU_ASSERT(strcmp(result, expect2) == 0);

  /* Set and get test3 */
  rc = hashmap_set(map, test3, expect3);
  MU_ASSERT(rc == 0);
  result = hashmap_get(map, test3);
  MU_ASSERT(strcmp(result, expect3) == 0);

  /* Clean up */
  hashmap_test_teardown(map);

  return 0;
}

int test_hashmap_delete(void) {
  /* Setup */
  char *deleted = NULL;
  char *result = NULL;

  hashmap_t *map = hashmap_test_setup();
  char *test1 = "test data 1";
  char *test2 = "test data 2";
  char *test3 = "xest data 3";
  char *expect1 = "THE VALUE 1";
  char *expect2 = "THE VALUE 2";
  char *expect3 = "THE VALUE 3";

  /* Delete test1 */
  deleted = hashmap_delete(map, test1);
  MU_ASSERT(deleted != NULL);
  MU_ASSERT(strcmp(deleted, expect1) == 0);
  free(deleted);

  result = hashmap_get(map, test1);
  MU_ASSERT(result == NULL);

  /* Delete test2 */
  deleted = hashmap_delete(map, test2);
  MU_ASSERT(deleted != NULL);
  MU_ASSERT(strcmp(deleted, expect2) == 0);
  free(deleted);

  result = hashmap_get(map, test2);
  MU_ASSERT(result == NULL);

  /* Delete test3 */
  deleted = hashmap_delete(map, test3);
  MU_ASSERT(deleted != NULL);
  MU_ASSERT(strcmp(deleted, expect3) == 0);
  free(deleted);

  result = hashmap_get(map, test3);
  MU_ASSERT(result == NULL);

  /* Clean up */
  hashmap_test_teardown(map);

  return 0;
}

int test_hashmap_traverse(void) {
  int retval;
  hashmap_t *map;

  /* setup */
  map = hashmap_test_setup();

  /* traverse good cb */
  traverse_called = 0;
  retval = hashmap_traverse(map, traverse_good_cb);
  MU_ASSERT(retval == 0);
  MU_ASSERT(traverse_called == 3);

  /* traverse good bad */
  traverse_called = 0;
  retval = hashmap_traverse(map, traverse_fail_cb);
  MU_ASSERT(retval == 1);
  MU_ASSERT(traverse_called == 2);

  /* clean up */
  hashmap_test_teardown(map);

  return 0;
}

#endif // USE_DATA_STRUCTURES

/******************************************************************************
 * TEST TIME
 ******************************************************************************/

int test_tic_toc() {
  struct timespec t_start = tic();
  sleep(1.0);
  MU_ASSERT(fabs(toc(&t_start) - 1.0) < 1e-2);
  return 0;
}

int test_mtoc() {
  struct timespec t_start = tic();
  sleep(1.0);
  MU_ASSERT(fabs(mtoc(&t_start) - 1000) < 1);
  return 0;
}

int test_time_now() {
  timestamp_t t_now = time_now();
  // printf("t_now: %ld\n", t_now);
  MU_ASSERT(t_now > 0);
  return 0;
}

/******************************************************************************
 * TEST NETWORK
 ******************************************************************************/

int test_tcp_server_setup() {
  tcp_server_t server;
  const int port = 8080;
  tcp_server_setup(&server, port);
  return 0;
}

int test_http_msg_setup() {
  http_msg_t msg;
  http_msg_setup(&msg);

  return 0;
}

int test_http_msg_print() {
  char buf[9046] = "\
GET /chat HTTP/1.1\r\n\
Host: example.com:8000\r\n\
Upgrade: websocket\r\n\
Connection: Upgrade\r\n\
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\
Sec-WebSocket-Version: 13\r\n\
\r\n";
  http_msg_t msg;
  http_msg_setup(&msg);
  http_parse_request(buf, &msg);
  // http_msg_print(&msg);
  http_msg_free(&msg);

  return 0;
}

int test_http_parse_request() {
  char buf[9046] = "\
GET /chat HTTP/1.1\r\n\
Host: example.com:8000\r\n\
Upgrade: websocket\r\n\
Connection: Upgrade\r\n\
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\
Sec-WebSocket-Version: 13\r\n\
\r\n";

  http_msg_t msg;
  http_msg_setup(&msg);
  http_parse_request(buf, &msg);
  // http_msg_print(&msg);
  http_msg_free(&msg);

  return 0;
}

int test_ws_hash() {
  const char *key = "dGhlIHNhbXBsZSBub25jZQ==";
  char *hash = ws_hash(key);
  MU_ASSERT(strcmp(hash, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") == 0);
  free(hash);
  return 0;
}

int test_ws_server() {
  ws_server();
  return 0;
}

/******************************************************************************
 * TEST MATHS
 ******************************************************************************/

int test_min() {
  MU_ASSERT(MIN(1, 2) == 1);
  MU_ASSERT(MIN(2, 1) == 1);
  return 0;
}

int test_max() {
  MU_ASSERT(MAX(1, 2) == 2);
  MU_ASSERT(MAX(2, 1) == 2);
  return 0;
}

int test_randf() {
  const real_t val = randf(0.0, 10.0);
  MU_ASSERT(val < 10.0);
  MU_ASSERT(val > 0.0);
  return 0;
}

int test_deg2rad() {
  MU_ASSERT(fltcmp(deg2rad(180.0f), M_PI) == 0);
  return 0;
}

int test_rad2deg() {
  MU_ASSERT(fltcmp(rad2deg(M_PI), 180.0f) == 0);
  return 0;
}

int test_fltcmp() {
  MU_ASSERT(fltcmp(1.0, 1.0) == 0);
  MU_ASSERT(fltcmp(1.0, 1.01) != 0);
  return 0;
}

int test_fltcmp2() {
  const real_t x = 1.0f;
  const real_t y = 1.0f;
  const real_t z = 1.01f;
  MU_ASSERT(fltcmp2(&x, &y) == 0);
  MU_ASSERT(fltcmp2(&x, &z) != 0);
  return 0;
}

int test_pythag() {
  MU_ASSERT(fltcmp(pythag(3.0, 4.0), 5.0) == 0);
  return 0;
}

int test_lerp() {
  MU_ASSERT(fltcmp(lerp(0.0, 1.0, 0.5), 0.5) == 0);
  MU_ASSERT(fltcmp(lerp(0.0, 10.0, 0.8), 8.0) == 0);
  return 0;
}

int test_lerp3() {
  real_t a[3] = {0.0, 1.0, 2.0};
  real_t b[3] = {1.0, 2.0, 3.0};
  real_t c[3] = {0.0, 0.0, 0.0};
  real_t t = 0.5;

  lerp3(a, b, t, c);
  MU_ASSERT(fltcmp(c[0], 0.5) == 0);
  MU_ASSERT(fltcmp(c[1], 1.5) == 0);
  MU_ASSERT(fltcmp(c[2], 2.5) == 0);

  return 0;
}

int test_sinc() {
  return 0;
}

int test_mean() {
  real_t vals[4] = {1.0, 2.0, 3.0, 4.0};
  MU_ASSERT(fltcmp(mean(vals, 4), 2.5) == 0);

  return 0;
}

int test_median() {
  {
    const real_t vals[5] = {2.0, 3.0, 1.0, 4.0, 5.0};
    const real_t retval = median(vals, 5);
    MU_ASSERT(fltcmp(retval, 3.0) == 0);
  }

  {
    const real_t vals2[6] = {2.0, 3.0, 1.0, 4.0, 5.0, 6.0};
    const real_t retval = median(vals2, 6);
    MU_ASSERT(fltcmp(retval, 3.5) == 0);
  }

  return 0;
}

int test_var() {
  real_t vals[4] = {1.0, 2.0, 3.0, 4.0};
  MU_ASSERT(fltcmp(var(vals, 4), 1.666666667) == 0);

  return 0;
}

int test_stddev() {
  real_t vals[4] = {1.0, 2.0, 3.0, 4.0};
  MU_ASSERT(fltcmp(stddev(vals, 4), sqrt(1.666666667)) == 0);

  return 0;
}

/******************************************************************************
 * TEST LINEAR ALGEBRA
 ******************************************************************************/

int test_eye() {
  real_t A[25] = {0.0};
  eye(A, 5, 5);

  /* print_matrix("I", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      real_t expected = (i == j) ? 1.0 : 0.0;
      MU_ASSERT(fltcmp(A[idx], expected) == 0);
      idx++;
    }
  }

  return 0;
}

int test_ones() {
  real_t A[25] = {0.0};
  ones(A, 5, 5);

  /* print_matrix("A", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      MU_ASSERT((fabs(A[idx] - 1.0) < 1e-5));
      idx++;
    }
  }

  return 0;
}

int test_zeros() {
  real_t A[25] = {0.0};
  zeros(A, 5, 5);

  /* print_matrix("A", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      MU_ASSERT((fabs(A[idx] - 0.0) < 1e-5));
      idx++;
    }
  }

  return 0;
}

int test_mat_set() {
  real_t A[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  mat_set(A, 3, 0, 0, 1.0);
  mat_set(A, 3, 1, 1, 1.0);
  mat_set(A, 3, 2, 2, 1.0);

  /* print_matrix("A", A, 3, 3); */
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 1), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 2, 2), 1.0) == 0);

  return 0;
}

int test_mat_val() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

  /* print_matrix("A", A, 3, 3); */
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 1), 2.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 2), 3.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 0), 4.0) == 0);

  return 0;
}

int test_mat_copy() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {0};

  mat_copy(A, 3, 3, B);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(B[i], i + 1.0) == 0);
  }

  return 0;
}

int test_mat_row_set() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[3] = {0.0, 0.0, 0.0};

  /* Set first row zeros */
  mat_row_set(A, 3, 0, B);
  for (int i = 0; i < 3; i++) {
    MU_ASSERT(fltcmp(A[i], 0.0) == 0);
  }

  /* Set second row zeros */
  mat_row_set(A, 3, 1, B);
  for (int i = 0; i < 6; i++) {
    MU_ASSERT(fltcmp(A[i], 0.0) == 0);
  }

  /* Set third row zeros */
  mat_row_set(A, 3, 1, B);
  for (int i = 0; i < 6; i++) {
    MU_ASSERT(fltcmp(A[i], 0.0) == 0);
  }

  return 0;
}

int test_mat_col_set() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[3] = {0.0, 0.0, 0.0};

  /* Set first column zeros */
  mat_col_set(A, 3, 3, 0, B);
  for (int i = 0; i < 3; i++) {
    MU_ASSERT(fltcmp(A[i * 3], 0.0) == 0);
  }

  /* Set second column zeros */
  mat_col_set(A, 3, 3, 1, B);
  for (int i = 0; i < 3; i++) {
    MU_ASSERT(fltcmp(A[(i * 3) + 1], 0.0) == 0);
  }

  /* Set third column zeros */
  mat_col_set(A, 3, 3, 2, B);
  for (int i = 0; i < 3; i++) {
    MU_ASSERT(fltcmp(A[(i * 3) + 2], 0.0) == 0);
  }

  /* Check whether full matrix is zeros */
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(A[i], 0.0) == 0);
  }

  return 0;
}

int test_mat_block_get() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[4] = {0.0};
  mat_block_get(A, 3, 1, 1, 2, 2, B);

  // print_matrix("A", A, 3, 3);
  // print_matrix("B", B, 2, 2);
  MU_ASSERT(fltcmp(mat_val(B, 2, 0, 0), 5.0) == 0);
  MU_ASSERT(fltcmp(mat_val(B, 2, 0, 1), 6.0) == 0);
  MU_ASSERT(fltcmp(mat_val(B, 2, 1, 0), 8.0) == 0);
  MU_ASSERT(fltcmp(mat_val(B, 2, 1, 1), 9.0) == 0);

  return 0;
}

int test_mat_block_set() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[4] = {0.0, 0.0, 0.0, 0.0};

  // print_matrix("A", A, 3, 3);
  // print_matrix("B", B, 2, 2);
  mat_block_set(A, 3, 1, 2, 1, 2, B);
  // print_matrix("A", A, 3, 3);
  // print_matrix("B", B, 2, 2);

  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 1), 0.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 2), 0.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 2, 1), 0.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 2, 2), 0.0) == 0);

  return 0;
}

int test_mat_diag_get() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t d[3] = {0.0, 0.0, 0.0};
  mat_diag_get(A, 3, 3, d);

  // print_matrix("A", A, 3, 3);
  // print_vector("d", d, 3);
  MU_ASSERT(fltcmp(d[0], 1.0) == 0);
  MU_ASSERT(fltcmp(d[1], 5.0) == 0);
  MU_ASSERT(fltcmp(d[2], 9.0) == 0);

  return 0;
}

int test_mat_diag_set() {
  real_t A[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  real_t d[4] = {1.0, 2.0, 3.0};
  mat_diag_set(A, 3, 3, d);

  // print_matrix("A", A, 3, 3);
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 1), 2.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 2, 2), 3.0) == 0);

  return 0;
}

int test_mat_triu() {
  // clang-format off
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  real_t U[16] = {0};
  // clang-format on
  mat_triu(A, 4, U);
  // print_matrix("U", U, 4, 4);

  return 0;
}

int test_mat_tril() {
  // clang-format off
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  real_t L[16] = {0};
  // clang-format on
  mat_tril(A, 4, L);
  // print_matrix("L", L, 4, 4);

  return 0;
}

int test_mat_trace() {
  // clang-format off
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  // clang-format on
  const real_t tr = mat_trace(A, 4, 4);
  MU_ASSERT(fltcmp(tr, 1.0 + 6.0 + 11.0 + 16.0) == 0.0);

  return 0;
}

int test_mat_transpose() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t At[9] = {0.0};
  real_t At_expected[9] = {1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0};
  mat_transpose(A, 3, 3, At);
  MU_ASSERT(mat_equals(At, At_expected, 3, 3, 1e-8) == 0);

  real_t B[2 * 3] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  real_t Bt[3 * 2] = {0};
  real_t Bt_expected[3 * 2] = {1.0, 4.0, 2.0, 5.0, 3.0, 6.0};
  mat_transpose(B, 2, 3, Bt);
  for (int i = 0; i < 6; i++) {
    MU_ASSERT(fltcmp(Bt[i], Bt_expected[i]) == 0);
  }

  return 0;
}

int test_mat_add() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  mat_add(A, B, C, 3, 3);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 10.0) == 0);
  }

  return 0;
}

int test_mat_sub() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  mat_sub(A, B, C, 3, 3);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 0.0) == 0);
  }

  return 0;
}

int test_mat_scale() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  mat_scale(A, 3, 3, 2.0);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(A[i], 2 * (i + 1)) == 0);
  }

  return 0;
}

int test_vec_add() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  vec_add(A, B, C, 9);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 10.0) == 0);
  }

  return 0;
}

int test_vec_sub() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  vec_sub(A, B, C, 9);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 0.0) == 0);
  }

  return 0;
}

int test_dot() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[3] = {1.0, 2.0, 3.0};
  real_t C[3] = {0.0};

  /* Multiply matrix A and B */
  dot(A, 3, 3, B, 3, 1, C);

  MU_ASSERT(fltcmp(C[0], 14.0) == 0);
  MU_ASSERT(fltcmp(C[1], 32.0) == 0);
  MU_ASSERT(fltcmp(C[2], 50.0) == 0);

  return 0;
}

int test_hat() {
  real_t x[3] = {1.0, 2.0, 3.0};
  real_t S[3 * 3] = {0};

  hat(x, S);

  MU_ASSERT(fltcmp(S[0], 0.0) == 0);
  MU_ASSERT(fltcmp(S[1], -3.0) == 0);
  MU_ASSERT(fltcmp(S[2], 2.0) == 0);

  MU_ASSERT(fltcmp(S[3], 3.0) == 0);
  MU_ASSERT(fltcmp(S[4], 0.0) == 0);
  MU_ASSERT(fltcmp(S[5], -1.0) == 0);

  MU_ASSERT(fltcmp(S[6], -2.0) == 0);
  MU_ASSERT(fltcmp(S[7], 1.0) == 0);
  MU_ASSERT(fltcmp(S[8], 0.0) == 0);

  return 0;
}

int test_check_jacobian() {
  const size_t m = 2;
  const size_t n = 3;
  const real_t threshold = 1e-6;
  const int print = 0;

  // Positive test
  {
    const real_t fdiff[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    const real_t jac[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    int retval = check_jacobian("test_check_jacobian",
                                fdiff,
                                jac,
                                m,
                                n,
                                threshold,
                                print);
    MU_ASSERT(retval == 0);
  }

  // Negative test
  {
    const real_t fdiff[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    const real_t jac[6] = {0.0, 1.0, 2.0, 3.1, 4.0, 5.0};
    int retval = check_jacobian("test_check_jacobian",
                                fdiff,
                                jac,
                                m,
                                n,
                                threshold,
                                print);
    MU_ASSERT(retval == -1);
  }

  return 0;
}

/******************************************************************************
 * TEST SVD
 ******************************************************************************/

int test_svd() {
  // Matrix A
  // clang-format off
  real_t A[6 * 4] = {
    7.52, -1.10, -7.95,  1.08,
    -0.76,  0.62,  9.34, -7.10,
     5.13,  6.62, -5.66,  0.87,
    -4.75,  8.52,  5.75,  5.30,
     1.33,  4.91, -5.49, -3.52,
    -2.40, -6.77,  2.34,  3.95
  };
  real_t A_copy[6 * 4] = {
    7.52, -1.10, -7.95,  1.08,
    -0.76,  0.62,  9.34, -7.10,
     5.13,  6.62, -5.66,  0.87,
    -4.75,  8.52,  5.75,  5.30,
     1.33,  4.91, -5.49, -3.52,
    -2.40, -6.77,  2.34,  3.95
  };
  // clang-format on

  // Decompose A with SVD
  // struct timespec t = tic();
  real_t U[6 * 4] = {0};
  real_t s[4] = {0};
  real_t V[4 * 4] = {0};
  svd(A, 6, 4, U, s, V);
  // printf("time taken: [%fs]\n", toc(&t));

  // Multiply the output to see if it can form matrix A again
  // U * S * Vt
  real_t S[4 * 4] = {0};
  real_t Vt[4 * 4] = {0};
  real_t US[6 * 4] = {0};
  real_t USVt[6 * 4] = {0};
  mat_diag_set(S, 4, 4, s);
  mat_transpose(V, 4, 4, Vt);
  dot(U, 6, 4, S, 4, 4, US);
  dot(US, 6, 4, Vt, 4, 4, USVt);

  // print_matrix("U", U, 6, 4);
  // print_matrix("S", S, 4, 4);
  // print_matrix("V", V, 4, 4);
  // print_matrix("USVt", USVt, 6, 4);
  MU_ASSERT(mat_equals(USVt, A_copy, 6, 4, 1e-5) == 0);

  return 0;
}

int test_svd_inv() {
  // clang-format off
  const int m = 4;
  const int n = 4;
  real_t A[4 * 4] = {
     7.52, -1.10, -7.95,  1.08,
    -0.76,  0.62,  9.34, -7.10,
     5.13,  6.62, -5.66,  0.87,
    -4.75,  8.52,  5.75,  5.30,
  };
  real_t A_copy[4 * 4] = {
     7.52, -1.10, -7.95,  1.08,
    -0.76,  0.62,  9.34, -7.10,
     5.13,  6.62, -5.66,  0.87,
    -4.75,  8.52,  5.75,  5.30,
  };
  // clang-format on

  // Invert matrix A using SVD
  // struct timespec t = tic();
  real_t A_inv[4 * 4] = {0};
  svd_inv(A, m, n, A_inv);
  // printf("time taken: [%fs]\n", toc(&t));

  // Inverse check: A * A_inv = eye
  MU_ASSERT(check_inv(A_copy, A_inv, 4) == 0);

  return 0;
}

/******************************************************************************
 * TEST CHOL
 ******************************************************************************/

int test_chol() {
  // clang-format off
  const int n = 3;
  real_t A[9] = {
    4.0, 12.0, -16.0,
    12.0, 37.0, -43.0,
    -16.0, -43.0, 98.0
  };
  // clang-format on

  // struct timespec t = tic();
  real_t L[9] = {0};
  chol(A, n, L);
  // printf("time taken: [%fs]\n", toc(&t));

  real_t Lt[9] = {0};
  real_t LLt[9] = {0};
  mat_transpose(L, n, n, Lt);
  dot(L, n, n, Lt, n, n, LLt);

  int debug = 0;
  if (debug) {
    print_matrix("L", L, n, n);
    printf("\n");
    print_matrix("Lt", Lt, n, n);
    printf("\n");
    print_matrix("LLt", LLt, n, n);
    printf("\n");
    print_matrix("A", A, n, n);
  }
  MU_ASSERT(mat_equals(A, LLt, n, n, 1e-5) == 0);

  return 0;
}

int test_chol_solve() {
  // clang-format off
  const int n = 3;
  real_t A[9] = {
    2.0, -1.0, 0.0,
    -1.0, 2.0, -1.0,
    0.0, -1.0, 1.0
  };
  real_t b[3] = {1.0, 0.0, 0.0};
  real_t x[3] = {0.0, 0.0, 0.0};
  // clang-format on

  // struct timespec t = tic();
  chol_solve(A, b, x, n);
  // printf("time taken: [%fs]\n", toc(&t));
  // print_vector("x", x, n);

  MU_ASSERT(fltcmp(x[0], 1.0) == 0);
  MU_ASSERT(fltcmp(x[1], 1.0) == 0);
  MU_ASSERT(fltcmp(x[2], 1.0) == 0);

  return 0;
}

/******************************************************************************
 * TEST TRANSFORMS
 ******************************************************************************/

int test_tf_rot_set() {
  real_t C[9];
  for (int i = 0; i < 9; i++) {
    C[i] = 1.0;
  }

  real_t T[16] = {0.0};
  tf_rot_set(T, C);
  /* print_matrix("T", T, 4, 4); */

  MU_ASSERT(fltcmp(T[0], 1.0) == 0);
  MU_ASSERT(fltcmp(T[1], 1.0) == 0);
  MU_ASSERT(fltcmp(T[2], 1.0) == 0);
  MU_ASSERT(fltcmp(T[3], 0.0) == 0);

  MU_ASSERT(fltcmp(T[4], 1.0) == 0);
  MU_ASSERT(fltcmp(T[5], 1.0) == 0);
  MU_ASSERT(fltcmp(T[6], 1.0) == 0);
  MU_ASSERT(fltcmp(T[7], 0.0) == 0);

  MU_ASSERT(fltcmp(T[8], 1.0) == 0);
  MU_ASSERT(fltcmp(T[9], 1.0) == 0);
  MU_ASSERT(fltcmp(T[10], 1.0) == 0);
  MU_ASSERT(fltcmp(T[11], 0.0) == 0);

  MU_ASSERT(fltcmp(T[12], 0.0) == 0);
  MU_ASSERT(fltcmp(T[13], 0.0) == 0);
  MU_ASSERT(fltcmp(T[14], 0.0) == 0);
  MU_ASSERT(fltcmp(T[15], 0.0) == 0);

  return 0;
}

int test_tf_trans_set() {
  real_t r[3] = {1.0, 2.0, 3.0};

  real_t T[16] = {0.0};
  tf_trans_set(T, r);
  /* print_matrix("T", T, 4, 4); */

  MU_ASSERT(fltcmp(T[0], 0.0) == 0);
  MU_ASSERT(fltcmp(T[1], 0.0) == 0);
  MU_ASSERT(fltcmp(T[2], 0.0) == 0);
  MU_ASSERT(fltcmp(T[3], 1.0) == 0);

  MU_ASSERT(fltcmp(T[4], 0.0) == 0);
  MU_ASSERT(fltcmp(T[5], 0.0) == 0);
  MU_ASSERT(fltcmp(T[6], 0.0) == 0);
  MU_ASSERT(fltcmp(T[7], 2.0) == 0);

  MU_ASSERT(fltcmp(T[8], 0.0) == 0);
  MU_ASSERT(fltcmp(T[9], 0.0) == 0);
  MU_ASSERT(fltcmp(T[10], 0.0) == 0);
  MU_ASSERT(fltcmp(T[11], 3.0) == 0);

  MU_ASSERT(fltcmp(T[12], 0.0) == 0);
  MU_ASSERT(fltcmp(T[13], 0.0) == 0);
  MU_ASSERT(fltcmp(T[14], 0.0) == 0);
  MU_ASSERT(fltcmp(T[15], 0.0) == 0);

  return 0;
}

int test_tf_trans_get() {
  // clang-format off
  real_t T[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  // clang-format on

  /* Get translation vector */
  real_t r[3];
  tf_trans_get(T, r);
  MU_ASSERT(fltcmp(r[0], 4.0) == 0);
  MU_ASSERT(fltcmp(r[1], 8.0) == 0);
  MU_ASSERT(fltcmp(r[2], 12.0) == 0);

  return 0;
}

int test_tf_rot_get() {
  /* Transform */
  // clang-format off
  real_t T[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  // clang-format on

  /* Get rotation matrix */
  real_t C[9];
  tf_rot_get(T, C);

  MU_ASSERT(fltcmp(C[0], 1.0) == 0);
  MU_ASSERT(fltcmp(C[1], 2.0) == 0);
  MU_ASSERT(fltcmp(C[2], 3.0) == 0);

  MU_ASSERT(fltcmp(C[3], 5.0) == 0);
  MU_ASSERT(fltcmp(C[4], 6.0) == 0);
  MU_ASSERT(fltcmp(C[5], 7.0) == 0);

  MU_ASSERT(fltcmp(C[6], 9.0) == 0);
  MU_ASSERT(fltcmp(C[7], 10.0) == 0);
  MU_ASSERT(fltcmp(C[8], 11.0) == 0);

  return 0;
}

int test_tf_quat_get() {
  /* Transform */
  // clang-format off
  real_t T[16] = {1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0};
  // clang-format on

  /* Create rotation matrix */
  const real_t ypr_in[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(ypr_in, C);
  tf_rot_set(T, C);

  /* Extract quaternion from transform */
  real_t q[4] = {0};
  tf_quat_get(T, q);

  /* Convert quaternion back to euler angles */
  real_t ypr_out[3] = {0};
  quat2euler(q, ypr_out);

  MU_ASSERT(fltcmp(rad2deg(ypr_out[0]), 10.0) == 0);
  MU_ASSERT(fltcmp(rad2deg(ypr_out[1]), 20.0) == 0);
  MU_ASSERT(fltcmp(rad2deg(ypr_out[2]), 30.0) == 0);

  return 0;
}

int test_tf_inv() {
  /* Create Transform */
  // clang-format off
  real_t T[16] = {1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0};
  // clang-format on
  /* -- Set rotation component */
  const real_t euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(euler, C);
  tf_rot_set(T, C);
  /* -- Set translation component */
  real_t r[3] = {1.0, 2.0, 3.0};
  tf_trans_set(T, r);

  /* Invert transform */
  real_t T_inv[16] = {0};
  tf_inv(T, T_inv);

  /* real_t Invert transform */
  real_t T_inv_inv[16] = {0};
  tf_inv(T_inv, T_inv_inv);

  /* Assert */
  int idx = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      MU_ASSERT(fltcmp(T_inv_inv[idx], T[idx]) == 0);
    }
  }

  return 0;
}

int test_tf_point() {
  /* Transform */
  // clang-format off
  real_t T[16] = {1.0, 0.0, 0.0, 1.0,
                  0.0, 1.0, 0.0, 2.0,
                  0.0, 0.0, 1.0, 3.0,
                  0.0, 0.0, 0.0, 1.0};
  // clang-format on

  /* Point */
  real_t p[3] = {1.0, 2.0, 3.0};

  /* Transform point */
  real_t result[3] = {0};
  tf_point(T, p, result);

  return 0;
}

int test_tf_hpoint() {
  /* Transform */
  // clang-format off
  real_t T[16] = {1.0, 0.0, 0.0, 1.0,
                  0.0, 1.0, 0.0, 2.0,
                  0.0, 0.0, 1.0, 3.0,
                  0.0, 0.0, 0.0, 1.0};
  // clang-format on

  /* Homogeneous point */
  real_t hp[4] = {1.0, 2.0, 3.0, 1.0};

  /* Transform homogeneous point */
  real_t result[4] = {0};
  tf_hpoint(T, hp, result);

  return 0;
}

int test_tf_perturb_rot() {
  /* Transform */
  // clang-format off
  real_t T[4 * 4] = {1.0, 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0, 2.0,
                     0.0, 0.0, 1.0, 3.0,
                     0.0, 0.0, 0.0, 1.0};
  // clang-format on

  /* Perturb rotation */
  const real_t step_size = 1e-2;
  tf_perturb_rot(T, step_size, 0);

  /* Assert */
  MU_ASSERT(fltcmp(T[0], 1.0) == 0);
  MU_ASSERT(fltcmp(T[5], 1.0) != 0);
  MU_ASSERT(fltcmp(T[10], 1.0) != 0);

  return 0;
}

int test_tf_perturb_trans() {
  /* Transform */
  // clang-format off
  real_t T[4 * 4] = {1.0, 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0, 2.0,
                     0.0, 0.0, 1.0, 3.0,
                     0.0, 0.0, 0.0, 1.0};
  // clang-format on

  /* Perturb translation */
  const real_t step_size = 1e-2;
  tf_perturb_trans(T, step_size, 0);

  /* Assert */
  MU_ASSERT(fltcmp(T[3], 1.01) == 0);
  MU_ASSERT(fltcmp(T[7], 2.0) == 0);
  MU_ASSERT(fltcmp(T[11], 3.0) == 0);

  return 0;
}

int test_tf_chain() {
  /* First transform */
  const real_t r0[3] = {0.0, 0.0, 0.1};
  const real_t euler0[3] = {deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  real_t T0[4 * 4] = {0};
  real_t C0[9] = {0};

  euler321(euler0, C0);
  tf_rot_set(T0, C0);
  tf_trans_set(T0, r0);
  T0[15] = 1.0;

  /* Second transform */
  const real_t r1[3] = {0.0, 0.0, 0.1};
  const real_t euler1[3] = {deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  real_t T1[4 * 4] = {0};
  real_t C1[9] = {0};

  euler321(euler1, C1);
  tf_rot_set(T1, C1);
  tf_trans_set(T1, r1);
  T1[15] = 1.0;

  /* Third transform */
  const real_t r2[3] = {0.0, 0.0, 0.1};
  const real_t euler2[3] = {deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  real_t T2[4 * 4] = {0};
  real_t C2[9] = {0};

  euler321(euler2, C2);
  tf_rot_set(T2, C2);
  tf_trans_set(T2, r2);
  T2[15] = 1.0;

  /* Chain transforms */
  const real_t *tfs[3] = {T0, T1, T2};
  const int N = 3;
  real_t T_out[4 * 4] = {0};
  tf_chain(tfs, N, T_out);

  return 0;
}

int test_euler321() {
  /* Euler to rotation matrix */
  const real_t euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(euler, C);

  /* Rotation matrix to quaternion */
  real_t q[4] = {0};
  rot2quat(C, q);

  /* Quaternion to Euler angles*/
  real_t euler2[3] = {0};
  quat2euler(q, euler2);

  MU_ASSERT(fltcmp(euler2[0], euler[0]) == 0);
  MU_ASSERT(fltcmp(euler2[1], euler[1]) == 0);
  MU_ASSERT(fltcmp(euler2[2], euler[2]) == 0);

  return 0;
}

int test_rot2quat() {
  /* Rotation matrix to quaternion */
  const real_t C[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  real_t q[4] = {0.0};
  rot2quat(C, q);

  MU_ASSERT(fltcmp(q[0], 1.0) == 0);
  MU_ASSERT(fltcmp(q[1], 0.0) == 0);
  MU_ASSERT(fltcmp(q[2], 0.0) == 0);
  MU_ASSERT(fltcmp(q[3], 0.0) == 0);

  return 0;
}

int test_quat2euler() {
  const real_t C[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  /* Rotation matrix to quaternion */
  real_t q[4] = {0.0};
  rot2quat(C, q);

  /* Quaternion to Euler angles */
  real_t ypr[3] = {0.0};
  quat2euler(q, ypr);

  MU_ASSERT(fltcmp(ypr[0], 0.0) == 0);
  MU_ASSERT(fltcmp(ypr[1], 0.0) == 0);
  MU_ASSERT(fltcmp(ypr[2], 0.0) == 0);

  return 0;
}

int test_quat2rot() {
  /* Euler to rotation matrix */
  const real_t euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(euler, C);

  /* Rotation matrix to quaternion */
  real_t q[4] = {0.0};
  rot2quat(C, q);
  /* print_vector("q", q, 4); */

  /* Quaternion to rotation matrix */
  real_t rot[9] = {0.0};
  quat2rot(q, rot);

  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], rot[i]) == 0);
  }

  return 0;
}

/******************************************************************************
 * TEST LIE
 ******************************************************************************/

int test_lie_Exp_Log() {
  const real_t phi[3] = {0.1, 0.2, 0.3};
  real_t C[3 * 3] = {0};
  lie_Exp(phi, C);

  real_t rvec[3] = {0};
  lie_Log(C, rvec);

  // print_vector("phi", phi, 3);
  // printf("\n");
  // print_matrix("C", C, 3, 3);
  // print_vector("rvec", rvec, 3);

  MU_ASSERT(fltcmp(phi[0], rvec[0]) == 0);
  MU_ASSERT(fltcmp(phi[1], rvec[1]) == 0);
  MU_ASSERT(fltcmp(phi[2], rvec[2]) == 0);

  return 0;
}

/******************************************************************************
 * TEST CV
 ******************************************************************************/

int test_image_setup() {
  return 0;
}

int test_image_load() {
  return 0;
}

int test_image_print_properties() {
  return 0;
}

int test_image_free() {
  return 0;
}

int test_linear_triangulation() {
  /* Setup camera */
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2;
  const real_t cy = image_height / 2;
  const real_t proj_params[4] = {fx, fy, cx, cy};
  real_t K[3 * 3];
  pinhole_K(proj_params, K);

  /* Setup camera pose T_WC0 */
  const real_t ypr_WC0[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC0[3] = {0.0, 0.0, 0.0};
  real_t T_WC0[4 * 4] = {0};
  tf_euler_set(T_WC0, ypr_WC0);
  tf_trans_set(T_WC0, r_WC0);

  /* Setup camera pose T_WC1 */
  const real_t euler_WC1[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC1[3] = {0.1, 0.1, 0.0};
  real_t T_WC1[4 * 4] = {0};
  tf_euler_set(T_WC1, euler_WC1);
  tf_trans_set(T_WC1, r_WC1);

  /* Setup projection matrices */
  real_t P0[3 * 4] = {0};
  real_t P1[3 * 4] = {0};
  pinhole_projection_matrix(proj_params, T_WC0, P0);
  pinhole_projection_matrix(proj_params, T_WC1, P1);

  /* Setup 3D and 2D correspondance points */
  int nb_tests = 100;
  for (int i = 0; i < nb_tests; i++) {
    const real_t p_W[3] = {5.0, randf(-1.0, 1.0), randf(-1.0, 1.0)};

    real_t T_C0W[4 * 4] = {0};
    real_t T_C1W[4 * 4] = {0};
    tf_inv(T_WC0, T_C0W);
    tf_inv(T_WC1, T_C1W);

    real_t p_C0[3] = {0};
    real_t p_C1[3] = {0};
    tf_point(T_C0W, p_W, p_C0);
    tf_point(T_C1W, p_W, p_C1);

    real_t z0[2] = {0};
    real_t z1[2] = {0};
    pinhole_project(proj_params, p_C0, z0);
    pinhole_project(proj_params, p_C1, z1);

    /* Test */
    real_t p_W_est[3] = {0};
    linear_triangulation(P0, P1, z0, z1, p_W_est);

    /* Assert */
    real_t diff[3] = {0};
    vec_sub(p_W, p_W_est, diff, 3);
    const real_t norm = vec_norm(diff, 3);
    /* print_vector("p_W [gnd]", p_W, 3); */
    /* print_vector("p_W [est]", p_W_est, 3); */
    MU_ASSERT(norm < 1e-4);
  }

  return 0;
}

int test_radtan4_distort() {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  radtan4_distort(params, p, p_d);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);

  return 0;
}

int test_radtan4_point_jacobian() {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t J_point[2 * 2] = {0};
  radtan4_point_jacobian(params, p, J_point);

  /* Calculate numerical diff */
  const real_t step = 1e-4;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 2] = {0};
  {
    real_t p_d[2] = {0};
    radtan4_distort(params, p, p_d);

    for (int i = 0; i < 2; i++) {
      real_t p_diff[2] = {p[0], p[1]};
      p_diff[i] = p[i] + step;

      real_t p_d_prime[2] = {0};
      radtan4_distort(params, p_diff, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 2] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_point", J_point, 2, 2);
  // print_matrix("J_numdiff", J_numdiff, 2, 2);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_point, 2, 2, tol, 0) == 0);

  return 0;
}

int test_radtan4_params_jacobian() {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t J_param[2 * 4] = {0};
  radtan4_params_jacobian(params, p, J_param);

  /* Calculate numerical diff */
  const real_t step = 1e-4;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 4] = {0};
  {
    real_t p_d[2] = {0};
    radtan4_distort(params, p, p_d);

    for (int i = 0; i < 4; i++) {
      real_t params_diff[4] = {params[0], params[1], params[2], params[3]};
      params_diff[i] = params[i] + step;

      real_t p_d_prime[2] = {0};
      radtan4_distort(params_diff, p, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 4] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_param", J_param, 2, 4);
  // print_matrix("J_numdiff", J_numdiff, 2, 4);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_param, 2, 4, tol, 0) == 0);

  return 0;
}

int test_equi4_distort() {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  equi4_distort(params, p, p_d);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);

  return 0;
}

int test_equi4_point_jacobian() {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t J_point[2 * 2] = {0};
  equi4_point_jacobian(params, p, J_point);

  /* Calculate numerical diff */
  const real_t step = 1e-4;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 2] = {0};
  {
    real_t p_d[2] = {0};
    equi4_distort(params, p, p_d);

    for (int i = 0; i < 2; i++) {
      real_t p_diff[2] = {p[0], p[1]};
      p_diff[i] = p[i] + step;

      real_t p_d_prime[2] = {0};
      equi4_distort(params, p_diff, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 2] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_point", J_point, 2, 2);
  // print_matrix("J_numdiff", J_numdiff, 2, 2);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_point, 2, 2, tol, 0) == 0);

  return 0;
}

int test_equi4_params_jacobian() {
  const real_t params[4] = {0.01, 0.01, 0.01, 0.01};
  const real_t p[2] = {0.1, 0.2};
  real_t J_param[2 * 4] = {0};
  equi4_params_jacobian(params, p, J_param);

  /* Calculate numerical diff */
  const real_t step = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 4] = {0};
  {
    real_t p_d[2] = {0};
    equi4_distort(params, p, p_d);

    for (int i = 0; i < 4; i++) {
      real_t params_diff[4] = {params[0], params[1], params[2], params[3]};
      params_diff[i] = params[i] + step;

      real_t p_d_prime[2] = {0};
      equi4_distort(params_diff, p, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 4] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_param", J_param, 2, 4);
  // print_matrix("J_numdiff", J_numdiff, 2, 4);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_param, 2, 4, tol, 0) == 0);

  return 0;
}

int test_pinhole_focal() {
  const real_t focal = pinhole_focal(640, 90.0);
  MU_ASSERT(fltcmp(focal, 320.0) == 0);
  return 0;
}

int test_pinhole_K() {
  const real_t params[4] = {1.0, 2.0, 3.0, 4.0};
  real_t K[3 * 3] = {0};
  pinhole_K(params, K);

  MU_ASSERT(fltcmp(K[0], 1.0) == 0);
  MU_ASSERT(fltcmp(K[1], 0.0) == 0);
  MU_ASSERT(fltcmp(K[2], 3.0) == 0);

  MU_ASSERT(fltcmp(K[3], 0.0) == 0);
  MU_ASSERT(fltcmp(K[4], 2.0) == 0);
  MU_ASSERT(fltcmp(K[5], 4.0) == 0);

  MU_ASSERT(fltcmp(K[6], 0.0) == 0);
  MU_ASSERT(fltcmp(K[7], 0.0) == 0);
  MU_ASSERT(fltcmp(K[8], 1.0) == 0);

  return 0;
}

int test_pinhole_projection_matrix() {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};

  /* Camera pose */
  const real_t ypr_WC0[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC0[3] = {0.0, 0.0, 0.0};
  real_t T_WC0[4 * 4] = {0};
  tf_euler_set(T_WC0, ypr_WC0);
  tf_trans_set(T_WC0, r_WC0);

  /* Camera projection matrix */
  real_t P[3 * 4] = {0};
  pinhole_projection_matrix(params, T_WC0, P);

  /* Project point using projection matrix */
  const real_t p_W[3] = {1.0, 0.1, 0.2};
  const real_t hp_W[4] = {p_W[0], p_W[1], p_W[2], 1.0};
  real_t hp[3] = {0};
  dot(P, 3, 4, hp_W, 4, 1, hp);
  real_t z[2] = {hp[0], hp[1]};

  /* Project point by inverting T_WC0 and projecting the point */
  real_t p_C[3] = {0};
  real_t T_C0W[4 * 4] = {0};
  real_t z_gnd[2] = {0};
  tf_inv(T_WC0, T_C0W);
  tf_point(T_C0W, p_W, p_C);
  pinhole_project(params, p_C, z_gnd);

  /* Assert */
  MU_ASSERT(fltcmp(z_gnd[0], z[0]) == 0);
  MU_ASSERT(fltcmp(z_gnd[1], z[1]) == 0);

  return 0;
}

int test_pinhole_project() {
  const real_t img_w = 640;
  const real_t img_h = 480;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};
  const real_t p_C[3] = {0.0, 0.0, 1.0};
  real_t z[2] = {0.0, 0.0};
  pinhole_project(params, p_C, z);

  /* print_vector("p_C", p_C, 3); */
  /* print_vector("z", z, 2); */
  MU_ASSERT(fltcmp(z[0], 320.0) == 0);
  MU_ASSERT(fltcmp(z[1], 240.0) == 0);

  return 0;
}

int test_pinhole_point_jacobian() {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};

  /* Calculate analytical jacobian */
  real_t J_point[2 * 2] = {0};
  pinhole_point_jacobian(params, J_point);

  /* Numerical differentiation */
  const real_t p_C[3] = {0.1, 0.2, 1.0};
  real_t z[2] = {0};
  pinhole_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 2] = {0};

  for (size_t i = 0; i < 2; i++) {
    real_t z_fd[2] = {0};
    real_t p_C_fd[3] = {p_C[0], p_C[1], p_C[2]};
    p_C_fd[i] += h;
    pinhole_project(params, p_C_fd, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 2] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  MU_ASSERT(check_jacobian("J_point", J_numdiff, J_point, 2, 2, tol, 0) == 0);

  return 0;
}

int test_pinhole_params_jacobian() {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 1.0};
  const real_t x[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};
  real_t J_params[2 * 4] = {0};
  pinhole_params_jacobian(params, x, J_params);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 4] = {0};

  for (size_t i = 0; i < 4; i++) {
    real_t z_fd[2] = {0};
    real_t params_fd[4] = {params[0], params[1], params[2], params[3]};
    params_fd[i] += h;
    pinhole_project(params_fd, p_C, z_fd);

    J_numdiff[i + 0] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 4] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  MU_ASSERT(check_jacobian("J_params", J_numdiff, J_params, 2, 4, tol, 0) == 0);

  return 0;
}

int test_pinhole_radtan4_project() {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.3;
  const real_t k2 = 0.01;
  const real_t p1 = 0.01;
  const real_t p2 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, p1, p2};

  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t x[2] = {0};
  pinhole_radtan4_project(params, p_C, x);

  /* print_vector("x", x, 2); */
  MU_ASSERT(fltcmp(x[0], 323.204000) == 0);
  MU_ASSERT(fltcmp(x[1], 166.406400) == 0);

  return 0;
}

int test_pinhole_radtan4_project_jacobian() {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.3;
  const real_t k2 = 0.01;
  const real_t p1 = 0.01;
  const real_t p2 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, p1, p2};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(params, p_C, J);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_radtan4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 3] = {0};

  for (size_t i = 0; i < 3; i++) {
    real_t z_fd[2] = {0};
    real_t p_C_fd[3] = {p_C[0], p_C[1], p_C[2]};
    p_C_fd[i] += h;

    pinhole_radtan4_project(params, p_C_fd, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 3] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 3); */
  /* print_matrix("J", J, 2, 3); */
  MU_ASSERT(check_jacobian("J", J_numdiff, J, 2, 3, tol, 0) == 0);

  return 0;
}

int test_pinhole_radtan4_params_jacobian() {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.3;
  const real_t k2 = 0.01;
  const real_t p1 = 0.01;
  const real_t p2 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, p1, p2};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(params, p_C, J_params);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_radtan4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 8] = {0};

  for (size_t i = 0; i < 8; i++) {
    real_t z_fd[2] = {0};

    real_t params_fd[8] = {0};
    memcpy(params_fd, params, sizeof(real_t) * 8);
    params_fd[i] += h;

    pinhole_radtan4_project(params_fd, p_C, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 8] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 8); */
  /* print_matrix("J_params", J_params, 2, 8); */
  MU_ASSERT(check_jacobian("J_params", J_numdiff, J_params, 2, 8, tol, 0) == 0);

  return 0;
}

int test_pinhole_equi4_project() {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.1;
  const real_t k2 = 0.01;
  const real_t k3 = 0.01;
  const real_t k4 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, k3, k4};

  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t x[2] = {0};
  pinhole_equi4_project(params, p_C, x);

  /* print_vector("x", x, 2); */
  MU_ASSERT(fltcmp(x[0], 323.199627) == 0);
  MU_ASSERT(fltcmp(x[1], 166.399254) == 0);

  return 0;
}

int test_pinhole_equi4_project_jacobian() {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.1;
  const real_t k2 = 0.01;
  const real_t k3 = 0.01;
  const real_t k4 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, k3, k4};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J[2 * 3] = {0};
  pinhole_equi4_project_jacobian(params, p_C, J);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_equi4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 3] = {0};

  for (size_t i = 0; i < 3; i++) {
    real_t z_fd[2] = {0};
    real_t p_C_fd[3] = {p_C[0], p_C[1], p_C[2]};
    p_C_fd[i] += h;

    pinhole_equi4_project(params, p_C_fd, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 3] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 3); */
  /* print_matrix("J", J, 2, 3); */
  MU_ASSERT(check_jacobian("J", J_numdiff, J, 2, 3, tol, 0) == 0);

  return 0;
}

int test_pinhole_equi4_params_jacobian() {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.1;
  const real_t k2 = 0.01;
  const real_t k3 = 0.01;
  const real_t k4 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, k3, k4};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J_params[2 * 8] = {0};
  pinhole_equi4_params_jacobian(params, p_C, J_params);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_equi4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 8] = {0};

  for (size_t i = 0; i < 8; i++) {
    real_t z_fd[2] = {0};

    real_t params_fd[8] = {0};
    memcpy(params_fd, params, sizeof(real_t) * 8);
    params_fd[i] += h;

    pinhole_equi4_project(params_fd, p_C, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 8] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 8); */
  /* print_matrix("J_params", J_params, 2, 8); */
  MU_ASSERT(check_jacobian("J_params", J_numdiff, J_params, 2, 8, tol, 0) == 0);

  return 0;
}

/******************************************************************************
 * TEST SENSOR FUSION
 ******************************************************************************/

int test_pose_setup() {
  timestamp_t ts = 1;
  pose_t pose;

  real_t data[7] = {0.1, 0.2, 0.3, 1.0, 1.1, 2.2, 3.3};
  pose_setup(&pose, ts, data);

  MU_ASSERT(pose.ts == 1);

  MU_ASSERT(fltcmp(pose.data[0], 0.1) == 0.0);
  MU_ASSERT(fltcmp(pose.data[1], 0.2) == 0.0);
  MU_ASSERT(fltcmp(pose.data[2], 0.3) == 0.0);
  MU_ASSERT(fltcmp(pose.data[3], 1.0) == 0.0);
  MU_ASSERT(fltcmp(pose.data[4], 1.1) == 0.0);
  MU_ASSERT(fltcmp(pose.data[5], 2.2) == 0.0);
  MU_ASSERT(fltcmp(pose.data[6], 3.3) == 0.0);

  return 0;
}

int test_imu_biases_setup() {
  timestamp_t ts = 1;
  imu_biases_t biases;

  real_t ba[3] = {1.0, 2.0, 3.0};
  real_t bg[3] = {4.0, 5.0, 6.0};
  imu_biases_setup(&biases, ts, ba, bg);

  MU_ASSERT(biases.ts == 1);

  MU_ASSERT(fltcmp(biases.ba[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(biases.ba[1], 2.0) == 0.0);
  MU_ASSERT(fltcmp(biases.ba[2], 3.0) == 0.0);

  MU_ASSERT(fltcmp(biases.bg[0], 4.0) == 0.0);
  MU_ASSERT(fltcmp(biases.bg[1], 5.0) == 0.0);
  MU_ASSERT(fltcmp(biases.bg[2], 6.0) == 0.0);

  return 0;
}

int test_feature_setup() {
  feature_t feature;

  real_t data[3] = {0.1, 0.2, 0.3};
  feature_setup(&feature, data);

  MU_ASSERT(fltcmp(feature.data[0], 0.1) == 0.0);
  MU_ASSERT(fltcmp(feature.data[1], 0.2) == 0.0);
  MU_ASSERT(fltcmp(feature.data[2], 0.3) == 0.0);

  return 0;
}

int test_extrinsics_setup() {
  extrinsics_t extrinsics;

  real_t data[7] = {1.0, 2.0, 3.0, 1.0, 0.1, 0.2, 0.3};
  extrinsics_setup(&extrinsics, data);

  MU_ASSERT(fltcmp(extrinsics.data[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsics.data[1], 2.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsics.data[2], 3.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsics.data[3], 1.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsics.data[4], 0.1) == 0.0);
  MU_ASSERT(fltcmp(extrinsics.data[5], 0.2) == 0.0);
  MU_ASSERT(fltcmp(extrinsics.data[6], 0.3) == 0.0);

  return 0;
}

int test_camera_params_setup() {
  camera_params_t camera;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&camera, cam_idx, cam_res, proj_model, dist_model, data);
  camera_params_print(&camera);

  return 0;
}

int test_pose_factor_setup() {
  /* Pose */
  timestamp_t ts = 1;
  pose_t pose;
  real_t data[7] = {0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0};
  pose_setup(&pose, ts, data);

  /* Setup pose factor */
  pose_factor_t pose_factor;
  real_t var[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  pose_factor_setup(&pose_factor, &pose, var);

  // print_matrix("pose_factor.pos_meas", pose_factor.pos_meas, 3, 1);
  // print_matrix("pose_factor.quat_meas", pose_factor.quat_meas, 4, 1);
  // print_matrix("pose_factor.covar", pose_factor.covar, 6, 6);

  return 0;
}

int test_pose_factor_eval() {
  /* Pose */
  timestamp_t ts = 1;
  pose_t pose;
  real_t data[7] = {0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0};
  pose_setup(&pose, ts, data);

  /* Setup pose factor */
  pose_factor_t pose_factor;
  real_t var[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  pose_factor_setup(&pose_factor, &pose, var);

  /* Evaluate pose factor */
  real_t *params[1] = {pose.data};
  real_t r[6] = {0};
  real_t J[6 * 6] = {0};
  real_t *jacs[1] = {J};
  const int retval = pose_factor_eval(&pose_factor, params, r, jacs);
  MU_ASSERT(retval == 0);

  /* Check Jacobians */
  const real_t step_size = 1e-8;
  const real_t tol = 1e-4;

  /* -- Check pose position jacobian */
  real_t J_fdiff[6 * 6] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[6] = {0};
    real_t r_diff[6] = {0};

    params[0][i] += step_size;
    pose_factor_eval(&pose_factor, params, r_fwd, NULL);
    params[0][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 6);
    vec_scale(r_diff, 6, 1.0 / step_size);
    mat_col_set(J_fdiff, 6, 6, i, r_diff);
  }
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[6] = {0};
    real_t r_diff[6] = {0};

    quat_perturb(params[0] + 3, i, step_size);
    pose_factor_eval(&pose_factor, params, r_fwd, NULL);
    quat_perturb(params[0] + 3, i, -step_size);

    vec_sub(r_fwd, r, r_diff, 6);
    vec_scale(r_diff, 6, 1.0 / step_size);
    mat_col_set(J_fdiff, 6, 6, i + 3, r_diff);
  }
  MU_ASSERT(check_jacobian("J", J_fdiff, J, 6, 6, tol, 0) == 0);

  return 0;
}

int test_ba_factor_setup() {
  /* Timestamp */
  timestamp_t ts = 0;

  /* Camera pose */
  const real_t pose_data[7] = {0.01, 0.02, 0.0, -0.5, 0.5, -0.5, 0.5};
  pose_t pose;
  pose_setup(&pose, ts, pose_data);

  /* Feature */
  const real_t p_W[3] = {1.0, 0.0, 0.0};
  feature_t feature;
  feature_setup(&feature, p_W);

  /* Camera parameters */
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_t cam;
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  /* Project point from world to image plane */
  real_t T_WC[4 * 4] = {0};
  real_t T_CW[4 * 4] = {0};
  real_t p_C[3] = {0};
  real_t z[2];
  tf(pose_data, T_WC);
  tf_inv(T_WC, T_CW);
  tf_point(T_CW, p_W, p_C);
  pinhole_radtan4_project(cam_data, p_C, z);

  /* Bundle adjustment factor */
  ba_factor_t ba_factor;
  real_t var[2] = {1.0, 1.0};
  ba_factor_setup(&ba_factor, &pose, &feature, &cam, z, var);

  return 0;
}

int test_ba_factor_eval() {
  // Timestamp
  timestamp_t ts = 0;

  // Camera pose
  const real_t pose_data[7] = {0.01, 0.01, 0.0, 0.5, -0.5, 0.5, -0.5};
  pose_t pose;
  pose_setup(&pose, ts, pose_data);

  // Feature
  const real_t p_W[3] = {1.0, 0.1, 0.2};
  feature_t feature;
  feature_setup(&feature, p_W);

  // Camera parameters
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {320, 240, 320, 240, 0.03, 0.01, 0.001, 0.001};
  camera_params_t cam;
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  // Project point from world to image plane
  real_t T_WC[4 * 4] = {0};
  real_t T_CW[4 * 4] = {0};
  real_t p_C[3] = {0.0};
  real_t z[2] = {0.0};
  tf(pose_data, T_WC);
  tf_inv(T_WC, T_CW);
  tf_point(T_CW, p_W, p_C);
  pinhole_radtan4_project(cam_data, p_C, z);

  // Bundle adjustment factor
  ba_factor_t ba_factor;
  real_t var[2] = {1.0, 1.0};
  ba_factor_setup(&ba_factor, &pose, &feature, &cam, z, var);

  // Evaluate bundle adjustment factor
  real_t *params[3] = {pose.data, feature.data, cam.data};
  real_t r[2] = {0};
  real_t J0[2 * 6] = {0};
  real_t J1[2 * 3] = {0};
  real_t J2[2 * 8] = {0};
  real_t *jacs[3] = {J0, J1, J2};
  ba_factor_eval(&ba_factor, params, r, jacs);

  // Check Jacobians
  const real_t step_size = 1e-8;
  const real_t tol = 1e-4;

  // -- Check pose jacobian
  real_t J0_fdiff[2 * 6] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[0][i] += step_size;
    ba_factor_eval(&ba_factor, params, r_fwd, NULL);
    params[0][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J0_fdiff, 6, 2, i, r_diff);
  }
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    quat_perturb(params[0] + 3, i, step_size);
    ba_factor_eval(&ba_factor, params, r_fwd, NULL);
    quat_perturb(params[0] + 3, i, -step_size);

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J0_fdiff, 6, 2, i + 3, r_diff);
  }
  MU_ASSERT(check_jacobian("J0", J0_fdiff, J0, 2, 6, tol, 0) == 0);

  // -- Check feature jacobian
  real_t J1_numdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[1][i] += step_size;
    ba_factor_eval(&ba_factor, params, r_fwd, NULL);
    params[1][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J1_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J1", J1_numdiff, J1, 2, 3, tol, 0) == 0);

  // -- Check camera parameters jacobian
  real_t J2_numdiff[2 * 8] = {0};
  for (int i = 0; i < 8; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[2][i] += step_size;
    ba_factor_eval(&ba_factor, params, r_fwd, NULL);
    params[2][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J2_numdiff, 8, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J2", J2_numdiff, J2, 2, 8, tol, 1) == 0);

  return 0;
}

int test_vision_factor_setup() {
  // Timestamp
  timestamp_t ts = 0;

  // Body pose
  pose_t pose;
  const real_t pose_data[7] = {0.01, 0.02, 0.0, 0.5, 0.5, -0.5, -0.5};
  pose_setup(&pose, ts, pose_data);

  // Extrinsics
  extrinsics_t extrinsics;
  const real_t exts_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  extrinsics_setup(&extrinsics, exts_data);

  // Feature
  feature_t feature;
  const real_t p_W[3] = {1.0, 0.0, 0.0};
  feature_setup(&feature, p_W);

  // Camera parameters
  camera_params_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  // Project point from world to image plane
  real_t T_WB[4 * 4] = {0};
  real_t T_BW[4 * 4] = {0};
  real_t T_BCi[4 * 4] = {0};
  real_t T_CiB[4 * 4] = {0};
  real_t T_CiW[4 * 4] = {0};
  real_t p_Ci[3] = {0};
  real_t z[2];
  tf(pose_data, T_WB);
  tf(exts_data, T_BCi);
  tf_inv(T_WB, T_BW);
  tf_inv(T_BCi, T_CiB);
  dot(T_CiB, 4, 4, T_BW, 4, 4, T_CiW);
  tf_point(T_CiW, p_W, p_Ci);
  pinhole_radtan4_project(cam_data, p_Ci, z);

  // Camera factor
  vision_factor_t vision_factor;
  real_t var[2] = {1.0, 1.0};
  vision_factor_setup(&vision_factor,
                      &pose,
                      &extrinsics,
                      &feature,
                      &cam,
                      z,
                      var);

  return 0;
}

int test_vision_factor_eval() {
  // Timestamp
  timestamp_t ts = 0;

  // Body pose
  pose_t pose;
  const real_t pose_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  pose_setup(&pose, ts, pose_data);

  // Extrinsics
  extrinsics_t cam_exts;
  const real_t exts_data[7] = {0.01, 0.02, 0.03, 0.5, 0.5, -0.5, -0.5};
  extrinsics_setup(&cam_exts, exts_data);

  // Feature
  feature_t feature;
  const real_t p_W[3] = {1.0, 0.0, 0.0};
  feature_setup(&feature, p_W);

  // Camera parameters
  camera_params_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {320, 240, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  // Project point from world to image plane
  real_t T_WB[4 * 4] = {0};
  real_t T_BW[4 * 4] = {0};
  real_t T_BCi[4 * 4] = {0};
  real_t T_CiB[4 * 4] = {0};
  real_t T_CiW[4 * 4] = {0};
  real_t p_Ci[3] = {0};
  real_t z[2];
  tf(pose_data, T_WB);
  tf(exts_data, T_BCi);
  tf_inv(T_WB, T_BW);
  tf_inv(T_BCi, T_CiB);
  dot(T_CiB, 4, 4, T_BW, 4, 4, T_CiW);
  tf_point(T_CiW, p_W, p_Ci);
  pinhole_radtan4_project(cam_data, p_Ci, z);

  // Setup camera factor
  vision_factor_t vision_factor;
  real_t var[2] = {1.0, 1.0};
  vision_factor_setup(&vision_factor, &pose, &cam_exts, &feature, &cam, z, var);

  // Evaluate camera factor
  real_t *params[4] = {pose.data, cam_exts.data, cam.data, feature.data};
  real_t r[2] = {0};
  real_t J0[2 * 6] = {0};
  real_t J1[2 * 6] = {0};
  real_t J2[2 * 8] = {0};
  real_t J3[2 * 3] = {0};
  real_t *jacs[4] = {J0, J1, J2, J3};
  vision_factor_eval(&vision_factor, params, r, jacs);

  // Check Jacobians
  real_t step_size = 1e-8;
  real_t tol = 1e-4;

  // -- Check pose position jacobian
  real_t J0_fdiff[2 * 6] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[0][i] += step_size;
    vision_factor_eval(&vision_factor, params, r_fwd, NULL);
    params[0][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J0_fdiff, 6, 2, i, r_diff);
  }
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    quat_perturb(params[0] + 3, i, step_size);
    vision_factor_eval(&vision_factor, params, r_fwd, NULL);
    quat_perturb(params[0] + 3, i, -step_size);

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J0_fdiff, 6, 2, i + 3, r_diff);
  }
  MU_ASSERT(check_jacobian("J0", J0_fdiff, J0, 2, 6, tol, 0) == 0);

  // -- Check extrinsics position jacobian
  real_t J1_fdiff[2 * 6] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[1][i] += step_size;
    vision_factor_eval(&vision_factor, params, r_fwd, NULL);
    params[1][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J1_fdiff, 6, 2, i, r_diff);
  }
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    quat_perturb(params[1] + 3, i, step_size);
    vision_factor_eval(&vision_factor, params, r_fwd, NULL);
    quat_perturb(params[1] + 3, i, -step_size);

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J1_fdiff, 6, 2, i + 3, r_diff);
  }
  MU_ASSERT(check_jacobian("J1", J1_fdiff, J1, 2, 6, tol, 0) == 0);

  // -- Check camera parameters jacobian
  real_t J2_fdiff[2 * 8] = {0};
  for (int i = 0; i < 8; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[2][i] += step_size;
    vision_factor_eval(&vision_factor, params, r_fwd, NULL);
    params[2][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J2_fdiff, 8, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J2", J2_fdiff, J2, 2, 8, tol, 0) == 0);

  // -- Check feature jacobian
  real_t J3_fdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[3][i] += step_size;
    vision_factor_eval(&vision_factor, params, r_fwd, NULL);
    params[3][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J3_fdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J3", J3_fdiff, J3, 2, 3, tol, 0) == 0);

  return 0;
}

static void setup_calib_gimbal_factor(calib_gimbal_factor_t *factor,
                                      extrinsics_t *fiducial,
                                      extrinsics_t *link0,
                                      extrinsics_t *link1,
                                      extrinsics_t *link2,
                                      joint_angle_t *joint0,
                                      joint_angle_t *joint1,
                                      joint_angle_t *joint2,
                                      extrinsics_t *cam_exts,
                                      camera_params_t *cam) {
  // Body pose T_WB
  real_t ypr_WB[3] = {0.0, 0.0, 0.0};
  real_t r_WB[3] = {0.0, 0.0, 0.0};
  real_t T_WB[4 * 4] = {0};
  tf_er(ypr_WB, r_WB, T_WB);

  // Fiducial pose T_WF
  real_t ypr_WF[3] = {-M_PI / 2.0, 0.0, M_PI / 2.0};
  real_t r_WF[3] = {0.5, 0.0, 0.0};
  real_t T_WF[4 * 4] = {0};
  tf_er(ypr_WF, r_WF, T_WF);

  // Relative fiducial pose T_BF
  real_t T_BW[4 * 4] = {0};
  real_t T_BF[4 * 4] = {0};
  tf_inv(T_WB, T_BW);
  dot(T_BW, 4, 4, T_WF, 4, 4, T_BF);

  real_t x_BF[7] = {0};
  tf_vector(T_BF, x_BF);
  extrinsics_setup(fiducial, x_BF);

  // Yaw link
  real_t ypr_BM0b[3] = {0.0, 0.0, 0.0};
  real_t r_BM0b[3] = {0.0, 0.0, 0.0};
  real_t T_BM0b[4 * 4] = {0};
  gimbal_setup_extrinsics(ypr_BM0b, r_BM0b, T_BM0b, link0);

  // Roll link
  real_t ypr_M0eM1b[3] = {0.0, M_PI / 2, 0.0};
  real_t r_M0eM1b[3] = {-0.1, 0.0, 0.15};
  real_t T_M0eM1b[4 * 4] = {0};
  gimbal_setup_extrinsics(ypr_M0eM1b, r_M0eM1b, T_M0eM1b, link1);

  // Pitch link
  real_t ypr_M1eM2b[3] = {0.0, 0.0, -M_PI / 2.0};
  real_t r_M1eM2b[3] = {0.0, -0.05, 0.1};
  real_t T_M1eM2b[4 * 4] = {0};
  gimbal_setup_extrinsics(ypr_M1eM2b, r_M1eM2b, T_M1eM2b, link2);

  // Joint0
  const real_t th0 = 0.0;
  real_t T_M0bM0e[4 * 4] = {0};
  gimbal_setup_joint(0, th0, T_M0bM0e, joint0);

  // Joint1
  const real_t th1 = 0.0;
  real_t T_M1bM1e[4 * 4] = {0};
  gimbal_setup_joint(1, th1, T_M1bM1e, joint1);

  // Joint2
  const real_t th2 = 0.0;
  real_t T_M2bM2e[4 * 4] = {0};
  gimbal_setup_joint(2, th2, T_M2bM2e, joint2);

  // Camera extrinsics T_M2eCi
  const real_t ypr_M2eC0[3] = {-M_PI / 2, M_PI / 2, 0.0};
  const real_t r_M2eC0[3] = {0.0, -0.05, 0.12};
  real_t T_M2eC0[4 * 4] = {0};
  gimbal_setup_extrinsics(ypr_M2eC0, r_M2eC0, T_M2eC0, cam_exts);

  // Camera parameters K
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(cam_res[0], fov);
  const real_t fy = pinhole_focal(cam_res[0], fov);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const real_t cam_params[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(cam,
                      cam_idx,
                      cam_res,
                      proj_model,
                      dist_model,
                      cam_params);

  // Form T_C0f
  const real_t *T_chain[7] = {
      T_BM0b,
      T_M0bM0e,
      T_M0eM1b,
      T_M1bM1e,
      T_M1eM2b,
      T_M2bM2e,
      T_M2eC0,
  };
  real_t T_BC0[4 * 4] = {0};
  real_t T_C0B[4 * 4] = {0};
  real_t T_C0F[4 * 4] = {0};
  tf_chain(T_chain, 7, T_BC0);
  tf_inv(T_BC0, T_C0B);
  dot(T_C0B, 4, 4, T_BF, 4, 4, T_C0F);

  // Project point to image plane
  const real_t p_FFi[3] = {0.0, 0.0, 0.0};
  real_t p_C0Fi[3] = {0};
  real_t z[2] = {0};
  tf_point(T_C0F, p_FFi, p_C0Fi);
  pinhole_radtan4_project(cam_params, p_C0Fi, z);

  // Setup factor
  const int tag_id = 0;
  const int corner_idx = 0;
  const real_t var[2] = {1.0, 1.0};
  calib_gimbal_factor_setup(factor,
                            fiducial,
                            link0,
                            link1,
                            link2,
                            joint0,
                            joint1,
                            joint2,
                            cam_exts,
                            cam,
                            tag_id,
                            corner_idx,
                            p_FFi,
                            z,
                            var);
}

int test_calib_gimbal_factor_setup() {
  calib_gimbal_factor_t factor;
  extrinsics_t fiducial;
  extrinsics_t link0;
  extrinsics_t link1;
  extrinsics_t link2;
  joint_angle_t joint0;
  joint_angle_t joint1;
  joint_angle_t joint2;
  extrinsics_t cam_exts;
  camera_params_t cam;
  setup_calib_gimbal_factor(&factor,
                            &fiducial,
                            &link0,
                            &link1,
                            &link2,
                            &joint0,
                            &joint1,
                            &joint2,
                            &cam_exts,
                            &cam);

  return 0;
}

int test_calib_gimbal_factor_eval() {
  calib_gimbal_factor_t factor;
  extrinsics_t fiducial;
  extrinsics_t link0;
  extrinsics_t link1;
  extrinsics_t link2;
  joint_angle_t joint0;
  joint_angle_t joint1;
  joint_angle_t joint2;
  extrinsics_t cam_exts;
  camera_params_t cam;
  setup_calib_gimbal_factor(&factor,
                            &fiducial,
                            &link0,
                            &link1,
                            &link2,
                            &joint0,
                            &joint1,
                            &joint2,
                            &cam_exts,
                            &cam);

  real_t *params[9] = {fiducial.data,
                       link0.data,
                       link1.data,
                       link2.data,
                       joint0.angle,
                       joint1.angle,
                       joint2.angle,
                       cam_exts.data,
                       cam.data};

  // Residual
  real_t r[2] = {0};

  // Jacobians
  real_t J_fiducial[2 * 6] = {0};
  real_t J_link0[2 * 6] = {0};
  real_t J_link1[2 * 6] = {0};
  real_t J_link2[2 * 6] = {0};
  real_t J_joint0[2 * 1] = {0};
  real_t J_joint1[2 * 1] = {0};
  real_t J_joint2[2 * 1] = {0};
  real_t J_cam_exts[2 * 6] = {0};
  real_t J_cam_params[2 * 8] = {0};
  real_t *jacs[9] = {J_fiducial,
                     J_link0,
                     J_link1,
                     J_link2,
                     J_joint0,
                     J_joint1,
                     J_joint2,
                     J_cam_exts,
                     J_cam_params};
  calib_gimbal_factor_eval(&factor, params, r, jacs);

  // Check Jacobians
  const double tol = 1e-4;
  const double step_size = 1e-8;

  // -- Check fiducial Jacobians
  CHECK_POSE_JACOBIAN("J_fiducial",
                      0,
                      r,
                      2,
                      params,
                      jacs,
                      &factor,
                      calib_gimbal_factor_eval,
                      step_size,
                      tol,
                      0);

  // -- Check link Jacobians
  for (int link_idx = 0; link_idx < 3; link_idx++) {
    char jac_name[100] = {'\0'};
    sprintf(jac_name, "J_link%d", link_idx);

    CHECK_POSE_JACOBIAN(jac_name,
                        1 + link_idx,
                        r,
                        2,
                        params,
                        jacs,
                        &factor,
                        calib_gimbal_factor_eval,
                        step_size,
                        tol,
                        0);
  }

  // -- Check joint Jacobians
  for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
    char jac_name[100] = {'\0'};
    sprintf(jac_name, "J_joint%d", joint_idx);

    int idx = 4 + joint_idx;
    real_t J_fdiff[2 * 1] = {0};
    real_t r_fwd[2] = {0};

    params[idx][0] += step_size;
    calib_gimbal_factor_eval(&factor, params, r_fwd, NULL);
    params[idx][0] -= step_size;

    vec_sub(r_fwd, r, J_fdiff, 2);
    vec_scale(J_fdiff, 2, 1.0 / step_size);
    MU_ASSERT(check_jacobian(jac_name, J_fdiff, jacs[idx], 2, 1, tol, 0) == 0);
  }

  // -- Check camera extrinsics Jacobian
  CHECK_POSE_JACOBIAN("J_cam_exts",
                      7,
                      r,
                      2,
                      params,
                      jacs,
                      &factor,
                      calib_gimbal_factor_eval,
                      step_size,
                      tol,
                      0);

  // -- Check camera parameter Jacobians
  {
    const char *jac_name = "J_cam";

    real_t J_fdiff[2 * 8] = {0};
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    for (int i = 0; i < 8; i++) {
      params[8][i] += step_size;
      calib_gimbal_factor_eval(&factor, params, r_fwd, NULL);
      params[8][i] -= step_size;

      vec_sub(r_fwd, r, r_diff, 2);
      vec_scale(r_diff, 2, 1.0 / step_size);
      mat_col_set(J_fdiff, 8, 2, i, r_diff);
    }
    MU_ASSERT(check_jacobian(jac_name, J_fdiff, jacs[8], 2, 8, tol, 0) == 0);
  }

  return 0;
}

int test_imu_buf_setup() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  return 0;
}

int test_imu_buf_add() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);

  MU_ASSERT(imu_buf.size == 1);
  MU_ASSERT(imu_buf.ts[0] == ts);
  MU_ASSERT(fltcmp(imu_buf.acc[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][2], 3.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][2], 3.0) == 0);

  return 0;
}

int test_imu_buf_clear() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);
  imu_buf_clear(&imu_buf);

  MU_ASSERT(imu_buf.size == 0);
  MU_ASSERT(imu_buf.ts[0] == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][0], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][1], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][2], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][0], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][1], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][2], 0.0) == 0);

  return 0;
}

int test_imu_buf_copy() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);

  imu_buf_t imu_buf2;
  imu_buf_setup(&imu_buf2);
  imu_buf_copy(&imu_buf, &imu_buf2);

  MU_ASSERT(imu_buf2.size == 1);
  MU_ASSERT(imu_buf2.ts[0] == ts);
  MU_ASSERT(fltcmp(imu_buf2.acc[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.acc[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.acc[0][2], 3.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.gyr[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.gyr[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.gyr[0][2], 3.0) == 0);

  return 0;
}

typedef struct imu_test_data_t {
  size_t nb_measurements;
  real_t *timestamps;
  real_t **poses;
  real_t **velocities;
  real_t **imu_acc;
  real_t **imu_gyr;
} imu_test_data_t;

static int setup_imu_test_data(imu_test_data_t *test_data) {
  // Circle trajectory configurations
  const real_t imu_rate = 200.0;
  const real_t circle_r = 1.0;
  const real_t circle_v = 0.1;
  const real_t circle_dist = 2.0 * M_PI * circle_r;
  const real_t time_taken = circle_dist / circle_v;
  const real_t w = -2.0 * M_PI * (1.0 / time_taken);
  const real_t theta_init = M_PI;
  const real_t yaw_init = M_PI / 2.0;

  // Allocate memory for test data
  test_data->nb_measurements = time_taken * imu_rate;
  test_data->timestamps = MALLOC(real_t, test_data->nb_measurements);
  test_data->poses = MALLOC(real_t *, test_data->nb_measurements);
  test_data->velocities = MALLOC(real_t *, test_data->nb_measurements);
  test_data->imu_acc = MALLOC(real_t *, test_data->nb_measurements);
  test_data->imu_gyr = MALLOC(real_t *, test_data->nb_measurements);

  // Simulate IMU poses
  const real_t dt = 1.0 / imu_rate;
  timestamp_t ts = 0.0;
  real_t theta = theta_init;
  real_t yaw = yaw_init;

  for (size_t k = 0; k < test_data->nb_measurements; k++) {
    // IMU pose
    // -- Position
    const real_t rx = circle_r * cos(theta);
    const real_t ry = circle_r * sin(theta);
    const real_t rz = 0.0;
    // -- Orientation
    const real_t ypr[3] = {yaw, 0.0, 0.0};
    real_t q[4] = {0};
    euler2quat(ypr, q);
    // -- Pose vector
    const real_t pose[7] = {rx, ry, rz, q[0], q[1], q[2], q[3]};
    // print_vector("pose", pose, 7);

    // Velocity
    const real_t vx = -circle_r * w * sin(theta);
    const real_t vy = circle_r * w * cos(theta);
    const real_t vz = 0.0;
    const real_t v_WS[3] = {vx, vy, vz};

    // Acceleration
    const real_t ax = -circle_r * w * w * cos(theta);
    const real_t ay = -circle_r * w * w * sin(theta);
    const real_t az = 0.0;
    const real_t a_WS[3] = {ax, ay, az};

    // Angular velocity
    const real_t wx = 0.0;
    const real_t wy = 0.0;
    const real_t wz = w;
    const real_t w_WS[3] = {wx, wy, wz};

    // IMU measurements
    real_t C_WS[3 * 3] = {0};
    real_t C_SW[3 * 3] = {0};
    quat2rot(q, C_WS);
    mat_transpose(C_WS, 3, 3, C_SW);
    // -- Accelerometer measurement
    real_t acc[3] = {0};
    dot(C_SW, 3, 3, a_WS, 3, 1, acc);
    acc[2] += 10.0;
    // -- Gyroscope measurement
    real_t gyr[3] = {0};
    dot(C_SW, 3, 3, w_WS, 3, 1, gyr);

    // Update
    test_data->timestamps[k] = ts;
    test_data->poses[k] = vector_malloc(pose, 7);
    test_data->velocities[k] = vector_malloc(v_WS, 3);
    test_data->imu_acc[k] = vector_malloc(acc, 3);
    test_data->imu_gyr[k] = vector_malloc(gyr, 3);

    theta += w * dt;
    yaw += w * dt;
    ts += sec2ts(dt);
  }

  return 0;
}

static void free_imu_test_data(imu_test_data_t *test_data) {
  for (size_t k = 0; k < test_data->nb_measurements; k++) {
    free(test_data->poses[k]);
    free(test_data->velocities[k]);
    free(test_data->imu_acc[k]);
    free(test_data->imu_gyr[k]);
  }

  free(test_data->timestamps);
  free(test_data->poses);
  free(test_data->velocities);
  free(test_data->imu_acc);
  free(test_data->imu_gyr);
}

int test_imu_factor_propagate_step() {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data);

  // Setup IMU buffer
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);
  for (int k = 0; k < 9; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *acc = test_data.imu_acc[k];
    const real_t *gyr = test_data.imu_gyr[k];
    imu_buf_add(&imu_buf, ts, acc, gyr);
  }

  // Setup state
  // const real_t *pose_init = test_data.poses[0];
  // const real_t *vel_init = test_data.velocities[0];
  real_t r[3] = {0.0, 0.0, 0.0};
  real_t v[3] = {0.0, 0.0, 0.0};
  real_t q[4] = {1.0, 0.0, 0.0, 0.0};
  real_t ba[3] = {0};
  real_t bg[3] = {0};

  // Integrate imu measuremenets
  FILE *est_csv = fopen("/tmp/imu_est.csv", "w");
  fprintf(est_csv, "ts,rx,ry,rz,qw,qx,qy,qz,vx,vy,vz\n");

  real_t Dt = 0.0;
  real_t dt = 0.0;
  for (int k = 0; k < imu_buf.size; k++) {
    if (k + 1 < imu_buf.size) {
      const timestamp_t ts_i = imu_buf.ts[k];
      const timestamp_t ts_j = imu_buf.ts[k + 1];
      dt = ts2sec(ts_j) - ts2sec(ts_i);
    }
    const timestamp_t ts = imu_buf.ts[k];
    const real_t *a = imu_buf.acc[k];
    const real_t *w = imu_buf.gyr[k];
    imu_factor_propagate_step(r, v, q, ba, bg, a, w, dt);
    Dt += dt;

    // real_t C_est[3 * 3] = {0};
    // real_t C_gnd[3 * 3] = {0};
    // real_t C_gnd_T[3 * 3] = {0};
    // real_t dC[3 * 3] = {0};
    // const real_t *pose = test_data.poses[k];
    // const real_t q_gnd[4] = {pose[3], pose[4], pose[5], pose[6]};
    // quat2rot(q, C_est);
    // quat2rot(q_gnd, C_gnd);
    // mat_transpose(C_gnd, 3, 3, C_gnd_T);
    // dot(C_gnd_T, 3, 3, C_est, 3, 3, dC);
    // const real_t ddeg = rad2deg(acos((mat_trace(dC, 3, 3) - 1.0) / 2.0));

    // const real_t *pose = test_data.poses[k];
    // const real_t r_gnd[3] = {pose[0], pose[1], pose[2]};
    // const real_t dr[3] = {r_gnd[0] - r[0], r_gnd[1] - r[1], r_gnd[2] -
    // r[2]}; const real_t dpos = vec_norm(dr, 3); printf("dpos: %f\n", dpos);

    fprintf(est_csv, "%ld,", ts);
    fprintf(est_csv, "%f,%f,%f,", r[0], r[1], r[2]);
    fprintf(est_csv, "%f,%f,%f,%f,", q[0], q[1], q[2], q[3]);
    fprintf(est_csv, "%f,%f,%f\n", v[0], v[1], v[2]);
  }
  fclose(est_csv);

  // printf("dr: %f, %f, %f\n", r[0], r[1], r[2]);
  // printf("dv: %f, %f, %f\n", v[0], v[1], v[2]);
  // printf("dq: %f, %f, %f, %f\n", q[0], q[1], q[2], q[3]);
  // printf("Dt: %f\n", Dt);

  // Save ground-truth data
  FILE *gnd_csv = fopen("/tmp/imu_gnd.csv", "w");
  fprintf(gnd_csv, "ts,rx,ry,rz,qw,qx,qy,qz,vx,vy,vz\n");
  for (size_t k = 0; k < test_data.nb_measurements; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *pose = test_data.poses[k];
    const real_t *v = test_data.velocities[k];
    const real_t r[3] = {pose[0], pose[1], pose[2]};
    const real_t q[4] = {pose[3], pose[4], pose[5], pose[6]};

    fprintf(gnd_csv, "%ld,", ts);
    fprintf(gnd_csv, "%f,%f,%f,", r[0], r[1], r[2]);
    fprintf(gnd_csv, "%f,%f,%f,%f,", q[0], q[1], q[2], q[3]);
    fprintf(gnd_csv, "%f,%f,%f\n", v[0], v[1], v[2]);
  }
  fclose(gnd_csv);

  return 0;
}

int test_imu_factor_setup() {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data);

  // Setup IMU buffer
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);
  for (int k = 0; k < 10; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *acc = test_data.imu_acc[k];
    const real_t *gyr = test_data.imu_gyr[k];
    imu_buf_add(&imu_buf, ts, acc, gyr);
  }

  // Setup IMU factor
  const int idx_i = 0;
  const int idx_j = 10 - 1;
  const timestamp_t ts_i = test_data.timestamps[idx_i];
  const timestamp_t ts_j = test_data.timestamps[idx_j];
  const real_t *v_i = test_data.velocities[idx_i];
  const real_t ba_i[3] = {0, 0, 0};
  const real_t bg_i[3] = {0, 0, 0};
  const real_t *v_j = test_data.velocities[idx_j];
  const real_t ba_j[3] = {0, 0, 0};
  const real_t bg_j[3] = {0, 0, 0};
  pose_t pose_i;
  pose_t pose_j;
  velocity_t vel_i;
  velocity_t vel_j;
  imu_biases_t biases_i;
  imu_biases_t biases_j;
  pose_setup(&pose_i, ts_i, test_data.poses[idx_i]);
  pose_setup(&pose_j, ts_j, test_data.poses[idx_j]);
  velocity_setup(&vel_i, ts_i, v_i);
  velocity_setup(&vel_j, ts_j, v_j);
  imu_biases_setup(&biases_i, ts_i, ba_i, bg_i);
  imu_biases_setup(&biases_j, ts_j, ba_j, bg_j);

  imu_params_t imu_params;
  imu_params.imu_idx = 0;
  imu_params.rate = 200.0;
  imu_params.sigma_a = 0.08;
  imu_params.sigma_g = 0.004;
  imu_params.sigma_aw = 0.00004;
  imu_params.sigma_gw = 2.0e-6;
  imu_params.g = 9.81;

  imu_factor_t imu_factor;
  imu_factor_setup(&imu_factor,
                   &imu_params,
                   &imu_buf,
                   &pose_i,
                   &vel_i,
                   &biases_i,
                   &pose_j,
                   &vel_j,
                   &biases_j);

  // printf("idx_i: %d, idx_j: %d\n", idx_i, idx_j);
  // pose_print("pose_i", &pose_i);
  // pose_print("pose_j", &pose_j);
  // print_vector("dr", imu_factor.dr, 3);
  // print_vector("dv", imu_factor.dv, 3);
  // print_quat("dq", imu_factor.dq);
  // printf("Dt: %f\n", imu_factor.Dt);
  // mat_save("/tmp/F_test.csv", imu_factor.F, 15, 15);
  // mat_save("/tmp/P_test.csv", imu_factor.P, 15, 15);

  MU_ASSERT(imu_factor.pose_i == &pose_i);
  MU_ASSERT(imu_factor.vel_i == &vel_i);
  MU_ASSERT(imu_factor.biases_i == &biases_i);
  MU_ASSERT(imu_factor.pose_i == &pose_i);
  MU_ASSERT(imu_factor.vel_j == &vel_j);
  MU_ASSERT(imu_factor.biases_j == &biases_j);

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

int test_imu_factor_eval() {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data);

  // Setup IMU buffer
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);
  for (int k = 0; k < 10; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *acc = test_data.imu_acc[k];
    const real_t *gyr = test_data.imu_gyr[k];
    imu_buf_add(&imu_buf, ts, acc, gyr);
  }

  // Setup IMU factor
  const int idx_i = 0;
  const int idx_j = 10 - 1;
  const timestamp_t ts_i = test_data.timestamps[idx_i];
  const timestamp_t ts_j = test_data.timestamps[idx_j];
  const real_t *v_i = test_data.velocities[idx_i];
  const real_t ba_i[3] = {0, 0, 0};
  const real_t bg_i[3] = {0, 0, 0};
  const real_t *v_j = test_data.velocities[idx_j];
  const real_t ba_j[3] = {0, 0, 0};
  const real_t bg_j[3] = {0, 0, 0};
  pose_t pose_i;
  pose_t pose_j;
  velocity_t vel_i;
  velocity_t vel_j;
  imu_biases_t biases_i;
  imu_biases_t biases_j;
  pose_setup(&pose_i, ts_i, test_data.poses[idx_i]);
  pose_setup(&pose_j, ts_j, test_data.poses[idx_j]);
  velocity_setup(&vel_i, ts_i, v_i);
  velocity_setup(&vel_j, ts_j, v_j);
  imu_biases_setup(&biases_i, ts_i, ba_i, bg_i);
  imu_biases_setup(&biases_j, ts_j, ba_j, bg_j);

  imu_params_t imu_params;
  imu_params.imu_idx = 0;
  imu_params.rate = 200.0;
  imu_params.sigma_a = 0.08;
  imu_params.sigma_g = 0.004;
  imu_params.sigma_aw = 0.00004;
  imu_params.sigma_gw = 2.0e-6;
  imu_params.g = 9.81;

  imu_factor_t imu_factor;
  imu_factor_setup(&imu_factor,
                   &imu_params,
                   &imu_buf,
                   &pose_i,
                   &vel_i,
                   &biases_i,
                   &pose_j,
                   &vel_j,
                   &biases_j);

  // Evaluate IMU factor
  real_t *params[8] = {pose_i.data,
                       vel_i.v,
                       biases_i.ba,
                       biases_i.bg,
                       pose_j.data,
                       vel_j.v,
                       biases_j.ba,
                       biases_j.bg};
  real_t r[15] = {0};
  real_t J0[15 * 3] = {0};
  real_t J1[15 * 3] = {0};
  real_t J2[15 * 3] = {0};
  real_t J3[15 * 3] = {0};
  real_t J4[15 * 3] = {0};
  real_t J5[15 * 3] = {0};
  real_t J6[15 * 3] = {0};
  real_t J7[15 * 3] = {0};
  real_t J8[15 * 3] = {0};
  real_t J9[15 * 3] = {0};
  real_t *jacs[10] = {J0, J1, J2, J3, J4, J5, J6, J7, J8, J9};
  imu_factor_eval(&imu_factor, params, r, jacs);
  // print_vector("r", r, 15);
  // print_matrix("J0", J0, 15, 3);

  // // Check Jacobians
  // const int r_size = 15;
  // real_t step_size = 1e-8;
  // real_t tol = 1e-4;

  // // -- Check pose i position jacobian
  // real_t J0_numdiff[15 * 3] = {0};
  // for (int i = 0; i < 3; i++) {
  //   real_t r_fwd[15] = {0};
  //   real_t r_diff[15] = {0};

  //   params[0][i] += step_size;
  //   imu_factor_eval(&imu_factor, params, r_fwd, NULL);
  //   params[0][i] -= step_size;

  //   vec_sub(r_fwd, r, r_diff, r_size);
  //   vec_scale(r_diff, r_size, 1.0 / step_size);
  //   mat_col_set(J0_numdiff, 3, r_size, i, r_diff);
  // }
  // print_matrix("J0_numdiff", J0_numdiff, 15, 3);
  // MU_ASSERT(check_jacobian("J0", J0_numdiff, J0, r_size, 3, tol, 1) == 0);

  // -- Check pose i rotation jacobian

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

#ifdef USE_CERES

/**
 * This is the equivalent of a use-defined CostFunction in the C++ Ceres API.
 * This is passed as a callback to the Ceres C API, which internally converts
 * the callback into a CostFunction.
 */
static int ceres_exp_residual(void *user_data,
                              double **parameters,
                              double *residuals,
                              double **jacobians) {
  double *measurement = (double *) user_data;
  double x = measurement[0];
  double y = measurement[1];
  double m = parameters[0][0];
  double c = parameters[1][0];
  residuals[0] = y - exp(m * x + c);
  if (jacobians == NULL) {
    return 1;
  }
  if (jacobians[0] != NULL) {
    jacobians[0][0] = -x * exp(m * x + c); /* dr/dm */
  }
  if (jacobians[1] != NULL) {
    jacobians[1][0] = -exp(m * x + c); /* dr/dc */
  }
  return 1;
}

int test_ceres_example() {
  int num_observations = 67;
  double data[] = {
      0.000000e+00, 1.133898e+00, 7.500000e-02, 1.334902e+00, 1.500000e-01,
      1.213546e+00, 2.250000e-01, 1.252016e+00, 3.000000e-01, 1.392265e+00,
      3.750000e-01, 1.314458e+00, 4.500000e-01, 1.472541e+00, 5.250000e-01,
      1.536218e+00, 6.000000e-01, 1.355679e+00, 6.750000e-01, 1.463566e+00,
      7.500000e-01, 1.490201e+00, 8.250000e-01, 1.658699e+00, 9.000000e-01,
      1.067574e+00, 9.750000e-01, 1.464629e+00, 1.050000e+00, 1.402653e+00,
      1.125000e+00, 1.713141e+00, 1.200000e+00, 1.527021e+00, 1.275000e+00,
      1.702632e+00, 1.350000e+00, 1.423899e+00, 1.425000e+00, 1.543078e+00,
      1.500000e+00, 1.664015e+00, 1.575000e+00, 1.732484e+00, 1.650000e+00,
      1.543296e+00, 1.725000e+00, 1.959523e+00, 1.800000e+00, 1.685132e+00,
      1.875000e+00, 1.951791e+00, 1.950000e+00, 2.095346e+00, 2.025000e+00,
      2.361460e+00, 2.100000e+00, 2.169119e+00, 2.175000e+00, 2.061745e+00,
      2.250000e+00, 2.178641e+00, 2.325000e+00, 2.104346e+00, 2.400000e+00,
      2.584470e+00, 2.475000e+00, 1.914158e+00, 2.550000e+00, 2.368375e+00,
      2.625000e+00, 2.686125e+00, 2.700000e+00, 2.712395e+00, 2.775000e+00,
      2.499511e+00, 2.850000e+00, 2.558897e+00, 2.925000e+00, 2.309154e+00,
      3.000000e+00, 2.869503e+00, 3.075000e+00, 3.116645e+00, 3.150000e+00,
      3.094907e+00, 3.225000e+00, 2.471759e+00, 3.300000e+00, 3.017131e+00,
      3.375000e+00, 3.232381e+00, 3.450000e+00, 2.944596e+00, 3.525000e+00,
      3.385343e+00, 3.600000e+00, 3.199826e+00, 3.675000e+00, 3.423039e+00,
      3.750000e+00, 3.621552e+00, 3.825000e+00, 3.559255e+00, 3.900000e+00,
      3.530713e+00, 3.975000e+00, 3.561766e+00, 4.050000e+00, 3.544574e+00,
      4.125000e+00, 3.867945e+00, 4.200000e+00, 4.049776e+00, 4.275000e+00,
      3.885601e+00, 4.350000e+00, 4.110505e+00, 4.425000e+00, 4.345320e+00,
      4.500000e+00, 4.161241e+00, 4.575000e+00, 4.363407e+00, 4.650000e+00,
      4.161576e+00, 4.725000e+00, 4.619728e+00, 4.800000e+00, 4.737410e+00,
      4.875000e+00, 4.727863e+00, 4.950000e+00, 4.669206e+00,
  };

  /* Note: Typically it is better to compact m and c into one block,
   * but in this case use separate blocks to illustrate the use of
   * multiple parameter blocks. */
  double m = 0.0;
  double c = 0.0;
  double *parameter_pointers[] = {&m, &c};
  int parameter_sizes[2] = {1, 1};
  ceres_problem_t *problem;
  ceres_init();
  problem = ceres_create_problem();

  /* Add all the residuals. */
  for (int i = 0; i < num_observations; ++i) {
    ceres_problem_add_residual_block(problem,
                                     ceres_exp_residual,
                                     &data[2 * i],
                                     NULL,
                                     NULL,
                                     1,
                                     2,
                                     parameter_sizes,
                                     parameter_pointers);
  }
  ceres_solve(problem);
  ceres_free_problem(problem);
  // printf("Initial m: 0.0, c: 0.0\n");
  // printf("Final m: %g, c: %g\n", m, c);

  return 0;
}

#endif // USE_CERES

int test_solver_setup() {
  solver_t solver;
  solver_setup(&solver);
  return 0;
}

typedef struct cam_view_t {
  pose_t pose;
  ba_factor_t factors[1000];
  int nb_factors;
  camera_params_t *cam_params;
} cam_view_t;

int test_solver_eval() {
  /* Load test data */
  const char *dir_path = TEST_SIM_DATA "/cam0";
  sim_camera_data_t *cam_data = sim_camera_data_load(dir_path);

  /* Camera parameters */
  camera_params_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t params[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, params);

  /* Features container */
  features_t features;
  features_setup(&features);

  /* Loop over simulated camera frames */
  const real_t var[2] = {1.0, 1.0};
  cam_view_t *cam_views = MALLOC(cam_view_t, cam_data->nb_frames);
  for (int k = 0; k < cam_data->nb_frames; k++) {
    /* Camera frame */
    const sim_camera_frame_t *frame = cam_data->frames[k];

    /* Pose */
    pose_t *pose = &cam_views[k].pose;
    pose_setup(pose, frame->ts, cam_data->poses[k]);

    /* Add factors */
    cam_views[k].nb_factors = frame->nb_measurements;
    for (int i = 0; i < frame->nb_measurements; i++) {
      const int feature_id = frame->feature_ids[i];
      const real_t *z = frame->keypoints[i];

      /* Feature */
      feature_t *feature = NULL;
      if (features_exists(&features, feature_id)) {
        feature = features_get(&features, feature_id);
      } else {
        const real_t param[3] = {0};
        feature = features_add(&features, feature_id, param);
      }

      /* Factor */
      ba_factor_t *factor = &cam_views[k].factors[i];
      ba_factor_setup(factor, pose, feature, &cam, z, var);
    }
  }

  /* solver_t solver; */
  /* solver_setup(&solver); */

  /* Clean up */
  free(cam_views);
  sim_camera_data_free(cam_data);

  return 0;
}

int test_calib_gimbal_load() {
  const char *data_path = "/tmp/sim_gimbal";
  calib_gimbal_t *calib = calib_gimbal_load(data_path);
  MU_ASSERT(calib != NULL);
  calib_gimbal_free(calib);

  return 0;
}

int test_calib_gimbal_solve() {
  // Setup
  const char *data_path = "/tmp/sim_gimbal";
  calib_gimbal_t *calib = calib_gimbal_load(data_path);
  MU_ASSERT(calib != NULL);

  const real_t var[2] = {1.0, 1.0};
  calib_gimbal_factor_t factors[2000];
  int num_factors = 0;

  // Setup factors
  for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
    // Links
    joint_angle_t *joints = calib->joints[view_idx];
    calib_gimbal_add_param(calib, &joints[0], JOINT_PARAM);
    calib_gimbal_add_param(calib, &joints[1], JOINT_PARAM);
    calib_gimbal_add_param(calib, &joints[2], JOINT_PARAM);

    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      calib_gimbal_view_t *view = calib->views[view_idx][cam_idx];
      for (int i = 0; i < view->num_corners; i++) {
        const int tag_id = view->tag_ids[i];
        const int corner_idx = view->corner_indices[i];
        const real_t *p_FFi = view->object_points[i];
        const real_t *z = view->keypoints[i];

        calib_gimbal_factor_setup(&factors[num_factors],
                                  calib->fiducial,
                                  calib->links[0],
                                  calib->links[1],
                                  calib->links[2],
                                  &calib->joints[view_idx][0],
                                  &calib->joints[view_idx][1],
                                  &calib->joints[view_idx][2],
                                  calib->cam_exts[cam_idx],
                                  calib->cam_params[cam_idx],
                                  tag_id,
                                  corner_idx,
                                  p_FFi,
                                  z,
                                  var);
        num_factors++;
      }
    }
  }

  // Links
  for (int i = 0; i < 3; i++) {
    calib_gimbal_add_param(calib, &calib->links[i], EXTRINSICS_PARAM);
  }

  // Camera extrinsics
  for (int i = 0; i < calib->num_cams; i++) {
    calib_gimbal_add_param(calib, &calib->cam_exts[i], EXTRINSICS_PARAM);
  }

  // Fiducial
  calib_gimbal_add_param(calib, &calib->fiducial, EXTRINSICS_PARAM);

  // Camera parameters
  for (int i = 0; i < calib->num_cams; i++) {
    calib_gimbal_add_param(calib, &calib->cam_params[i], CAMERA_PARAM);
  }
  calib_gimbal_print_params(calib);

  // Evaluate factors
  int r_idx = 0;
  int r_size = num_factors * 2;
  real_t *r = CALLOC(real_t, r_size);

  for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      for (int i = 0; i < calib->views[view_idx][cam_idx]->num_corners; i++) {
        real_t *params[9] = {calib->fiducial->data,
                             calib->links[0]->data,
                             calib->links[1]->data,
                             calib->links[2]->data,
                             calib->joints[view_idx][0].angle,
                             calib->joints[view_idx][1].angle,
                             calib->joints[view_idx][2].angle,
                             calib->cam_exts[cam_idx]->data,
                             calib->cam_params[cam_idx]->data};

        real_t J_fiducial[2 * 6] = {0};
        real_t J_link0[2 * 6] = {0};
        real_t J_link1[2 * 6] = {0};
        real_t J_link2[2 * 6] = {0};
        real_t J_joint0[2 * 1] = {0};
        real_t J_joint1[2 * 1] = {0};
        real_t J_joint2[2 * 1] = {0};
        real_t J_cam_exts[2 * 6] = {0};
        real_t J_cam_params[2 * 8] = {0};
        real_t *jacs[9] = {J_fiducial,
                           J_link0,
                           J_link1,
                           J_link2,
                           J_joint0,
                           J_joint1,
                           J_joint2,
                           J_cam_exts,
                           J_cam_params};
        calib_gimbal_factor_eval(&factors[r_idx], params, &r[r_idx * 2], jacs);

        // for (int i = 0; i < 9; i++) {
        //   int *idx_i = calib->param_indices[i];
        //   int size_i = calib->param_sizes[i];
        //   const real_t *J_i = jacs[i];
        //   real_t *J_i_trans = CALLOC(real_t, 2 * size_i);
        //   mat_transpose(J_i, r_size, size_i, J_i_trans);

        //   for (int j = i; j < num_params; j++) {
        //     int *idx_j = calib->param_indices[j];
        //     int size_j = calib->param_sizes[j];
        //     const real_t *J_j = jacs[j];
        //     real_t *H_ij = MALLOC(real_t, size_i * size_j);
        //     dot(J_i_trans, size_i, r_size, J_j, r_size, size_j, H_ij);

        //     // Fill Hessian H
        //     // H_ij = J_i' * J_j
        //     // H_ji = H_ij'
        //     int stride = H_size;
        //     int rs = *idx_i;
        //     int cs = *idx_j;
        //     int re = rs + size_i;
        //     int ce = cs + size_j;
        //     if (i == j) {
        //       mat_block_set(H, stride, rs, re, cs, ce, H_ij);
        //     } else {
        //       real_t *H_ji = MALLOC(real_t, size_i * size_j);
        //       mat_transpose(H_ij, size_i, size_j, H_ji);
        //       mat_block_set(H, stride, rs, re, cs, ce, H_ij);
        //       mat_block_set(H, stride, cs, ce, rs, re, H_ji);
        //     }

        //     // Fill in the R.H.S of H dx = g
        //     // g = -J_i * r
        //     mat_scale(J_i_trans, H_size, r_size, -1);
        //     dot(J_i_trans, H_size, r_size, r, r_size, 1, g);
        //   }
        // }

        r_idx++;
      }
    }
  }

  // Clean up
  free(r);
  calib_gimbal_free(calib);

  return 0;
}

/******************************************************************************
 * TEST DATASET
 ******************************************************************************/

int test_assoc_pose_data() {
  const double threshold = 0.01;
  const char *matches_fpath = "./gnd_est_matches.csv";
  const char *gnd_data_path = "test_data/euroc/MH01_groundtruth.csv";
  const char *est_data_path = "test_data/euroc/MH01_estimate.csv";

  /* Load ground-truth poses */
  int nb_gnd_poses = 0;
  pose_t *gnd_poses = load_poses(gnd_data_path, &nb_gnd_poses);

  /* Load estimate poses */
  int nb_est_poses = 0;
  pose_t *est_poses = load_poses(est_data_path, &nb_est_poses);

  /* Associate data */
  size_t nb_matches = 0;
  int **matches = assoc_pose_data(gnd_poses,
                                  nb_gnd_poses,
                                  est_poses,
                                  nb_est_poses,
                                  threshold,
                                  &nb_matches);
  printf("Time Associated:\n");
  printf(" - [%s]\n", gnd_data_path);
  printf(" - [%s]\n", est_data_path);
  printf("threshold:  %.4f [s]\n", threshold);
  printf("nb_matches: %ld\n", nb_matches);

  /* Save matches to file */
  FILE *matches_csv = fopen(matches_fpath, "w");
  fprintf(matches_csv, "#gnd_idx,est_idx\n");
  for (size_t i = 0; i < nb_matches; i++) {
    uint64_t gnd_ts = gnd_poses[matches[i][0]].ts;
    uint64_t est_ts = est_poses[matches[i][1]].ts;
    double t_diff = fabs(ts2sec(gnd_ts - est_ts));
    if (t_diff > threshold) {
      printf("ERROR! Time difference > threshold!\n");
      printf("ground_truth_index: %d\n", matches[i][0]);
      printf("estimate_index: %d\n", matches[i][1]);
      break;
    }
    fprintf(matches_csv, "%d,%d\n", matches[i][0], matches[i][1]);
  }
  fclose(matches_csv);

  /* Clean up */
  for (size_t i = 0; i < nb_matches; i++) {
    free(matches[i]);
  }
  free(matches);
  free(gnd_poses);
  free(est_poses);

  return 0;
}

/******************************************************************************
 * TEST SIM
 ******************************************************************************/

// SIM FEATURES //////////////////////////////////////////////////////////////

int test_sim_features_load() {
  const char *csv_file = TEST_SIM_DATA "/features.csv";
  sim_features_t *features_data = sim_features_load(csv_file);
  MU_ASSERT(features_data->nb_features > 0);
  sim_features_free(features_data);
  return 0;
}

// SIM IMU DATA //////////////////////////////////////////////////////////////

int test_sim_imu_data_load() {
  const char *csv_file = TEST_SIM_DATA "/imu0/data.csv";
  sim_imu_data_t *imu_data = sim_imu_data_load(csv_file);
  sim_imu_data_free(imu_data);
  return 0;
}

// SIM CAMERA DATA ///////////////////////////////////////////////////////////

int test_sim_camera_frame_load() {
  const char *frame_csv = TEST_SIM_DATA "/cam0/data/100000000.csv";
  sim_camera_frame_t *frame_data = sim_camera_frame_load(frame_csv);

  MU_ASSERT(frame_data != NULL);
  MU_ASSERT(frame_data->ts == 100000000);
  MU_ASSERT(frame_data->feature_ids[0] == 1);

  sim_camera_frame_free(frame_data);

  return 0;
}

int test_sim_camera_data_load() {
  const char *dir_path = TEST_SIM_DATA "/cam0";
  sim_camera_data_t *cam_data = sim_camera_data_load(dir_path);
  sim_camera_data_free(cam_data);
  return 0;
}

/*******************************************************************************
 * TEST GUI
 ******************************************************************************/
#ifdef USE_GUI

// TEST OPENGL UTILS
// ///////////////////////////////////////////////////////////

int test_gl_zeros() {
  // clang-format off
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat expected[3*3] = {0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0};
  // clang-format on

  gl_zeros(A, 3, 3);
  gl_print_matrix("A", A, 3, 3);
  MU_ASSERT(gl_equals(A, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_ones() {
  // clang-format off
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat expected[3*3] = {1.0, 1.0, 1.0,
                           1.0, 1.0, 1.0,
                           1.0, 1.0, 1.0};
  // clang-format on

  gl_ones(A, 3, 3);
  gl_print_matrix("A", A, 3, 3);
  MU_ASSERT(gl_equals(A, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_eye() {
  /* Check 4x4 matrix */
  // clang-format off
  GLfloat A[4*4] = {0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0};
  GLfloat A_expected[4*4] = {1.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0};
  // clang-format on
  gl_eye(A, 4, 4);
  gl_print_matrix("A", A, 4, 4);
  MU_ASSERT(gl_equals(A, A_expected, 4, 4, 1e-8));

  /* Check 3x4 matrix */
  // clang-format off
  GLfloat B[3*4] = {0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0};
  GLfloat B_expected[3*4] = {1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0,
                             0.0, 0.0, 0.0};
  // clang-format on
  gl_eye(B, 3, 4);
  gl_print_matrix("B", B, 3, 4);
  MU_ASSERT(gl_equals(B, B_expected, 3, 4, 1e-8));

  return 0;
}

int test_gl_equals() {
  // clang-format off
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat B[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat C[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 10.0};
  // clang-format on

  /* Assert */
  MU_ASSERT(gl_equals(A, B, 3, 3, 1e-8) == 1);
  MU_ASSERT(gl_equals(A, C, 3, 3, 1e-8) == 0);

  return 0;
}

int test_gl_matf_set() {
  // clang-format off
  GLfloat A[3*4] = {0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0};
  // clang-format on

  gl_matf_set(A, 3, 4, 0, 1, 1.0);
  gl_matf_set(A, 3, 4, 1, 0, 2.0);
  gl_matf_set(A, 3, 4, 0, 2, 3.0);
  gl_matf_set(A, 3, 4, 2, 0, 4.0);
  gl_print_matrix("A", A, 3, 4);

  return 0;
}

int test_gl_matf_val() {
  // clang-format off
  GLfloat A[3*4] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0,
                    10.0, 11.0, 12.0};
  // clang-format on

  const float tol = 1e-4;
  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 0, 0) - 1.0) < tol);
  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 1, 0) - 2.0) < tol);
  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 2, 0) - 3.0) < tol);

  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 0, 1) - 4.0) < tol);
  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 1, 1) - 5.0) < tol);
  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 2, 1) - 6.0) < tol);

  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 0, 2) - 7.0) < tol);
  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 1, 2) - 8.0) < tol);
  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 2, 2) - 9.0) < tol);

  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 0, 3) - 10.0) < tol);
  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 1, 3) - 11.0) < tol);
  MU_ASSERT(fabs(gl_matf_val(A, 3, 4, 2, 3) - 12.0) < tol);

  return 0;
}

int test_gl_transpose() {
  /* Transpose a 3x3 matrix */
  // clang-format off
  GLfloat A[3*3] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0};
  // clang-format on
  GLfloat A_t[3 * 3] = {0};

  gl_transpose(A, 3, 3, A_t);
  gl_print_matrix("A", A, 3, 3);
  gl_print_matrix("A_t", A_t, 3, 3);

  /* Transpose a 3x4 matrix */
  // clang-format off
  GLfloat B[3*4] = {1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0,
                    10.0, 11.0, 12.0};
  // clang-format on
  GLfloat B_t[3 * 4] = {0};
  gl_transpose(B, 3, 4, B_t);
  gl_print_matrix("B", B, 3, 4);
  gl_print_matrix("B_t", B_t, 4, 3);

  return 0;
}

int test_gl_vec3_cross() {
  const GLfloat u[3] = {1.0f, 2.0f, 3.0f};
  const GLfloat v[3] = {4.0f, 5.0f, 6.0f};
  GLfloat z[3] = {0};
  gl_vec3f_cross(u, v, z);

  /* Assert */
  GLfloat expected[3] = {-3.0f, 6.0f, -3.0f};
  gl_print_vector("z", z, 3);
  gl_print_vector("expected", z, 3);
  MU_ASSERT(gl_equals(z, expected, 3, 1, 1e-8));

  return 0;
}

int test_gl_dot() {
  // clang-format off
  GLfloat A[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  GLfloat B[3*3] = {1.0, 4.0, 7.0,
                    2.0, 5.0, 8.0,
                    3.0, 6.0, 9.0};
  // clang-format on
  GLfloat C[3 * 3] = {0.0};
  gl_dot(A, 3, 3, B, 3, 3, C);

  /* Assert */
  // clang-format off
  GLfloat expected[3*3] = {30.0f, 66.0f, 102.0f,
                           36.0f, 81.0f, 126.0f,
                           42.0f, 96.0f, 150.0f};
  // clang-format on
  gl_print_matrix("C", C, 3, 3);
  gl_print_matrix("expected", expected, 3, 3);
  MU_ASSERT(gl_equals(C, expected, 3, 3, 1e-8));

  return 0;
}

int test_gl_norm() {
  const GLfloat x[3] = {1.0f, 2.0f, 3.0f};
  const GLfloat n = gl_norm(x, 3);

  /* Assert */
  const GLfloat expected = 3.741657f;
  MU_ASSERT(fabs(n - expected) < 1e-6);

  return 0;
}

int test_gl_normalize() {
  GLfloat x[3] = {1.0f, 2.0f, 3.0f};
  gl_normalize(x, 3);

  /* Assert */
  const GLfloat expected[3] = {0.26726f, 0.53452f, 0.80178f};
  MU_ASSERT(gl_equals(x, expected, 3, 1, 1e-5));

  return 0;
}

int test_gl_perspective() {
  const GLfloat fov = gl_deg2rad(60.0);
  const GLfloat window_width = 1000.0f;
  const GLfloat window_height = 1000.0f;
  const GLfloat ratio = window_width / window_height;
  const GLfloat near = 0.1f;
  const GLfloat far = 100.0f;

  GLfloat P[4 * 4] = {0};
  gl_perspective(fov, ratio, near, far, P);

  // clang-format off
  const GLfloat P_expected[4*4] = {1.886051, 0.000000, 0.000000, 0.000000,
                                   0.000000, 1.732051, 0.000000, 0.000000,
                                   0.000000, 0.000000, -1.002002, -1.000000,
                                   0.000000, 0.000000, -0.200200, 0.000000};
  // clang-format on
  printf("fov: %f\n", fov);
  printf("ratio: %f\n", ratio);
  printf("near: %f\n", near);
  printf("far: %f\n", far);
  printf("\n");
  gl_print_matrix("P", P, 4, 4);
  gl_print_matrix("P_expected", P_expected, 4, 4);
  MU_ASSERT(gl_equals(P, P_expected, 4, 4, 1e-4));

  return 0;
}

int test_gl_lookat() {
  const GLfloat yaw = -0.785398;
  const GLfloat pitch = 0.000000;
  const GLfloat radius = 10.000000;
  const GLfloat focal[3] = {0.000000, 0.000000, 0.000000};
  const GLfloat world_up[3] = {0.000000, 1.000000, 0.000000};

  GLfloat eye[3];
  eye[0] = focal[0] + radius * sin(yaw);
  eye[1] = focal[1] + radius * cos(pitch);
  eye[2] = focal[2] + radius * cos(yaw);

  GLfloat V[4 * 4] = {0};
  gl_lookat(eye, focal, world_up, V);

  // clang-format off
  const GLfloat V_expected[4*4] = {0.707107, 0.500000, -0.500000, 0.000000,
                                   -0.000000, 0.707107, 0.707107, 0.000000,
                                   0.707107, -0.500000, 0.500000, 0.000000,
                                   0.000000, 0.000000, -14.142136, 1.000000};
  // clang-format on
  /* gl_print_vector("eye", eye, 3); */
  /* gl_print_vector("focal", focal, 3); */
  /* gl_print_vector("world_up", world_up, 3); */
  /* printf("\n"); */
  /* gl_print_matrix("V", V, 4, 4); */
  /* gl_print_matrix("V_expected", V_expected, 4, 4); */
  MU_ASSERT(gl_equals(V, V_expected, 4, 4, 1e-4));

  return 0;
}

// TEST SHADER ///////////////////////////////////////////////////////////////

int test_gl_shader_compile() {
  /* SDL init */
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    printf("SDL_Init Error: %s/n", SDL_GetError());
    return -1;
  }

  /* Window */
  const char *title = "Hello World!";
  const int x = 100;
  const int y = 100;
  const int w = 640;
  const int h = 480;
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_Window *window = SDL_CreateWindow(title, x, y, w, h, SDL_WINDOW_OPENGL);
  if (window == NULL) {
    printf("SDL_CreateWindow Error: %s/n", SDL_GetError());
    return -1;
  }

  /* OpenGL context */
  SDL_GLContext context = SDL_GL_CreateContext(window);
  SDL_GL_SetSwapInterval(1);
  UNUSED(context);

  /* GLEW */
  GLenum err = glewInit();
  if (err != GLEW_OK) {
    FATAL("glewInit failed: %s", glewGetErrorString(err));
  }

  char *glcube_vs = file_read("./shaders/cube.vert");
  const GLuint vs = gl_shader_compile(glcube_vs, GL_VERTEX_SHADER);
  free(glcube_vs);
  MU_ASSERT(vs != GL_FALSE);

  char *glcube_fs = file_read("./shaders/cube.frag");
  const GLuint fs = gl_shader_compile(glcube_fs, GL_VERTEX_SHADER);
  free(glcube_fs);
  MU_ASSERT(fs != GL_FALSE);

  return 0;
}

int test_gl_shaders_link() {
  /* SDL init */
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    printf("SDL_Init Error: %s/n", SDL_GetError());
    return -1;
  }

  /* Window */
  const char *title = "Hello World!";
  const int x = 100;
  const int y = 100;
  const int w = 640;
  const int h = 480;
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_Window *window = SDL_CreateWindow(title, x, y, w, h, SDL_WINDOW_OPENGL);
  if (window == NULL) {
    printf("SDL_CreateWindow Error: %s/n", SDL_GetError());
    return -1;
  }

  /* OpenGL context */
  SDL_GLContext context = SDL_GL_CreateContext(window);
  SDL_GL_SetSwapInterval(1);
  UNUSED(context);

  /* GLEW */
  GLenum err = glewInit();
  if (err != GLEW_OK) {
    FATAL("glewInit failed: %s", glewGetErrorString(err));
  }

  /* Cube vertex shader */
  char *glcube_vs = file_read("./shaders/cube.vert");
  const GLuint vs = gl_shader_compile(glcube_vs, GL_VERTEX_SHADER);
  free(glcube_vs);
  MU_ASSERT(vs != GL_FALSE);

  /* Cube fragment shader */
  char *glcube_fs = file_read("./shaders/cube.frag");
  const GLuint fs = gl_shader_compile(glcube_fs, GL_FRAGMENT_SHADER);
  free(glcube_fs);
  MU_ASSERT(fs != GL_FALSE);

  /* Link shakders */
  const GLuint gs = GL_FALSE;
  const GLuint prog = gl_shaders_link(vs, fs, gs);
  MU_ASSERT(prog != GL_FALSE);

  return 0;
}

// TEST GL PROGRAM ///////////////////////////////////////////////////////////

int test_gl_prog_setup() {
  /* SDL init */
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    printf("SDL_Init Error: %s/n", SDL_GetError());
    return -1;
  }

  /* Window */
  const char *title = "Hello World!";
  const int x = 100;
  const int y = 100;
  const int w = 640;
  const int h = 480;
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_Window *window = SDL_CreateWindow(title, x, y, w, h, SDL_WINDOW_OPENGL);
  if (window == NULL) {
    printf("SDL_CreateWindow Error: %s/n", SDL_GetError());
    return -1;
  }

  /* OpenGL context */
  SDL_GLContext context = SDL_GL_CreateContext(window);
  SDL_GL_SetSwapInterval(1);
  UNUSED(context);

  /* GLEW */
  GLenum err = glewInit();
  if (err != GLEW_OK) {
    FATAL("glewInit failed: %s", glewGetErrorString(err));
  }

  /* Shader program */
  char *glcube_vs = file_read("./shaders/cube.vert");
  char *glcube_fs = file_read("./shaders/cube.frag");
  const GLuint program_id = gl_prog_setup(glcube_vs, glcube_fs, NULL);
  free(glcube_vs);
  free(glcube_fs);
  MU_ASSERT(program_id != GL_FALSE);

  return 0;
}

// TEST GL-CAMERA ////////////////////////////////////////////////////////////

int test_gl_camera_setup() {
  int window_width = 640;
  int window_height = 480;

  gl_camera_t camera;
  gl_camera_setup(&camera, &window_width, &window_height);

  const GLfloat focal_expected[3] = {0.0f, 0.0f, 0.0f};
  const GLfloat world_up_expected[3] = {0.0f, 1.0f, 0.0f};
  const GLfloat position_expected[3] = {0.0f, 0.0f, 0.0f};
  const GLfloat right_expected[3] = {-1.0f, 0.0f, 0.0f};
  const GLfloat up_expected[3] = {0.0f, 1.0f, 0.0f};
  const GLfloat front_expected[3] = {0.0f, 0.0f, -1.0f};
  const GLfloat yaw_expected = gl_deg2rad(0.0f);
  const GLfloat pitch_expected = gl_deg2rad(0.0f);
  const GLfloat fov_expected = gl_deg2rad(45.0f);
  const GLfloat near_expected = 0.1f;
  const GLfloat far_expected = 100.0f;

  MU_ASSERT(camera.window_width == &window_width);
  MU_ASSERT(camera.window_height == &window_height);

  MU_ASSERT(gl_equals(camera.focal, focal_expected, 3, 1, 1e-8) == 1);
  MU_ASSERT(gl_equals(camera.world_up, world_up_expected, 3, 1, 1e-8) == 1);
  MU_ASSERT(gl_equals(camera.position, position_expected, 3, 1, 1e-8) == 1);
  MU_ASSERT(gl_equals(camera.right, right_expected, 3, 1, 1e-8) == 1);
  MU_ASSERT(gl_equals(camera.up, up_expected, 3, 1, 1e-8) == 1);
  MU_ASSERT(gl_equals(camera.front, front_expected, 3, 1, 1e-8) == 1);
  MU_ASSERT(fabs(camera.yaw - yaw_expected) < 1e-8);
  MU_ASSERT(fabs(camera.pitch - pitch_expected) < 1e-8);

  MU_ASSERT(fabs(camera.fov - fov_expected) < 1e-8);
  MU_ASSERT(fabs(camera.near - near_expected) < 1e-8);
  MU_ASSERT(fabs(camera.far - far_expected) < 1e-8);

  return 0;
}

// TEST GUI //////////////////////////////////////////////////////////////////

int test_gui() {
  gui_t gui;
  gui.window_title = "Test";
  gui.window_width = 640;
  gui.window_height = 480;

  gui_setup(&gui);
  gui_loop(&gui);

  return 0;
}

// TEST IMSHOW ///////////////////////////////////////////////////////////////

int test_imshow() {
  imshow_t imshow;
  imshow.window_title = "Test";
  imshow_setup(&imshow, "test_data/images/awesomeface.png");

  /* int x = 100; */
  /* int y = 100; */
  /* int radius = 10; */
  /* SDL_Color color; */
  /* color.r = 0; */
  /* color.g = 0; */
  /* color.b = 0; */
  /* color.a = 255; */
  /* draw_circle(imshow.renderer, x, y, radius, color); */

  imshow_loop(&imshow);

  return 0;
}

#endif // USE_GUI

void test_suite() {
  // MACROS
  MU_ADD_TEST(test_median_value);
  MU_ADD_TEST(test_mean_value);

  // FILE SYSTEM
  MU_ADD_TEST(test_path_file_name);
  MU_ADD_TEST(test_path_file_ext);
  MU_ADD_TEST(test_path_dir_name);
  MU_ADD_TEST(test_path_join);
  MU_ADD_TEST(test_list_files);
  MU_ADD_TEST(test_list_files_free);
  MU_ADD_TEST(test_file_read);
  MU_ADD_TEST(test_skip_line);
  MU_ADD_TEST(test_file_rows);
  MU_ADD_TEST(test_file_copy);

  // DATA
  MU_ADD_TEST(test_string_malloc);
  MU_ADD_TEST(test_dsv_rows);
  MU_ADD_TEST(test_dsv_cols);
  MU_ADD_TEST(test_dsv_fields);
  MU_ADD_TEST(test_dsv_data);
  MU_ADD_TEST(test_dsv_free);

  // DATA-STRUCTURE
#ifdef USE_DATA_STRUCTURES
  MU_ADD_TEST(test_darray_new_and_destroy);
  MU_ADD_TEST(test_darray_push_pop);
  MU_ADD_TEST(test_darray_contains);
  MU_ADD_TEST(test_darray_copy);
  MU_ADD_TEST(test_darray_new_element);
  MU_ADD_TEST(test_darray_set_and_get);
  MU_ADD_TEST(test_darray_update);
  MU_ADD_TEST(test_darray_remove);
  MU_ADD_TEST(test_darray_expand_and_contract);
  MU_ADD_TEST(test_list_new_and_destroy);
  // MU_ADD_TEST(test_list_push_pop);
  MU_ADD_TEST(test_list_shift);
  MU_ADD_TEST(test_list_unshift);
  MU_ADD_TEST(test_list_remove);
  MU_ADD_TEST(test_list_remove_destroy);
  MU_ADD_TEST(test_mstack_new_and_destroy);
  MU_ADD_TEST(test_mstack_push);
  MU_ADD_TEST(test_mstack_pop);
  MU_ADD_TEST(test_queue_new_and_destroy);
  MU_ADD_TEST(test_queue_enqueue_dequeue);
  MU_ADD_TEST(test_hashmap_new_destroy);
  MU_ADD_TEST(test_hashmap_clear_destroy);
  MU_ADD_TEST(test_hashmap_get_set);
  MU_ADD_TEST(test_hashmap_delete);
  MU_ADD_TEST(test_hashmap_traverse);
#endif // USE_DATA_STRUCTURES

  // TIME
  MU_ADD_TEST(test_tic_toc);
  MU_ADD_TEST(test_mtoc);
  MU_ADD_TEST(test_time_now);

  // NETWORK
  MU_ADD_TEST(test_tcp_server_setup);
  MU_ADD_TEST(test_http_parse_request);
  MU_ADD_TEST(test_ws_hash);
  // MU_ADD_TEST(test_ws_server);

  // MATHS
  MU_ADD_TEST(test_min);
  MU_ADD_TEST(test_max);
  MU_ADD_TEST(test_randf);
  MU_ADD_TEST(test_deg2rad);
  MU_ADD_TEST(test_rad2deg);
  MU_ADD_TEST(test_fltcmp);
  MU_ADD_TEST(test_fltcmp2);
  MU_ADD_TEST(test_pythag);
  MU_ADD_TEST(test_lerp);
  MU_ADD_TEST(test_lerp3);
  MU_ADD_TEST(test_sinc);
  MU_ADD_TEST(test_mean);
  MU_ADD_TEST(test_median);
  MU_ADD_TEST(test_var);
  MU_ADD_TEST(test_stddev);

  // LINEAR ALGEBRA
  MU_ADD_TEST(test_eye);
  MU_ADD_TEST(test_ones);
  MU_ADD_TEST(test_zeros);
  MU_ADD_TEST(test_mat_set);
  MU_ADD_TEST(test_mat_val);
  MU_ADD_TEST(test_mat_copy);
  MU_ADD_TEST(test_mat_row_set);
  MU_ADD_TEST(test_mat_col_set);
  MU_ADD_TEST(test_mat_block_get);
  MU_ADD_TEST(test_mat_block_set);
  MU_ADD_TEST(test_mat_diag_get);
  MU_ADD_TEST(test_mat_diag_set);
  MU_ADD_TEST(test_mat_triu);
  MU_ADD_TEST(test_mat_tril);
  MU_ADD_TEST(test_mat_trace);
  MU_ADD_TEST(test_mat_transpose);
  MU_ADD_TEST(test_mat_add);
  MU_ADD_TEST(test_mat_sub);
  MU_ADD_TEST(test_mat_scale);
  MU_ADD_TEST(test_vec_add);
  MU_ADD_TEST(test_vec_sub);
  MU_ADD_TEST(test_dot);
  MU_ADD_TEST(test_hat);
  MU_ADD_TEST(test_check_jacobian);

  // SVD
  MU_ADD_TEST(test_svd);
  MU_ADD_TEST(test_svd_inv);

  // CHOL
  MU_ADD_TEST(test_chol);
  MU_ADD_TEST(test_chol_solve);

  // TRANSFORMS
  MU_ADD_TEST(test_tf_rot_set);
  MU_ADD_TEST(test_tf_trans_set);
  MU_ADD_TEST(test_tf_trans_get);
  MU_ADD_TEST(test_tf_rot_get);
  MU_ADD_TEST(test_tf_quat_get);
  MU_ADD_TEST(test_tf_inv);
  MU_ADD_TEST(test_tf_point);
  MU_ADD_TEST(test_tf_hpoint);
  MU_ADD_TEST(test_tf_perturb_rot);
  MU_ADD_TEST(test_tf_perturb_trans);
  MU_ADD_TEST(test_tf_chain);
  MU_ADD_TEST(test_euler321);
  MU_ADD_TEST(test_rot2quat);
  MU_ADD_TEST(test_quat2euler);
  MU_ADD_TEST(test_quat2rot);

  // LIE
  MU_ADD_TEST(test_lie_Exp_Log);

  // CV
  MU_ADD_TEST(test_image_setup);
  MU_ADD_TEST(test_image_load);
  MU_ADD_TEST(test_image_print_properties);
  MU_ADD_TEST(test_image_free);
  MU_ADD_TEST(test_linear_triangulation);
  MU_ADD_TEST(test_radtan4_distort);
  MU_ADD_TEST(test_radtan4_point_jacobian);
  MU_ADD_TEST(test_radtan4_params_jacobian);
  MU_ADD_TEST(test_equi4_distort);
  MU_ADD_TEST(test_equi4_point_jacobian);
  MU_ADD_TEST(test_equi4_params_jacobian);
  MU_ADD_TEST(test_pinhole_focal);
  MU_ADD_TEST(test_pinhole_K);
  MU_ADD_TEST(test_pinhole_projection_matrix);
  MU_ADD_TEST(test_pinhole_project);
  MU_ADD_TEST(test_pinhole_point_jacobian);
  MU_ADD_TEST(test_pinhole_params_jacobian);
  MU_ADD_TEST(test_pinhole_radtan4_project);
  MU_ADD_TEST(test_pinhole_radtan4_project_jacobian);
  MU_ADD_TEST(test_pinhole_radtan4_params_jacobian);
  MU_ADD_TEST(test_pinhole_equi4_project);
  MU_ADD_TEST(test_pinhole_equi4_project_jacobian);
  MU_ADD_TEST(test_pinhole_equi4_params_jacobian);

  // SENSOR FUSION
  MU_ADD_TEST(test_pose_setup);
  MU_ADD_TEST(test_imu_biases_setup);
  MU_ADD_TEST(test_feature_setup);
  MU_ADD_TEST(test_extrinsics_setup);
  MU_ADD_TEST(test_pose_factor_setup);
  MU_ADD_TEST(test_pose_factor_eval);
  MU_ADD_TEST(test_ba_factor_setup);
  MU_ADD_TEST(test_ba_factor_eval);
  MU_ADD_TEST(test_vision_factor_setup);
  MU_ADD_TEST(test_vision_factor_eval);
  MU_ADD_TEST(test_calib_gimbal_factor_setup);
  MU_ADD_TEST(test_calib_gimbal_factor_eval);
  MU_ADD_TEST(test_imu_buf_setup);
  MU_ADD_TEST(test_imu_buf_add);
  MU_ADD_TEST(test_imu_buf_clear);
  MU_ADD_TEST(test_imu_buf_copy);
  MU_ADD_TEST(test_imu_factor_propagate_step);
  MU_ADD_TEST(test_imu_factor_setup);
  MU_ADD_TEST(test_imu_factor_eval);
#ifdef USE_CERES
  MU_ADD_TEST(test_ceres_example);
#endif // USE_CERES
  MU_ADD_TEST(test_solver_setup);
  MU_ADD_TEST(test_solver_eval);
  MU_ADD_TEST(test_calib_gimbal_load);
  MU_ADD_TEST(test_calib_gimbal_solve);

  // DATASET
  // MU_ADD_TEST(test_assoc_pose_data);

  // SIM
  MU_ADD_TEST(test_sim_features_load);
  MU_ADD_TEST(test_sim_imu_data_load);
  MU_ADD_TEST(test_sim_camera_frame_load);
  MU_ADD_TEST(test_sim_camera_data_load);

  // GUI
#ifdef USE_GUI
  MU_ADD_TEST(test_gl_zeros);
  MU_ADD_TEST(test_gl_ones);
  MU_ADD_TEST(test_gl_eye);
  MU_ADD_TEST(test_gl_equals);
  MU_ADD_TEST(test_gl_matf_set);
  MU_ADD_TEST(test_gl_matf_val);
  MU_ADD_TEST(test_gl_transpose);
  MU_ADD_TEST(test_gl_vec3_cross);
  MU_ADD_TEST(test_gl_dot);
  MU_ADD_TEST(test_gl_norm);
  MU_ADD_TEST(test_gl_normalize);
  MU_ADD_TEST(test_gl_perspective);
  MU_ADD_TEST(test_gl_lookat);
  MU_ADD_TEST(test_gl_shader_compile);
  MU_ADD_TEST(test_gl_shaders_link);
  MU_ADD_TEST(test_gl_prog_setup);
  MU_ADD_TEST(test_gl_camera_setup);
  MU_ADD_TEST(test_gui);
  MU_ADD_TEST(test_imshow);
#endif // USE_GUI
}

MU_RUN_TESTS(test_suite)

#endif // PROTO_UNITTEST
