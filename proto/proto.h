#ifndef PROTO_H
#define PROTO_H

#define _DEFAULT_SOURCE 1

/** PROTO SETTINGS **/
#define PRECISION 2
#define MAX_LINE_LENGTH 9046
#define USE_CBLAS
#define USE_LAPACK
#define USE_CERES
#define USE_STB_IMAGE
// #define USE_GUI

#ifndef WARN_UNUSED
#define WARN_UNUSED __attribute__((warn_unused_result))
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>
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

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

#define SDL_DISABLE_IMMINTRIN_H 1
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

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
int file_exists(const char *fp);
int file_rows(const char *fp);
int file_copy(const char *src, const char *dest);

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

// LIST ////////////////////////////////////////////////////////////////////////

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

// STACK ///////////////////////////////////////////////////////////////////////

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

// QUEUE ///////////////////////////////////////////////////////////////////////

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

// HTTP ////////////////////////////////////////////////////////////////////////

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
  void *payload_data;
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
void ws_send(int connfd, char *msg);
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
#define SIGN2(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

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

int check_jacobian(const char *jac_name,
                   const real_t *fdiff,
                   const real_t *jac,
                   const size_t m,
                   const size_t n,
                   const real_t tol,
                   const int verbose);

/******************************************************************************
 * SVD
 ******************************************************************************/

#ifdef USE_LAPACK
void lapack_svd(
    real_t *A, const int m, const int n, real_t **s, real_t **U, real_t **V_t);
void lapack_svd_inverse(real_t *A, const int m, const int n, real_t *A_inv);
#endif

/******************************************************************************
 * CHOL
 ******************************************************************************/

void chol(const real_t *A, const size_t n, real_t *L);
void chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n);

#ifdef USE_LAPACK
void lapack_chol(const real_t *A, const size_t m, real_t *L);
void lapack_chol_solve(const real_t *A,
                       const real_t *b,
                       real_t *x,
                       const size_t n);
#endif

/******************************************************************************
 * TRANSFORMS
 ******************************************************************************/

void tf(const real_t params[7], real_t T[4 * 4]);
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

// GEOMETRY ////////////////////////////////////////////////////////////////////

void linear_triangulation(const real_t P_i[3 * 4],
                          const real_t P_j[3 * 4],
                          const real_t z_i[2],
                          const real_t z_j[2],
                          real_t p[3]);

// RADTAN //////////////////////////////////////////////////////////////////////

void radtan4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]);
void radtan4_point_jacobian(const real_t params[4],
                            const real_t p[2],
                            real_t J_point[2 * 2]);
void radtan4_params_jacobian(const real_t params[4],
                             const real_t p[2],
                             real_t J_param[2 * 4]);

// EQUI ////////////////////////////////////////////////////////////////////////

void equi4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]);
void equi4_point_jacobian(const real_t params[4],
                          const real_t p[2],
                          real_t J_point[2 * 2]);
void equi4_params_jacobian(const real_t params[4],
                           const real_t p[2],
                           real_t J_param[2 * 4]);

// PINHOLE /////////////////////////////////////////////////////////////////////

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

// PINHOLE-RADTAN4 /////////////////////////////////////////////////////////////

void pinhole_radtan4_project(const real_t params[8],
                             const real_t p_C[3],
                             real_t x[2]);
void pinhole_radtan4_project_jacobian(const real_t params[8],
                                      const real_t p_C[3],
                                      real_t J[2 * 3]);
void pinhole_radtan4_params_jacobian(const real_t params[8],
                                     const real_t p_C[3],
                                     real_t J[2 * 8]);

// PINHOLE-EQUI4 ///////////////////////////////////////////////////////////////

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
#define CAM_PARAM 5

// POSE ////////////////////////////////////////////////////////////////////////

typedef struct pose_t {
  timestamp_t ts;
  real_t pos[3];
  real_t quat[4];
} pose_t;

void pose_setup(pose_t *pose, const timestamp_t ts, const real_t *param);
void pose_print(const char *prefix, const pose_t *pose);

// VELOCITY ////////////////////////////////////////////////////////////////////

typedef struct velocity_t {
  timestamp_t ts;
  real_t v[3];
} velocity_t;

void velocity_setup(velocity_t *vel, const timestamp_t ts, const real_t v[3]);

// IMU BIASES /////////////////////////////////////////////////////////////////

typedef struct imu_biases_t {
  timestamp_t ts;
  real_t ba[3];
  real_t bg[3];
} imu_biases_t;

void imu_biases_setup(imu_biases_t *sb,
                      const timestamp_t ts,
                      const real_t ba[3],
                      const real_t bg[3]);

// FEATURE /////////////////////////////////////////////////////////////////////

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

// EXTRINSICS //////////////////////////////////////////////////////////////////

typedef struct extrinsics_t {
  real_t pos[3];
  real_t quat[4];
} extrinsics_t;

void extrinsics_setup(extrinsics_t *extrinsics, const real_t *param);
void extrinsics_print(const char *prefix, const extrinsics_t *exts);

// CAMERA PARAMS ///////////////////////////////////////////////////////////////

typedef struct camera_params_t {
  int cam_idx;
  int resolution[2];
  char proj_model[20];
  char dist_model[20];
  real_t data[8];
} camera_params_t;

void camera_params_setup(camera_params_t *camera,
                         const int cam_idx,
                         const int cam_res[2],
                         const char *proj_model,
                         const char *dist_model,
                         const real_t *data);
void camera_params_print(const camera_params_t *camera);

// POSE FACTOR /////////////////////////////////////////////////////////////////

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

int check_factor_quaternion_jacobian(const void *factor,
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
  int nb_params;

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
int pose_factor_ceres_eval(void *factor,
                           double **params,
                           double *residuals,
                           double **jacobians);

// BA FACTOR ///////////////////////////////////////////////////////////////////

typedef struct ba_factor_t {
  const pose_t *pose;
  const camera_params_t *camera;
  const feature_t *feature;
  int nb_params;

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
int ba_factor_ceres_eval(void *factor,
                         double **params,
                         double *residuals,
                         double **jacobians);

// CAMERA FACTOR ///////////////////////////////////////////////////////////////

typedef struct cam_factor_t {
  const pose_t *pose;
  const extrinsics_t *extrinsics;
  const camera_params_t *camera;
  const feature_t *feature;
  int nb_params;

  real_t covar[2 * 2];
  real_t sqrt_info[2 * 2];
  real_t z[2];
} cam_factor_t;

void cam_factor_setup(cam_factor_t *factor,
                      const pose_t *pose,
                      const extrinsics_t *extrinsics,
                      const feature_t *feature,
                      const camera_params_t *camera,
                      const real_t z[2],
                      const real_t var[2]);
int cam_factor_eval(cam_factor_t *factor,
                    real_t **params,
                    real_t *residuals,
                    real_t **jacobians);
int cam_factor_ceres_eval(void *factor,
                          double **params,
                          double *residuals,
                          double **jacobians);

// IMU FACTOR //////////////////////////////////////////////////////////////////

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
  int nb_params;

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
  cam_factor_t *cam_factors;
  int nb_cam_factors;

  imu_factor_t *imu_factors;
  int nb_imu_factors;

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
void solver_optimize(solver_t *solver);

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

// SIM FEATURES ////////////////////////////////////////////////////////////////

typedef struct sim_features_t {
  real_t **features;
  int nb_features;
} sim_features_t;

sim_features_t *sim_features_load(const char *csv_path);
void sim_features_free(sim_features_t *features_data);

// SIM IMU DATA ////////////////////////////////////////////////////////////////

typedef struct sim_imu_data_t {
  real_t **data;
  int nb_measurements;
} sim_imu_data_t;

sim_imu_data_t *sim_imu_data_load(const char *csv_path);
void sim_imu_data_free(sim_imu_data_t *imu_data);

// SIM CAM DATA ////////////////////////////////////////////////////////////////

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

// OPENGL UTILS ////////////////////////////////////////////////////////////////

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

// SHADER //////////////////////////////////////////////////////////////////////

GLuint shader_compile(const char *shader_src, const int type);
GLuint shaders_link(const GLuint vertex_shader,
                    const GLuint fragment_shader,
                    const GLuint geometry_shader);

// GL PROGRAM //////////////////////////////////////////////////////////////////

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

// GL-CAMERA ///////////////////////////////////////////////////////////////////

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

// GL-PRIMITIVES ///////////////////////////////////////////////////////////////

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

// GUI /////////////////////////////////////////////////////////////////////////

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

// IMSHOW //////////////////////////////////////////////////////////////////////

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
