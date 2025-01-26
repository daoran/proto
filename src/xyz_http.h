#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <inttypes.h>

#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>

#include <openssl/sha.h>

/** Macro that adds the ability to switch between C / C++ style mallocs */
#ifdef __cplusplus

#ifndef MALLOC
#define MALLOC(TYPE, N) (TYPE *) malloc(sizeof(TYPE) * (N));
#endif // MALLOC

#ifndef CALLOC
#define CALLOC(TYPE, N) (TYPE *) calloc((N), sizeof(TYPE));
#endif // CALLOC

#else

#ifndef MALLOC
#define MALLOC(TYPE, N) malloc(sizeof(TYPE) * (N));
#endif // MALLOC

#ifndef CALLOC
#define CALLOC(TYPE, N) calloc((N), sizeof(TYPE));
#endif // CALLOC
#endif

#ifndef status_t
#define status_t __attribute__((warn_unused_result)) int
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
#ifndef HTTP_INFO
#define HTTP_INFO(...)                                                         \
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
#ifndef HTTP_ERROR
#define HTTP_ERROR(...)                                                        \
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
#ifndef HTTP_WARN
#define HTTP_WARN(...)                                                         \
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
 * Free memory
 */
#ifndef FREE_MEM
#define FREE_MEM(TARGET, FREE_FUNC)                                            \
  if (TARGET) {                                                                \
    FREE_FUNC((void *) TARGET);                                                \
  }
#endif

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

typedef struct tcp_server_t {
  int port;
  int sockfd;
  int conn;
  void *(*conn_handler)(void *);
} tcp_server_t;

typedef struct tcp_client_t {
  char server_ip[1024];
  int server_port;
  int sockfd;
  int (*loop_cb)(struct tcp_client_t *);
} tcp_client_t;

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

size_t string_copy(char *dst, const char *src);
void string_cat(char *dst, const char *src);
char *string_malloc(const char *s);

char *base64_encode(const uint8_t *data, size_t in_len, size_t *out_len);
uint8_t *base64_decode(const char *data, size_t in_len, size_t *out_len);

status_t ip_port_info(const int sockfd, char *ip, int *port);
status_t tcp_server_setup(tcp_server_t *server, const int port);
status_t tcp_server_loop(tcp_server_t *server);
status_t tcp_client_setup(tcp_client_t *client,
                        const char *server_ip,
                        const int server_port);
status_t tcp_client_loop(tcp_client_t *client);

void http_msg_setup(http_msg_t *msg);
void http_msg_free(http_msg_t *msg);
void http_msg_print(http_msg_t *msg);
int http_parse_request(char *msg_str, http_msg_t *msg);

ws_frame_t *ws_frame_malloc(void);
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
int ws_server(void);
