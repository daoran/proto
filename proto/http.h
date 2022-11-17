#ifndef HTTP_H
#define HTTP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

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

#endif // HTTP_H

//////////////////////////////////////////////////////////////////////////////
//                             IMPLEMENTATION                               //
//////////////////////////////////////////////////////////////////////////////

#ifdef HTTP_IMPLEMENTATION

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
    HTTP_ERROR("Listen failed...");
    return -1;
  }

  // Accept incomming connections
  struct sockaddr_in sockaddr;
  socklen_t len = sizeof(sockaddr);
  int connfd = accept(server.sockfd, (struct sockaddr *) &sockaddr, &len);
  if (connfd < 0) {
    HTTP_ERROR("Server acccept failed!");
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

#endif // HTTP_IMPLEMENTATION

//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

#ifdef HTTP_UNITTEST

#include <stdio.h>

// UNITESTS GLOBAL VARIABLES
static int nb_tests = 0;
static int nb_passed = 0;
static int nb_failed = 0;

#define ENABLE_TERM_COLORS 0
#if ENABLE_TERM_COLORS == 1
#define TERM_RED "\x1B[1;31m"
#define TERM_GRN "\x1B[1;32m"
#define TERM_WHT "\x1B[1;37m"
#define TERM_NRM "\x1B[1;0m"
#else
#define TERM_RED
#define TERM_GRN
#define TERM_WHT
#define TERM_NRM
#endif

/**
 * Run unittests
 * @param[in] test_name Test name
 * @param[in] test_ptr Pointer to unittest
 */
void run_test(const char *test_name, int (*test_ptr)()) {
  if ((*test_ptr)() == 0) {
    printf("-> [%s] " TERM_GRN "OK!\n" TERM_NRM, test_name);
    fflush(stdout);
    nb_passed++;
  } else {
    printf(TERM_RED "FAILED!\n" TERM_NRM);
    fflush(stdout);
    nb_failed++;
  }
  nb_tests++;
}

/**
 * Add unittest
 * @param[in] TEST Test function
 */
#define TEST(TEST_FN) run_test(#TEST_FN, TEST_FN);

/**
 * Unit-test assert
 * @param[in] TEST Test condition
 */
#define TEST_ASSERT(TEST)                                                      \
  do {                                                                         \
    if ((TEST) == 0) {                                                         \
      printf(TERM_RED "ERROR!" TERM_NRM " [%s:%d] %s FAILED!\n",               \
             __func__,                                                         \
             __LINE__,                                                         \
             #TEST);                                                           \
      return -1;                                                               \
    }                                                                          \
  } while (0)

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
  TEST_ASSERT(strcmp(hash, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") == 0);
  free(hash);
  return 0;
}

int test_ws_server() {
  ws_server();
  return 0;
}

int main(int argc, char *argv[]) {
  TEST(test_http_parse_request);
  TEST(test_ws_hash);
  // TEST(test_ws_server);

  return (nb_failed) ? -1 : 0;
}

#endif // HTTP_UNITTESTS
