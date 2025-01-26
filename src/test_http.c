#include "munit.h"
#include "xyz_http.h"

int test_http_msg_setup(void) {
  http_msg_t msg;
  http_msg_setup(&msg);

  return 0;
}

int test_http_msg_print(void) {
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

int test_http_parse_request(void) {
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

int test_ws_hash(void) {
  const char *key = "dGhlIHNhbXBsZSBub25jZQ==";
  char *hash = ws_hash(key);
  MU_ASSERT(strcmp(hash, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") == 0);
  free(hash);
  return 0;
}

int test_ws_server(void) {
  ws_server();
  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_http_parse_request);
  MU_ADD_TEST(test_ws_hash);
}
MU_RUN_TESTS(test_suite)
