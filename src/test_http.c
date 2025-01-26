#include "xyz_http.h"

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
void run_test(const char *test_name, int (*test_ptr)(void)) {
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
  TEST_ASSERT(strcmp(hash, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") == 0);
  free(hash);
  return 0;
}

int test_ws_server(void) {
  ws_server();
  return 0;
}

int main(int argc, char *argv[]) {
  TEST(test_http_parse_request);
  TEST(test_ws_hash);
  // TEST(test_ws_server);

  return (nb_failed) ? -1 : 0;
}
