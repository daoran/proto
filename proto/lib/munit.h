#ifndef MUNIT_H
#define MUNIT_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>

/* GLOBAL VARIABLES */
static int nb_tests = 0;
static int nb_passed = 0;
static int nb_failed = 0;
static char *test_target_name = NULL;

/* MUNIT */
#define MU_LOG_DIR "/tmp"
#define MU_KEEP_LOGS 1
#define MU_ENABLE_TERM_COLORS 0

#if MU_ENABLE_TERM_COLORS == 1
#define MU_RED "\x1B[1;31m"
#define MU_GRN "\x1B[1;32m"
#define MU_WHT "\x1B[1;37m"
#define MU_NRM "\x1B[1;0m"
#else
#define MU_RED
#define MU_GRN
#define MU_WHT
#define MU_NRM
#endif

/**
 * Redirect stdout and stderr to file.
 * @param[in] output_path Output file path
 * @param[out] stdout_fd Standard out file descriptor
 * @param[out] stderr_fd Standard error file descriptor
 * @param[out] output_fd Output file descriptor
 */
int streams_redirect(const char *output_path,
                     int *stdout_fd,
                     int *stderr_fd,
                     int *output_fd) {
  /* Obtain stdout and stderr file descriptors */
  *stdout_fd = dup(STDOUT_FILENO);
  *stderr_fd = dup(STDERR_FILENO);

  /* Open stdout log file */
  *output_fd = open(output_path, O_RDWR | O_CREAT, 0600);
  if (*output_fd == -1) {
    perror("opening output.log");
    return -1;
  }

  /* Redirect stdout */
  if (dup2(*output_fd, STDOUT_FILENO) == -1) {
    perror("cannot redirect stdout");
    return -1;
  }

  /* Redirect stderr */
  if (dup2(*output_fd, STDERR_FILENO) == -1) {
    perror("cannot redirect stderr");
    return -1;
  }

  return 0;
}

/**
 * Restore stdout and stderr
 * @param[in] stdout_fd Standard output file descriptor
 * @param[in] stderr_fd Standard error file descriptor
 * @param[in] output_fd Output file descriptor
 */
void streams_restore(const int stdout_fd,
                     const int stderr_fd,
                     const int output_fd) {
  /* Flush stdout, stderr and close output file */
  fflush(stdout);
  fflush(stderr);
  close(output_fd);

  /* Restore stdout and stderr */
  dup2(stdout_fd, STDOUT_FILENO);
  dup2(stderr_fd, STDERR_FILENO);
}

/**
 * Print test log
 * @param[in] log_path Path to test log
 */
void mu_print_log(const char *log_path) {
  /* Open log file */
  printf("TEST LOG [%s]\n", log_path);
  FILE *log_file = fopen(log_path, "rb");
  if (log_file == NULL) {
    return;
  }

  /* Get log file length */
  fseek(log_file, 0, SEEK_END);
  size_t log_length = ftell(log_file);
  fseek(log_file, 0, SEEK_SET);

  /* Print log and close log file */
  char buf[9046] = {0};
  fread(buf, 1, log_length, log_file);
  printf("%s\n\n", buf);
  fflush(stdout);
  fclose(log_file);
}

/**
 * Mark test passed
 * @param[in] test_name Test name
 * @param[in] log_path Path to log file
 * @param[in] keep_log Flag to keep log or not
 */
void mu_test_passed(const char *test_name,
                    const char *log_path,
                    const int keep_log) {
  printf("-> [%s] ", test_name);
  printf(MU_GRN "OK!\n" MU_NRM);
  fflush(stdout);
  nb_passed++;
  if (keep_log == 0) {
    remove(log_path);
  }
}

/**
 * Mark test failed
 * @param[in] test_name Test name
 * @param[in] log_path Path to log file
 */
void mu_test_failed(const char *test_name, const char *log_path) {
  printf("-> [%s] ", test_name);
  printf(MU_RED "FAILED!\n" MU_NRM);
  fflush(stdout);
  mu_print_log(log_path);
  nb_failed++;
}

/**
 * Print test stats
 */
void mu_print_stats() {
  printf(MU_WHT "%d tests" MU_NRM ", ", nb_tests);
  printf(MU_GRN "%d passed" MU_NRM ", ", nb_passed);
  printf(MU_RED "%d failed\n" MU_NRM, nb_failed);
  printf("\n");
}

/**
 * Run unittests
 * @param[in] test_name Test name
 * @param[in] test_ptr Pointer to unittest
 * @param[in] keep_logs Flag to keep log or not
 */
void mu_run_test(const char *test_name,
                 int (*test_ptr)(),
                 const int keep_logs) {
  /* Check if test target is set and current test is test target */
  if (test_target_name != NULL && strcmp(test_target_name, test_name) != 0) {
    return;
  }

  /* Redirect stdout and stderr to file */
  char log_path[1024] = {0};
  sprintf(log_path, "%s/mu_%s.log", MU_LOG_DIR, test_name);

  int stdout_fd = 0;
  int stderr_fd = 0;
  int log_fd = 0;
  if (streams_redirect(log_path, &stdout_fd, &stderr_fd, &log_fd) == -1) {
    printf("Failed to redirect streams!\n");
    exit(-1);
  }

  /* Run test */
  int test_retval = (*test_ptr)();

  /* Restore stdout and stderr */
  streams_restore(stdout_fd, stderr_fd, log_fd);

  /* Keep track of test results */
  if (test_retval == 0) {
    mu_test_passed(test_name, log_path, MU_KEEP_LOGS);
  } else {
    mu_test_failed(test_name, log_path);
  }
  nb_tests++;
}

/**
 * Run python script
 * @param[in] script_path Path to python3 script
 */
int mu_run_python(const char *script_path) {
  char cmd[1024] = {0};
  sprintf(cmd, "python3 %s", script_path);
  if (system(cmd) != 0) {
    printf("Python3 script [%s] failed !", script_path);
    return -1;
  }
  return 0;
}

/**
 * Unit-test assert
 * @param[in] TEST Test condition
 */
#define MU_ASSERT(TEST)                                                        \
  do {                                                                         \
    if ((TEST) == 0) {                                                         \
      printf(MU_RED "ERROR!" MU_NRM " [%s:%d] %s\n",                           \
             __func__,                                                         \
             __LINE__,                                                         \
             #TEST);                                                           \
      return -1;                                                               \
    }                                                                          \
  } while (0)

/**
 * Add unittest
 * @param[in] TEST Test function
 */
#define MU_ADD_TEST(TEST) mu_run_test(#TEST, TEST, MU_KEEP_LOGS);

/**
 * Run all unit-tests
 * @param[in] TEST_SUITE Test suite
 */
#define MU_RUN_TESTS(TEST_SUITE)                                               \
  int main(int argc, char *argv[]) {                                           \
    if (argc == 3 && strcmp(argv[1], "--target") == 0) {                       \
      test_target_name = argv[2];                                              \
      printf("TEST TARGET [%s]\n", test_target_name);                          \
    }                                                                          \
                                                                               \
    TEST_SUITE();                                                              \
    mu_print_stats();                                                          \
    return (nb_failed) ? -1 : 0;                                               \
  }

#endif // MUNIT_H
