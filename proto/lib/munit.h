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

#define MU_LOG_ERROR(TEST, FUNC, LINENO)                                       \
  printf(MU_RED "ERROR!" MU_NRM " [%s:%d] %s\n", FUNC, LINENO, #TEST);

#define MU_CHECK(TEST)                                                         \
  do {                                                                         \
    if ((TEST) == 0) {                                                         \
      MU_LOG_ERROR(TEST, __func__, __LINE__);                                  \
      return -1;                                                               \
    }                                                                          \
  } while (0)

#define MU_PRINT_LOG(LOG_PATH)                                                 \
  /* Open log file */                                                          \
  printf("TEST LOG [%s]\n", LOG_PATH);                                         \
  FILE *log_file = fopen(LOG_PATH, "rb");                                      \
  if (log_file == NULL) {                                                      \
    return;                                                                    \
  }                                                                            \
                                                                               \
  /* Get log file length */                                                    \
  fseek(log_file, 0, SEEK_END);                                                \
  size_t log_length = ftell(log_file);                                         \
  fseek(log_file, 0, SEEK_SET);                                                \
                                                                               \
  char buf[9046] = {0};                                                        \
  fread(buf, 1, log_length, log_file);                                         \
  printf("%s\n\n", buf);                                                       \
  fflush(stdout);                                                              \
  fclose(log_file);

#define MU_TEST_PASSED(TEST, LOG_PATH)                                         \
  printf("-> [%s] ", #TEST);                                                   \
  printf(MU_GRN "OK!\n" MU_NRM);                                               \
  fflush(stdout);                                                              \
  remove(LOG_PATH);                                                            \
  nb_passed++;

#define MU_TEST_FAILED(TEST, LOG_PATH)                                         \
  printf("-> [%s] ", #TEST);                                                   \
  printf(MU_RED "FAILED!\n" MU_NRM);                                           \
  fflush(stdout);                                                              \
  MU_PRINT_LOG(LOG_PATH);                                                      \
  nb_failed++;

#define MU_PRINT_STATS()                                                       \
  printf(MU_WHT "%d tests" MU_NRM ", ", nb_tests);                             \
  printf(MU_GRN "%d passed" MU_NRM ", ", nb_passed);                           \
  printf(MU_RED "%d failed\n" MU_NRM, nb_failed);                              \
  printf("\n");

#define MU_ADD_TEST(TEST)                                                      \
  do {                                                                         \
    /* Check if test target is set and current test is test target */          \
    if (test_target_name != NULL && strcmp(test_target_name, #TEST) != 0) {    \
      continue;                                                                \
    }                                                                          \
                                                                               \
    /* Redirect stdout and stderr to file */                                   \
    char *log_path = MU_LOG_DIR "/mu_" #TEST ".log";                           \
    int stdout_fd = 0;                                                         \
    int stderr_fd = 0;                                                         \
    int log_fd = 0;                                                            \
    if (streams_redirect(log_path, &stdout_fd, &stderr_fd, &log_fd) == -1) {   \
      printf("Failed to redirect streams!\n");                                 \
      exit(-1);                                                                \
    }                                                                          \
                                                                               \
    /* Run test */                                                             \
    int test_retval = TEST();                                                  \
                                                                               \
    /* Restore stdout and stderr */                                            \
    streams_restore(stdout_fd, stderr_fd, log_fd);                             \
                                                                               \
    /* Keep track of test results */                                           \
    if (test_retval == -1) {                                                   \
      MU_TEST_FAILED(TEST, log_path);                                          \
    } else {                                                                   \
      MU_TEST_PASSED(TEST, log_path);                                          \
    }                                                                          \
    nb_tests++;                                                                \
  } while (0)

#define MU_RUN_TESTS(TEST_SUITE)                                               \
  int main(int argc, char *argv[]) {                                           \
    if (argc == 3 && strcmp(argv[1], "--target") == 0) {                       \
      test_target_name = argv[2];                                              \
      printf("TEST TARGET [%s]\n", test_target_name);                          \
    }                                                                          \
                                                                               \
    TEST_SUITE();                                                              \
    MU_PRINT_STATS();                                                          \
    return (nb_failed) ? -1 : 0;                                               \
  }

#define RUN_PYTHON_SCRIPT(A)                                                   \
  if (system("python3 " A) != 0) {                                             \
    printf("Python script [%s] failed !", A);                                  \
    return -1;                                                                 \
  }

/**
 * Redirect stdout and stderr to file.
 * @param[in] output_path Output file path
 * @param[out] stdout_fd Standard output file descriptor (for restoring later)
 * @param[out] stderr_fd Standard error file descriptor (for restoring later)
 * @param[out] output_fd Output file descriptor (for closing later)
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
 * @param[int] stdout_fd Standard output file descriptor
 * @param[int] stderr_fd Standard error file descriptor
 * @param[int] output_fd Output file descriptor
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

#endif // MUNIT_H
