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

/* MACROS */
#define ENABLE_TERM_COLORS 0
#define KNRM "\x1B[1;0m"
#define KRED "\x1B[1;31m"
#define KGRN "\x1B[1;32m"
#define KYEL "\x1B[1;33m"
#define KBLU "\x1B[1;34m"
#define KMAG "\x1B[1;35m"
#define KCYN "\x1B[1;36m"
#define KWHT "\x1B[1;37m"

/* MUNIT */
#if ENABLE_TERM_COLORS
#define MU_LOG_ERROR(TEST, FUNC, LINENO)                                       \
  printf("%sERROR!%s [%s:%d] %s %sFAILED!%s\n",                                \
         KRED,                                                                 \
         KNRM,                                                                 \
         FUNC,                                                                 \
         LINENO,                                                               \
         #TEST,                                                                \
         KRED,                                                                 \
         KNRM);

#define MU_TEST_PASSED(TEST)                                                   \
  printf("%s-> [%s] %s", KBLU, #TEST, KNRM);                                   \
  printf("%sOK!%s\n", KGRN, KNRM);                                             \
  fflush(stdout);                                                              \
  nb_passed++;

#define MU_TEST_FAILED(TEST)                                                   \
  printf("%s-> [%s] %s", KBLU, #TEST, KNRM);                                   \
  printf("%sFAILED!%s\n", KRED, KNRM);                                         \
  fflush(stdout);                                                              \
  nb_failed++;

#define MU_PRINT_STATS()                                                       \
  printf("%s%d tests%s, ", KWHT, nb_tests, KNRM);                              \
  printf("%s%d passed%s, ", KGRN, nb_tests, KNRM);                             \
  printf("%s%d failed%s\n\n", KRED, nb_failed, KNRM);

#else
#define MU_LOG_ERROR(TEST, FUNC, LINENO)                                       \
  printf("ERROR! [%s:%d] %s FAILED!\n", FUNC, LINENO, #TEST);

#define MU_TEST_PASSED(TEST)                                                   \
  printf("-> [%s] ", #TEST);                                                   \
  printf("OK!\n");                                                             \
  fflush(stdout);                                                              \
  nb_passed++;

#define MU_TEST_FAILED(TEST)                                                   \
  printf("-> [%s] ", #TEST);                                                   \
  printf("FAILED!\n");                                                         \
  fflush(stdout);                                                              \
  nb_failed++;

#define MU_PRINT_STATS()                                                       \
  printf("%d tests, ", nb_tests);                                              \
  printf("%d passed, ", nb_passed);                                            \
  printf("%d failed\n", nb_failed);                                            \
  printf("\n");

#endif

#define MU_CHECK(TEST)                                                         \
  do {                                                                         \
    if ((TEST) == 0) {                                                         \
      MU_LOG_ERROR(TEST, __func__, __LINE__);                                  \
      return -1;                                                               \
    }                                                                          \
  } while (0)

#define MU_ADD_TEST(TEST)                                                      \
  do {                                                                         \
    if (test_target_name != NULL && strcmp(test_target_name, #TEST) != 0) {    \
      continue;                                                                \
    }                                                                          \
                                                                               \
    const char *output_path = "/tmp/output.log";                               \
    int stdout_fd = 0;                                                         \
    int stderr_fd = 0;                                                         \
    int output_fd = 0;                                                         \
    if (streams_redirect(output_path, &stdout_fd, &stderr_fd, &output_fd) ==   \
        -1) {                                                                  \
      printf("Failed to redirect streams!\n");                                 \
      exit(-1);                                                                \
    }                                                                          \
                                                                               \
    int retval = TEST();                                                       \
                                                                               \
    streams_restore(stdout_fd, stderr_fd, output_fd);                          \
                                                                               \
    if (retval == -1) {                                                        \
      MU_TEST_FAILED(TEST);                                                    \
    } else {                                                                   \
      MU_TEST_PASSED(TEST);                                                    \
    }                                                                          \
    nb_tests++;                                                                \
  } while (0)

#define MU_REPORT()                                                            \
  do {                                                                         \
    MU_PRINT_STATS();                                                          \
    if (nb_failed != 0) {                                                      \
      return -1;                                                               \
    } else {                                                                   \
      return 0;                                                                \
    }                                                                          \
  } while (0)

#define MU_RUN_TESTS(TEST_SUITE)                                               \
  int main(int argc, char *argv[]) {                                           \
    if (argc == 3 && strcmp(argv[1], "--target") == 0) {                       \
      test_target_name = argv[2];                                              \
      printf("TEST TARGET [%s]\n", test_target_name);                          \
    }                                                                          \
                                                                               \
    TEST_SUITE();                                                              \
    MU_REPORT();                                                               \
    return 0;                                                                  \
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
  *output_fd = open(output_path, O_RDWR | O_CREAT | O_APPEND, 0600);
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
