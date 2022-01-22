#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <time.h>


#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

static float randf(float a, float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

struct timespec tic() {
  struct timespec time_start;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  return time_start;
}

float toc(struct timespec *tic) {
  struct timespec toc;
  float time_elasped;

  clock_gettime(CLOCK_MONOTONIC, &toc);
  time_elasped = (toc.tv_sec - tic->tv_sec);
  time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

  return time_elasped;
}

static void mat_set(double *A,
                    const size_t stride,
                    const size_t i,
                    const size_t j,
                    const double val) {
  assert(A != NULL);
  assert(stride != 0);

  A[(i * stride) + j] = val;
}

double *create_random_sq_matrix(const size_t m) {
  double *A = (double *) malloc(sizeof(double) * m * m);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < m; j++) {
      mat_set(A, m, i, j, randf(-1.0, 1.0));
    }
  }

  return A;
}
