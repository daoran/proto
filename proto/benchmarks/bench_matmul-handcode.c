#include <assert.h>
#include <unistd.h>

#include "benchmark.hpp"

void dot(const double *A,
         const size_t A_m,
         const size_t A_n,
         const double *B,
         const size_t B_m,
         const size_t B_n,
         double *C) {
  assert(A != NULL && B != NULL && A != C && B != C);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

  size_t m = A_m;
  size_t n = B_n;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      double sum = 0.0;
      for (size_t k = 0; k < A_n; k++) {
        sum += A[(i * A_n) + j] * B[(i * B_n) + j];
      }
      C[(i * n) + j] = sum;
    }
  }
}

int main() {
  // Array - Blas
  for (size_t k = 10; k < 1000; k += 10) {
    size_t m = k;
    double *A = create_random_sq_matrix(m);
    double *B = create_random_sq_matrix(m);
    double *C = (double *) malloc(sizeof(double) * m * m);

    // Hand coded dot()
    /* sleep(1); */
    struct timespec t = tic();
    dot(A, m, m, B, m, m, C);
    printf("matrix_size: %ld\tdot(): %fs\n", m, toc(&t));

    free(A);
    free(B);
    free(C);
  }

  return 0;
}
