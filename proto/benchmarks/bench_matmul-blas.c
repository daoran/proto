#include <cblas.h>
#include <assert.h>
#include <unistd.h>

#include "benchmark.hpp"

void dot_cblas(const double *A,
               const size_t A_m,
               const size_t A_n,
               const double *B,
               const size_t B_m,
               const size_t B_n,
               double *C) {
  UNUSED(B_m);
  assert(A != NULL && B != NULL && C != NULL);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

  cblas_dgemm(CblasColMajor, /* Matrix data arrangement */
              CblasNoTrans,  /* Transpose A */
              CblasNoTrans,  /* Transpose B */
              A_m,           /* Number of rows in A and C */
              B_n,           /* Number of cols in B and C */
              A_n,           /* Number of cols in A */
              1.0,           /* Scaling factor for the product of A and B */
              A,             /* Matrix A */
              A_n,           /* First dimension of A */
              B,             /* Matrix B */
              B_n,           /* First dimension of B */
              1.0,           /* Scale factor for C */
              C,             /* Output */
              A_m            /* First dimension of C */
  );
}

int main() {
  // Array - Blas
  for (size_t k = 10; k < 1000; k += 10) {
    size_t m = k;
    double *A = create_random_sq_matrix(m);
    double *B = create_random_sq_matrix(m);
    double *C = (double *) malloc(sizeof(double) * m * m);

    /* CBLAS dot() */
    /* sleep(1); */
    struct timespec t = tic();
    dot_cblas(A, m, m, B, m, m, C);
    printf("matrix_size: %ld\tdot_cblas(): %fs\n", m, toc(&t));

    free(A);
    free(B);
    free(C);
  }

  return 0;
}
