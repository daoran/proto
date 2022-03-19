#include "benchmark.hpp"
#include <lapacke.h>

int svd(double *A, int m, int n, double *U, double *s, double *V_t) {
  const int lda = n;
  const int ldu = m;
  const int ldvt = n;
  const char jobu = 'A';
  const char jobvt = 'A';
  const int superb_size = (m < n) ? m : n;
  double *superb = malloc(sizeof(double) * (superb_size - 1));
  lapack_int retval = LAPACKE_dgesvd(LAPACK_ROW_MAJOR,
                                     jobu,
                                     jobvt,
                                     m,
                                     n,
                                     A,
                                     lda,
                                     s,
                                     U,
                                     ldu,
                                     V_t,
                                     ldvt,
                                     superb);
  if (retval > 0) {
    return -1;
  }

  /* Clean up */
  free(superb);

  return 0;
}

int main() {
  for (size_t k = 10; k <= 1000; k += 10) {
    size_t m = k;
    double *A = create_random_sq_matrix(m);
    double *U = malloc(sizeof(double) * m * m);
    double *d = malloc(sizeof(double) * m);
    double *V_t = malloc(sizeof(double) * m * m);

    sleep(0.1);
    struct timespec t = tic();
    svd(A, m, m, U, d, V_t);
    printf("matrix_size: %ld\t: dgesvd: %fs\n", m, toc(&t));
    free(A);
    free(U);
    free(d);
    free(V_t);
  }

  return 0;
}
