#include <Eigen/Dense>

#include "benchmark.hpp"

int main() {
  // Array - Blas
  for (size_t k = 10; k < 1000; k+=10) {
    size_t m = k;
    double *A = create_random_sq_matrix(m);
    double *B = create_random_sq_matrix(m);
    double *C = (double *) malloc(sizeof(double) * m * m);

    sleep(0.1);
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;
    A_.resize(m, m);
    B_.resize(m, m);
    C_.resize(m, m);

    size_t index = 0;
    for (size_t i = 0; i < m; i++) {
      for (size_t j = 0; j < m; j++) {
        A_(i, j) = A[index];
        B_(i, j) = B[index];
        index++;
      }
    }

    struct timespec t = tic();
    C_ = A_ * B_;
    printf("matrix_size: %ld\teigen matmul: %fs\n", m, toc(&t));

    free(A);
    free(B);
    free(C);
  }

  return 0;
}
