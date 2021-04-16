#include <Eigen/Dense>

#include "benchmark.hpp"

#include <iostream>

int main() {
  for (size_t k = 10; k <= 1000; k+=10) {
    size_t m = k;
    double *A = create_random_sq_matrix(m);

    sleep(0.1);
    Eigen::MatrixXd A_;
    A_.resize(m, m);
    size_t index = 0;
    for (size_t i = 0; i < m; i++) {
      for (size_t j = 0; j < m; j++) {
        A_(i, j) = A[index];
      }
    }

    struct timespec t = tic();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A_, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::VectorXd d = svd.singularValues();
    Eigen::MatrixXd S = d.asDiagonal();
    Eigen::MatrixXd V = svd.matrixV();
    printf("matrix_size: %ld\tEigen SVD: %fs\n", A_.rows(), toc(&t));

    bool retval = A_.isApprox(U * S * V.transpose());
    if (retval == false) {
      exit(-1);
    }

    free(A);
  }

  return 0;
}
