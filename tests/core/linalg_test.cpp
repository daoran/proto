#include "prototype/core/linalg.hpp"
#include "prototype/munit.hpp"

namespace prototype {

int test_zeros() {
  matx_t A = zeros(2, 2);

  MU_CHECK_EQ(A.rows(), 2);
  MU_CHECK_EQ(A.cols(), 2);

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      MU_CHECK_FLOAT(0.0, A(i, j));
    }
  }

  return 0;
}

int test_I() {
  matx_t A = I(2, 2);

  MU_CHECK_EQ(A.rows(), 2);
  MU_CHECK_EQ(A.cols(), 2);

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (i != j) {
        MU_CHECK_FLOAT(0.0, A(i, j));
      } else {
        MU_CHECK_FLOAT(1.0, A(i, j));
      }
    }
  }

  return 0;
}

int test_ones() {
  matx_t A = ones(2, 2);

  MU_CHECK_EQ(A.rows(), 2);
  MU_CHECK_EQ(A.cols(), 2);
  std::cout << A << std::endl;

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      MU_CHECK_FLOAT(1.0, A(i, j));
    }
  }

  return 0;
}

int test_hstack() {
  mat2_t A;
  A.fill(1);

  mat2_t B;
  B.fill(2);

  matx_t C = hstack(A, B);
  std::cout << C << std::endl;

  MU_CHECK_EQ(2, C.rows());
  MU_CHECK_EQ(4, C.cols());
  MU_CHECK(C.block(0, 0, 2, 2).isApprox(A));
  MU_CHECK(C.block(0, 2, 2, 2).isApprox(B));

  return 0;
}

int test_vstack() {
  mat2_t A;
  A.fill(1);

  mat2_t B;
  B.fill(2);

  matx_t C = vstack(A, B);
  std::cout << C << std::endl;

  MU_CHECK_EQ(4, C.rows());
  MU_CHECK_EQ(2, C.cols());
  MU_CHECK(C.block(0, 0, 2, 2).isApprox(A));
  MU_CHECK(C.block(2, 0, 2, 2).isApprox(B));

  return 0;
}

int test_dstack() {
  mat2_t A;
  A.fill(1);

  mat2_t B;
  B.fill(2);

  matx_t C = dstack(A, B);
  std::cout << C << std::endl;

  MU_CHECK_EQ(4, C.rows());
  MU_CHECK_EQ(4, C.cols());
  MU_CHECK(C.block(0, 0, 2, 2).isApprox(A));
  MU_CHECK(C.block(0, 2, 2, 2).isApprox(zeros(2, 2)));
  MU_CHECK(C.block(2, 2, 2, 2).isApprox(B));
  MU_CHECK(C.block(2, 0, 2, 2).isApprox(zeros(2, 2)));

  return 0;
}

int test_skew() {
  vec3_t v{1.0, 2.0, 3.0};
  mat3_t X = skew(v);

  mat3_t X_expected;
  // clang-format off
  X_expected << 0.0, -3.0, 2.0,
                3.0, 0.0, -1.0,
                -2.0, 1.0, 0.0;
  // clang-format on

  MU_CHECK(X_expected.isApprox(X));

  return 0;
}

int test_skewsq() { return 0; }

int test_enforce_psd() {
  mat3_t A;
  // clang-format off
  A << 0.0, -3.0, 2.0,
       3.0, 0.0, -1.0,
       -2.0, 1.0, 0.0;
  // clang-format on

  mat3_t B = enforce_psd(A);
  for (int i = 0; i < B.rows(); i++) {
    for (int j = 0; j < B.cols(); j++) {
      MU_CHECK(B(i, j) >= 0);
    }
  }

  return 0;
}

int test_nullspace() {
  mat3_t A;
  // clang-format off
  A << 1.0, 2.0, 3.0,
       1.0, 2.0, 3.0,
       1.0, 2.0, 3.0;
  // clang-format on

  matx_t B = nullspace(A);
  std::cout << B << std::endl;
  std::cout << A * B << std::endl;

  matx_t C = A * B;
  for (int i = 0; i < C.rows(); i++) {
    for (int j = 0; j < C.cols(); j++) {
      MU_CHECK_FLOAT(0.0, C(i, j));
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_zeros);
  MU_ADD_TEST(test_I);
  MU_ADD_TEST(test_ones);
  MU_ADD_TEST(test_hstack);
  MU_ADD_TEST(test_vstack);
  MU_ADD_TEST(test_dstack);
  MU_ADD_TEST(test_skew);
  MU_ADD_TEST(test_skewsq);
  MU_ADD_TEST(test_enforce_psd);
  MU_ADD_TEST(test_nullspace);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
