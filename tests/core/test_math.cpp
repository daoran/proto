#include "prototype/core/math.hpp"
#include "prototype/munit.hpp"

namespace proto {

int test_median() {
  std::vector<double> v;

  v.push_back(6);
  v.push_back(3);
  v.push_back(4);
  v.push_back(1);
  v.push_back(5);
  v.push_back(8);

  MU_CHECK_FLOAT(4.5, median(v));

  v.push_back(9);
  MU_CHECK_FLOAT(5.0, median(v));

  return 0;
}

int test_deg2radAndrad2deg() {
  double d_deg;
  double d_rad;

  d_deg = 10;
  d_rad = deg2rad(d_deg);
  MU_CHECK_FLOAT(d_deg, rad2deg(d_rad));

  return 0;
}

int test_wrap180() {
  double retval;

  // normal cases
  retval = wrap180(90.0);
  MU_CHECK_FLOAT(90.0, retval);

  retval = wrap180(180.0);
  MU_CHECK_FLOAT(-180.0, retval);

  retval = wrap180(270.0);
  MU_CHECK_FLOAT(-90.0, retval);

  retval = wrap180(360.0);
  MU_CHECK_FLOAT(0.0, retval);

  // edge cases
  retval = wrap180(-180.0);
  MU_CHECK_FLOAT(-180.0, retval);

  retval = wrap180(-90.0);
  MU_CHECK_FLOAT(-90.0, retval);

  retval = wrap180(450.0);
  MU_CHECK_FLOAT(90.0, retval);

  return 0;
}

int test_wrap360() {
  double retval;

  // normal cases
  retval = wrap360(90.0);
  MU_CHECK_FLOAT(90.0, retval);

  retval = wrap360(180.0);
  MU_CHECK_FLOAT(180.0, retval);

  retval = wrap360(270.0);
  MU_CHECK_FLOAT(270.0, retval);

  retval = wrap360(360.0);
  MU_CHECK_FLOAT(0.0, retval);

  retval = wrap360(450.0);
  MU_CHECK_FLOAT(90.0, retval);

  // edge cases
  retval = wrap360(-180.0);
  MU_CHECK_FLOAT(180.0, retval);

  retval = wrap360(-90.0);
  MU_CHECK_FLOAT(270.0, retval);

  return 0;
}

int test_cross_track_error() {
  vec2_t pos, p1, p2;

  pos << 2, 2;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK_FLOAT(0.0, cross_track_error(p1, p2, pos));

  pos << 2, 3;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK(0.0 < cross_track_error(p1, p2, pos));

  return 0;
}

int test_point_left_right() {
  vec2_t pos, p1, p2;

  pos << 2, 3;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK_EQ(1, point_left_right(p1, p2, pos));

  pos << 2, 1;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK_EQ(2, point_left_right(p1, p2, pos));

  pos << 2, 2;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK_EQ(0, point_left_right(p1, p2, pos));

  pos << 2, 1;
  p1 << 5, 5;
  p2 << 1, 1;
  MU_CHECK_EQ(1, point_left_right(p1, p2, pos));

  pos << 2, 3;
  p1 << 5, 5;
  p2 << 1, 1;
  MU_CHECK_EQ(2, point_left_right(p1, p2, pos));

  pos << 2, 2;
  p1 << 5, 5;
  p2 << 1, 1;
  MU_CHECK_EQ(0, point_left_right(p1, p2, pos));

  return 0;
}

int test_closest_point() {
  int retval;
  vec2_t p1, p2, p3, closest;

  // setup
  p1 << 0, 0;
  p2 << 5, 0;

  // point middle of point a, b
  p3 << 2, 2;
  retval = closest_point(p1, p2, p3, closest);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_FLOAT(2.0, closest(0));
  MU_CHECK_FLOAT(0.0, closest(1));

  // // point before of point a
  // p3 << -1, 2;
  // retval = closest_point(p1, p2, p3, closest);
  // MU_CHECK_EQ(1, retval);
  // MU_CHECK_FLOAT(-1.0, closest(0));
  // MU_CHECK_FLOAT(0.0, closest(1));
  //
  // // point after point b
  // p3 << 6, 2;
  // retval = closest_point(p1, p2, p3, closest);
  // MU_CHECK_EQ(2, retval);
  // MU_CHECK_FLOAT(6.0, closest(0));
  // MU_CHECK_FLOAT(0.0, closest(1));
  //
  // // if point 1 and 2 are same
  // p1 << 0, 0;
  // p2 << 0, 0;
  // p3 << 0, 2;
  // retval = closest_point(p1, p2, p3, closest);
  // MU_CHECK_EQ(-1, retval);
  // MU_CHECK_FLOAT(0.0, closest(0));
  // MU_CHECK_FLOAT(0.0, closest(1));

  return 0;
}

int test_lerp() {
  const vec2_t a{0.0, 5.0};
  const vec2_t b{5.0, 0.0};
  const vec2_t result = lerp(a, b, 0.8);
  std::cout << result << std::endl;

  return 0;
}

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
  MU_ADD_TEST(test_median);
  MU_ADD_TEST(test_deg2radAndrad2deg);
  MU_ADD_TEST(test_wrap180);
  MU_ADD_TEST(test_wrap360);
  MU_ADD_TEST(test_cross_track_error);
  MU_ADD_TEST(test_point_left_right);
  MU_ADD_TEST(test_closest_point);
  MU_ADD_TEST(test_lerp);
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

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
