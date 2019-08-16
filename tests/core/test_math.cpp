#include <unistd.h>

#include "proto/munit.hpp"
#include "proto/core/math.hpp"
#include "proto/core/data.hpp"

namespace proto {

/******************************************************************************
 * Algebra
 *****************************************************************************/

int test_sign() {
  MU_CHECK(sign(1.0) == 1);
  MU_CHECK(fltcmp(sign(0.0), 0.0) == 0);
  MU_CHECK(sign(-1.0) == -1);
  return 0;
}

int test_fltcmp() {
  MU_CHECK(fltcmp(1.0, 1.0) == 0);
  MU_CHECK(fltcmp(1.0, 0.9999) == 1);
  MU_CHECK(fltcmp(1.0, 0.0) == 1);
  MU_CHECK(fltcmp(0.0, 1.0) == -1);
  return 0;
}

int test_linspace() {
  const auto range = linspace(0.0, 5.0, 10);

  for (const auto &el : range) {
    std::cout << el << std::endl;
  }

  MU_CHECK(fltcmp(range.front(), 0.0) == 0);
  MU_CHECK(fltcmp(range.back(), 5.0) == 0);
  MU_CHECK(range.size() == 10);

  return 0;
}

int test_linspace_timestamps() {
  const timestamp_t ts_start = 0;
  const timestamp_t ts_end = 5e9;
  const timestamps_t range = linspace(ts_start, ts_end, 10);

  for (const auto &el : range) {
    std::cout << el << std::endl;
  }

  MU_CHECK(range.front() == 0);
  MU_CHECK(range.back() == 5e9);
  MU_CHECK(range.size() == 10);

  return 0;
}


/******************************************************************************
 * Geometry
 *****************************************************************************/

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

int test_latlon_offset() {
  // UWaterloo 110 yards Canadian Football field from one end to another
  double lat = 43.474357;
  double lon = -80.550415;

  double offset_N = 44.1938;
  double offset_E = 90.2336;

  double lat_new = 0.0;
  double lon_new = 0.0;

  // calculate football field GPS coordinates
  latlon_offset(lat, lon, offset_N, offset_E, &lat_new, &lon_new);
  std::cout << "lat new: " << lat_new << std::endl;
  std::cout << "lon new: " << lon_new << std::endl;

  // gps coordinates should be close to (43.474754, -80.549298)
  MU_CHECK_NEAR(43.474754, lat_new, 0.0015);
  MU_CHECK_NEAR(-80.549298, lon_new, 0.0015);

  return 0;
}

int test_latlon_diff() {
  // UWaterloo 110 yards Canadian Football field from one end to another
  double lat_ref = 43.474357;
  double lon_ref = -80.550415;
  double lat = 43.474754;
  double lon = -80.549298;

  double dist_N = 0.0;
  double dist_E = 0.0;

  // calculate football field distance
  latlon_diff(lat_ref, lon_ref, lat, lon, &dist_N, &dist_E);
  double dist = sqrt(pow(dist_N, 2) + pow(dist_E, 2));
  std::cout << "distance north: " << dist_N << std::endl;
  std::cout << "distance east: " << dist_E << std::endl;

  // 110 yards is approx 100 meters
  MU_CHECK_NEAR(100, dist, 1.0);

  return 0;
}

int test_latlon_dist() {
  // UWaterloo 110 yards Canadian Football field from one end to another
  double lat_ref = 43.474357;
  double lon_ref = -80.550415;
  double lat = 43.474754;
  double lon = -80.549298;

  // calculate football field distance
  double dist = latlon_dist(lat_ref, lon_ref, lat, lon);
  std::cout << "distance: " << dist << std::endl;

  // 110 yards is approx 100 meters
  MU_CHECK_NEAR(100, dist, 1.0);

  return 0;
}

/******************************************************************************
 * Linear Algebra
 *****************************************************************************/

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

/******************************************************************************
 * Statistics
 *****************************************************************************/

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

int test_mvn() {
  std::default_random_engine engine;
  const int nb_tests = 10000;  // number of experiments

  matx_t results{3, nb_tests};
  for (int i = 0; i < nb_tests; i++) {
    results.block<3, 1>(0, i) = mvn(engine);
  }
  mat2csv("/tmp/mvn.csv", results);

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_mvn.m /tmp/mvn.csv");
  }

  return 0;
}

int test_gauss_normal() {
  const int nb_tests = 10000;  // number of experiments

  vecx_t results{nb_tests};
  for (int i = 0; i < nb_tests; i++) {
    results(i) = gauss_normal();
  }
  mat2csv("/tmp/gauss_normal.csv", results);

  // Debug
  // const bool debug = true;
  const bool debug = false;
  if (debug) {
    OCTAVE_SCRIPT("scripts/core/plot_gauss_normal.m /tmp/gauss_normal.csv");
  }

  return 0;
}

/******************************************************************************
 * Transform
 *****************************************************************************/

int test_tf_rot() {
  mat4_t T_WS = I(4);
  T_WS.block<3, 3>(0, 0) = 1.0 * ones(3);
  T_WS.block<3, 1>(0, 3) = 2.0 * ones(3, 1);

  MU_CHECK((1.0 * ones(3) - tf_rot(T_WS)).norm() < 1e-5);

  return 0;
}

int test_tf_trans() {
  mat4_t T_WS = I(4);
  T_WS.block<3, 3>(0, 0) = 1.0 * ones(3);
  T_WS.block<3, 1>(0, 3) = 2.0 * ones(3, 1);

  MU_CHECK((2.0 * ones(3, 1) - tf_trans(T_WS)).norm() < 1e-5);

  return 0;
}

/******************************************************************************
 * Time
 *****************************************************************************/

int test_ticAndToc() {
  struct timespec start = tic();
  usleep(10 * 1000);
  MU_CHECK(toc(&start) < 0.011);
  MU_CHECK(toc(&start) > 0.009);
  MU_CHECK(mtoc(&start) < 11.0);
  MU_CHECK(mtoc(&start) > 9.0);

  return 0;
}

void test_suite() {
  // Algebra
  MU_ADD_TEST(test_sign);
  MU_ADD_TEST(test_linspace);
  MU_ADD_TEST(test_linspace_timestamps);

  // Geometry
  MU_ADD_TEST(test_deg2radAndrad2deg);
  MU_ADD_TEST(test_wrap180);
  MU_ADD_TEST(test_wrap360);
  MU_ADD_TEST(test_cross_track_error);
  MU_ADD_TEST(test_point_left_right);
  MU_ADD_TEST(test_closest_point);
  MU_ADD_TEST(test_lerp);
  MU_ADD_TEST(test_latlon_offset);
  MU_ADD_TEST(test_latlon_diff);
  MU_ADD_TEST(test_latlon_dist);

  // Linear algebra
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

  // Statistics
  MU_ADD_TEST(test_median);
  MU_ADD_TEST(test_mvn);
  MU_ADD_TEST(test_gauss_normal);

  // Transform
  MU_ADD_TEST(test_tf_rot);
  MU_ADD_TEST(test_tf_trans);

  // Time
  MU_ADD_TEST(test_ticAndToc);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
