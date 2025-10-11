#include <gtest/gtest.h>

#include "Equi4.hpp"

namespace xyz {

TEST(Equi4, distort) {
  const int num_points = 100;
  const Vec4 dist_params{0.1, 0.01, 0.01, 0.01};
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  for (int i = 0; i < num_points; i++) {
    // Distort point
    const Vec3 point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const Vec2 p{point(0) / point(2), point(1) / point(2)};
    const Vec2 p_d = Equi4::distort(dist_params, p);

    // Use opencv to use equi distortion to distort point
    const std::vector<cv::Point2f> points{cv::Point2f(p(0), p(1))};
    const cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    const cv::Vec4f D(k1, k2, k3, k4);
    std::vector<cv::Point2f> image_points;
    cv::fisheye::distortPoints(points, image_points, K, D);
    const Vec2 expected{image_points[0].x, image_points[0].y};

    // // Debug
    // std::cout << p_d.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    ASSERT_TRUE((p_d - expected).norm() < 1.0e-5);
  }
}

TEST(Equi4, undistort) {
  const int num_points = 100;
  const Vec4 dist_params{0.1, 0.2, 0.3, 0.4};

  for (int i = 0; i < num_points; i++) {
    // Distort point
    const Vec3 point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const Vec2 p{point.x() / point.z(), point.y() / point.z()};
    const Vec2 p_d = Equi4::distort(dist_params, p);
    const Vec2 p_ud = Equi4::undistort(dist_params, p_d);

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << p_d.transpose() << std::endl;
    // std::cout << p_ud.transpose() << std::endl;
    // std::cout << std::endl;

    ASSERT_TRUE((p - p_ud).norm() < 1.0e-5);
  }
}

} // namespace xyz
