#include <gtest/gtest.h>

#include "camera/distortion/RadTan4.hpp"

namespace xyz {

TEST(RadTan4, distort) {
  const int nb_points = 100;
  const Vec4 dist_params{0.1, 0.01, 0.01, 0.01};
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double p1 = dist_params(2);
  const double p2 = dist_params(3);

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const Vec3 p{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const Vec2 x{p(0) / p(2), p(1) / p(2)};
    const Vec2 pixel = RadTan4::distort(dist_params, x);

    // Use opencv to use radtan distortion to distort point
    const std::vector<cv::Point3f> points{cv::Point3f(p(0), p(1), p(2))};
    const cv::Vec3f rvec;
    const cv::Vec3f tvec;
    const cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    const cv::Vec4f D(k1, k2, p1, p2);
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(points, rvec, tvec, K, D, image_points);
    const Vec2 expected{image_points[0].x, image_points[0].y};

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << pixel.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    ASSERT_TRUE((pixel - expected).norm() < 1.0e-5);
  }
}

TEST(RadTan4, undistort) {
  const int nb_points = 100;
  const Vec4 dist_params{0.1, 0.01, 0.01, 0.01};

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const Vec3 point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const Vec2 p{point(0) / point(2), point(1) / point(2)};
    const Vec2 p_d = RadTan4::distort(dist_params, p);
    const Vec2 p_ud = RadTan4::undistort(dist_params, p_d);

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << p_ud.transpose() << std::endl;
    // std::cout << std::endl;

    ASSERT_TRUE((p - p_ud).norm() < 1.0e-5);
  }
}

} // namespace xyz
