#include "prototype/munit.hpp"
#include "prototype/core.hpp"
#include "calibration/camchain.hpp"
#include "calibration/calib_data.hpp"
#include "calibration/residual.hpp"

namespace prototype {

int test_CalibData_load() {
  CalibData data;

  int retval = data.load("/home/chutsu/se_datasets/calib/gimbal/dataset_4/measurements");
  MU_CHECK(retval == 0);
  MU_CHECK(data.nb_measurements > 0);

  const std::string base_dir = "/home/chutsu/se_datasets/calib/gimbal/dataset_4";

  Camchain camchain;
  if (camchain.load(3, base_dir + "/camchain.yaml") != 0) {
    FATAL("Failed to load camchain file!");
  }

  for (int j = 0; j < data.nb_measurements; j++) {
    printf("Measurement [%d]\n", j);

    const std::string img0_file = base_dir + "/cam0/" + std::to_string(j) + ".jpg";
    const std::string img1_file = base_dir + "/cam2/" + std::to_string(j) + ".jpg";
    const cv::Mat img0 = cv::imread(img0_file);
    const cv::Mat img1 = camchain.cam[2].undistortImage(cv::imread(img1_file), 0.0);

    cv::Mat image;
    cv::hconcat(img0, img1, image);

    // cv::Mat image(480, 768 * 2, CV_8UC3, cv::Scalar(0));
    for (int i = 0; i < data.Q_s[j].rows(); i++) {
      // Set colour
      const int blue = randi(0, 255);
      const int green = randi(0, 255);
      const int red = randi(0, 255);
      const cv::Scalar color(blue, green, red);

      // Show pixel measurements from static camera
      const Vec2 p_s = data.Q_s[j].row(i);
      const cv::Point2f pt_s(p_s(0), p_s(1));
      cv::circle(image, pt_s, 2, color, -1);

      // Show pixel measurements from dynamic camera
      const Vec2 p_d = data.Q_d[j].row(i);
      cv::Point2f pt_d(p_d(0), p_d(1));
      pt_d = camchain.cam[2].undistortPoint(pt_d);

      Vec3 x{pt_d.x, pt_d.y, 1.0};
      x = camchain.cam[2].K() * x;
      pt_d.x = x(0) + 752;
      pt_d.y = x(1);

      cv::circle(image, pt_d, 2, color, -1);
    }


    // Show
    cv::imshow("Image", image);
    cv::waitKey(0);

    // cv::imshow("Image0", img0);
    // cv::imshow("Image1", img1);
    // cv::waitKey(0);
  }

  // Setup optimization problem
  const Mat3 K_s = camchain.cam[0].K();
  const Mat3 K_d = camchain.cam[2].K();
  const Vec4 D_s = camchain.cam[0].D();
  const Vec4 D_d = camchain.cam[2].D();
  const double theta1_offset = camchain.theta1_offset;
  const double theta2_offset = camchain.theta2_offset;

  for (int i = 0; i < data.nb_measurements; i++) {
    for (int j = 0; j < data.P_s[i].rows(); j++) {
      const Vec3 P_s = data.P_s[i].row(j);
      const Vec3 P_d = data.P_d[i].row(j);
      // const Vec2 Q_s = data.Q_s[i].row(j);
      // const Vec2 Q_d = data.Q_d[i].row(j);

      // Undistort pixel measurements from static camera
      const Vec2 p_s = data.Q_s[j].row(i);
      cv::Point2f pt_s(p_s(0), p_s(1));
      pt_s = camchain.cam[0].undistortPoint(pt_s);
      Vec3 x_s{pt_s.x, pt_s.y, 1.0};
      x_s = camchain.cam[0].K() * x_s;
      const Vec2 Q_s{x_s(0), x_s(1)};

      // Undistort pixel measurements from dynamic camera
      const Vec2 p_d = data.Q_d[j].row(i);
      cv::Point2f pt_d(p_d(0), p_d(1));
      pt_d = camchain.cam[2].undistortPoint(pt_d);
      Vec3 x_d{pt_d.x, pt_d.y, 1.0};
      x_d = camchain.cam[2].K() * x_d;
      const Vec2 Q_d{x_d(0), x_d(1)};

      GimbalCalibResidual residual(P_s, P_d,
                                   Q_s, Q_d,
                                   K_s, K_d,
                                   D_s, D_d,
                                   theta1_offset, theta2_offset);

      // residual(&camchain.tau_s,
      //          &camchain.tau_d,
      //          &camchain.w1,
      //          &camchain.w2,
      //          &camchain.theta1_offset,
      //          &camchain.theta2_offset);
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CalibData_load);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
