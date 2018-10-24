#include "prototype/munit.hpp"
#include "camera/ids.hpp"
#include "calibration/calib_validator.hpp"
#include "gimbal/gimbal.hpp"

namespace prototype {

#define TEST_CALIB_FILE "test_configs/gimbal/calibration/se_camchain.yaml"
#define TEST_CALIB_2_FILE "test_configs/gimbal/calibration/se_camchain2.yaml"
#define TEST_CALIB_3_FILE "test_configs/gimbal/calibration/se_camchain3.yaml"
#define TEST_TARGET_FILE "test_configs/gimbal/calibration/chessboard.yaml"
#define TEST_CAM0_CONFIG "test_configs/camera/ueye/cam0.yaml"
#define TEST_CAM1_CONFIG "test_configs/camera/ueye/cam1.yaml"
#define TEST_CAM2_CONFIG "test_configs/camera/ueye/cam2.yaml"
#define TEST_IMAGE "test_data/calibration2/img_0.jpg"
#define TEST_CHESSBOARD_CAM0 "test_data/chessboard/cam0/"
#define TEST_CHESSBOARD_CAM1 "test_data/chessboard/cam1/"
#define TEST_GIMBAL_CONFIG "test_configs/gimbal/config2.yaml"

int test_CalibValidator_constructor() {
  CalibValidator validator;

  return 0;
}

int test_CalibValidator_load() {
  // Load validator
  CalibValidator validator;
  if (validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE) != 0) {
    LOG_ERROR("Failed to load validator!");
    return -1;
  }

  std::cout << validator.camchain.cam[0] << std::endl;
  std::cout << validator.camchain.cam[1] << std::endl;
  std::cout << validator.camchain.cam[2] << std::endl;
  std::cout << "T_C1_C0:\n"
            << validator.camchain.T_C1_C0 << std::endl
            << std::endl;
  std::cout << validator.gimbal_model << std::endl;

  return 0;
}

int test_CalibValidator_validate() {
  // Load validator
  CalibValidator validator;
  if (validator.load(3, TEST_CALIB_2_FILE, TEST_TARGET_FILE) != 0) {
    LOG_ERROR("Failed to load validator!");
    return -1;
  }

  cv::Mat image = cv::imread("test_data/chessboard/cam0/image_0.jpg");
  cv::Mat result = validator.validate(0, image);
  // cv::imshow("Image", result);
  // cv::waitKey();

  return 0;
}

int test_CalibValidator_validateStereo() {
  // Load validator
  CalibValidator validator;
  if (validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE) != 0) {
    LOG_ERROR("Failed to load validator!");
    return -1;
  }

  std::cout << TEST_CALIB_2_FILE << std::endl;
  std::cout << validator.camchain.cam[0] << std::endl;
  std::cout << validator.camchain.cam[1] << std::endl;
  std::cout << validator.camchain.T_C1_C0 << std::endl;

  const cv::Mat img0 = cv::imread(TEST_CHESSBOARD_CAM0 "image_0.jpg");
  const cv::Mat img1 = cv::imread(TEST_CHESSBOARD_CAM1 "image_0.jpg");
  const cv::Mat result = validator.validateStereo(img0, img1);
  cv::imshow("Image", result);
  cv::waitKey();

  return 0;
}

int test_CalibValidator_validateTriclops() {
  // Load validator
  CalibValidator validator;
  const std::string measurements_dir = "/home/chutsu/se_datasets/calib/gimbal/dataset_4/measurements";
  const std::string validation_dir = "/home/chutsu/se_datasets/calib/gimbal/validation_2";

  const std::string camchain_file = measurements_dir + "/camchain_optimized.yaml";
  const std::string joint_file = validation_dir + "/joint.csv";
  const std::string target_file = validation_dir + "/chessboard.yaml";

  if (validator.load(3, camchain_file, target_file) != 0) {
    LOG_ERROR("Failed to load validator!");
    return -1;
  }

  std::cout << "camchain_file: " << camchain_file << std::endl;
  std::cout << validator.camchain.cam[0] << std::endl;
  std::cout << validator.camchain.cam[1] << std::endl;
  std::cout << validator.camchain.T_C1_C0 << std::endl;

  MatX joint_data;
  if (csv2mat(joint_file, false, joint_data) != 0) {
    LOG_ERROR("Failed to load joint data!");
    return -1;
  }

  for (int i = 0; i < 50; i++) {
    const std::string cam0_img_file = validation_dir + "/cam0/" + std::to_string(i) + ".jpg";
    const std::string cam1_img_file = validation_dir + "/cam1/" + std::to_string(i) + ".jpg";
    const std::string cam2_img_file = validation_dir + "/cam2/" + std::to_string(i) + ".jpg";
    const cv::Mat cam0_img = cv::imread(cam0_img_file);
    const cv::Mat cam1_img = cv::imread(cam1_img_file);
    const cv::Mat cam2_img = cv::imread(cam2_img_file);
    // const cv::Mat result = validator.validateStereo(cam0_img, cam1_img);
    const double roll = joint_data(i, 0);
    const double pitch = joint_data(i, 1);
    const cv::Mat result = validator.validateTriclops(cam0_img, cam1_img, cam2_img, roll, pitch);
    cv::imshow("Image", result);
    cv::waitKey();
  }

  return 0;
}

int test_CalibValidator_validate_live() {
  // Load validator
  CalibValidator validator;
  if (validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE) != 0) {
    LOG_ERROR("Failed to load validator!");
    return -1;
  }

  // Load camera
  IDSCamera camera;
  camera.configure(TEST_CAM0_CONFIG);

  // Loop webcam
  while (true) {
    cv::Mat image;
    if (camera.getFrame(image) != 0) {
      return -1;
    }

    cv::Mat result = validator.validate(0, image);
    cv::imshow("Validation", result);
    if (cv::waitKey(1) == 113) {
      break;
    }
  }

  // Needed to exit cleanly
  cv::destroyAllWindows();

  return 0;
}

int test_CalibValidator_validateStereo_live() {
  // Load validator
  CalibValidator validator;
  if (validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE) != 0) {
    LOG_ERROR("Failed to load validator!");
    return -1;
  }

  // Load camera
  IDSCamera cam0, cam1;
  cam0.configure(TEST_CAM0_CONFIG);
  cam1.configure(TEST_CAM1_CONFIG);

  // Loop webcam
  while (true) {
    cv::Mat img0, img1;
    if (cam0.getFrame(img0) != 0) {
      return -1;
    }
    if (cam1.getFrame(img1) != 0) {
      return -1;
    }

    cv::Mat result = validator.validateStereo(img0, img1);
    cv::imshow("Validation", result);
    if (cv::waitKey(1) == 113) {
      break;
    }
  }

  // Needed to exit cleanly
  cv::destroyAllWindows();

  return 0;
}

int test_CalibValidator_validateTriclops_live() {
  // Load validator
  CalibValidator validator;
  if (validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE) != 0) {
    LOG_ERROR("Failed to load validation!");
    return -1;
  }

  // Load gimbal
  Gimbal gimbal;
  if (gimbal.configure(TEST_GIMBAL_CONFIG) != 0) {
    LOG_ERROR("Failed to load gimbal!");
    return -1;
  }
  gimbal.off();

  // Load camera
  IDSCamera cam0, cam1, cam2;
  if (cam0.configure(TEST_CAM0_CONFIG) != 0) {
    LOG_ERROR("Failed to load cam0!");
    return -1;
  }
  if (cam1.configure(TEST_CAM1_CONFIG) != 0) {
    LOG_ERROR("Failed to load cam1!");
    return -1;
  }
  if (cam2.configure(TEST_CAM2_CONFIG) != 0) {
    LOG_ERROR("Failed to load cam2!");
    return -1;
  }

  // Loop webcam
  while (true) {
    cv::Mat img0, img1, img2;
    if (cam0.getFrame(img0) != 0) {
      return -1;
    }
    if (cam1.getFrame(img1) != 0) {
      return -1;
    }
    if (cam2.getFrame(img2) != 0) {
      return -1;
    }

    gimbal.update();
    double joint_roll = gimbal.camera_angles(0) - gimbal.frame_angles(0);
    double joint_pitch = gimbal.camera_angles(1) - gimbal.frame_angles(1);

    cv::Mat result =
        validator.validateTriclops(img0, img1, img2, joint_roll, joint_pitch);
    cv::imshow("Validation", result);
    if (cv::waitKey(1) == 113) {
      break;
    }
  }

  // Needed to exit cleanly
  cv::destroyAllWindows();

  return 0;
}


void test_suite() {
  // MU_ADD_TEST(test_CalibValidator_constructor);
  // MU_ADD_TEST(test_CalibValidator_load);
  // MU_ADD_TEST(test_CalibValidator_validate);
  // MU_ADD_TEST(test_CalibValidator_validateStereo);
  MU_ADD_TEST(test_CalibValidator_validateTriclops);
  // MU_ADD_TEST(test_CalibValidator_validate_live);
  // MU_ADD_TEST(test_CalibValidator_validateStereo_live);
  // MU_ADD_TEST(test_CalibValidator_validateTriclops_live);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
