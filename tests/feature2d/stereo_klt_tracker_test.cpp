#include "prototype/munit.hpp"
#include "dataset/kitti/kitti.hpp"
#include "feature2d/stereo_klt_tracker.hpp"

namespace prototype {

static const std::string DATASET_PATH = "/data/kitti/raw";

struct test_config {
  RawDataset raw_dataset;
  StereoKLTTracker tracker;
  test_config() {}
};

struct test_config test_setup(const int type = STATIC_STEREO_TRACK) {
  struct test_config test;

  test.raw_dataset = RawDataset(DATASET_PATH, "2011_09_26", "0001", "extract");
  if (test.raw_dataset.load() != 0) {
    exit(-1);
  }

  // Obtain cam0 to cam1 transform
  const Mat3 R_cam1_cam0 = test.raw_dataset.calib_cam_to_cam.R[1];
  const Vec3 t_cam1_cam0 = test.raw_dataset.calib_cam_to_cam.T[1];
  const Mat4 T_cam1_cam0 = transformation_matrix(R_cam1_cam0, t_cam1_cam0);

  // Create camera properties
  const Vec2 image_size = test.raw_dataset.calib_cam_to_cam.S[0];
  CameraProperty cam0(0,
                      "pinhole",
                      test.raw_dataset.calib_cam_to_cam.K[0],
                      "radtan",
                      test.raw_dataset.calib_cam_to_cam.D[0],
                      image_size);
  CameraProperty cam1(1,
                      "pinhole",
                      test.raw_dataset.calib_cam_to_cam.K[1],
                      "radtan",
                      test.raw_dataset.calib_cam_to_cam.D[1],
                      image_size);

  // Initialize tracker
  if (type == STATIC_STEREO_TRACK) {
    test.tracker = StereoKLTTracker(cam0, cam1, T_cam1_cam0, 2, 20);

  } else if (type == DYNAMIC_STEREO_TRACK) {
    // clang-format off
    GimbalModel gimbal_model;
    gimbal_model.tau_s << -0.0411708, -0.0903084, 0.0862729, 0.0197209, -0.0330993, -0.0218305;
    gimbal_model.tau_d << 0.00381052, 0.000789979, -0.0343282, 1.57809, 0.00168973, -1.56902;
    gimbal_model.w1 << 0.0387271, -0.000660779, 1.5725;
    gimbal_model.w2 << 0.0, 0.0, 0.0;
    gimbal_model.theta1_offset = -1.5707;
    gimbal_model.theta2_offset = 0.0;
    // clang-format on

    test.tracker = StereoKLTTracker(cam0, cam1, gimbal_model, 2, 20);
  }

  return test;
}

int test_StereoKLTTracker_match() {
  struct test_config test = test_setup();

  const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[0]);
  const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[0]);
  std::vector<cv::Point2f> cam0_pts = test.tracker.detect(cam0_img);
  std::vector<cv::Point2f> cam1_pts = cam0_pts;
  std::vector<uchar> mask;
  test.tracker.match(cam0_img, cam1_img, cam0_pts, cam1_pts, mask);

  // Visualize
  // cv::Mat match = draw_matches(cam0_img, cam1_img, cam0_pts, cam1_pts, mask);
  // cv::imshow("Matches", match);
  // cv::waitKey(0);

  // Assert
  MU_CHECK(cam0_pts.size() > 0);
  MU_CHECK(cam1_pts.size() > 0);
  MU_CHECK(cam0_pts.size() == cam1_pts.size());
  MU_CHECK(cam0_pts.size() == mask.size());

  return 0;
}

int test_StereoKLTTracker_initialize() {
  struct test_config test = test_setup();

  const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[0]);
  const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[0]);
  test.tracker.initialize(cam0_img, cam1_img);

  // Assert
  MU_CHECK_FLOAT(test.tracker.counter_frame_id, 0);
  MU_CHECK(test.tracker.cam0_pts.size() > 0);
  MU_CHECK(test.tracker.cam1_pts.size() > 0);
  MU_CHECK(test.tracker.prev_cam0_img.empty() == false);
  MU_CHECK(test.tracker.prev_cam1_img.empty() == false);

  return 0;
}

int test_StereoKLTTracker_track() {
  struct test_config test = test_setup();

  // Initialize tracker
  const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[0]);
  const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[0]);
  test.tracker.initialize(cam0_img, cam1_img);

  // Update tracker
  test.tracker.show_matches = false;

  for (int i = 1; i < 5; i++) {
    const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[i]);
    const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[i]);
    test.tracker.trackFeatures(cam0_img, cam1_img);

    // Break loop if 'q' was pressed
    if (test.tracker.show_matches && cv::waitKey(0) == 113) {
      break;
    }
  }

  return 0;
}

int test_StereoKLTTracker_update_static() {
  struct test_config test = test_setup();

  // Update tracker
  test.tracker.show_matches = false;
  test.tracker.show_tracking = true;

  for (size_t i = 0; i < test.raw_dataset.cam0.size(); i++) {
    const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[i]);
    const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[i]);
    test.tracker.update(cam0_img, cam1_img);

    auto tracks = test.tracker.getLostTracks();
    int nb_tracks = 0;
    for (auto track : tracks) {
      MU_CHECK(track.T_cam1_cam0.size() == 1);
      if (track.trackedLength() > 10) {
        nb_tracks++;
      }
    }

    // Break loop if 'q' was pressed
    if (test.tracker.show_matches && cv::waitKey(0) == 113) {
      break;
    }
  }

  return 0;
}

int test_StereoKLTTracker_update_dynamic() {
  struct test_config test = test_setup(DYNAMIC_STEREO_TRACK);

  // Update tracker
  test.tracker.show_matches = true;
  test.tracker.show_tracking = false;
  double roll = 0.0;
  double pitch = 0.0;

  for (size_t i = 0; i < test.raw_dataset.cam0.size(); i++) {
    const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[i]);
    const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[i]);
    test.tracker.update(cam0_img, cam1_img, roll, pitch);
    // roll += 0.01;
    // pitch += 0.01;

    auto tracks = test.tracker.getLostTracks();
    // for (auto track : tracks) {
    //   MU_CHECK((track.T_cam1_cam0[0] - track.T_cam1_cam0[1]).norm() > 0.01);
    //   MU_CHECK(track.T_cam1_cam0.size() == track.track0.size());
    // }

    // Break loop if 'q' was pressed
    if (test.tracker.show_matches && cv::waitKey(0) == 113) {
      break;
    }
  }

  return 0;
}

int test_StereoKLTTracker_getLostTracks() {
  struct test_config test = test_setup();

  test.tracker.show_matches = false;
  for (int i = 1; i < 30; i++) {
    const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[i]);
    const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[i]);
    test.tracker.update(cam0_img, cam1_img);

    auto tracks = test.tracker.getLostTracks();
    for (auto track : tracks) {
      MU_CHECK(track.track0.size() == track.track1.size());
    }

    // Save feature tracks and plot
    // std::string output_dir = "/tmp/tracker_test/frame_" + std::to_string(i);
    // save_feature_tracks(tracks, output_dir);
    // std::string cmd = "python3 scripts/plot_feature_tracks.py " + output_dir;
    // if (std::system(cmd.c_str()) != 0) {
    //   return -1;
    // }

    // Break loop if 'q' was pressed
    if (test.tracker.show_matches && cv::waitKey(0) == 113) {
      break;
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_StereoKLTTracker_match);
  MU_ADD_TEST(test_StereoKLTTracker_initialize);
  MU_ADD_TEST(test_StereoKLTTracker_track);
  MU_ADD_TEST(test_StereoKLTTracker_update_static);
  MU_ADD_TEST(test_StereoKLTTracker_update_dynamic);
  MU_ADD_TEST(test_StereoKLTTracker_getLostTracks);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
