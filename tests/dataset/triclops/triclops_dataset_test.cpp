#include "prototype/munit.hpp"
#include "dataset/triclops/triclops_dataset.hpp"
#include "quaternion/jpl.hpp"

namespace prototype {
namespace triclops {

#define TEST_DATA "test_data/triclops"

int stereo_camera_cb(const cv::Mat &frame0, const cv::Mat &frame1) {
  cv::Mat stereo_img;
  cv::hconcat(frame0, frame1, stereo_img);

  cv::imshow("Stereo", stereo_img);
  cv::waitKey(1);
  return 0;
}

int test_TriclopsDataset_constructor() {
  TriclopsDataset triclops_data("/tmp");

  MU_CHECK(triclops_data.ok == false);
  MU_CHECK_EQ("/tmp", triclops_data.data_path);

  return 0;
}

int test_TriclopsDataset_loadCamchain() {
  TriclopsDataset triclops_data(TEST_DATA);
  int retval = triclops_data.loadCamchain();

  // std::cout << triclops_data.camchain << std::endl;

  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(triclops_data.camchain.cam[0].camera_model, "pinhole");
  MU_CHECK_EQ(triclops_data.camchain.cam[1].camera_model, "pinhole");
  MU_CHECK_EQ(triclops_data.camchain.cam[2].camera_model, "pinhole");

  return 0;
}

int test_TriclopsDataset_loadCameraData() {
  TriclopsDataset triclops_data(TEST_DATA);
  int retval = triclops_data.loadCameraData();

  // Get timestamps and calculate relative time
  auto it = triclops_data.timeline.begin();
  auto it_end = triclops_data.timeline.end();
  while (it != it_end) {
    const long ts = it->first;
    triclops_data.timestamps.push_back(ts);

    // Advance to next non-duplicate entry.
    do {
      ++it;
    } while (ts == it->first);
  }

  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(10, triclops_data.cam0_data.timestamps.size());
  MU_CHECK_EQ(10, triclops_data.cam1_data.timestamps.size());
  MU_CHECK_EQ(10, triclops_data.timestamps.size());

  return 0;
}

int test_TriclopsDataset_loadIMUData() {
  TriclopsDataset triclops_data(TEST_DATA);
  int retval = triclops_data.loadIMUData();

  MU_CHECK_EQ(0, retval);

  return 0;
}


int test_TriclopsDataset_load() {
  TriclopsDataset triclops_data(TEST_DATA);

  int retval = triclops_data.load();
  // triclops_data.stereo_camera_cb = stereo_camera_cb;
  // triclops_data.run();

  MU_CHECK_EQ(0, retval);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_TriclopsDataset_constructor);
  MU_ADD_TEST(test_TriclopsDataset_loadCamchain);
  MU_ADD_TEST(test_TriclopsDataset_loadCameraData);
  MU_ADD_TEST(test_TriclopsDataset_loadIMUData);
  MU_ADD_TEST(test_TriclopsDataset_load);
}

} // namespace triclops
} // namespace prototype

MU_RUN_TESTS(prototype::triclops::test_suite);
