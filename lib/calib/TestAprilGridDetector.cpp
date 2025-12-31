#include <gtest/gtest.h>

#include "calib/AprilGrid.hpp"
#include "calib/AprilGridConfig.hpp"
#include "calib/AprilGridDetector.hpp"

#define TEST_IMAGE TEST_DATA "/aprilgrid/aprilgrid.png"

namespace cartesian {

TEST(AprilGridDetector, detect) {
  const int target_id = 0;
  const int tag_rows = 6;
  const int tag_cols = 6;
  const double tag_size = 0.088;
  const double tag_spacing = 0.3;
  AprilGridConfig config{target_id, tag_rows, tag_cols, tag_size, tag_spacing};
  AprilGridDetector detector(config);

  const timestamp_t ts = 0;
  const int camera_id = 0;
  const auto img = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
  auto grids = detector.detect(ts, camera_id, img);
  ASSERT_EQ(grids.size(), 1);

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  grids[0]->getMeasurements(tag_ids, corner_indicies, keypoints, object_points);
  ASSERT_EQ(tag_ids.size(), tag_rows * tag_cols * 4);
  ASSERT_EQ(corner_indicies.size(), tag_rows * tag_cols * 4);
  ASSERT_EQ(keypoints.size(), tag_rows * tag_cols * 4);
  ASSERT_EQ(object_points.size(), tag_rows * tag_cols * 4);
}

} // namespace cartesian
