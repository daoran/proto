#pragma once

#include <string>
#include <algorithm>
#include <set>
#include <unordered_map>

#include "../Core.hpp"
#include "../camera/CameraModel.hpp"

#include "CalibTarget.hpp"

// AprilTags3 by Ed Olsen
extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/common/getopt.h"
}

// AprilTags by Michael Kaess
#include "ethz_apriltag/TagDetector.h"
#include "ethz_apriltag/Tag36h11.h"

namespace xyz {

namespace internal {

/** Grid Detector */
class GridDetector : public ethz_apriltag::TagDetector {
public:
  GridDetector() : TagDetector(ethz_apriltag::tagCodes36h11) {
    thisTagFamily.blackBorder = 2;
  }
};

} // namespace internal

/** AprilGrid */
class AprilGrid : public CalibTarget {
public:
  AprilGrid(const timestamp_t &timestamp,
            const int tag_rows,
            const int tag_cols,
            const double tag_size,
            const double tag_spacing);
  AprilGrid(const AprilGrid &src);
  ~AprilGrid() = default;

  /** Calculate width and height */
  Vec2 getWidthHeight() const;

  /** Calculate center */
  Vec2 getCenter() const;

  /** Calculate AprilGrid tag index based on tag ID */
  void getGridIndex(const int tag_id, int &i, int &j) const;

  /** Calculate object point based on tag ID and corner index */
  Vec3 getObjectPoint(const int tag_id, const int corner_index) const;

  /** Get measurements */
  void getMeasurements(std::vector<int> &tag_ids,
                       std::vector<int> &corner_indicies,
                       Vec2s &keypoints,
                       Vec3s &object_points) const override;

  /** Save AprilGrid **/
  int save(const std::string &save_path) const;

  /** Load AprilGrid **/
  static std::shared_ptr<AprilGrid> load(const std::string &data_path);

  /** Load CalibTargets **/
  static std::vector<std::shared_ptr<CalibTarget>>
  loadDirectory(const std::string &dir_path);
};

/** AprilGrid Detector **/
class AprilGridDetector {
private:
  int tag_rows_ = 0;
  int tag_cols_ = 0;
  double tag_size_ = 0.0;
  double tag_spacing_ = 0.0;
  int min_border_dist_ = 5;

  // Ed Olsen's AprilTag detector
  apriltag_family_t *tf_ = tag36h11_create();
  apriltag_detector_t *det_ = apriltag_detector_create();

  void olsenDetect(const cv::Mat &image,
                   std::vector<int> &tag_ids,
                   std::vector<int> &corner_indicies,
                   std::vector<Vec2> &keypoints);

  // Michale Kaess's AprilTag detector
  internal::GridDetector detector;

  void kaessDetect(const cv::Mat &image,
                   std::vector<int> &tag_ids,
                   std::vector<int> &corner_indicies,
                   std::vector<Vec2> &keypoints);

public:
  AprilGridDetector(const int tag_rows,
                    const int tag_cols,
                    const double tag_size,
                    const double tag_spacing);

  virtual ~AprilGridDetector();

  /** Detect AprilGrid **/
  std::shared_ptr<AprilGrid> detect(const timestamp_t ts, const cv::Mat &image);
};

} // namespace xyz
