#pragma once

#include <string>
#include <algorithm>
#include <set>
#include <unordered_map>

#include "../core/Core.hpp"
#include "../camera/CameraModel.hpp"
#include "CalibTarget.hpp"
#include "AprilGridConfig.hpp"

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

/** AprilGrid */
class AprilGrid : public CalibTarget {
private:
  AprilGridConfig config_;

  // Data
  struct TagDetection {
    std::set<int> corner_indicies;
    std::unordered_map<int, Vec2> keypoints;
  };
  std::unordered_map<int, TagDetection> data_;

public:
  AprilGrid(const timestamp_t &timestamp,
            const int camera_id,
            const AprilGridConfig &config);
  AprilGrid(const AprilGrid &src);
  ~AprilGrid() = default;

  /** Check if detected */
  bool detected() const override;

  /** Return number detected */
  int getNumDetected() const override;

  /** Return AprilGrid config */
  AprilGridConfig getConfig() const;

  /** Return number of tag rows */
  int getTagRows() const;

  /** Return number of tag cols */
  int getTagCols() const;

  /** Return number of tag size */
  double getTagSize() const;

  /** Return number of tag spacing */
  double getTagSpacing() const;

  /** Return tag id offset */
  int getTagIdOffset() const;

  /** Calculate width and height */
  Vec2 getWidthHeight() const;

  /** Calculate center */
  Vec2 getCenter() const;

  /** Calculate AprilGrid tag index based on tag id */
  void getGridIndex(const int tag_id, int &i, int &j) const;

  /** Calculate object point based on tag id and corner index */
  Vec3 getObjectPoint(const int tag_id, const int corner_index) const;

  /** Calculate object point based on point id */
  Vec3 getObjectPoint(const int point_id) const;

  /** Get the 2D center of the AprilGrid */
  Vec2 getCenter2d() const;

  /** Get the 3D center of the AprilGrid */
  Vec3 getCenter3d() const;

  /** Get measurements */
  void getMeasurements(std::vector<int> &tag_ids,
                       std::vector<int> &corner_indicies,
                       Vec2s &keypoints,
                       Vec3s &object_points) const;

  /** Get measurements */
  void getMeasurements(std::vector<int> &point_ids,
                       Vec2s &keypoints,
                       Vec3s &object_points) const override;

  /** Check to see if AprilGrid has specific tag id and corner index */
  bool has(const int tag_id, const int corner_index) const;

  /** Add measurmeent */
  void add(const int tag_id, const int corner_index, const Vec2 &kp);

  /** Remove measurmeent */
  void remove(const int tag_id, const int corner_index);

  /** Remove measurmeent */
  void remove(const int tag_id);

  /** Save AprilGrid **/
  int save(const std::string &save_path) const;

  /** Load AprilGrid **/
  static std::shared_ptr<AprilGrid> load(const std::string &data_path);

  /** Load CalibTargets **/
  static std::vector<std::shared_ptr<CalibTarget>>
  loadDirectory(const std::string &dir_path);

  /** Draw AprilGrid */
  cv::Mat draw(const cv::Mat &image,
               const int marker_size = 2,
               const cv::Scalar &color = cv::Scalar{0, 0, 255}) const;

  /** Imshow AprilGrid */
  void imshow(const std::string &title, const cv::Mat &image) const;
};

} // namespace xyz
