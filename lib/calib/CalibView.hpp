#pragma once

#include "CalibData.hpp"
#include "../ImuError.hpp"
#include "../ReprojectionError.hpp"

namespace xyz {

/** Calibration view */
class CalibView {
private:
  std::shared_ptr<ceres::Problem> &problem_;
  PoseManifold pose_plus_;
  timestamp_t ts_ = 0;
  const std::map<int, CalibTargetPtr> view_data_;
  std::map<int, CameraGeometryPtr> &camera_geometries_;
  std::map<int, Vec3> &target_points_;
  Vec7 pose_;

  std::map<int, std::vector<std::shared_ptr<ReprojectionError>>> resblocks_;
  std::map<int, std::vector<ceres::ResidualBlockId>> resblock_ids_;

public:
  CalibView() = delete;

  CalibView(std::shared_ptr<ceres::Problem> &problem,
            const timestamp_t ts,
            const std::map<int, CalibTargetPtr> &view_data,
            std::map<int, CameraGeometryPtr> &camera_geometries,
            std::map<int, Vec3> &target_points,
            const Vec7 &pose);

  virtual ~CalibView() = default;

  /** Get number of detections */
  int numDetections() const;

  /** Get number of detections by camera */
  int numDetections(const int camera_index) const;

  /** Get camera indicies */
  std::vector<int> getCameraIndices() const;
};

} // namespace xyz
