#include "CalibView.hpp"

namespace xyz {

CalibView::CalibView(std::shared_ptr<ceres::Problem> &problem,
                     const timestamp_t ts,
                     const std::map<int, CalibTargetPtr> &view_data,
                     std::map<int, CameraGeometryPtr> &camera_geometries,
                     std::map<int, Vec3> &target_points,
                     const Vec7 &pose)
    : problem_{problem}, ts_{ts}, view_data_{view_data},
      camera_geometries_{camera_geometries},
      target_points_{target_points}, pose_{pose} {
  // Add pose
  problem_->AddParameterBlock(pose_.data(), 7);
  problem_->SetManifold(pose_.data(), &pose_plus_);

  // Add camera residuals
  const Mat2 covar = I(2);
  for (const auto &[camera_index, calib_target] : view_data) {
    // Check if detected
    if (calib_target->detected() == false) {
      continue;
    }

    // Get calibration target measurements
    std::vector<int> point_ids;
    Vec2s keypoints;
    Vec3s object_points;
    calib_target->getMeasurements(point_ids, keypoints, object_points);

    // Add residual blocks
    for (size_t i = 0; i < point_ids.size(); i++) {
      // Add target point
      const int point_id = point_ids[i];
      Vec3 &pt = target_points[point_id];
      problem_->AddParameterBlock(pt.data(), 3);
      problem_->SetParameterBlockConstant(pt.data());

      // Create residual
      auto camera_geometry = camera_geometries_[camera_index];
      auto pose_ptr = pose_.data();
      auto pt_ptr = pt.data();
      auto kp = keypoints[i];
      auto resblock = CalibCameraError::create(camera_geometry,
                                               pose_ptr,
                                               pt_ptr,
                                               kp,
                                               covar);

      // Add residual block
      auto res_ptr = resblock.get();
      auto loss_ptr = nullptr;
      auto param_ptrs = resblock->getParamPtrs();
      auto res_id = problem_->AddResidualBlock(res_ptr, loss_ptr, param_ptrs);

      // Book keeping
      resblocks_[camera_index].push_back(resblock);
      resblock_ids_[camera_index].push_back(res_id);
    }
  }
}

int CalibView::numDetections() const {
  int num_detections = 0;
  for (const auto &[camera_index, camera_residuals] : resblocks_) {
    num_detections += camera_residuals.size();
  }
  return num_detections;
}

int CalibView::numDetections(const int camera_index) const {
  return resblocks_.at(camera_index).size();
}

std::vector<int> CalibView::getCameraIndices() const {
  std::vector<int> camera_indicies;
  for (const auto &[camera_index, camera_residuals] : resblocks_) {
    camera_indicies.push_back(camera_index);
  }
  return camera_indicies;
}

} // namespace xyz
