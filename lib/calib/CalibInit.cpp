#include "CalibInit.hpp"

namespace cartesian {

Mat4 CalibInit::initializeCameraImuExtrinsic(
    const Timeline &timeline,
    const std::map<int, std::shared_ptr<CameraGeometry>> &cam_geoms,
    const ImuParams &imu_params) {
  // Get cam0 calibration targets and imu data
  ImuBuffer imu_buffer;
  std::map<timestamp_t, std::shared_ptr<CalibTarget>> cam0_targets;
  for (const auto &ts : timeline.timestamps) {
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        const auto &cam_id = target_event->camera_id;
        if (cam_id == 0) {
          cam0_targets[ts] = target_event->calib_target;
        }
      }

      if (auto imu_event = dynamic_cast<ImuEvent *>(event)) {
        imu_buffer.add(ts, imu_event->acc, imu_event->gyr);
      }
    }
  }

  // Get camera poses
  const auto cam0_geom = cam_geoms.at(0);
  std::map<timestamp_t, Mat4> cam0_poses;
  for (const auto &[ts, target] : cam0_targets) {
    // Get calibration target measurements
    std::vector<int> point_ids;
    std::vector<int> corner_indicies;
    Vec2s keypoints;
    Vec3s object_points;
    target->getMeasurements(point_ids, keypoints, object_points);
    if (point_ids.size() < 10) {
      continue;
    }

    // Estimate relative pose T_CiTj
    Mat4 T_CiTj;
    SolvePnp pnp{cam0_geom};
    int status = pnp.estimate(keypoints, object_points, T_CiTj);
    if (status != 0) {
      continue;
    }

    // Add camera pose
    cam0_poses[ts] = T_CiTj.inverse();
  }

  // Form relative camera poses T_Ckm1_Ck
  std::vector<timestamp_t> cam0_timestamps;
  std::map<timestamp_t, Mat4> cam0_relposes;
  Mat4 T_TjCi_km1 = cam0_poses.begin()->second;
  for (const auto &[ts, T_TjCi_k] : cam0_poses) {
    cam0_timestamps.push_back(ts);
    cam0_relposes[ts] = T_TjCi_km1.inverse() * T_TjCi_k;
    T_TjCi_km1 = cam0_relposes[ts];
  }

  // Form relative imu poses T_Skm1_Sk
  std::vector<timestamp_t> imu0_timestamps;
  std::map<timestamp_t, Mat4> imu0_relposes;
  for (size_t k = 1; k < cam0_timestamps.size(); ++k) {
    const timestamp_t ts_km1 = cam0_timestamps[k - 1];
    const timestamp_t ts_k = cam0_timestamps[k];
    if (ts_km1 < imu_buffer.timestamps.front()) {
      continue;
    }

    const ImuBuffer &imu_data = imu_buffer.extract(ts_km1, ts_k);
    const ImuPreintegrate imu_pint{imu_params, imu_data};
    imu0_timestamps.push_back(ts_k);
    imu0_relposes[ts_k] = tf(imu_pint.dq, imu_pint.dr);
  }

  Mat3 R = SO3::solve_handeye(cam0_relposes, imu0_relposes);
  Vec3 t{0.0, 0.0, 0.0};

  return tf(R, t);
}

} // namespace cartesian
