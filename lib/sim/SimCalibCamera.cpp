#include "SimCalibCamera.hpp"

namespace xyz {

SimCalibCamera::SimCalibCamera() {
  setup_calib_targets();
  setup_camera_geometries();
  setup_camera_poses();
  simulate_camera_views();
}

void SimCalibCamera::setup_calib_targets() {
  // Add target config
  const int target_id = 0;
  const int tag_rows = 8;
  const int tag_cols = 10;
  const double tag_size = 0.08;
  const double tag_spacing = 0.3;
  const int tag_id_offset = 0;
  const AprilGridConfig target_config{
      target_id,
      tag_rows,
      tag_cols,
      tag_size,
      tag_spacing,
      tag_id_offset,
  };
  target_configs.emplace(target_id, target_config);

  // Add target pose
  const auto target_center = target_config.getCenter();
  const Vec3 target_pos{1.0, target_center.x(), 0.0};
  const Vec3 target_euler{M_PI / 2.0, 0.0, -M_PI / 2.0};
  const Mat3 target_rot = euler321(target_euler);
  const Mat4 T_WTj = tf(target_rot, target_pos);
  target_poses.emplace(0, T_WTj);
}

void SimCalibCamera::setup_camera_geometries() {
  const std::string cam_model = "BrownConrady4";
  const Vec2i cam_res{640, 480};
  const double fx = pinhole_focal(cam_res[0], 90.0);
  const double fy = pinhole_focal(cam_res[0], 90.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const double k1 = 0.1;
  const double k2 = 0.01;
  const double p1 = 0.001;
  const double p2 = 0.001;

  // cam0
  // -- Intrinsic
  VecX cam0_int;
  cam0_int.resize(8);
  cam0_int << fx, fy, cx, cy, k1, k2, p1, p2;
  // -- Extrinsic
  const Vec3 cam0_ext_pos{0.0, 0.0, 0.0};
  const Vec3 cam0_ext_euler{0.0, 0.0, 0.0};
  const Mat3 cam0_ext_rot = euler321(cam0_ext_euler);
  const Vec7 cam0_ext = tf_vec(tf(cam0_ext_rot, cam0_ext_pos));
  // -- Camera geometry
  cameras.emplace(0, CameraGeometry(0, cam_model, cam_res, cam0_int, cam0_ext));

  // cam1
  // -- Intrinsic
  VecX cam1_int;
  cam1_int.resize(8);
  cam1_int << fx, fy, cx, cy, k1, k2, p1, p2;
  // -- Extrinsic
  const Vec3 cam1_ext_pos{0.05, 0.0, 0.0};
  const Vec3 cam1_ext_euler{0.0, 0.0, 0.0};
  const Mat3 cam1_ext_rot = euler321(cam1_ext_euler);
  const Vec7 cam1_ext = tf_vec(tf(cam1_ext_rot, cam1_ext_pos));
  // -- Camera geometry
  cameras.emplace(1, CameraGeometry(1, cam_model, cam_res, cam1_int, cam1_ext));
}

void SimCalibCamera::setup_camera_poses() {
  const auto half_x = sample_x / 2.0;
  const auto half_y = sample_y / 2.0;
  const auto half_z = sample_z / 2.0;
  const auto range_x = linspace(-half_x, half_x, sample_num_x);
  const auto range_y = linspace(-half_y, half_y, sample_num_y);
  const auto range_z = linspace(-half_z, half_z, sample_num_z);

  const auto target = target_configs.at(0);
  const Mat4 T_WT0 = target_poses.at(0);
  const Vec2 center = target.getCenter();
  const Vec3 p_center = tf_point(T_WT0, Vec3{center.x(), center.y(), 0.0});

  // const time_t time_now = time(NULL);
  // timestamp_t ts = time_now * 1e9;
  timestamp_t ts = 0;
  timestamp_t ts_diff = (1.0 / camera_rate) * 1e9;

  for (const auto y : range_y) {
    for (const auto z : range_z) {
      for (const auto x : range_x) {
        // Transform camera position in target frame to world frame
        const Vec3 p_T{x + center.x(), y + center.y(), z + sample_z_offset};
        const Vec3 p_W = tf_point(T_WT0, p_T);

        // Form camera rotation
        // clang-format off
        const Vec3 world_up{0.0, 0.0, 1.0};
        const Vec3 dir = (p_center - p_W).normalized();
        const Vec3 right= dir.cross(world_up).normalized();
        const Vec3 up = dir.cross(right);
        Mat3 R;
        R << right.x(), up.x(), dir.x(),
             right.y(), up.y(), dir.y(),
             right.z(), up.z(), dir.z();
        // clang-format on

        // Form pose transform
        Mat4 T_WC0 = I(4);
        T_WC0.block<3, 3>(0, 0) = R;
        T_WC0.block<3, 1>(0, 3) = p_W;

        camera_poses[ts] = T_WC0;
        ts += ts_diff;
      }
    }
  }
}

void SimCalibCamera::simulate_camera_views() {
  // Simulate single camera view
  auto sim_view = [&](const timestamp_t ts,
                      const int camera_id,
                      const int target_id,
                      const Mat4 T_WC0) {
    const auto res = cameras.at(camera_id).getResolution();
    const auto intrinsic = cameras.at(camera_id).getIntrinsic();
    const auto T_C0Ci = tf(cameras.at(camera_id).getExtrinsic());
    const auto camera = cameras.at(camera_id).getCameraModel();
    const auto T_WT = target_poses[target_id];
    const auto T_CiF = T_C0Ci.inverse() * T_WC0.inverse() * T_WT;

    const AprilGridConfig &config = target_configs[target_id];
    auto target = std::make_shared<AprilGrid>(ts, camera_id, config);

    for (int tag_id = 0; tag_id < config.getNumTags(); ++tag_id) {
      for (int corner_index = 0; corner_index < 4; ++corner_index) {
        const Vec3 p_F = config.getObjectPoint(tag_id, corner_index);
        const Vec3 p_Ci = tf_point(T_CiF, p_F);
        Vec2 z{0.0, 0.0};
        if (camera->project(res, intrinsic, p_Ci, z) != 0) {
          continue;
        }
        target->add(tag_id, corner_index, z);
      }
    }

    return target;
  };

  // Simulate camera views
  for (const auto &[ts, T_WC0] : camera_poses) {
    for (const auto &[camera_id, _] : cameras) {
      for (const auto &[target_id, _] : target_configs) {
        const auto target = sim_view(ts, camera_id, target_id, T_WC0);
        camera_views[ts][camera_id][target_id] = target;
      }
    }
  }
}

Timeline SimCalibCamera::get_timeline() const {
  Timeline timeline;

  for (const auto &[ts, camera_map] : camera_views) {
    for (const auto &[camera_id, target_map] : camera_map) {
      for (const auto &[target_id, target] : target_map) {
        timeline.add(ts, camera_id, target);
      }
    }
  }

  return timeline;
}

int SimCalibCamera::save_target_configs(const fs::path &yaml_path) const {
  const auto fp = fopen(yaml_path.c_str(), "w");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s] for saving!", yaml_path.c_str());
    return -1;
  }

  for (const auto &[target_id, config] : target_configs) {
    fprintf(fp, "target%d:\n", target_id);
    fprintf(fp, "  tag_rows: %d\n", config.tag_rows);
    fprintf(fp, "  tag_cols: %d\n", config.tag_cols);
    fprintf(fp, "  tag_size: %f\n", config.tag_size);
    fprintf(fp, "  tag_spacing: %f\n", config.tag_spacing);
    fprintf(fp, "  tag_id_offset: %d\n", config.tag_id_offset);
    fprintf(fp, "\n");
  }
  fclose(fp);

  return 0;
}

int SimCalibCamera::save_target_poses(const fs::path &csv_path) const {
  const auto fp = fopen(csv_path.c_str(), "w");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s] for saving!", csv_path.c_str());
    return -1;
  }

  fprintf(fp, "#target_id, x, y, z, qx, qy, qz, qw\n");
  for (const auto &[target_id, T_WTj] : target_poses) {
    const auto pos = tf_trans(T_WTj);
    const auto quat = tf_quat(T_WTj);
    fprintf(fp, "%d, ", target_id);
    fprintf(fp, "%f, %f, %f, ", pos.x(), pos.y(), pos.z());
    fprintf(fp, "%f, %f, %f, %f\n", quat.x(), quat.y(), quat.z(), quat.w());
  }
  fclose(fp);

  return 0;
}

int SimCalibCamera::save_camera_geometries(const fs::path &yaml_path) const {
  const auto fp = fopen(yaml_path.c_str(), "w");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s] for saving!", yaml_path.c_str());
    return -1;
  }

  for (const auto &[camera_id, camera] : cameras) {
    const auto res = camera.getResolution();
    const auto camera_model = camera.getCameraModelString();
    fprintf(fp, "camera%d:\n", camera_id);
    fprintf(fp, "  camera_model: \"%s\"\n", camera_model.c_str());
    fprintf(fp, "  resolution:   [%d, %d]\n", res.x(), res.y());
    fprintf(fp, "  intrinsic:    %s\n", vec2str(camera.getIntrinsic()).c_str());
    fprintf(fp, "  extrinsic:    %s\n", vec2str(camera.getExtrinsic()).c_str());
    fprintf(fp, "\n");
  }
  fclose(fp);

  return 0;
}

int SimCalibCamera::save_camera_poses(const fs::path &csv_path) const {
  const auto fp = fopen(csv_path.c_str(), "w");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s] for saving!", csv_path.c_str());
    return -1;
  }

  fprintf(fp, "#ts, x, y, z, qx, qy, qz, qw\n");
  for (const auto &[ts, T_WC0] : camera_poses) {
    const auto pos = tf_trans(T_WC0);
    const auto quat = tf_quat(T_WC0);
    fprintf(fp, "%ld, ", ts);
    fprintf(fp, "%f, %f, %f, ", pos.x(), pos.y(), pos.z());
    fprintf(fp, "%f, %f, %f, %f\n", quat.x(), quat.y(), quat.z(), quat.w());
  }
  fclose(fp);

  return 0;
}

int SimCalibCamera::save_camera_views(const fs::path &save_dir) const {
  // Create target directories
  for (const auto &[target_id, _] : target_configs) {
    const fs::path &target_str = "target" + std::to_string(target_id);
    const fs::path &target_dir = save_dir / target_str;
    if (dir_create(target_dir) != 0) {
      LOG_ERROR("Could not create dir [%s]!", target_dir.c_str());
      return -1;
    }
  }

  // Save camera views
  for (const auto &[ts, camera_map] : camera_views) {
    for (const auto &[camera_id, target_map] : camera_map) {
      for (const auto &[target_id, target] : target_map) {
        const fs::path &target_str = "target" + std::to_string(target_id);
        const fs::path &camera_str = "cam" + std::to_string(camera_id);
        const fs::path &fname = std::to_string(ts) + ".csv";
        const fs::path &save_path = save_dir / target_str / camera_str / fname;
        target->save(save_path);
      }
    }
  }

  return 0;
}

int SimCalibCamera::save(const fs::path &save_dir) const {
  // Check save dir
  if (dir_create(save_dir) != 0) {
    LOG_ERROR("Could not create dir [%s]!", save_dir.c_str());
    return -1;
  }

  // Save
  const auto target_configs_path = save_dir / "target_configs.yaml";
  const auto target_poses_path = save_dir / "target_poses.csv";
  const auto camera_geometries_path = save_dir / "camera_geometries.yaml";
  const auto camera_poses_path = save_dir / "camera_poses.csv";

  save_target_configs(target_configs_path);
  save_target_poses(target_poses_path);
  save_camera_geometries(camera_geometries_path);
  save_camera_poses(camera_poses_path);
  save_camera_views(save_dir);

  return 0;
}

} // namespace xyz
