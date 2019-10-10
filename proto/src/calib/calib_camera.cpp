#include "proto/calib/calib_camera.hpp"

namespace proto {

static int process_aprilgrid(const aprilgrid_t &aprilgrid,
                             double *intrinsics,
                             double *distortion,
                             calib_pose_param_t *pose,
                             ceres::Problem &problem) {
  for (const auto &tag_id : aprilgrid.ids) {
    // Get keypoints
    vec2s_t keypoints;
    if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }

    // Get object points
    vec3s_t object_points;
    if (aprilgrid_object_points(aprilgrid, tag_id, object_points) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object points!");
      return -1;
    }

    // Form residual block
    for (size_t i = 0; i < 4; i++) {
      const auto kp = keypoints[i];
      const auto obj_pt = object_points[i];

      const auto residual = new pinhole_radtan4_residual_t{kp, obj_pt};
      const auto cost_func =
          new ceres::AutoDiffCostFunction<pinhole_radtan4_residual_t,
                                          2, // Size of: residual
                                          4, // Size of: intrinsics
                                          4, // Size of: distortion
                                          4, // Size of: q_CF
                                          3  // Size of: r_CF
                                          >(residual);

      // const auto cost_func = new intrinsics_residual_t{kp, obj_pt};
      problem.AddResidualBlock(cost_func, // Cost function
                               NULL,      // Loss function
                               intrinsics,
                               distortion,
                               pose->q.coeffs().data(),
                               pose->r.data());
    }
  }

  return 0;
}

static int save_results(const std::string &save_path,
                        const vec2_t &resolution,
                        const pinhole_t &pinhole,
                        const radtan4_t &radtan) {
  std::ofstream outfile(save_path);

  // Check if file is ok
  if (outfile.good() != true) {
    return -1;
  }

  // Save results
  const std::string indent = "  ";
  {
    const std::string res = vec2str(resolution);
    const std::string intrinsics = arr2str(*pinhole.data, 4);
    const std::string distortion = arr2str(*radtan.data, 4);
    outfile << "cam0:" << std::endl;
    outfile << indent << "camera_model: \"pinhole\"" << std::endl;
    outfile << indent << "distortion_model: \"radtan\"" << std::endl;
    outfile << indent << "resolution: " << res << std::endl;
    outfile << indent << "intrinsics: " << intrinsics << std::endl;
    outfile << indent << "distortion: " << distortion << std::endl;
    outfile << std::endl;
  }

  // Finsh up
  outfile.close();

  return 0;
}

mat4s_t calib_generate_poses(const calib_target_t &target) {
  const double target_width = (target.tag_rows - 1.0) * target.tag_size;
  const double target_height = (target.tag_cols - 1.0) * target.tag_size;
  const vec3_t target_center{target_width / 2.0, target_height / 2.0, 0.0};

  // Pose settings
  const auto x_range = linspace(-0.1, 0.1, 5);
  const auto y_range = linspace(-0.1, 0.1, 5);
  const auto z_range = linspace(0.3, 0.5, 5);

  // Generate camera positions infrom of the AprilGrid target in the target
  // frame, r_TC.
  vec3s_t cam_positions;
  for (const auto &x : x_range) {
    for (const auto &y : y_range) {
      for (const auto &z : z_range) {
        const vec3_t r_TC = vec3_t{x, y, z} + target_center;
        cam_positions.push_back(r_TC);
      }
    }
  }

  // For each position create a camera pose that "looks at" the AprilGrid
  // center in the target frame, T_TC.
  mat4s_t poses;
  for (const auto cam_position : cam_positions) {
    mat4_t T_TC = lookat(cam_position, target_center);

    // Perturb rotation
    mat3_t C_TC = tf_rot(T_TC);
    const vec3_t rpy{randf(-0.4, 0.4), randf(-0.4, 0.4), randf(-0.4, 0.4)};
    const mat3_t C_perturb = euler321(rpy);
    C_TC = C_perturb * C_TC;
    T_TC.block(0, 0, 3, 3) = C_TC;

    // mat4_t T_TC = tf(I(3), cam_position);
    poses.push_back(T_TC);
  }

  return poses;
}

int calib_camera_solve(const aprilgrids_t &aprilgrids,
                       pinhole_t &pinhole,
                       radtan4_t &radtan,
                       mat4s_t &T_CF) {
  // Optimization variables
  std::vector<calib_pose_param_t> T_CF_params;
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    T_CF_params.emplace_back(aprilgrids[i].T_CF);
  }

  // Setup optimization problem
  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);
  ceres::EigenQuaternionParameterization quaternion_parameterization;

  // Process all aprilgrid data
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    int retval = process_aprilgrid(aprilgrids[i],
                                   *pinhole.data,
                                   *radtan.data,
                                   &T_CF_params[i],
                                   problem);
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
    problem.SetParameterization(T_CF_params[i].q.coeffs().data(),
                                &quaternion_parameterization);
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  // options.check_gradients = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  // Clean up
  T_CF.clear();
  for (auto pose_param : T_CF_params) {
    T_CF.emplace_back(tf(pose_param.q, pose_param.r));
  }

  return 0;
}

int calib_camera_solve(const std::string &config_file) {
  // Calibration config data
  std::string data_path;
  std::string results_path;
  bool imshow = false;
  vec2_t resolution{0.0, 0.0};
  double lens_hfov = 0.0;
  double lens_vfov = 0.0;
  std::string camera_model;
  std::string distortion_model;

  // Parse calib config file
  config_t config{config_file};
  parse(config, "settings.data_path", data_path);
  parse(config, "settings.results_path", results_path);
  parse(config, "settings.imshow", imshow, true);
  parse(config, "cam0.resolution", resolution);
  parse(config, "cam0.lens_hfov", lens_hfov);
  parse(config, "cam0.lens_vfov", lens_vfov);
  parse(config, "cam0.camera_model", camera_model);
  parse(config, "cam0.distortion_model", distortion_model);

  // Load calibration target
  calib_target_t calib_target;
  if (calib_target_load(calib_target, config_file, "calib_target") != 0) {
    LOG_ERROR("Failed to load calib target in [%s]!", config_file.c_str());
    return -1;
  }

  // Prepare aprilgrid data directory
  const auto grid_data_path = data_path + "/grid0/data";
  if (dir_exists(grid_data_path) == false) {
    dir_create(grid_data_path);
  }

  // Preprocess calibration data
  const auto cam_data_path = data_path + "/cam0/data";
  int retval = preprocess_camera_data(calib_target,
                                      cam_data_path,
                                      resolution,
                                      lens_hfov,
                                      lens_vfov,
                                      grid_data_path,
                                      imshow);
  if (retval != 0) {
    LOG_ERROR("Failed to preprocess calibration data!");
    return -1;
  }

  // Load calibration data
  aprilgrids_t grids;
  timestamps_t timestamps;
  retval = load_camera_calib_data(grid_data_path, grids, timestamps);
  if (retval != 0) {
    LOG_ERROR("Failed to load camera calibration data!");
    return -1;
  }

  // Setup initial camera intrinsics and distortion for optimization
  const double image_width = resolution(0);
  const double image_height = resolution(1);
  const double fx = pinhole_focal_length(image_width, lens_hfov);
  const double fy = pinhole_focal_length(image_height, lens_vfov);
  const double cx = resolution(0) / 2.0;
  const double cy = resolution(1) / 2.0;
  pinhole_t pinhole{fx, fy, cx, cy};
  radtan4_t radtan{0.01, 0.0001, 0.0001, 0.0001};

  // Calibrate camera
  LOG_INFO("Calibrating camera!");
  mat4s_t T_CF;
  if (calib_camera_solve(grids, pinhole, radtan, T_CF) != 0) {
    LOG_ERROR("Failed to calibrate camera data!");
    return -1;
  }

  // Show results
  std::cout << "Optimization results:" << std::endl;
  std::cout << pinhole << std::endl;
  std::cout << radtan << std::endl;
  calib_camera_stats<pinhole_radtan4_residual_t>(
    grids,
    *pinhole.data,
    *radtan.data,
    T_CF,
    ""
  );

  // Save results
  printf("\x1B[92mSaving optimization results to [%s]\033[0m\n",
         results_path.c_str());
  if (save_results(results_path, resolution, pinhole, radtan) != 0) {
    LOG_ERROR("Failed to save results to [%s]!", results_path.c_str());
    return -1;
  }


  return 0;
}

} //  namespace proto
