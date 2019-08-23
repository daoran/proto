#include "proto/calib/calib_stereo.hpp"

namespace proto {

static int process_aprilgrid(const aprilgrid_t &cam0_aprilgrid,
                             const aprilgrid_t &cam1_aprilgrid,
                             double *cam0_intrinsics,
                             double *cam0_distortion,
                             double *cam1_intrinsics,
                             double *cam1_distortion,
                             calib_pose_param_t *T_C0C1,
                             calib_pose_param_t *T_C0F,
                             ceres::Problem *problem) {
  for (const auto &tag_id : cam0_aprilgrid.ids) {
    // Get keypoints
    vec2s_t cam0_keypoints;
    if (aprilgrid_get(cam0_aprilgrid, tag_id, cam0_keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }
    vec2s_t cam1_keypoints;
    if (aprilgrid_get(cam1_aprilgrid, tag_id, cam1_keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }

    // Get object points
    vec3s_t object_points;
    if (aprilgrid_object_points(cam0_aprilgrid, tag_id, object_points) != 0) {
      LOG_ERROR("Failed to calculate AprilGrid object points!");
      return -1;
    }

    // Form residual block
    for (size_t i = 0; i < 4; i++) {
      const auto kp0 = cam0_keypoints[i];
      const auto kp1 = cam1_keypoints[i];
      const auto obj_pt = object_points[i];
      const auto residual = new stereo_residual_t{kp0, kp1, obj_pt};

      const auto cost_func =
          new ceres::AutoDiffCostFunction<stereo_residual_t,
                                          4, // Size of: residual
                                          4, // Size of: cam0_intrinsics
                                          4, // Size of: cam0_distortion
                                          4, // Size of: cam1_intrinsics
                                          4, // Size of: cam1_distortion
                                          4, // Size of: q_C0C1
                                          3, // Size of: t_C0C1
                                          4, // Size of: q_C0F
                                          3  // Size of: t_C0F
                                          >(residual);

      problem->AddResidualBlock(cost_func, // Cost function
                                NULL,      // Loss function
                                cam0_intrinsics,
                                cam0_distortion,
                                cam1_intrinsics,
                                cam1_distortion,
                                T_C0C1->q.coeffs().data(),
                                T_C0C1->r.data(),
                                T_C0F->q.coeffs().data(),
                                T_C0F->r.data());
    }
  }

  return 0;
}

static int save_results(const std::string &save_path,
                        const vec2_t &cam0_resolution,
                        const vec2_t &cam1_resolution,
                        const pinhole_t &cam0_pinhole,
                        const radtan4_t &cam0_radtan,
                        const pinhole_t &cam1_pinhole,
                        const radtan4_t &cam1_radtan,
                        const mat4_t &T_C0C1) {
  std::ofstream outfile(save_path);
  const std::string indent = "  ";

  // Check if file is ok
  if (outfile.good() != true) {
    return -1;
  }

  // Save cam0 results
  {
    const std::string resolution = vec2str(cam0_resolution);
    const std::string intrinsics = arr2str(*cam0_pinhole.data, 4);
    const std::string distortion = arr2str(*cam0_radtan.data, 4);
    outfile << "cam0:" << std::endl;
    outfile << indent << "camera_model: \"pinhole\"" << std::endl;
    outfile << indent << "distortion_model: \"radtan\"" << std::endl;
    outfile << indent << "resolution: " << resolution << std::endl;
    outfile << indent << "intrinsics: " << intrinsics << std::endl;
    outfile << indent << "distortion: " << distortion << std::endl;
    outfile << std::endl;
  }

  // Save cam1 results
  {
    const std::string resolution = vec2str(cam1_resolution);
    const std::string intrinsics = arr2str(*cam1_pinhole.data, 4);
    const std::string distortion = arr2str(*cam1_radtan.data, 4);
    outfile << "cam1:" << std::endl;
    outfile << indent << "camera_model: \"pinhole\"" << std::endl;
    outfile << indent << "distortion_model: \"radtan\"" << std::endl;
    outfile << indent << "resolution: " << resolution << std::endl;
    outfile << indent << "intrinsics: " << intrinsics << std::endl;
    outfile << indent << "distortion: " << distortion << std::endl;
    outfile << std::endl;
  }

  // Save camera extrinsics
  outfile << "T_C0C1:" << std::endl;
  outfile << indent << "rows: 4" << std::endl;
  outfile << indent << "cols: 4" << std::endl;
  outfile << indent << "data: [" << std::endl;
  outfile << mat2str(T_C0C1, indent + indent) << std::endl;
  outfile << indent << "]" << std::endl;
  outfile << std::endl;

  // Finsh up
  outfile.close();

  return 0;
}

int calib_stereo_solve(const std::vector<aprilgrid_t> &cam0_aprilgrids,
                       const std::vector<aprilgrid_t> &cam1_aprilgrids,
                       pinhole_t &cam0_pinhole,
                       radtan4_t &cam0_radtan,
                       pinhole_t &cam1_pinhole,
                       radtan4_t &cam1_radtan,
                       mat4_t &T_C0C1,
                       mat4s_t &T_C0F) {
  assert(cam0_aprilgrids.size() == cam1_aprilgrids.size());

  // Optimization variables
  calib_pose_param_t extrinsic_param{T_C0C1};
  std::vector<calib_pose_param_t> pose_params;
  for (size_t i = 0; i < cam0_aprilgrids.size(); i++) {
    pose_params.emplace_back(cam0_aprilgrids[i].T_CF);
  }

  // Setup optimization problem
  // clang-format off
  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  std::unique_ptr<ceres::Problem> problem(new ceres::Problem(problem_options));
  ceres::EigenQuaternionParameterization quaternion_parameterization;
  // clang-format on

  // Process all aprilgrid data
  for (size_t i = 0; i < cam0_aprilgrids.size(); i++) {
    int retval = process_aprilgrid(cam0_aprilgrids[i],
                                   cam1_aprilgrids[i],
                                   *cam0_pinhole.data,
                                   *cam0_radtan.data,
                                   *cam1_pinhole.data,
                                   *cam1_radtan.data,
                                   &extrinsic_param,
                                   &pose_params[i],
                                   problem.get());
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }

    problem->SetParameterization(pose_params[i].q.coeffs().data(),
                                 &quaternion_parameterization);
  }
  problem->SetParameterization(extrinsic_param.q.coeffs().data(),
                               &quaternion_parameterization);

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  std::cout << summary.FullReport() << std::endl;

  // Finish up
  T_C0C1 = tf(extrinsic_param.q.toRotationMatrix(), extrinsic_param.r);
  for (auto pose_param : pose_params) {
    T_C0F.emplace_back(tf(pose_param.q, pose_param.r));
  }

  return 0;
}

int calib_stereo_solve(const std::string &config_file) {
  // Calibration config
  struct calib_config_t {
    std::string cam0_image_path;
    std::string cam1_image_path;
    std::string cam0_preprocess_path;
    std::string cam1_preprocess_path;
    std::string results_file;

    vec2_t cam0_resolution{0.0, 0.0};
    double cam0_lens_hfov = 0.0;
    double cam0_lens_vfov = 0.0;
    std::string cam0_camera_model;
    std::string cam0_distortion_model;

    vec2_t cam1_resolution{0.0, 0.0};
    double cam1_lens_hfov = 0.0;
    double cam1_lens_vfov = 0.0;
    std::string cam1_camera_model;
    std::string cam1_distortion_model;
  };

  // Parse calibration config
  config_t config{config_file};
  calib_config_t calib;

  parse(config, "settings.cam0_image_path", calib.cam0_image_path);
  parse(config, "settings.cam1_image_path", calib.cam1_image_path);
  parse(config, "settings.cam0_preprocess_path", calib.cam0_preprocess_path);
  parse(config, "settings.cam1_preprocess_path", calib.cam1_preprocess_path);
  parse(config, "settings.results_file", calib.results_file);

  parse(config, "cam0.resolution", calib.cam0_resolution);
  parse(config, "cam0.lens_hfov", calib.cam0_lens_hfov);
  parse(config, "cam0.lens_vfov", calib.cam0_lens_vfov);
  parse(config, "cam0.camera_model", calib.cam0_camera_model);
  parse(config, "cam0.distortion_model", calib.cam0_distortion_model);

  parse(config, "cam1.resolution", calib.cam1_resolution);
  parse(config, "cam1.lens_hfov", calib.cam1_lens_hfov);
  parse(config, "cam1.lens_vfov", calib.cam1_lens_vfov);
  parse(config, "cam1.camera_model", calib.cam1_camera_model);
  parse(config, "cam1.distortion_model", calib.cam1_distortion_model);

  // Load calibration target
  calib_target_t calib_target;
  if (calib_target_load(calib_target, config_file, "calib_target") != 0) {
    LOG_ERROR("Failed to load calib target [%s]!", config_file.c_str());
    return -1;
  }

  // Preprocess calibration data
  int retval = preprocess_stereo_data(calib_target,
                                      calib.cam0_image_path,
                                      calib.cam1_image_path,
                                      calib.cam0_resolution,
                                      calib.cam1_resolution,
                                      calib.cam0_lens_hfov,
                                      calib.cam0_lens_vfov,
                                      calib.cam1_lens_hfov,
                                      calib.cam1_lens_vfov,
                                      calib.cam0_preprocess_path,
                                      calib.cam1_preprocess_path);
  if (retval != 0) {
    LOG_ERROR("Failed to preprocess calibration data!");
    return -1;
  }

  // Load stereo calibration data
  aprilgrids_t cam0_aprilgrids;
  aprilgrids_t cam1_aprilgrids;
  retval = load_stereo_calib_data(calib.cam0_preprocess_path,
                                  calib.cam1_preprocess_path,
                                  cam0_aprilgrids,
                                  cam1_aprilgrids);
  if (retval != 0) {
    LOG_ERROR("Failed to load calibration data!");
    return -1;
  }

  // Setup initial cam0 intrinsics and distortion
  // -- cam0
  const double cam0_img_w = calib.cam0_resolution(0);
  const double cam0_img_h = calib.cam0_resolution(1);
  const double cam0_lens_hfov = calib.cam0_lens_hfov;
  const double cam0_lens_vfov = calib.cam0_lens_vfov;

  const double cam0_fx = pinhole_focal_length(cam0_img_w, cam0_lens_hfov);
  const double cam0_fy = pinhole_focal_length(cam0_img_h, cam0_lens_vfov);
  const double cam0_cx = cam0_img_w / 2.0;
  const double cam0_cy = cam0_img_h / 2.0;
  pinhole_t cam0_pinhole{cam0_fx, cam0_fy, cam0_cx, cam0_cy};
  radtan4_t cam0_radtan{0.01, 0.0001, 0.0001, 0.0001};
  // -- cam1
  const double cam1_img_w = calib.cam1_resolution(0);
  const double cam1_img_h = calib.cam1_resolution(1);
  const double cam1_lens_hfov = calib.cam1_lens_hfov;
  const double cam1_lens_vfov = calib.cam1_lens_vfov;

  const double cam1_fx = pinhole_focal_length(cam1_img_w, cam1_lens_hfov);
  const double cam1_fy = pinhole_focal_length(cam1_img_h, cam1_lens_vfov);
  const double cam1_cx = cam1_img_w / 2.0;
  const double cam1_cy = cam1_img_h / 2.0;
  pinhole_t cam1_pinhole{cam1_fx, cam1_fy, cam1_cx, cam1_cy};
  radtan4_t cam1_radtan{0.01, 0.0001, 0.0001, 0.0001};

  // Calibrate stereo
  LOG_INFO("Calibrating stereo camera!");
  mat4_t T_C0C1 = I(4);
  mat4s_t poses;
  retval = calib_stereo_solve(cam0_aprilgrids,
                              cam1_aprilgrids,
                              cam0_pinhole,
                              cam0_radtan,
                              cam1_pinhole,
                              cam1_radtan,
                              T_C0C1,
                              poses);
  if (retval != 0) {
    LOG_ERROR("Failed to calibrate stereo cameras!");
    return -1;
  }

  // Show results
  std::cout << "Optimization results:" << std::endl;
  // -- cam0
  std::cout << "cam0:" << std::endl;
  std::cout << cam0_pinhole << std::endl;
  std::cout << cam0_radtan << std::endl;
  // -- cam1
  std::cout << "cam1:" << std::endl;
  std::cout << cam1_pinhole << std::endl;
  std::cout << cam1_radtan << std::endl;
  // -- cam0-cam1 extrinsics
  std::cout << "T_C0C1:\n" << T_C0C1 << std::endl;
  std::cout << std::endl;

  // Save results
  const std::string save_path = calib.results_file;
  LOG_INFO("Saving optimization results to [%s]", save_path.c_str());
  retval = save_results(save_path,
                        calib.cam0_resolution,
                        calib.cam1_resolution,
                        cam0_pinhole,
                        cam0_radtan,
                        cam1_pinhole,
                        cam1_radtan,
                        T_C0C1);
  if (retval != 0) {
    LOG_ERROR("Failed to save results to [%s]!", save_path.c_str());
    return -1;
  }

  return 0;
}

} //  namespace proto
