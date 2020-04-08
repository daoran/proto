#include "proto/calib/calib_camera.hpp"

namespace proto {

/*****************************************************************************
 *                             MONCULAR CAMERA
 ****************************************************************************/

mat4s_t calib_generate_poses(const calib_target_t &target) {
  const real_t target_width = (target.tag_rows - 1.0) * target.tag_size;
  const real_t target_height = (target.tag_cols - 1.0) * target.tag_size;
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

// // aprilgrid_t
// // nbv_create_aprilgrid(const calib_target_t &target,
// //                      const camera_geometry_t<pinhole_t, radtan4_t> &camera,
// //                      const vec2_t &resolution,
// //                      const mat4_t &T_CF) {
// //   // Create AprilGrid
// //   aprilgrid_t grid;
// //   grid.tag_rows = target.tag_rows;
// //   grid.tag_cols = target.tag_cols;
// //   grid.tag_size = target.tag_size;
// //   grid.tag_spacing = target.tag_spacing;
// //
// //   // For every AprilGrid tag
// //   grid.detected = true;
// //   grid.nb_detections = (grid.tag_rows * grid.tag_cols);
// //   for (int i = 0; i < grid.nb_detections; i++) {
// //     // Check if object points are observable
// //     vec3s_t object_points;
// //     aprilgrid_object_points(grid, i, object_points);
// //     vec2s_t observed_keypoints;
// //     vec3s_t observed_points_CF;
// //
// //     for (const auto &p_F : object_points) {
// //       const vec4_t hp_F = p_F.homogeneous();
// //       const vec3_t p_C = (T_CF * hp_F).head(3);
// //       const vec2_t kp = camera_geometry_project(camera, p_C);
// //
// //       const bool x_ok = (kp(0) >= 0 && kp(0) <= resolution(0));
// //       const bool y_ok = (kp(1) >= 0 && kp(1) <= resolution(1));
// //       if (x_ok && y_ok) {
// //         observed_keypoints.push_back(kp);
// //         observed_points_CF.push_back(p_C);
// //       }
// //     }
// //
// //     // Add keypoints and points
// //     if (observed_keypoints.size() == 4) {
// //       grid.ids.push_back(i);
// //       for (size_t i = 0; i < 4; i++) {
// //         grid.keypoints.push_back(observed_keypoints[i]);
// //         grid.points_CF.push_back(observed_points_CF[i]);
// //       }
// //     }
// //   }
// //
// //   // Esimated pose
// //   grid.estimated = true;
// //   grid.T_CF = T_CF;
// //
// //   return grid;
// // }
//
// // void nbv_draw_aprilgrid(const aprilgrid_t grid,
// //                         const pinhole_t &pinhole,
// //                         const radtan4_t &radtan,
// //                         const mat4_t &T_CT,
// //                         cv::Mat &frame) {
// //   const cv::Scalar blue(255, 0, 0);
// //   const cv::Scalar green(0, 255, 0);
// //   const cv::Scalar red(0, 0, 255);
// //
// //   for (const auto &tag_id : grid.ids) {
// //     // Get tag object points
// //     vec3s_t object_points;
// //     aprilgrid_object_points(grid, tag_id, object_points);
// //
// //     // Transform object points to camera frame
// //     const size_t nb_pts = object_points.size();
// //     const matx_t hp_T = vecs2mat(object_points);
// //     const matx_t hp_C = T_CT * hp_T;
// //     const matx_t p_C = hp_C.block(0, 0, 3, nb_pts);
// //
// //     // Draw a rectangle for each tag
// //     for (int i = 0; i < 4; i++) {
// //       cv::Point2f p1;
// //       {
// //         const vec3_t p = p_C.block(0, i, 3, 1);
// //         const vec2_t p_proj{p(0) / p(2), p(1) / p(2)};
// //         const vec2_t pt_D = distort(radtan, p_proj);
// //         const vec2_t pixel = project(pinhole, pt_D);
// //         p1 = cv::Point2f(pixel(0), pixel(1));
// //       }
// //
// //       cv::Point2f p2;
// //       {
// //         const size_t idx = ((i + 1) == 4) ? 0 : i + 1;
// //         const vec3_t p = p_C.block(0, idx, 3, 1);
// //         const vec2_t p_proj{p(0) / p(2), p(1) / p(2)};
// //         const vec2_t pt_D = distort(radtan, p_proj);
// //         const vec2_t pixel = project(pinhole, pt_D);
// //         p2 = cv::Point2f(pixel(0), pixel(1));
// //       }
// //
// //       cv::line(frame, p1, p2, red, 3);
// //     }
// //   }
// // }
// //
// // void nbv_find(const calib_target_t &target,
// //               const aprilgrids_t &aprilgrids,
// //               const pinhole_t &pinhole,
// //               const radtan4_t &radtan,
// //               mat4_t &nbv_pose,
// //               aprilgrid_t &nbv_grid) {
// //   // Evalute NBV
// //   real_t entropy_best = 0.0;
// //   const camera_geometry_t<pinhole_t, radtan4_t> camera{pinhole, radtan};
// //   const vec2_t resolution{pinhole.cx * 2.0, pinhole.cy * 2.0};
// //
// //   // -- Loop over all NBV poses and find one with lowest entropy
// //   for (const auto &T_TC : calib_generate_poses(target)) {
// //     // -- Setup AprilGrids with added NBV grid
// //     aprilgrids_t grids = aprilgrids;
// //     const mat4_t T_CT = T_TC.inverse();
// //     nbv_grid = nbv_create_aprilgrid(target, camera, resolution, T_CT);
// //     grids.push_back(nbv_grid);
// //
// //     // -- Setup camera and distortion model
// //     pinhole_t pinhole_copy = pinhole;
// //     radtan4_t radtan_copy = radtan;
// //     mat4s_t T_CT_ignore;
// //
// //     // -- Evaluate NBV
// //     const real_t entropy =
// //         calib_camera_nbv_solve(grids, pinhole_copy, radtan_copy, T_CT_ignore);
// //     if (entropy < entropy_best) {
// //       entropy_best = entropy;
// //       nbv_pose = T_CT;
// //     }
// //   }
// // }
//
// // static void initialize_intrinsics(cv::VideoCapture &camera,
// //                                   const calib_target_t &target,
// //                                   aprilgrids_t &aprilgrids,
// //                                   pinhole_t &pinhole,
// //                                   radtan4_t &radtan,
// //                                   mat4s_t &T_CT) {
// //   // Guess the camera intrinsics and distortion
// //   cv::Mat frame;
// //   camera.read(frame);
// //   const auto detector = aprilgrid_detector_t();
// //   const real_t fx = pinhole_focal(frame.cols, 120.0);
// //   const real_t fy = pinhole_focal(frame.rows, 120.0);
// //   const real_t cx = frame.cols / 2.0;
// //   const real_t cy = frame.rows / 2.0;
// //   pinhole = pinhole_t{frame.cols, frame.rows, fx, fy, cx, cy};
// //   radtan = radtan4_t{0.0, 0.0, 0.0, 0.0};
// //   const mat3_t K = pinhole.K();
// //   const vec4_t D = radtan.vec();
// //
// //   while (aprilgrids.size() < 5) {
// //     // Get image
// //     cv::Mat frame;
// //     camera.read(frame);
// //
// //     // Detect AprilGrid
// //     aprilgrid_t grid;
// //     aprilgrid_set_properties(grid,
// //                              target.tag_rows,
// //                              target.tag_cols,
// //                              target.tag_size,
// //                              target.tag_spacing);
// //     aprilgrid_detect(grid, detector, frame, K, D);
// //
// //     // Show image and get user input
// //     cv::imshow("Image", frame);
// //     char key = (char) cv::waitKey(1);
// //     if (key == 'q') {
// //       break;
// //     } else if (key == 'c' && grid.detected) {
// //       aprilgrids.push_back(grid);
// //     }
// //   }
// //
// //   calib_camera_nbv_solve(aprilgrids, pinhole, radtan, T_CT);
// //   std::cout << "pinhole: " << pinhole << std::endl;
// //   std::cout << "radtan: " << radtan << std::endl;
// // }
//
// static int process_aprilgrid(const aprilgrid_t &aprilgrid,
//                              real_t *intrinsics,
//                              real_t *distortion,
//                              calib_pose_t *pose,
//                              ceres::Problem *problem) {
//   for (const auto &tag_id : aprilgrid.ids) {
//     // Get keypoints
//     vec2s_t keypoints;
//     if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
//       LOG_ERROR("Failed to get AprilGrid keypoints!");
//       return -1;
//     }
//
//     // Get object points
//     vec3s_t object_points;
//     if (aprilgrid_object_points(aprilgrid, tag_id, object_points) != 0) {
//       LOG_ERROR("Failed to calculate AprilGrid object points!");
//       return -1;
//     }
//
//     // Form residual block
//     for (size_t i = 0; i < 4; i++) {
//       const auto kp = keypoints[i];
//       const auto obj_pt = object_points[i];
//
//       const auto residual = new pinhole_radtan4_residual_t{kp, obj_pt};
//       const auto cost_func =
//           new ceres::AutoDiffCostFunction<pinhole_radtan4_residual_t,
//                                           2, // Size of: residual
//                                           4, // Size of: intrinsics
//                                           4, // Size of: distortion
//                                           4, // Size of: q_CF
//                                           3  // Size of: r_CF
//                                           >(residual);
//
//       // const auto cost_func = new intrinsics_residual_t{kp, obj_pt};
//       problem->AddResidualBlock(cost_func, // Cost function
//                                 NULL,      // Loss function
//                                 intrinsics,
//                                 distortion,
//                                 pose->q,
//                                 pose->r);
//     }
//   }
//
//   return 0;
// }
//
// // real_t calib_camera_nbv_solve(aprilgrids_t &aprilgrids,
// //                               pinhole_t &pinhole,
// //                               radtan4_t &radtan,
// //                               mat4s_t &T_CF) {
// //   // Optimization variables
// //   std::vector<calib_pose_t> T_CF_params;
// //   for (size_t i = 0; i < aprilgrids.size(); i++) {
// //     T_CF_params.emplace_back(aprilgrids[i].T_CF);
// //   }
// //
// //   // Setup optimization problem
// //   ceres::Problem::Options problem_options;
// //   problem_options.local_parameterization_ownership =
// //       ceres::DO_NOT_TAKE_OWNERSHIP;
// //   std::unique_ptr<ceres::Problem> problem(new ceres::Problem(problem_options));
// //   ceres::EigenQuaternionParameterization quaternion_parameterization;
// //
// //   // Process all aprilgrid data
// //   for (size_t i = 0; i < aprilgrids.size(); i++) {
// //     int retval = process_aprilgrid(aprilgrids[i],
// //                                    *pinhole.data,
// //                                    *radtan.data,
// //                                    &T_CF_params[i],
// //                                    problem.get());
// //     if (retval != 0) {
// //       LOG_ERROR("Failed to add AprilGrid measurements to problem!");
// //       return -1;
// //     }
// //     problem->SetParameterization(T_CF_params[i].q,
// //                                  &quaternion_parameterization);
// //   }
// //
// //   // Set solver options
// //   ceres::Solver::Options options;
// //   options.minimizer_progress_to_stdout = false;
// //   options.max_num_iterations = 100;
// //   // options.check_gradients = true;
// //
// //   // Solve
// //   ceres::Solver::Summary summary;
// //   ceres::Solve(options, problem.get(), &summary);
// //
// //   // Estimate covariance
// //   real_t entropy = 0.0;
// //   {
// //     // Estimate pinhole and radtan covariance
// //     std::vector<std::pair<const real_t *, const real_t *>> covar_blocks;
// //     covar_blocks.push_back({*pinhole.data, *pinhole.data});
// //     covar_blocks.push_back({*radtan.data, *radtan.data});
// //     covar_blocks.push_back({*pinhole.data, *radtan.data});
// //     ceres::Covariance::Options options;
// //     ceres::Covariance covar_est(options);
// //     covar_est.Compute(covar_blocks, problem.get());
// //
// //     // Extract pinhole and radtan covariance
// //     real_t pinhole_covar[4 * 4];
// //     real_t radtan_covar[4 * 4];
// //     real_t pinhole_radtan_covar[4 * 4];
// //     covar_est.GetCovarianceBlock(*pinhole.data, *pinhole.data, pinhole_covar);
// //     covar_est.GetCovarianceBlock(*radtan.data, *radtan.data, radtan_covar);
// //     covar_est.GetCovarianceBlock(*pinhole.data,
// //                                  *radtan.data,
// //                                  pinhole_radtan_covar);
// //
// //     // Calculate shannon entropy
// //     matx_t covar;
// //     covar.resize(8, 8);
// //     covar.block(0, 0, 4, 4) = mat4_t{pinhole_covar};
// //     covar.block(0, 4, 4, 4) = mat4_t{pinhole_radtan_covar};
// //     covar.block(4, 4, 4, 4) = mat4_t{radtan_covar};
// //     covar.block(4, 0, 4, 4) = covar.block(0, 4, 4, 4).transpose();
// //     entropy = shannon_entropy(covar);
// //   }
// //
// //   // Update aprilgrid relative pose
// //   for (size_t idx = 0; idx < T_CF.size(); idx++) {
// //     aprilgrids[idx].T_CF = T_CF[idx];
// //   }
// //
// //   // Add results to T_CF
// //   T_CF.clear();
// //   for (auto pose_param : T_CF_params) {
// //     T_CF.push_back(pose_param.T());
// //   }
// //
// //   return entropy;
// // }
//
// // static void validate_calibration(const calib_target_t &target,
// //                                  cv::VideoCapture &camera,
// //                                  pinhole_t &pinhole,
// //                                  radtan4_t &radtan) {
// //   pinhole_radtan4_t camera_geometry{pinhole, radtan};
// //   const mat3_t cam_K = pinhole.K();
// //   const vec4_t cam_D = radtan.vec();
// //   const auto detector = aprilgrid_detector_t();
// //
// //   while (true) {
// //     // Get image
// //     cv::Mat frame;
// //     camera.read(frame);
// //
// //     aprilgrid_t grid;
// //     aprilgrid_set_properties(grid,
// //                              target.tag_rows,
// //                              target.tag_cols,
// //                              target.tag_size,
// //                              target.tag_spacing);
// //     aprilgrid_detect(grid, detector, frame, cam_K, cam_D);
// //
// //     if (grid.detected) {
// //       cv::Mat validate_frame;
// //       validate_frame = validate_intrinsics(frame,
// //                                            grid.keypoints,
// //                                            grid.points_CF,
// //                                            camera_geometry);
// //       cv::imshow("Image", validate_frame);
// //     } else {
// //       cv::imshow("Image", frame);
// //     }
// //
// //     // Show image and get user input
// //     char key = (char) cv::waitKey(1);
// //     if (key == 'q') {
// //       break;
// //     }
// //   }
// // }
//
// // int calib_camera_nbv(const std::string &target_path, const size_t max_frames) {
// //   // Setup calibration target
// //   calib_target_t target;
// //   if (calib_target_load(target, target_path) != 0) {
// //     LOG_ERROR("Failed to load calib target [%s]!", target_path.c_str());
// //     return -1;
// //   }
// //
// //   // Setup camera
// //   cv::VideoCapture camera(2);
// //   if (camera.isOpened() == false) {
// //     return -1;
// //   }
// //   sleep(2);
// //
// //   // Initialize intrinsics
// //   aprilgrids_t aprilgrids;
// //   pinhole_t pinhole;
// //   radtan4_t radtan;
// //   mat4s_t target_poses;
// //   initialize_intrinsics(camera,
// //                         target,
// //                         aprilgrids,
// //                         pinhole,
// //                         radtan,
// //                         target_poses);
// //   LOG_INFO("Initialized intrinsics and distortion!");
// //
// //   // Loop camera feed
// //   const auto detector = aprilgrid_detector_t();
// //   mat4_t nbv_pose;
// //   aprilgrid_t nbv_grid;
// //   nbv_find(target, aprilgrids, pinhole, radtan, nbv_pose, nbv_grid);
// //
// //   while (true) {
// //     // Get image
// //     cv::Mat frame;
// //     camera.read(frame);
// //
// //     // Detect AprilGrid
// //     aprilgrid_t grid;
// //     aprilgrid_set_properties(grid,
// //                              target.tag_rows,
// //                              target.tag_cols,
// //                              target.tag_size,
// //                              target.tag_spacing);
// //     const mat3_t K = pinhole.K();
// //     const vec4_t D{radtan.k1, radtan.k2, radtan.p1, radtan.p2};
// //     aprilgrid_detect(grid, detector, frame, K, D);
// //
// //     // Draw NBV
// //     nbv_draw_aprilgrid(nbv_grid, pinhole, radtan, nbv_pose, frame);
// //
// //     // Calculate position difference between desired and actual
// //     const vec3_t pos_desired = tf_trans(nbv_pose);
// //     const vec3_t pos_actual = tf_trans(grid.T_CF);
// //     const real_t pos_diff = (pos_desired - pos_actual).norm();
// //     bool pos_ok = (pos_diff < 0.03) ? true : false;
// //
// //     // Calculate attitute difference between desired and actual
// //     // const vec3_t rpy_desired = quat2euler(tf_quat(nbv_pose));
// //     // const vec3_t rpy_actual = quat2euler(tf_quat(grid.T_CF));
// //     // const real_t rpy_diff = (rpy_desired - rpy_actual).norm();
// //     // bool rpy_ok = (rad2deg(rpy_diff) < 5.0) ? true : false;
// //     bool rpy_ok = true;
// //
// //     // Update NBV if pose achieved and new AprilGrid, optimize and find NBV
// //     // printf("pos diff [%.2f]\t rpy_diff [%.2f]\n", pos_diff, rpy_diff);
// //     printf("pos diff [%.2f]\n", pos_diff);
// //     if (pos_ok && rpy_ok && grid.detected) {
// //       LOG_INFO("NBV Pose reached!");
// //       aprilgrids.push_back(grid);
// //       calib_camera_nbv_solve(aprilgrids, pinhole, radtan, target_poses);
// //
// //       LOG_INFO("Finding NBV ...");
// //       nbv_find(target, aprilgrids, pinhole, radtan, nbv_pose, nbv_grid);
// //       LOG_INFO("New NBV!");
// //     }
// //
// //     // Termination criteria
// //     if (aprilgrids.size() > max_frames) {
// //       break;
// //     }
// //
// //     // Show image and get user input
// //     cv::imshow("Image", frame);
// //     // cv::Mat frame_flipped;
// //     // cv::flip(frame, frame_flipped, 1);
// //     // cv::imshow("Image", frame_flipped);
// //     char key = (char) cv::waitKey(1);
// //     if (key == 'q') {
// //       break;
// //     }
// //   }
// //
// //   std::cout << pinhole << std::endl;
// //   std::cout << radtan << std::endl;
// //   validate_calibration(target, camera, pinhole, radtan);
// //
// //   return 0;
// // }

// int calib_camera_batch(const std::string &target_path,
//                        const size_t max_frames) {
//   // Setup calibration target
//   calib_target_t target;
//   if (calib_target_load(target, target_path) != 0) {
//     LOG_ERROR("Failed to load calib target [%s]!", target_path.c_str());
//     return -1;
//   }
//
//   // Setup camera
//   cv::VideoCapture camera(2);
//   if (camera.isOpened() == false) {
//     return -1;
//   }
//   // sleep(2);
//
//   // Guess the camera intrinsics and distortion
//   cv::Mat frame;
//   camera.read(frame);
//   const auto detector = aprilgrid_detector_t();
//   const real_t fx = pinhole_focal(frame.cols, 120.0);
//   const real_t fy = pinhole_focal(frame.rows, 120.0);
//   const real_t cx = frame.cols / 2.0;
//   const real_t cy = frame.rows / 2.0;
//   pinhole_t pinhole{frame.cols, frame.rows, fx, fy, cx, cy};
//   radtan4_t radtan{0.0, 0.0, 0.0, 0.0};
//   const mat3_t K = pinhole.K();
//   const vec4_t D = radtan.vec();
//
//   aprilgrids_t aprilgrids;
//   while (aprilgrids.size() < max_frames) {
//     // Get image
//     cv::Mat frame;
//     camera.read(frame);
//
//     // Detect AprilGrid
//     aprilgrid_t grid;
//     aprilgrid_set_properties(grid,
//                              target.tag_rows,
//                              target.tag_cols,
//                              target.tag_size,
//                              target.tag_spacing);
//     aprilgrid_detect(grid, detector, frame, K, D);
//
//     // Show image and get user input
//     cv::imshow("Image", frame);
//     char key = (char) cv::waitKey(1);
//     if (key == 'q') {
//       break;
//     } else if (key == 'c' && grid.detected) {
//       aprilgrids.push_back(grid);
//     }
//   }
//
//   mat4s_t target_poses;
//   calib_camera_solve(aprilgrids, pinhole, radtan, target_poses);
//   std::cout << pinhole << std::endl;
//   std::cout << radtan << std::endl;
//   // validate_calibration(target, camera, pinhole, radtan);
//
//   return 0;
// }

} //  namespace proto
