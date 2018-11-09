#include "prototype/calib/calib_camera.hpp"

namespace prototype {

static int process_aprilgrid(const aprilgrid_t &aprilgrid,
                             vec4_t &intrinsics,
                             vec4_t &distortion,
                             pose_param_t *pose,
                             ceres::Problem *problem) {
  for (const auto &tag_id : aprilgrid.ids) {
    // Get keypoints
    std::vector<vec2_t> keypoints;
    if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
      LOG_ERROR("Failed to get AprilGrid keypoints!");
      return -1;
    }

    // Get object points
    std::vector<vec3_t> object_points;
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
                                          3  // Size of: t_CF
                                          >(residual);

      problem->AddResidualBlock(cost_func, // Cost function
                                NULL,      // Loss function
                                intrinsics.data(),
                                distortion.data(),
                                pose->q.coeffs().data(),
                                pose->t.data());
    }
  }

  return 0;
}

int calib_camera_solve(const std::vector<aprilgrid_t> &aprilgrids,
                       pinhole_t &pinhole,
                       radtan4_t &radtan,
                       std::vector<mat4_t> &poses) {
  // Optimization variables
  vec4_t intrinsics{pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy};
  vec4_t distortion{radtan.k1, radtan.k2, radtan.p1, radtan.p2};
  std::vector<pose_param_t *> pose_params;

  // Setup optimization problem
  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem *problem = new ceres::Problem(problem_options);
  ceres::EigenQuaternionParameterization quaternion_parameterization;

  // Process all aprilgrid data
  for (const auto &aprilgrid : aprilgrids) {
    pose_params.push_back(new pose_param_t(aprilgrid.T_CF));

    int retval = process_aprilgrid(aprilgrid,
                                   intrinsics,
                                   distortion,
                                   pose_params.back(),
                                   problem);
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }

    problem->SetParameterization(pose_params.back()->q.coeffs().data(),
                                 &quaternion_parameterization);
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  // Map results back to pinhole and radtan
  pinhole =
      pinhole_t{intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]};
  radtan =
      radtan4_t{distortion[0], distortion[1], distortion[2], distortion[3]};

  // Clean up
  for (auto pose_ptr : pose_params) {
    poses.emplace_back(transform(pose_ptr->q, pose_ptr->t));
    free(pose_ptr);
  }
  free(problem);

  return 0;
}

} //  namespace prototype
