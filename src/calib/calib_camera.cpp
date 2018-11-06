#include "prototype/calib/calib_camera.hpp"

namespace prototype {

int calib_camera_solve(const std::vector<aprilgrid_t> &aprilgrids,
                       pinhole_t &pinhole,
                       radtan4_t &radtan) {
  // Optimization variables
  double intrinsics[4] = {pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy};
  double distortion[4] = {radtan.k1, radtan.k2, radtan.p1, radtan.p2};
  std::vector<double *> quaternions;
  std::vector<double *> translations;

  // Setup optimization problem
  ceres::Problem problem;
  ceres::EigenQuaternionParameterization quaternion_parameterization;

  for (const auto &aprilgrid: aprilgrids) {
    const mat4_t T_CF = aprilgrid.T_CF;
    const quat_t q_CF{T_CF.block<3, 3>(0, 0)};
    const vec3_t t_CF{T_CF.block<3, 1>(0, 3)};
    quaternions.push_back(quat2array(q_CF));
    translations.push_back(vec2array(t_CF));

    size_t nb_measurements = aprilgrid.ids.size();
    for (size_t i = 0; i < nb_measurements; i++) {
      const auto tag_id = aprilgrid.ids[i];

      // Get keypoints
      std::vector<vec2_t> keypoints;
      if (aprilgrid_get(aprilgrid, tag_id, keypoints) != 0) {
        LOG_ERROR("Failed to calculate AprilGrid object point!");
        return -1;
      }

      for (size_t j = 0; j < 1; j++) {
        // Get object point
        const int corner_id = j;
        vec3_t object_point;
        if (aprilgrid_object_point(aprilgrid, tag_id, corner_id, object_point) != 0) {
          LOG_ERROR("Failed to calculate AprilGrid object point!");
        }

        // Form residual and cost function
        const auto kp = keypoints[j];
        const auto residual = new pinhole_radtan4_residual_t{kp, object_point};

        const auto cost_func =
          new ceres::AutoDiffCostFunction<pinhole_radtan4_residual_t, // Residual type
                                          2, // Size of: residual
                                          4, // Size of: intrinsics
                                          4, // Size of: distortion
                                          4, // Size of: q_CF
                                          3  // Size of: t_CF
                                          >(residual);

        // Add residual block
        problem.AddResidualBlock(cost_func, // Cost function
                                 NULL,      // Loss function
                                 intrinsics,
                                 distortion,
                                 quaternions.back(),
                                 translations.back());
      }

    }
    problem.SetParameterization(quaternions.back(), &quaternion_parameterization);
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  // Show results
  std::cout << "Optimized intrinsics and distortions:" <<  std::endl;
  std::cout << "fx: " << intrinsics[0] << std::endl;
  std::cout << "fy: " << intrinsics[1] << std::endl;
  std::cout << "cx: " << intrinsics[2] << std::endl;
  std::cout << "cy: " << intrinsics[3] << std::endl;
  std::cout << std::endl;

  std::cout << "k1: " << distortion[0] << std::endl;
  std::cout << "k2: " << distortion[1] << std::endl;
  std::cout << "p1: " << distortion[2] << std::endl;
  std::cout << "p2: " << distortion[3] << std::endl;
  std::cout << std::endl;

  // Free pose
  for (size_t i = 0; i < quaternions.size(); i++) {
    free(quaternions[i]);
    free(translations[i]);
  }

  // Map results back to pinhole and radtan
  pinhole.fx = intrinsics[0];
  pinhole.fy = intrinsics[1];
  pinhole.cx = intrinsics[2];
  pinhole.cy = intrinsics[3];

  radtan.k1 = distortion[0];
  radtan.k2 = distortion[1];
  radtan.p1 = distortion[2];
  radtan.p2 = distortion[3];

  return 0;
}

} //  namespace prototype
