#include "prototype/calib/calib_camera.hpp"

namespace proto {

pinhole_radtan4_residual_t::pinhole_radtan4_residual_t(const vec2_t &z,
                                                       const vec3_t &p_F)
    : z_{z(0), z(1)}, p_F_{p_F(0), p_F(1), p_F(2)} {}

pinhole_radtan4_residual_t::~pinhole_radtan4_residual_t() {}

static int process_aprilgrid(const aprilgrid_t &aprilgrid,
                             double *intrinsics,
                             double *distortion,
                             calib_pose_param_t *pose,
                             ceres::Problem *problem) {
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
      problem->AddResidualBlock(cost_func, // Cost function
                                NULL,      // Loss function
                                intrinsics,
                                distortion,
                                pose->q.coeffs().data(),
                                pose->r.data());
    }
  }

  return 0;
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
  std::unique_ptr<ceres::Problem> problem(new ceres::Problem(problem_options));
  ceres::EigenQuaternionParameterization quaternion_parameterization;

  // Process all aprilgrid data
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    int retval = process_aprilgrid(aprilgrids[i],
                                   *pinhole.data,
                                   *radtan.data,
                                   &T_CF_params[i],
                                   problem.get());
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
    problem->SetParameterization(T_CF_params[i].q.coeffs().data(),
                                 &quaternion_parameterization);
  }

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  // options.check_gradients = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  std::cout << summary.FullReport() << std::endl;

  // {
  //   ceres::Covariance::Options options;
  //   ceres::Covariance covar(options);
  //
  //   std::vector<std::pair<const double *, const double *>> covar_blocks;
  //   covar_blocks.push_back({*pinhole.data, *pinhole.data});
  //   covar_blocks.push_back({*radtan.data, *radtan.data});
  //   covar_blocks.push_back({*pinhole.data, *radtan.data});
  //   covar.Compute(covar_blocks, problem.get());
  //
  //   double pinhole_covar[4 * 4];
  //   double radtan_covar[4 * 4];
  //   double pinhole_radtan_covar[4 * 4];
  //   covar.GetCovarianceBlock(*pinhole.data, *pinhole.data, pinhole_covar);
  //   covar.GetCovarianceBlock(*radtan.data, *radtan.data, radtan_covar);
  //   covar.GetCovarianceBlock(*pinhole.data, *radtan.data,
  //   pinhole_radtan_covar);
  // }

  // Clean up
  T_CF.clear();
  for (auto pose_param : T_CF_params) {
    T_CF.emplace_back(tf(pose_param.q, pose_param.r));
  }

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

} //  namespace proto
