#include "prototype/calib/calib_stereo.hpp"

namespace prototype {

stereo_residual_t::stereo_residual_t(const vec2_t &z_C0,
                                     const vec2_t &z_C1,
                                     const vec3_t &p_F)
    : z_C0_{z_C0(0), z_C0(1)},
      z_C1_{z_C1(0), z_C1(1)},
      p_F_{p_F(0), p_F(1), p_F(2)} {}

stereo_residual_t::~stereo_residual_t() {}

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
                                T_C0C1->t.data(),
                                T_C0F->q.coeffs().data(),
                                T_C0F->t.data());
    }
  }

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
  std::unique_ptr<calib_pose_param_t> extrinsic_param(new calib_pose_param_t(T_C0C1));
  std::vector<calib_pose_param_t> pose_params;
  for (size_t i = 0; i < cam0_aprilgrids.size(); i++) {
    pose_params.emplace_back(cam0_aprilgrids[i].T_CF);
  }

  // Setup optimization problem
  ceres::Problem::Options problem_options;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  std::unique_ptr<ceres::Problem> problem(new ceres::Problem(problem_options));
  ceres::EigenQuaternionParameterization quaternion_parameterization;

  // Process all aprilgrid data
  for (size_t i = 0; i < cam0_aprilgrids.size(); i++) {
    int retval = process_aprilgrid(cam0_aprilgrids[i],
                                   cam1_aprilgrids[i],
                                   *cam0_pinhole.data,
                                   *cam0_radtan.data,
                                   *cam1_pinhole.data,
                                   *cam1_radtan.data,
                                   extrinsic_param.get(),
                                   &pose_params[i],
                                   problem.get());
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }

    problem->SetParameterization(pose_params[i].q.coeffs().data(),
                                 &quaternion_parameterization);
  }
  problem->SetParameterization(extrinsic_param->q.coeffs().data(),
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
  T_C0C1 = transform(extrinsic_param->q.toRotationMatrix(), extrinsic_param->t);
  for (auto pose_param : pose_params) {
    T_C0F.emplace_back(transform(pose_param.q, pose_param.t));
  }

  return 0;
}

} //  namespace prototype
