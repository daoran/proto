#include "proto/calib/calib_vicon_marker.hpp"

namespace proto {

vicon_marker_residual_t::vicon_marker_residual_t(const vec2_t &z,
                                                 const vec3_t &p_F)
    : z_{z(0), z(1)}, p_F_{p_F(0), p_F(1), p_F(2)} {}

vicon_marker_residual_t::~vicon_marker_residual_t() {}

static int process_aprilgrid(const aprilgrid_t &aprilgrid,
                             double *intrinsics,
                             double *distortion,
                             calib_pose_param_t *T_MC,
                             calib_pose_param_t *T_WM,
                             calib_pose_param_t *T_WF,
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
      const auto residual = new vicon_marker_residual_t{kp, obj_pt};

      const auto cost_func =
          new ceres::AutoDiffCostFunction<vicon_marker_residual_t,
                                          2, // Size of: residual
                                          4, // Size of: intrinsics
                                          4, // Size of: distortion
                                          4, // Size of: q_MC
                                          3, // Size of: t_MC
                                          4, // Size of: q_WM
                                          3, // Size of: t_WM
                                          4, // Size of: q_WF
                                          3  // Size of: t_WF
                                          >(residual);

      problem->AddResidualBlock(cost_func, // Cost function
                                NULL,      // Loss function
                                intrinsics,
                                distortion,
                                T_MC->q.coeffs().data(),
                                T_MC->r.data(),
                                T_WM->q.coeffs().data(),
                                T_WM->r.data(),
                                T_WF->q.coeffs().data(),
                                T_WF->r.data());
    }
  }

  return 0;
}

double evaluate_vicon_marker_cost(const std::vector<aprilgrid_t> &aprilgrids,
                                  mat4s_t &T_WM,
                                  pinhole_t &pinhole,
                                  radtan4_t &radtan,
                                  mat4_t &T_MC) {
  assert(aprilgrids.size() > 0);
  assert(T_WM.size() > 0);
  assert(T_WM.size() == aprilgrids.size());

  // Optimization variables
  calib_pose_param_t T_MC_param{T_MC};
  calib_pose_param_t T_WF_param{T_WM[0] * T_MC * aprilgrids[0].T_CF};
  std::vector<calib_pose_param_t> T_WM_params;
  for (size_t i = 0; i < T_WM.size(); i++) {
    T_WM_params.push_back(T_WM[i]);
  }

  // Setup optimization problem
  ceres::Problem::Options problem_options;
  std::unique_ptr<ceres::Problem> problem(new ceres::Problem(problem_options));

  // Process all aprilgrid data
  for (size_t i = 0; i < aprilgrids.size(); i++) {
    int retval = process_aprilgrid(aprilgrids[i],
                                   *pinhole.data,
                                   *radtan.data,
                                   &T_MC_param,
                                   &T_WM_params[i],
                                   &T_WF_param,
                                   problem.get());
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }
  }

  double cost;
  problem->Evaluate(ceres::Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
  return cost;
}

int calib_vicon_marker_solve(const std::vector<aprilgrid_t> &aprilgrids,
                             mat4s_t &T_WM,
                             pinhole_t &pinhole,
                             radtan4_t &radtan,
                             mat4_t &T_MC,
                             mat4_t &T_WF) {
  assert(aprilgrids.size() > 0);
  assert(T_WM.size() > 0);
  assert(T_WM.size() == aprilgrids.size());

  // Optimization variables
  calib_pose_param_t T_MC_param{T_MC};
  calib_pose_param_t T_WF_param{T_WM[0] * T_MC * aprilgrids[0].T_CF};
  std::vector<calib_pose_param_t> T_WM_params;
  for (size_t i = 0; i < T_WM.size(); i++) {
    T_WM_params.push_back(T_WM[i]);
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
                                   &T_MC_param,
                                   &T_WM_params[i],
                                   &T_WF_param,
                                   problem.get());
    if (retval != 0) {
      LOG_ERROR("Failed to add AprilGrid measurements to problem!");
      return -1;
    }

    // Fixing the marker pose - assume vicon is calibrated and accurate
    problem->SetParameterBlockConstant(T_WM_params[i].q.coeffs().data());
    problem->SetParameterBlockConstant(T_WM_params[i].r.data());
    problem->SetParameterization(T_WM_params[i].q.coeffs().data(),
                                 &quaternion_parameterization);
  }
  problem->SetParameterBlockConstant(*pinhole.data);
  problem->SetParameterBlockConstant(*radtan.data);
  problem->SetParameterization(T_MC_param.q.coeffs().data(),
                               &quaternion_parameterization);
  problem->SetParameterization(T_WF_param.q.coeffs().data(),
                               &quaternion_parameterization);

  // Set solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  options.num_threads = 4;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  std::cout << summary.FullReport() << std::endl;

  // Finish up
  // -- Marker pose
  T_WM.clear();
  for (const auto T_WM_param : T_WM_params) {
    T_WM.push_back(tf(T_WM_param.q, T_WM_param.r));
  }
  // -- Marker to camera extrinsics
  T_MC = tf(T_MC_param.q, T_MC_param.r);
  // -- Fiducial pose
  T_WF = tf(T_WF_param.q, T_WF_param.r);

  return 0;
}

} //  namespace proto
