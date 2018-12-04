#include "prototype/calib/calib_camera.hpp"

namespace prototype {

pinhole_radtan4_residual_t::pinhole_radtan4_residual_t(const vec2_t &z,
                                                       const vec3_t &p_F)
    : z_{z(0), z(1)}, p_F_{p_F(0), p_F(1), p_F(2)} {}

pinhole_radtan4_residual_t::~pinhole_radtan4_residual_t() {}

intrinsics_residual_t::intrinsics_residual_t(const vec2_t &z,
                                             const vec3_t &p_F)
    : z_{z(0), z(1)}, p_F_{p_F(0), p_F(1), p_F(2)} {}

intrinsics_residual_t::~intrinsics_residual_t() {}

bool intrinsics_residual_t::Evaluate(double const* const* parameters,
                                     double* residuals,
                                     double** jacobians) const {
  // Map optimization variables
  const mat3_t K = pinhole_K(&parameters[0][0]);
  const vec4_t D(&parameters[1][0]);
  const quat_t q_CF(parameters[2][3],
                    parameters[2][0],
                    parameters[2][1],
                    parameters[2][2]);
  const vec3_t r_CF(&parameters[3][0]);

  // Form transform
  const mat3_t C_CF = q_CF.toRotationMatrix();
  const mat4_t T_CF = transform(C_CF, r_CF);

  // Project
  const vec3_t p_F(p_F_);
  const vec3_t p_C = (T_CF * p_F.homogeneous()).head(3);
  const vec2_t zhat = pinhole_radtan4_project(K, D, p_C);

  // Residual
  residuals[0] = z_[0] - zhat(0);
  residuals[1] = z_[1] - zhat(1);

  // Calculate jacobians
  if (jacobians != NULL) {
    // Jacobian w.r.t. pinhole camera intrinsics K
    if (jacobians[0] != NULL) {
      // dr__zhat
      const mat2_t dr__dzhat = -1 * I(2);

      // dzhat__dpinhole
      const vec2_t p{p_C(0) / p_C(2), p_C(1) / p_C(2)};
      const vec2_t p_dash = distort(radtan4_t(D), p);
      matx_t dzhat__dpinhole = zeros(2, 4);
      // -- Row 1
      dzhat__dpinhole(0, 0) = p_dash(0);
      dzhat__dpinhole(0, 1) = 0.0;
      dzhat__dpinhole(0, 2) = 1.0;
      dzhat__dpinhole(0, 3) = 0.0;
      // -- Row 2
      dzhat__dpinhole(1, 0) = 0.0;
      dzhat__dpinhole(1, 1) = p_dash(1);
      dzhat__dpinhole(1, 2) = 0.0;
      dzhat__dpinhole(1, 3) = 1.0;

      // Fill in jacobian
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobians[0]);
      J = dr__dzhat * dzhat__dpinhole;
    }

    // Jacobian w.r.t. radtan4 distortion D
    if (jacobians[1] != NULL) {
      // dr__zhat
      const mat2_t dr__dzhat = -1 * I(2);

      // // dzhat__dradtan
      // const double x = p_C(0) / p_C(2);
      // const double y = p_C(1) / p_C(2);
      // const double x2 = x * x;
      // const double y2 = y * y;
      // matx_t dzhat__dradtan = zeros(2, 4);
      // // -- Row 1
      // dzhat__dradtan(0, 0) = x * (x2 + y2);
      // dzhat__dradtan(0, 1) = x * (x2 + y2) * (x2 + y2);
      // dzhat__dradtan(0, 2) = 2 * x * y;
      // dzhat__dradtan(0, 3) = 3 * x2 + y2;
      // // -- Row 2
      // dzhat__dradtan(0, 0) = y * (x2 + y2);
      // dzhat__dradtan(0, 1) = y * (x2 + y2) * (x2 + y2);
      // dzhat__dradtan(0, 2) = x2 + 3 * y2;
      // dzhat__dradtan(0, 3) = 2 * x * y;

      // dzhat__dradtan (symbolic diff)
      // -- Setup
      const double px = p_C(0);
      const double py = p_C(1);
      const double pz = p_C(2);
      const double px2 = px * px;
      const double py2 = py * py;
      const double pz2 = pz * pz;
      const double x2 = px2 / pz2;
      const double y2 = py2 / pz2;
      const double r2 = x2 + y2;
      const double r4 = r2 * r2;
      const double fx = K(0, 0);
      const double fy = K(1, 1);
      matx_t dzhat__dradtan = zeros(2, 4);
      // -- Row 1
      dzhat__dradtan(0, 0) = fx * px * r2 / pz;
      dzhat__dradtan(0, 1) = fx * px * r4 / pz;
      dzhat__dradtan(0, 2) = 2 * fx * px * py / pz2;
      dzhat__dradtan(0, 3) = fx * (3 * x2 + y2);
      // -- Row 2
      dzhat__dradtan(1, 0) = fy * py * r2 / pz;
      dzhat__dradtan(1, 1) = fy * py * r4 / pz;
      dzhat__dradtan(1, 2) = fy * (x2 + 3 * y2);
      dzhat__dradtan(1, 3) = 2 * fy * px * py / pz2;

      // Fill in jacobian
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobians[1]);
      J = dr__dzhat * dzhat__dradtan;
    }

    // // Jacobian w.r.t. quaternion q_CF
    // if (jacobians[2] != NULL) {
    //   // dr__zhat
    //   const mat2_t dr__dzhat = -1 * I(2);
    //
    //   // dzhat__dp_C
    //   matx_t dzhat__dp_C = zeros(2, 3);
    //
    //   // dp_F_dq_CF
    //   const double qw = q_CF.w();
    //   const vec3_t qv{q_CF.x(), q_CF.y(), q_CF.z()};
    //   const vec3_t dp__dqw = 2 * (qw * p_F + qv.cross(p_F));
    //   const mat3_t dp__dqv = 2 * (qv.transpose() * p_F * I(3) + qv * p_F.transpose() - p_F * qv.transpose() - qw * skew(p_F));
    //   mat34_t dp_C__dq_CF;
    //   dp_C__dq_CF.block(0, 0, 3, 1) = dp__dqw;
    //   dp_C__dq_CF.block(0, 1, 3, 3) = dp__dqv;
    //
    //   // Fill in jacobian
    //   Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobians[2]);
    //   J = dr__dzhat * dzhat__dp_C * dp_C__dq_CF;
    // }
    //
    // // Jacobian w.r.t. translation r_CF
    // if (jacobians[3] != NULL) {
    //   // dr__zhat
    //   const mat2_t dr__dzhat = -1 * I(2);
    //
    //   // dp_C__dr_CF
    //   const mat3_t dp_C__dr_CF = I(3);
    //
    //   matx_t dp__dp_C = zeros(2, 3);
    //   dp__dp_C(0, 0) = 1.0 / p_C(3);
    //   dp__dp_C(1, 1) = 1.0 / p_C(3);
    //   dp__dp_C(0, 2) = -(p_C(1) / p_C(3)*p_C(3));
    //   dp__dp_C(1, 2) = -(p_C(2) / p_C(3)*p_C(3));
    //
    //   // dzhat__dp_C
    //   const vec2_t p{p_C(0) / p_C(2), p_C(1) / p_C(2)};
    //   mat2_t dzhat__dp;
    //   distort(radtan4_t{D}, p, dzhat__dp);
    //
    //   // Fill in jacobian
    //   Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[3]);
    //   J = dr__dzhat * dzhat__dp * dp__dp_C * dp_C__dr_CF;
    // }
  }

  return true;
}

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
                                pose->t.data());
    }
  }
  // problem->SetParameterBlockConstant(distortion);
  // problem->SetParameterBlockConstant(pose->q.coeffs().data());
  // problem->SetParameterBlockConstant(pose->t.data());

  return 0;
}

int calib_camera_solve(const std::vector<aprilgrid_t> &aprilgrids,
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
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
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

  // Clean up
  T_CF.clear();
  for (auto pose_param : T_CF_params) {
    T_CF.emplace_back(transform(pose_param.q, pose_param.t));
  }

  return 0;
}

} //  namespace prototype
