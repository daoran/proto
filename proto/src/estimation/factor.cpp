#include "proto/estimation/factor.hpp"

namespace proto {

/*****************************************************************************
 * Bundle Adjustment Factor
 ****************************************************************************/

bool ba_factor_t::Evaluate(double const *const *parameters,
                           double *residuals,
                           double **jacobians) const {
  // Pose of sensor in world frame
  // pose = (qw, qx, qy, qz), (x, y, z)
  const double *cam_pose = parameters[0];
  const quat_t q_WC(cam_pose[0], cam_pose[1], cam_pose[2], cam_pose[3]);
  const vec3_t r_WC(cam_pose[4], cam_pose[5], cam_pose[6]);
  const mat4_t T_WC = tf(q_WC, r_WC);

  // Landmark
  const double *point = parameters[1];
  const vec3_t p_W{point[0], point[1], point[2]};

  // Transform point from world to camera frame
  const mat4_t T_CW = T_WC.inverse();
  const mat3_t C_CW = tf_rot(T_CW);
  const vec4_t hp_C = T_CW * p_W.homogeneous();
  const vec3_t p_C{hp_C(0) / hp_C(3), hp_C(1) / hp_C(3), hp_C(2) / hp_C(3)};

  // Check validity of the point, simple depth test.
  if (fabs(p_C(2)) < 0.05) { // 5cm
    return -1;
  }

  // Calculate residual
  mat_t<2, 3> J_project;
  const vec2_t z_hat = project(p_C, J_project);
  const vec2_t error = z - z_hat;
  map_vec_t<2> r(residuals);
  r = sq_info * error;

  // Calculate Jacobian
  if (jacobians == nullptr) {
    return 0;
  }

  // Point jacobian
  if (jacobians[0]) {
    map_mat_t<2, 3, row_major_t> J(jacobians[0]);
    J = -1 * J_project * C_CW;
  }

  // Camera pose jacobian
  if (jacobians[1]) {
    const mat_t<3, 3> dp_C__dp_W = C_CW;
    const mat_t<3, 3> dp_W__dr_WC = I(3);
    const mat_t<3, 3> dp_W__dtheta = -skew(C_CW.transpose() * hp_C.head(3));
    const mat_t<2, 3> dh_dr_WC = J_project * dp_C__dp_W * dp_W__dr_WC;
    const mat_t<2, 3> dh_dtheta = J_project * dp_C__dp_W * dp_W__dtheta;

    map_mat_t<2, 6, row_major_t> J(jacobians[1]);
    J.block(0, 0, 2, 3) = -1 * dh_dtheta;
    J.block(0, 3, 2, 3) = -1 * dh_dr_WC;
  }

  return 0;
}

/*****************************************************************************
 * Camera Factor
 ****************************************************************************/

// bool cam_factor_t::Evaluate(double const * const * parameters,
//                             double *residuals,
//                             double **jacobians) const {
//   UNUSED(residuals);
//   UNUSED(jacobians);
//   // Transform point from world to camera frame
//   const mat4_t T_WC = T_WS->T() * T_SC->T();
//   const mat4_t T_CW = T_WC.inverse();
//   const vec3_t p_C = (T_CW * p_W->vec().homogeneous()).head(3);
//
//   // Check validity of the point, simple depth test.
//   if (fabs(p_C(2)) < 0.05) {
//    return -1;
//   }
//
//   // Calculate residual
//   // const vec3_t x_C = normalize(p_C);
//   // const mat3_t K = I(3);
//   // const vec2_t pixel = (K * x_C).head(2);
//   // vec2_t error = measurement - pixel;
//   // map_vec_t<2> r(residuals);
//   // r = sq_info * error;
//   //
//   // // Calculate Jacobian
//   // if (jacobians == nullptr) {
//   //   return 0;
//   // }
//
// 	// // Jacobian w.r.t. sensor pose T_WS
// 	// if (jacobians[0] != nullptr) {
// 	// 	mat_t<3, 3> dp_C__dp_W = C_SC.transpose() * C_WS.transpose();
// 	// 	mat_t<3, 3> dp_W__dr_WS = I(3);
// 	// 	mat_t<3, 3> dp_W__dtheta = -skew(C_WS * hp_S.head(3));
// 	// 	mat_t<2, 3> dh_dr_WS = Jh_weighted * dp_C__dp_W * dp_W__dr_WS;
// 	// 	mat_t<2, 3> dh_dtheta = Jh_weighted * dp_C__dp_W * dp_W__dtheta;
//   //
// 	// 	map_mat_t<2, 6, row_major_t> J0(jacobians[0]);
// 	// 	J0.block(0, 0, 2, 3) = dh_dr_WS;
// 	// 	J0.block(0, 3, 2, 3) = dh_dtheta;
// 	// 	if (!valid) {
// 	// 		J0.setZero();
// 	// 	}
// 	// }
//
//   // // Jacobian w.r.t. fiducial pose T_WF
//   // if (jacobians[1] != nullptr) {
// 	// 	Eigen::Matrix<double, 3, 3> dp_C__dp_W = C_SC.transpose() *
// C_WS.transpose();
// 	// 	Eigen::Matrix<double, 3, 3> dp_W__dr_WF = I(3);
// 	// 	Eigen::Matrix<double, 3, 3> dp_W__dtheta = -skew(C_WF * p_F_);
// 	// 	Eigen::Matrix<double, 2, 3> dh__dr_WF = -1 * Jh_weighted *
// dp_C__dp_W * dp_W__dr_WF;
// 	// 	Eigen::Matrix<double, 2, 3> dh__dtheta = -1 * Jh_weighted *
// dp_C__dp_W * dp_W__dtheta;
//   //
// 	// 	map_mat_t<2, 6, row_major_t> J1(jacobians[1]);
// 	// 	J1.block(0, 0, 2, 3) = dh__dr_WF;
// 	// 	J1.block(0, 3, 2, 3) = dh__dtheta;
// 	// 	if (!valid) {
// 	// 		J1.setZero();
// 	// 	}
// 	// }
//
// 	// // Jacobian w.r.t. sensor-camera extrinsic pose T_SCi
// 	// if (jacobians[2] != nullptr) {
// 	// 	mat_t<3, 3> dp_C__dp_S = C_SC.transpose();
// 	// 	mat_t<3, 3> dp_S__dr_SC = I(3);
// 	// 	mat_t<3, 3> dp_S__dtheta = -skew(C_SC * hp_C.head(3));
// 	// 	mat_t<2, 3> dh__dr_SC = Jh_weighted * dp_C__dp_S * dp_S__dr_SC;
// 	// 	mat_t<2, 3> dh__dtheta = Jh_weighted * dp_C__dp_S *
// dp_S__dtheta;
//   //
// 	// 	map_mat_t<2, 6> J2(jacobians[2]);
// 	// 	J2.block(0, 0, 2, 3) = dh__dr_SC;
// 	// 	J2.block(0, 3, 2, 3) = dh__dtheta;
// 	// 	if (!valid) {
// 	// 		J2.setZero();
// 	// 	}
// 	// }
//
// 	// // Jacobian w.r.t. camera intrinsics + distortion params
// 	// if (jacobians[3] != nullptr) {
// 	// 	map_mat_t<2, 8> J3(jacobians[3]);
// 	// 	J3 = -1 * J_cam_params;
// 	// }
//
//   return 0;
// }

/*****************************************************************************
 * Factor Graph
 ****************************************************************************/

void graph_free(graph_t &graph) {
  for (const auto &factor : graph.factors) {
    delete factor;
  }
  graph.factors.clear();

  for (const auto &kv : graph.variables) {
    delete kv.second;
  }
  graph.variables.clear();
}

size_t graph_next_variable_id(graph_t &graph) {
  const size_t id = graph.variables.size();
  return id;
}

size_t graph_next_factor_id(graph_t &graph) {
  const size_t id = graph.factors.size();
  return id;
}

size_t graph_add_pose(graph_t &graph,
                      const timestamp_t &ts,
                      const mat4_t &pose) {
  const auto id = graph_next_variable_id(graph);
  const auto pose_param = new pose_t{ts, id, pose};
  graph.variables.insert({id, pose_param});
  graph.problem.AddParameterBlock(pose_param->data(), pose_param->dimensions());
  return id;
}

size_t graph_add_landmark(graph_t &graph, const vec3_t &landmark) {
  const auto id = graph_next_variable_id(graph);
  const auto landmark_param = new landmark_t{id, landmark};
  graph.variables.insert({id, landmark_param});
  graph.problem.AddParameterBlock(landmark_param->data(),
                                  landmark_param->dimensions());
  return id;
}

size_t graph_add_factor(graph_t &graph,
                        factor_t *factor,
                        const std::vector<size_t> param_block_ids) {
  const size_t factor_id = graph_next_factor_id(graph);
  graph.factors.push_back(factor);

  std::vector<double *> param_blocks;
  for (const auto &id : param_block_ids) {
    param_blocks.push_back(graph.variables.at(id)->data());
  }
  graph.problem.AddResidualBlock(factor, nullptr, param_blocks);

  return factor_id;
}

size_t graph_add_ba_factor(graph_t &graph,
                           const timestamp_t &ts,
                           const vec2_t &z,
                           const size_t point_id,
                           const size_t pose_id) {
  const size_t factor_id = graph_next_factor_id(graph);
  const auto factor = new ba_factor_t{ts, factor_id, z};
  graph_add_factor(graph, factor, {point_id, pose_id});
  return factor_id;
}

int graph_solve(graph_t &graph) {
  ceres::Solve(graph.solver_options, &graph.problem, &graph.solver_summary);
  graph.solver_summary.FullReport();
  return 0;
}

} // namespace proto
