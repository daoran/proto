#include "proto/estimation/factor.hpp"

namespace proto {

static mat3_t camera_rotation_jacobian(const quat_t &q_WC,
																			 const vec3_t &r_WC,
																			 const vec3_t &p_W) {
  return q_WC.toRotationMatrix().transpose() * skew(p_W - r_WC);
}

static mat3_t camera_translation_jacobian(const quat_t &q_WC) {
  return -q_WC.toRotationMatrix().transpose();
}

static mat3_t target_point_jacobian(const quat_t &q_WC) {
  return q_WC.toRotationMatrix().transpose();
}

/*****************************************************************************
 * Bundle Adjustment Factor
 ****************************************************************************/

int ba_factor_t::eval(double *residuals, double **jacobians) const {
  // Transform point from world to camera frame
  const mat4_t T_CW = T_WC->T().inverse();
  const mat3_t C_CW = tf_rot(T_CW);
  const vec4_t hp_C = T_CW * p_W->vec().homogeneous();
  const vec3_t p_C{hp_C(0)/hp_C(3), hp_C(1)/hp_C(3), hp_C(2)/hp_C(3)};

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

int cam_factor_t::eval(double *residuals, double **jacobians) const {
  UNUSED(residuals);
  UNUSED(jacobians);
  // Transform point from world to camera frame
  const mat4_t T_WC = T_WS->T() * T_SC->T();
  const mat4_t T_CW = T_WC.inverse();
  const vec3_t p_C = (T_CW * p_W->vec().homogeneous()).head(3);

  // Check validity of the point, simple depth test.
  if (fabs(p_C(2)) < 0.05) {
   return -1;
  }

  // Calculate residual
  // const vec3_t x_C = normalize(p_C);
  // const mat3_t K = I(3);
  // const vec2_t pixel = (K * x_C).head(2);
  // vec2_t error = measurement - pixel;
  // map_vec_t<2> r(residuals);
  // r = sq_info * error;
  //
  // // Calculate Jacobian
  // if (jacobians == nullptr) {
  //   return 0;
  // }

	// // Jacobian w.r.t. sensor pose T_WS
	// if (jacobians[0] != nullptr) {
	// 	mat_t<3, 3> dp_C__dp_W = C_SC.transpose() * C_WS.transpose();
	// 	mat_t<3, 3> dp_W__dr_WS = I(3);
	// 	mat_t<3, 3> dp_W__dtheta = -skew(C_WS * hp_S.head(3));
	// 	mat_t<2, 3> dh_dr_WS = Jh_weighted * dp_C__dp_W * dp_W__dr_WS;
	// 	mat_t<2, 3> dh_dtheta = Jh_weighted * dp_C__dp_W * dp_W__dtheta;
  //
	// 	map_mat_t<2, 6, row_major_t> J0(jacobians[0]);
	// 	J0.block(0, 0, 2, 3) = dh_dr_WS;
	// 	J0.block(0, 3, 2, 3) = dh_dtheta;
	// 	if (!valid) {
	// 		J0.setZero();
	// 	}
	// }

  // // Jacobian w.r.t. fiducial pose T_WF
  // if (jacobians[1] != nullptr) {
	// 	Eigen::Matrix<double, 3, 3> dp_C__dp_W = C_SC.transpose() * C_WS.transpose();
	// 	Eigen::Matrix<double, 3, 3> dp_W__dr_WF = I(3);
	// 	Eigen::Matrix<double, 3, 3> dp_W__dtheta = -skew(C_WF * p_F_);
	// 	Eigen::Matrix<double, 2, 3> dh__dr_WF = -1 * Jh_weighted * dp_C__dp_W * dp_W__dr_WF;
	// 	Eigen::Matrix<double, 2, 3> dh__dtheta = -1 * Jh_weighted * dp_C__dp_W * dp_W__dtheta;
  //
	// 	map_mat_t<2, 6, row_major_t> J1(jacobians[1]);
	// 	J1.block(0, 0, 2, 3) = dh__dr_WF;
	// 	J1.block(0, 3, 2, 3) = dh__dtheta;
	// 	if (!valid) {
	// 		J1.setZero();
	// 	}
	// }

	// // Jacobian w.r.t. sensor-camera extrinsic pose T_SCi
	// if (jacobians[2] != nullptr) {
	// 	mat_t<3, 3> dp_C__dp_S = C_SC.transpose();
	// 	mat_t<3, 3> dp_S__dr_SC = I(3);
	// 	mat_t<3, 3> dp_S__dtheta = -skew(C_SC * hp_C.head(3));
	// 	mat_t<2, 3> dh__dr_SC = Jh_weighted * dp_C__dp_S * dp_S__dr_SC;
	// 	mat_t<2, 3> dh__dtheta = Jh_weighted * dp_C__dp_S * dp_S__dtheta;
  //
	// 	map_mat_t<2, 6> J2(jacobians[2]);
	// 	J2.block(0, 0, 2, 3) = dh__dr_SC;
	// 	J2.block(0, 3, 2, 3) = dh__dtheta;
	// 	if (!valid) {
	// 		J2.setZero();
	// 	}
	// }

	// // Jacobian w.r.t. camera intrinsics + distortion params
	// if (jacobians[3] != nullptr) {
	// 	map_mat_t<2, 8> J3(jacobians[3]);
	// 	J3 = -1 * J_cam_params;
	// }

  return 0;
}

/*****************************************************************************
 * Factor Graph
 ****************************************************************************/

size_t graph_add_pose(graph_t &graph,
                      const timestamp_t &ts,
                      const mat4_t &pose) {
  // if (graph.variables) {
  //
  // }

  const auto id = graph.variables.size();
  graph.variables.insert({id, new pose_t{ts, id, pose}});
  return id;
}

size_t graph_add_landmark(graph_t &graph, const vec3_t &landmark) {
  const auto id = graph.variables.size();
  graph.variables.insert({id, new landmark_t{id, landmark}});
  return id;
}

size_t graph_add_ba_factor(graph_t &graph,
                           const timestamp_t &ts,
                           const vec2_t &z,
                           const vec3_t &p_W,
                           const mat4_t &T_WC) {
  // Add variables
  const size_t lm_id = graph_add_landmark(graph, p_W);
  const size_t pose_id = graph_add_pose(graph, ts, T_WC);

  // Add factor
  const size_t id = graph.factors.size();
  const auto landmark = static_cast<landmark_t *>(graph.variables[lm_id]);
  const auto pose = static_cast<pose_t *>(graph.variables[pose_id]);
  const auto factor = new ba_factor_t{ts, id, z, landmark, pose};
  graph.factors.push_back(factor);

  // Keep track of sizes
  graph.residual_size += factor->residual_size;
  graph.param_size += factor->param_sizes[0];

  return id;
}

int graph_solve(graph_t &graph, int max_iter) {
  // Setup residuals
  vecx_t r = zeros(graph.residual_size, 1);

  // Setup Jacobian
  matx_t H = zeros(graph.param_size, graph.param_size);

  // Form Hessian matrix H
  for (const auto &factor : graph.factors) {

    for (size_t i = 0; i < factor->param_blocks.size(); i++) {
      for (size_t j = i + 1; j < factor->param_blocks.size(); j++) {

      }
    }


  }

  // // Gauss-Newton
  // for (int iter = 0; iter < max_iter; iter++) {
  //   // Evaluate and form global Jacobian
  //   // for (const auto &cam_factors : graph.cam_factors) {
  //   //   for (const auto &kv : cam_factors.second) {
  //   //     const cam_factor_t &error = kv.second;
  //   //     // error.eval(r, J);
  //   //   }
  //   // }
  //
  //   // Calculate state update dx
  //   const matx_t H = J.transpose() * J;
  //   const vecx_t b = -J.transpose() * r;
  //   const vecx_t dx = -H.inverse() * b;
  //   const double cost = 0.5 * r.transpose() * r;
  //   printf("iter: %d\tcost: %f\n", iter, cost);
  //
  //   // Update state vector
  //   // -- Sensor poses
  //   // for (size_t k = 0; k < nb_poses; k+=6) {
  //   //   // Update sensor rotation
  //   //   // const vec3_t dalpha = dx.segment<3>(k);
  //   //   // const quat_t dq = quat_delta(dalpha);
  //   //   // graph.T_WS[k]->q = dq * graph.T_WS[k]->q;
  //   //
  //   //   // Update sensor position
  //   //   // const vec3_t dr_WS = dx.segment<3>(k + 3);
  //   //   // graph.T_WS[k]->r += dr_WS;
  //   // }
  //   // -- Landmarks
  //   // for (size_t i = 0; i < nb_landmarks; i++) {
  //   //   const vec3_t dp_W = dx.segment<3>(i);
  //   //   graph.landmarks[i]->p_W += dp_W;
  //   // }
  //
  // }

  return 0;
}

} // namespace proto
