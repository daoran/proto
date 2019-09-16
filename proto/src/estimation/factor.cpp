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

void graph_free(graph_t &graph) {
  for (const auto &factor: graph.factors) {
    delete factor;
  }
  graph.factors.clear();

  for (const auto &kv: graph.variables) {
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
  graph.variables.insert({id, new pose_t{ts, id, pose}});
  graph.nb_poses++;
  return id;
}

size_t graph_add_landmark(graph_t &graph, const vec3_t &landmark) {
  const auto id = graph_next_variable_id(graph);
  graph.variables.insert({id, new landmark_t{id, landmark}});
  graph.nb_landmarks++;
  return id;
}

size_t graph_add_factor(graph_t &graph, factor_t *factor) {
  const size_t id = graph_next_factor_id(graph);
  graph.factors.push_back(factor);
  graph.residual_size += factor->residual_size;;
  return id;
}

size_t graph_add_ba_factor(graph_t &graph,
                           const timestamp_t &ts,
                           const vec2_t &z,
                           landmark_t *p_W,
                           pose_t *T_WS) {
  const size_t id = graph_next_factor_id(graph);
  const auto factor = new ba_factor_t{ts, id, z, p_W, T_WS};
  graph_add_factor(graph, factor);
  return id;
}

int graph_eval(graph_t &graph) {
  // Evaluate factors
  for (size_t i = 0; i < graph.factors.size(); i++) {
    const auto &factor = graph.factors[i];

    // Prepare residual and jacobian memory
    vecx_t r_i(factor->residual_size);
    std::vector<matx_t> J(factor->param_blocks.size());
    std::vector<double *> J_ptrs(factor->param_blocks.size());
    for (size_t i = 0; i < factor->param_blocks.size(); i++) {
      const size_t rows = factor->residual_size;
      const size_t cols = factor->param_sizes[i];
      J[i].resize(rows, cols);
      J_ptrs[i] = J[i].data();
    }

    // Evaluate
    if (factor->eval(r_i.data(), J_ptrs.data()) == 0) {
      graph.residuals.insert({factor, r_i});
      for (size_t i = 0; i < factor->param_blocks.size(); i++) {
        graph.jacobians[factor].emplace_back(J[i]);
      }
    }
  }

  return 0;
}

void graph_setup_problem(graph_t &graph, matx_t &J, vecx_t &r) {
  // Calculate parameter block column indices
  size_t pose_idx = 0;
  size_t landmark_idx = graph.nb_poses * 6;

  for (auto *factor : graph.factors) {
    for (auto *var : factor->param_blocks) {

      // Check if variable already has an index
      if (graph.param_index.find(var) != graph.param_index.end()) {
        continue;
      }

      // Set parameter block index
      if (dynamic_cast<landmark_t *>(var)) {
        graph.param_index[var] = landmark_idx;
        landmark_idx += 3;
      } else if (dynamic_cast<pose_t *>(var)) {
        graph.param_index[var] = pose_idx;
        pose_idx += 6;
      }
    }
  }

  // Form Jacobian J and residual r
  assert(graph.residual_size != 0);
  assert((graph.nb_poses + graph.nb_landmarks) != 0);
  size_t m = graph.residual_size;
  size_t n = (graph.nb_poses * 6) + (graph.nb_landmarks * 3);
  J.resize(m, n);
  J.setZero();
  r.resize(m);
  r.setZero();

  size_t row_idx = 0;
  for (auto *factor: graph.factors) {
    for (size_t i = 0; i < factor->param_blocks.size(); i++) {
      auto *param_block = factor->param_blocks[i];

      // Stack Jacobian J
      if (auto *var = dynamic_cast<pose_t *>(param_block)) {
        const auto col_idx = graph.param_index[var];
        J.block(row_idx, col_idx, 2, 6) = graph.jacobians[factor][i];
        // J.block(row_idx, col_idx, 2, 6) = 2 * ones(2, 6); //< debug
      }
      if (auto *var = dynamic_cast<landmark_t *>(param_block)) {
        const auto col_idx = graph.param_index[var];
        J.block(row_idx, col_idx, 2, 3) = graph.jacobians[factor][i];
        // J.block(row_idx, col_idx, 2, 3) = ones(2, 3); //< debug
      }
    }

    // Stack residual
    r.segment(row_idx, factor->residual_size) = graph.residuals[factor];

    // Update row index
    row_idx += factor->residual_size;
  }
}

void graph_update(graph_t &graph, const vecx_t &dx) {
  for (auto *factor : graph.factors) {
    for (size_t i = 0; i < factor->param_blocks.size(); i++) {
      auto *param_block = factor->param_blocks[i];
      size_t param_size = param_block->local_size;
      size_t param_index = graph.param_index[param_block];
      const vecx_t dx_i = dx.segment(param_index, param_size);
      param_block->plus(dx_i);
    }
  }
}

int graph_solve(graph_t &graph, int max_iter) {
  // Solve graph with Gause-Newton
  for (int iter = 0; iter < max_iter; iter++) {
    matx_t J;
    vecx_t r;
    graph_eval(graph);
    graph_setup_problem(graph, J, r);

    // Calculate state update dx
    const matx_t H = J.transpose() * J;
    const vecx_t b = -J.transpose() * r;
    const vecx_t dx = -H.inverse() * b;
    const double cost = 0.5 * r.transpose() * r;
    printf("iter: %d\tcost: %f\n", iter, cost);

    graph_update(graph, dx);
  }

  return 0;
}

} // namespace proto
