#include "proto/estimation/factor.hpp"

namespace proto {

/*****************************************************************************
 * Factor
 ****************************************************************************/

factor_t::factor_t() {}

factor_t::factor_t(const size_t id_, const timestamp_t &ts_)
  : id{id_}, ts{ts_} {}

factor_t::~factor_t() {}

void factor_t::addParam(size_t &id) {
  param_ids.push_back(id);
}

/*****************************************************************************
 * Camera Factor
 ****************************************************************************/

cam_factor_t::cam_factor_t(const timestamp_t &ts_,
                           const size_t id_,
                           const vec2_t &z_,
                           mat2_t info_)
    : factor_t{ts_, id_}, z{z_}, info{info_} {
  Eigen::LLT<mat2_t> llt_info(info);
  sq_info = llt_info.matrixL().transpose();
}

bool cam_factor_t::eval(double const * const *params,
                        double *residuals,
                        double **jacobians) const {
  assert(param_ids.size() == 3);

  // Map out parameters
  // -- Sensor world pose
  const quat_t q_WS{params[0][0], params[0][1], params[0][2], params[0][3]};
  const vec3_t r_WS{params[0][4], params[0][5], params[0][6]};
  const mat4_t T_WS = tf(q_WS, r_WS);
  // -- Sensor-camera relative pose
  const quat_t q_SC{params[1][0], params[1][1], params[1][2], params[1][3]};
  const vec3_t r_SC{params[1][4], params[1][5], params[1][6]};
  const mat4_t T_SC = tf(q_WS, r_WS);
  // -- World landmark
  const vec3_t p_W{params[2][0], params[2][1], params[2][2]};
  // -- Camera parameters
  const double* cam_params = params[3];

  // Transform point from world to camera frame
  const mat4_t T_WC = T_WS * T_SC;
  const mat4_t T_CW = T_WC.inverse();
  const vec3_t p_C = tf_point(T_CW, p_W);
  // -- Check validity of the point, simple depth test.
  if (fabs(p_C(2)) < 0.05) {
   return false;
  }

  // Calculate residual
  const vec3_t x_C = normalize(p_C);
  const mat3_t K = I(3);
  const vec2_t pixel = (K * x_C).head(2);
  const vec2_t error = z - pixel;
  map_vec_t<2> r(residuals);
  r = sq_info * error;

  // Calculate Jacobian
  if (jacobians == nullptr) {
    return true;
  }
  const mat3_t C_SC = tf_rot(T_SC);
  const mat3_t C_WS = tf_rot(T_WS);

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

// 	// Jacobian w.r.t. sensor-camera extrinsic pose T_SCi
// 	if (jacobians[1] != nullptr) {
// 		mat_t<3, 3> dp_C__dp_S = C_SC.transpose();
// 		mat_t<3, 3> dp_S__dr_SC = I(3);
// 		mat_t<3, 3> dp_S__dtheta = -skew(C_SC * hp_C.head(3));
// 		mat_t<2, 3> dh__dr_SC = Jh_weighted * dp_C__dp_S * dp_S__dr_SC;
// 		mat_t<2, 3> dh__dtheta = Jh_weighted * dp_C__dp_S *
// dp_S__dtheta;
//
// 		map_mat_t<2, 6> J2(jacobians[2]);
// 		J2.block(0, 0, 2, 3) = dh__dr_SC;
// 		J2.block(0, 3, 2, 3) = dh__dtheta;
// 		if (!valid) {
// 			J2.setZero();
// 		}
// 	}

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

graph_t::graph_t() {}

graph_t::~graph_t() {
  for (const auto &factor : factors) {
    delete factor;
  }
  factors.clear();

  for (const auto &kv : params) {
    delete kv.second;
  }
  params.clear();
}

size_t graph_t::next_param_id() {
  return params.size();
}

size_t graph_t::next_factor_id() {
  return factors.size();
}

size_t graph_add_pose(graph_t &graph,
                      const timestamp_t &ts,
                      const mat4_t &pose) {
  const auto id = graph.next_param_id();
  const auto pose_param = new pose_t{id, ts, pose};
  graph.params.insert({id, pose_param});
  return id;
}

size_t graph_add_landmark(graph_t &graph, const vec3_t &landmark) {
  const auto id = graph.next_param_id();
  const auto landmark_param = new landmark_t{id, landmark};
  graph.params.insert({id, landmark_param});
  return id;
}

size_t graph_add_factor(graph_t &graph, factor_t *factor) {
  const size_t factor_id = graph.next_factor_id();
  graph.factors.push_back(factor);

  // std::vector<double *> param_blocks;
  // for (const auto &id : param_block_ids) {
  //   param_blocks.push_back(graph.params.at(id)->data());
  // }

  return factor_id;
}

void graph_eval(graph_t &graph) {
  vecx_t r = zeros(graph.residual_size);
  matx_t J = zeros(graph.residual_size, graph.param_size);

  // Determine parameter order
  std::map<double *, size_t> param_idx;
  size_t param_counter = 0;

  for (const auto &factor : graph.factors) {
    for (size_t i = 0; i < factor->param_ids.size(); i++) {
      const auto param_id = factor->param_ids[i];
      const auto param = graph.params[param_id];
      param_idx[param->data()] = param_counter;
      param_counter += param->local_size;
    }
  }

  // // Loop over time
  // double *params[10] = {0};
  // for (const auto &factor : graph.factors) {
  //   for (size_t i = 0; i < factor->param_ids.size(); i++) {
  //     const auto param_id = factor->param_ids[i];
  //     params[i] = graph.params[param_id]->data();
  //   }
  //
  //   // factor->eval(params, );
  //   memset(params, '\0', sizeof(double *) * 10);
  // }
}

int graph_solve(graph_t &graph) {
  return 0;
}

} // namespace proto
