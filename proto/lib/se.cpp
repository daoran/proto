#include "se.hpp"

namespace proto {

/****************************** PARAMETERS ***********************************/

void pose_print(const std::string &prefix, const pose_t &pose) {
  const quat_t q = pose.rot();
  const vec3_t r = pose.trans();

  printf("[%s] ", prefix.c_str());
  printf("q: (%f, %f, %f, %f)", q.w(), q.x(), q.y(), q.z());
  printf("\t");
  printf("r: (%f, %f, %f)\n", r(0), r(1), r(2));
}

void landmarks_print(const landmarks_t &landmarks) {
  printf("nb_landmarks: %zu\n", landmarks.size());
  printf("landmarks:\n");
  for (const auto &lm : landmarks) {
    const auto x = lm.param(0);
    const auto y = lm.param(1);
    const auto z = lm.param(2);
    printf("-- [%ld]: (%.2f, %.2f, %.2f)\n", lm.id, x, y, z);
  }
}

void keypoints_print(const keypoints_t &keypoints) {
  printf("nb_keypoints: %zu\n", keypoints.size());
  printf("keypoints:\n");
  for (size_t i = 0; i < keypoints.size(); i++) {
    printf("-- (%f, %f)\n", keypoints[i](0), keypoints[i](1));
  }
}

/******************************** FACTORS ************************************/

int check_jacobians(factor_t *factor,
                    const int param_idx,
                    const std::string &jac_name,
                    const real_t step_size,
                    const real_t threshold) {
  // Calculate baseline
  factor->eval();
  const vecx_t e = factor->residuals;
  const matx_t J = factor->jacobians[param_idx];

  // Numerical diff
  param_t *param = factor->params[param_idx];
  matx_t fdiff = zeros(e.rows(), param->local_size);
  for (long i = 0; i < param->local_size; i++) {
    // Perturb and evaluate
    param->perturb(i, step_size);
    factor->eval();
    auto e_prime = factor->residuals;
    param->perturb(i, -step_size);

    // Forward finite difference
    fdiff.block(0, i, e.rows(), 1) = (e_prime - e) / step_size;
  }

  return check_jacobian(jac_name, fdiff, J, threshold, true);
}

void imu_propagate(const imu_data_t &imu_data,
                   const vec3_t &g,
                   const vec_t<7> &pose_i,
                   const vec_t<9> &sb_i,
                   vec_t<7> &pose_j,
                   vec_t<9> &sb_j) {
  assert(imu_data.size() > 2);
  auto na = zeros(3, 1);
  auto ng = zeros(3, 1);
  pose_j = pose_i;
  sb_j = sb_i;

  quat_t q_WS{pose_i(0), pose_i(1), pose_i(2), pose_i(3)};
  vec3_t r_WS{pose_i(4), pose_i(5), pose_i(6)};
  vec3_t v_WS = sb_i.segment(0, 3);
  vec3_t ba = sb_i.segment(3, 3);
  vec3_t bg = sb_i.segment(6, 3);

  real_t dt = 0.0025;
  for (size_t k = 0; k < imu_data.timestamps.size(); k++) {
    // Calculate dt
    if ((k + 1) < imu_data.timestamps.size()) {
      dt = ns2sec(imu_data.timestamps[k + 1] - imu_data.timestamps[k]);
    }
    const real_t dt_sq = dt * dt;

    // Update position and velocity
    const vec3_t a = (imu_data.accel[k] - ba - na);
    r_WS += v_WS * dt + 0.5 * (q_WS * a) * dt_sq + (0.5 * g * dt_sq);
    v_WS += (q_WS * a) * dt + (g * dt);

    // Update rotation
    const vec3_t w = (imu_data.gyro[k] - bg - ng);
    const real_t scalar = 1.0;
    const vec3_t vector = 0.5 * w * dt;
    q_WS *= quat_t{scalar, vector(0), vector(1), vector(2)};
  }

  // Set results
  pose_j << q_WS.w(), q_WS.x(), q_WS.y(), q_WS.z(), r_WS;
  sb_j.segment(0, 3) = v_WS;
  sb_j.segment(3, 3) = ba;
  sb_j.segment(6, 3) = bg;
}


/********************************* GRAPH ************************************/

id_t graph_add_pose(graph_t &graph,
                    const timestamp_t &ts,
                    const vec_t<7> &pose,
                    const bool fixed) {
  const auto id = graph.next_param_id++;
  const auto param = new pose_t{id, ts, pose, fixed};
  graph.params.insert({id, param});
  return id;
}

id_t graph_add_pose(graph_t &graph,
                    const timestamp_t &ts,
                    const mat4_t &pose,
                    const bool fixed) {
  const auto id = graph.next_param_id++;
  const auto param = new pose_t{id, ts, pose, fixed};
  graph.params.insert({id, param});
  return id;
}

id_t graph_add_fiducial_pose(graph_t &graph,
                             const mat4_t &pose,
                             const bool fixed) {
  const auto id = graph.next_param_id++;
  const auto param = new fiducial_pose_t{id, pose, fixed};
  graph.params.insert({id, param});
  return id;
}

id_t graph_add_extrinsic(graph_t &graph,
                         const mat4_t &pose,
                         const bool fixed) {
  const auto id = graph.next_param_id++;
  const auto param = new extrinsic_t{id, pose, fixed};
  graph.params.insert({id, param});
  return id;
}

id_t graph_add_landmark(graph_t &graph,
                        const vec3_t &landmark,
                        const bool fixed) {
  const auto id = graph.next_param_id++;
  const auto param = new landmark_t{id, landmark, fixed};
  graph.params.insert({id, param});
  return id;
}

id_t graph_add_camera(graph_t &graph,
                      const int cam_index,
                      const int resolution[2],
                      const vecx_t &proj_params,
                      const vecx_t &dist_params,
                      bool fixed) {
  const auto id = graph.next_param_id++;
  const auto param = new camera_params_t{id, cam_index, resolution,
                                         proj_params, dist_params,
                                         fixed};
  graph.params.insert({id, param});
  return id;
}

id_t graph_add_speed_bias(graph_t &graph,
                          const timestamp_t &ts,
                          const vec3_t &v,
                          const vec3_t &ba,
                          const vec3_t &bg) {
  const auto id = graph.next_param_id++;
  const auto param = new sb_params_t{id, ts, v, ba, bg};
  graph.params.insert({id, param});
  return id;
}

id_t graph_add_speed_bias(graph_t &graph,
                          const timestamp_t &ts,
                          const vec_t<9> &sb) {
  const vec3_t &v = sb.head(3);
  const vec3_t &ba = sb.segment(3, 3);
  const vec3_t &bg = sb.segment(6, 3);
  return graph_add_speed_bias(graph, ts, v, ba, bg);
}

vecx_t graph_get_estimate(graph_t &graph, id_t id) {
  return graph.params[id]->param;
}

id_t graph_add_pose_factor(graph_t &graph,
                           const id_t pose_id,
                           const mat_t<6, 6> &covar) {
  // Create factor
  const id_t f_id = graph.next_factor_id++;
  auto param = graph.params[pose_id];
  auto factor = new pose_factor_t{f_id, covar, param};

  // Add factor to graph
  graph.factors[f_id] = factor;

  // Point params to factor
  param->factor_ids.push_back(f_id);

  return f_id;
}

id_t graph_add_camera_params_factor(graph_t &graph,
                                    const id_t cam_params_id,
                                    const matx_t &covar) {
  // Create factor
  const id_t f_id = graph.next_factor_id++;
  auto param = graph.params[cam_params_id];
  auto factor = new camera_params_factor_t{f_id, covar, param};

  // Add factor to graph
  graph.factors[f_id] = factor;

  // Point params to factor
  param->factor_ids.push_back(f_id);

  return f_id;
}

id_t graph_add_landmark_factor(graph_t &graph,
                               const id_t landmark_id,
                               const mat_t<3, 3> &covar) {
  // Create factor
  const id_t f_id = graph.next_factor_id++;
  auto param = graph.params[landmark_id];
  auto factor = new landmark_factor_t{f_id, covar, param};

  // Add factor to graph
  graph.factors[f_id] = factor;

  // Point params to factor
  param->factor_ids.push_back(f_id);

  return f_id;
}

id_t graph_add_imu_factor(graph_t &graph,
                          const int imu_index,
                          const timestamps_t &imu_ts,
                          const vec3s_t &imu_accel,
                          const vec3s_t &imu_gyro,
                          const id_t pose0_id,
                          const id_t sb0_id,
                          const id_t pose1_id,
                          const id_t sb1_id) {
  // Create factor
  const id_t f_id = graph.next_factor_id++;
  std::vector<param_t *> params{
    graph.params[pose0_id],
    graph.params[sb0_id],
    graph.params[pose1_id],
    graph.params[sb1_id]
  };
  auto factor = new imu_factor_t(f_id, imu_index, imu_ts,
                                 imu_gyro, imu_accel,
                                 I(15), params);

  // Add factor to graph
  graph.factors[f_id] = factor;

  // Point params to factor
  for (auto *param : params) {
    param->factor_ids.push_back(f_id);
  }

  return f_id;
}

// Note: this function does not actually perform marginalization, it simply
// marks it to be marginalized.
void graph_mark_param(graph_t &graph, const id_t param_id) {
  assert(graph.params.count(param_id) == 1);
  auto param = graph.params[param_id];
  param->marginalize = true;
  param->type = "marg_" + param->type;

  for (const auto factor_id : param->factor_ids) {
    graph.factors[factor_id]->marginalize = true;
  }
}

void graph_rm_param(graph_t &graph, const id_t param_id) {
  auto &param = graph.params[param_id];
  graph.params.erase(param_id);
  delete param;
}

void graph_rm_factor(graph_t &graph, const id_t factor_id) {
  auto &factor = graph.factors[factor_id];
  graph.factors.erase(factor->id);
  delete factor;
}

vecx_t graph_residuals(graph_t &graph) {
  // Calculate residual size
  std::vector<bool> factors_ok;
  size_t residuals_size = 0;
  for (const auto &kv : graph.factors) {
    auto factor = kv.second;

    if (factor->eval(false) == 0) {
      factors_ok.push_back(true);
      residuals_size += factor->residuals.size();
    } else {
      factors_ok.push_back(false);
    }
  }

  // Form residual vector
  vecx_t r = zeros(residuals_size, 1);
  size_t idx = 0;
  size_t k = 0;
  for (const auto &kv : graph.factors) {
    if (factors_ok[k++]) {
      auto &factor = kv.second;
      r.segment(idx, factor->residuals.size()) = factor->residuals;
      idx += factor->residuals.size();
    }
  }

  return r;
}

matx_t graph_jacobians(graph_t &graph, size_t *marg_size, size_t *remain_size) {
  // First pass: Determine what parameters we have
  std::unordered_set<id_t> param_tracker;
  std::unordered_map<std::string, int> param_counter;
  std::set<std::string> marg_param_types;
  *marg_size = 0;
  *remain_size = 0;

  for (const auto &kv : graph.factors) {
    auto &factor = kv.second;

    for (const auto &param : factor->params) {
      // Check if param is already tracked or fixed
      if (param_tracker.count(param->id) > 0 || param->fixed) {
        continue; // Skip this param
      }

      // Keep track of param blocks
      param_counter[param->type] += param->local_size;
      param_tracker.insert(param->id);

      // Change parameter type if marked for marginalization
      if (param->marginalize) {
        marg_param_types.insert(param->type);
        *marg_size += param->local_size;
      } else {
        *remain_size += param->local_size;
      }
    }
  }

  // Second pass: Assign jacobian order for each parameter and evaluate factor
  std::unordered_map<std::string, int> param_cs;  // Map param type to col index
  std::vector<std::string> param_order;

  // -- Setup param order
  for (const auto &param_type : marg_param_types) {
    param_order.push_back(param_type);
  }
  for (const auto &param_type : graph.param_order) {
    param_order.push_back(param_type);
  }

  // -- Check which param is not in defined param order
  for (int i = 0; i < (int) param_order.size(); i++) {
    if (param_counter.find(param_order[i]) == param_counter.end()) {
      FATAL("Param [%s] not found!", param_order[i].c_str());
    }
  }

  // -- Assign param start index
  for (int i = 0; i < (int) param_order.size(); i++) {
    auto param_i = param_order[i];
    param_cs[param_i] = 0;

    int j = i - 1;
    while (j > -1) {
      auto param_j = param_order[j];
      param_cs[param_order[i]] += param_counter[param_j];
      j--;
    }
  }

  // -- Assign param global index
  size_t residuals_size = 0;
  size_t params_size = 0;
  std::vector<bool> factor_ok;
  graph.param_index.clear();

  for (const auto &kv : graph.factors) {
    auto factor = kv.second;

    // Evaluate factor
    if (factor->eval() != 0) {
      factor_ok.push_back(false);
      continue; // Skip this factor's jacobians and residuals
    }
    residuals_size += factor->residuals.size();
    factor_ok.push_back(true);

    // Assign parameter order in jacobian
    for (const auto &param : factor->params) {
      // Check if param is already tracked or fixed
      if (graph.param_index.count(param->id) > 0 || param->fixed) {
        continue; // Skip this param
      }

      // Assign jacobian column index for parameter
      graph.param_index.insert({param->id, param_cs[param->type]});
      param_cs[param->type] += param->local_size;
      params_size += param->local_size;
    }
  }

  // Third pass: Form residuals and jacobians
  matx_t J = zeros(residuals_size, params_size);

  size_t rs = 0;
  size_t cs = 0;
  size_t i = 0;
  for (auto &kv : graph.factors) {
    const auto &factor = kv.second;
    if (factor_ok[i++] == false) {
      continue; // Skip this factor
    }

    // Form jacobian
    for (size_t j = 0; j < factor->params.size(); j++) {
      const auto &param = factor->params.at(j);
      const long rows = factor->residuals.size();
      const long cols = param->local_size;

      if (graph.param_index.count(param->id)) {
        cs = graph.param_index[param->id];
        J.block(rs, cs, rows, cols) = factor->jacobians[j];
      }
    }

    // Update residual start
    rs += factor->residuals.size();
  }

  return J;
}

void graph_eval(graph_t &graph, matx_t &H, vecx_t &g,
                size_t *marg_size, size_t *remain_size) {
  // First pass: Determine what parameters we have
  std::unordered_set<id_t> param_tracker;
  std::unordered_map<std::string, int> param_counter;
  std::set<std::string> marg_param_types;
  *marg_size = 0;
  *remain_size = 0;

  for (const auto &kv : graph.factors) {
    auto &factor = kv.second;

    for (const auto &param : factor->params) {
      // Check if param is already tracked or fixed
      if (param_tracker.count(param->id) > 0 || param->fixed) {
        continue; // Skip this param
      }

      // Keep track of param blocks
      param_counter[param->type] += param->local_size;
      param_tracker.insert(param->id);

      // Change parameter type if marked for marginalization
      if (param->marginalize) {
        marg_param_types.insert(param->type);
        *marg_size += param->local_size;
      } else {
        *remain_size += param->local_size;
      }
    }
  }

  // Second pass: Assign jacobian order for each parameter and evaluate factor
  std::unordered_map<std::string, int> param_cs;  // Map param type to col index
  std::vector<std::string> param_order;

  // -- Setup param order
  for (const auto &param_type : marg_param_types) {
    param_order.push_back(param_type);
  }
  for (const auto &param_type : graph.param_order) {
    param_order.push_back(param_type);
  }

  // -- Check which param is not in defined param order
  for (int i = 0; i < (int) param_order.size(); i++) {
    if (param_counter.find(param_order[i]) == param_counter.end()) {
      FATAL("Param [%s] not found!", param_order[i].c_str());
    }
  }

  // -- Assign param start index
  for (int i = 0; i < (int) param_order.size(); i++) {
    auto param_i = param_order[i];
    param_cs[param_i] = 0;

    int j = i - 1;
    while (j > -1) {
      auto param_j = param_order[j];
      param_cs[param_order[i]] += param_counter[param_j];
      j--;
    }
  }

  // -- Assign param global index
  size_t residuals_size = 0;
  size_t params_size = 0;
  std::vector<int> factor_ok;
  graph.param_index.clear();

  for (const auto &kv : graph.factors) {
    auto factor = kv.second;

    // Evaluate factor
    if (factor->eval() != 0) {
      factor_ok.push_back(0);
      continue; // Skip this factor's jacobians and residuals
    }
    residuals_size += factor->residuals.size();
    factor_ok.push_back(1);

    // Assign parameter order in jacobian
    for (const auto &param : factor->params) {
      // Check if param is already tracked or fixed
      if (graph.param_index.count(param->id) > 0 || param->fixed) {
        continue; // Skip this param
      }

      // Assign jacobian column index for parameter
      graph.param_index.insert({param->id, param_cs[param->type]});
      param_cs[param->type] += param->local_size;
      params_size += param->local_size;
    }
  }

  // Third pass: Form L.H.S and R.H.S of H dx = g
  H = zeros(params_size, params_size);
  g = zeros(params_size, 1);

  size_t k = 0;
  for (auto &kv : graph.factors) {
    const auto &factor = kv.second;
    if (factor_ok[k++] == 0) {
      continue; // Skip this factor
    }

    // Form Hessian H
    for (size_t i = 0; i < factor->params.size(); i++) {
      const auto &param_i = factor->params.at(i);
      const auto idx_i = graph.param_index[param_i->id];
      const auto size_i = param_i->local_size;
      const matx_t &J_i = factor->jacobians[i];

      for (size_t j = i; j < factor->params.size(); j++) {
        const auto &param_j = factor->params.at(j);
        const auto idx_j = graph.param_index[param_j->id];
        const auto size_j = param_j->local_size;
        const matx_t &J_j = factor->jacobians[j];

        if (i == j) {  // Diagonal
          H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
        } else {  // Off-diagonal
          H.block(idx_i, idx_j, size_i, size_j) += J_i.transpose() * J_j;
          H.block(idx_j, idx_i, size_j, size_i) =
            H.block(idx_i, idx_j, size_i, size_j).transpose();
        }
      }

      // Form R.H.S. vector g
      g.segment(idx_i, size_i) -= J_i.transpose() * factor->residuals;
    }
  }
}

vecx_t graph_get_state(const graph_t &graph) {
  size_t param_size = 0;
  for (const auto &kv : graph.params) {
    auto &param = kv.second;
    param_size += param->param.size();
  }

  size_t i = 0;
  vecx_t params{param_size};
  for (const auto &kv : graph.params) {
    auto &param = kv.second;
    params.segment(i, param->param.size()) = param->param;
    i += param->param.size();
  }

  return params;
}

void graph_set_state(graph_t &graph, const vecx_t &x) {
  size_t i = 0;
  for (const auto &kv : graph.params) {
    auto &param = kv.second;
    param->param = x.segment(i, param->param.size());
    i += param->param.size();
  }
}

void graph_print_params(const graph_t &graph) {
  for (const auto &kv : graph.params) {
    auto &param = kv.second;
    auto data = vec2str(param->param);
    printf("[%zu][%s]%s\n", param->id, param->type.c_str(), data.c_str());
  }
}

void graph_update(graph_t &graph, const vecx_t &dx, const size_t offset) {
  assert(dx.rows() > 0);

  for (const auto &kv: graph.param_index) {
    const auto &param_id = kv.first;
    const auto &param = graph.params[param_id];
    if (param->marginalize == false && param->fixed == false) {
      const auto index = kv.second - offset;
      assert((index + param->local_size) <= (size_t) dx.size());
      param->plus(dx.segment(index, param->local_size));
    }
  }
}

} // namespace proto
