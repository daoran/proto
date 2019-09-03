#include "proto/estimation/factor.hpp"

mat2_t cam_error_t::intrinsics_point_jacobian(const mat3_t &K) const {
  mat2_t J = zeros(2, 2);
  J(1, 1) = K(1, 1);
  J(2, 2) = K(2, 2);
  return J;
}

matx_t cam_error_t::project_jacobian(const vec3_t &p_C) const {
  const double x = p_C(1);
  const double y = p_C(2);
  const double z = p_C(3);

  matx_t J = zeros(2, 3);
  J(1, 1) = 1.0 / z;
  J(2, 2) = 1.0 / z;
  J(1, 3) = -x / z*z;
  J(2, 3) = -y / z*z;
  return J;
}

mat3_t cam_error_t::camera_rotation_jacobian(const quat_t &q_WC,
                                const vec3_t &r_WC,
                                const vec3_t &p_W) const {
  return q_WC.toRotationMatrix().transpose() * skew(p_W - r_WC);
}

mat3_t cam_error_t::camera_translation_jacobian(const quat_t &q_WC) const {
  return -q_WC.toRotationMatrix().transpose();
}

mat3_t cam_error_t::target_point_jacobian(const quat_t &q_WC) const {
  return q_WC.toRotationMatrix().transpose();
}

void cam_error_t::eval(vecx_t &r, matx_t &J) const {
  // Setup
  const mat4_t T_WS = sensor_pose->T();
  const mat4_t T_SC = sensor_camera_extrinsic->T();
  const mat4_t T_WC = T_WS * T_SC;
  const quat_t q_WC = tf_quat(T_WS);
  const vec3_t r_WC = tf_trans(T_WS);
  const mat4_t T_CW = T_WC.inverse();

  // Loop over observations at time k
  size_t meas_idx = 0;
  for (size_t i = 0; i < landmarks.size(); i++) {
    // Transform point from world to camera frame
    const vec3_t p_W = landmarks[i]->p_W;
    const vec3_t p_C = (T_CW * p_W.homogeneous()).head(3);

    // Calculate residual
    const vec3_t x_C = normalize(p_C);
    const mat3_t K = I(3);
    const vec2_t pixel = (K * x_C).head(2);
    r.segment<2>(i * 2) = (z[i] - pixel);

    // Camera pose jacobian
    // -- Calculate jacobians
    const mat2_t J_K = intrinsics_point_jacobian(K);
    const matx_t J_P = project_jacobian(p_C);
    const mat3_t J_C = camera_rotation_jacobian(q_WC, r_WC, p_W);
    const mat3_t J_r = camera_translation_jacobian(q_WC);
    // -- Fill in the jacobian
    const size_t rs = meas_idx * 2;
    const matx_t J_cam_rot = -1 * J_K * J_P * J_C;
    const matx_t J_cam_pos = -1 * J_K * J_P * J_r;
    J.block(rs, 0, 2, 3) = J_cam_rot;
    J.block(rs, 3, 2, 3) = J_cam_rot;

    // // Point Jacobian
    // // -- Setup row start, row end, column start and column end
    // cs = (nb_poses * 6) + ((p_ids(j) - 1) * 3) + 1;
    // ce = cs + 2;
    // // -- Fill in the big jacobian
    // J_point = -1 * J_K * J_P * target_point_jacobian(q_WC);
    // J(rs:re, cs:ce) = J_point;
  }
}

void graph_delete(graph_t &graph) {
  // Delete landmarks
  {
    auto it = graph.landmarks.begin();
    while (it != graph.landmarks.end()) {
      delete graph.landmarks[it->first];
      it = graph.landmarks.erase(it);
    }
  }

  // Delete T_WS
  {
    auto it = graph.T_WS.begin();
    while (it != graph.T_WS.end()) {
      delete graph.T_WS[it->first];
      it = graph.T_WS.erase(it);
    }
  }

  // Delete T_SC
  {
    auto it = graph.T_SC.begin();
    while (it != graph.T_SC.end()) {
      delete graph.T_SC[it->first];
      it = graph.T_SC.erase(it);
    }
  }

  // Delete cam errors
  auto cam_it = graph.cam_errors.begin();
  while (cam_it != graph.cam_errors.end()) {
    auto err_it = graph.cam_errors[cam_it->first].begin();
    while (err_it != graph.cam_errors[cam_it->first].end()) {
      delete graph.cam_errors[cam_it->first][err_it->first];
      err_it = graph.cam_errors[cam_it->first].erase(err_it);
    }

    cam_it = graph.cam_errors.erase(cam_it);
  }
}

void graph_set_sensor_camera_extrinsic(graph_t &graph,
                                       const int cam_idx,
                                       const mat4_t &T_SC) {
  graph.T_SC[0] = new pose_t(0, cam_idx, T_SC);
}

size_t graph_add_cam_error(graph_t &graph,
                           const timestamp_t &ts,
                           const int cam_idx,
                           const vec2s_t &z,
                           const vec3s_t &p_W,
                           const mat4_t &T_WS) {
  // Add landmarks
  std::vector<landmark_t *> landmarks;
  for (size_t i = 0; i < z.size(); i++) {
    const size_t id = graph.landmarks.size();
    landmarks.push_back(new landmark_t{ts, id, p_W[i]});
    graph.landmarks.insert({id, landmarks[i]});
  }

  // Add sensor pose
  {
    const size_t id = graph.T_WS.size();
    const auto T_WS_k = new pose_t{ts, id, T_WS};
    graph.T_WS.insert({id, T_WS_k});
  }

  // Add reprojection error
  {
    const size_t id = graph.cam_errors[cam_idx].size();
    const auto error = new cam_error_t{ts, id, z, landmarks};
    graph.cam_errors[cam_idx].insert({id, error});
    return id;
  }
}

void graph_solve(graph_t &graph, int max_iter) {
  // Calculate number of cameras and measurements
  size_t nb_meas = 0;
  size_t nb_cams = graph.T_SC.size();
  size_t nb_poses = graph.T_WS.size();
  for (size_t cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
    nb_meas += graph.cam_errors[cam_idx].size();
  }

  // Setup residuals
  const size_t r_rows = nb_meas * 2;
  vecx_t r = zeros(r_rows, 1);

  // Setup Jacobian
  const size_t J_rows = nb_meas * 2;
  const size_t J_cols = (nb_poses * 6) + (nb_meas * 3);
  matx_t J = zeros(J_rows, J_cols);

  // Gauss-Newton
  for (int iter = 0; iter < max_iter; iter++) {
    // Evaluate and form global Jacobian
    for (const auto &cam_errors : graph.cam_errors) {
      for (const auto &kv : cam_errors.second) {
        const cam_error_t *error = kv.second;
        error->eval(r, J);
      }
    }

    // Calculate state update dx
    const matx_t H = J.transpose() * J;
    const vecx_t b = -J.transpose() * r;
    const vecx_t dx = -H.inverse() * b;
    const double cost = 0.5 * r.transpose() * r;
    printf("iter: %d\tcost: %f\n", iter, cost);

    // Update state vector
    // -- Sensor poses
    // for (size_t k = 0; k < nb_poses; k+=6) {
    //   // Update sensor rotation
    //   // const vec3_t dalpha = dx.segment<3>(k);
    //   // const quat_t dq = quat_delta(dalpha);
    //   // graph.T_WS[k]->q = dq * graph.T_WS[k]->q;
    //
    //   // Update sensor position
    //   // const vec3_t dr_WS = dx.segment<3>(k + 3);
    //   // graph.T_WS[k]->r += dr_WS;
    // }
    // -- Landmarks
    // for (size_t i = 0; i < nb_landmarks; i++) {
    //   const vec3_t dp_W = dx.segment<3>(i);
    //   graph.landmarks[i]->p_W += dp_W;
    // }

  }
}
