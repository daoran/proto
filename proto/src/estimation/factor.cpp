#include "proto/estimation/factor.hpp"

namespace proto {

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

int cam_error_t::eval(double *residuals, double **jacobians) const {
  // Transform point from world to camera frame
  const mat4_t T_WS = sensor_pose->T();
  const mat4_t T_SC = sensor_camera_extrinsic->T();
  const mat4_t T_WC = T_WS * T_SC;
  const mat4_t T_CW = T_WC.inverse();
  const vec3_t p_W = landmark->p_W;
  const vec3_t p_C = (T_CW * p_W.homogeneous()).head(3);

  // Calculate residual
  const vec3_t x_C = normalize(p_C);
  const mat3_t K = I(3);
  const vec2_t pixel = (K * x_C).head(2);
  Eigen::Map<Eigen::Matrix<double, 2, 1>> r(residuals);
  r.segment<2>(0) = z - pixel;

  // Calculate Jacobian
  if (jacobians == nullptr) {
    return 0;
  }

  return 0;
}

void graph_set_sensor_camera_extrinsic(graph_t &graph,
                                       const int cam_idx,
                                       const mat4_t &T_SC) {
  graph.T_SC[0] = pose_t(0, cam_idx, T_SC);
}

size_t graph_add_camera_error(graph_t &graph,
                              const timestamp_t &ts,
                              const int cam_idx,
                              const vec2_t &z,
                              const vec3_t &p_W,
                              const mat4_t &T_WS) {
  // Add landmarks
  const size_t lm_id = graph.landmarks.size();
  graph.landmarks.insert({lm_id, {ts, lm_id, p_W}});

  // Add sensor pose
  const size_t pose_id = graph.T_WS.size();
  graph.T_WS.insert({pose_id, {ts, pose_id, T_WS}});

  // Add cam error
  const size_t error_id = graph.cam_errors[cam_idx].size();
  graph.cam_errors[cam_idx].insert(
    {error_id, {ts, error_id, z, lookup(graph.landmarks, lm_id)}}
  );
  return error_id;
}

int graph_solve(graph_t &graph, int max_iter) {
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
    // for (const auto &cam_errors : graph.cam_errors) {
    //   for (const auto &kv : cam_errors.second) {
    //     const cam_error_t &error = kv.second;
    //     // error.eval(r, J);
    //   }
    // }

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

  return 0;
}

} // namespace proto
