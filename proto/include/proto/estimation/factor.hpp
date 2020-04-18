#ifndef PROTO_ESTIMATION_FACTOR_HPP
#define PROTO_ESTIMATION_FACTOR_HPP

#include "proto/core/core.hpp"

namespace proto {

/*****************************************************************************
 *                                FACTOR
 ****************************************************************************/

struct factor_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  size_t id = 0;
  std::vector<size_t> param_ids;
  matx_t info;
  vecx_t residuals;
  matxs_t jacobians;

  factor_t() {}
  factor_t(const size_t id_, const matx_t &info_) : id{id_}, info{info_} {}
  virtual ~factor_t() {}
  virtual bool eval(real_t const *const *parameters) = 0;
};

/*****************************************************************************
 *                              POSE FACTOR
 ****************************************************************************/

struct pose_factor_t : factor_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	const mat4_t pose_meas;

  pose_factor_t(const size_t id_, const mat4_t &pose_, const mat_t<6, 6> &info_)
		: factor_t{id_, info_}, pose_meas{pose_} {
    jacobians.push_back(zeros(6, 6));
	}

	bool eval(real_t const *const *params) {
    assert(param_ids.size() == 1);

		// Calculate delta pose
    const mat4_t pose_est = tf(params[0]);
		const mat4_t delta_pose = pose_meas * pose_est.inverse();

		// Calculate pose error
		const quat_t dq = tf_quat(delta_pose);
		const vec3_t dtheta = 2 * dq.coeffs().head<3>();
		residuals = zeros(6, 1);
		residuals.head<3>() = dtheta;
		residuals.tail<3>() = tf_trans(pose_meas) - tf_trans(pose_est);

		// Calculate jacobian
		// clang-format off
    jacobians[0].setIdentity();
    jacobians[0] *= -1.0;
		mat3_t dq_mul_xyz;
		dq_mul_xyz << dq.w(), -dq.z(), dq.y(),
                  dq.z(), dq.w(), -dq.x(),
                  -dq.y(), dq.x(), dq.w();
    jacobians[0].block<3, 3>(0, 0) = -dq_mul_xyz;
		// clang-format on

		return true;
	}
};


/*****************************************************************************
 *                               BA FACTOR
 ****************************************************************************/

template <typename CM>
struct ba_factor_t : factor_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const timestamp_t ts = 0;
  const int cam_index = 0;
  const int img_w = 0;
  const int img_h = 0;
  const vec2_t z{0.0, 0.0};

  ba_factor_t(const size_t id_,
              const timestamp_t &ts_,
              const int cam_index_,
              const int img_w_,
              const int img_h_,
              const vec2_t &z_,
              const mat2_t &info_ = I(2))
      : factor_t{id_, info_},
        ts{ts_}, cam_index{cam_index_},
        img_w{img_w_}, img_h{img_h_},
        z{z_} {
    jacobians.push_back(zeros(2, 6));  // T_WC
    jacobians.push_back(zeros(2, CM::proj_params_size));  // Projection model
    jacobians.push_back(zeros(2, CM::dist_params_size));  // Distortion model
    jacobians.push_back(zeros(2, 3));  // p_W
  }

  bool eval(real_t const *const *params) {
    assert(param_ids.size() == 4);

    // Map out parameters
    const mat4_t T_WC = tf(params[0]);
    const CM cm{img_w, img_h, params[1], params[2]};
    const vec3_t p_W{params[3]};

    // Transform point from world to camera frame
    const mat4_t T_CW = T_WC.inverse();
    const vec3_t p_C = tf_point(T_CW, p_W);

    // Project point in camera frame to image plane
    vec2_t z_hat;
    mat_t<2, 3> J_h;
    int retval = cm.project(p_C, z_hat, J_h);
    if (retval != 0) {
      LOG_ERROR("Failed to project point!");
      switch (retval) {
      case -1: LOG_ERROR("Point is not infront of camera!"); break;
      case -2: LOG_ERROR("Projected point is outside the image plane!"); break;
      }

			jacobians[0] = zeros(2, 6);  // T_WC
			jacobians[1] = zeros(2, CM::proj_params_size);  // Projection model
			jacobians[2] = zeros(2, CM::dist_params_size);  // Distortion model
			jacobians[3] = zeros(2, 3);  // p_W
      return false;
    }

    // Calculate residual
    residuals = z - z_hat;

    // Calculate Jacobians
    const vec2_t p{p_C(0) / p_C(2), p_C(1) / p_C(2)};
    const vec2_t p_dist = cm.distortion.distort(p);
    const mat3_t C_WC = tf_rot(T_WC);
    const mat3_t C_CW = C_WC.transpose();
    const vec3_t r_WC = tf_trans(T_WC);

    // -- Jacobian w.r.t. sensor pose T_WS
    jacobians[0].block(0, 0, 2, 3) = -1 * J_h * C_CW * skew(p_W - r_WC);
    jacobians[0].block(0, 3, 2, 3) = -1 * J_h * -C_CW;
    // -- Jacobian w.r.t. projection model
    jacobians[1] = -1 * cm.J_proj(p_dist);
    // -- Jacobian w.r.t. distortion model
    jacobians[2] = -1 * cm.J_dist(p);
    // -- Jacobian w.r.t. landmark
    jacobians[3] = -1 * J_h * C_CW;

    return 0;
  }
};

/*****************************************************************************
 *                             CAMERA FACTOR
 ****************************************************************************/

template <typename CM>
struct cam_factor_t : factor_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const timestamp_t ts = 0;
  const int cam_index = 0;
  const int img_w = 0;
  const int img_h = 0;
  const vec2_t z{0.0, 0.0};

  cam_factor_t(const size_t id_,
               const timestamp_t &ts_,
               const int cam_index_,
               const int img_w_,
               const int img_h_,
               const vec2_t &z_,
               const mat2_t &info_ = I(2))
      : factor_t{id_, info_},
        ts{ts_}, cam_index{cam_index_},
        img_w{img_w_}, img_h{img_h_},
        z{z_} {
    jacobians.push_back(zeros(2, 6));  // T_WS
    jacobians.push_back(zeros(2, 6));  // T_SC
    jacobians.push_back(zeros(2, 3));  // p_W
    jacobians.push_back(zeros(2, 4));  // Camera model
    jacobians.push_back(zeros(2, 4));  // Distortion model
  }

  bool eval(real_t const *const *params) {
    assert(param_ids.size() == 5);

    // Map out parameters
    const mat4_t T_WS = tf(params[0]);
    const mat4_t T_SC = tf(params[1]);
    const CM cam_model{img_w, img_h, params[2], params[3]};
    const vec3_t p_W{params[4]};

    // Transform point from world to camera frame
    const mat4_t T_WC = T_WS * T_SC;
    const mat4_t T_CW = T_WC.inverse();
    const vec3_t p_C = tf_point(T_CW, p_W);

    // Project point in camera frame to image plane
    vec2_t z_hat;
    mat_t<2, 3> J_h;
    int retval = cam_model.project(p_C, z_hat, J_h);
    if (retval != 0) {
      LOG_ERROR("Failed to project point!");
      switch (retval) {
      case -1: LOG_ERROR("Point is not infront of camera!"); break;
      case -2: LOG_ERROR("Projected point is outside the image plane!"); break;
      }
      return false;
    }

    // Calculate residual
    residuals = z - z_hat;

    // Calculate Jacobians
    const mat3_t C_SC = tf_rot(T_SC);
    const mat3_t C_CS = C_SC.transpose();
    const mat3_t C_WS = tf_rot(T_WS);
    const mat3_t C_SW = C_WS.transpose();
    const mat3_t C_CW = C_CS * C_SW;
    const vec3_t r_WS = tf_trans(T_WS);
    const vec2_t p{p_C(0) / p_C(2), p_C(1) / p_C(2)};
    const vec2_t p_dist = cam_model.distortion.distort(p);

    // -- Jacobian w.r.t. sensor pose T_WS
    jacobians[0].block(0, 0, 2, 3) = -1 * J_h * C_CS * C_SW * skew(p_W - r_WS);
    jacobians[0].block(0, 3, 2, 3) = -1 * J_h * C_CS * -C_SW;
    // -- Jacobian w.r.t. sensor-camera extrinsic pose T_SCi
    jacobians[1].block(0, 0, 2, 3) = -1 * J_h * C_CS * skew(C_SC * p_C);
    jacobians[1].block(0, 3, 2, 3) = -1 * J_h * -C_CS;
    // -- Jacobian w.r.t. camera model
    jacobians[2] = -1 * cam_model.J_proj(p_dist);
    // -- Jacobian w.r.t. distortion model
    jacobians[3] = -1 * cam_model.J_dist(p);
    // -- Jacobian w.r.t. landmark
    jacobians[4] = -1 * J_h * C_CW;

    return 0;
  }
};

/*****************************************************************************
 *                              IMU FACTOR
 ****************************************************************************/

struct imu_factor_t : factor_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const timestamps_t imu_ts;
  const vec3s_t imu_accel;
  const vec3s_t imu_gyro;
  const vec3_t g{0.0, 0.0, 9.81};

  mat_t<15, 15> P = zeros(15, 15);  // Covariance matrix
  mat_t<12, 12> Q = zeros(12, 12);  // noise matrix
  mat_t<15, 15> F = zeros(15, 15);  // Transition matrix

  // Delta position, velocity and rotation between timestep i and j
  // (i.e start and end of imu measurements)
  vec3_t dp{0.0, 0.0, 0.0};
  vec3_t dv{0.0, 0.0, 0.0};
  quat_t dq{1.0, 0.0, 0.0, 0.0};

  // Accelerometer and gyroscope biases
  vec3_t bg{0.0, 0.0, 0.0};
  vec3_t ba{0.0, 0.0, 0.0};

  imu_factor_t(const size_t id_,
               const timestamps_t imu_ts_,
               const vec3s_t imu_accel_,
               const vec3s_t imu_gyro_ ,
               const mat_t<15, 15> &info_ = I(15))
      : factor_t{id_, info_},
        imu_ts{imu_ts_},
        imu_accel{imu_accel_},
        imu_gyro{imu_gyro_} {
    residuals = zeros(15, 1);
    jacobians.push_back(zeros(15, 6));  // T_WS at timestep i
    jacobians.push_back(zeros(15, 9));  // Speed and bias at timestep i
    jacobians.push_back(zeros(15, 6));  // T_WS at timestep j
    jacobians.push_back(zeros(15, 9));  // Speed and bias at timestep j

    propagate(imu_ts_, imu_accel_, imu_gyro_);
  }

  void reset() {
    P = zeros(15, 15);
    F = zeros(15, 15);

    dp = zeros(3);
    dv = zeros(3);
    dq = quat_t{1.0, 0.0, 0.0, 0.0};
    ba = zeros(3);
    bg = zeros(3);
  }

  void propagate(const timestamps_t &ts,
                 const vec3s_t &a_m,
                 const vec3s_t &w_m) {
    assert(w_m.size() == a_m.size());

    real_t dt_prev = ns2sec(ts[1] - ts[0]);
    for (size_t i = 0; i < w_m.size(); i++) {
      // Calculate dt
      real_t dt = 0.0;
      if ((i + 1) < w_m.size()) {
        dt = ns2sec(ts[i + 1] - ts[i]);
        dt_prev = dt;
      } else {
        dt = dt_prev;
      }
      // printf("i: %zu\n", i);

      // Update relative position and velocity
      dp = dp + dv * dt + 0.5 * (dq * (a_m[i] - ba)) * dt * dt;
      dv = dv + (dq * (a_m[i] - ba)) * dt;

      // Update relative rotation
      const real_t scalar = 1.0;
      const vec3_t vector = 0.5 * (w_m[i] - bg) * dt;
      const quat_t dq_i{scalar, vector(0), vector(1), vector(2)};
      dq = dq * dq_i;

      // Transition matrix F
      const mat3_t C_ji = dq.toRotationMatrix();
      mat_t<15, 15> F_i = zeros(15, 15);
      F_i.block<3, 3>(0, 3) = I(3);
      F_i.block<3, 3>(3, 6) = -C_ji * skew(a_m[i] - ba);
      F_i.block<3, 3>(3, 9) = -C_ji;
      F_i.block<3, 3>(6, 6) = -skew(w_m[i] - bg);
      F_i.block<3, 3>(6, 12) = -I(3);

      // Input matrix G
      mat_t<15, 12> G_i = zeros(15, 12);
      G_i.block<3, 3>(3, 0) = -C_ji;
      G_i.block<3, 3>(6, 3) = -I(3);
      G_i.block<3, 3>(9, 6) = I(3);
      G_i.block<3, 3>(12, 9) = I(3);

      // Update covariance matrix
      const mat_t<15, 15> I_Fi_dt = (I(15) + F * dt);
      const mat_t<15, 12> Gi_dt = (G_i * dt);
      P = I_Fi_dt * P * I_Fi_dt.transpose() + Gi_dt * Q * Gi_dt.transpose();

      // Update Jacobian
      F = I_Fi_dt * F;
    }
  }

  bool eval(real_t const *const *params) {
    // Map out parameters
    // -- Sensor pose at timestep i
    const mat4_t T_i = tf(params[0]);
    const mat3_t C_i = tf_rot(T_i);
    const mat3_t C_i_inv = C_i.transpose();
    const quat_t q_i = tf_quat(T_i);
    const vec3_t r_i = tf_trans(T_i);
    // -- Speed and bias at timestamp i
    const vec_t<9> sb_i{params[1]};
    const vec3_t v_i = sb_i.segment<3>(0);
    const vec3_t ba_i = sb_i.segment<3>(3);
    const vec3_t bg_i = sb_i.segment<3>(6);
    // -- Sensor pose at timestep j
    const mat4_t T_j = tf(params[2]);
    const quat_t q_j = tf_quat(T_j);
    const vec3_t r_j = tf_trans(T_j);
    // -- Speed and bias at timestep j
    const vec_t<9> sb_j{params[3]};
    const vec3_t v_j = sb_j.segment<3>(0);
    const vec3_t ba_j = sb_j.segment<3>(3);
    const vec3_t bg_j = sb_j.segment<3>(6);

    // Obtain Jacobians for gyro and accel bias
    const mat3_t dp_dbg = F.block<3, 3>(0, 9);
    const mat3_t dp_dba = F.block<3, 3>(0, 12);
    const mat3_t dv_dbg = F.block<3, 3>(3, 9);
    const mat3_t dv_dba = F.block<3, 3>(3, 12);
    const mat3_t dq_dbg = F.block<3, 3>(6, 12);

    // Calculate residuals
    const real_t dt_ij = ns2sec(imu_ts.back() - imu_ts.front());
    const real_t dt_ij_sq = dt_ij * dt_ij;
    const vec3_t dbg = bg_i - bg;
    const vec3_t dba = ba_i - ba;
    const vec3_t alpha = dp + dp_dbg * dbg + dp_dba * dba;
    const vec3_t beta = dv + dv_dbg * dbg + dv_dba * dba;
    const quat_t gamma = dq * quat_delta(dq_dbg * dbg);

    const quat_t q_i_inv = q_i.inverse();
    const quat_t q_j_inv = q_j.inverse();
    const quat_t gamma_inv = gamma.inverse();

    // clang-format off
    residuals << C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g * dt_ij_sq) - alpha,
                 C_i_inv * (v_j - v_i + g * dt_ij) - beta,
                 2.0 * (gamma_inv * (q_i_inv * q_j)).vec(),
                 ba_j - ba_i,
                 bg_j - bg_i;
    // clang-format on

    // Calculate jacobians
    // clang-format off
    // -- Sensor pose at i Jacobian
    jacobians[0] = zeros(15, 6);
    jacobians[0].block<3, 3>(0, 0) = skew(C_i_inv * (r_j - r_i - v_i * dt_ij + 0.5 * g * dt_ij_sq));
    jacobians[0].block<3, 3>(0, 3) = -C_i_inv;
    jacobians[0].block<3, 3>(3, 0) = skew(C_i_inv * (v_j - v_i + g * dt_ij));
    jacobians[0].block<3, 3>(6, 0) = -quat_mat_xyz(quat_lmul(q_j_inv * q_i) * quat_rmul(gamma));
    // -- Speed and bias at i Jacobian
    jacobians[1] = zeros(15, 9);
    jacobians[1].block<3, 3>(0, 0) = -C_i_inv * dt_ij;
    jacobians[1].block<3, 3>(0, 3) = -dp_dba;
    jacobians[1].block<3, 3>(0, 6) = -dp_dbg;
    jacobians[1].block<3, 3>(3, 0) = -C_i_inv;
    jacobians[1].block<3, 3>(3, 3) = -dv_dba;
    jacobians[1].block<3, 3>(3, 6) = -dv_dbg;
    jacobians[1].block<3, 3>(9, 3) = -I(3);
    jacobians[1].block<3, 3>(12, 6) = -I(3);
    // -- Sensor pose at j Jacobian
    jacobians[2] = zeros(15, 6);
    jacobians[2].block<3, 3>(0, 3) = C_i_inv;
    jacobians[2].block<3, 3>(6, 0) = quat_lmul_xyz(gamma_inv * q_i_inv * q_j_inv);
    // -- Speed and bias at j Jacobian
    jacobians[3] = zeros(15, 9);
    jacobians[3].block<3, 3>(3, 0) = C_i_inv;
    jacobians[3].block<3, 3>(9, 3) = I(3);
    jacobians[3].block<3, 3>(12, 6) = I(3);
    // clang-format on

    return true;
  }
};

/*****************************************************************************
 *                              FACTOR GRAPH
 ****************************************************************************/

struct graph_t {
  std::unordered_map<size_t, param_t *> params;
  std::unordered_map<param_t *, size_t> param_index;
  std::deque<factor_t *> factors;

  graph_t() {}

  ~graph_t() {
    for (const auto &factor : factors) {
      delete factor;
    }
    factors.clear();

    for (const auto &kv : params) {
      delete kv.second;
    }
    params.clear();
  }
};

size_t graph_add_pose(graph_t &graph,
                      const timestamp_t &ts,
                      const mat4_t &pose) {
  const auto id = graph.params.size();
  const auto param = new pose_t{id, ts, pose};
  graph.params.insert({id, param});
  return id;
}

size_t graph_add_landmark(graph_t &graph, const vec3_t &landmark) {
  const auto id = graph.params.size();
  const auto param = new landmark_t{id, landmark};
  graph.params.insert({id, param});
  return id;
}

size_t graph_add_proj_params(graph_t &graph,
                             const int cam_index,
                             const vecx_t &params,
                             bool fixed = false) {
  const auto id = graph.params.size();
  const auto param = new proj_param_t{id, cam_index, params};
  param->fixed = fixed;
  graph.params.insert({id, param});
  return id;
}

size_t graph_add_dist_params(graph_t &graph,
                             const int cam_index,
                             const vecx_t &params,
                             bool fixed = false) {
  const auto id = graph.params.size();
  const auto param = new dist_param_t{id, cam_index, params};
  param->fixed = fixed;
  graph.params.insert({id, param});
  return id;
}

size_t graph_add_sb_params(graph_t &graph,
                           const timestamp_t &ts,
                           const vec3_t &v,
                           const vec3_t &ba,
                           const vec3_t &bg) {
  const auto id = graph.params.size();
  const auto param = new sb_param_t{id, ts, v, ba, bg};
  graph.params.insert({id, param});
  return id;
}

size_t graph_add_pose_factor(graph_t &graph,
                             const size_t pose_id,
                             const mat4_t &T,
                             const mat_t<6, 6> &info = I(6)) {
  // Create factor
  const auto f_id = graph.factors.size();
  auto factor = new pose_factor_t{f_id, T, info};
  factor->param_ids.push_back(pose_id);

  // Add factor to graph
  graph.factors.push_back(factor);

  return f_id;
}

template <typename CM>
size_t graph_add_ba_factor(graph_t &graph,
                           const timestamp_t &ts,
                           const int cam_idx,
                           const int img_w,
                           const int img_h,
                           const size_t cam_pose_id,
                           const size_t landmark_id,
                           const size_t proj_param_id,
                           const size_t dist_param_id,
                           const vec2_t &z,
                           const mat2_t &info = I(2)) {
  // Create factor
  const auto f_id = graph.factors.size();
  auto factor = new ba_factor_t<CM>{
    ts, f_id,
    cam_idx, img_w, img_h,
    z, info
  };
  factor->param_ids.push_back(cam_pose_id);
  factor->param_ids.push_back(proj_param_id);
  factor->param_ids.push_back(dist_param_id);
  factor->param_ids.push_back(landmark_id);

  // Add factor to graph
  graph.factors.push_back(factor);

  return f_id;
}

template <typename CM>
size_t graph_add_cam_factor(graph_t &graph,
                            const timestamp_t &ts,
                            const int cam_idx,
                            const int img_w,
                            const int img_h,
                            const size_t sensor_pose_id,
                            const size_t imu_cam_pose_id,
                            const size_t landmark_id,
                            const size_t proj_param_id,
                            const size_t dist_param_id,
                            const vec2_t &z,
                            const mat2_t &info = I(2)) {
  // Create factor
  const auto f_id = graph.factors.size();
  auto factor = new cam_factor_t<CM>{
    ts, f_id,
    cam_idx, img_w, img_h,
    z, info
  };
  factor->param_ids.push_back(sensor_pose_id);
  factor->param_ids.push_back(imu_cam_pose_id);
  factor->param_ids.push_back(landmark_id);
  factor->param_ids.push_back(proj_param_id);
  factor->param_ids.push_back(dist_param_id);

  // Add factor to graph
  graph.factors.push_back(factor);

  return f_id;
}

size_t graph_add_imu_factor(graph_t &graph,
                            const int imu_index,
                            const timestamps_t &imu_ts,
                            const vec3s_t &imu_accel,
                            const vec3s_t &imu_gyro,
                            const size_t pose0_id,
                            const size_t sb0_id,
                            const size_t pose1_id,
                            const size_t sb1_id) {
  // Create factor
  const auto f_id = graph.factors.size();
  auto factor = new imu_factor_t(imu_index, imu_ts, imu_gyro, imu_accel);
  factor->param_ids.push_back(pose0_id);
  factor->param_ids.push_back(sb0_id);
  factor->param_ids.push_back(pose1_id);
  factor->param_ids.push_back(sb1_id);

  // Add factor to graph
  graph.factors.push_back(factor);

  return f_id;
}

void graph_eval(graph_t &graph, vecx_t &r) {
  for (const auto &factor : graph.factors) {
    std::vector<real_t *> factor_params;
    for (size_t i = 0; i < factor->param_ids.size(); i++) {
      const auto param_id = factor->param_ids[i];
      const auto &param = graph.params.at(param_id);
			factor_params.push_back(param->data());
    }
    factor->eval(factor_params.data());
  }
}

void graph_eval(graph_t &graph, vecx_t &r, matx_t &J) {
	// First pass: Determine what parameters we have
  std::unordered_set<param_t *> param_tracker;
  size_t pose_param_size = 0;
  size_t landmark_param_size = 0;
  size_t proj_param_size = 0;
  size_t dist_param_size = 0;

	for (const auto &factor : graph.factors) {
		// Obtain params for factor
		for (size_t i = 0; i < factor->param_ids.size(); i++) {
			const auto param_id = factor->param_ids[i];
			const auto &param = graph.params.at(param_id);

			// Check if param has already been tracked or fixed
			if (param_tracker.count(param) > 0 || param->fixed) {
				continue;
			}

			// Keep track of param blocks
			switch (param->type) {
      case POSE: pose_param_size += param->local_size; break;
			case PROJECTION: proj_param_size += param->local_size; break;
			case DISTORTION: dist_param_size += param->local_size; break;
			case LANDMARK: landmark_param_size += param->local_size; break;
			default: FATAL("Unsupported param type [%d]!", param->type); break;
			}
      param_tracker.insert(param);
		}
	}

  // Second pass: Assign jacobian order for each parameter and evaluate factor
  size_t residuals_size = 0;
  size_t params_size = 0;
	size_t pose_cs = 0;
	size_t proj_cs = pose_param_size;
	size_t dist_cs = proj_cs + proj_param_size;
	size_t landmark_cs = dist_cs + dist_param_size;

	graph.param_index.clear();
  for (const auto &factor : graph.factors) {
    std::vector<real_t *> factor_params;
    for (size_t i = 0; i < factor->param_ids.size(); i++) {
      const auto param_id = factor->param_ids[i];
      const auto &param = graph.params.at(param_id);
			factor_params.push_back(param->data());

      // Check if param has already been tracked
      if (graph.param_index.count(param) > 0 || param->fixed) {
        continue;
      }

			// Assign jacobian column index for parameter
			switch (param->type) {
			case POSE:
				graph.param_index.insert({param, pose_cs});
				pose_cs += param->local_size;
				break;
      case PROJECTION:
				graph.param_index.insert({param, proj_cs});
				proj_cs += param->local_size;
				break;
      case DISTORTION:
				graph.param_index.insert({param, dist_cs});
				dist_cs += param->local_size;
				break;
      case LANDMARK:
				graph.param_index.insert({param, landmark_cs});
				landmark_cs += param->local_size;
				break;
			default:
				FATAL("Unsupported param type!");
				break;
			}
      params_size += param->local_size;
    }

		// Evaluate factor
    factor->eval(factor_params.data());
		residuals_size += factor->residuals.size();
  }

  // Third pass: Form residuals and jacobians
  r.resize(residuals_size);
  J = zeros(residuals_size, params_size);

  size_t rs = 0;
  size_t cs = 0;
  for (const auto &factor : graph.factors) {
    for (size_t i = 0; i < factor->param_ids.size(); i++) {
      const auto param_id = factor->param_ids[i];
      const auto &param = graph.params.at(param_id);
      const long rows = factor->residuals.size();
      const long cols = param->local_size;

      if (graph.param_index.count(param)) {
        cs = graph.param_index[param];
        J.block(rs, cs, rows, cols) = factor->jacobians[i];
        // J.block(rs, cs, rows, cols) = ones(rows, cols);
      }
    }
    r.segment(rs, factor->residuals.size()) = factor->residuals;
    rs += factor->residuals.size();
  }
}

void graph_update(graph_t &graph, const vecx_t &dx) {
  for (const auto &kv: graph.param_index) {
    const auto &param = kv.first;
    const auto index = kv.second;
    param->plus(dx.segment(index, param->local_size));
  }
	graph.param_index.clear();
}

/*****************************************************************************
 *                               TINY SOLVER
 ****************************************************************************/

struct tiny_solver_t {
  // Optimization parameters
  int max_iter = 10;
  real_t lambda = 1e8;

  tiny_solver_t() {}

  int solve(graph_t &graph)  {
    real_t lambda_k = lambda;
    real_t cost = 0.0;
    real_t cost_prev = 0.0;

		struct timespec t_start = tic();
    for (int iter = 0; iter < max_iter; iter++) {
      vecx_t e;
      matx_t E;
			graph_eval(graph, e, E);

      // Form weight matrix
      // W = diag(repmat(sigma, data->nb_measurements, 1));

      // Solve Gauss-Newton system [H dx = g]: Solve for dx
      matx_t H = E.transpose() * E; // Hessian approx: H = J^t J
      H = H + lambda_k * I(H.rows()); // LM dampening
      // matx_t H_diag = (H.diagonal().asDiagonal());
      // H = H + lambda * H_diag;			// R. Fletcher trust region mod
      const vecx_t g = -E.transpose() * e;
      // printf("cond(H): %f\n", cond(H));
      const vecx_t dx = H.ldlt().solve(g);   // Cholesky decomp

      // vecx_t g = -E.transpose() * e;
      // matx_t H = E.transpose() * E;
      // vecx_t p = (H.diagonal() + vecx_t::Constant(H.cols(), 10)).cwiseSqrt().cwiseInverse();
      // H = p.asDiagonal() * H * p.asDiagonal();
      // g = p.asDiagonal() * g;
      // printf("cond(H): %f\n", cond(H));
      // const vecx_t dx = p.asDiagonal() * H.ldlt().solve(g);

			// Calculate cost
      cost = 0.5 * static_cast<real_t>(e.transpose() * e);
      const real_t cost_diff = (iter == 0) ? 0.0 : (cost - cost_prev);

      // Calculate reprojection error
      size_t nb_keypoints = e.size() / 2.0;
      real_t sse = 0.0;
      for (size_t i = 0; i < nb_keypoints; i++) {
        sse += e.segment(i * 2, 2).norm();
      }
      const real_t rmse = sqrt(sse / nb_keypoints);
      printf("rmse reproj error: %.2f  ", rmse);

      // Calculate
			if (cost_diff > 0) {
        lambda_k /= 10.0;
			} else {
        lambda_k *= 10.0;
			}
      printf("iter[%d]  ", iter);
      printf("cost[%.4e]  ", cost);
      printf("cost change[%.4e]  ", cost_diff);
      printf("iter time: %fs\n", toc(&t_start));

      // Termination criteria
      if (iter != 0 && cost_diff > -1.0e-4) {
        printf("Done!\n");
        break;
      }

			// if (cost_diff < 0) {
        graph_update(graph, dx);
        cost_prev = cost;
      // }
			t_start = tic();
    }

    return 0;
  }

};

} // namespace proto
#endif // PROTO_ESTIMATION_FACTOR_HPP
