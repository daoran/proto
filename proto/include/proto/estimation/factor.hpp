#ifndef PROTO_ESTIMATION_FACTOR_HPP
#define PROTO_ESTIMATION_FACTOR_HPP

#include "proto/core/core.hpp"

namespace proto {

struct param_t {
  bool fixed = false;

  std::string type;
  size_t id = 0;
  timestamp_t ts = 0;
  long local_size = 0;
  long global_size = 0;
  vecx_t param;

  param_t() {}

  param_t(const std::string &type_,
          const size_t id_,
          const long local_size_,
          const long global_size_,
          const bool fixed_=false)
    : param_t{type_, id_, 0, local_size_, global_size_, fixed_} {}

  param_t(const std::string &type_,
          const size_t id_,
          const timestamp_t &ts_,
          const long local_size_,
          const long global_size_,
          const bool fixed_=false)
    : fixed{fixed_},
      type{type_},
      id{id_},
      ts{ts_},
      local_size{local_size_},
      global_size{global_size_},
      param{zeros(global_size_, 1)} {}

  virtual ~param_t() {}

  virtual void plus(const vecx_t &) = 0;
  virtual void perturb(const int i, const real_t step_size) = 0;
};

struct pose_t : param_t {
  pose_t() {}

  pose_t(const size_t id_,
         const timestamp_t &ts_,
         const mat4_t &T,
         const bool fixed_=false)
      : param_t{"pose_t", id_, ts_, 6, 7, fixed_} {
    const quat_t q{tf_quat(T)};
    const vec3_t r{tf_trans(T)};

    param(0) = q.w();
    param(1) = q.x();
    param(2) = q.y();
    param(3) = q.z();

    param(4) = r(0);
    param(5) = r(1);
    param(6) = r(2);
  }

  quat_t rot() const {
    return quat_t{param[0], param[1], param[2], param[3]};
  }

  vec3_t trans() const {
    return vec3_t{param[4], param[5], param[6]};
  }

  mat4_t tf() const {
    return proto::tf(rot(), trans());
  }

  quat_t rot() { return static_cast<const pose_t &>(*this).rot(); }
  vec3_t trans() { return static_cast<const pose_t &>(*this).trans(); }
  mat4_t tf() { return static_cast<const pose_t &>(*this).tf(); }

  void set_trans(const vec3_t &r) {
    param(4) = r(0);
    param(5) = r(1);
    param(6) = r(2);
  }

  void set_rot(const quat_t &q) {
    param(0) = q.w();
    param(1) = q.x();
    param(2) = q.y();
    param(3) = q.z();
  }

  void set_rot(const mat3_t &C) {
    quat_t q{C};
    param(0) = q.w();
    param(1) = q.x();
    param(2) = q.y();
    param(3) = q.z();
  }

  void plus(const vecx_t &dx) {
    // Rotation component
    const vec3_t dalpha{dx(0), dx(1), dx(2)};
    const quat_t dq = quat_delta(dalpha);
    const quat_t q{param[0], param[1], param[2], param[3]};
    const quat_t q_updated = dq * q;
    param(0) = q_updated.w();
    param(1) = q_updated.x();
    param(2) = q_updated.y();
    param(3) = q_updated.z();

    // Translation component
    param(4) += dx(3);
    param(5) += dx(4);
    param(6) += dx(5);
  }

  void perturb(const int i, const real_t step_size) {
    if (i >= 0 && i < 3) {
      const auto T_WS_diff = tf_perturb_rot(this->tf(), step_size, i);
      this->set_rot(tf_rot(T_WS_diff));
      this->set_trans(tf_trans(T_WS_diff));
    } else if (i >= 3 && i <= 5) {
      const auto T_WS_diff = tf_perturb_trans(this->tf(), step_size, i - 3);
      this->set_rot(tf_rot(T_WS_diff));
      this->set_trans(tf_trans(T_WS_diff));
    } else {
      FATAL("Invalid perturbation index [%d]!", i);
    }
  }
};

struct landmark_t : param_t {
  landmark_t() {}

  landmark_t(const size_t id_, const vec3_t &p_W_, const bool fixed_=false)
    : param_t{"landmark_t", id_, 3, 3, fixed_} {
    param = p_W_;
  }

  void plus(const vecx_t &dx) { param += dx; }
  void perturb(const int i, const real_t step_size) { param[i] += step_size; }
};

struct camera_params_t : param_t {
  int cam_index = 0;
  int resolution[2] = {0, 0};
  long proj_size = 0;
  long dist_size = 0;

  camera_params_t() {}

  camera_params_t(const size_t id_,
                  const int cam_index_,
                  const int resolution_[2],
                  const vecx_t &proj_params_,
                  const vecx_t &dist_params_,
                  const bool fixed_=false)
    : param_t{"camera_params_t", id_, proj_params_.size() + dist_params_.size(),
              proj_params_.size() + dist_params_.size(),
              fixed_},
      cam_index{cam_index_},
      resolution{resolution_[0], resolution_[1]},
      proj_size{proj_params_.size()},
      dist_size{dist_params_.size()} {
    param.resize(proj_size + dist_size);
    param.head(proj_size) = proj_params_;
    param.tail(dist_size) = dist_params_;
  }

  vecx_t proj_params() { return param.head(proj_size); }
  vecx_t dist_params() { return param.tail(dist_size); }
  void plus(const vecx_t &dx) { param += dx; }
  void perturb(const int i, const real_t step_size) { param(i) += step_size; }
};

struct sb_params_t : param_t {
  sb_params_t() {}

  sb_params_t(const size_t id_,
             const timestamp_t &ts_,
             const vec3_t &v_,
             const vec3_t &ba_,
             const vec3_t &bg_,
             const bool fixed_=false)
    : param_t{"sb_params_t", id_, ts_, 9, 9, fixed_} {
    param << v_, ba_, bg_;
  }

  void plus(const vecx_t &dx) { param += dx; }
  void perturb(const int i, const real_t step_size) { param(i) += step_size; }
};

typedef std::vector<pose_t> poses_t;
typedef std::vector<landmark_t> landmarks_t;
typedef std::vector<vec2_t> keypoints_t;

void pose_print(const std::string &prefix, const pose_t &pose);
poses_t load_poses(const std::string &csv_path);

std::vector<keypoints_t> load_keypoints(const std::string &data_path);
void keypoints_print(const keypoints_t &keypoints);

/*****************************************************************************
 *                                FACTOR
 ****************************************************************************/

struct factor_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  size_t id = 0;
  matx_t info;
  std::vector<param_t *> params;
  vecx_t residuals;
  matxs_t jacobians;

  factor_t() {}
  factor_t(const size_t id_,
           const matx_t &info_,
           const std::vector<param_t *> &params_)
    : id{id_}, info{info_}, params{params_} {}
  virtual ~factor_t() {}
  virtual int eval(bool jacs=true) = 0;
};

int check_jacobians(factor_t *factor,
                    const int param_idx,
                    const std::string &jac_name,
                    const real_t step_size,
                    const real_t threshold);

/*****************************************************************************
 *                              POSE FACTOR
 ****************************************************************************/

struct pose_factor_t : factor_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	const mat4_t pose_meas;

  pose_factor_t(const size_t id_,
                const mat4_t &pose_,
                const mat_t<6, 6> &info_,
                const std::vector<param_t *> &params_)
      : factor_t{id_, info_, params_}, pose_meas{pose_} {
    residuals = zeros(6, 1);
    jacobians.push_back(zeros(6, 6));
	}

	int eval(bool jacs=true) {
    assert(params.size() == 1);

		// Calculate delta pose
    const mat4_t pose_est = tf(params[0]->param);
		const mat4_t delta_pose = pose_meas * pose_est.inverse();

		// Calculate pose error
		const quat_t dq = tf_quat(delta_pose);
		const vec3_t dtheta = 2 * dq.coeffs().head<3>();
		residuals.head<3>() = dtheta;
		residuals.tail<3>() = tf_trans(pose_meas) - tf_trans(pose_est);

		// Calculate jacobian
		// clang-format off
		if (jacs) {
      jacobians[0].setIdentity();
      jacobians[0] *= -1.0;
      mat3_t dq_mul_xyz;
      dq_mul_xyz << dq.w(), -dq.z(), dq.y(),
                    dq.z(), dq.w(), -dq.x(),
                    -dq.y(), dq.x(), dq.w();
      jacobians[0].block<3, 3>(0, 0) = -dq_mul_xyz;
    }
		// clang-format on

		return 0;
	}
};

/*****************************************************************************
 *                               BA FACTOR
 ****************************************************************************/

template <typename CM>
struct ba_factor_t : factor_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  int cam_index = 0;
  int resolution[2] = {0, 0};

  timestamp_t ts = 0;
  vec2_t z{0.0, 0.0};

  ba_factor_t(const size_t id_,
              const timestamp_t &ts_,
              const vec2_t &z_,
              const mat2_t &info_,
              const std::vector<param_t *> &params_)
      : factor_t{id_, info_, params_}, ts{ts_}, z{z_} {
    residuals = zeros(2, 1);
    jacobians.push_back(zeros(2, 6));  // T_WC
    jacobians.push_back(zeros(2, 3));  // p_W
    jacobians.push_back(zeros(2, CM::params_size));  // Camera params

    auto cam_params = static_cast<camera_params_t *>(params_[2]);
    cam_index = cam_params->cam_index;
    resolution[0] = cam_params->resolution[0];
    resolution[1] = cam_params->resolution[1];
  }

  int eval(bool jacs=true) {
    assert(params.size() == 4);

    // Map out parameters
    const mat4_t T_WC = tf(params[0]->param);
    const vec3_t p_W{params[1]->param};
    const CM cm{resolution, params[2]->param};

    // Transform point from world to camera frame
    const mat4_t T_CW = T_WC.inverse();
    const vec3_t p_C = tf_point(T_CW, p_W);

    // Project point in camera frame to image plane
    vec2_t z_hat;
    mat_t<2, 3> J_h;
    int retval = cm.project(p_C, z_hat, J_h);
    if (retval != 0) {
      // switch (retval) {
      // case -1: LOG_ERROR("Point is not infront of camera!"); break;
      // case -2: LOG_ERROR("Projected point is outside the image plane!"); break;
      // }
			jacobians[0] = zeros(2, 6);  // T_WC
			jacobians[1] = zeros(2, 3);  // p_W
			jacobians[2] = zeros(2, CM::params_size);  // Projection model
      return -1;
    }

    // Calculate residual
    residuals = z - z_hat;

    // Calculate Jacobians
    if (jacs) {
      const vec2_t p{p_C(0) / p_C(2), p_C(1) / p_C(2)};
      const vec2_t p_dist = cm.distortion.distort(p);
      const mat3_t C_WC = tf_rot(T_WC);
      const mat3_t C_CW = C_WC.transpose();
      const vec3_t r_WC = tf_trans(T_WC);

      // -- Jacobian w.r.t. sensor pose T_WS
      jacobians[0].block(0, 0, 2, 3) = -1 * J_h * C_CW * skew(p_W - r_WC);
      jacobians[0].block(0, 3, 2, 3) = -1 * J_h * -C_CW;
      // -- Jacobian w.r.t. landmark
      jacobians[1] = -1 * J_h * C_CW;
      // -- Jacobian w.r.t. camera parameters
      const auto proj_size = cm.proj_params_size;
      const auto dist_size = cm.dist_params_size;
      jacobians[2].block(0, 0, 2, proj_size) = -1 * cm.J_proj(p_dist);
      jacobians[2].block(0, dist_size, 2, proj_size) = -1 * cm.J_dist(p);
    }

    return 0;
  }
};

/*****************************************************************************
 *                             CAMERA FACTOR
 ****************************************************************************/

template <typename CM>
struct cam_factor_t : factor_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  int cam_index = 0;
  int resolution[2] = {0, 0};

  timestamp_t ts = 0;
  vec2_t z{0.0, 0.0};

  cam_factor_t(const size_t id_,
               const timestamp_t &ts_,
               const vec2_t &z_,
               const mat2_t &info_,
               const std::vector<param_t *> &params_)
      : factor_t{id_, info_, params_}, ts{ts_}, z{z_} {
    residuals = zeros(2, 1);
    jacobians.push_back(zeros(2, 6));  // T_WS
    jacobians.push_back(zeros(2, 6));  // T_SC
    jacobians.push_back(zeros(2, 3));  // p_W
    jacobians.push_back(zeros(2, CM::params_size));  // Camera params

    auto cam_params = static_cast<camera_params_t *>(params_[3]);
    cam_index = cam_params->cam_index;
    resolution[0] = cam_params->resolution[0];
    resolution[1] = cam_params->resolution[1];
  }

  int eval(bool jacs=true) {
    assert(params.size() == 5);

    // Map out parameters
    const mat4_t T_WS = tf(params[0]->param);
    const mat4_t T_SC = tf(params[1]->param);
    const vec3_t p_W{params[2]->param};
    const CM cm{resolution, params[3]->param};

    // Transform point from world to camera frame
    const mat4_t T_WC = T_WS * T_SC;
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
      return -1;
    }

    // Calculate residual
    residuals = z - z_hat;

    // Calculate Jacobians
    if (jacs) {
      const mat3_t C_SC = tf_rot(T_SC);
      const mat3_t C_CS = C_SC.transpose();
      const mat3_t C_WS = tf_rot(T_WS);
      const mat3_t C_SW = C_WS.transpose();
      const mat3_t C_CW = C_CS * C_SW;
      const vec3_t r_WS = tf_trans(T_WS);
      const vec2_t p{p_C(0) / p_C(2), p_C(1) / p_C(2)};
      const vec2_t p_dist = cm.distortion.distort(p);

      // -- Jacobian w.r.t. sensor pose T_WS
      jacobians[0].block(0, 0, 2, 3) = -1 * J_h * C_CS * C_SW * skew(p_W - r_WS);
      jacobians[0].block(0, 3, 2, 3) = -1 * J_h * C_CS * -C_SW;
      // -- Jacobian w.r.t. sensor-camera extrinsic pose T_SCi
      jacobians[1].block(0, 0, 2, 3) = -1 * J_h * C_CS * skew(C_SC * p_C);
      jacobians[1].block(0, 3, 2, 3) = -1 * J_h * -C_CS;
      // -- Jacobian w.r.t. landmark
      jacobians[2] = -1 * J_h * C_CW;
      // -- Jacobian w.r.t. camera model
      const auto proj_size = cm.proj_params_size;
      const auto dist_size = cm.dist_params_size;
      jacobians[3].block(0, 0, 2, proj_size) = -1 * cm.J_proj(p_dist);
      jacobians[3].block(0, dist_size, 2, proj_size) = -1 * cm.J_dist(p);
    }

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
               const mat_t<15, 15> &info_,
               const std::vector<param_t *> &params_)
      : factor_t{id_, info_, params_},
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

  int eval(bool jacs=true) {
    // Map out parameters
    // -- Sensor pose at timestep i
    const mat4_t T_i = tf(params[0]->param);
    const mat3_t C_i = tf_rot(T_i);
    const mat3_t C_i_inv = C_i.transpose();
    const quat_t q_i = tf_quat(T_i);
    const vec3_t r_i = tf_trans(T_i);
    // -- Speed and bias at timestamp i
    const vec_t<9> sb_i{params[1]->param};
    const vec3_t v_i = sb_i.segment<3>(0);
    const vec3_t ba_i = sb_i.segment<3>(3);
    const vec3_t bg_i = sb_i.segment<3>(6);
    // -- Sensor pose at timestep j
    const mat4_t T_j = tf(params[2]->param);
    const quat_t q_j = tf_quat(T_j);
    const vec3_t r_j = tf_trans(T_j);
    // -- Speed and bias at timestep j
    const vec_t<9> sb_j{params[3]->param};
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
    if (jacs) {
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
    }

    return 0;
  }
};

/*****************************************************************************
 *                              FACTOR GRAPH
 ****************************************************************************/

struct graph_t {
  std::unordered_map<size_t, param_t *> params;
  std::unordered_map<param_t *, size_t> param_index;
  std::vector<param_t *> param_order;
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

size_t graph_add_camera(graph_t &graph,
                        const int cam_index,
                        const int resolution[2],
                        const vecx_t &proj_params,
                        const vecx_t &dist_params,
                        bool fixed=false) {
  const auto id = graph.params.size();
  const auto param = new camera_params_t{id, cam_index, resolution,
                                         proj_params, dist_params,
                                         fixed};
  graph.params.insert({id, param});
  return id;
}

size_t graph_add_speed_bias(graph_t &graph,
                            const timestamp_t &ts,
                            const vec3_t &v,
                            const vec3_t &ba,
                            const vec3_t &bg) {
  const auto id = graph.params.size();
  const auto param = new sb_params_t{id, ts, v, ba, bg};
  graph.params.insert({id, param});
  return id;
}

size_t graph_add_pose_factor(graph_t &graph,
                             const size_t pose_id,
                             const mat4_t &T,
                             const mat_t<6, 6> &info = I(6)) {
  // Create factor
  const auto f_id = graph.factors.size();
  std::vector<param_t *> params{graph.params[pose_id]};
  auto factor = new pose_factor_t{f_id, T, info, params};

  // Add factor to graph
  graph.factors.push_back(factor);

  return f_id;
}

template <typename CM>
size_t graph_add_ba_factor(graph_t &graph,
                           const timestamp_t &ts,
                           const size_t cam_pose_id,
                           const size_t landmark_id,
                           const size_t cam_params_id,
                           const vec2_t &z,
                           const mat2_t &info = I(2) * 0.5) {

  // Create factor
  const auto f_id = graph.factors.size();
  std::vector<param_t *> params{
    graph.params[cam_pose_id],
    graph.params[landmark_id],
    graph.params[cam_params_id],
  };
  auto factor = new ba_factor_t<CM>{ts, f_id, z, info, params};

  // Add factor to graph
  graph.factors.push_back(factor);

  return f_id;
}

template <typename CM>
size_t graph_add_cam_factor(graph_t &graph,
                            const timestamp_t &ts,
                            const size_t sensor_pose_id,
                            const size_t imu_cam_pose_id,
                            const size_t landmark_id,
                            const size_t cam_params_id,
                            const vec2_t &z,
                            const mat2_t &info = I(2)) {
  // Create factor
  const auto f_id = graph.factors.size();
  std::vector<param_t *> params{
    graph.params[sensor_pose_id],
    graph.params[imu_cam_pose_id],
    graph.params[landmark_id],
    graph.params[cam_params_id]
  };
  auto factor = new cam_factor_t<CM>{ts, f_id, z, info, params};

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
  std::vector<param_t *> params{
    graph.params[pose0_id],
    graph.params[sb0_id],
    graph.params[pose1_id],
    graph.params[sb1_id]
  };
  auto factor = new imu_factor_t(imu_index, imu_ts, imu_gyro, imu_accel, I(15), params);

  // Add factor to graph
  graph.factors.push_back(factor);

  return f_id;
}

void graph_residuals(graph_t &graph, vecx_t &r) {
  std::vector<real_t> residuals;
  for (const auto &factor : graph.factors) {
    if (factor->eval(false) != false) {
      for (long i = 0; i < factor->residuals.size(); i++) {
        residuals.push_back(factor->residuals(i));
      }
    }
  }

  // Convert std::vector<real_t> to vecx_t
  r.resize(residuals.size());
  for (long i = 0; i < r.size(); i++) {
    r(i) = residuals[i];
  }
}

void graph_eval(graph_t &graph, vecx_t &r, matx_t &J) {
	// First pass: Determine what parameters we have
  std::unordered_set<param_t *> param_tracker;
  std::unordered_map<std::string, int> param_counter;

	for (const auto &factor : graph.factors) {
		for (const auto &param : factor->params) {
			// Check if param is already tracked or fixed
			if (param_tracker.count(param) > 0 || param->fixed) {
				continue; // Skip this param
			}

			// Keep track of param blocks
			param_counter[param->type] += param->local_size;
      param_tracker.insert(param);
		}
	}

  // Second pass: Assign jacobian order for each parameter and evaluate factor
  size_t residuals_size = 0;
  size_t params_size = 0;
  std::unordered_map<std::string, int> param_cs;
  param_cs["pose_t"] = 0;
  param_cs["camera_params_t"] = param_counter["pose_t"];
  param_cs["landmark_t"] = param_counter["pose_t"] + param_counter["camera_params_t"];

  std::vector<bool> factor_ok;
	graph.param_index.clear();
  for (const auto &factor : graph.factors) {
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
      if (graph.param_index.count(param) > 0 || param->fixed) {
        continue; // Skip this param
      }

			// Assign jacobian column index for parameter
      graph.param_index.insert({param, param_cs[param->type]});
      param_cs[param->type] += param->local_size;
      params_size += param->local_size;
    }
  }

  // Third pass: Form residuals and jacobians
  r = zeros(residuals_size, 1);
  J = zeros(residuals_size, params_size);

  size_t rs = 0;
  size_t cs = 0;
  for (size_t i = 0; i < graph.factors.size(); i++) {
    const auto &factor = graph.factors[i];
    if (factor_ok[i] == false) {
      continue; // Skip this factor
    }

    // Form jacobian
    for (size_t j = 0; j < factor->params.size(); j++) {
      const auto &param = factor->params.at(j);
      const long rows = factor->residuals.size();
      const long cols = param->local_size;

      if (graph.param_index.count(param)) {
        cs = graph.param_index[param];
        J.block(rs, cs, rows, cols) = factor->jacobians[j];
        // J.block(rs, cs, rows, cols) = ones(rows, cols);
      }
    }

    // Form residual
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

  int iter = 0;
  real_t cost = 0.0;
  vecx_t jacobi_scaling;
  vecx_t lm_diagonal;
  vecx_t e;
  matx_t E;
  matx_t H;

  tiny_solver_t() {}

  void eval(graph_t &graph) {
    graph_eval(graph, e, E);
    H = E.transpose() * E;

    if (iter == 0) {
      jacobi_scaling = 1.0 / (1.0 + E.colwise().norm().array());
      lm_diagonal.resize(H.rows());
    }
    cost = e.squaredNorm() / 2.0;
		printf("iter[%d]  ", iter);
		printf("cost[%.4e]\n", cost);
  }

  int solve(graph_t &graph)  {
    real_t initial_trust_region_radius = 1e4;
    real_t u = 1.0 / initial_trust_region_radius;
    real_t v = 2.0;
		eval(graph);

		struct timespec t_start = tic();
    for (iter = 0; iter < max_iter; iter++) {
			// Precondition H
      const real_t min_diagonal = 1e-6;
      const real_t max_diagonal = 1e32;
      matx_t H_regularized = H;
      for (int i = 0; i < H.rows(); ++i) {
        lm_diagonal[i] = std::sqrt(u * std::min(std::max(H(i, i), min_diagonal), max_diagonal));
        H_regularized(i, i) += lm_diagonal[i] * lm_diagonal[i];
      }
			// printf("cond(H): %f\n", cond(H_regularized));

			// Update
      const vecx_t g = -E.transpose() * e;
      const vecx_t lm_step = H_regularized.ldlt().solve(g);
      const vecx_t dx = jacobi_scaling.asDiagonal() * lm_step;
      graph_update(graph, dx);

			// Evaluate
      vecx_t e_k;
      matx_t E_k;
      graph_eval(graph, e_k, E_k);
      const real_t cost_change = (2 * cost - e_k.squaredNorm());
      const real_t model_cost_change = lm_step.dot(2 * g - H * lm_step);

      // printf("iter[%d]  ", iter);
      // printf("cost[%.4e]  ", e_k.squaredNorm() / 2.0);
      // printf("iter time: %fs\n", toc(&t_start));

			real_t rho(cost_change / model_cost_change);
      if (rho > 0) {
        // Accept the Levenberg-Marquardt step because the linear
        // model fits well.
        // x = x_new_;

        eval(graph);
        // if (summary.gradient_max_norm < options.gradient_tolerance) {
        //   summary.status = GRADIENT_TOO_SMALL;
        //   break;
        // }

        // if (cost_ < options.cost_threshold) {
        //   summary.status = COST_TOO_SMALL;
        //   break;
        // }

        real_t tmp = real_t(2.0 * rho - 1.0);
        u = u * std::max(1.0 / 3.0, 1.0 - tmp * tmp * tmp);
        v = 2;
        continue;
      }

      // Reject the update because either the normal equations failed to solve
      // or the local linear model was not good (rho < 0). Instead, increase u
      // to move closer to gradient descent.
      u *= v;
      v *= 2;
			graph_update(graph, -dx);

      // Calculate reprojection error
      // size_t nb_keypoints = e.size() / 2.0;
      // real_t sse = 0.0;
      // for (size_t i = 0; i < nb_keypoints; i++) {
      //   sse += e.segment(i * 2, 2).norm();
      // }
      // const real_t rmse = sqrt(sse / nb_keypoints);
      // printf("rmse reproj error: %.2f  ", rmse);
    }
    printf("solver took: %fs\n", toc(&t_start));

    return 0;
  }

};

} // namespace proto
#endif // PROTO_ESTIMATION_FACTOR_HPP
