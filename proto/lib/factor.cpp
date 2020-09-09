#include "factor.hpp"

namespace proto {

/*****************************************************************************
 *                             FACTOR GRAPH
 *****************************************************************************/

/****************************** PARAMETERS ***********************************/

param_t::param_t() {}

param_t::param_t(const std::string &type_,
                 const id_t id_,
                 const timestamp_t &ts_,
                 const long local_size_,
                 const long global_size_,
                 const bool fixed_)
  : fixed{fixed_},
    type{type_},
    id{id_},
    ts{ts_},
    local_size{local_size_},
    global_size{global_size_},
    param{zeros(global_size_, 1)} {}

param_t::param_t(const std::string &type_,
                 const id_t id_,
                 const long local_size_,
                 const long global_size_,
                 const bool fixed_)
    : param_t{type_, id_, 0, local_size_, global_size_, fixed_} {}

param_t::~param_t() {}

void param_t::mark_marginalize() {
  marginalize = true;
  type = "marg_" + type;
}

pose_t::pose_t() {}

pose_t::pose_t(const id_t id_,
               const timestamp_t &ts_,
               const vec_t<7> &pose,
               const bool fixed_)
    : param_t{"pose_t", id_, ts_, 6, 7, fixed_} {
  param = pose;
}

pose_t::pose_t(const id_t id_,
         const timestamp_t &ts_,
         const mat4_t &T,
         const bool fixed_)
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

quat_t pose_t::rot() const {
  return quat_t{param[0], param[1], param[2], param[3]};
}

vec3_t pose_t::trans() const {
  return vec3_t{param[4], param[5], param[6]};
}

mat4_t pose_t::tf() const {
  return proto::tf(rot(), trans());
}

quat_t pose_t::rot() {
  return static_cast<const pose_t &>(*this).rot();
}

vec3_t pose_t::trans() {
  return static_cast<const pose_t &>(*this).trans();
}

mat4_t pose_t::tf() {
  return static_cast<const pose_t &>(*this).tf();
}

void pose_t::set_trans(const vec3_t &r) {
  param(4) = r(0);
  param(5) = r(1);
  param(6) = r(2);
}

void pose_t::set_rot(const quat_t &q) {
  param(0) = q.w();
  param(1) = q.x();
  param(2) = q.y();
  param(3) = q.z();
}

void pose_t::set_rot(const mat3_t &C) {
  quat_t q{C};
  param(0) = q.w();
  param(1) = q.x();
  param(2) = q.y();
  param(3) = q.z();
}

void pose_t::plus(const vecx_t &dx) {
  // Rotation component
  const vec3_t dalpha{dx(0), dx(1), dx(2)};
  const quat_t dq = quat_delta(dalpha);
  const quat_t q = rot();
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

void pose_t::perturb(const int i, const real_t step_size) {
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

fiducial_pose_t::fiducial_pose_t() {}

fiducial_pose_t::fiducial_pose_t(const id_t id_, const mat4_t &T, const bool fixed_)
  : pose_t{id_, 0, T, fixed_} {
  this->type = "fiducial_pose_t";
}

extrinsic_t::extrinsic_t() {}

extrinsic_t::extrinsic_t(const id_t id_, const mat4_t &T, const bool fixed_)
  : pose_t{id_, 0, T, fixed_} {
  this->type = "extrinsic_t";
}

landmark_t::landmark_t() {}

landmark_t::landmark_t(const id_t id_, const vec3_t &p_W_, const bool fixed_)
  : param_t{"landmark_t", id_, 3, 3, fixed_} {
  param = p_W_;
}

void landmark_t::plus(const vecx_t &dx) {
  param += dx;
}

void landmark_t::perturb(const int i, const real_t step_size) {
  param[i] += step_size;
}

camera_params_t::camera_params_t() {}

camera_params_t::camera_params_t(const id_t id_,
                  const int cam_index_,
                  const int resolution_[2],
                  const vecx_t &proj_params_,
                  const vecx_t &dist_params_,
                  const bool fixed_)
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

vecx_t camera_params_t::proj_params() {
  return param.head(proj_size);
}

vecx_t camera_params_t::dist_params() {
  return param.tail(dist_size);
}

void camera_params_t::plus(const vecx_t &dx) {
  param += dx;
}

void camera_params_t::perturb(const int i, const real_t step_size) {
  param(i) += step_size;
}

sb_params_t::sb_params_t() {}

sb_params_t::sb_params_t(const id_t id_,
                         const timestamp_t &ts_,
                         const vec3_t &v_,
                         const vec3_t &ba_,
                         const vec3_t &bg_,
                         const bool fixed_)
  : param_t{"sb_params_t", id_, ts_, 9, 9, fixed_} {
  param << v_, ba_, bg_;
}

void sb_params_t::plus(const vecx_t &dx) {
  param += dx;
}

void sb_params_t::perturb(const int i, const real_t step_size) {
  param(i) += step_size;
}

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

factor_t::factor_t() {}

factor_t::factor_t(const id_t id_,
                   const matx_t &covar_,
                   const std::vector<param_t *> &params_)
  : id{id_}, covar{covar_}, info{covar_.inverse()}, params{params_} {
  Eigen::LLT<matx_t> llt_info(info);
  sqrt_info = llt_info.matrixL().transpose();
}

factor_t::factor_t(const id_t id_, const matx_t &covar_, param_t * &param_)
    : id{id_}, covar{covar_}, info{covar_.inverse()}, params{param_} {
    Eigen::LLT<matx_t> llt_info(info);
    sqrt_info = llt_info.matrixL().transpose();
  }

factor_t::~factor_t() {}

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

pose_factor_t::pose_factor_t(const id_t id_,
                const mat_t<6, 6> &covar_,
                param_t *param_)
      : factor_t{id_, covar_, param_}, pose_meas{tf(param_->param)} {
    type = "pose_factor_t";
    residuals = zeros(6, 1);
    jacobians.push_back(zeros(6, 6));
  }

  int pose_factor_t::eval(bool jacs) {
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

extrinsic_factor_t::extrinsic_factor_t(const id_t id_,
                     const mat_t<6, 6> &covar_,
                     param_t *param_)
    : pose_factor_t{id_, covar_, param_} {}

speed_bias_factor_t::speed_bias_factor_t(const id_t id_,
                      const mat_t<9, 9> &covar_,
                      param_t * param_)
      : factor_t{id_, covar_, param_}, sb_meas{param_->param} {
    type = "speed_bias_factor_t";
    residuals = zeros(9, 1);
    jacobians.push_back(zeros(9, 9));
  }

  int speed_bias_factor_t::eval(bool jacs) {
    assert(params.size() == 1);

    // Calculate delta sb
    const vec_t<9> sb_est = params[0]->param;
    const vec_t<9> error = sb_meas - sb_est;
    residuals = error;

    // Calculate jacobian
    if (jacs) {
      jacobians[0] = -1.0 * I(9);
    }

    return 0;
  }

camera_params_factor_t::camera_params_factor_t(const id_t id_,
                         const matx_t &covar_,
                         param_t *param_)
      : factor_t(id_, covar_, {param_}), meas{param_->param} {
    type = "camera_params_factor_t";
    residuals = zeros(9, 1);
    jacobians.push_back(zeros(9, 9));
  }

  int camera_params_factor_t::eval(bool jacs) {
    assert(params.size() == 1);

    // Calculate delta sb
    const vecx_t est = params[0]->param;
    const vecx_t error = meas - est;
    residuals = error;

    // Calculate jacobian
    if (jacs) {
      jacobians[0] = -1.0 * I(est.rows());
    }

    return 0;
  }

landmark_factor_t::landmark_factor_t(const id_t id_,
                    const matx_t &covar_,
                    param_t *param_)
      : factor_t(id_, covar_, {param_}), meas{param_->param} {
    type = "landmark_factor_t";
    residuals = zeros(3, 1);
    jacobians.push_back(zeros(3, 3));
  }

int landmark_factor_t::eval(bool jacs) {
  assert(params.size() == 1);

  // Calculate delta sb
  const vecx_t est = params[0]->param;
  const vecx_t error = meas - est;
  residuals = error;

  // Calculate jacobian
  if (jacs) {
    jacobians[0] = -1.0 * I(est.rows());
  }

  return 0;
}

imu_factor_t::imu_factor_t(const id_t id_,
               const int imu_index_,
               const timestamps_t imu_ts_,
               const vec3s_t imu_accel_,
               const vec3s_t imu_gyro_ ,
               const mat_t<15, 15> &covar_,
               const std::vector<param_t *> &params_)
    : factor_t{id_, covar_, params_},
      imu_index{imu_index_},
      imu_ts{imu_ts_},
      imu_accel{imu_accel_},
      imu_gyro{imu_gyro_} {
  type = "imu_factor_t";
  residuals = zeros(15, 1);
  jacobians.push_back(zeros(15, 6));  // T_WS at timestep i
  jacobians.push_back(zeros(15, 9));  // Speed and bias at timestep i
  jacobians.push_back(zeros(15, 6));  // T_WS at timestep j
  jacobians.push_back(zeros(15, 9));  // Speed and bias at timestep j

  propagate(imu_ts_, imu_accel_, imu_gyro_);
}

void imu_factor_t::reset() {
  P = zeros(15, 15);
  F = zeros(15, 15);

  dp = zeros(3);
  dv = zeros(3);
  dq = quat_t{1.0, 0.0, 0.0, 0.0};
  ba = zeros(3);
  bg = zeros(3);
}

void imu_factor_t::propagate(const timestamps_t &ts,
                             const vec3s_t &a_m,
                             const vec3s_t &w_m) {
  assert(ts.size() == a_m.size());
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

int imu_factor_t::eval(bool jacs) {
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

graph_t::graph_t() {}

graph_t::~graph_t() {
  for (const auto &kv : factors) {
    delete kv.second;
  }
  factors.clear();

  for (const auto &kv : params) {
    delete kv.second;
  }
  params.clear();
}

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

/******************************* TINY SOLVER ********************************/

tiny_solver_t::tiny_solver_t() {}

void tiny_solver_t::load_config(const config_t &config,
                                const std::string &prefix) {
  const std::string key = (prefix == "") ? "" : prefix + ".";
  parse(config, key + "verbose", verbose);
  parse(config, key + "marg_type", marg_type);
  parse(config, key + "max_iter", max_iter);
  parse(config, key + "time_limit", time_limit);
  parse(config, key + "lambda", lambda);
}

real_t tiny_solver_t::eval(graph_t &graph) {
  graph_eval(graph, H, g, &marg_size, &remain_size);
  e = graph_residuals(graph);
  return 0.5 * e.transpose() * e;
}

void tiny_solver_t::update(graph_t &graph, const real_t lambda_k) {
  assert(H.size() != 0);
  assert(g.size() != 0);

  // -- Marginalize?
  if (marg_size) {
    if (marg_type == "sibley") {
      if (schurs_complement(H, g, marg_size, H.rows() - marg_size) != 0) {
        marg_size = 0;
      }
    } else if (marg_type == "drop") {
      marg_size = 0;
    } else {
      FATAL("marg_type [%s] not implemented!\n", marg_type.c_str());
    }
  }
  // -- Damp the Hessian matrix H
  const matx_t H_diag = (H.diagonal().asDiagonal());
  H = H + lambda_k * H_diag;
  // -- Solve for dx
  dx = H.ldlt().solve(g);
  // -- Update
  graph_update(graph, dx, marg_size);
}

int tiny_solver_t::solve(graph_t &graph)  {
  struct timespec solve_tic = tic();
  real_t lambda_k = lambda;

  // Solve
  for (iter = 0; iter < max_iter; iter++) {
    // Cost k
    x = graph_get_state(graph);
    graph_eval(graph, H, g, &marg_size, &remain_size);
    const matx_t H_diag = (H.diagonal().asDiagonal());
    H = H + lambda_k * H_diag;
    dx = H.ldlt().solve(g);
    e = graph_residuals(graph);
    cost = 0.5 * e.transpose() * e;

    // Cost k+1
    graph_update(graph, dx);
    // graph_eval(graph, H, g, &marg_size, &remain_size);
    // const matx_t H_diag_kp1 = (H.diagonal().asDiagonal());
    // H = H + lambda_k * H_diag_kp1;
    // dx = H.ldlt().solve(g);
    e = graph_residuals(graph);
    const real_t cost_k = 0.5 * e.transpose() * e;

    // cost = eval(graph);
    // x = graph_get_state(graph);
    //
    // update(graph, lambda);
    // const real_t cost_k = eval(graph);

    const real_t cost_delta = cost_k - cost;
    const real_t solve_time = toc(&solve_tic);
    const real_t iter_time = (iter == 0) ? 0 : (solve_time / iter);

    if (verbose) {
      printf("iter[%d] ", iter);
      printf("cost[%.2e] ", cost);
      printf("cost_k[%.2e] ", cost_k);
      printf("cost_delta[%.2e] ", cost_delta);
      printf("lambda[%.2e] ", lambda_k);
      printf("iter_time[%.4f] ", iter_time);
      printf("solve_time[%.4f]  ", solve_time);
      printf("\n");

      // // Calculate reprojection error
      // size_t nb_keypoints = e.size() / 2.0;
      // real_t sse = 0.0;
      // for (size_t i = 0; i < nb_keypoints; i++) {
      //   sse += pow(e.segment(i * 2, 2).norm(), 2);
      // }
      // const real_t rmse = sqrt(sse / nb_keypoints);
      // printf("rmse reproj error: %.2f\n", rmse);
    }

    // Determine whether to accept update
    if (cost_k < cost) {
      // Accept update
      // printf("improvement!\n");
      lambda_k /= update_factor;
      cost = cost_k;
    } else {
      // Reject update
      // printf("no improvement!\n");
      graph_set_state(graph, x); // Restore state
      lambda_k *= update_factor;
    }

    // Termination criterias
    if (fabs(cost_delta) < cost_change_threshold) {
      break;
    } else if ((solve_time + iter_time) > time_limit) {
      break;
    }
  }

  solve_time = toc(&solve_tic);
  if (verbose) {
    printf("cost: %.2e\t", cost);
    printf("solver took: %.4fs\n", solve_time);
  }

  return 0;
}

} // namespace proto
