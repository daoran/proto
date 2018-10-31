#include "prototype/msckf/gmsckf.hpp"

namespace prototype {

GMSCKF::GMSCKF() {
  // Create Chi-Square lookup table
  for (int i = 1; i < 100; ++i) {
    boost::math::chi_squared chi_squared_dist(i);
    this->chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.05);
  }
}

int GMSCKF::configure(const std::string &config_file) {
  // clang-format off
  // Load config file
  ConfigParser parser;
  IMUStateConfig imu_config;
  // -- General Settings
  parser.addParam("gmsckf.max_window_size", &this->max_window_size);
  parser.addParam("gmsckf.min_track_length", &this->min_track_length);
  parser.addParam("gmsckf.max_nb_tracks", &this->max_nb_tracks);
  parser.addParam("gmsckf.enable_chisq_test", &this->enable_chisq_test, true);
  parser.addParam("gmsckf.enable_ns_trick", &this->enable_ns_trick);
  parser.addParam("gmsckf.enable_qr_trick", &this->enable_qr_trick);
  parser.addParam("gmsckf.qr_mode", &this->qr_mode);
  // -- IMU Settings
  parser.addParam("gmsckf.imu.initial_covariance.q_init_var", &imu_config.q_init_var);
  parser.addParam("gmsckf.imu.initial_covariance.bg_init_var", &imu_config.bg_init_var);
  parser.addParam("gmsckf.imu.initial_covariance.v_init_var", &imu_config.v_init_var);
  parser.addParam("gmsckf.imu.initial_covariance.ba_init_var", &imu_config.ba_init_var);
  parser.addParam("gmsckf.imu.initial_covariance.p_init_var", &imu_config.p_init_var);
  parser.addParam("gmsckf.imu.process_noise.w_var", &imu_config.w_var);
  parser.addParam("gmsckf.imu.process_noise.dbg_var", &imu_config.dbg_var);
  parser.addParam("gmsckf.imu.process_noise.a_var", &imu_config.a_var);
  parser.addParam("gmsckf.imu.process_noise.dba_var", &imu_config.dba_var);
  parser.addParam("gmsckf.imu.constants.gravity_constant", &imu_config.g_G);
  // -- Camera Settings
  parser.addParam("gmsckf.camera.extrinsics.p_IC", &this->p_IC);
  parser.addParam("gmsckf.camera.extrinsics.q_CI", &this->q_CI);
  parser.addParam("gmsckf.camera.measurement_noise.img_var", &this->img_var);
  // -- Gimbal Settings
  parser.addParam("gimbal.tau_s", &gimbal_model.tau_s);
  parser.addParam("gimbal.tau_d", &gimbal_model.tau_d);
  parser.addParam("gimbal.w1", &gimbal_model.w1);
  parser.addParam("gimbal.w2", &gimbal_model.w2);
  parser.addParam("gimbal.theta1_offset", &gimbal_model.theta1_offset);
  parser.addParam("gimbal.theta2_offset", &gimbal_model.theta2_offset);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }
  // clang-format on

  // Set IMU Settings
  // clang-format off
  // -- Set estimate covariance matrix
  vecx_t init_var = zeros(x_imu_sz, 1);
  init_var << imu_config.q_init_var,
              imu_config.bg_init_var,
              imu_config.v_init_var,
              imu_config.ba_init_var,
              imu_config.p_init_var;
  this->P_imu = init_var.asDiagonal();

  // -- Set noise covariance matrix
  vecx_t n_imu = zeros(12, 1);
  n_imu << imu_config.w_var,
           imu_config.dbg_var,
           imu_config.a_var,
           imu_config.dba_var;
  this->Q_imu = n_imu.asDiagonal();
  // clang-format on

  return 0;
}

vecx_t GMSCKF::getState() {
  vecx_t state = zeros(9, 1);
  state.segment(0, 3) = this->p_G;
  state.segment(3, 3) = this->v_G;
  state.segment(6, 3) = quat2euler(this->q_IG);
  return state;
}

matx_t GMSCKF::P() {
  // GMSCKF not initialized yet
  const int N = this->N();
  if (N == 0) {
    return this->P_imu;
  }

  // Form P
  const int P_size = x_imu_sz + x_cam_sz * this->N();
  matx_t P;
  P = zeros(P_size, P_size);
  P.block(0, 0, x_imu_sz, x_imu_sz) = this->P_imu;
  P.block(0, x_imu_sz, x_imu_sz, N * x_cam_sz) = this->P_imu_cam;
  P.block(x_imu_sz, 0, N * x_cam_sz, x_imu_sz) = this->P_imu_cam.transpose();
  P.block(x_imu_sz, x_imu_sz, N * x_cam_sz, N * x_cam_sz) = this->P_cam;

  return P;
}

matx_t GMSCKF::J(const vec4_t &cam_q_CI,
                 const vec3_t &cam_p_IC,
                 const vec4_t &q_hat_IG,
                 const int N) {
  const mat3_t C_CI = C(cam_q_CI);
  const mat3_t C_IG = C(q_hat_IG);

  matx_t J = zeros(x_cam_sz, x_imu_sz + x_cam_sz * N);
  // -- First row --
  J.block(0, 0, 3, 3) = C_CI;
  // -- Second row --
  J.block(3, 0, 3, 3) = skew(C_IG.transpose() * cam_p_IC);
  J.block(3, 12, 3, 3) = I(3);

  return J;
}

int GMSCKF::N() { return static_cast<int>(this->cam_states.size()); }

void GMSCKF::H(const FeatureTrack &track,
               const CameraStates &track_cam_states,
               const vec3_t &p_G_fj,
               matx_t &H_f_j,
               matx_t &H_x_j) {
  // Setup
  const int N = this->N();             // Number of camera states
  const int M = track.trackedLength(); // Length of feature track

  // Measurement jacobian w.r.t state
  H_x_j = zeros(4 * M, x_imu_sz + x_cam_sz * N);

  // Measurement jacobian w.r.t feature
  H_f_j = zeros(4 * M, 3);

  // Pose index
  FrameID pose_idx = track.frame_start - this->cam_states[0].frame_id;

  // Form measurement jacobians
  for (int i = 0; i < M; i++) {
    // Static camera pose in global frame
    const mat3_t C_CsG = C(track_cam_states[i].q_CG);
    const vec3_t p_G_Cs = track_cam_states[i].p_G;

    // Dynamic camera pose in static camera frame
    const vec2_t theta = track_cam_states[i].theta;
    const mat4_t T_CdCs = this->gimbal_model.T_ds(theta);
    const mat3_t C_CdCs = T_CdCs.block(0, 0, 3, 3);

    // Feature position in static camera frame
    const vec3_t p_Cs_f = C_CsG * (p_G_fj - p_G_Cs);

    // Feature position in dynamic camera frame
    const vec3_t p_Cd_f = (T_CdCs * p_Cs_f.homogeneous()).head(3);

    // dz / dp_Cs_f
    matx_t dz_dp_Cs_f = zeros(4, 3);
    dz_dp_Cs_f(0, 0) = 1 / p_Cs_f(2);
    dz_dp_Cs_f(1, 1) = 1 / p_Cs_f(2);
    dz_dp_Cs_f(0, 2) = -p_Cs_f(0) / (p_Cs_f(2) * p_Cs_f(2));
    dz_dp_Cs_f(1, 2) = -p_Cs_f(1) / (p_Cs_f(2) * p_Cs_f(2));

    // dz / dp_Cd_f
    matx_t dz_dp_Cd_f = zeros(4, 3);
    dz_dp_Cd_f(2, 0) = 1 / p_Cd_f(2);
    dz_dp_Cd_f(3, 1) = 1 / p_Cd_f(2);
    dz_dp_Cd_f(2, 2) = -p_Cd_f(0) / (p_Cd_f(2) * p_Cd_f(2));
    dz_dp_Cd_f(3, 2) = -p_Cd_f(1) / (p_Cd_f(2) * p_Cd_f(2));

    // dp_Cs_f / dx_Cs
    matx_t dp_Cs_f_dx_Cs = zeros(3, 6);
    dp_Cs_f_dx_Cs.leftCols(3) = skew(p_Cs_f);
    dp_Cs_f_dx_Cs.rightCols(3) = -C_CsG;

    // dp_Cd_f / dx_Cs
    matx_t dp_Cd_f_dx_Cs = zeros(3, 6);
    dp_Cd_f_dx_Cs.leftCols(3) = C_CdCs * skew(p_Cs_f);
    dp_Cd_f_dx_Cs.rightCols(3) = C_CdCs * -C_CsG;

    // dp_Cs_f / dp_G_f
    const mat3_t dp_Cs_f_dp_G_f = C_CsG;

    // dp_Cd_f / dp_G_f
    const mat3_t dp_Cd_f_dp_G_f = C_CdCs * C_CsG;

    // Stack Jacobians
    // -- Row start index
    const int rs = 4 * i;
    // -- H_x_j measurement jacobian w.r.t i-th camera pose
    const int cs_H_Cs = x_imu_sz + (x_cam_sz * pose_idx);
    H_x_j.block(rs, cs_H_Cs, 4, 6) =
        dz_dp_Cs_f * dp_Cs_f_dx_Cs + dz_dp_Cd_f * dp_Cs_f_dx_Cs;
    // -- H_f_j measurement jacobian w.r.t feature
    H_f_j.block(rs, 0, 4, 3) =
        dz_dp_Cs_f * dp_Cs_f_dp_G_f + dz_dp_Cd_f * dp_Cd_f_dp_G_f;

    // TODO: Modify measurement jacobian according to OC-EKF

    // Update pose_idx
    pose_idx++;
  }
}

int GMSCKF::initialize(const long ts,
                       const vec4_t &q_IG,
                       const vec3_t &v_G,
                       const vec3_t &p_G,
                       const vec2_t &theta) {
  this->timestamp_first = ts;
  this->last_updated = ts;
  this->q_IG = q_IG;       ///< JPL Quaternion in Global frame
  this->b_g = zeros(3, 1); ///< Bias of gyroscope
  this->v_G = v_G;         ///< Velocity in Global frame
  this->b_a = zeros(3, 1); ///< Bias of accelerometer
  this->p_G = p_G;         ///< Position in Global frame
  this->augmentState(theta);

  return 0;
}

int GMSCKF::initialize(const long ts,
                       const std::vector<vec3_t> &imu_gyro_buffer,
                       const std::vector<vec3_t> &imu_accel_buffer,
                       const vec2_t &theta) {
  assert(imu_gyro_buffer.size() == imu_accel_buffer.size());

  // Calculate sum of angular velocity and linear acceleration
  vec3_t sum_angular_vel = vec3_t::Zero();
  vec3_t sum_linear_acc = vec3_t::Zero();
  const int buf_size = imu_gyro_buffer.size();
  for (int i = 0; i < buf_size; i++) {
    sum_angular_vel += imu_gyro_buffer[i];
    sum_linear_acc += imu_accel_buffer[i];
  }

  // Initialize gyro bias
  this->b_g = sum_angular_vel / (double) buf_size;

  // Initialize gravity
  vec3_t g_imu = sum_linear_acc / (double) buf_size;
  this->g_G = vec3_t(0.0, 0.0, -g_imu.norm());

  // Initialize the initial orientation
  Eigen::Quaterniond q_GI =
      Eigen::Quaterniond::FromTwoVectors(g_imu, -this->g_G);
  this->q_IG = rot2quat(q_GI.toRotationMatrix().transpose());

  // Initialize the other variables
  this->timestamp_first = ts;
  this->last_updated = ts;
  this->v_G = zeros(3, 1);
  this->b_a = zeros(3, 1);
  this->p_G = zeros(3, 1);
  this->augmentState(theta);

  return 0;
}

void GMSCKF::augmentState(const vec2_t &theta) {
  // Camera pose jacobian
  const matx_t J = this->J(this->q_CI, this->p_IC, this->q_IG, this->N());

  // Form temporary matrix to augment new camera state
  const int N = this->N();
  const int X0_size = x_imu_sz + N * x_cam_sz;
  const int X_rows = X0_size + J.rows();
  const int X_cols = X0_size;
  matx_t X;
  X.resize(X_rows, X_cols);
  X.block(0, 0, X0_size, X0_size) = I(X0_size);
  X.block(X0_size, 0, J.rows(), J.cols()) = J;

  // Augment GMSCKF covariance matrix (with new camera state)
  const matx_t P = X * this->P() * X.transpose();
  const matx_t P_fixed = (P + P.transpose()) / 2.0; // Ensure symmetry

  // Decompose covariance into its own constituents
  // clang-format off
  this->P_imu = P_fixed.block(0, 0, x_imu_sz, x_imu_sz);
  this->P_cam = P_fixed.block(x_imu_sz, x_imu_sz, P.rows() - x_imu_sz, P.cols() - x_imu_sz);
  this->P_imu_cam = P_fixed.block(0, x_imu_sz, x_imu_sz, P.cols() - x_imu_sz);
  // clang-format on

  // Add new camera state to sliding window by using current IMU pose
  // estimate to calculate camera pose
  // -- Create camera state in global frame
  const vec4_t imu_q_IG = this->q_IG;
  const vec3_t imu_p_G = this->p_G;
  const vec4_t cam_q_CG = quatlcomp(this->q_CI) * imu_q_IG;
  const vec3_t cam_p_G = imu_p_G + C(imu_q_IG).transpose() * this->p_IC;
  // -- Add camera state to sliding window
  this->cam_states.emplace_back(this->counter_frame_id,
                                cam_p_G,
                                cam_q_CG,
                                theta);

  this->counter_frame_id++;
}

CameraStates GMSCKF::getTrackCameraStates(const FeatureTrack &track) {
  // Pre-check
  assert(this->N() != 0);

  // Calculate camera states where feature was observed
  const FrameID fstart = track.frame_start;
  const FrameID fend = track.frame_end;
  const FrameID cstart = fstart - this->cam_states[0].frame_id;
  const FrameID cend = this->N() - (this->cam_states.back().frame_id - fend);

  // Copy camera states
  auto first = this->cam_states.begin() + cstart;
  auto last = this->cam_states.begin() + cend;
  CameraStates track_cam_states{first, last};
  assert(track_cam_states.front().frame_id == track.frame_start);
  assert(track_cam_states.back().frame_id == track.frame_end);

  return track_cam_states;
}

CameraStates GMSCKF::getAllCameraStates() {
  CameraStates retval;

  // Add old camera states
  for (const auto &cam_state : this->old_cam_states) {
    retval.push_back(cam_state);
  }

  // Add current camera states
  for (const auto &cam_state : this->cam_states) {
    retval.push_back(cam_state);
  }

  return retval;
}

matx_t GMSCKF::F(const vec3_t &w_hat,
                 const vec4_t &q_hat,
                 const vec3_t &a_hat) {
  matx_t F = zeros(x_imu_sz);

  // -- First row block --
  F.block<3, 3>(0, 0) = -skew(w_hat);
  F.block<3, 3>(0, 3) = -I(3);
  // -- Third Row block --
  F.block<3, 3>(6, 0) = -C(q_hat).transpose() * skew(a_hat);
  F.block<3, 3>(6, 9) = -C(q_hat).transpose();
  // -- Fifth Row block --
  F.block<3, 3>(12, 6) = I(3);

  return F;
}

matx_t GMSCKF::G(const vec4_t &q_hat) {
  matx_t G = zeros(x_imu_sz, 12);

  // -- First row block --
  G.block<3, 3>(0, 0) = -I(3);
  // -- Second row block --
  G.block<3, 3>(3, 3) = I(3);
  // -- Third row block --
  G.block<3, 3>(6, 6) = -C(q_hat).transpose();
  // -- Fourth row block --
  G.block<3, 3>(9, 9) = I(3);

  return G;
}

int GMSCKF::predictionUpdate(const vec3_t &a_m,
                             const vec3_t &w_m,
                             const long ts) {
  const double dt = (ts - this->last_updated) * 1e-9;
  this->imu_timestamps.push_back((ts - this->timestamp_first) * 1.0e-9);
  if (dt < 0.0) {
    LOG_ERROR("IMU timestamp: [%lu]", ts);
    LOG_ERROR("GMSCKF.last_updated: [%lu]", this->last_updated);
    FATAL("Calculated dt is negative! [%.4f]", dt);
  }

  // Calculate new accel and gyro estimates
  const vec3_t a_hat = a_m - this->b_a;
  const vec3_t w_hat = w_m - this->b_g;

  // Build the transition F and input G matrices
  const matx_t F = this->F(w_hat, this->q_IG, a_hat);
  const matx_t G = this->G(this->q_IG);

  // Propagate IMU states
  // clang-format off
  if (this->rk4) {
    // Quaternion zeroth order integration
    const vec4_t dq_dt = quatzoi(this->q_IG, w_hat, dt);
    const vec4_t dq_dt2 = quatzoi(this->q_IG, w_hat, dt * 0.5);
    const mat3_t dR_dt_transpose = C(dq_dt).transpose();
    const mat3_t dR_dt2_transpose = C(dq_dt2).transpose();

    // 4th order Runge-Kutta
    // -- k1 = f(tn, yn)
    const vec3_t k1_v_dot = C(this->q_IG).transpose() * a_hat + this->g_G;
    const vec3_t k1_p_dot = this->v_G;
    // -- k2 = f(tn + dt / 2, yn + k1 * dt / 2)
    const vec3_t k1_v = this->v_G + k1_v_dot * dt / 2.0;
    const vec3_t k2_v_dot = dR_dt2_transpose * a_hat + this->g_G;
    const vec3_t k2_p_dot = k1_v;
    // -- k3 = f(tn + dt / 2, yn + k2 * dt / 2)
    const vec3_t k2_v = this->v_G + k2_v_dot * dt / 2;
    const vec3_t k3_v_dot = dR_dt2_transpose * a_hat + this->g_G;
    const vec3_t k3_p_dot = k2_v;
    // -- k4 = f(tn + dt, yn + k3)
    const vec3_t k3_v = this->v_G + k3_v_dot * dt;
    const vec3_t k4_v_dot = dR_dt_transpose * a_hat + this->g_G;
    const vec3_t k4_p_dot = k3_v;
    // -- yn + 1 = yn + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    this->q_IG = quatnormalize(dq_dt);
    this->v_G = this->v_G + dt / 6 * (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
    this->p_G = this->p_G + dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);

  } else {
    // -- Orientation
    this->q_IG += 0.5 * Omega(w_hat) * this->q_IG * dt;
    this->q_IG = quatnormalize(this->q_IG);
    // -- Velocity
    this->v_G += (C(this->q_IG).transpose() * a_hat + this->g_G) * dt;
    // -- Position
    this->p_G += v_G * dt;

  }
  // clang-format on

  // Update covariance
  // clang-format off
  // -- Approximate matrix exponential to the 3rd order using the power series,
  //    which can be considered to be accurate enough assuming dt is within
  //    0.01s.
  const matx_t F_dt = F * dt;
  const matx_t F_dt_sq = F_dt * F_dt;
  const matx_t F_dt_cube = F_dt_sq * F_dt;
  const matx_t Phi = I(x_imu_sz) + F_dt + 0.5 * F_dt_sq + (1.0 / 6.0) * F_dt_cube;
  // -- Update
  // this->P = Phi * this->P * Phi.transpose() + this->Q;
  // this->P = Phi * this->P * Phi.transpose() + (G * this->Q_imu * G.transpose()) * dt;
  this->P_imu = Phi * this->P_imu * Phi.transpose() + (Phi * G * this->Q_imu * G.transpose() * Phi.transpose()) * dt;
  this->P_imu = enforce_psd(this->P_imu);
  // TODO: Modify transition matrix according to OC-EKF
  // clang-format on

  // Update covariance matrices
  this->P_cam = this->P_cam;
  this->P_imu_cam = Phi * this->P_imu_cam;
  this->last_updated = ts;

  return 0;
}

int GMSCKF::chiSquaredTest(const matx_t &H, const vecx_t &r, const int dof) {
  const matx_t P1 = H * this->P() * H.transpose();
  const matx_t P2 = this->img_var * I(H.rows(), H.rows());
  const double gamma = r.transpose() * (P1 + P2).ldlt().solve(r);

  if (gamma < this->chi_squared_table[dof]) {
    return 0;
  } else {
    return -1;
  }
}

int GMSCKF::residualizeTrack(const FeatureTrack &track,
                             matx_t &H_o_j,
                             vecx_t &r_o_j) {
  // Pre-check
  if (track.trackedLength() >= (size_t) this->max_window_size) {
    return -1;
  }

  // Estimate j-th feature position in global frame
  const CameraStates track_cam_states = this->getTrackCameraStates(track);
  vec3_t p_G_f;
  CeresFeatureEstimator feature_estimator(track,
                                          track_cam_states,
                                          this->gimbal_model);
  if (feature_estimator.estimate(p_G_f) != 0) {
    return -2;
  }

  // Calculate residuals
  vecx_t r_j = zeros(4 * track_cam_states.size(), 1);
  for (size_t i = 0; i < track_cam_states.size(); i++) {
    // Transform feature from global frame to i-th camera frame
    const auto cam_state = track_cam_states[i];
    const mat3_t C_CsG = C(cam_state.q_CG);
    const mat4_t T_CdCs = this->gimbal_model.T_ds(cam_state.theta);
    const vec3_t p_Cs_f = C_CsG * (p_G_f - cam_state.p_G);
    const vec3_t p_Cd_f = (T_CdCs * p_Cs_f.homogeneous()).head(3);
    const double u_Cs = p_Cs_f(0) / p_Cs_f(2);
    const double v_Cs = p_Cs_f(1) / p_Cs_f(2);
    const double u_Cd = p_Cd_f(0) / p_Cd_f(2);
    const double v_Cd = p_Cd_f(1) / p_Cd_f(2);
    const vec4_t z_hat{u_Cs, v_Cs, u_Cd, v_Cd};

    // Calculate reprojection error and add it to the residual vector
    assert(track.type == DYNAMIC_STEREO_TRACK);
    const vec2_t z_Cs = track.track0[i].getKeyPoint();
    const vec2_t z_Cd = track.track1[i].getKeyPoint();
    const vec4_t z{z_Cs(0), z_Cs(1), z_Cd(0), z_Cd(1)};
    const int rs = 4 * i;
    r_j.block(rs, 0, 4, 1) = z - z_hat;
  }

  // Form jacobian of measurement w.r.t both state and feature
  matx_t H_f_j;
  matx_t H_x_j;
  this->H(track, track_cam_states, p_G_f, H_f_j, H_x_j);

  // Perform Null Space Trick
  if (this->enable_ns_trick) {
    // Perform null space trick to decorrelate feature position error
    // away state errors by removing the measurement jacobian w.r.t.
    // feature position via null space projection [Section D:
    // Measurement Model, Mourikis2007]
    const unsigned int settings = Eigen::ComputeFullU | Eigen::ComputeThinV;
    Eigen::JacobiSVD<matx_t> svd(H_f_j, settings);
    const matx_t A_j = svd.matrixU().rightCols(H_f_j.rows() - 3);
    H_o_j = A_j.transpose() * H_x_j;
    r_o_j = A_j.transpose() * r_j;

  } else {
    H_o_j = H_x_j;
    r_o_j = r_j;
  }

  // Peform chi squared test
  const int dof = track.trackedLength() - 1;
  if (this->enable_chisq_test) {
    if (this->chiSquaredTest(H_o_j, r_o_j, dof) != 0) {
      return -3;
    }
  }

  return 0;
}

int GMSCKF::calcResiduals(const FeatureTracks &tracks,
                          matx_t &T_H,
                          vecx_t &r_n) {
  // Residualize feature tracks
  matx_t H_o;
  vecx_t r_o;

  for (auto track : tracks) {
    matx_t H_j;
    vecx_t r_j;

    if (this->residualizeTrack(track, H_j, r_j) == 0) {
      // Stack measurement jacobian matrix and residual vector
      if (r_o.rows() > 0) {
        H_o = vstack(H_o, H_j);
        r_o = vstack(r_o, r_j);
      } else {
        H_o = H_j;
        r_o = r_j;
      }
    }
  }

  // No residuals, do not continue
  if (r_o.rows() == 0) {
    LOG_WARN("No residuals!");
    return -1;
  }
  if (r_o.maxCoeff() > 0.1) {
    LOG_WARN("Large residual! [%.4f]", r_o.maxCoeff());
    /* return -1; */
  }

  // Reduce EKF measurement update computation with QR decomposition
  if (H_o.rows() > H_o.cols() && this->enable_qr_trick) {
    // Perform QR decompostion
    if (this->qr_mode == "sparse") {
      // -- Sparse QR method (faster)
      Eigen::SparseMatrix<double> H_sparse = H_o.sparseView();
      Eigen::SPQR<Eigen::SparseMatrix<double>> spqr_helper;
      spqr_helper.setSPQROrdering(SPQR_ORDERING_NATURAL);
      spqr_helper.compute(H_sparse);

      matx_t H_temp;
      vecx_t r_temp;
      (spqr_helper.matrixQ().transpose() * H_o).evalTo(H_temp);
      (spqr_helper.matrixQ().transpose() * r_o).evalTo(r_temp);

      T_H = H_temp.topRows(x_imu_sz + x_cam_sz * this->N());
      r_n = r_temp.head(x_imu_sz + x_cam_sz * this->N());

    } else if (this->qr_mode == "dense") {
      // -- Dense QR method (slower)
      Eigen::HouseholderQR<matx_t> QR(H_o);
      matx_t Q = QR.householderQ();
      matx_t Q1 = Q.leftCols(x_imu_sz + x_cam_sz * this->N());
      T_H = Q1.transpose() * H_o;
      r_n = Q1.transpose() * r_o;

    } else {
      FATAL("Invalid QR decomposition mode [%s]!", this->qr_mode.c_str());
    }

  } else {
    T_H = H_o;
    r_n = r_o;
  }

  return 0;
}

void GMSCKF::correctIMUState(const vecx_t &dx) {
  const vecx_t dx_imu = dx.head(x_imu_sz);

  // Split dx into its constituent components
  const vec3_t dtheta_IG = dx_imu.segment(0, 3);
  const vec3_t db_g = dx_imu.segment(3, 3);
  const vec3_t dv_G = dx_imu.segment(6, 3);
  const vec3_t db_a = dx_imu.segment(9, 3);
  const vec3_t dp_G = dx_imu.segment(12, 3);

  // Time derivative of quaternion (small angle approx)
  const vec4_t dq_IG = quatsmallangle(dtheta_IG);

  // Correct IMU state
  // -- State
  this->q_IG = quatnormalize(quatlcomp(dq_IG) * this->q_IG);
  this->b_g += db_g;
  this->v_G += dv_G;
  this->b_a += db_a;
  this->p_G += dp_G;
}

void GMSCKF::correctCameraStates(const vecx_t &dx) {
  for (int i = 0; i < this->N(); i++) {
    const int rs = x_imu_sz + i * x_cam_sz;
    const vecx_t dx_cam{dx.block(rs, 0, x_cam_sz, 1)};
    this->cam_states[i].correct(dx_cam);
  }
}

CameraStates GMSCKF::pruneCameraState() {
  // Pre-check
  const int N = this->N();
  if (N <= this->max_window_size) {
    return CameraStates();
  }

  // Prune camera states
  const int prune_sz = N - this->max_window_size;
  CameraStates pruned_camera_states;
  for (int i = 0; i < prune_sz; i++) {
    pruned_camera_states.push_back(this->cam_states[i]);
  }
  this->cam_states.erase(this->cam_states.begin(),
                         this->cam_states.begin() + prune_sz);

  // Adjust covariance matrix
  {
    const int rs = 0;
    const int cs = CameraState::size * prune_sz;
    const int imu_cam_rows = x_imu_sz;
    const int imu_cam_cols = this->P_cam.cols() - CameraState::size * prune_sz;
    this->P_imu_cam = this->P_imu_cam.block(rs, cs, imu_cam_rows, imu_cam_cols);
  }
  {
    const int rs = CameraState::size * prune_sz;
    const int cs = CameraState::size * prune_sz;
    const int cam_rows = this->P_cam.rows() - CameraState::size * prune_sz;
    const int cam_cols = this->P_cam.cols() - CameraState::size * prune_sz;
    this->P_cam = this->P_cam.block(rs, cs, cam_rows, cam_cols);
  }

  return pruned_camera_states;
}

int GMSCKF::measurementUpdate(const FeatureTracks &tracks,
                              const vec2_t &theta) {
  // Add a camera state to state vector
  this->augmentState(theta);
  this->cam_timestamps.push_back(
      (this->imu_timestamps.back() - this->timestamp_first) * 1.0e-9);

  // Make sure there aren't too many tracks to residualize
  FeatureTracks filtered_tracks;
  if (tracks.size() > static_cast<size_t>(this->max_nb_tracks)) {
    for (int i = 0; i < this->max_nb_tracks; i++) {
      filtered_tracks.push_back(tracks[i]);
    }
  } else {
    filtered_tracks = tracks;
  }

  std::cout << "filtered_tracks:  " << filtered_tracks.size() << std::endl;

  // Calculate residuals
  matx_t T_H;
  vecx_t r_n;
  if (this->calcResiduals(filtered_tracks, T_H, r_n) != 0) {
    /* LOG_WARN("No tracks made it through!"); */
    return 0;
  }

  // Calculate the Kalman gain.
  const matx_t P = this->P();
  const matx_t R_n = this->img_var * I(T_H.rows());
  // -- Using Cholesky decomposition for matrix inversion
  // Note: Below is equiv to P * T_H^T * (T_H * P * T_H^T + R_n)^-1
  const matx_t S = T_H * P * T_H.transpose() + R_n;
  const matx_t K = S.ldlt().solve(T_H * P).transpose();
  // -- Conventional Kalman gain calculation
  // const matx_t K =
  //     P * T_H.transpose() * (T_H * P * T_H.transpose() + R_n).inverse();

  // Correct states
  const vecx_t dx = K * r_n;
  this->correctIMUState(dx);
  this->correctCameraStates(dx);
  LOG_INFO("Correct estimation!");

  // Update covariance matrices
  const matx_t I_KH = I(K.rows(), T_H.cols()) - K * T_H;
  // -- Simplest, most unstable form
  // const matx_t P_new = I_KH * P;
  // -- Symmetric and positive Joseph form
  // const matx_t P_new = I_KH * P * I_KH.transpose() + K * R_n * K.transpose();
  // -- Upenn's version?
  const matx_t P_new = ((I_KH * P) + (I_KH * P).transpose()) / 2.0;
  // -- Partition out main covariance matrix back into its constituents
  const int N = this->N();
  this->P_imu = P_new.block(0, 0, x_imu_sz, x_imu_sz);
  this->P_cam = P_new.block(x_imu_sz, x_imu_sz, N * x_cam_sz, N * x_cam_sz);
  this->P_imu_cam = P_new.block(0, x_imu_sz, x_imu_sz, N * x_cam_sz);

  // Prune camera state to maintain sliding window size
  const CameraStates pruned_cam_states = this->pruneCameraState();
  for (const auto &cam_state : pruned_cam_states) {
    this->old_cam_states.push_back(cam_state);
  }

  return 0;
}

} //  namespace prototype
