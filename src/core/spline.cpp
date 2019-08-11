#include "proto/core/spline.hpp"

namespace proto {

/*****************************************************************************
 * Continuous trajectory generator
 *****************************************************************************/

ctraj_t::ctraj_t(const timestamps_t &timestamps,
                 const vec3s_t &positions,
                 const quats_t &orientations)
    : timestamps{timestamps},
      positions{positions},
      orientations{orientations},
      ts_s_start{ts2sec(timestamps.front())},
      ts_s_end{ts2sec(timestamps.back())},
      ts_s_gap{ts_s_end - ts_s_start} {
  assert(timestamps.size() == positions.size());
  assert(timestamps.size() == orientations.size());
  assert(timestamps.size() > (spline_degree + 1));
  ctraj_init(*this);
}

inline static double ts_normalize(const ctraj_t &ctraj, const timestamp_t ts) {
  const double ts_s_k = ts2sec(ts);
  const double ts_s_start = ctraj.ts_s_start;
  const double ts_s_end = ctraj.ts_s_end;
  return (ts_s_k - ts_s_start) / (ts_s_end - ts_s_start);
}

void ctraj_init(ctraj_t &ctraj) {
  assert(timestamps.size() == positions.size());
  assert(timestamps.size() == orientations.size());
  assert(ctraj.timestamps.size() > (ctraj.spline_degree + 1));

  // Create knots
  const size_t nb_knots = ctraj.timestamps.size();
  Eigen::RowVectorXd knots{nb_knots};
  for (size_t i = 0; i < ctraj.timestamps.size(); i++) {
    knots(i) = ts_normalize(ctraj, ctraj.timestamps[i]);
  }

  // Prep position data
  matx_t pos{3, nb_knots};
  for (size_t i = 0; i < nb_knots; i++) {
    pos.block<3, 1>(0, i) = ctraj.positions[i];
  }

  // Prep orientation data
  matx_t rvec{3, nb_knots};
  Eigen::AngleAxisd aa{ctraj.orientations[0]};
  rvec.block<3, 1>(0, 0) = aa.angle() * aa.axis();

  for (size_t i = 1; i < nb_knots; i++) {
    const Eigen::AngleAxisd aa{ctraj.orientations[i]};
    const vec3_t rvec_k = aa.angle() * aa.axis();
    const vec3_t rvec_km1 = rvec.block<3, 1>(0, i - 1);

    // Calculate delta from rvec_km1 to rvec_k
    vec3_t delta = rvec_k - rvec_km1;
    while (delta.squaredNorm() > (M_PI * M_PI)) {
      delta -= 2 * M_PI * delta.normalized();
    }

    // Add new rotation vector
    rvec.block<3, 1>(0, i) = rvec_km1 + delta;
  }

  // Create splines
  const int spline_degree = 3;
  ctraj.pos_spline = SPLINE3D(pos, knots, spline_degree);
  ctraj.rvec_spline = SPLINE3D(rvec, knots, spline_degree);
}

mat4_t ctraj_get_pose(const ctraj_t &ctraj, const timestamp_t ts) {
  const double u = ts_normalize(ctraj, ts);

  // Translation
  const vec3_t r = ctraj.pos_spline(u);

  // Orientation
  const vec3_t rvec = ctraj.rvec_spline(u);
  if (rvec.norm() < 1e-12) { // Check angle is not zero
    return tf(quat_t(), r);
  }
  const Eigen::AngleAxisd aa{rvec.norm(), rvec.normalized()};
  const quat_t q{aa};

  return tf(q, r);
}

vec3_t ctraj_get_velocity(const ctraj_t &ctraj, const timestamp_t ts) {
  assert(ts >= ctraj.timestamps.front());
  assert(ts <= ctraj.timestamps.back());

  const double u = ts_normalize(ctraj, ts);
  const double scale = (1 / ctraj.ts_s_gap);

  return ctraj.pos_spline.derivatives(u, 1).col(1) * scale;
}

vec3_t ctraj_get_acceleration(const ctraj_t &ctraj, const timestamp_t ts) {
  assert(ts >= ctraj.timestamps.front());
  assert(ts <= ctraj.timestamps.back());

  const double u = ts_normalize(ctraj, ts);
  const double scale = pow(1 / ctraj.ts_s_gap, 2);

  return ctraj.pos_spline.derivatives(u, 2).col(2) * scale;
}

static mat3_t so3_exp(const vec3_t &phi) {
  const double norm = phi.norm();
  if (norm < 1e-3) {
    return mat3_t{I(3) + skew(phi)};
  }

  const mat3_t phi_skew = skew(phi);
  mat3_t C = I(3);
  C += (sin(norm) / norm) * phi_skew;
  C += ((1 - cos(norm)) / (norm * norm)) * (phi_skew * phi_skew);

  return C;
}

vec3_t ctraj_get_angular_velocity(const ctraj_t &ctraj, const timestamp_t ts) {
  assert(ts >= ctraj.timestamps.front());
  assert(ts <= ctraj.timestamps.back());

  const double u = ts_normalize(ctraj, ts);
  const double scale = 1 / ctraj.ts_s_gap;

  const auto rvec_spline_deriv = ctraj.rvec_spline.derivatives(u, 1);
  const vec3_t rvec = rvec_spline_deriv.col(0);
  const vec3_t rvec_deriv = rvec_spline_deriv.col(1) * scale;

  // Check magnitude of the rotation vector
  const double rvec_norm = rvec.norm();
  if (rvec_norm < 1e-12) {
    return vec3_t{rvec_deriv};
  }

  // Calculate angular velocity
  const mat3_t axis_skew = skew(rvec.normalized());
  vec3_t w =
    (I(3) + axis_skew * (1.0 - cos(rvec_norm)) / rvec_norm
    + axis_skew * axis_skew * (rvec_norm - sin(rvec_norm)) / rvec_norm)
    * rvec_deriv;

  return w;
}

int ctraj_save(const ctraj_t &ctraj, const std::string &save_path) {
  // Setup output file
  std::ofstream file{save_path};
  if (file.good() != true) {
    LOG_ERROR("Failed to open file for output!");
    return -1;
  }

  // Output trajectory timestamps, positions and orientations as csv
  for (size_t i = 0; i < ctraj.timestamps.size(); i++) {
    file << ctraj.timestamps[i] << ",";
    file << ctraj.positions[i](0) << ",";
    file << ctraj.positions[i](1) << ",";
    file << ctraj.positions[i](2) << ",";
    file << ctraj.orientations[i].w() << ",";
    file << ctraj.orientations[i].x() << ",";
    file << ctraj.orientations[i].y() << ",";
    file << ctraj.orientations[i].z() << std::endl;
  }

  // Close file
  file.close();
  return 0;
}

/*****************************************************************************
 * IMU measurements generator
 *****************************************************************************/

void sim_imu_measurement(
    sim_imu_t &imu,
    std::default_random_engine &rndeng,
    const timestamp_t &ts,
    const mat4_t &T_WS_W,
    const vec3_t &w_WS_W,
    const vec3_t &a_WS_W,
    vec3_t &a_WS_S,
    vec3_t &w_WS_S) {
  // Delta time according to sample rate
  double dt = 1.0 / imu.rate;

  // Check consistency of time increments:
  if (imu.started == false) {
    if (fabs(ts2sec(ts - imu.ts_prev) - dt) < (dt / 2.0)) {
      FATAL("Inconsisten sample rate with parameter setting: %f < %f",
            fabs(double(ts - imu.ts_prev) * 1.0e-9 - dt),
            dt / 2.0);
    }
  }

  // IMU initialised?
  if (imu.started == false) {
    // Stationary properties of an Ornstein-Uhlenbeck process
    imu.b_g = mvn(rndeng) * imu.sigma_gw_c * sqrt(imu.tau_g / 2.0);
    imu.b_a = mvn(rndeng) * imu.sigma_aw_c * sqrt(imu.tau_a / 2.0);
    imu.started = true;

  } else {
    // Propagate biases (slow moving signal)
    const vec3_t w_a = mvn(rndeng);  // Accel white noise
    imu.b_a += -imu.b_a / imu.tau_a * dt + w_a * imu.sigma_aw_c * sqrt(dt);
    const vec3_t w_g = mvn(rndeng);  // Gyro white noise
    imu.b_g += -imu.b_g / imu.tau_g * dt + w_g * imu.sigma_gw_c * sqrt(dt);
  }

  // Compute gyro measurement
  const mat3_t C_SW = tf_rot(T_WS_W).transpose();
  const vec3_t w_g = mvn(rndeng);  // Gyro white noise
  w_WS_S = C_SW * w_WS_W + imu.b_g + w_g * imu.sigma_g_c * sqrt(dt);

  // Compute accel measurement
  const vec3_t g{0.0, 0.0, -imu.g}; // Gravity vector
  const vec3_t w_a = mvn(rndeng); // Accel white noise
  a_WS_S = C_SW * (a_WS_W - g) + imu.b_a + w_a * imu.sigma_a_c * sqrt(dt);
  // TODO: check global gravity direction!

  // Saturate
  // elementwiseSaturate(imu.g_max, w_WS_S);
  // elementwiseSaturate(imu.a_max, a_WS_S);
  imu.ts_prev = ts;
}

} // namespace proto
