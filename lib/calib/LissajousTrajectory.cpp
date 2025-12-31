#include "LissajousTrajectory.hpp"

namespace cartesian {

LissajousTrajectory::LissajousTrajectory(const std::string &traj_type_,
                                         const timestamp_t ts_start_,
                                         const Mat4 &T_WT_,
                                         const Mat4 &T_TO_,
                                         const double calib_width_,
                                         const double calib_height_,
                                         const double R_,
                                         const double T_)
    : traj_type{traj_type_}, ts_start{ts_start_}, T_WT{T_WT_}, T_TO{T_TO_},
      calib_width{calib_width_},
      calib_height{calib_height_}, R{R_}, T{T_}, f{1.0 / T}, w{2.0 * M_PI * f} {
  // Trajectory type
  if (traj_type == "fig8-horiz") {
    a = 2.0 * M_PI * 1.0;
    b = 2.0 * M_PI * 2.0;
    delta = M_PI;
    A = calib_width * 0.3;
    B = calib_height * 0.3;
    yaw_scale = 1.0;
    pitch_scale = 2.0;
    yaw_bound = -atan2(A, R);
    pitch_bound = -atan2(B, R);

  } else if (traj_type == "fig8-vert") {
    a = 2.0 * M_PI * 2.0;
    b = 2.0 * M_PI * 1.0;
    delta = M_PI;
    A = calib_width * 0.3;
    B = calib_height * 0.3;
    yaw_scale = 2.0;
    pitch_scale = 1.0;
    yaw_bound = -atan2(A, R);
    pitch_bound = -atan2(B, R);

  } else if (traj_type == "s-horiz") {
    a = 2.0 * M_PI * 1.0;
    b = 2.0 * M_PI * 3.0;
    delta = M_PI;
    A = calib_width * 0.5;
    B = calib_height * 0.5;
    yaw_scale = 1.0;
    pitch_scale = 0.0;
    yaw_bound = -atan2(A, R);
    pitch_bound = -atan2(B, R);

  } else if (traj_type == "s-vert") {
    a = 2.0 * M_PI * 3.0;
    b = 2.0 * M_PI * 1.0;
    delta = M_PI;
    A = calib_width * 0.55;
    B = calib_height * 0.55;
    yaw_scale = 0.0;
    pitch_scale = 1.0;
    yaw_bound = -atan2(A, R);
    pitch_bound = -atan2(B, R);

  } else if (traj_type == "pan-vert") {
    a = 2.0 * M_PI * 0.0;
    b = 2.0 * M_PI * 1.0;
    delta = 0.0;
    A = calib_width * 0.95;
    B = calib_height * 0.95;
    yaw_scale = 1.0;
    pitch_scale = 1.0;
    yaw_bound = 0.0;
    pitch_bound = -atan2(B, R);

  } else if (traj_type == "pan-horiz") {
    a = 2.0 * M_PI * 1.0;
    b = 2.0 * M_PI * 0.0;
    delta = 0.0;
    A = calib_width * 0.95;
    B = calib_height * 0.95;
    yaw_scale = 1.0;
    pitch_scale = 1.0;
    yaw_bound = atan2(A, R);
    pitch_bound = 0.0;

  } else if (traj_type == "diag0") {
    a = 2.0 * M_PI * 1.0;
    b = 2.0 * M_PI * 1.0;
    delta = 0.0;
    A = calib_width * 0.6;
    B = calib_height * 0.6;
    yaw_scale = 1.0;
    pitch_scale = 1.0;
    yaw_bound = atan2(A, R);
    pitch_bound = -atan2(B, R);

  } else if (traj_type == "diag1") {
    a = 2.0 * M_PI * 1.0;
    b = 2.0 * M_PI * 1.0;
    delta = M_PI;
    A = calib_width * 0.6;
    B = calib_height * 0.6;
    yaw_scale = 1.0;
    pitch_scale = 1.0;
    yaw_bound = -atan2(A, R);
    pitch_bound = -atan2(B, R);

  } else {
    throw std::runtime_error("Implementation Error!");
  }
}

Quat LissajousTrajectory::get_q_OS(const timestamp_t ts_k) const {
  const double t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  const double w = 2.0 * M_PI * f;
  const double k = pow(sin(0.25 * w * t), 2.0);

  const double att_x = M_PI + pitch_bound * sin(2 * M_PI * pitch_scale * k);
  const double att_y = yaw_bound * sin(2 * M_PI * yaw_scale * k);
  const double att_z = 0.0;

  const double phi = att_x;
  const double theta = att_y;
  const double psi = att_z;

  const double c_phi = cos(phi / 2.0);
  const double c_theta = cos(theta / 2.0);
  const double c_psi = cos(psi / 2.0);
  const double s_phi = sin(phi / 2.0);
  const double s_theta = sin(theta / 2.0);
  const double s_psi = sin(psi / 2.0);

  const double qw = c_psi * c_theta * c_phi + s_psi * s_theta * s_phi;
  const double qx = c_psi * c_theta * s_phi - s_psi * s_theta * c_phi;
  const double qy = c_psi * s_theta * c_phi + s_psi * c_theta * s_phi;
  const double qz = s_psi * c_theta * c_phi - c_psi * s_theta * s_phi;

  const double mag = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  const Quat q_OS{qw / mag, qx / mag, qy / mag, qz / mag};

  return q_OS;
}

Quat LissajousTrajectory::get_q_OS_dot(const timestamp_t ts_k) const {
  const double t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  // clang-format off
  const double w = 2.0 * M_PI * f;
  const double q_OS_w_dot = -1.5707963267948966*pitch_bound*pitch_scale*w*sin(0.25*t*w)*sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)*cos(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2))*cos(0.25*t*w)*cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2)))/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)) - 1.5707963267948966*w*yaw_bound*yaw_scale*sin(0.25*t*w)*sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2)))*cos(0.25*t*w)*cos(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))*cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2));
  const double q_OS_x_dot = 1.5707963267948966*pitch_bound*pitch_scale*w*sin(0.25*t*w)*cos(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2))*cos(0.25*t*w)*cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2)))*cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)) - 1.5707963267948966*w*yaw_bound*yaw_scale*sin(0.25*t*w)*sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2)))*sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)*cos(0.25*t*w)*cos(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2));
  const double q_OS_y_dot = -1.5707963267948966*pitch_bound*pitch_scale*w*sin(0.25*t*w)*sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2)))*sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)*cos(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2))*cos(0.25*t*w)/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)) + 1.5707963267948966*w*yaw_bound*yaw_scale*sin(0.25*t*w)*cos(0.25*t*w)*cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2)))*cos(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))*cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2));
  const double q_OS_z_dot = -1.5707963267948966*pitch_bound*pitch_scale*w*sin(0.25*t*w)*sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2)))*cos(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2))*cos(0.25*t*w)*cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)) - 1.5707963267948966*w*yaw_bound*yaw_scale*sin(0.25*t*w)*sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966)*cos(0.25*t*w)*cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2)))*cos(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))/sqrt(pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2) + pow(sin(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2)*pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2) + pow(cos(0.5*yaw_bound*sin(6.2831853071795862*yaw_scale*pow(sin(0.25*t*w), 2))), 2)*pow(cos(0.5*pitch_bound*sin(6.2831853071795862*pitch_scale*pow(sin(0.25*t*w), 2)) + 1.5707963267948966), 2));
  // clang-format on

  return Quat{q_OS_w_dot, q_OS_x_dot, q_OS_y_dot, q_OS_z_dot};
}

Mat4 LissajousTrajectory::get_pose(const timestamp_t ts_k) const {
  const double t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  // Setup
  const double w = 2.0 * M_PI * f;
  const double theta = pow(sin(0.25 * w * t), 2);

  // Rotation
  const double pitch = pitch_bound * sin(w * T * pitch_scale * theta);
  const double yaw = yaw_bound * sin(w * T * yaw_scale * theta);
  const Vec3 rpy{M_PI + pitch, yaw, 0.0};
  const Mat3 C_OS = euler321(rpy);

  // Position
  const double x = A * sin(a * theta + delta);
  const double y = B * sin(b * theta);
  const double z = sqrt(R * R - x * x - y * y);
  const Vec3 r_OS{x, y, z};

  // Form pose
  const Mat4 T_OS = tf(C_OS, r_OS);
  const Mat4 T_WS = T_WT * T_TO * T_OS;

  return T_WS;
}

Vec3 LissajousTrajectory::get_velocity(const timestamp_t ts_k) const {
  const double t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  const double vx = 0.5 * A * a * w * sin(0.25 * t * w) * cos(0.25 * t * w) *
                    cos(a * pow(sin(0.25 * t * w), 2) + delta);
  const double vy = 0.5 * B * b * w * sin(0.25 * t * w) *
                    cos(b * pow(sin(0.25 * t * w), 2)) * cos(0.25 * t * w);
  const double vz =
      (-0.5 * pow(A, 2) * a * w * sin(0.25 * t * w) *
           sin(a * pow(sin(0.25 * t * w), 2) + delta) * cos(0.25 * t * w) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta) -
       0.5 * pow(B, 2) * b * w * sin(b * pow(sin(0.25 * t * w), 2)) *
           sin(0.25 * t * w) * cos(b * pow(sin(0.25 * t * w), 2)) *
           cos(0.25 * t * w)) /
      sqrt(-pow(A, 2) * pow(sin(a * pow(sin(0.25 * t * w), 2) + delta), 2) -
           pow(B, 2) * pow(sin(b * pow(sin(0.25 * t * w), 2)), 2) + pow(R, 2));
  const Vec3 v_OS{vx, vy, vz};

  // Transform velocity from calib origin frame (O) to world frame (W)
  const Mat4 T_WO = T_WT * T_TO;
  const Mat3 C_WO = tf_rot(T_WO);
  const Vec3 v_WS = C_WO * v_OS;

  return v_WS;
}

Vec3 LissajousTrajectory::get_acceleration(const timestamp_t ts_k) const {
  const double t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  const double ax =
      A * a * pow(w, 2) *
      (-0.03125 * a * (1 - cos(1.0 * t * w)) *
           sin(-1.0 / 2.0 * a * cos(0.5 * t * w) + (1.0 / 2.0) * a + delta) -
       0.125 * pow(sin(0.25 * t * w), 2) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta) +
       0.125 * pow(cos(0.25 * t * w), 2) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta));
  const double ay =
      -0.25 * B * pow(b, 2) * pow(w, 2) * sin(b * pow(sin(0.25 * t * w), 2)) *
          pow(sin(0.25 * t * w), 2) * pow(cos(0.25 * t * w), 2) -
      0.125 * B * b * pow(w, 2) * pow(sin(0.25 * t * w), 2) *
          cos(b * pow(sin(0.25 * t * w), 2)) +
      0.125 * B * b * pow(w, 2) * cos(b * pow(sin(0.25 * t * w), 2)) *
          pow(cos(0.25 * t * w), 2);
  const double az =
      (-0.5 * pow(A, 2) * a * w * sin(0.25 * t * w) *
           sin(a * pow(sin(0.25 * t * w), 2) + delta) * cos(0.25 * t * w) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta) -
       0.5 * pow(B, 2) * b * w * sin(b * pow(sin(0.25 * t * w), 2)) *
           sin(0.25 * t * w) * cos(b * pow(sin(0.25 * t * w), 2)) *
           cos(0.25 * t * w)) *
          (0.5 * pow(A, 2) * a * w * sin(0.25 * t * w) *
               sin(a * pow(sin(0.25 * t * w), 2) + delta) * cos(0.25 * t * w) *
               cos(a * pow(sin(0.25 * t * w), 2) + delta) +
           0.5 * pow(B, 2) * b * w * sin(b * pow(sin(0.25 * t * w), 2)) *
               sin(0.25 * t * w) * cos(b * pow(sin(0.25 * t * w), 2)) *
               cos(0.25 * t * w)) /
          pow(-pow(A, 2) * pow(sin(a * pow(sin(0.25 * t * w), 2) + delta), 2) -
                  pow(B, 2) * pow(sin(b * pow(sin(0.25 * t * w), 2)), 2) +
                  pow(R, 2),
              3.0 / 2.0) +
      (0.25 * pow(A, 2) * pow(a, 2) * pow(w, 2) * pow(sin(0.25 * t * w), 2) *
           pow(sin(a * pow(sin(0.25 * t * w), 2) + delta), 2) *
           pow(cos(0.25 * t * w), 2) -
       0.25 * pow(A, 2) * pow(a, 2) * pow(w, 2) * pow(sin(0.25 * t * w), 2) *
           pow(cos(0.25 * t * w), 2) *
           pow(cos(a * pow(sin(0.25 * t * w), 2) + delta), 2) +
       0.125 * pow(A, 2) * a * pow(w, 2) * pow(sin(0.25 * t * w), 2) *
           sin(a * pow(sin(0.25 * t * w), 2) + delta) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta) -
       0.125 * pow(A, 2) * a * pow(w, 2) *
           sin(a * pow(sin(0.25 * t * w), 2) + delta) *
           pow(cos(0.25 * t * w), 2) *
           cos(a * pow(sin(0.25 * t * w), 2) + delta) +
       0.25 * pow(B, 2) * pow(b, 2) * pow(w, 2) *
           pow(sin(b * pow(sin(0.25 * t * w), 2)), 2) *
           pow(sin(0.25 * t * w), 2) * pow(cos(0.25 * t * w), 2) -
       0.25 * pow(B, 2) * pow(b, 2) * pow(w, 2) * pow(sin(0.25 * t * w), 2) *
           pow(cos(b * pow(sin(0.25 * t * w), 2)), 2) *
           pow(cos(0.25 * t * w), 2) +
       0.125 * pow(B, 2) * b * pow(w, 2) * sin(b * pow(sin(0.25 * t * w), 2)) *
           pow(sin(0.25 * t * w), 2) * cos(b * pow(sin(0.25 * t * w), 2)) -
       0.125 * pow(B, 2) * b * pow(w, 2) * sin(b * pow(sin(0.25 * t * w), 2)) *
           cos(b * pow(sin(0.25 * t * w), 2)) * pow(cos(0.25 * t * w), 2)) /
          sqrt(-pow(A, 2) * pow(sin(a * pow(sin(0.25 * t * w), 2) + delta), 2) -
               pow(B, 2) * pow(sin(b * pow(sin(0.25 * t * w), 2)), 2) +
               pow(R, 2));
  const Vec3 a_OS{ax, ay, az};

  // Transform velocity from calib origin frame (O) to world frame (W)
  const Mat4 T_WO = T_WT * T_TO;
  const Mat3 C_WO = tf_rot(T_WO);
  const Vec3 a_WS = C_WO * a_OS;

  return a_WS;
}

Vec3 LissajousTrajectory::get_angular_velocity(const timestamp_t ts_k) const {
  const double t = ts2sec(ts_k) - ts2sec(ts_start);
  if (t < 0.0 || t > T) {
    FATAL("Implementation Error!");
  }

  // Angular velocity
  const Quat q_OS = get_q_OS(ts_k);
  const Quat q_OS_dot = get_q_OS_dot(ts_k);
  const Vec3 w_OS = 2.0 * (q_OS_dot * q_OS.inverse()).vec();

  // Transform velocity from calib origin frame (O) to world frame (W)
  const Mat4 T_WO = T_WT * T_TO;
  const Mat3 C_WO = tf_rot(T_WO);
  const Vec3 w_WS = C_WO * w_OS;

  return w_WS;
}

int LissajousTrajectory::save(const std::string &save_path) const {
  // Setup output file
  std::ofstream file{save_path};
  if (file.good() != true) {
    LOG_ERROR("Failed to open file for output!");
    return -1;
  }

  // File Header
  file << "#ts,";                                    // Timestamp
  file << "r_WS_W_x,r_WS_W_y,r_WS_W_z,";             // Position
  file << "q_WS_W_x,q_WS_W_y,q_WS_W_z,q_WS_W_w,";    // Rotation
  file << "v_WS_W_x,v_WS_W_y,v_WS_W_z,";             // Velocity
  file << "a_WS_S_x,a_WS_S_y,a_WS_S_z,";             // Acceleartion
  file << "w_WS_S_x,w_WS_S_y,w_WS_S_z" << std::endl; // Angular velocity

  // Output trajectory timestamps, positions and orientations as csv
  const timestamp_t ts_end = ts_start + sec2ts(T);
  const size_t num_positions = 1000;
  const auto timestamps = linspace(ts_start, ts_end, num_positions);

  for (const auto ts : timestamps) {
    // Sensor pose, velocity, acceleration and angular velocity
    const Mat4 T_WS_W = get_pose(ts);
    const Mat3 C_WS_W = tf_rot(T_WS_W);
    const Vec3 r_WS_W = tf_trans(T_WS_W);
    const Quat q_WS_W = tf_quat(T_WS_W);
    const Vec3 v_WS_W = get_velocity(ts);
    const Vec3 a_WS_W = get_acceleration(ts);
    const Vec3 w_WS_W = get_angular_velocity(ts);

    // Compute accelerometer and gyroscope measurement
    const Vec3 g_W{0.0, 0.0, 10.0}; // Gravity vector
    const Mat3 C_SW_W = C_WS_W.transpose();
    const Vec3 w_WS_S = C_SW_W * w_WS_W;
    const Vec3 a_WS_S = C_SW_W * (a_WS_W + g_W);

    // Timestamp
    file << ts << ",";
    // Position in world frame
    file << r_WS_W.x() << ",";
    file << r_WS_W.y() << ",";
    file << r_WS_W.z() << ",";
    // Rotation in world frame
    file << q_WS_W.x() << ",";
    file << q_WS_W.y() << ",";
    file << q_WS_W.z() << ",";
    file << q_WS_W.w() << ",";
    // Velocity in world frame
    file << v_WS_W.x() << ",";
    file << v_WS_W.y() << ",";
    file << v_WS_W.z() << ",";
    // Acceleartion in body frame
    file << a_WS_S.x() << ",";
    file << a_WS_S.y() << ",";
    file << a_WS_S.z() << ",";
    // Angular velocity in body frame
    file << w_WS_S.x() << ",";
    file << w_WS_S.y() << ",";
    file << w_WS_S.z() << std::endl;
  }

  // Close file
  file.close();
  return 0;
}

} // namespace cartesian
