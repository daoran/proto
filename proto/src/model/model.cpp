#include "proto/model/model.hpp"

namespace proto {

/****************************************************************************
 *                                GIMBAL
 ***************************************************************************/

mat4_t dh_transform(const double theta,
                    const double d,
                    const double a,
                    const double alpha) {
  // clang-format off
  mat4_t T;
  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
       sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
       0.0, sin(alpha), cos(alpha), d,
       0.0, 0.0, 0.0, 1.0;
  // clang-format on

  return T;
}

gimbal_model_t::gimbal_model_t() {}

gimbal_model_t::gimbal_model_t(const vec6_t &tau_s,
                               const vec6_t &tau_d,
                               const double Lambda1,
                               const vec3_t w1,
                               const double Lambda2,
                               const vec3_t w2,
                               const double theta1_offset,
                               const double theta2_offset)
    : tau_s{tau_s}, tau_d{tau_d}, Lambda1{Lambda1}, w1{w1}, Lambda2{Lambda2},
      w2{w2}, theta1_offset{theta1_offset}, theta2_offset{theta2_offset} {}

gimbal_model_t::~gimbal_model_t() {}

void gimbal_model_set_attitude(gimbal_model_t &model,
                               const double roll_,
                               const double pitch_) {
  model.Lambda1 = roll_;
  model.Lambda2 = pitch_;
}

vec2_t gimbal_model_get_joint_angles(const gimbal_model_t &model) {
  return vec2_t{model.Lambda1, model.Lambda2};
}

mat4_t gimbal_model_T_BS(const gimbal_model_t &model) {
  mat4_t T_sb = zeros(4, 4);
  T_sb.block(0, 0, 3, 3) = euler321(model.tau_s.tail(3));
  T_sb.block(0, 3, 3, 1) = model.tau_s.head(3);
  T_sb(3, 3) = 1.0;

  return T_sb;
}

mat4_t gimbal_model_T_EB(const gimbal_model_t &model) {
  const double theta1 = model.Lambda1 + model.theta1_offset;
  const double d1 = model.w1[0];
  const double a1 = model.w1[1];
  const double alpha1 = model.w1[2];

  const double theta2 = model.Lambda2 + model.theta2_offset;
  const double d2 = model.w2[0];
  const double a2 = model.w2[1];
  const double alpha2 = model.w2[2];

  const mat4_t T_1b = dh_transform(theta1, d1, a1, alpha1).inverse();
  const mat4_t T_e1 = dh_transform(theta2, d2, a2, alpha2).inverse();
  const mat4_t T_EB = T_e1 * T_1b;

  return T_EB;
}

mat4_t gimbal_model_T_DE(const gimbal_model_t &model) {
  mat4_t T_DE = zeros(4, 4);
  T_DE.block(0, 0, 3, 3) = euler321(model.tau_d.tail(3));
  T_DE.block(0, 3, 3, 1) = model.tau_d.head(3);
  T_DE(3, 3) = 1.0;

  return T_DE;
}

mat4_t gimbal_model_T_DS(const gimbal_model_t &model) {
  const auto T_DE = gimbal_model_T_DE(model);
  const auto T_EB = gimbal_model_T_EB(model);
  const auto T_BS = gimbal_model_T_BS(model);
  return T_DE * T_EB * T_BS;
}

mat4_t gimbal_model_T_DS(gimbal_model_t &model, const vec2_t &theta) {
  gimbal_model_set_attitude(model, theta(0), theta(1));
  const auto T_DE = gimbal_model_T_DE(model);
  const auto T_EB = gimbal_model_T_EB(model);
  const auto T_BS = gimbal_model_T_BS(model);
  return T_DE * T_EB * T_BS;
}

std::ostream &operator<<(std::ostream &os, const gimbal_model_t &model) {
  os << "tau_s: " << model.tau_s.transpose() << std::endl;
  os << "tau_d: " << model.tau_d.transpose() << std::endl;
  os << "w1: " << model.w1.transpose() << std::endl;
  os << "w2: " << model.w2.transpose() << std::endl;
  os << "Lambda1: " << model.Lambda1 << std::endl;
  os << "Lambda2: " << model.Lambda2 << std::endl;
  os << "theta1_offset: " << model.theta1_offset << std::endl;
  os << "theta2_offset: " << model.theta2_offset << std::endl;
  return os;
}

/*****************************************************************************
 *                                TWO WHEEL
 ****************************************************************************/

void circle_trajectory(const double r,
                       const double v,
                       double *w,
                       double *time) {
  const double dist = 2 * M_PI * r;
  *time = dist / v;
  *w = (2 * M_PI) / *time;
}

void two_wheel_update(two_wheel_t &tm, const double dt) {
  const vec3_t p_G_prev = tm.p_G;
  const vec3_t v_G_prev = tm.v_G;
  const vec3_t rpy_G_prev = tm.rpy_G;

  tm.p_G += euler321(tm.rpy_G) * tm.v_B * dt;
  tm.v_G = (tm.p_G - p_G_prev) / dt;
  tm.a_G = (tm.v_G - v_G_prev) / dt;

  tm.rpy_G += euler321(tm.rpy_G) * tm.w_B * dt;
  tm.w_G = tm.rpy_G - rpy_G_prev;
  tm.a_B = euler123(tm.rpy_G) * tm.a_G;

  // Wrap angles to +/- pi
  for (int i = 0; i < 3; i++) {
    tm.rpy_G(i) = (tm.rpy_G(i) > M_PI) ? tm.rpy_G(i) - 2 * M_PI : tm.rpy_G(i);
    tm.rpy_G(i) = (tm.rpy_G(i) < -M_PI) ? tm.rpy_G(i) + 2 * M_PI : tm.rpy_G(i);
  }
}

/*****************************************************************************
 *                                 MAV
 ****************************************************************************/

int mav_model_update(mav_model_t &model,
                     const vec4_t &motor_inputs,
                     const double dt) {
  const double ph = model.attitude(0);
  const double th = model.attitude(1);
  const double ps = model.attitude(2);

  const double p = model.angular_velocity(0);
  const double q = model.angular_velocity(1);
  const double r = model.angular_velocity(2);

  const double x = model.position(0);
  const double y = model.position(1);
  const double z = model.position(2);

  const double vx = model.linear_velocity(0);
  const double vy = model.linear_velocity(1);
  const double vz = model.linear_velocity(2);

  const double Ix = model.Ix;
  const double Iy = model.Iy;
  const double Iz = model.Iz;

  const double kr = model.kr;
  const double kt = model.kt;

  const double m = model.m;
  const double g = model.g;

  // convert motor inputs to angular p, q, r and total thrust
  // clang-format off
  mat4_t A;
  A << 1.0, 1.0, 1.0, 1.0,
       0.0, -model.l, 0.0, model.l,
       -model.l, 0.0, model.l, 0.0,
       -model.d, model.d, -model.d, model.d;
  // clang-format on
  const vec4_t tau = A * motor_inputs;
  const double tauf = tau(0);
  const double taup = tau(1);
  const double tauq = tau(2);
  const double taur = tau(3);

  // update
  // clang-format off
  model.attitude(0) = ph + (p + q * sin(ph) * tan(th) + r * cos(ph) * tan(th)) * dt;
  model.attitude(1) = th + (q * cos(ph) - r * sin(ph)) * dt;
  model.attitude(2) = ps + ((1 / cos(th)) * (q * sin(ph) + r * cos(ph))) * dt;
  model.angular_velocity(0) = p + (-((Iz - Iy) / Ix) * q * r - (kr * p / Ix) + (1 / Ix) * taup) * dt;
  model.angular_velocity(1) = q + (-((Ix - Iz) / Iy) * p * r - (kr * q / Iy) + (1 / Iy) * tauq) * dt;
  model.angular_velocity(2) = r + (-((Iy - Ix) / Iz) * p * q - (kr * r / Iz) + (1 / Iz) * taur) * dt;
  model.position(0) = x + vx * dt;
  model.position(1) = y + vy * dt;
  model.position(2) = z + vz * dt;
  model.linear_velocity(0) = vx + ((-kt * vx / m) + (1 / m) * (cos(ph) * sin(th) * cos(ps) + sin(ph) * sin(ps)) * tauf) * dt;
  model.linear_velocity(1) = vy + ((-kt * vy / m) + (1 / m) * (cos(ph) * sin(th) * sin(ps) - sin(ph) * cos(ps)) * tauf) * dt;
  model.linear_velocity(2) = vz + (-(kt * vz / m) + (1 / m) * (cos(ph) * cos(th)) * tauf - g) * dt;
  // clang-format on

  // constrain yaw to be [-180, 180]
  // if (model.attitude(2) > M_PI) {
  //   model.attitude(2) -= 2 * M_PI;
  // } else if (model.attitude(2) < -M_PI) {
  //   model.attitude(2) += 2 * M_PI;
  // }

  return 0;
}

} //  namespace proto
