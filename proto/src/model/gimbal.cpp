#include "proto/model/gimbal.hpp"

namespace proto {

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

} //  namespace proto
