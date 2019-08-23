#include "proto/model/mav.hpp"

namespace proto {

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
