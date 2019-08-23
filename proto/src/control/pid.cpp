#include "proto/control/pid.hpp"

namespace proto {

pid_t::pid_t() {}

pid_t::pid_t(const double k_p_, const double k_i_, const double k_d_)
    : k_p{k_p_}, k_i{k_i_}, k_d{k_d_} {}

pid_t::~pid_t() {}

std::ostream &operator<<(std::ostream &os, const pid_t &pid) {
  os << "error_prev:" << pid.error_prev << std::endl;
  os << "error_sum:" << pid.error_sum << std::endl;
  os << "error_p:" << pid.error_p << std::endl;
  os << "error_i:" << pid.error_i << std::endl;
  os << "error_d:" << pid.error_d << std::endl;
  os << "k_p:" << pid.k_p << std::endl;
  os << "k_i:" << pid.k_i << std::endl;
  os << "k_d:" << pid.k_d << std::endl;
  return os;
}

double pid_update(pid_t &p,
                  const double setpoint,
                  const double actual,
                  const double dt) {
  // Calculate errors
  const double error = setpoint - actual;
  p.error_sum += error * dt;

  // Calculate output
  p.error_p = p.k_p * error;
  p.error_i = p.k_i * p.error_sum;
  p.error_d = p.k_d * (error - p.error_prev) / dt;
  const double output = p.error_p + p.error_i + p.error_d;

  p.error_prev = error;
  return output;
}

double pid_update(pid_t &p, const double error, const double dt) {
  return pid_update(p, error, 0.0, dt);
}

void pid_reset(pid_t &p) {
  p.error_prev = 0.0;
  p.error_sum = 0.0;

  // p.error_p = 0.0;
  // p.error_i = 0.0;
  // p.error_d = 0.0;
}

} //  namespace proto
