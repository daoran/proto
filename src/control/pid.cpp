#include "prototype/control/pid.hpp"

namespace prototype {

struct pid pid_setup(const double k_p, const double k_i, const double k_d) {
  struct pid p;
  p.k_p = k_p;
  p.k_i = k_i;
  p.k_d = k_d;

  return p;
}

double pid_update(struct pid &p,
                  const double setpoint,
                  const double input,
                  const double dt) {
  // Calculate errors
  double error = setpoint - input;
  p.error_sum += error * dt;

  // Calculate output
  p.error_p = p.k_p * error;
  p.error_i = p.k_i * p.error_sum;
  p.error_d = p.k_d * (error - p.error_prev) / dt;
  double output = p.error_p + p.error_i + p.error_d;

  // Update error
  p.error_prev = error;

  return output;
}

double pid_update(struct pid &p, const double error, const double dt) {
  return pid_update(p, error, 0.0, dt);
}

void pid_reset(struct pid &p) {
  p.error_prev = 0.0;
  p.error_sum = 0.0;

  p.error_p = 0.0;
  p.error_i = 0.0;
  p.error_d = 0.0;
}

} //  namespace prototype
