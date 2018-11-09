#ifndef PROTOTYPE_CONTROL_PID_HPP
#define PROTOTYPE_CONTROL_PID_HPP

#include <float.h>
#include <iostream>
#include <math.h>

namespace prototype {

/**
 * PID Controller
 */
struct pid_t {
  double error_prev = 0.0;
  double error_sum = 0.0;

  double error_p = 0.0;
  double error_i = 0.0;
  double error_d = 0.0;

  double k_p = 0.0;
  double k_i = 0.0;
  double k_d = 0.0;

  pid_t();
  pid_t(const double k_p, const double k_i, const double k_d);
  ~pid_t();
};

/**
 * `pid_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const pid_t &pid);

/**
 * Update controller
 *
 * @returns Controller command
 */
double pid_update(pid_t &p,
                  const double setpoint,
                  const double actual,
                  const double dt);

/**
 * Update controller
 *
 * @returns Controller command
 */
double pid_update(pid_t &p, const double error, const double dt);

/**
 * Reset controller
 */
void pid_reset(pid_t &p);

} //  namespace prototype
#endif // PROTOTYPE_CONTROL_PID_HPP
