/**
 * @file
 * @ingroup control
 */
#ifndef PROTOTYPE_CONTROL_PID_HPP
#define PROTOTYPE_CONTROL_PID_HPP

#include <float.h>
#include <iostream>
#include <math.h>

namespace prototype {
/**
 * @addtogroup control
 * @{
 */

/**
 * PID Controller
 */
struct pid {
  double error_prev = 0.0;
  double error_sum = 0.0;

  double error_p = 0.0;
  double error_i = 0.0;
  double error_d = 0.0;

  double k_p = 0.0;
  double k_i = 0.0;
  double k_d = 0.0;
};

/**
 * Setup PID
 *
 * @param[in] k_p Proportional constant
 * @param[in] k_i Integral constant
 * @param[in] k_d Derivative constant
 */
struct pid pid_setup(const double k_p, const double k_i, const double k_d);

/**
 * Update controller
 *
 * @param[in/out] PID controller
 * @param[in] setpoint Setpoint
 * @param[in] actual Actual
 * @param[in] dt Time step
 * @returns Controller command
 */
double pid_update(struct pid &p,
                  const double setpoint,
                  const double actual,
                  const double dt);

/**
 * Update controller
 *
 * @param[in/out] PID controller
 * @param[in] error Error between setpoint and actual
 * @param[in] dt Difference in time
 * @returns Controller command
 */
double pid_update(struct pid &p, const double error, const double dt);

/**
 * Reset controller
 */
void pid_reset(struct pid &p);

/** @} group control */
} //  namespace prototype
#endif // PROTOTYPE_CONTROL_PID_HPP
