#include "control.h"

void pid_ctrl_setup(pid_ctrl_t *pid,
                    const real_t kp,
                    const real_t ki,
                    const real_t kd) {
  pid->error_prev = 0.0;
  pid->error_sum = 0.0;

  pid->error_p = 0.0;
  pid->error_i = 0.0;
  pid->error_d = 0.0;

  pid->k_p = kp;
  pid->k_i = ki;
  pid->k_d = kd;
}

real_t pid_ctrl_update(pid_ctrl_t *pid,
                       const real_t setpoint,
                       const real_t input,
                       const real_t dt) {
  // Calculate errors
  real_t error = setpoint - input;
  pid->error_sum += error * dt;

  // Calculate output
  pid->error_p = pid->k_p * error;
  pid->error_i = pid->k_i * pid->error_sum;
  pid->error_d = pid->k_d * (error - pid->error_prev) / dt;
  real_t output = pid->error_p + pid->error_i + pid->error_d;

  // Update error
  pid->error_prev = error;

  return output;
}

void pid_ctrl_reset(pid_ctrl_t *pid) {
  pid->error_prev = 0;
  pid->error_sum = 0;

  pid->error_p = 0;
  pid->error_i = 0;
  pid->error_d = 0;
}
