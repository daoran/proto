#pragma once

#include <inttypes.h>

// Float Precision
#ifndef real_t
typedef double real_t;
#endif

// Timestamp Type
#ifndef timestamp_t
typedef int64_t timestamp_t;
#endif

typedef struct pid_ctrl_t {
  real_t error_prev;
  real_t error_sum;

  real_t error_p;
  real_t error_i;
  real_t error_d;

  real_t k_p;
  real_t k_i;
  real_t k_d;
} pid_ctrl_t;

void pid_ctrl_setup(pid_ctrl_t *pid,
                    const real_t kp,
                    const real_t ki,
                    const real_t kd);
real_t pid_ctrl_update(pid_ctrl_t *pid,
                       const real_t setpoint,
                       const real_t input,
                       const real_t dt);
void pid_ctrl_reset(pid_ctrl_t *pid);
