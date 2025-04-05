#pragma once

#include <inttypes.h>

// Settings
#ifndef PRECISION
#define PRECISION 2
#endif

// Float Precision
#ifndef REAL_TYPE
#define REAL_TYPE
#if PRECISION == 1
typedef float real_t;
#elif PRECISION == 2
typedef double real_t;
#else
#error "Floating Point Precision not defined!"
#endif
#endif

// Timestamp Type
#ifndef TIMESTAMP_TYPE
#define TIMESTAMP_TYPE
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
