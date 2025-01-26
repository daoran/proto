#pragma once

#include "xyz.h"
#include "xyz_control.h"

typedef struct gimbal_model_t {
  real_t x[6];
} gimbal_model_t;

typedef struct gimbal_ctrl_t {
  real_t dt;
  pid_ctrl_t roll;
  pid_ctrl_t pitch;
  pid_ctrl_t yaw;

  real_t setpoints[3];
  real_t outputs[3];
} gimbal_ctrl_t;

void gimbal_model_setup(gimbal_model_t *gimbal);
void gimbal_model_update(gimbal_model_t *gimbal,
                         const real_t u[3],
                         const real_t dt);
void gimbal_ctrl_setup(gimbal_ctrl_t *ctrl);
void gimbal_ctrl_update(gimbal_ctrl_t *ctrl,
                        const real_t setpoints[3],
                        const real_t actual[3],
                        const real_t dt,
                        real_t outputs[3]);
