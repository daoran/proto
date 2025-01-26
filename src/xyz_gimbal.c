#include "xyz_gimbal.h"

void gimbal_model_setup(gimbal_model_t *gimbal) {
  gimbal->x[0] = 0.0;
  gimbal->x[1] = 0.0;

  gimbal->x[2] = 0.0;
  gimbal->x[3] = 0.0;

  gimbal->x[4] = 0.0;
  gimbal->x[5] = 0.0;
}

void gimbal_model_update(gimbal_model_t *gimbal,
                         const real_t u[3],
                         const real_t dt) {
  const real_t ph = gimbal->x[0];
  const real_t ph_vel = gimbal->x[1];

  const real_t th = gimbal->x[2];
  const real_t th_vel = gimbal->x[3];

  const real_t psi = gimbal->x[4];
  const real_t psi_vel = gimbal->x[5];

  gimbal->x[0] = ph + ph_vel * dt;
  gimbal->x[1] = ph_vel + u[0] * dt;

  gimbal->x[2] = th + th_vel * dt;
  gimbal->x[3] = th_vel + u[1] * dt;

  gimbal->x[4] = psi + psi_vel * dt;
  gimbal->x[5] = psi_vel + u[2] * dt;
}

void gimbal_ctrl_setup(gimbal_ctrl_t *ctrl) {
  ctrl->dt = 0;
  pid_ctrl_setup(&ctrl->roll, 0.5, 0.0, 20.0);
  pid_ctrl_setup(&ctrl->pitch, 0.5, 0.0, 20.0);
  pid_ctrl_setup(&ctrl->yaw, 0.1, 0.0, 6.0);

  zeros(ctrl->setpoints, 3, 1);
  zeros(ctrl->outputs, 3, 1);
}

void gimbal_ctrl_update(gimbal_ctrl_t *ctrl,
                        const real_t setpoints[3],
                        const real_t actual[3],
                        const real_t dt,
                        real_t outputs[3]) {
  // Limit rate to 1000Hz
  ctrl->dt += dt;
  if (ctrl->dt <= 0.001) {
    return;
  }

  // Roll, pitch and yaw joints
  real_t r = pid_ctrl_update(&ctrl->roll, setpoints[0], actual[0], ctrl->dt);
  real_t p = pid_ctrl_update(&ctrl->pitch, setpoints[1], actual[1], ctrl->dt);
  real_t y = pid_ctrl_update(&ctrl->yaw, setpoints[2], actual[2], ctrl->dt);
  outputs[0] = r;
  outputs[1] = p;
  outputs[2] = y;

  // Update
  ctrl->outputs[0] = r;
  ctrl->outputs[1] = p;
  ctrl->outputs[2] = y;
  ctrl->dt = 0.0;
}
