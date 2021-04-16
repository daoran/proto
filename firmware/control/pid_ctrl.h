/*******************************************************************************
 *                                 PID CTRL
 ******************************************************************************/

typedef struct pid_ctrl_t {
  float error_prev;
  float error_sum;

  float error_p;
  float error_i;
  float error_d;

  float kp;
  float ki;
  float kd;
} pid_ctrl_t;

void pid_ctrl_setup(pid_ctrl_t *pid,
                    const float kp,
                    const float ki,
                    const float kd) {
  pid->error_prev = 0.0f;
  pid->error_sum = 0.0f;

  pid->error_p = 0.0f;
  pid->error_i = 0.0f;
  pid->error_d = 0.0f;

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

float pid_ctrl_update(pid_ctrl_t *pid,
                      const float setpoint,
                      const float actual,
                      const float dt) {
  // Calculate errors
  const float error = setpoint - actual;
  pid->error_sum += error * dt;

  // Calculate output
  pid->error_p = pid->kp * error;
  pid->error_i = pid->ki * pid->error_sum;
  pid->error_d = pid->kd * (error - pid->error_prev) / dt;
  const float output = pid->error_p + pid->error_i + pid->error_d;

  pid->error_prev = error;
  return output;
}

void pid_ctrl_reset(pid_ctrl_t *pid) {
  pid->error_prev = 0.0;
  pid->error_sum = 0.0;

  pid->error_p = 0.0;
  pid->error_i = 0.0;
  pid->error_d = 0.0;
}
