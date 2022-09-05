#include "firmware.h"

// GLOBAL VARIABLES
uint32_t time_prev_us = 0;
float estimation_time_s = 0;
float control_time_s = 0;
uint32_t control_last_updated = 0;
sbus_t sbus;
uart_t uart;
pwm_t pwm;
mpu6050_t imu;
compl_filter_t filter;
att_ctrl_t att_ctrl;

// IMU Task
void estimation_task(const float dt_s) {
  estimation_time_s += dt_s;
  if (estimation_time_s < IMU_SAMPLE_PERIOD_S) {
    return;
  }

  mpu6050_get_data(&imu);
  compl_filter_update(&filter, imu.accel, imu.gyro, dt_s);
  // uart_printf(&uart, "roll:%f ", rad2deg(filter.roll));
  // uart_printf(&uart, "pitch:%f ", rad2deg(filter.pitch));
  // uart_printf(&uart, "yaw:%f ", rad2deg(filter.yaw));
  // uart_printf(&uart, "\r\n");

  // Reset timer
  estimation_time_s = 0.0;
}

// Control Task
void control_task(const uint32_t time_us, const float dt_s) {
  // Check loop time
  control_time_s += dt_s;
  if (control_time_s < SBUS_SAMPLE_PERIOD_S) {
    return;
  }

  // Get SBUS data
  if (sbus_update(&sbus) != 0) {
    return;
  }

  // Arm switch on?
  if (sbus.ch[4] < ((PWM_VALUE_MAX - PWM_VALUE_MIN) / 2.0)) {
    control_time_s = 0;
    pwm_set(&pwm, 0, 0.0);
    pwm_set(&pwm, 1, 0.0);
    pwm_set(&pwm, 2, 0.0);
    pwm_set(&pwm, 3, 0.0);
    return;
  }

  // Calculate motor controls
  const float thrust_desired =
      (sbus.ch[0] - PWM_VALUE_MIN) / (PWM_VALUE_MAX - PWM_VALUE_MIN);
  const float roll_actual = filter.roll;
  const float pitch_actual = filter.pitch;
  const float yaw_actual = filter.yaw;

  const float roll_desired = 0.0;
  const float pitch_desired = 0.0;
  const float yaw_desired = 0.0;
  float outputs[4] = {0.0, 0.0, 0.0, 0.0};
  att_ctrl_update(&att_ctrl,
                  roll_desired,
                  pitch_desired,
                  yaw_desired,
                  thrust_desired,
                  roll_actual,
                  pitch_actual,
                  yaw_actual,
                  dt_s,
                  outputs,
                  &uart);

  // uart_printf(&uart, "ch0:%d ", ch[0]);
  // uart_printf(&uart, "ch1:%d ", ch[1]);
  // uart_printf(&uart, "ch2:%d ", ch[2]);
  // uart_printf(&uart, "ch3:%d ", ch[3]);
  // uart_printf(&uart, "ch4:%d ", ch[4]);
  // uart_printf(&uart, "\r\n");

  // uart_printf(&uart, "outputs[0]:%f ", outputs[0]);
  // uart_printf(&uart, "outputs[1]:%f ", outputs[1]);
  // uart_printf(&uart, "outputs[2]:%f ", outputs[2]);
  // uart_printf(&uart, "outputs[3]:%f ", outputs[3]);
  // uart_printf(&uart, "\r\n");

  // Set motor speeds
  // pwm_set(&pwm, 0, outputs[0]);
  // pwm_set(&pwm, 1, outputs[1]);
  // pwm_set(&pwm, 2, outputs[2]);
  // pwm_set(&pwm, 3, outputs[3]);

  if ((time_us - control_last_updated) > 300000) {
    pwm_set(&pwm, 0, 0.0);
    pwm_set(&pwm, 1, 0.0);
    pwm_set(&pwm, 2, 0.0);
    pwm_set(&pwm, 3, 0.0);
  } else {
    pwm_set(&pwm, 0, outputs[0]);
    pwm_set(&pwm, 1, outputs[1]);
    pwm_set(&pwm, 2, outputs[2]);
    pwm_set(&pwm, 3, outputs[3]);
  }

  uart_printf(&uart, "control_time_prev:%d ", control_last_updated);
  uart_printf(&uart, "control_time_now:%d ", time_us);
  uart_printf(&uart, "control_time_diff:%d ", time_us - control_last_updated);
  uart_printf(&uart, "\r\n");

  // Reset timer
  control_time_s = 0;
  control_last_updated = time_us;
}

// Setup
void setup() {
  // Comms
  pwm_setup(&pwm);
  uart_setup(&uart);
  i2c_setup();
  sbus_setup(&sbus);
  delay(1000);

  // Sensors
  mpu6050_setup(&imu);
  mpu6050_get_data(&imu);

  // Control
  compl_filter_setup(&filter, COMPL_FILTER_ALPHA, imu.accel);
  att_ctrl_setup(&att_ctrl);
}

// Loop
void loop() {
  // for (uint8_t i = 0; i < 4; i++) {
  //   pwm_set(&pwm, i, 0.1);
  //   delay(500);
  //   pwm_set(&pwm, i, 0.0);
  // }
  // delay(5000);

  const uint32_t time_now_us = micros();
  const uint32_t dt_us = time_now_us - time_prev_us;
  const float dt_s = dt_us * 1e-6;
  estimation_task(dt_s);
  control_task(time_now_us, dt_s);
  time_prev_us = time_now_us;
}
