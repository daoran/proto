#include "firmware.h"

// GLOBAL VARIABLES
uint32_t task0_last_updated_us = 0;
uint32_t task1_last_updated_us = 0;
uint32_t telem_last_updated_us = 0;
uint8_t arm = 0;
uint8_t failsafe = 0;

float thrust_desired = 0.0;
float roll_desired = 0.0;
float pitch_desired = 0.0;
float yaw_desired = 0.0;
float outputs[4] = {0.0, 0.0, 0.0, 0.0};

sbus_t sbus;
uart_t uart;
pwm_t pwm;
mpu6050_t imu;
mahony_filter_t filter;
att_ctrl_t att_ctrl;

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
  const float roll = atan2(imu.accel[1], imu.accel[2]);
  const float pitch =
      atan2(-imu.accel[0],
            sqrt(imu.accel[1] * imu.accel[1] + imu.accel[2] * imu.accel[2]));
  const float yaw = 0.0;
  const float rpy[3] = {roll, pitch, yaw};
  float q[4] = {1.0, 0.0, 0.0, 0.0};
  euler2quat(rpy, q);
  mahony_filter_setup(&filter);
  filter.q[0] = q[0];
  filter.q[1] = q[1];
  filter.q[2] = q[2];
  filter.q[3] = q[3];
  att_ctrl_setup(&att_ctrl);

  // Update times
  const uint32_t time_now_us = micros();
  task0_last_updated_us = time_now_us;
  task1_last_updated_us = time_now_us;
}

// Estimation Task
void task0() {
  // Check loop time
  const uint32_t time_now_us = micros();
  const float dt_s = (time_now_us - task0_last_updated_us) * 1e-6;
  if (dt_s < IMU_SAMPLE_PERIOD_S) {
    return;
  }

  // Estimate attitude
  mpu6050_get_data(&imu);
  const float acc[3] = {imu.accel[0], imu.accel[1], imu.accel[2]};
  const float gyr[3] = {imu.gyro[0] - imu.gyro_offset[0],
                        imu.gyro[1] - imu.gyro_offset[1],
                        imu.gyro[2] - imu.gyro_offset[2]};
  mahony_filter_update(&filter, acc, gyr, dt_s);

  // Check tilt
  const float max_tilt = deg2rad(60.0);
  if (filter.roll > max_tilt || filter.roll < -max_tilt) {
    failsafe = 1;
  } else if (filter.pitch > max_tilt || filter.pitch < -max_tilt) {
    failsafe = 1;
  }

  // Calculate control outputs
  att_ctrl_update(&att_ctrl,
                  roll_desired,
                  pitch_desired,
                  yaw_desired,
                  thrust_desired,
                  filter.roll,
                  filter.pitch,
                  filter.yaw,
                  dt_s,
                  outputs,
                  &uart);

  // Set motor PWMs
  if (arm && failsafe == 0) {
    pwm_set(&pwm, 0, outputs[0]);
    pwm_set(&pwm, 1, outputs[1]);
    pwm_set(&pwm, 2, outputs[2]);
    pwm_set(&pwm, 3, outputs[3]);
  } else {
    pwm_set(&pwm, 0, 0.0);
    pwm_set(&pwm, 1, 0.0);
    pwm_set(&pwm, 2, 0.0);
    pwm_set(&pwm, 3, 0.0);
  }

  // Update last updated
  task0_last_updated_us = time_now_us;
}

// Control Task
void task1() {
  // Check loop time
  const uint32_t time_now_us = micros();
  const float dt_s = (time_now_us - task1_last_updated_us) * 1e-6;
  if (dt_s < (1.0 / 50.0)) {
    return;
  }

  // Get SBUS data
  if (sbus_update(&sbus) != 0) {
    return;
  }

  // Parse SBUS signal
  arm = sbus_arm(&sbus);
  thrust_desired = sbus_thrust(&sbus);
  roll_desired = sbus_roll(&sbus);
  pitch_desired = sbus_pitch(&sbus);
  yaw_desired = sbus_yaw(&sbus);

  // Radio lost?
  if (us2sec(time_now_us - task1_last_updated_us) > 0.3) {
    arm = 0;
    failsafe = 1;
  }

  // Reset failsafe
  if (arm == 0) {
    failsafe = 0;
    mahony_filter_reset_yaw(&filter);
  }

  // Update last updated
  task1_last_updated_us = time_now_us;
}

void transmit_telemetry() {
  // Check loop time
  const uint32_t time_now_us = micros();
  const float dt_s = (time_now_us - telem_last_updated_us) * 1e-6;
  if (dt_s < (1.0 / 100.0)) {
    return;
  }

  uart_printf(&uart, "ts:%d ", micros());
  char dt_str[10]; //  Hold The Convert Data
  dtostrf(dt_s, 10, 10, dt_str);
  uart_printf(&uart, "dt:%s ", dt_str);

  for (uint8_t i = 0; i < 16; i++) {
    uart_printf(&uart, "ch[%d]:%d ", i, sbus.ch[i]);
  }

  uart_printf(&uart, "accel_x:%f ", imu.accel[0]);
  uart_printf(&uart, "accel_y:%f ", imu.accel[1]);
  uart_printf(&uart, "accel_z:%f ", imu.accel[2]);
  uart_printf(&uart, "gyro_x:%f ", imu.gyro[0]);
  uart_printf(&uart, "gyro_y:%f ", imu.gyro[1]);
  uart_printf(&uart, "gyro_z:%f ", imu.gyro[2]);
  uart_printf(&uart, "roll:%f ", rad2deg(filter.roll));
  uart_printf(&uart, "pitch:%f ", rad2deg(filter.pitch));
  uart_printf(&uart, "yaw:%f ", rad2deg(filter.yaw));

  uart_printf(&uart, "thrust_desired:%f ", rad2deg(thrust_desired));
  uart_printf(&uart, "roll_desired:%f ", rad2deg(roll_desired));
  uart_printf(&uart, "pitch_desired:%f ", rad2deg(pitch_desired));
  uart_printf(&uart, "yaw_desired:%f ", rad2deg(yaw_desired));

  uart_printf(&uart, "outputs[0]:%f ", outputs[0]);
  uart_printf(&uart, "outputs[1]:%f ", outputs[1]);
  uart_printf(&uart, "outputs[2]:%f ", outputs[2]);
  uart_printf(&uart, "outputs[3]:%f ", outputs[3]);

  uart_printf(&uart, "\r\n");

  // Update last updated
  telem_last_updated_us = time_now_us;
}

// Loop
void loop() {
  task0();
  task1();
  transmit_telemetry();
}
