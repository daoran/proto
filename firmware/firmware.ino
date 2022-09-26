#include "firmware.h"

// GLOBAL VARIABLES
// -- Timers
uint32_t task_control_last_updated_us = 0;
uint32_t task_rc_last_updated_us = 0;
uint32_t task_telem_last_updated_us = 0;
// -- Flags
uint8_t mav_arm = 0;
uint8_t mav_ready = 0;
uint8_t mav_failsafe = 0;
// -- Control
float attitude_offset[3] = {0.0, 0.0, 0.0};
float thrust_desired = 0.0;
float roll_desired = 0.0;
float pitch_desired = 0.0;
float yaw_desired = 0.0;
float outputs[4] = {0.0, 0.0, 0.0, 0.0};
// -- Telemetry data
#define TELEM_BUF_SIZE 10
uint16_t buf_size = 0;
uint32_t ts_buf[TELEM_BUF_SIZE] = {0};
float acc_buf[TELEM_BUF_SIZE][3] = {0};
float gyr_buf[TELEM_BUF_SIZE][3] = {0};
float attitude_buf[TELEM_BUF_SIZE][3] = {0};
float desired_buf[TELEM_BUF_SIZE][4] = {0};
float outputs_buf[TELEM_BUF_SIZE][4] = {0};
uint8_t mav_arm_buf[TELEM_BUF_SIZE] = {0};
uint8_t mav_ready_buf[TELEM_BUF_SIZE] = {0};
uint8_t mav_failsafe_buf[TELEM_BUF_SIZE] = {0};
uint16_t sbus_buf[TELEM_BUF_SIZE][16] = {0};
// -- Components
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
  mpu6050_calibrate(&imu);

  // Control
  mahony_filter_setup(&filter, imu.accel);
  att_ctrl_setup(&att_ctrl);

  // Update times
  const uint32_t time_now_us = micros();
  task_control_last_updated_us = time_now_us;
  task_rc_last_updated_us = time_now_us;
}

// Reset timers
void reset_timers() {
  const uint32_t time_now_us = micros();
  task_control_last_updated_us = time_now_us;
  task_rc_last_updated_us = time_now_us;
}

// Reset flags
void reset_flags() {
  mav_arm = 0;
  mav_ready = 0;
  mav_failsafe = 0;
}

// Startup Sequence
void startup_seqeunce() {
  // Calibrate gyroscope
  const uint16_t nb_samples = 1000;
  float w[3] = {0.0, 0.0, 0.0};
  for (uint32_t i = 0; i < nb_samples; i++) {
    mpu6050_get_data(&imu);
    w[0] += imu.gyro[0];
    w[1] += imu.gyro[1];
    w[2] += imu.gyro[2];
  }
  imu.gyro_offset[0] = w[0] / (float) nb_samples;
  imu.gyro_offset[1] = w[1] / (float) nb_samples;
  imu.gyro_offset[2] = w[2] / (float) nb_samples;

  // Motor startup sequence
  pwm_set(&pwm, 2, THROTTLE_MIN);
  delay(500);
  pwm_set(&pwm, 3, THROTTLE_MIN);
  delay(500);
  pwm_set(&pwm, 1, THROTTLE_MIN);
  delay(500);
  pwm_set(&pwm, 0, THROTTLE_MIN);
  delay(500);

  // Reset timing
  reset_timers();
  mav_ready = 1;
}

// Estimation Task
void task_control() {
  // Check loop time
  const uint32_t time_now_us = micros();
  const float dt_s = (time_now_us - task_control_last_updated_us) * 1e-6;
  if (dt_s < IMU_SAMPLE_PERIOD_S) {
    return;
  }

  // Launch startup sequence
  if (mav_arm == 1 && mav_ready == 0) {
    startup_seqeunce();
    return;
  }

  /* uart_printf(&uart, "ts:%d ", time_now_us); */
  /* char dt_str[10]; //  Hold The Convert Data */
  /* dtostrf(dt_s, 10, 10, dt_str); */
  /* uart_printf(&uart, "dt:%s ", dt_str); */
  /* uart_printf(&uart, "\r\n"); */

  // Estimate attitude
  mpu6050_get_data(&imu);
  const float acc[3] = {imu.accel[0], imu.accel[1], imu.accel[2]};
  const float gyr[3] = {imu.gyro[0] - imu.gyro_offset[0],
                        imu.gyro[1] - imu.gyro_offset[1],
                        imu.gyro[2] - imu.gyro_offset[2]};
  mahony_filter_update(&filter, acc, gyr, dt_s);
  const float roll_actual = filter.roll - attitude_offset[0];
  const float pitch_actual = filter.pitch - attitude_offset[1];
  const float yaw_actual = filter.yaw - attitude_offset[2];

  // Check tilt
  const uint8_t roll_ok = within(roll_actual, -MAX_TILT_RAD, MAX_TILT_RAD);
  const uint8_t pitch_ok = within(pitch_actual, -MAX_TILT_RAD, MAX_TILT_RAD);
  if (roll_ok == 0 || pitch_ok == 0) {
    mav_failsafe = 1;
  }

  // Calculate control outputs
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

  // Set motor PWMs
  if (mav_arm && mav_failsafe == 0) {
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

  // Record data
  ts_buf[buf_size] = time_now_us;
  acc_buf[buf_size][0] = acc[0];
  acc_buf[buf_size][1] = acc[1];
  acc_buf[buf_size][2] = acc[2];
  gyr_buf[buf_size][0] = gyr[0];
  gyr_buf[buf_size][1] = gyr[1];
  gyr_buf[buf_size][2] = gyr[2];
  attitude_buf[buf_size][0] = roll_actual;
  attitude_buf[buf_size][1] = pitch_actual;
  attitude_buf[buf_size][2] = yaw_actual;
  desired_buf[buf_size][0] = roll_desired;
  desired_buf[buf_size][1] = pitch_desired;
  desired_buf[buf_size][2] = yaw_desired;
  desired_buf[buf_size][3] = thrust_desired;
  outputs_buf[buf_size][0] = outputs[0];
  outputs_buf[buf_size][1] = outputs[1];
  outputs_buf[buf_size][2] = outputs[2];
  outputs_buf[buf_size][3] = outputs[3];
  mav_arm_buf[buf_size] = mav_arm;
  mav_ready_buf[buf_size] = mav_ready;
  mav_failsafe_buf[buf_size] = mav_failsafe;
  for (uint8_t i = 0; i < 16; i++) {
    sbus_buf[buf_size][i] = sbus.ch[i];
  }
  buf_size++;

  if (buf_size >= TELEM_BUF_SIZE) {
    task_batch_telem();
  }

  // Update last updated
  task_control_last_updated_us = time_now_us;
}

// RC Task
void task_rc() {
  // Check loop time
  const uint32_t time_now_us = micros();
  const float dt_s = (time_now_us - task_rc_last_updated_us) * 1e-6;
  if (dt_s < RC_SAMPLE_PERIOD_S) {
    return;
  }

  // Get SBUS data
  if (sbus_update(&sbus) != 0) {
    return;
  }

  // Parse SBUS signal
  mav_arm = sbus_arm(&sbus);
  thrust_desired = sbus_thrust(&sbus);
  roll_desired = sbus_roll(&sbus);
  pitch_desired = sbus_pitch(&sbus);
  yaw_desired = sbus_yaw(&sbus);

  // Radio lost?
  if (us2sec(time_now_us - task_rc_last_updated_us) > 0.3) {
    mav_arm = 0;
    mav_ready = 0;
    mav_failsafe = 1;
  }

  // Reset failsafe
  if (mav_arm == 0) {
    mav_ready = 0;
    mav_failsafe = 0;
    mahony_filter_reset_yaw(&filter);
  }

  // Update last updated
  task_rc_last_updated_us = time_now_us;
}

void task_telem() {
  // Check loop time
  const uint32_t time_now_us = micros();
  const float dt_s = (time_now_us - task_telem_last_updated_us) * 1e-6;
  if (dt_s < TELEM_SAMPLE_PERIOD_S) {
    return;
  }

  uart_printf(&uart, "ts:%d ", micros());
  char dt_str[10]; //  Hold The Convert Data
  dtostrf(dt_s, 10, 10, dt_str);
  uart_printf(&uart, "dt:%s ", dt_str);
  uart_printf(&uart, "mav_arm:%d ", mav_arm);
  uart_printf(&uart, "mav_ready:%d ", mav_ready);

  for (uint8_t i = 0; i < 16; i++) {
    uart_printf(&uart, "ch[%d]:%d ", i, sbus.ch[i]);
  }

  uart_printf(&uart, "accel_x:%f ", imu.accel[0]);
  uart_printf(&uart, "accel_y:%f ", imu.accel[1]);
  uart_printf(&uart, "accel_z:%f ", imu.accel[2]);

  uart_printf(&uart, "gyro_x:%f ", imu.gyro[0] - imu.gyro_offset[0]);
  uart_printf(&uart, "gyro_y:%f ", imu.gyro[1] - imu.gyro_offset[1]);
  uart_printf(&uart, "gyro_z:%f ", imu.gyro[2] - imu.gyro_offset[2]);

  uart_printf(&uart, "roll:%f ", rad2deg(filter.roll - attitude_offset[0]));
  uart_printf(&uart, "pitch:%f ", rad2deg(filter.pitch - attitude_offset[1]));
  uart_printf(&uart, "yaw:%f ", rad2deg(filter.yaw - attitude_offset[2]));

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
  task_telem_last_updated_us = time_now_us;
}

void task_batch_telem() {
  const uint32_t ts_telem = micros();

  for (uint16_t k = 0; k < buf_size; k++) {
    char s[1024] = {0};

    sprintf(s, "telem_ts:%ld ", ts_telem);
    sprintf(s + strlen(s), "ts:%ld ", ts_buf[k]);
    sprintf(s + strlen(s), "mav_arm:%d ", mav_arm);
    sprintf(s + strlen(s), "mav_ready:%d ", mav_ready);

    for (uint8_t i = 0; i < 16; i++) {
      sprintf(s + strlen(s), "ch[%d]:%d ", i, sbus_buf[k][i]);
    }

    sprintf(s + strlen(s), "accel_x:%f ", acc_buf[k][0]);
    sprintf(s + strlen(s), "accel_y:%f ", acc_buf[k][1]);
    sprintf(s + strlen(s), "accel_z:%f ", acc_buf[k][2]);

    sprintf(s + strlen(s), "gyro_x:%f ", gyr_buf[k][0]);
    sprintf(s + strlen(s), "gyro_y:%f ", gyr_buf[k][1]);
    sprintf(s + strlen(s), "gyro_z:%f ", gyr_buf[k][2]);

    sprintf(s + strlen(s), "roll:%f ", rad2deg(attitude_buf[k][0]));
    sprintf(s + strlen(s), "pitch:%f ", rad2deg(attitude_buf[k][1]));
    sprintf(s + strlen(s), "yaw:%f ", rad2deg(attitude_buf[k][2]));

    sprintf(s + strlen(s), "roll_desired:%f ", rad2deg(desired_buf[k][0]));
    sprintf(s + strlen(s), "pitch_desired:%f ", rad2deg(desired_buf[k][1]));
    sprintf(s + strlen(s), "yaw_desired:%f ", rad2deg(desired_buf[k][2]));
    sprintf(s + strlen(s), "thrust_desired:%f ", desired_buf[k][3]);

    sprintf(s + strlen(s), "outputs[0]:%f ", outputs_buf[k][0]);
    sprintf(s + strlen(s), "outputs[1]:%f ", outputs_buf[k][1]);
    sprintf(s + strlen(s), "outputs[2]:%f ", outputs_buf[k][2]);
    sprintf(s + strlen(s), "outputs[3]:%f ", outputs_buf[k][3]);

    sprintf(s + strlen(s), "\r\n");

    uart_printf(&uart, "%s", s);

    // Clear buffer
    ts_buf[k] = 0;
    acc_buf[k][0] = 0;
    acc_buf[k][1] = 0;
    acc_buf[k][2] = 0;
    gyr_buf[k][0] = 0;
    gyr_buf[k][1] = 0;
    gyr_buf[k][2] = 0;
    attitude_buf[k][0] = 0;
    attitude_buf[k][1] = 0;
    attitude_buf[k][2] = 0;
    desired_buf[k][0] = 0;
    desired_buf[k][1] = 0;
    desired_buf[k][2] = 0;
    desired_buf[k][3] = 0;
    outputs_buf[k][0] = 0;
    outputs_buf[k][1] = 0;
    outputs_buf[k][2] = 0;
    outputs_buf[k][3] = 0;
    mav_arm_buf[k] = 0;
    mav_ready_buf[k] = 0;
    mav_failsafe_buf[k] = 0;
    sbus_buf[k][0] = 0;
    sbus_buf[k][1] = 0;
    sbus_buf[k][2] = 0;
    sbus_buf[k][3] = 0;
    sbus_buf[k][4] = 0;
    sbus_buf[k][5] = 0;
    sbus_buf[k][6] = 0;
    sbus_buf[k][7] = 0;
    sbus_buf[k][8] = 0;
    sbus_buf[k][9] = 0;
    sbus_buf[k][10] = 0;
    sbus_buf[k][11] = 0;
    sbus_buf[k][12] = 0;
    sbus_buf[k][13] = 0;
    sbus_buf[k][14] = 0;
    sbus_buf[k][15] = 0;
  }

  // Reset buffer size
  buf_size = 0;
}

// Loop
void loop() {
  task_control();
  task_rc();
  /* task_telem(); */
}
