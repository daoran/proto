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
float attitude_offset[3] = {0.0, 0.0, 0.0};       // roll, pitch, yaw
float setpoint_desired[4] = {0.0, 0.0, 0.0, 0.0}; // roll, pitch, yaw, thrust
float outputs[4] = {0.0, 0.0, 0.0, 0.0};          // motor 0, 1, 2, 3

#define VEL_BUF_SIZE 5
float buf_vz[VEL_BUF_SIZE] = {0};
uint8_t buf_vz_idx = 0;
// -- Components
sbus_t sbus;
uart_t uart;
pwm_t pwm;
mpu6050_t imu;
mahony_filter_t filter;
att_ctrl_t att_ctrl;
telemetry_t telem;

// Setup
void setup() {
  // Comms
  pwm_setup(&pwm);
  uart_setup(&uart);
  i2c_setup();
  sbus_setup(&sbus);
  telemetry_setup(&telem);
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

  // Estimate velocity
  const float euler[3] = {roll_actual, pitch_actual, yaw_actual};
  const float v_S[3] = {acc[0] * IMU_SAMPLE_PERIOD_S,
                        acc[1] * IMU_SAMPLE_PERIOD_S,
                        acc[2] * IMU_SAMPLE_PERIOD_S};
  float C_WS[3 * 3] = {0};
  float v_WS[3] = {0.0, 0.0, 0.0};
  euler321(euler, C_WS);
  dot(C_WS, 3, 3, v_S, 3, 1, v_WS);
  v_WS[2] -= imu.gravity * IMU_SAMPLE_PERIOD_S;

  // float vz = v_WS[2];
  float vz = 0.0;
  if (buf_vz_idx >= VEL_BUF_SIZE) {
    // Make a copy of the buffer
    float vals[VEL_BUF_SIZE] = {0};
    for (uint8_t i = 0; i < VEL_BUF_SIZE; i++) {
      vals[i] = buf_vz[i];
    }

    // Sort values
    qsort(vals, VEL_BUF_SIZE, sizeof(float), fltcmp);

    // Get median value
    float median = 0.0;
    uint8_t n = VEL_BUF_SIZE;
    if ((n % 2) == 0) {
      const uint8_t bwd_idx = (uint8_t)(n - 1) / 2.0;
      const uint8_t fwd_idx = (uint8_t)(n + 1) / 2.0;
      median = (vals[bwd_idx] + vals[fwd_idx]) / 2.0;
    } else {
      const uint8_t mid_idx = n / 2.0;
      median = vals[mid_idx];
    }

    // Set median
    vz = median;

    // Reset
    for (uint8_t i = 0; i < VEL_BUF_SIZE; i++) {
      buf_vz[i] = 0;
    }
    buf_vz_idx = 0;

  } else {
    vz = v_WS[2];
    buf_vz[buf_vz_idx++] = vz;
  }

  // Check tilt
  // const float actual[4] = {roll_actual, pitch_actual, yaw_actual, v_WS[2]};
  const float actual[4] = {roll_actual, pitch_actual, yaw_actual, vz};
  const uint8_t roll_ok = within(actual[0], -MAX_TILT_RAD, MAX_TILT_RAD);
  const uint8_t pitch_ok = within(actual[1], -MAX_TILT_RAD, MAX_TILT_RAD);
  if (roll_ok == 0 || pitch_ok == 0) {
    mav_failsafe = 1;
  }

  // Calculate control outputs
  att_ctrl_update(&att_ctrl,
                  setpoint_desired[0],
                  setpoint_desired[1],
                  setpoint_desired[2],
                  setpoint_desired[3],
                  actual[0],
                  actual[1],
                  actual[2],
                  IMU_SAMPLE_PERIOD_S,
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

  // Telemetry
  telemetry_record(&telem,
                   time_now_us,
                   mav_arm,
                   mav_ready,
                   mav_failsafe,
                   acc,
                   gyr,
                   actual,
                   setpoint_desired,
                   outputs,
                   sbus.ch);
  if (telem.buf_size >= TELEM_BUF_SIZE) {
    telemetry_transmit(&telem, &uart);
    telemetry_reset(&telem);
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
  setpoint_desired[0] = sbus_roll(&sbus);
  setpoint_desired[1] = sbus_pitch(&sbus);
  setpoint_desired[2] = sbus_yaw(&sbus);
  setpoint_desired[3] = sbus_thrust(&sbus);

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

// Loop
void loop() {
  task_control();
  task_rc();
}
