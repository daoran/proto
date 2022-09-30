#include "firmware.h"

// GLOBAL VARIABLES
// -- Timers
uint32_t task_control_last_updated_us = 0;
uint32_t task_rc_last_updated_us = 0;
uint32_t task_bmp_last_updated_us = 0;
uint32_t task_telem_last_updated_us = 0;
// -- Flags
uint8_t mav_arm = 0;
uint8_t mav_ready = 0;
uint8_t mav_failsafe = 0;
// -- Control
float acc[3] = {0.0, 0.0, 0.0};
float gyr[3] = {0.0, 0.0, 0.0};
float state[4] = {0.0, 0.0, 0.0, 0.0};      // roll, pitch, yaw, height
float attitude_offset[3] = {0.0, 0.0, 0.0}; // roll, pitch, yaw
float setpoint[4] = {0.0, 0.0, 0.0, 0.0};   // roll, pitch, yaw, thrust
float outputs[4] = {0.0, 0.0, 0.0, 0.0};    // motor 0, 1, 2, 3
// -- Components
sbus_t sbus;
uart_t uart;
pwm_t pwm;
mpu6050_t imu;
bmp280_t bmp;
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
  bmp280_setup(&bmp);

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

  // Get IMU data
  mpu6050_get_data(&imu);
  acc[0] = imu.accel[0];
  acc[1] = imu.accel[1];
  acc[2] = imu.accel[2];
  gyr[0] = imu.gyro[0] - imu.gyro_offset[0];
  gyr[1] = imu.gyro[1] - imu.gyro_offset[1];
  gyr[2] = imu.gyro[2] - imu.gyro_offset[2];

  // Esimate attitude
  mahony_filter_update(&filter, acc, gyr, dt_s);
  state[0] = filter.roll - attitude_offset[0];
  state[1] = filter.pitch - attitude_offset[1];
  state[2] = filter.yaw - attitude_offset[2];

  // Check tilt
  const float actual[3] = {state[0], state[1], state[2]};
  const uint8_t roll_ok = within(actual[0], -MAX_TILT_RAD, MAX_TILT_RAD);
  const uint8_t pitch_ok = within(actual[1], -MAX_TILT_RAD, MAX_TILT_RAD);
  if (roll_ok == 0 || pitch_ok == 0) {
    mav_failsafe = 1;
  }

  // Calculate control outputs
  att_ctrl_update(&att_ctrl, setpoint, actual, IMU_SAMPLE_PERIOD_S, outputs);

  // Set motor PWMs
#if DEBUG_MODE == 0
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
#else
  pwm_set(&pwm, 0, 0.0);
  pwm_set(&pwm, 1, 0.0);
  pwm_set(&pwm, 2, 0.0);
  pwm_set(&pwm, 3, 0.0);
#endif

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
  setpoint[0] = sbus_roll(&sbus);
  setpoint[1] = sbus_pitch(&sbus);
  setpoint[2] = sbus_yaw(&sbus);
  setpoint[3] = sbus_thrust(&sbus);

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

// Pressure Task
void task_bmp() {
  // Check loop time
  const uint32_t time_now_us = micros();
  const float dt_s = (time_now_us - task_bmp_last_updated_us) * 1e-6;
  if (dt_s < BMP_SAMPLE_PERIOD_S) {
    return;
  }

  // Get temperature and pressure
  float temperature = 0.0f;
  float pressure_Pa = 0.0f;
  bmp280_get_data(&bmp, &temperature, &pressure_Pa);

  // Calculate height above sealevel
  const float pressure_hPa = pressure_Pa / 100;
  const float sealevel_hPa = 1013.25f;
  state[3] = 44330.0 * (1.0 - pow(pressure_hPa / sealevel_hPa, 0.1903));

  // Update last updated
  task_bmp_last_updated_us = time_now_us;
}

// Telemetry Task
void task_telem() {
  // Check loop time
  const uint32_t time_now_us = micros();
  const float dt_s = (time_now_us - task_telem_last_updated_us) * 1e-6;
  if (dt_s < TELEM_SAMPLE_PERIOD_S) {
    return;
  }

  // Telemetry
  telemetry_record(&telem,
                   time_now_us,
                   mav_arm,
                   mav_ready,
                   mav_failsafe,
                   acc,
                   gyr,
                   state,
                   setpoint,
                   outputs,
                   sbus.ch);
  telemetry_transmit(&telem, &uart);
  telemetry_reset(&telem);

  // Update last updated
  task_telem_last_updated_us = time_now_us;
}

// Loop
void loop() {
  task_control();
  task_rc();
  // task_bmp();
#if DEBUG_MODE == 1
  task_telem();
#endif
}
