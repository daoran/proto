#include "firmware.h"

// GLOBAL VARIABLES
uint32_t time_prev_us = 0;
uint32_t imu_time_us = 0;

uint16_t ch[16] = {0};
uint8_t frame_lost = 0;
uint8_t failsafe_activated = 0;
uart_t uart;

uint8_t pwm_pins[4] = {2, 3, 4, 5};
uint8_t nb_pwm_pins = 4;
pwm_t pwm;
mpu6050_t imu;
compl_filter_t filter;
att_ctrl_t att_ctrl;

// TESTS /////////////////////////////////////////////////////////////////////

void test_uart() {
  uart_printf(&uart, "Hello World!\r\n");
  delay(1000);
}

void test_imu() {
  mpu6050_get_data(&imu);
  uart_printf(&uart, "ax:%f ", imu.accel[0]);
  uart_printf(&uart, "ay:%f ", imu.accel[1]);
  uart_printf(&uart, "az:%f ", imu.accel[2]);
  uart_printf(&uart, "gx:%f  ", imu.gyro[0]);
  uart_printf(&uart, "gy:%f  ", imu.gyro[1]);
  uart_printf(&uart, "gz:%f  ", imu.gyro[2]);
  uart_printf(&uart, "\n");
  delay(10);
}

// void test_bmp() {
//   if (bmp280_data_ready(&bmp)) {
//     float temperature = 0.0f;
//     float pressure = 0.0f;
//     bmp280_get_data(&bmp, &temperature, &pressure);
//     uart_printf(&uart, "%f\n", temperature);
//     delay(10);
//   }
// }

//////////////////////////////////////////////////////////////////////////////

void setup() {
  // Comms
  pwm_setup(&pwm, pwm_pins, nb_pwm_pins);
  uart_setup(&uart);
  i2c_setup();
  sbus_setup();
  delay(1000);

  // Sensors
  mpu6050_setup(&imu);
  mpu6050_get_data(&imu);

  // Control
  compl_filter_setup(&filter, COMPL_FILTER_ALPHA, imu.accel);
  att_ctrl_setup(&att_ctrl);
}

void test() {
  // test_uart();
  // test_imu();
  // test_bmp();
  // test_uds();
}

void loop() {
  const uint32_t time_now_us = micros();
  const uint32_t dt_us = time_now_us - time_prev_us;
  const float dt_s = dt_us * 1e-6;
  imu_time_us += dt_us;

  mpu6050_get_data(&imu);
  compl_filter_update(&filter, imu.accel, imu.gyro, dt_s);
  // uart_printf(&uart, "rate:%f ", 1.0 / dt_s);
  uart_printf(&uart, "roll:%f ", rad2deg(filter.roll));
  uart_printf(&uart, "pitch:%f ", rad2deg(filter.pitch));
  uart_printf(&uart, "yaw:%f ", rad2deg(filter.yaw));
  uart_printf(&uart, "\r\n");

  const float roll_actual = filter.roll;
  const float pitch_actual = filter.pitch;
  const float yaw_actual = filter.yaw;

  // if (sbus_update(ch, &frame_lost, &failsafe_activated) == 0) {
  //   const float thrust_desired =
  //       (ch[0] - PWM_VALUE_MIN) / (PWM_VALUE_MAX - PWM_VALUE_MIN);
  //   const float roll_desired = 0.0;
  //   const float pitch_desired = 0.0;
  //   const float yaw_desired = 0.0;
  //   float outputs[4] = {0.0, 0.0, 0.0, 0.0};
  //   att_ctrl_update(&att_ctrl,
  //                   roll_desired,
  //                   pitch_desired,
  //                   yaw_desired,
  //                   thrust_desired,
  //                   roll_actual,
  //                   pitch_actual,
  //                   yaw_actual,
  //                   dt_s,
  //                   outputs);
  //   pwm_set(&pwm, 0, outputs[0]);
  //   pwm_set(&pwm, 1, outputs[1]);
  //   pwm_set(&pwm, 2, outputs[2]);
  //   pwm_set(&pwm, 3, outputs[3]);

  //   // pwm_set(&pwm, 0, thrust_desired);
  //   // pwm_set(&pwm, 1, thrust_desired);
  //   // pwm_set(&pwm, 2, thrust_desired);
  //   // pwm_set(&pwm, 3, thrust_desired);

  //   uart_printf(&uart, "time: %d ", time_now_us);
  //   uart_printf(&uart, "roll_desired: %f ", roll_desired);
  //   uart_printf(&uart, "pitch_desired: %f ", pitch_desired);
  //   uart_printf(&uart, "yaw_desired: %f ", yaw_desired);
  //   uart_printf(&uart, "roll_actual: %f ", roll_actual);
  //   uart_printf(&uart, "pitch_actual: %f ", pitch_actual);
  //   uart_printf(&uart, "yaw_actual: %f ", yaw_actual);
  //   uart_printf(&uart, "\r\n");
  // }

  // Update time
  time_prev_us = time_now_us;
}
