#include "firmware.h"

// GLOBAL VARIABLES
fcu_t fcu;

// TESTS /////////////////////////////////////////////////////////////////////

void test_uart() {
  uart_printf(&fcu.uart, "Hello World!\r\n");
  delay(1000);
}

void test_imu() {
  mpu6050_get_data(&fcu.imu);
  uart_printf(&fcu.uart, "ax:%f ", fcu.imu.accel[0]);
  uart_printf(&fcu.uart, "ay:%f ", fcu.imu.accel[1]);
  uart_printf(&fcu.uart, "az:%f ", fcu.imu.accel[2]);
  uart_printf(&fcu.uart, "gx:%f  ", fcu.imu.gyro[0]);
  uart_printf(&fcu.uart, "gy:%f  ", fcu.imu.gyro[1]);
  uart_printf(&fcu.uart, "gz:%f  ", fcu.imu.gyro[2]);
  uart_printf(&fcu.uart, "\n");
  delay(10);
}

void test_bmp() {
  if (bmp280_data_ready(&fcu.bmp)) {
    float temperature = 0.0f;
    float pressure = 0.0f;
    bmp280_get_data(&fcu.bmp, &temperature, &pressure);
    uart_printf(&fcu.uart, "%f\n", temperature);
    delay(10);
  }
}

void test_uds() {
  const float temp = 20.0;
  const float dist = hcsr04_measure(&fcu.uds, temp);
  uart_printf(&fcu.uart, "dist:%f\n", dist);
  delay(100);
}

//////////////////////////////////////////////////////////////////////////////

uint16_t ch[16] = {0};
uint8_t frame_lost = 0;
uint8_t failsafe_activated = 0;
uart_t uart;

uint8_t pwm_pins[4] = {2, 3, 4, 5};
uint8_t nb_pwm_pins = 4;
pwm_t pwm;

mpu6050_t imu;

void setup() {
  // Teensy 4.0 LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Comms
  /* pwm_setup(&pwm, pwm_pins, nb_pwm_pins); */
  uart_setup(&uart);
  i2c_setup();
  sbus_setup();
  delay(1000);

  // Sensors
  mpu6050_setup(&imu);
}

void loop() {
  mpu6050_get_data(&imu);
  /* uart_printf(&uart, "ax:%f ", imu.accel[0]); */
  /* uart_printf(&uart, "ay:%f ", imu.accel[1]); */
  /* uart_printf(&uart, "az:%f ", imu.accel[2]); */
  /* uart_printf(&uart, "gx:%f  ", imu.gyro[0]); */
  /* uart_printf(&uart, "gy:%f  ", imu.gyro[1]); */
  /* uart_printf(&uart, "gz:%f  ", imu.gyro[2]); */
  /* uart_printf(&uart, "\n"); */
  float phi = 0.0;
  float theta = 0.0;
  compute_tilt(imu.accel, &phi, &theta);
  uart_printf(&uart, "roll:%f  ", rad2deg(phi));
  uart_printf(&uart, "pitch:%f  ", rad2deg(theta));
  uart_printf(&uart, "\n");
  delay(10);

  /* if (sbus_update(ch, &frame_lost, &failsafe_activated) == 0) { */
  /*   uart_printf(&uart, "%d ", ch[0]); */
  /*   uart_printf(&uart, "%d ", ch[1]); */
  /*   uart_printf(&uart, "%d ", ch[2]); */
  /*   uart_printf(&uart, "%d ", ch[3]); */
  /*   uart_printf(&uart, "%d ", ch[4]); */
  /*   uart_printf(&uart, "%d ", ch[5]); */
  /*   uart_printf(&uart, "%d ", ch[6]); */
  /*   uart_printf(&uart, "%d ", ch[7]); */
  /*   uart_printf(&uart, "%d ", frame_lost); */
  /*   uart_printf(&uart, "%d ", failsafe_activated); */
  /*   uart_printf(&uart, "\r\n"); */

  /*   const float throttle = (ch[0] - 180.0) / (1800.0 - 180.0); */
  /*   pwm_set(&pwm, 0, throttle); */
  /*   pwm_set(&pwm, 1, throttle); */
  /*   pwm_set(&pwm, 2, throttle); */
  /*   pwm_set(&pwm, 3, throttle); */

  /*   delay(10); */
  /* } */

  // test_uart();
  // test_imu();
  // test_bmp();
  // test_uds();
}
