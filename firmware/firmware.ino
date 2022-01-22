#include <Wire.h>
#include <Arduino.h>

/* #include "comms/gpio.h" */
#include "comms/uart.h"
#include "comms/i2c.h"
/* #include "comms/pwm.h" */
#include "comms/sbus.h"

#include "sensors/mpu6050.h"
#include "sensors/bmp280.h"

// GLOBAL VARIABLES
uart_t uart;
mpu6050_t imu;
bmp280_t bmp;

struct fcu_t {
  mpu6050_t imu;
  bmp280_t bmp;
};

void fcu_setup(struct fcu_t *fcu) {
  mpu6050_setup(fcu->imu);
  bmp280_setup(fcu->bmp);
}


void setup() {
  i2c_setup();
  uart_setup(&uart);

//  delay(1000);
//  i2c_print_addrs(&uart);
//  delay(1000);

//  sbus_setup();

  /* delay(1000); */
  /* mpu6050_setup(&imu); */

  bmp280_setup(&bmp);
}

void loop() {
  /* i2c_print_addrs(&uart); */
  /* delay(1000); */

//  sbus_update(&uart);
//  delay(1000);

  if (bmp280_data_ready(&bmp)) {
  float temperature = 0.0f;
  float pressure = 0.0f;
  bmp280_get_data(&bmp, &temperature, &pressure);
  uart_printf(&uart, "%f\n", 44330 * (1.0 - pow(pressure / 100 / 1013.25, 0.1903)));
  //uart_printf(&uart, "%f\n", temperature);
  delay(10);
  }

  // mpu6050_get_data(&imu);
  // uart_printf(&uart, "%f ", imu.accel[0]);
  // uart_printf(&uart, "%f ", imu.accel[1]);
  // uart_printf(&uart, "%f\n", imu.accel[2]);
  /* uart_printf(&uart, "gx: %f  ", imu.gyro[0]); */
  /* uart_printf(&uart, "gy: %f  ", imu.gyro[1]); */
  /* uart_printf(&uart, "gz: %f  ", imu.gyro[2]); */
  // uart_printf(&uart, "\n");

  /* delay(1000); */
}
