#pragma once
#include "firmwhare.h"

void test_uart(uart_t *uart) {
  uart_printf(uart, "Hello World!\r\n");
  delay(1000);
}

void test_imu(uart_t *uart, imu_t *imu) {
  mpu6050_get_data(imu);
  uart_printf(uart, "ax:%f ", imu.accel[0]);
  uart_printf(uart, "ay:%f ", imu.accel[1]);
  uart_printf(uart, "az:%f ", imu.accel[2]);
  uart_printf(uart, "gx:%f  ", imu.gyro[0]);
  uart_printf(uart, "gy:%f  ", imu.gyro[1]);
  uart_printf(uart, "gz:%f  ", imu.gyro[2]);
  uart_printf(uart, "\n");
  delay(10);
}

void test_bmp(uart_t *uart, bmp280_t *bmp) {
  if (bmp280_data_ready(&bmp)) {
    float temperature = 0.0f;
    float pressure = 0.0f;
    bmp280_get_data(&bmp, &temperature, &pressure);
    uart_printf(&uart, "%f\n", temperature);
    delay(10);
  }
}
