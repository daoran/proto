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

void setup() {
  // Teensy 4.0 LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Flight controller unit
  fcu_setup(&fcu);
}

void loop() {
  // test_uart();
  // test_imu();
  // test_bmp();
  // test_uds();
}
