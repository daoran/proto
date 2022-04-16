#include <Wire.h>
#include <Arduino.h>

#include "core.h"
#include "comms.h"
#include "controls.h"
#include "sensors.h"

// FCU ///////////////////////////////////////////////////////////////////////

// CONFIG
#define HCSR04_PIN_TRIGGER 20
#define HCSR04_PIN_ECHO 21
#define HCSR04_MAX_DIST_CM 400
#define HCSR04_MAX_TIMEOUT_MS 0

typedef struct fcu_t {
  // Comms
  uart_t uart;

  // Sensors
  mpu6050_t imu;
  bmp280_t bmp;
  hcsr04_t uds;
} fcu_t;

void fcu_setup(struct fcu_t *fcu) {
  // Comms
  uart_setup(&fcu->uart);
  i2c_setup();
  //  sbus_setup();

  // Sensors
  mpu6050_setup(&fcu->imu);
  bmp280_setup(&fcu->bmp);
  hcsr04_setup(&fcu->uds,
               HCSR04_PIN_TRIGGER,
               HCSR04_PIN_ECHO,
               HCSR04_MAX_DIST_CM,
               HCSR04_MAX_TIMEOUT_MS);
}

//////////////////////////////////////////////////////////////////////////////

// GLOBAL VARIABLES
fcu_t fcu;

void setup() {
  // Teensy 4.0 LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Flight controller unit
  fcu_setup(&fcu);
}

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

void loop() {
  // i2c_print_addrs(&fcu.uart);
  // delay(1000);

  // test_uart();
  // test_imu();
  // test_bmp();
  // test_uds();

  //  sbus_update(&uart);
  //  delay(1000);
}
