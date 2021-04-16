#include <Wire.h>
#include <Arduino.h>

#include "core.h"


// GLOBAL VARIABLES
auto t_prev = micros();
twi i2c;

/* uint8_t pins[4] = {0, 1, 2, 3}; */
/* uint8_t nb_pins = 4; */
/* pwm_t pwm{pins, nb_pins}; */
uart_t serial;
gpio_t gpio;

/*
  void test_tps() {
  if (fcu.tps.data_ready()) {
    float temperature = 0.0f;
    float pressure = 0.0f;
    fcu.tps.get_data(temperature, pressure);

    serial.printf("%f  ", temperature);
    serial.printf("%f\n\r", pressure);

    serial.printf("temperature: %f [deg]  ", temperature);
    serial.printf("pressure: %f [Pa]", pressure);


    auto t_now = micros();
    float t_elapsed = ((float) t_now - t_prev);

    serial.printf("time elapsed: %f [us]\n\r", t_elapsed);
  }
  }

  void test_sensors() {
  test_tps();
  test_imu();
  t_prev = micros();
  }

  void test_pwm() {
  pwm.set(0, 0.5);
  float duration = pulseIn(1, HIGH);
  //serial.printf("pwm width: %f [us]\n\r", duration);
  }
*/

void setup() {}

void loop() {
  /* serial.printf("Hello World\n"); */
  /* delay(100); */
}
