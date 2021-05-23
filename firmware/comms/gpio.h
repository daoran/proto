#pragma once

#include <Arduino.h>

void gpio_set_pin_mode(const int pin, const int mode) {
  pinMode(pin, mode);
}

void gpio_digital_write(const int pin, const int mode) {
  digitalWrite(pin, mode);
}

int gpio_digital_read(const int pin) {
  return digitalRead(pin);
}

void gpio_analog_write(const int pin, const int mode) {
  analogWrite(pin, mode);
}

int gpio_analog_read(const int pin) {
  return analogRead(pin);
}

void delay_ms(const int ms) {
  delay(ms);
}

void delay_us(const int us) {
  delayMicroseconds(us);
}
