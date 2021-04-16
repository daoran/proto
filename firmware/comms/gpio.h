#include <Wire.h>
#include <Arduino.h>

class gpio_t {
  public:
    gpio_t() {}

    void set_pin(const int pin, const int mode) {
      pinMode(pin, mode);
    }

    void digital_write(const int pin, const int mode) {
      digitalWrite(pin, mode);
    }

    int digital_read(const int pin) {
      return digitalRead(pin);
    }

    void analog_write(const int pin, const int mode) {
      analogWrite(pin, mode);
    }

    int analog_read(const int pin) {
      return analogRead(pin);
    }

    void delay(const int ms) {
      delay(ms);
    }

    void delay_us(const int us) {
      delayMicroseconds(us);
    }

    int pulse_in(const int pin, const int val, const int timeout_us = 1000000) {
      return pulseIn(pin, val, timeout_us);
    }

    unsigned long pulse_in_long(const int pin,
                                const int val,
                                const int timeout_us = 1000000) {
      return pulseIn(pin, val, timeout_us);
    }
};
