#pragma once
#include <Wire.h>

#include "../core.h"

typedef struct pwm_t {
  uint8_t *pins;
  uint8_t nb_pins;
  int res;
  float freq;
  float range_max;
  float min;
  float max;
} pwm_t ;

void pwm_setup(pwm_t *pwm,
               uint8_t *pins,
               const uint8_t nb_pins,
               const int res = 15,
               const float freq = 1000) {
  for (uint8_t i = 0; i < nb_pins; i++) {
    analogWriteFrequency(pins[i], freq);
  }

  float range_max = 0.0f;
  analogWriteResolution(res);
  switch (res) {
  case 16: range_max = 65535.0f; break;
  case 15: range_max = 32767.0f; break;
  case 14: range_max = 16383.0f; break;
  case 13: range_max = 8191.0f; break;
  case 12: range_max = 4095.0f; break;
  case 11: range_max = 2047.0f; break;
  case 10: range_max = 1023.0f; break;
  case 9:  range_max = 511.0f; break;
  case 8:  range_max = 255.0f; break;
  case 7:  range_max = 127.0f; break;
  case 6:  range_max = 63.0f; break;
  case 5:  range_max = 31.0f; break;
  case 4:  range_max = 15.0f; break;
  case 3:  range_max = 7.0f; break;
  case 2:  range_max = 3.0f; break;
  default: range_max = 255.0f; break;
  }

  pwm->pins = pins;
  pwm->nb_pins = nb_pins;
  pwm->res = res;
  pwm->freq = freq;
  pwm->range_max = range_max;
}

void pwm_set(pwm_t *pwm, const uint8_t idx, const float val) {
  const float time_width = pwm->min + ((pwm->max - pwm->min) * val);
  const float duty_cycle = (1.0 / time_width) / pwm->freq;
  analogWrite(pwm->pins[idx], pwm->range_max * duty_cycle);
}
