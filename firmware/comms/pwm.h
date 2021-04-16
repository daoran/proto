#include <Wire.h>
#include <Arduino.h>

class pwm_t {
  public:
    const uint8_t *pins = nullptr;
    const uint8_t nb_pins = 0;

    float freq;
    float range_max;
    float pwm_min = 0.001;  // s
    float pwm_max = 0.002;  // s

    pwm_t(const uint8_t *pins_,
          const uint8_t nb_pins_,
          const int res = 15,
          const float freq_ = 1000)
      : pins{pins_}, nb_pins{nb_pins_}, freq{freq_} {
      /*
        for (const auto pin : pins) {
        analogWriteFrequency(pin, freq);
        }

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
      */
    }

    void set(const uint8_t idx, const float val) {
      /*
        const float time_width = pwm_min + ((pwm_max - pwm_min) * val);
        const float duty_cycle = (1.0 / time_width) / freq;
        analogWrite(pins[idx], range_max * duty_cycle);
      */
    }
};
