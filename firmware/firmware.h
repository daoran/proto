#pragma once
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <Wire.h>
#include <Arduino.h>

#include "config.h"

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

inline float deg2rad(const float d) {
  return d * (M_PI / 180.0);
}

inline float rad2deg(const float r) {
  return r * (180.0 / M_PI);
}

inline uint8_t within(const float value, const float lower, const float upper) {
  if (lower <= value && value <= upper) {
    return 1;
  }
  return 0;
}

inline float us2sec(const uint32_t us) {
  return us * 1e-6;
}

inline int8_t int8(const uint8_t *data, const size_t offset) {
  return (int8_t)(data[offset]);
}

inline uint8_t uint8(const uint8_t *data, const size_t offset) {
  return (uint8_t)(data[offset]);
}

inline int16_t int16(const uint8_t *data, const size_t offset) {
  return (int16_t)((data[offset + 1] << 8) | (data[offset]));
}

inline uint16_t uint16(const uint8_t *data, const size_t offset) {
  return (uint16_t)((data[offset + 1] << 8) | (data[offset]));
}

inline int32_t int32(const uint8_t *data, const size_t offset) {
  return (int32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                   (data[offset + 1] << 8) | (data[offset]));
}

inline uint32_t uint32(const uint8_t *data, const size_t offset) {
  return (uint32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                    (data[offset + 1] << 8) | (data[offset]));
}

float inv_sqrt(const float x) {
  // Source: https://en.wikipedia.org/wiki/Fast_inverse_square_root
  union {
    float f;
    uint32_t i;
  } conv = {.f = x};
  conv.i = 0x5f3759df - (conv.i >> 1);
  conv.f *= 1.5F - (x * 0.5F * conv.f * conv.f);
  return conv.f;
}

void euler2quat(const float rpy[3], float q[4]) {
  const float phi = rpy[0];
  const float theta = rpy[1];
  const float psi = rpy[2];

  const float cphi = cos(phi / 2.0);
  const float ctheta = cos(theta / 2.0);
  const float cpsi = cos(psi / 2.0);
  const float sphi = sin(phi / 2.0);
  const float stheta = sin(theta / 2.0);
  const float spsi = sin(psi / 2.0);

  const float qx = sphi * ctheta * cpsi - cphi * stheta * spsi;
  const float qy = cphi * stheta * cpsi + sphi * ctheta * spsi;
  const float qz = cphi * ctheta * spsi - sphi * stheta * cpsi;
  const float qw = cphi * ctheta * cpsi + sphi * stheta * spsi;
  const float N = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  q[0] = qw / N;
  q[1] = qx / N;
  q[2] = qy / N;
  q[3] = qz / N;
}

void euler321(const float euler[3], float C[3 * 3]) {
  assert(euler != NULL);
  assert(C != NULL);

  const float phi = euler[0];
  const float theta = euler[1];
  const float psi = euler[2];

  const float cphi = cos(phi);
  const float sphi = sin(phi);
  const float cpsi = cos(psi);
  const float spsi = sin(psi);
  const float ctheta = cos(theta);
  const float stheta = sin(theta);

  /* 1st row */
  C[0] = cpsi * ctheta;
  C[1] = cpsi * stheta * sphi - spsi * cphi;
  C[2] = cpsi * stheta * cphi + spsi * sphi;

  /* 2nd row */
  C[3] = spsi * ctheta;
  C[4] = spsi * stheta * sphi + cpsi * cphi;
  C[5] = spsi * stheta * cphi - cpsi * sphi;

  /* 3rd row */
  C[6] = -stheta;
  C[7] = ctheta * sphi;
  C[8] = ctheta * cphi;
}

void quat2euler(const float q[4], float euler[3]) {
  assert(q != NULL);
  assert(euler != NULL);

  const float qw = q[0];
  const float qx = q[1];
  const float qy = q[2];
  const float qz = q[3];

  const float qw2 = qw * qw;
  const float qx2 = qx * qx;
  const float qy2 = qy * qy;
  const float qz2 = qz * qz;

  const float t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const float t2 = asin(2 * (qy * qw - qx * qz));
  const float t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  euler[0] = t1;
  euler[1] = t2;
  euler[2] = t3;
}

void rot2quat(const float C[3 * 3], float q[4]) {
  assert(C != NULL);
  assert(q != NULL);

  const float C00 = C[0];
  const float C01 = C[1];
  const float C02 = C[2];
  const float C10 = C[3];
  const float C11 = C[4];
  const float C12 = C[5];
  const float C20 = C[6];
  const float C21 = C[7];
  const float C22 = C[8];

  const float tr = C00 + C11 + C22;
  float S = 0.0f;
  float qw = 0.0f;
  float qx = 0.0f;
  float qy = 0.0f;
  float qz = 0.0f;

  if (tr > 0) {
    S = sqrt(tr + 1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (C21 - C12) / S;
    qy = (C02 - C20) / S;
    qz = (C10 - C01) / S;
  } else if ((C00 > C11) && (C[0] > C22)) {
    S = sqrt(1.0 + C[0] - C11 - C22) * 2; // S=4*qx
    qw = (C21 - C12) / S;
    qx = 0.25 * S;
    qy = (C01 + C10) / S;
    qz = (C02 + C20) / S;
  } else if (C11 > C22) {
    S = sqrt(1.0 + C11 - C[0] - C22) * 2; // S=4*qy
    qw = (C02 - C20) / S;
    qx = (C01 + C10) / S;
    qy = 0.25 * S;
    qz = (C12 + C21) / S;
  } else {
    S = sqrt(1.0 + C22 - C[0] - C11) * 2; // S=4*qz
    qw = (C10 - C01) / S;
    qx = (C02 + C20) / S;
    qy = (C12 + C21) / S;
    qz = 0.25 * S;
  }

  q[0] = qw;
  q[1] = qx;
  q[2] = qy;
  q[3] = qz;
}

float quat_norm(const float q[4]) {
  const float qw2 = q[0] * q[0];
  const float qx2 = q[1] * q[1];
  const float qy2 = q[2] * q[2];
  const float qz2 = q[3] * q[3];
  return sqrt(qw2 + qx2 + qy2 + qz2);
}

void quat_normalize(float q[4]) {
  const float qw2 = q[0] * q[0];
  const float qx2 = q[1] * q[1];
  const float qy2 = q[2] * q[2];
  const float qz2 = q[3] * q[3];
  const float inv_norm = inv_sqrt(qw2 + qx2 + qy2 + qz2);
  q[0] *= inv_norm;
  q[1] *= inv_norm;
  q[2] *= inv_norm;
  q[3] *= inv_norm;
}

void tf_trans(const float T[16], float r[3]) {
  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

void tf_rot(const float T[16], float C[9]) {
  C[0] = T[0];
  C[1] = T[1];
  C[2] = T[2];
  C[3] = T[3];
  C[4] = T[4];
  C[5] = T[5];
  C[6] = T[6];
  C[7] = T[7];
  C[8] = T[8];
}

void tf_quat(const float T[4 * 4], float q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  float C[3 * 3] = {0};
  tf_rot(T, C);
  rot2quat(C, q);
}

// void __assert(const char *__func,
//               const char *__file,
//               int __lineno, const char *__sexp) {
//   // transmit diagnostic informations through serial link.
//   Serial.println(__func);
//   Serial.println(__file);
//   Serial.println(__lineno, DEC);
//   Serial.println(__sexp);
//   Serial.flush();
//   // abort program execution.
//   abort();
// }

// GPIO /////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////////

// UART //////////////////////////////////////////////////////////////////////

typedef struct uart_t {
  Stream *serial_in;
  Print *serial_out;
} uart_t;

void uart_setup(uart_t *uart, int baud = 115200) {
  uart->serial_in = &Serial;
  uart->serial_out = &Serial;
  Serial1.begin(baud);
}

uint8_t uart_in_ready(const uart_t *uart) {
  return uart->serial_in->available();
}

void uart_read_byte(const uart_t *uart, uint8_t *b) {
  *b = uart->serial_in->read();
}

void uart_read_bytes(const uart_t *uart, uint8_t *data, const size_t len) {
  uart->serial_in->readBytes(data, len);
}

void uart_write_byte(const uart_t *uart, const uint8_t b) {
  uart->serial_out->write(b);
}

void uart_write_bytes(const uart_t *uart,
                      const uint8_t *data,
                      const size_t len) {
  uart->serial_out->write(data, len);
}

/*
  Simple printf for writing to an Arduino serial port.  Allows specifying
  Serial..Serial3.

  const HardwareSerial&, the serial port to use (Serial..Serial3)
  const char* fmt, the formatting string followed by the data to be formatted

      int d = 65;
      float f = 123.4567;
      char* str = "Hello";
      uart_t::printf(Serial, "<fmt>", d);

  Example:
    uart_t::printf(Serial, "Sensor %d is %o and reads %1f\n", d, d, f) will
    output "Sensor 65 is on and reads 123.5" to the serial port.

  Formatting strings <fmt>
  %B    - binary (d = 0b1000001)
  %b    - binary (d = 1000001)
  %c    - character (s = H)
  %d/%i - integer (d = 65)\
  %f    - float (f = 123.45)
  %3f   - float (f = 123.346) three decimal places specified by %3.
  %s    - char* string (s = Hello)
  %X    - hexidecimal (d = 0x41)
  %x    - hexidecimal (d = 41)
  %%    - escaped percent ("%")
*/
void uart_printf(const uart_t *uart, const char *fmt, ...) {
  va_list argv;
  va_start(argv, fmt);

  for (int i = 0; fmt[i] != '\0'; i++) {
    if (fmt[i] == '%') {
      // Look for specification of number of decimal places
      int places = 4;
      if (fmt[i + 1] >= '0' && fmt[i + 1] <= '9') {
        places = fmt[i + 1] - '0';
        i++;
      }

      switch (fmt[++i]) {
        case 'B':
          uart->serial_out->print("0b");
        case 'b':
          uart->serial_out->print(va_arg(argv, int), BIN);
          break;
        case 'c':
          uart->serial_out->print((char) va_arg(argv, int));
          break;
        case 'd':
        case 'i':
          uart->serial_out->print(va_arg(argv, int), DEC);
          break;
        case 'f':
          uart->serial_out->print(va_arg(argv, double), places);
          break;
        case 'l':
          uart->serial_out->print(va_arg(argv, long), DEC);
          break;
        case 's':
          uart->serial_out->print(va_arg(argv, const char *));
          break;
        case 'X':
          uart->serial_out->print("0x");
          uart->serial_out->print(va_arg(argv, int), HEX);
          break;
        case '%':
          uart->serial_out->print(fmt[i]);
          break;
        default:
          uart->serial_out->print("?");
          break;
      }
    } else {
      uart->serial_out->print(fmt[i]);
    }
  }
  va_end(argv);
}

//////////////////////////////////////////////////////////////////////////////

// I2C ///////////////////////////////////////////////////////////////////////

void i2c_setup() {
  // Wire.setClock(400000);
  Wire.setClock(3400000);
  Wire.begin();
}

uint8_t i2c_read_byte(const uint8_t dev_addr, const uint8_t reg_addr) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();

  uint8_t size = 1;
  uint8_t last = 1;
  Wire.requestFrom(dev_addr, size, last);
  const uint8_t data = Wire.read();

  return data;
}

void i2c_read_bytes(const uint8_t dev_addr,
                    const uint8_t reg_addr,
                    const size_t length,
                    uint8_t *data) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();

  Wire.requestFrom(dev_addr, length);
  for (size_t i = 0; i < length; i++) {
    data[i] = Wire.read();
  }
}

uint8_t i2c_read_u8(const uint8_t dev_addr, const uint8_t reg_addr) {
  return i2c_read_byte(dev_addr, reg_addr);
}

int8_t i2c_read_s8(const uint8_t dev_addr, const uint8_t reg_addr) {
  return i2c_read_byte(dev_addr, reg_addr);
}

uint16_t i2c_read_u16(const uint8_t dev_addr, const uint8_t reg_addr) {
  uint8_t bytes[2] = {0};
  i2c_read_bytes(dev_addr, reg_addr, 2, bytes);
  return uint16(bytes, 0);
}

int16_t i2c_read_s16(const uint8_t dev_addr, const uint8_t reg_addr) {
  return i2c_read_u16(dev_addr, reg_addr);
}

uint32_t i2c_read_u32(const uint8_t dev_addr, const uint8_t reg_addr) {
  uint8_t bytes[4] = {0};
  i2c_read_bytes(dev_addr, reg_addr, 4, bytes);
  return uint32(bytes, 0);
}

int32_t i2c_read_s32(const uint8_t dev_addr, const uint8_t reg_addr) {
  return i2c_read_u32(dev_addr, reg_addr);
}

void i2c_write_byte(const uint8_t dev_addr,
                    const uint8_t reg_addr,
                    const uint8_t value) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(value);
  Wire.endTransmission();
}

void i2c_write_bytes(const uint8_t dev_addr,
                     const uint8_t reg_addr,
                     const uint8_t *data,
                     const size_t length) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(data, length);
  Wire.endTransmission();
}

void i2c_scan_addrs(uint8_t *addrs, uint8_t *nb_addrs) {
  *nb_addrs = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      addrs[*nb_addrs] = addr;
      *nb_addrs += 1;
    }
  }
}

void i2c_print_addrs(uart_t *uart) {
  uint8_t addrs[128] = {0};
  uint8_t nb_addrs = 0;
  i2c_scan_addrs(addrs, &nb_addrs);

  uart_printf(uart, "Scan I2C devices\n");
  uart_printf(uart, "----------------\n", nb_addrs);
  uart_printf(uart, "nb_devices: %d\n", nb_addrs);
  for (uint8_t i = 0; i < nb_addrs; i++) {
    uart_printf(uart, "[%d]: %X\n", i, addrs[i]);
  }
  uart_printf(uart, "\n");
}

////////////////////////////////////////////////////////////////////////////////

//// PWM ///////////////////////////////////////////////////////////////////////

typedef struct pwm_t {
  float value[PWM_NUM_PINS];
} pwm_t;

void pwm_setup(pwm_t *pwm) {
  // Setup PWM Resolution
  analogWriteResolution(PWM_RESOLUTION_BITS);

  // Set PWM Frequency
  analogWriteFrequency(PWM_PIN_0, PWM_FREQUENCY_HZ);
  analogWriteFrequency(PWM_PIN_1, PWM_FREQUENCY_HZ);
  analogWriteFrequency(PWM_PIN_2, PWM_FREQUENCY_HZ);
  analogWriteFrequency(PWM_PIN_3, PWM_FREQUENCY_HZ);

  // Initialize PWMs
  analogWrite(PWM_PIN_0, PWM_PERIOD_MIN * PWM_RANGE_MAX);
  analogWrite(PWM_PIN_1, PWM_PERIOD_MIN * PWM_RANGE_MAX);
  analogWrite(PWM_PIN_2, PWM_PERIOD_MIN * PWM_RANGE_MAX);
  analogWrite(PWM_PIN_3, PWM_PERIOD_MIN * PWM_RANGE_MAX);
  pwm->value[0] = 0.0;
  pwm->value[1] = 0.0;
  pwm->value[2] = 0.0;
  pwm->value[3] = 0.0;
}

void pwm_set(pwm_t *pwm, const uint8_t pin_idx, const float val) {
  const uint8_t pins[4] = {PWM_PIN_0, PWM_PIN_1, PWM_PIN_2, PWM_PIN_3};
  const float duty_cycle = PWM_PERIOD_MIN + (PWM_PERIOD_DIFF * val);
  const float analog_value = duty_cycle * PWM_RANGE_MAX;
  analogWrite(pins[pin_idx], analog_value);
  pwm->value[pin_idx] = val;
}

//////////////////////////////////////////////////////////////////////////////

// SBUS //////////////////////////////////////////////////////////////////////

typedef struct sbus_t {
  uint16_t ch[16];
  uint8_t frame_lost;
  uint8_t failsafe_activated;
} sbus_t;

void sbus_setup(sbus_t *sbus) {
  // The SBUS protocol uses inverted serial logic with:
  // - Baud rate of 100000
  // - 8 data bits
  // - Even parity bit
  // - 2 stop bits
  HardwareSerial *bus = &Serial2;
  bus->begin(100000, SERIAL_8E2);
  bus->flush();

  for (uint8_t i = 0; i < 16; i++) {
    sbus->ch[i] = 0;
  }
  sbus->frame_lost = 0;
  sbus->failsafe_activated = 0;
}

int8_t sbus_read_byte(uint8_t *data) {
  if (Serial2.available()) {
    *data = Serial2.read();
    return 0;
  } else {
    return -1;
  }
}

void sbus_read(uint8_t *data, const size_t size) {
  Serial2.readBytes(data, size);
}

void sbus_write_byte(const uint8_t data) {
  Serial2.write(data);
}

void sbus_write(const uint8_t *data, const size_t size) {
  Serial2.write(data, size);
}

int8_t sbus_update(sbus_t *sbus) {
  // Get sbus frame data
  uint8_t frame[25] = {0};
  // if (sbus_read_byte(&frame[0]) == -1 || frame[0] != 0x0F) {
  //   Serial2.clear();
  //   return -1;
  // }
  // sbus_read(frame, 25);
  // Serial2.clear();

  uint8_t new_frame = 0;
  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    if (byte != 0x0f) {
      continue;
    } else {
      Serial2.readBytes(&frame[1], 24);
      Serial2.clear();
      new_frame = 1;
      break;
    }
  }
  if (new_frame == 0) {
    return -1;
  }

  // Parse sbus frame
  // -- Parse flag
  sbus->frame_lost = (frame[23] & (1 << 5));
  sbus->failsafe_activated = (frame[23] & (1 << 4));
  // -- Parse channel data
  for (uint8_t i = 0; i < 16; i++) {
    sbus->ch[i] = 0;
  }
  sbus->ch[0] = ((frame[1] | frame[2] << 8) & 0x07FF);
  sbus->ch[1] = ((frame[2] >> 3 | frame[3] << 5) & 0x07FF);
  sbus->ch[2] = ((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF);
  sbus->ch[3] = ((frame[5] >> 1 | frame[6] << 7) & 0x07FF);
  sbus->ch[4] = ((frame[6] >> 4 | frame[7] << 4) & 0x07FF);
  sbus->ch[5] = ((frame[7] >> 7 | frame[8] << 1 | frame[8] << 9) & 0x07FF);
  sbus->ch[6] = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF);
  sbus->ch[7] = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF);
  sbus->ch[8] = ((frame[12] | frame[13] << 8) & 0x07FF);
  sbus->ch[9] = ((frame[13] >> 3 | frame[14] << 5) & 0x07FF);
  sbus->ch[10] = ((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF);
  sbus->ch[11] = ((frame[16] >> 1 | frame[17] << 7) & 0x07FF);
  sbus->ch[12] = ((frame[17] >> 4 | frame[18] << 4) & 0x07FF);
  sbus->ch[13] = ((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF);
  sbus->ch[14] = ((frame[20] >> 2 | frame[21] << 6) & 0x07FF);
  sbus->ch[15] = ((frame[21] >> 5 | frame[22] << 3) & 0x07FF);

  return 0;
}

float sbus_thrust(const sbus_t *sbus) {
  return (sbus->ch[0] - PWM_VALUE_MIN) / (PWM_VALUE_MAX - PWM_VALUE_MIN);
}

float sbus_roll(const sbus_t *sbus) {
  // Within deadband
  if (sbus->ch[1] > 950 && sbus->ch[1] < 1050) {
    return 0.0;
  }

  // Outside deadband
  float max_tilt = deg2rad(60.0);
  float val = (sbus->ch[1] - PWM_VALUE_MIN) / (PWM_VALUE_MAX - PWM_VALUE_MIN);
  return max_tilt * val;
}

float sbus_pitch(const sbus_t *sbus) {
  // Within deadband
  if (sbus->ch[2] > 950 && sbus->ch[2] < 1050) {
    return 0.0;
  }

  // Outside deadband
  float max_tilt = deg2rad(60.0);
  float val = (sbus->ch[2] - PWM_VALUE_MIN) / (PWM_VALUE_MAX - PWM_VALUE_MIN);
  return max_tilt * val;
}

float sbus_yaw(const sbus_t *sbus) {
  // Within deadband
  if (sbus->ch[3] > 950 && sbus->ch[3] < 1050) {
    return 0.0;
  }

  // Outside deadband
  float max_tilt = deg2rad(60.0);
  float val = (sbus->ch[3] - PWM_VALUE_MIN) / (PWM_VALUE_MAX - PWM_VALUE_MIN);
  return max_tilt * val;
}

uint8_t sbus_arm(const sbus_t *sbus) {
  return (sbus->ch[4] > ((PWM_VALUE_MAX - PWM_VALUE_MIN) / 2.0));
}

//////////////////////////////////////////////////////////////////////////////

// HCSR04 ////////////////////////////////////////////////////////////////////

typedef struct hcsr04_t {
  uint8_t pin_trigger;
  uint8_t pin_echo;
  unsigned short max_dist_cm;
  unsigned long max_timeout_ms;
} hcsr04_t;

void hcsr04_setup(hcsr04_t *sensor,
                  uint8_t pin_trigger,
                  uint8_t pin_echo,
                  unsigned short max_dist_cm,
                  unsigned long max_timeout_ms) {
  sensor->pin_trigger = pin_trigger;
  sensor->pin_echo = pin_echo;
  sensor->max_dist_cm = max_dist_cm;
  sensor->max_timeout_ms = max_timeout_ms;

  pinMode(pin_trigger, OUTPUT);
  pinMode(pin_echo, INPUT);
}

float hcsr04_measure(hcsr04_t *sensor, float temperature) {
  // Ensure trigger pin is LOW.
  digitalWrite(sensor->pin_trigger, LOW);
  delayMicroseconds(2);

  // Hold trigger for 10ms (signal for sensor to measure distance)
  digitalWrite(sensor->pin_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor->pin_trigger, LOW);

  // Speed of sound [cm / ms]
  const float sos = 0.03313 + 0.0000606 * temperature;
  // Cair ≈ (331.3 + 0.606 * theta) m/s

  // Compute max delay based on max distance with 25% margin in ms
  unsigned long max_dist = 2.5 * sensor->max_dist_cm / sos;
  if (sensor->max_timeout_ms > 0) {
    max_dist = min(max_dist, sensor->max_timeout_ms);
  }

  // Measure the length of echo signal, which is equal to the time needed for
  // sound to go there and back. Cannot measure beyond max distance.
  unsigned long dt_ms = pulseIn(sensor->pin_echo, HIGH, max_dist);

  // Return distance in cm
  float dist_cm = dt_ms / 2.0 * sos;
  if (dist_cm == 0 || dist_cm > sensor->max_dist_cm) {
    return -1.0;
  } else {
    return dist_cm;
  }
}

// BMP280 ////////////////////////////////////////////////////////////////////

// BMP280 Addresses
#define BMP280_ADDR 0x77     // Primary I2C Address
#define BMP280_ADDR_ALT 0x76 // Alternate Address

// BMP280 Registers
#define BMP280_REG_DIG_T1 0x88
#define BMP280_REG_DIG_T2 0x8A
#define BMP280_REG_DIG_T3 0x8C
#define BMP280_REG_DIG_P1 0x8E
#define BMP280_REG_DIG_P2 0x90
#define BMP280_REG_DIG_P3 0x92
#define BMP280_REG_DIG_P4 0x94
#define BMP280_REG_DIG_P5 0x96
#define BMP280_REG_DIG_P6 0x98
#define BMP280_REG_DIG_P7 0x9A
#define BMP280_REG_DIG_P8 0x9C
#define BMP280_REG_DIG_P9 0x9E
#define BMP280_REG_DIG_H1 0xA1
#define BMP280_REG_DIG_H2 0xE1
#define BMP280_REG_DIG_H3 0xE3
#define BMP280_REG_DIG_H4 0xE4
#define BMP280_REG_DIG_H5 0xE5
#define BMP280_REG_DIG_H6 0xE7
#define BMP280_REG_CHIPID 0xD0
#define BMP280_REG_VERSION 0xD1
#define BMP280_REG_SOFTRESET 0xE0
#define BMP280_REG_CAL26 0xE1 // R calibration stored in 0xE1-0xF0
#define BMP280_REG_CONTROLHUMID 0xF2
#define BMP280_REG_STATUS 0XF3
#define BMP280_REG_CONTROL 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESSUREDATA 0xF7
#define BMP280_REG_TEMPDATA 0xFA
#define BMP280_REG_HUMIDDATA 0xFD

// BMP280 Sampling rates
#define BMP280_SAMPLING_NONE 0b000
#define BMP280_SAMPLING_X1 0b001
#define BMP280_SAMPLING_X2 0b010
#define BMP280_SAMPLING_X4 0b011
#define BMP280_SAMPLING_X8 0b100
#define BMP280_SAMPLING_X16 0b101

// BMP280 Power modes
#define BMP280_MODE_SLEEP 0b00
#define BMP280_MODE_FORCED 0b01
#define BMP280_MODE_NORMAL 0b11

// BMP280 Filter values
#define BMP280_FILTER_OFF 0b000
#define BMP280_FILTER_X2 0b001
#define BMP280_FILTER_X4 0b010
#define BMP280_FILTER_X8 0b011
#define BMP280_FILTER_X16 0b10

// BMP280 Standby duration in ms
#define BMP280_STANDBY_MS_0_5 0b000
#define BMP280_STANDBY_MS_62_5 0b001
#define BMP280_STANDBY_MS_125 0b010
#define BMP280_STANDBY_MS_250 0b011
#define BMP280_STANDBY_MS_500 0b100
#define BMP280_STANDBY_MS_1000 0b101
#define BMP280_STANDBY_MS_2000 0b110
#define BMP280_STANDBY_MS_4000 0b111

typedef struct bmp280_t {
  // Temperature compensation value
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;

  // Pressure compenstation value
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;

  // State
  uint8_t previous_measuring;
} bmp280_t;

void bmp280_setup(bmp280_t *bmp280);
void bmp280_get_calib_data(bmp280_t *bmp280);
int bmp280_data_ready(bmp280_t *bmp280);
void bmp280_get_data(const bmp280_t *bmp280,
                     float *temperature,
                     float *pressure);

void bmp280_setup(bmp280_t *bmp280) {
  uint8_t sensor_addr = BMP280_ADDR_ALT;

  // Reset the device using soft-reset
  i2c_write_byte(sensor_addr, BMP280_REG_SOFTRESET, 0xB6);

  // Wait for chip to wake up.
  delay(10);

  // Get calibration data
  bmp280_get_calib_data(bmp280);

  // Configure sensor
  uint8_t t_sb = BMP280_STANDBY_MS_0_5; // Standby time in normal mode
  uint8_t filter = BMP280_FILTER_X16;   // Filter settings
  uint8_t spi3w_en = 0;                 // Enable 3-wire SPI
  uint8_t config = (t_sb << 5) | (filter << 2) | spi3w_en;
  i2c_write_byte(sensor_addr, BMP280_REG_CONFIG, config);

  // Configure measurement
  uint8_t mode = BMP280_MODE_NORMAL;    // Device mode
  uint8_t osrs_p = BMP280_SAMPLING_X16; // Pressure oversampling
  uint8_t osrs_t = BMP280_SAMPLING_X2;  // Temperature oversampling
  uint8_t meas_config = (osrs_t << 5) | (osrs_p << 2) | mode;
  i2c_write_byte(sensor_addr, BMP280_REG_CONTROL, meas_config);

  // Wait
  delay(100);
}

void bmp280_get_calib_data(bmp280_t *bmp280) {
  uint8_t sensor_addr = BMP280_ADDR_ALT;

  bmp280->dig_T1 = i2c_read_u16(sensor_addr, BMP280_REG_DIG_T1);
  bmp280->dig_T2 = i2c_read_s16(sensor_addr, BMP280_REG_DIG_T2);
  bmp280->dig_T3 = i2c_read_s16(sensor_addr, BMP280_REG_DIG_T3);

  bmp280->dig_P1 = i2c_read_u16(sensor_addr, BMP280_REG_DIG_P1);
  bmp280->dig_P2 = i2c_read_s16(sensor_addr, BMP280_REG_DIG_P2);
  bmp280->dig_P3 = i2c_read_s16(sensor_addr, BMP280_REG_DIG_P3);
  bmp280->dig_P4 = i2c_read_s16(sensor_addr, BMP280_REG_DIG_P4);
  bmp280->dig_P5 = i2c_read_s16(sensor_addr, BMP280_REG_DIG_P5);
  bmp280->dig_P6 = i2c_read_s16(sensor_addr, BMP280_REG_DIG_P6);
  bmp280->dig_P7 = i2c_read_s16(sensor_addr, BMP280_REG_DIG_P7);
  bmp280->dig_P8 = i2c_read_s16(sensor_addr, BMP280_REG_DIG_P8);
  bmp280->dig_P9 = i2c_read_s16(sensor_addr, BMP280_REG_DIG_P9);
}

int bmp280_data_ready(bmp280_t *bmp280) {
  uint8_t sensor_addr = BMP280_ADDR_ALT;

  const uint8_t reg = i2c_read_byte(sensor_addr, BMP280_REG_STATUS);
  const uint8_t measuring = (reg & 0b00001000) >> 3;

  // Has measuring bit flipped?
  if (measuring ^ bmp280->previous_measuring) {
    bmp280->previous_measuring = measuring;
    if (measuring == 0) {
      return 1;
    }
  }

  return 0;
}

/** Returns temperature in Celcius degrees and pressure in Pa **/
void bmp280_get_data(const bmp280_t *bmp280,
                     float *temperature,
                     float *pressure) {
  uint8_t sensor_addr = BMP280_ADDR_ALT;

  // Get data
  uint8_t data[6] = {0};
  i2c_read_bytes(sensor_addr, BMP280_REG_PRESSUREDATA, 6, data);

  // Compensate for temperature
  int32_t t_fine = 0;
  {
    int32_t T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    int32_t T1 = bmp280->dig_T1;
    int32_t T2 = bmp280->dig_T2;
    int32_t T3 = bmp280->dig_T3;
    int32_t var1 = ((((T >> 3) - (T1 << 1))) * T2) >> 11;
    int32_t var2 = (((((T >> 4) - T1) * ((T >> 4) - T1)) >> 12) * (T3)) >> 14;
    t_fine = var1 + var2;
    *temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;
  }

  // Compensate for pressure
  {
    int32_t P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int64_t P1 = bmp280->dig_P1;
    int64_t P2 = bmp280->dig_P2;
    int64_t P3 = bmp280->dig_P3;
    int64_t P4 = bmp280->dig_P4;
    int64_t P5 = bmp280->dig_P5;
    int64_t P6 = bmp280->dig_P6;
    int64_t P7 = bmp280->dig_P7;
    int64_t P8 = bmp280->dig_P8;
    int64_t P9 = bmp280->dig_P9;

    int64_t var1, var2 = 0;
    var1 = ((int64_t) t_fine) - 128000;
    var2 = var1 * var1 * P6;
    var2 = var2 + ((var1 * P5) << 17);
    var2 = var2 + (P4 << 35);
    var1 = ((var1 * var1 * P3) >> 8) + ((var1 * P2) << 12);
    var1 = (((((int64_t) 1) << 47) + var1)) * P1 >> 33;

    // Avoid exception caused by division by zero
    if (var1 == 0) {
      *pressure = -1;
      return;
    }

    int64_t p = 1048576 - P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (P8 * p) >> 19;
    *pressure = ((float) (((p + var1 + var2) >> 8) + (P7 << 4))) / 256.0f;
  }
}

//////////////////////////////////////////////////////////////////////////////

// MPU6050 ///////////////////////////////////////////////////////////////////

// GENERAL
#define MPU6050_ADDRESS 0x68
#define MPU6050_ADDRESS_AD0_LOW 0x68  // addr pin low (GND) [default]
#define MPU6050_ADDRESS_AD0_HIGH 0x69 // addr pin high (VCC)

// REGISTER ADDRESSES
#define MPU6050_XG_OFFS_TC 0x00
#define MPU6050_YG_OFFS_TC 0x01
#define MPU6050_ZG_OFFS_TC 0x02
#define MPU6050_X_FINE_GAIN 0x03
#define MPU6050_Y_FINE_GAIN 0x04
#define MPU6050_Z_FINE_GAIN 0x05
#define MPU6050_XA_OFFS_H 0x06
#define MPU6050_XA_OFFS_L_TC 0x07
#define MPU6050_YA_OFFS_H 0x08
#define MPU6050_YA_OFFS_L_TC 0x09
#define MPU6050_ZA_OFFS_H 0x0A
#define MPU6050_ZA_OFFS_L_TC 0x0B
#define MPU6050_XG_OFFS_USRH 0x13
#define MPU6050_XG_OFFS_USRL 0x14
#define MPU6050_YG_OFFS_USRH 0x15
#define MPU6050_YG_OFFS_USRL 0x16
#define MPU6050_ZG_OFFS_USRH 0x17
#define MPU6050_ZG_OFFS_USRL 0x18
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_FF_THR 0x1D
#define MPU6050_FF_DUR 0x1E
#define MPU6050_MOT_THR 0x1F
#define MPU6050_MOT_DUR 0x20
#define MPU6050_ZRMOT_THR 0x21
#define MPU6050_ZRMOT_DUR 0x22
#define MPU6050_FIFO_EN 0x23
#define MPU6050_I2C_MST_CTRL 0x24
#define MPU6050_I2C_SLV0_ADDR 0x25
#define MPU6050_I2C_SLV0_REG 0x26
#define MPU6050_I2C_SLV0_CTRL 0x27
#define MPU6050_I2C_SLV1_ADDR 0x28
#define MPU6050_I2C_SLV1_REG 0x29
#define MPU6050_I2C_SLV1_CTRL 0x2A
#define MPU6050_I2C_SLV2_ADDR 0x2B
#define MPU6050_I2C_SLV2_REG 0x2C
#define MPU6050_I2C_SLV2_CTRL 0x2D
#define MPU6050_I2C_SLV3_ADDR 0x2E
#define MPU6050_I2C_SLV3_REG 0x2F
#define MPU6050_I2C_SLV3_CTRL 0x30
#define MPU6050_I2C_SLV4_ADDR 0x31
#define MPU6050_I2C_SLV4_REG 0x32
#define MPU6050_I2C_SLV4_DO 0x33
#define MPU6050_I2C_SLV4_CTRL 0x34
#define MPU6050_I2C_SLV4_DI 0x35
#define MPU6050_I2C_MST_STATUS 0x36
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_DMP_INT_STATUS 0x39
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_EXT_SENS_DATA_00 0x49
#define MPU6050_EXT_SENS_DATA_01 0x4A
#define MPU6050_EXT_SENS_DATA_02 0x4B
#define MPU6050_EXT_SENS_DATA_03 0x4C
#define MPU6050_EXT_SENS_DATA_04 0x4D
#define MPU6050_EXT_SENS_DATA_05 0x4E
#define MPU6050_EXT_SENS_DATA_06 0x4F
#define MPU6050_EXT_SENS_DATA_07 0x50
#define MPU6050_EXT_SENS_DATA_08 0x51
#define MPU6050_EXT_SENS_DATA_09 0x52
#define MPU6050_EXT_SENS_DATA_10 0x53
#define MPU6050_EXT_SENS_DATA_11 0x54
#define MPU6050_EXT_SENS_DATA_12 0x55
#define MPU6050_EXT_SENS_DATA_13 0x56
#define MPU6050_EXT_SENS_DATA_14 0x57
#define MPU6050_EXT_SENS_DATA_15 0x58
#define MPU6050_EXT_SENS_DATA_16 0x59
#define MPU6050_EXT_SENS_DATA_17 0x5A
#define MPU6050_EXT_SENS_DATA_18 0x5B
#define MPU6050_EXT_SENS_DATA_19 0x5C
#define MPU6050_EXT_SENS_DATA_20 0x5D
#define MPU6050_EXT_SENS_DATA_21 0x5E
#define MPU6050_EXT_SENS_DATA_22 0x5F
#define MPU6050_EXT_SENS_DATA_23 0x60
#define MPU6050_MOT_DETECT_STATUS 0x61
#define MPU6050_I2C_SLV0_DO 0x63
#define MPU6050_I2C_SLV1_DO 0x64
#define MPU6050_I2C_SLV2_DO 0x65
#define MPU6050_I2C_SLV3_DO 0x66
#define MPU6050_I2C_MST_DELAY_CTRL 0x67
#define MPU6050_SIGNAL_PATH_RESET 0x68
#define MPU6050_MOT_DETECT_CTRL 0x69
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_BANK_SEL 0x6D
#define MPU6050_MEM_START_ADDR 0x6E
#define MPU6050_MEM_R_W 0x6F
#define MPU6050_DMP_CFG_1 0x70
#define MPU6050_DMP_CFG_2 0x71
#define MPU6050_FIFO_COUNTH 0x72
#define MPU6050_FIFO_COUNTL 0x73
#define MPU6050_FIFO_R_W 0x74
#define MPU6050_WHO_AM_I 0x75

typedef struct mpu6050_t {
  int8_t dplf_config;
  float sample_rate;
  float accel_sensitivity;
  float gyro_sensitivity;

  float accel[3];
  float gyro[3];
  float temperature;

  float gyro_offset[3];
} mpu6050_t;

void mpu6050_setup(mpu6050_t *imu);
void mpu6050_calibrate(mpu6050_t *imu);
void mpu6050_get_data(mpu6050_t *imu);

void mpu6050_setup(mpu6050_t *imu) {
  // Config
  imu->dplf_config = 0;
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_CONFIG, imu->dplf_config);

  // Power management
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);

  // Configure gyroscope
  const int8_t gyro_config = 1 << 3;
  imu->gyro_sensitivity = 65.5;
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, gyro_config);

  // Configure accelerometer
  const int8_t accel_config = 2 << 3;
  imu->accel_sensitivity = 4096.0;
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, accel_config);

  // Sample rate
  uint16_t gyro_rate = 0;
  if (imu->dplf_config == 0 || imu->dplf_config == 7) {
    gyro_rate = 8000;
  } else if (imu->dplf_config >= 1 || imu->dplf_config <= 6) {
    gyro_rate = 1000;
  }
  const uint8_t sample_div = i2c_read_byte(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV);
  imu->sample_rate = gyro_rate / (1 + sample_div);

  // Calibrate IMU
  mpu6050_calibrate(imu);
}

void mpu6050_calibrate(mpu6050_t *imu) {
  float wx = 0.0;
  float wy = 0.0;
  float wz = 0.0;

  uint16_t nb_samples = 5000;
  for (uint16_t i = 0; i < nb_samples; i++) {
    mpu6050_get_data(imu);
    wx += imu->gyro[0];
    wy += imu->gyro[1];
    wz += imu->gyro[2];
  }

  imu->gyro_offset[0] = wx / (float) nb_samples;
  imu->gyro_offset[1] = wy / (float) nb_samples;
  imu->gyro_offset[2] = wz / (float) nb_samples;
}

void mpu6050_get_data(mpu6050_t *imu) {
  // Read data
  uint8_t raw_data[14] = {0};
  i2c_read_bytes(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, raw_data);

  // Accelerometer
  const float g = 9.81; // Gravitational constant
  const int16_t raw_x = (raw_data[0] << 8) | (raw_data[1]);
  const int16_t raw_y = (raw_data[2] << 8) | (raw_data[3]);
  const int16_t raw_z = (raw_data[4] << 8) | (raw_data[5]);
  imu->accel[0] = (raw_x / imu->accel_sensitivity) * g;
  imu->accel[1] = (raw_y / imu->accel_sensitivity) * g;
  imu->accel[2] = (raw_z / imu->accel_sensitivity) * g;

  // Temperature
  const int8_t raw_temp = (raw_data[6] << 8) | (raw_data[7]);
  imu->temperature = raw_temp / 340.0 + 36.53;

  // Gyroscope
  const int16_t gyro_raw_x = (raw_data[8] << 8) | (raw_data[9]);
  const int16_t gyro_raw_y = (raw_data[10] << 8) | (raw_data[11]);
  const int16_t gyro_raw_z = (raw_data[12] << 8) | (raw_data[13]);
  imu->gyro[0] = deg2rad(gyro_raw_x / imu->gyro_sensitivity);
  imu->gyro[1] = deg2rad(gyro_raw_y / imu->gyro_sensitivity);
  imu->gyro[2] = deg2rad(gyro_raw_z / imu->gyro_sensitivity);
}

//////////////////////////////////////////////////////////////////////////////

//// MPU9250 ///////////////////////////////////////////////////////////////////

//// GENERAL
//#define MPU9250_ADDRESS 0x68
//#define MPU9250_ADDRESS_AD0_LOW 0x68  // addr pin low (GND) [default]
//#define MPU9250_ADDRESS_AD0_HIGH 0x69 // addr pin high (VCC)

//// REGISTER ADDRESSES
//#define MPU9250_SELF_TEST_X_GYRO 0x00
//#define MPU9250_SELF_TEST_Y_GYRO 0x01
//#define MPU9250_SELF_TEST_Z_GYRO 0x02
//#define MPU9250_SELF_TEST_X_ACCEL 0x0D
//#define MPU9250_SELF_TEST_Y_ACCEL 0x0E
//#define MPU9250_SELF_TEST_Z_ACCEL 0x0F
//#define MPU9250_XG_OFFSET_H 0x13
//#define MPU9250_XG_OFFSET_L 0x14
//#define MPU9250_YG_OFFSET_H 0x15
//#define MPU9250_YG_OFFSET_L 0x16
//#define MPU9250_ZG_OFFSET_H 0x17
//#define MPU9250_ZG_OFFSET_L 0x18
//#define MPU9250_SMPLRT_DIV 0x19
//#define MPU9250_CONFIG 0x1A
//#define MPU9250_GYRO_CONFIG 0x1B
//#define MPU9250_ACCEL_CONFIG 0x1C
//#define MPU9250_ACCEL_CONFIG2 0x1D
//#define MPU9250_LP_ACCEL_ODR 0x1E
//#define MPU9250_WOM_THR 0x1F
//#define MPU9250_FIFO_EN 0x23
//#define MPU9250_I2C_MST_CTRL 0x24
//#define MPU9250_I2C_SLV0_ADDR 0x25
//#define MPU9250_I2C_SLV0_REG 0x26
//#define MPU9250_I2C_SLV0_CTRL 0x27
//#define MPU9250_I2C_SLV1_ADDR 0x28
//#define MPU9250_I2C_SLV1_REG 0x29
//#define MPU9250_I2C_SLV1_CTRL 0x2A
//#define MPU9250_I2C_SLV2_ADDR 0x28
//#define MPU9250_I2C_SLV2_REG 0x2C
//#define MPU9250_I2C_SLV2_CTRL 0x2D
//#define MPU9250_I2C_SLV3_ADDR 0x2E
//#define MPU9250_I2C_SLV3_REG 0x2F
//#define MPU9250_I2C_SLV3_CTRL 0x30
//#define MPU9250_I2C_SLV4_ADDR 0x31
//#define MPU9250_I2C_SLV4_REG 0x32
//#define MPU9250_I2C_SLV4_DO 0x33
//#define MPU9250_I2C_SLV4_CTRL 0x34
//#define MPU9250_I2C_SLV4_DI 0x35
//#define MPU9250_I2C_MST_STATUS 0x36
//#define MPU9250_INT_PIN_CFG 0x37
//#define MPU9250_INT_ENABLE 0x38
//#define MPU9250_INT_STATUS 0x3A
//#define MPU9250_ACCEL_XOUT_H 0x3B
//#define MPU9250_ACCEL_XOUT_L 0x3C
//#define MPU9250_ACCEL_YOUT_H 0x3D
//#define MPU9250_ACCEL_YOUT_L 0x3E
//#define MPU9250_ACCEL_ZOUT_H 0x3F
//#define MPU9250_ACCEL_ZOUT_L 0x40
//#define MPU9250_TEMP_OUT_H 0x41
//#define MPU9250_TEMP_OUT_L 0x42
//#define MPU9250_GYRO_XOUT_H 0x43
//#define MPU9250_GYRO_XOUT_L 0x44
//#define MPU9250_GYRO_YOUT_H 0x45
//#define MPU9250_GYRO_YOUT_L 0x46
//#define MPU9250_GYRO_ZOUT_H 0x47
//#define MPU9250_GYRO_ZOUT_L 0x48
//#define MPU9250_EXT_SENS_DATA_00 0x49
//#define MPU9250_EXT_SENS_DATA_01 0x4A
//#define MPU9250_EXT_SENS_DATA_02 0x4B
//#define MPU9250_EXT_SENS_DATA_03 0x4C
//#define MPU9250_EXT_SENS_DATA_04 0x4D
//#define MPU9250_EXT_SENS_DATA_05 0x4E
//#define MPU9250_EXT_SENS_DATA_06 0x4F
//#define MPU9250_EXT_SENS_DATA_07 0x50
//#define MPU9250_EXT_SENS_DATA_08 0x51
//#define MPU9250_EXT_SENS_DATA_09 0x52
//#define MPU9250_EXT_SENS_DATA_10 0x53
//#define MPU9250_EXT_SENS_DATA_11 0x54
//#define MPU9250_EXT_SENS_DATA_12 0x55
//#define MPU9250_EXT_SENS_DATA_13 0x56
//#define MPU9250_EXT_SENS_DATA_14 0x57
//#define MPU9250_EXT_SENS_DATA_15 0x58
//#define MPU9250_EXT_SENS_DATA_16 0x59
//#define MPU9250_EXT_SENS_DATA_17 0x5A
//#define MPU9250_EXT_SENS_DATA_18 0x5B
//#define MPU9250_EXT_SENS_DATA_19 0x5C
//#define MPU9250_EXT_SENS_DATA_20 0x5D
//#define MPU9250_EXT_SENS_DATA_21 0x5E
//#define MPU9250_EXT_SENS_DATA_22 0x5F
//#define MPU9250_EXT_SENS_DATA_23 0x60
//#define MPU9250_I2C_SLV0_DO 0x63
//#define MPU9250_I2C_SLV1_DO 0x64
//#define MPU9250_I2C_SLV2_DO 0x65
//#define MPU9250_I2C_SLV3_DO 0x66
//#define MPU9250_I2C_MST_DELAY_CTRL 0x67
//#define MPU9250_SIGNAL_PATH_RESET 0x68
//#define MPU9250_MOT_DETECT_CTRL 0x69
//#define MPU9250_USER_CTRL 0x6A
//#define MPU9250_PWR_MGMT_1 0x6B
//#define MPU9250_PWR_MGMT_2 0x6C
//#define MPU9250_FIFO_COUNTH 0x72
//#define MPU9250_FIFO_COUNTL 0x73
//#define MPU9250_FIFO_R_W 0x74
//#define MPU9250_WHO_AM_I 0x75
//#define MPU9250_XA_OFFSET_H 0x77
//#define MPU9250_XA_OFFSET_L 0x78
//#define MPU9250_YA_OFFSET_H 0x7A
//#define MPU9250_YA_OFFSET_L 0x7B
//#define MPU9250_ZA_OFFSET_H 0x7D
//#define MPU9250_ZA_OFFSET_L 0x7E

//#define AK8963_I2C_ADDR 0x0c
//#define AK8963_Device_ID 0x48
//#define AK8963_WHO_AM_I 0x00
//#define AK8963_INFO 0x01
//#define AK8963_ST1 0x02
//#define AK8963_XOUT_L 0x03
//#define AK8963_XOUT_H 0x04
//#define AK8963_YOUT_L 0x05
//#define AK8963_YOUT_H 0x06
//#define AK8963_ZOUT_L 0x07
//#define AK8963_ZOUT_H 0x08
//#define AK8963_ST2 0x09
//#define AK8963_CNTL 0x0A
//#define AK8963_ASTC 0x0C
//#define AK8963_I2CDIS 0x0F
//#define AK8963_ASAX 0x10
//#define AK8963_ASAY 0x11
//#define AK8963_ASAZ 0x12

// typedef struct mpu9250_t {
//  float accel_sensitivity;
//  float gyro_sensitivity;
//  float mag_sensitivity;
//  float accel[3];
//  float gyro[3];
//  float gyro_offset[3];

//  float temperature;
//  float sample_rate;
//  int8_t dplf_config;
//} mpu9250_t;

// void mpu9250_setup(mpu9250_t *imu);
// void mpu9250_calibrate(mpu9250_t *imu);
// void mpu9250_get_data(mpu6050_t *imu);

// void mpu9250_setup(mpu9250_t *imu) {
//  int8_t retval = 0;
//  int8_t dplf = 6;
//  int8_t gyro_range = 1;
//  int8_t accel_range = 2;

//  uint8_t config = 0;
//  uint8_t gyro_config = 0;
//  uint8_t accel_config = 0;
//  i2c_write_byte(MPU9250_ADDRESS, MPU9250_CONFIG, config);
//  i2c_write_byte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, gyro_config);
//  i2c_write_byte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, accel_config);
//}

// void mpu9250_calibrate(mpu9250_t *imu) {
//  float wx = 0.0;
//  float wy = 0.0;
//  float wz = 0.0;

//  uint16_t nb_samples = 5000;
//  for (uint16_t i = 0; i < nb_samples; i++) {
//    mpu9250_get_data(imu);
//    wx += imu->gyro[0];
//    wy += imu->gyro[1];
//    wz += imu->gyro[2];
//  }

//  imu->gyro_offset[0] = wx / (float) nb_samples;
//  imu->gyro_offset[1] = wy / (float) nb_samples;
//  imu->gyro_offset[2] = wz / (float) nb_samples;
//}

// void mpu9250_get_data(mpu9250_t *imu) {
//  // Read data
//  uint8_t raw_data[14] = {0};
//  i2c_read_bytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 14, raw_data);

//  // Accelerometer
//  const float g = 9.81; // Gravitational constant
//  const int16_t raw_x = (raw_data[0] << 8) | (raw_data[1]);
//  const int16_t raw_y = (raw_data[2] << 8) | (raw_data[3]);
//  const int16_t raw_z = (raw_data[4] << 8) | (raw_data[5]);
//  imu->accel[0] = (raw_x / imu->accel_sensitivity) * g;
//  imu->accel[1] = (raw_y / imu->accel_sensitivity) * g;
//  imu->accel[2] = (raw_z / imu->accel_sensitivity) * g;

//  // Temperature
//  const int8_t raw_temp = (raw_data[6] << 8) | (raw_data[7]);
//  imu->temperature = raw_temp / 340.0 + 36.53;

//  // Gyroscope
//  const int16_t gyro_raw_x = (raw_data[8] << 8) | (raw_data[9]);
//  const int16_t gyro_raw_y = (raw_data[10] << 8) | (raw_data[11]);
//  const int16_t gyro_raw_z = (raw_data[12] << 8) | (raw_data[13]);
//  imu->gyro[0] = deg2rad(gyro_raw_x / imu->gyro_sensitivity);
//  imu->gyro[1] = deg2rad(gyro_raw_y / imu->gyro_sensitivity);
//  imu->gyro[2] = deg2rad(gyro_raw_z / imu->gyro_sensitivity);

//  const int16_t mag_raw_x = (raw_data[16] << 8 | raw_data[15]);
//  const int16_t mag_raw_y = (raw_data[18] << 8 | raw_data[17]);
//  const int16_t mag_raw_z = (raw_data[20] << 8 | raw_data[19]);
//  imu->mag[0] = mag_raw_x * imu->mag_scale[0];
//  imu->mag[1] = mag_raw_y * imu->mag_scale[1];
//  imu->mag[2] = mag_raw_z * imu->mag_scale[2];
//}

// void mpu9250_magnetometer_read(const mpu9250_t *imu,
//                               uint8_t reg,
//                               uint8_t count,
//                               uint8_t *data) {
//  i2c_write_byte(MPU9250_I2C_ADDR,
//                 MPU9250_I2C_SLV0_ADDR,
//                 AK8963_I2C_ADDR | 0x80);
//  i2c_write_byte(MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_REG, reg);
//  i2c_write_byte(MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_EN, count);
//  delay(1);
//  i2c_read_bytes(MPU9250_I2C_ADDR, MPU9250_EXT_SENS_DATA_00, count, data);
//}

// void mpu9250_magnetometer_write(const mpu9250_t *imu,
//                                const uint8_t reg,
//                                const uint8_t data) {
//  i2c_write_byte(MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
//  i2c_write_byte(MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_REG, reg);
//  i2c_write_byte(MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_DO, data);
//  i2c_write_byte(MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_CTRL, 0x80 |
//  sizeof(data));
//}

////////////////////////////////////////////////////////////////////////////////

// MAHONY FILTER /////////////////////////////////////////////////////////////

typedef struct mahony_filter_t {
  float q[4];

  float kp;
  float ki;

  float integralFBx;
  float integralFBy;
  float integralFBz;

  float roll;
  float pitch;
  float yaw;

} mahony_filter_t;

void mahony_filter_setup(mahony_filter_t *filter) {
  filter->q[0] = 1.0;
  filter->q[1] = 0.0;
  filter->q[2] = 0.0;
  filter->q[3] = 0.0;

  filter->kp = 2.0 * 0.5;
  filter->ki = 2.0 * 0.0;

  filter->integralFBx = 0.0;
  filter->integralFBy = 0.0;
  filter->integralFBz = 0.0;

  filter->roll = 0.0;
  filter->pitch = 0.0;
  filter->yaw = 0.0;
}

void mahony_filter_update(mahony_filter_t *filter,
                          const float a[3],
                          const float w[3],
                          const float dt) {
  // Normalise accelerometer measurement
  const float acc_inv_norm = inv_sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  const float ax = a[0] * acc_inv_norm;
  const float ay = a[1] * acc_inv_norm;
  const float az = a[2] * acc_inv_norm;

  // Estimated direction of gravity and vector perpendicular to magnetic
  // flux
  const float qw = filter->q[0];
  const float qx = filter->q[1];
  const float qy = filter->q[2];
  const float qz = filter->q[3];
  const float halfvx = qx * qz - qw * qy;
  const float halfvy = qw * qx + qy * qz;
  const float halfvz = qw * qw - 0.5f + qz * qz;

  // Error is sum of cross product between estimated and measured direction
  // of gravity
  const float halfex = (ay * halfvz - az * halfvy);
  const float halfey = (az * halfvx - ax * halfvz);
  const float halfez = (ax * halfvy - ay * halfvx);

  // Compute and apply integral feedback if enabled
  float wx = w[0];
  float wy = w[1];
  float wz = w[2];
  if (filter->ki > 0.0f) {
    // Integral error scaled by Ki
    filter->integralFBx += filter->ki * halfex * dt;
    filter->integralFBy += filter->ki * halfey * dt;
    filter->integralFBz += filter->ki * halfez * dt;

    // Apply integral feedback
    wx += filter->integralFBx;
    wy += filter->integralFBy;
    wz += filter->integralFBz;
  } else {
    // Prevent integral windup
    filter->integralFBx = 0.0f;
    filter->integralFBy = 0.0f;
    filter->integralFBz = 0.0f;
  }

  // Integrate rate of change of quaternion
  wx += filter->kp * halfex;
  wy += filter->kp * halfey;
  wz += filter->kp * halfez;
  wx *= (0.5f * dt);
  wy *= (0.5f * dt);
  wz *= (0.5f * dt);
  filter->q[0] += -qx * wx - qy * wy - qz * wz;
  filter->q[1] += qw * wx + qy * wz - qz * wy;
  filter->q[2] += qw * wy - qx * wz + qz * wx;
  filter->q[3] += qw * wz + qx * wy - qy * wx;
  quat_normalize(filter->q);

  // Quaternion to euler
  float rpy[3] = {0.0, 0.0, 0.0};
  quat2euler(filter->q, rpy);
  filter->roll = rpy[0];
  filter->pitch = rpy[1];
  filter->yaw = rpy[2];
}

void mahony_filter_reset_yaw(mahony_filter_t *filter) {
  const float rpy[3] = {filter->roll, filter->pitch, 0.0};
  filter->yaw = 0.0;
  euler2quat(rpy, filter->q);
}

//////////////////////////////////////////////////////////////////////////////

// PID CONTROLLER ////////////////////////////////////////////////////////////

typedef struct pid_ctrl_t {
  float error_prev;
  float error_sum;

  float error_p;
  float error_i;
  float error_d;

  float kp;
  float ki;
  float kd;
} pid_ctrl_t;

void pid_ctrl_setup(pid_ctrl_t *pid,
                    const float kp,
                    const float ki,
                    const float kd) {
  pid->error_prev = 0.0f;
  pid->error_sum = 0.0f;

  pid->error_p = 0.0f;
  pid->error_i = 0.0f;
  pid->error_d = 0.0f;

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

float pid_ctrl_update(pid_ctrl_t *pid, const float error, const float dt) {
  // Calculate errors
  pid->error_sum += error * dt;

  // Calculate output
  pid->error_p = pid->kp * error;
  pid->error_i = pid->ki * pid->error_sum;
  pid->error_d = pid->kd * (error - pid->error_prev) / dt;
  const float output = pid->error_p + pid->error_i + pid->error_d;

  pid->error_prev = error;
  return output;
}

void pid_ctrl_reset(pid_ctrl_t *pid) {
  pid->error_prev = 0.0;
  pid->error_sum = 0.0;

  pid->error_p = 0.0;
  pid->error_i = 0.0;
  pid->error_d = 0.0;
}

//////////////////////////////////////////////////////////////////////////////

// ATTITUDE CONTROLLER ///////////////////////////////////////////////////////

typedef struct att_ctrl_t {
  pid_ctrl_t roll_pid;
  pid_ctrl_t pitch_pid;
  pid_ctrl_t yaw_pid;
} att_ctrl_t;

void att_ctrl_setup(att_ctrl_t *att_ctrl) {
  pid_ctrl_t *roll_pid = &att_ctrl->roll_pid;
  pid_ctrl_t *pitch_pid = &att_ctrl->pitch_pid;
  pid_ctrl_t *yaw_pid = &att_ctrl->yaw_pid;
  pid_ctrl_setup(roll_pid, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD);
  pid_ctrl_setup(pitch_pid, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD);
  // pid_ctrl_setup(pitch_pid, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);
  pid_ctrl_setup(yaw_pid, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
}

void att_ctrl_update(att_ctrl_t *att_ctrl,
                     const float roll_desired,
                     const float pitch_desired,
                     const float yaw_desired,
                     const float thrust_desired,
                     const float roll_actual,
                     const float pitch_actual,
                     const float yaw_actual,
                     const float dt,
                     float outputs[4],
                     uart_t *uart) {
  // Calculate roll error
  const float roll_error = roll_desired - roll_actual;

  // Calculate pitch error
  const float pitch_error = pitch_desired - pitch_actual;

  // Calculate yaw error
  float yaw_error = yaw_desired - yaw_actual;
  if (yaw_error > M_PI) {
    yaw_error -= M_PI;
  } else if (yaw_error < -M_PI) {
    yaw_error += M_PI;
  }
  // yaw_error += (yaw_error < -M_PI) ? +M_PI : 0.0;

  // PID controller on roll, pitch and yaw
  const float r = pid_ctrl_update(&att_ctrl->roll_pid, roll_error, dt);
  const float p = pid_ctrl_update(&att_ctrl->pitch_pid, pitch_error, dt);
  const float y = pid_ctrl_update(&att_ctrl->yaw_pid, yaw_error, dt);

  // Map PIDs to motor outputs:
  //
  //             x
  //    3     1  ^
  //     \___/   |
  //     |___|   |
  //     /   \   |
  //    2     0  |
  // y <---------o
  //
  outputs[0] = thrust_desired - r + p - y;
  outputs[1] = thrust_desired - r - p + y;
  outputs[2] = thrust_desired + r + p + y;
  outputs[3] = thrust_desired + r - p - y;

  if (uart) {
    // char dt_str[10]; //  Hold The Convert Data
    // dtostrf(dt, 10, 10, dt_str);
    // uart_printf(uart, "dt:%s ", dt_str);
    //
    // uart_printf(uart, "Min:-1.0 ");
    // uart_printf(uart, "roll_actual:%f ", roll_actual);
    // uart_printf(uart, "roll_desired:%f ", roll_desired);
    // uart_printf(uart, "r:%f ", r);
    // uart_printf(uart, "t:%f ", thrust_desired);
    // uart_printf(uart, "Max:1.0 ");
    // // uart_printf(uart, "pitch_error:%f ", pitch_error);
    // uart_printf(uart, "yaw_error:%f ", yaw_error);
    // // uart_printf(uart, "r:%f ", 2.0 * roll_error);
    // // uart_printf(uart, "p:%f ", p);
    // uart_printf(uart, "\r\n");
  }

  // Clamp outputs between 0.0 and 1.0
  for (uint8_t i = 0; i < 4; i++) {
    outputs[i] = (outputs[i] < 0.0) ? 0.0 : outputs[i];
    outputs[i] = (outputs[i] > 1.0) ? 1.0 : outputs[i];
  }
}

//////////////////////////////////////////////////////////////////////////////
