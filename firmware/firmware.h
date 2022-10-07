#pragma once
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <Wire.h>
#include <Arduino.h>

#include "config.h"

#ifndef CMP_TOL
#define CMP_TOL 1e-6
#endif

/**
 * Mark variable unused
 */
#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

/** Return max **/
#define MAX(x, y)                                                              \
  ({                                                                           \
    __typeof__(x) _x = (x);                                                    \
    __typeof__(y) _y = (y);                                                    \
    _x > _y ? _x : _y;                                                         \
  })

/** Return min **/
#define MIN(x, y)                                                              \
  ({                                                                           \
    __typeof__(x) _x = (x);                                                    \
    __typeof__(y) _y = (y);                                                    \
    _x < _y ? _x : _y;                                                         \
  })

/**
 * Median value in buffer
 */
#define MEDIAN_VALUE(DATA_TYPE, DATA_CMP, BUF, BUF_SIZE, MEDIAN_VAR)           \
  {                                                                            \
    DATA_TYPE VALUES[BUF_SIZE] = {0};                                          \
    for (size_t i = 0; i < BUF_SIZE; i++) {                                    \
      VALUES[i] = BUF[i];                                                      \
    }                                                                          \
                                                                               \
    qsort(VALUES, BUF_SIZE, sizeof(DATA_TYPE), DATA_CMP);                      \
    if ((BUF_SIZE % 2) == 0) {                                                 \
      const size_t bwd_idx = (size_t)(BUF_SIZE - 1) / 2.0;                     \
      const size_t fwd_idx = (size_t)(BUF_SIZE + 1) / 2.0;                     \
      MEDIAN_VAR = (VALUES[bwd_idx] + VALUES[fwd_idx]) / 2.0;                  \
    } else {                                                                   \
      const size_t mid_idx = (BUF_SIZE - 1) / 2;                               \
      MEDIAN_VAR = VALUES[mid_idx];                                            \
    }                                                                          \
  }

/**
 * Convert degrees to radians
 */
inline float deg2rad(const float d) {
  return d * (M_PI / 180.0);
}

/**
 * Convert radians to degrees
 */
inline float rad2deg(const float r) {
  return r * (180.0 / M_PI);
}

/**
 * Compare int.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int uint8cmp(const void *x, const void *y) {
  if (*(uint8_t *) x < *(uint8_t *) y) {
    return -1;
  } else if (*(uint8_t *) x > *(uint8_t *) y) {
    return 1;
  }
  return 0;
}

/**
 * Compare int.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int int8cmp(const void *x, const void *y) {
  if (*(int8_t *) x < *(int8_t *) y) {
    return -1;
  } else if (*(int8_t *) x > *(int8_t *) y) {
    return 1;
  }
  return 0;
}

/**
 * Compare int.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int uint16cmp(const void *x, const void *y) {
  if (*(uint16_t *) x < *(uint16_t *) y) {
    return -1;
  } else if (*(uint16_t *) x > *(uint16_t *) y) {
    return 1;
  }
  return 0;
}

/**
 * Compare int.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int int16cmp(const void *x, const void *y) {
  if (*(int16_t *) x < *(int16_t *) y) {
    return -1;
  } else if (*(int16_t *) x > *(int16_t *) y) {
    return 1;
  }
  return 0;
}

/**
 * Compare int.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int uint32cmp(const void *x, const void *y) {
  if (*(uint32_t *) x < *(uint32_t *) y) {
    return -1;
  } else if (*(uint32_t *) x > *(uint32_t *) y) {
    return 1;
  }
  return 0;
}

/**
 * Compare int.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int int32cmp(const void *x, const void *y) {
  if (*(int32_t *) x < *(int32_t *) y) {
    return -1;
  } else if (*(int32_t *) x > *(int32_t *) y) {
    return 1;
  }
  return 0;
}

/**
 * Compare int.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int uint64cmp(const void *x, const void *y) {
  if (*(uint64_t *) x < *(uint64_t *) y) {
    return -1;
  } else if (*(uint64_t *) x > *(uint64_t *) y) {
    return 1;
  }
  return 0;
}

/**
 * Compare int.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int int64cmp(const void *x, const void *y) {
  if (*(int64_t *) x < *(int64_t *) y) {
    return -1;
  } else if (*(int64_t *) x > *(int64_t *) y) {
    return 1;
  }
  return 0;
}

/**
 * Compare int.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int intcmp(const void *x, const void *y) {
  if (*(int *) x < *(int *) y) {
    return -1;
  } else if (*(int *) x > *(int *) y) {
    return 1;
  }
  return 0;
}

/**
 * Compare reals.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int fltcmp(const void *x, const void *y) {
  if (fabs(*(float *) x - *(float *) y) < CMP_TOL) {
    return 0;
  } else if (x > y) {
    return 1;
  }

  return -1;
}

/**
 * Check if `value` is within `lower` and `upper`.
 * @returns 1 or 0 for success or failure
 */
inline uint8_t within(const float value, const float lower, const float upper) {
  if (lower <= value && value <= upper) {
    return 1;
  }
  return 0;
}

/**
 * Convert micro-seconds (us) to seconds (s)
 */
inline float us2sec(const uint32_t us) {
  return us * 1e-6;
}

/**
 * Form `int8_t` from bytes from `data` with an `offset`
 */
inline int8_t int8(const uint8_t *data, const size_t offset) {
  return (int8_t)(data[offset]);
}

/**
 * Form `uint8_t` from bytes from `data` with an `offset`
 */
inline uint8_t uint8(const uint8_t *data, const size_t offset) {
  return (uint8_t)(data[offset]);
}

/**
 * Form `int16_t` from bytes from `data` with an `offset`
 */
inline int16_t int16(const uint8_t *data, const size_t offset) {
  return (int16_t)((data[offset + 1] << 8) | (data[offset]));
}

/**
 * Form `uint16_t` from bytes from `data` with an `offset`
 */
inline uint16_t uint16(const uint8_t *data, const size_t offset) {
  return (uint16_t)((data[offset + 1] << 8) | (data[offset]));
}

/**
 * Form `int32_t` from bytes from `data` with an `offset`
 */
inline int32_t int32(const uint8_t *data, const size_t offset) {
  return (int32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                   (data[offset + 1] << 8) | (data[offset]));
}

/**
 * Form `uint32_t` from bytes from `data` with an `offset`
 */
inline uint32_t uint32(const uint8_t *data, const size_t offset) {
  return (uint32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                    (data[offset + 1] << 8) | (data[offset]));
}

/**
 * Convert float to bytes
 */
void float2bytes(const float f, uint8_t b[4]) {
  union {
    uint8_t b[4];
    float f;
  } data;

  data.f = f;
  b[0] = data.b[0];
  b[1] = data.b[1];
  b[2] = data.b[2];
  b[3] = data.b[3];
}

/**
 * Convert bytes to float
 */
void bytes2float(const uint8_t b[4], float *f) {
  union {
    char b[4];
    float f;
  } data;

  data.b[0] = b[0];
  data.b[1] = b[1];
  data.b[2] = b[2];
  data.b[3] = b[3];
  *f = data.f;
}

/**
 * Fast inverse square-root
 */
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

/**
 * Calculate mean from vector `x` of length `n`.
 * @returns Mean of x
 */
float mean(const float *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  float sum = 0.0;
  for (size_t i = 0; i < n; i++) {
    sum += x[i];
  }
  return sum / n;
}

/**
 * Calculate median from vector `x` of length `n`.
 * @returns Median of x
 */
float median(const float *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  // Make a copy of the original input vector x
  float *vals = (float *) malloc(sizeof(float) * n);
  for (size_t i = 0; i < n; i++) {
    vals[i] = x[i];
  }

  // Sort the values
  qsort(vals, n, sizeof(float), fltcmp);

  // Get median value
  float median_value = 0.0;
  if ((n % 2) == 0) {
    const int bwd_idx = (int) (n - 1) / 2.0;
    const int fwd_idx = (int) (n + 1) / 2.0;
    median_value = (vals[bwd_idx] + vals[fwd_idx]) / 2.0;
  } else {
    const int midpoint_idx = n / 2.0;
    median_value = vals[midpoint_idx];
  }

  // Clean up
  free(vals);

  return median_value;
}

/**
 * Convert Euler angles `rpy` in radians to a Hamiltonian Quaternion.
 */
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

/**
 * Convert Euler angles `rpy` (roll, pitch, yaw) in degrees to a 3x3 rotation
 * matrix `C`.
 */
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

/**
 * Dot product of two matrices or vectors `A` and `B` of size `A_m x A_n` and
 * `B_m x B_n`. Results are written to `C`.
 */
void dot(const float *A,
         const size_t A_m,
         const size_t A_n,
         const float *B,
         const size_t B_m,
         const size_t B_n,
         float *C) {
  size_t m = A_m;
  size_t n = B_n;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      for (size_t k = 0; k < A_n; k++) {
        C[(i * n) + j] += A[(i * A_n) + k] * B[(k * B_n) + j];
      }
    }
  }
}

/**
 * Convert Quaternion `q` to Euler angles 3x1 vector `euler`.
 */
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

/**
 * Convert 3x3 rotation matrix `C` to Quaternion `q`.
 */
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

/**
 * Return Quaternion norm
 */
float quat_norm(const float q[4]) {
  const float qw2 = q[0] * q[0];
  const float qx2 = q[1] * q[1];
  const float qy2 = q[2] * q[2];
  const float qz2 = q[3] * q[3];
  return sqrt(qw2 + qx2 + qy2 + qz2);
}

/**
 * Normalize Quaternion
 */
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

/**
 * Get the translational vector `r` from the 4x4 transformation matrix `T`.
 */
void tf_trans_get(const float T[16], float r[3]) {
  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

/**
 * Get the rotation matrix `C` from the 4x4 transformation matrix `T`.
 */
void tf_rot_get(const float T[16], float C[9]) {
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

/**
 * Get the quaternion `q` from the 4x4 transformation matrix `T`.
 */
void tf_quat_get(const float T[4 * 4], float q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  float C[3 * 3] = {0};
  tf_rot_get(T, C);
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

void uart_setup(uart_t *uart, int baud = 921600) {
  uart->serial_in = &Serial;
  uart->serial_out = &Serial;
  Serial.begin(baud);
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

int8_t sbus_update(sbus_t *sbus) {
  // Get sbus frame data
  uint8_t f[25] = {0};
  uint8_t new_frame = 0;
  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    if (byte != 0x0f) {
      continue;
    } else {
      Serial2.readBytes(&f[1], 24);
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
  sbus->frame_lost = (f[23] & (1 << 5));
  sbus->failsafe_activated = (f[23] & (1 << 4));
  // -- Parse channel data
  sbus->ch[0] = ((f[1] | f[2] << 8) & 0x07FF);
  sbus->ch[1] = ((f[2] >> 3 | f[3] << 5) & 0x07FF);
  sbus->ch[2] = ((f[3] >> 6 | f[4] << 2 | f[5] << 10) & 0x07FF);
  sbus->ch[3] = ((f[5] >> 1 | f[6] << 7) & 0x07FF);
  sbus->ch[4] = ((f[6] >> 4 | f[7] << 4) & 0x07FF);
  sbus->ch[5] = ((f[7] >> 7 | f[8] << 1 | f[8] << 9) & 0x07FF);
  sbus->ch[6] = ((f[9] >> 2 | f[10] << 6) & 0x07FF);
  sbus->ch[7] = ((f[10] >> 5 | f[11] << 3) & 0x07FF);
  sbus->ch[8] = ((f[12] | f[13] << 8) & 0x07FF);
  sbus->ch[9] = ((f[13] >> 3 | f[14] << 5) & 0x07FF);
  sbus->ch[10] = ((f[14] >> 6 | f[15] << 2 | f[16] << 10) & 0x07FF);
  sbus->ch[11] = ((f[16] >> 1 | f[17] << 7) & 0x07FF);
  sbus->ch[12] = ((f[17] >> 4 | f[18] << 4) & 0x07FF);
  sbus->ch[13] = ((f[18] >> 7 | f[19] << 1 | f[20] << 9) & 0x07FF);
  sbus->ch[14] = ((f[20] >> 2 | f[21] << 6) & 0x07FF);
  sbus->ch[15] = ((f[21] >> 5 | f[22] << 3) & 0x07FF);

  return 0;
}

float sbus_value(const sbus_t *sbus, const uint8_t ch_idx) {
  return (sbus->ch[ch_idx] - PWM_VALUE_MIN) / (PWM_VALUE_MAX - PWM_VALUE_MIN);
}

float sbus_thrust(const sbus_t *sbus) {
  return (sbus->ch[0] - PWM_VALUE_MIN) / (PWM_VALUE_MAX - PWM_VALUE_MIN);
}

float sbus_altitude(const sbus_t *sbus) {
  return (sbus->ch[0] - PWM_VALUE_MIN) / (PWM_VALUE_MAX - PWM_VALUE_MIN);
}

float sbus_roll(const sbus_t *sbus) {
  // Within deadband
  if (sbus->ch[1] > 950 && sbus->ch[1] < 1050) {
    return 0.0;
  }

  // Outside deadband
  const float pwm_half_diff = (PWM_VALUE_MAX - PWM_VALUE_MIN) / 2.0;
  const float pwm_mean = pwm_half_diff + PWM_VALUE_MIN;
  return (MAX_TILT_RAD / 1.5) * ((sbus->ch[1] - pwm_mean) / pwm_half_diff);
}

float sbus_pitch(const sbus_t *sbus) {
  // Within deadband
  if (sbus->ch[2] > 950 && sbus->ch[2] < 1050) {
    return 0.0;
  }

  // Outside deadband
  const float pwm_half_diff = (PWM_VALUE_MAX - PWM_VALUE_MIN) / 2.0;
  const float pwm_mean = pwm_half_diff + PWM_VALUE_MIN;
  return (MAX_TILT_RAD / 1.5) * ((sbus->ch[2] - pwm_mean) / pwm_half_diff);
}

float sbus_yaw(const sbus_t *sbus) {
  // Within deadband
  if (sbus->ch[3] > 950 && sbus->ch[3] < 1050) {
    return 0.0;
  }

  // Outside deadband
  const float pwm_half_diff = (PWM_VALUE_MAX - PWM_VALUE_MIN) / 2.0;
  const float pwm_mean = pwm_half_diff + PWM_VALUE_MIN;
  return (MAX_TILT_RAD / 1.5) * ((sbus->ch[3] - pwm_mean) / pwm_half_diff);
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
  float gravity;
} mpu6050_t;

void mpu6050_setup(mpu6050_t *imu);
void mpu6050_calibrate(mpu6050_t *imu);
void mpu6050_get_data(mpu6050_t *imu);

void mpu6050_setup(mpu6050_t *imu) {
  // Config
  imu->dplf_config = 3;
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
}

void mpu6050_calibrate(mpu6050_t *imu) {
  float wx_sum = 0.0;
  float wy_sum = 0.0;
  float wz_sum = 0.0;
  float ax_sum = 0.0;
  float ay_sum = 0.0;
  float az_sum = 0.0;

  uint16_t nb_samples = 2000;
  for (uint32_t i = 0; i < nb_samples; i++) {
    mpu6050_get_data(imu);

    wx_sum += imu->gyro[0];
    wy_sum += imu->gyro[1];
    wz_sum += imu->gyro[2];

    ax_sum += imu->accel[0];
    ay_sum += imu->accel[1];
    az_sum += imu->accel[2];
  }

  imu->gyro_offset[0] = wx_sum / (float) nb_samples;
  imu->gyro_offset[1] = wy_sum / (float) nb_samples;
  imu->gyro_offset[2] = wz_sum / (float) nb_samples;

  ax_sum = ax_sum / (float) nb_samples;
  ay_sum = ay_sum / (float) nb_samples;
  az_sum = az_sum / (float) nb_samples;

  imu->gravity = sqrt(ax_sum * ax_sum + ay_sum * ay_sum + az_sum * az_sum);
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

void mahony_filter_setup(mahony_filter_t *filter, const float a[3]) {
  const float roll = atan2(a[1], a[2]);
  const float pitch = atan2(-a[0], sqrt(a[1] * a[1] + a[2] * a[2]));
  const float yaw = 0.0;
  const float rpy[3] = {roll, pitch, yaw};
  euler2quat(rpy, filter->q);

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

  filter->integralFBx = 0.0f;
  filter->integralFBy = 0.0f;
  filter->integralFBz = 0.0f;
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
  pid_ctrl_setup(yaw_pid, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
}

void att_ctrl_update(att_ctrl_t *att_ctrl,
                     const float setpoint[4],
                     const float actual[3],
                     const float dt,
                     float outputs[4]) {
  // Calculate roll error
  const float roll_desired = setpoint[0];
  const float roll_actual = actual[0];
  const float roll_error = roll_desired - roll_actual;

  // Calculate pitch error
  const float pitch_desired = setpoint[1];
  const float pitch_actual = actual[1];
  const float pitch_error = pitch_desired - pitch_actual;

  // Calculate yaw error
  const float yaw_desired = setpoint[2];
  const float yaw_actual = actual[2];
  float yaw_error = yaw_desired - yaw_actual;
  if (yaw_error > M_PI) {
    yaw_error -= M_PI;
  } else if (yaw_error < -M_PI) {
    yaw_error += M_PI;
  }

  // PID controller on roll, pitch and yaw
  const float r = pid_ctrl_update(&att_ctrl->roll_pid, roll_error, dt);
  const float p = pid_ctrl_update(&att_ctrl->pitch_pid, pitch_error, dt);
  const float y = pid_ctrl_update(&att_ctrl->yaw_pid, yaw_error, dt);
  const float t = setpoint[3];

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
  outputs[0] = t - r + p - y;
  outputs[1] = t - r - p + y;
  outputs[2] = t + r + p + y;
  outputs[3] = t + r - p - y;

  // Clamp outputs between 0.0 and 1.0
  for (uint8_t i = 0; i < 4; i++) {
    outputs[i] = (outputs[i] < THROTTLE_MIN) ? THROTTLE_MIN : outputs[i];
    outputs[i] = (outputs[i] > THROTTLE_MAX) ? THROTTLE_MAX : outputs[i];
  }
}

//////////////////////////////////////////////////////////////////////////////

// TELEMETRY DATA ////////////////////////////////////////////////////////////

#define TELEM_BUF_SIZE 10

typedef struct telemetry_t {
  uint16_t buf_size;

  uint32_t ts[TELEM_BUF_SIZE];
  uint8_t mav_arm[TELEM_BUF_SIZE];
  uint8_t mav_ready[TELEM_BUF_SIZE];
  uint8_t mav_failsafe[TELEM_BUF_SIZE];

  float acc[TELEM_BUF_SIZE][3];
  float gyr[TELEM_BUF_SIZE][3];
  float attitude[TELEM_BUF_SIZE][3];
  float height[TELEM_BUF_SIZE];

  uint16_t sbus[TELEM_BUF_SIZE][16];
  float setpoint[TELEM_BUF_SIZE][4];
  float outputs[TELEM_BUF_SIZE][4];
} telemetry_t;

void telemetry_setup(telemetry_t *telem) {
  telem->buf_size = 0;

  for (size_t k = 0; k < TELEM_BUF_SIZE; k++) {
    telem->ts[k] = 0;
    telem->mav_arm[k] = 0;
    telem->mav_ready[k] = 0;
    telem->mav_failsafe[k] = 0;

    for (size_t i = 0; i < 3; i++) {
      telem->acc[k][i] = 0;
      telem->gyr[k][i] = 0;
      telem->attitude[k][i] = 0;
    }

    for (size_t i = 0; i < 16; i++) {
      telem->sbus[k][i] = 0;
    }
    for (size_t i = 0; i < 4; i++) {
      telem->setpoint[k][i] = 0;
      telem->outputs[k][i] = 0;
    }
  }
}

void telemetry_reset(telemetry_t *telem) {
  telemetry_setup(telem);
}

void telemetry_record(telemetry_t *telem,
                      const uint32_t ts_us,
                      const uint8_t mav_arm,
                      const uint8_t mav_ready,
                      const uint8_t mav_failsafe,
                      const float acc[3],
                      const float gyr[3],
                      const float state[4],
                      const float setpoint[3],
                      const float outputs[4],
                      const uint16_t sbus_channels[16]) {
  const uint16_t buf_size = telem->buf_size;

  telem->ts[buf_size] = ts_us;
  telem->mav_arm[buf_size] = mav_arm;
  telem->mav_ready[buf_size] = mav_ready;
  telem->mav_failsafe[buf_size] = mav_failsafe;

  telem->acc[buf_size][0] = acc[0];
  telem->acc[buf_size][1] = acc[1];
  telem->acc[buf_size][2] = acc[2];

  telem->gyr[buf_size][0] = gyr[0];
  telem->gyr[buf_size][1] = gyr[1];
  telem->gyr[buf_size][2] = gyr[2];

  telem->attitude[buf_size][0] = state[0];
  telem->attitude[buf_size][1] = state[1];
  telem->attitude[buf_size][2] = state[2];
  telem->height[buf_size] = state[3];

  telem->setpoint[buf_size][0] = setpoint[0];
  telem->setpoint[buf_size][1] = setpoint[1];
  telem->setpoint[buf_size][2] = setpoint[2];
  telem->setpoint[buf_size][3] = setpoint[3];

  telem->outputs[buf_size][0] = outputs[0];
  telem->outputs[buf_size][1] = outputs[1];
  telem->outputs[buf_size][2] = outputs[2];
  telem->outputs[buf_size][3] = outputs[3];

  for (uint8_t i = 0; i < 16; i++) {
    telem->sbus[buf_size][i] = sbus_channels[i];
  }
  telem->buf_size++;
}

void telemetry_transmit(telemetry_t *telem, uart_t *uart) {
  const uint32_t ts_telem = micros();

  for (uint16_t k = 0; k < telem->buf_size; k++) {
    char s[1024] = {0};
    sprintf(s, "telem_ts:%ld ", ts_telem);
    sprintf(s + strlen(s), "ts:%ld ", telem->ts[k]);
    sprintf(s + strlen(s), "mav_arm:%d ", telem->mav_arm[k]);
    sprintf(s + strlen(s), "mav_ready:%d ", telem->mav_ready[k]);

    for (uint8_t i = 0; i < 16; i++) {
      sprintf(s + strlen(s), "ch[%d]:%d ", i, telem->sbus[k][i]);
    }

    sprintf(s + strlen(s), "accel_x:%f ", telem->acc[k][0]);
    sprintf(s + strlen(s), "accel_y:%f ", telem->acc[k][1]);
    sprintf(s + strlen(s), "accel_z:%f ", telem->acc[k][2]);

    sprintf(s + strlen(s), "gyro_x:%f ", telem->gyr[k][0]);
    sprintf(s + strlen(s), "gyro_y:%f ", telem->gyr[k][1]);
    sprintf(s + strlen(s), "gyro_z:%f ", telem->gyr[k][2]);

    sprintf(s + strlen(s), "roll:%f ", rad2deg(telem->attitude[k][0]));
    sprintf(s + strlen(s), "pitch:%f ", rad2deg(telem->attitude[k][1]));
    sprintf(s + strlen(s), "yaw:%f ", rad2deg(telem->attitude[k][2]));
    sprintf(s + strlen(s), "height:%f ", telem->height[k]);

    const float r = rad2deg(telem->setpoint[k][0]);
    const float p = rad2deg(telem->setpoint[k][1]);
    const float y = rad2deg(telem->setpoint[k][2]);
    sprintf(s + strlen(s), "roll_setpoint:%f ", r);
    sprintf(s + strlen(s), "pitch_setpoint:%f ", p);
    sprintf(s + strlen(s), "yaw_setpoint:%f ", y);
    sprintf(s + strlen(s), "thrust_setpoint:%f ", telem->setpoint[k][3]);

    sprintf(s + strlen(s), "outputs[0]:%f ", telem->outputs[k][0]);
    sprintf(s + strlen(s), "outputs[1]:%f ", telem->outputs[k][1]);
    sprintf(s + strlen(s), "outputs[2]:%f ", telem->outputs[k][2]);
    sprintf(s + strlen(s), "outputs[3]:%f ", telem->outputs[k][3]);

    sprintf(s + strlen(s), "\r\n");

    uart_printf(uart, "%s", s);
  }
}

//////////////////////////////////////////////////////////////////////////////
