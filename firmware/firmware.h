#pragma once
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <Wire.h>
#include <Arduino.h>

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

/* void __assert(const char *__func, */
/*               const char *__file, */
/*               int __lineno, const char *__sexp) { */
/*   // transmit diagnostic informations through serial link. */
/*   Serial.println(__func); */
/*   Serial.println(__file); */
/*   Serial.println(__lineno, DEC); */
/*   Serial.println(__sexp); */
/*   Serial.flush(); */
/*   // abort program execution. */
/*   abort(); */
/* } */

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
  Serial.begin(baud);
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
      int places = 2;
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

#define I2C_MAX_BUF_LEN 1024

void i2c_setup() {
  Wire.setClock(400000);
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
  uint8_t *pins;
  uint8_t nb_pins;
  int res;
  float freq;
  float range_max;
  float min;
  float max;
} pwm_t;

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
    case 16:
      range_max = 65535.0f;
      break;
    case 15:
      range_max = 32767.0f;
      break;
    case 14:
      range_max = 16383.0f;
      break;
    case 13:
      range_max = 8191.0f;
      break;
    case 12:
      range_max = 4095.0f;
      break;
    case 11:
      range_max = 2047.0f;
      break;
    case 10:
      range_max = 1023.0f;
      break;
    case 9:
      range_max = 511.0f;
      break;
    case 8:
      range_max = 255.0f;
      break;
    case 7:
      range_max = 127.0f;
      break;
    case 6:
      range_max = 63.0f;
      break;
    case 5:
      range_max = 31.0f;
      break;
    case 4:
      range_max = 15.0f;
      break;
    case 3:
      range_max = 7.0f;
      break;
    case 2:
      range_max = 3.0f;
      break;
    default:
      range_max = 255.0f;
      break;
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

//////////////////////////////////////////////////////////////////////////////

// SBUS //////////////////////////////////////////////////////////////////////

void sbus_setup() {
  // The SBUS protocol uses inverted serial logic with:
  // - Baud rate of 100000
  // - 8 data bits
  // - Even parity bit
  // - 2 stop bits
  HardwareSerial *bus = &Serial1;
  bus->begin(100000, SERIAL_8E2);
  bus->flush();
}

uint8_t sbus_read_byte() {
  return Serial1.read();
}

void sbus_read(uint8_t *data, const size_t size) {
  Serial1.readBytes(data, size);
}

void sbus_write_byte(const uint8_t data) {
  Serial1.write(data);
}

void sbus_write(const uint8_t *data, const size_t size) {
  Serial1.write(data, size);
}

void sbus_update(uart_t *uart) {
  // Get sbus frame data
  /* uint8_t frame[25] = {0}; */
  /* frame[0] = sbus_read_byte(); */
  /* if (frame[0] != 0x0F) { */
  /*   uart_printf(uart, "Failed!\n"); */
  /*   return; */
  /* } else { */
  /*   sbus_read(&frame[1], 24); */
  /* } */
  if (Serial1.available()) {
    uart_printf(uart, "%X\n", sbus_read_byte());
  } else {
    uart_printf(uart, "nothing...\n");
  }

  // Parse sbus frame
  // -- Parse flag
  /* uint8_t frame_lost = (frame[23] & (1 << 5)); */
  /* uint8_t failsafe_activated = (frame[23] & (1 << 4)); */
  // -- Parse channel data
  /* uint16_t ch[16] = {0}; */
  /* ch[0] = ((frame[1] | frame[2] << 8) & 0x07FF); */
  /* ch[1] = ((frame[2] >> 3 | frame[3] << 5) & 0x07FF); */
  /* ch[2] = ((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF); */
  /* ch[3] = ((frame[5] >> 1 | frame[6] << 7) & 0x07FF); */
  /* ch[4] = ((frame[6] >> 4 | frame[7] << 4) & 0x07FF); */
  /* ch[5] = ((frame[7] >> 7 | frame[8] << 1 | frame[8] << 9) & 0x07FF); */
  /* ch[6] = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF); */
  /* ch[7] = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF); */
  /* ch[8] = ((frame[12] | frame[13] << 8) & 0x07FF); */
  /* ch[9] = ((frame[13] >> 3 | frame[14] << 5) & 0x07FF); */
  /* ch[10] = ( (frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF);
   */
  /* ch[11] = ((frame[16] >> 1 | frame[17] << 7) & 0x07FF); */
  /* ch[12] = ((frame[17] >> 4 | frame[18] << 4) & 0x07FF); */
  /* ch[13] = ( (frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF);
   */
  /* ch[14] = ((frame[20] >> 2 | frame[21] << 6) & 0x07FF); */
  /* ch[15] = ((frame[21] >> 5 | frame[22] << 3) & 0x07FF); */
  /*  */
  /* char ch_str[10] = {0}; */
  /* itoa(ch[0], ch_str, 10); */

  /* uart_printf(uart, "HERE!\n"); */
  /* uart_printf(uart, "%d, ", ch[0]); */
  /* uart_printf(uart, "%d, ", ch[1]); */
  /* uart_printf(uart, "%d, ", ch[2]); */
  /* uart_printf(uart, "%d\n", ch[3]); */

  /* uart_printf(uart, ch_str); */
  /* uart_printf(uart, "\r\n"); */
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
#define MPU6050_REG_XG_OFFS_TC 0x00
#define MPU6050_REG_YG_OFFS_TC 0x01
#define MPU6050_REG_ZG_OFFS_TC 0x02
#define MPU6050_REG_X_FINE_GAIN 0x03
#define MPU6050_REG_Y_FINE_GAIN 0x04
#define MPU6050_REG_Z_FINE_GAIN 0x05
#define MPU6050_REG_XA_OFFS_H 0x06
#define MPU6050_REG_XA_OFFS_L_TC 0x07
#define MPU6050_REG_YA_OFFS_H 0x08
#define MPU6050_REG_YA_OFFS_L_TC 0x09
#define MPU6050_REG_ZA_OFFS_H 0x0A
#define MPU6050_REG_ZA_OFFS_L_TC 0x0B
#define MPU6050_REG_XG_OFFS_USRH 0x13
#define MPU6050_REG_XG_OFFS_USRL 0x14
#define MPU6050_REG_YG_OFFS_USRH 0x15
#define MPU6050_REG_YG_OFFS_USRL 0x16
#define MPU6050_REG_ZG_OFFS_USRH 0x17
#define MPU6050_REG_ZG_OFFS_USRL 0x18
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_FF_THR 0x1D
#define MPU6050_REG_FF_DUR 0x1E
#define MPU6050_REG_MOT_THR 0x1F
#define MPU6050_REG_MOT_DUR 0x20
#define MPU6050_REG_ZRMOT_THR 0x21
#define MPU6050_REG_ZRMOT_DUR 0x22
#define MPU6050_REG_FIFO_EN 0x23
#define MPU6050_REG_I2C_MST_CTRL 0x24
#define MPU6050_REG_I2C_SLV0_ADDR 0x25
#define MPU6050_REG_I2C_SLV0_REG 0x26
#define MPU6050_REG_I2C_SLV0_CTRL 0x27
#define MPU6050_REG_I2C_SLV1_ADDR 0x28
#define MPU6050_REG_I2C_SLV1_REG 0x29
#define MPU6050_REG_I2C_SLV1_CTRL 0x2A
#define MPU6050_REG_I2C_SLV2_ADDR 0x2B
#define MPU6050_REG_I2C_SLV2_REG 0x2C
#define MPU6050_REG_I2C_SLV2_CTRL 0x2D
#define MPU6050_REG_I2C_SLV3_ADDR 0x2E
#define MPU6050_REG_I2C_SLV3_REG 0x2F
#define MPU6050_REG_I2C_SLV3_CTRL 0x30
#define MPU6050_REG_I2C_SLV4_ADDR 0x31
#define MPU6050_REG_I2C_SLV4_REG 0x32
#define MPU6050_REG_I2C_SLV4_DO 0x33
#define MPU6050_REG_I2C_SLV4_CTRL 0x34
#define MPU6050_REG_I2C_SLV4_DI 0x35
#define MPU6050_REG_I2C_MST_STATUS 0x36
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_REG_INT_ENABLE 0x38
#define MPU6050_REG_DMP_INT_STATUS 0x39
#define MPU6050_REG_INT_STATUS 0x3A
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_ACCEL_XOUT_L 0x3C
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_YOUT_L 0x3E
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_ACCEL_ZOUT_L 0x40
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_TEMP_OUT_L 0x42
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_GYRO_XOUT_L 0x44
#define MPU6050_REG_GYRO_YOUT_H 0x45
#define MPU6050_REG_GYRO_YOUT_L 0x46
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define MPU6050_REG_GYRO_ZOUT_L 0x48
#define MPU6050_REG_EXT_SENS_DATA_00 0x49
#define MPU6050_REG_EXT_SENS_DATA_01 0x4A
#define MPU6050_REG_EXT_SENS_DATA_02 0x4B
#define MPU6050_REG_EXT_SENS_DATA_03 0x4C
#define MPU6050_REG_EXT_SENS_DATA_04 0x4D
#define MPU6050_REG_EXT_SENS_DATA_05 0x4E
#define MPU6050_REG_EXT_SENS_DATA_06 0x4F
#define MPU6050_REG_EXT_SENS_DATA_07 0x50
#define MPU6050_REG_EXT_SENS_DATA_08 0x51
#define MPU6050_REG_EXT_SENS_DATA_09 0x52
#define MPU6050_REG_EXT_SENS_DATA_10 0x53
#define MPU6050_REG_EXT_SENS_DATA_11 0x54
#define MPU6050_REG_EXT_SENS_DATA_12 0x55
#define MPU6050_REG_EXT_SENS_DATA_13 0x56
#define MPU6050_REG_EXT_SENS_DATA_14 0x57
#define MPU6050_REG_EXT_SENS_DATA_15 0x58
#define MPU6050_REG_EXT_SENS_DATA_16 0x59
#define MPU6050_REG_EXT_SENS_DATA_17 0x5A
#define MPU6050_REG_EXT_SENS_DATA_18 0x5B
#define MPU6050_REG_EXT_SENS_DATA_19 0x5C
#define MPU6050_REG_EXT_SENS_DATA_20 0x5D
#define MPU6050_REG_EXT_SENS_DATA_21 0x5E
#define MPU6050_REG_EXT_SENS_DATA_22 0x5F
#define MPU6050_REG_EXT_SENS_DATA_23 0x60
#define MPU6050_REG_MOT_DETECT_STATUS 0x61
#define MPU6050_REG_I2C_SLV0_DO 0x63
#define MPU6050_REG_I2C_SLV1_DO 0x64
#define MPU6050_REG_I2C_SLV2_DO 0x65
#define MPU6050_REG_I2C_SLV3_DO 0x66
#define MPU6050_REG_I2C_MST_DELAY_CTRL 0x67
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68
#define MPU6050_REG_MOT_DETECT_CTRL 0x69
#define MPU6050_REG_USER_CTRL 0x6A
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_PWR_MGMT_2 0x6C
#define MPU6050_REG_BANK_SEL 0x6D
#define MPU6050_REG_MEM_START_ADDR 0x6E
#define MPU6050_REG_MEM_R_W 0x6F
#define MPU6050_REG_DMP_CFG_1 0x70
#define MPU6050_REG_DMP_CFG_2 0x71
#define MPU6050_REG_FIFO_COUNTH 0x72
#define MPU6050_REG_FIFO_COUNTL 0x73
#define MPU6050_REG_FIFO_R_W 0x74
#define MPU6050_REG_WHO_AM_I 0x75

typedef struct mpu6050_t {
  int8_t ok;

  float accel_sensitivity;
  float gyro_sensitivity;
  float accel[3];
  float gyro[3];

  float temperature;
  float sample_rate;
  int8_t dplf_config;
} mpu6050_t;

int8_t mpu6050_setup(mpu6050_t *imu);
uint8_t mpu6050_ping(const mpu6050_t *imu);
int8_t mpu6050_get_data(mpu6050_t *imu);
int8_t mpu6050_set_dplf(const mpu6050_t *imu, const uint8_t setting);
int8_t mpu6050_get_dplf(const mpu6050_t *imu);
int8_t mpu6050_set_sample_rate_div(const mpu6050_t *imu, const int8_t div);
int8_t mpu6050_get_sample_rate_div(const mpu6050_t *imu);
int8_t mpu6050_get_sample_rate(const mpu6050_t *imu);
int8_t mpu6050_get_gyro_range(const mpu6050_t *imu, int8_t *range);
int8_t mpu6050_set_gyro_range(const mpu6050_t *imu, const int8_t range);
int8_t mpu6050_get_accel_range(const mpu6050_t *imu, int8_t *range);
int8_t mpu6050_set_accel_range(const mpu6050_t *imu, const int8_t range);

int8_t mpu6050_setup(mpu6050_t *imu) {
  int8_t retval = 0;
  int8_t dplf = 1;
  int8_t accel_range = 0;
  int8_t gyro_range = 0;

  // Set dplf
  mpu6050_set_dplf(imu, dplf);
  retval = mpu6050_get_dplf(imu);
  if (retval > 7 || retval < 0) {
    return -1;
  } else {
    imu->dplf_config = retval;
  }

  // Set power management register
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x00);

  // Configure gyro range
  if (mpu6050_set_gyro_range(imu, gyro_range) != 0) {
    return -1;
  }
  switch (gyro_range) {
    case 0:
      imu->gyro_sensitivity = 131.0;
      break;
    case 1:
      imu->gyro_sensitivity = 65.5;
      break;
    case 2:
      imu->gyro_sensitivity = 32.8;
      break;
    case 3:
      imu->gyro_sensitivity = 16.4;
      break;
    default:
      goto error;
  }

  // Configure accel range
  if (mpu6050_set_accel_range(imu, accel_range) != 0) {
    goto error;
  }
  switch (accel_range) {
    case 0:
      imu->accel_sensitivity = 16384.0;
      break;
    case 1:
      imu->accel_sensitivity = 8192.0;
      break;
    case 2:
      imu->accel_sensitivity = 4096.0;
      break;
    case 3:
      imu->accel_sensitivity = 2048.0;
      break;
    default:
      return -3;
  }

  // Get sample rate
  imu->sample_rate = mpu6050_get_sample_rate(imu);

  return 0;
error:
  return -1;
}

uint8_t mpu6050_ping(const mpu6050_t *imu) {
  UNUSED(imu);
  return i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_WHO_AM_I);
}

int8_t mpu6050_get_data(mpu6050_t *imu) {
  // Read data
  uint8_t raw_data[14] = {0};
  i2c_read_bytes(MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H, 14, raw_data);

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

  return 0;
}

int8_t mpu6050_set_dplf(const mpu6050_t *imu, const uint8_t setting) {
  UNUSED(imu);

  /*
      DPLF_CFG    Accelerometer
      ----------------------------------------
                  Bandwidth(Hz) | Delay(ms)
      0           260             0
      1           184             2.0
      2           94              3.0
      3           44              4.9
      4           21              8.5
      5           10              13.8
      6           5               19.0
      7           RESERVED        RESERVED

      DPLF_CFG    Gyroscope
      ----------------------------------------------
                  Bandwidth(Hz) | Delay(ms) | Fs(kHz)
      0           256             0.98        8
      1           188             1.9         1
      2           98              2.8         1
      3           42              4.8         1
      4           20              8.3         1
      5           10              13.4        1
      6           5               18.5        1
      7           RESERVED        RESERVED    8
  */

  // Check setting range
  if (setting > 7) {
    return -2;
  }

  // Set DPLF
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_CONFIG, (char) setting);

  return 0;
}

int8_t mpu6050_get_dplf(const mpu6050_t *imu) {
  UNUSED(imu);

  uint8_t data[1] = {0x00};
  i2c_read_bytes(MPU6050_ADDRESS, MPU6050_REG_CONFIG, 1, data);
  data[0] = data[0] & 0b00000111;
  return data[0];
}

int8_t mpu6050_set_sample_rate_div(const mpu6050_t *imu, const int8_t div) {
  UNUSED(imu);
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_SMPLRT_DIV, div);
  return 0;
}

int8_t mpu6050_get_sample_rate_div(const mpu6050_t *imu) {
  UNUSED(imu);
  return i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_SMPLRT_DIV);
}

int8_t mpu6050_get_sample_rate(const mpu6050_t *imu) {
  // Get sample rate divider
  uint16_t sample_div = 0;
  int8_t rate_div = mpu6050_get_sample_rate_div(imu);
  if (rate_div != -1 || rate_div != -2) {
    sample_div = (float) rate_div;
  } else {
    return -1;
  }

  // Get gyro sample rate
  uint16_t gyro_rate = 0;
  uint8_t dlpf_cfg = mpu6050_get_sample_rate_div(imu);
  if (dlpf_cfg == 0 || dlpf_cfg == 7) {
    gyro_rate = 8000;
  } else if (dlpf_cfg >= 1 || dlpf_cfg <= 6) {
    gyro_rate = 1000;
  } else {
    return -2;
  }

  // Calculate sample rate
  return gyro_rate / (1 + sample_div);
}

int8_t mpu6050_get_gyro_range(const mpu6050_t *imu, int8_t *range) {
  UNUSED(imu);

  // Get gyro config
  const uint8_t data = i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG);

  // Get gyro range bytes
  *range = (data >> 3) & 0b00000011;

  return 0;
}

int8_t mpu6050_set_gyro_range(const mpu6050_t *imu, const int8_t range) {
  UNUSED(imu);

  // Pre-check
  if (range > 3 || range < 0) {
    return -2;
  }

  // Set sample rate
  uint8_t data = range << 3;
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_GYRO_CONFIG, data);

  // Double check
  int8_t gyro_range = 0;
  int8_t retval = mpu6050_get_gyro_range(imu, &gyro_range);
  if (retval != 0 || (gyro_range != range)) {
    return -1;
  }

  return 0;
}

int8_t mpu6050_get_accel_range(const mpu6050_t *imu, int8_t *range) {
  UNUSED(imu);

  // Get accel config
  uint8_t data = i2c_read_byte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG);

  // Get accel range bytes
  *range = (data >> 3) & 0b00000011;

  return 0;
}

int8_t mpu6050_set_accel_range(const mpu6050_t *imu, const int8_t range) {
  UNUSED(imu);

  // Pre-check
  if (range > 3 || range < 0) {
    return -2;
  }

  // Set sample rate
  uint8_t data = range << 3;
  i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_ACCEL_CONFIG, data);

  // Double check
  int8_t accel_range = 0;
  int8_t retval = mpu6050_get_accel_range(imu, &accel_range);
  if (retval != 0 || (accel_range != range)) {
    return -1;
  }

  return 0;
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

float pid_ctrl_update(pid_ctrl_t *pid,
                      const float setpoint,
                      const float actual,
                      const float dt) {
  // Calculate errors
  const float error = setpoint - actual;
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
