#pragma once
#include <Wire.h>
#include <Arduino.h>

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

////////////////////////////////////////////////////////////////////////////////
