#pragma once

#include <Wire.h>

#include "../core.h"

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
  return uint16(bytes);
}

int16_t i2c_read_s16(const uint8_t dev_addr, const uint8_t reg_addr) {
  return i2c_read_u16(dev_addr, reg_addr);
}

uint32_t i2c_read_u32(const uint8_t dev_addr, const uint8_t reg_addr) {
  uint8_t bytes[4] = {0};
  i2c_read_bytes(dev_addr, reg_addr, 4, bytes);
  return uint32(bytes);
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
