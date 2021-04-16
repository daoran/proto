#include <Wire.h>
#include <Arduino.h>

class twi {
  public:
    const uint32_t BUF_MAX = 1024;

    twi() {
      Wire.setClock(400000);
      Wire.begin();
    }

    uint8_t read_byte(const uint8_t dev_addr, const uint8_t reg_addr) {
      Wire.beginTransmission(dev_addr);
      Wire.write(reg_addr);
      Wire.endTransmission();

      uint8_t size = 1;
      uint8_t last = 1;
      Wire.requestFrom(dev_addr, size, last);
      const uint8_t data = Wire.read();

      return data;
    }

    void read_bytes(const uint8_t dev_addr,
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

    uint8_t read_u8(const uint8_t dev_addr, const uint8_t reg_addr) {
      return read_byte(dev_addr, reg_addr);
    }

    int8_t read_s8(const uint8_t dev_addr, const uint8_t reg_addr) {
      return read_byte(dev_addr, reg_addr);
    }

    uint16_t read_u16(const uint8_t dev_addr, const uint8_t reg_addr) {
      uint8_t bytes[2] = {0};
      read_bytes(dev_addr, reg_addr, 2, bytes);
      return uint16(bytes);
    }

    int16_t read_s16(const uint8_t dev_addr, const uint8_t reg_addr) {
      return read_u16(dev_addr, reg_addr);
    }

    uint32_t read_u32(const uint8_t dev_addr, const uint8_t reg_addr) {
      uint8_t bytes[4] = {0};
      read_bytes(dev_addr, reg_addr, 4, bytes);
      return uint32(bytes);
    }

    int32_t read_s32(const uint8_t dev_addr, const uint8_t reg_addr) {
      return read_u32(dev_addr, reg_addr);
    }

    void write_byte(const uint8_t dev_addr,
                    const uint8_t reg_addr,
                    const uint8_t value) {
      Wire.beginTransmission(dev_addr);
      Wire.write(reg_addr);
      Wire.write(value);
      Wire.endTransmission();
    }

    void write_bytes(const uint8_t dev_addr,
                     const uint8_t reg_addr,
                     const uint8_t *data,
                     const size_t length) {
      Wire.beginTransmission(dev_addr);
      Wire.write(reg_addr);
      Wire.write(data, length);
      Wire.endTransmission();
    }

    void scan_addrs(uint8_t *addrs, uint8_t *nb_addrs) {
      *nb_addrs = 0;

      for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          addrs[*nb_addrs] = addr;
          *nb_addrs += 1;
        }
      }
    }

    /*
      void scan_addrs(uart_t &serial) {
      uint8_t addrs[128] = {0};
      uint8_t nb_addrs;
      scan_addrs(addrs, &nb_addrs);

      serial.println("scanning i2c devices:\n\r");
      serial.println("nb_addrs: %d\n\r", nb_addrs);
      for (uint8_t i = 0; i < nb_addrs; i++) {
        serial.printf("%X\n\r", addrs[i]);
      }

      delay(5000);
      }
    */
};
