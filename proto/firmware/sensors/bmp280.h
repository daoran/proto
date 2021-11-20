#pragma once

#include "../comms/i2c.h"

// BMP280 Addresses
#define BMP280_ADDR 0x77      // Primary I2C Address
#define BMP280_ADDR_ALT 0x76  // Alternate Address

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
#define BMP280_REG_CAL26 0xE1    // R calibration stored in 0xE1-0xF0
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
void bmp280_get_data(const bmp280_t *bmp280, float *temperature, float *pressure);

void bmp280_setup(bmp280_t *bmp280) {
  uint8_t sensor_addr = BMP280_ADDR_ALT;

  // Reset the device using soft-reset
  i2c_write_byte(sensor_addr, BMP280_REG_SOFTRESET, 0xB6);

  // Wait for chip to wake up.
  delay(10);

  // Get calibration data
  bmp280_get_calib_data(bmp280);

  // Configure sensor
  uint8_t t_sb = BMP280_STANDBY_MS_0_5;  // Standby time in normal mode
  uint8_t filter = BMP280_FILTER_X16;    // Filter settings
  uint8_t spi3w_en = 0;                  // Enable 3-wire SPI
  uint8_t config = (t_sb << 5) | (filter << 2) | spi3w_en;
  i2c_write_byte(sensor_addr, BMP280_REG_CONFIG, config);

  // Configure measurement
  uint8_t mode = BMP280_MODE_NORMAL;     // Device mode
  uint8_t osrs_p = BMP280_SAMPLING_X16;  // Pressure oversampling
  uint8_t osrs_t = BMP280_SAMPLING_X2;   // Temperature oversampling
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
    var1 = (((((int64_t)1) << 47) + var1)) * P1 >> 33;

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

