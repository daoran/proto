#include "proto/driver/pca9685.hpp"

namespace proto {

int pca9685_configure(pca9685_t &pwm, const int freq) {
  // Setup
  i2c_set_slave(pwm.i2c, PCA9685_I2C_ADDR);
  pca9685_reset(pwm);
  pca9685_set_pwm(pwm, 0);

  // Configure mode 2 register
  i2c_write_byte(pwm.i2c, PCA9685_MODE2, 0x04);
  i2c_write_byte(pwm.i2c, PCA9685_MODE1, 0x01);
  usleep(PCA9685_WAIT_MS);

  // Configure mode 1 register
  char mode_1;
  i2c_read_byte(pwm.i2c, PCA9685_MODE1, &mode_1);
  mode_1 = mode_1 & ~0x10;
  i2c_write_byte(pwm.i2c, PCA9685_MODE1, mode_1);
  usleep(PCA9685_WAIT_MS);

  // Set frequency
  pca9685_set_frequency(pwm, freq);
  usleep(PCA9685_WAIT_MS);

  return 0;
}

void pca9685_set_frequency(const pca9685_t &pwm, const int freq) {
  // Setup
  i2c_set_slave(pwm.i2c, PCA9685_I2C_ADDR);

  // Set pca9685 to sleep
  char mode_1_old;
  i2c_read_byte(pwm.i2c, PCA9685_MODE1, &mode_1_old);
  char mode_1_new = (mode_1_old & 0x7F) | 0x10;
  i2c_write_byte(pwm.i2c, PCA9685_MODE1, mode_1_new);

  // Set pwm prescaler
  float prescale = (25000000 / (4096.0 * freq)) - 1;
  prescale = floor(prescale + 0.5);
  // LOG_INFO("prescale: %d", (int) prescale);
  i2c_write_byte(pwm.i2c, PCA9685_PRE_SCALE, (int) prescale);
  i2c_write_byte(pwm.i2c, PCA9685_MODE1, mode_1_old);

  // Wait for oscillator
  usleep(PCA9685_WAIT_MS);

  // Reset
  i2c_write_byte(pwm.i2c, PCA9685_MODE1, mode_1_old | (1 << 7));
}

void pca9685_set_pwm(const pca9685_t &pwm,
                     const int8_t channel,
                     const int16_t off) {
  i2c_set_slave(pwm.i2c, PCA9685_I2C_ADDR);
  i2c_write_byte(pwm.i2c, PCA9685_LED0_ON_L + (4 * channel), 0 & 0xFF);
  i2c_write_byte(pwm.i2c, PCA9685_LED0_ON_H + (4 * channel), 0 >> 8);
  i2c_write_byte(pwm.i2c, PCA9685_LED0_OFF_L + (4 * channel), off & 0xFF);
  i2c_write_byte(pwm.i2c, PCA9685_LED0_OFF_H + (4 * channel), off >> 8);
}

void pca9685_set_pwm(const pca9685_t &pwm, const int16_t off) {
  i2c_set_slave(pwm.i2c, PCA9685_I2C_ADDR);
  i2c_write_byte(pwm.i2c, PCA9685_ALL_LED_ON_L, 0 & 0xFF);
  i2c_write_byte(pwm.i2c, PCA9685_ALL_LED_ON_H, 0 >> 8);
  i2c_write_byte(pwm.i2c, PCA9685_ALL_LED_OFF_L, off & 0xFF);
  i2c_write_byte(pwm.i2c, PCA9685_ALL_LED_OFF_H, off >> 8);
}

void pca9685_reset(const pca9685_t &pwm) {
  i2c_set_slave(pwm.i2c, 0x00);
  i2c_write_raw_byte(pwm.i2c, 0x06);
  usleep(PCA9685_WAIT_MS);
}

} // namespace proto
