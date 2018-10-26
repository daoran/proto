#include "prototype/driver/i2c.hpp"

namespace prototype {

int i2c_setup(i2c_t &i2c) {
  int fd;
  int adapter_nr;
  char filename[20];

  // Setup
  adapter_nr = 1; // probably dynamically determined
  memset(filename, '\0', sizeof(char) * 20);
  snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);

  // Open i2c connection
  fd = open(filename, O_RDWR);
  if (fd < 0) {
    return -1;
  } else {
    i2c.fd = fd;
  }

  return 0;
}

int i2c_set_slave(const i2c_t &i2c, const char slave_addr) {
  return ioctl(i2c.fd, I2C_SLAVE, slave_addr);
}

int i2c_read_byte(const i2c_t &i2c, const char reg_addr, char *data) {
  char buf[1];

  buf[0] = reg_addr;
  if (write(i2c.fd, buf, 1) != 1) {
    return -1;
  }

  if (read(i2c.fd, data, 1) != 1) {
    return -2;
  }

  return 0;
}

int i2c_read_bytes(const i2c_t &i2c, const char reg_addr, char *data, size_t length) {
  char buf[1];

  buf[0] = reg_addr;
  if (write(i2c.fd, buf, 1) != 1) {
    return -1;
  }

  if (read(i2c.fd, data, length) != (int) length) {
    return -2;
  }

  return 0;
}

int i2c_write_byte(const i2c_t &i2c, const char reg_addr, const char byte) {
  char buf[2];

  buf[0] = reg_addr;
  buf[1] = byte;
  if (write(i2c.fd, buf, 2) != 1) {
    return -1;
  }

  return 0;
}

int i2c_write_raw_byte(const i2c_t &i2c, const char byte) {
  if (write(i2c.fd, &byte, 1) != 1) {
    return -1;
  }

  return 0;
}

int i2c_write_bytes(const i2c_t &i2c, const char reg_addr, const char *data, const size_t length) {
  int i;
  char buf[I2C_BUF_MAX];

  // Create buf
  memset(buf, '\0', sizeof(char) * I2C_BUF_MAX);
  buf[0] = reg_addr;
  for (i = 1; i < (int) length + 1; i++) {
    buf[i] = data[i];
  }

  // Write bytes
  if (write(i2c.fd, buf, length + 1) != 1) {
    return -1;
  }

  return 0;
}

} //  namespace prototype
