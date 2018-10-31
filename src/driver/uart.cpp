#include "prototype/driver/uart.hpp"

namespace prototype {

int uart_connect(uart_t &uart) {
  uart.connection = open(uart.port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (uart.connection < 0) {
    return -1;
  }
  uart.connected = true;

  return 0;
}

int uart_disconnect(uart_t &uart) {
  if (close(uart.connection) != 0) {
    return -1;
  }
  uart.connected = false;
  uart.connection = -1;

  return 0;
}

int uart_configure(const uart_t &uart, const int speed, const int parity) {
  // Setup
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  // Set speed
  if (tcgetattr(uart.connection, &tty) != 0) {
    LOG_ERROR("Error %d from tcgetattr!", errno);
    return -1;
  }
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // Set parity
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK; // disable break processing
  tty.c_lflag = 0;     // no signaling chars, no echo, no canonical processing
  tty.c_oflag = 0;     // no remapping, no delays
  tty.c_cc[VMIN] = 0;  // read doesn't block
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls, enable reading
  tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(uart.connection, TCSANOW, &tty) != 0) {
    LOG_ERROR("Error %d from tcsetattr!", errno);
    return -1;
  }

  return 0;
}

int uart_set_blocking(const uart_t &uart, const bool blocking) {
  // Setup
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  // Get terminal attributes
  if (tcgetattr(uart.connection, &tty) != 0) {
    LOG_ERROR("Error %d from tcgetattr!", errno);
    return -1;
  }

  // Set terminal attributes - blocking
  tty.c_cc[VMIN] = blocking ? 1 : 0;
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
  if (tcsetattr(uart.connection, TCSANOW, &tty) != 0) {
    LOG_ERROR("Error %d from tcsetattr!", errno);
    return -1;
  }

  return 0;
}

} //  namespace prototype
