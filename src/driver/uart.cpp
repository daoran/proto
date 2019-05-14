#include "prototype/driver/uart.hpp"

namespace proto {

// Source: https://stackoverflow.com/a/38318768/154688
static int set_interface_attribs(int fd, int speed) {
  struct termios tty;
  if (tcgetattr(fd, &tty) < 0) {
    printf("Error from tcgetattr: %s\n", strerror(errno));
    return -1;
  }

  cfsetospeed(&tty, (speed_t) speed);
  cfsetispeed(&tty, (speed_t) speed);

  tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;         /* 8-bit characters */
  tty.c_cflag &= ~PARENB;     /* no parity bit */
  tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
  tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

  /* Setup for non-canonical mode */
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  /* Fetch bytes as they become available */
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    printf("Error from tcsetattr: %s\n", strerror(errno));
    return -1;
  }
  return 0;
}

int uart_connect(uart_t &uart) {
  // Connect
  uart.connection = open(uart.port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (uart.connection < 0) {
    return -1;
  }
  uart.connected = true;

	// Set serial attributes
  if (set_interface_attribs(uart.connection,  uart.speed) != 0) {
    return -1;
  }

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

int uart_write(const uart_t &uart,
               const uint8_t *payload,
               const size_t length) {
  ssize_t retval = write(uart.connection, payload, length);
  if (retval != (ssize_t) length) {
    return -1;
  }

  return 0;
}

int uart_read(const uart_t &uart, uint8_t *payload, const size_t length) {
  ssize_t retval = read(uart.connection, payload, length);
  if (retval < 0) {
    return -1;
  }

  return 0;
}

} //  namespace proto
