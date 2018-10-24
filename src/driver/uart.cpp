#include "prototype/driver/uart.hpp"

namespace prototype {

UART::UART() : port{port}, speed{speed} {}

int UART::connect() {
  this->connection = open(this->port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (this->connection < 0) {
    std::cout << "failed to connect to UART!" << std::endl;
    return -1;
  }

  return 0;
}

int UART::setInterfaceAttributes(int speed, int parity) {
  // Setup
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  // Set speed
  if (tcgetattr(this->connection, &tty) != 0) {
    std::cout << "error " << errno << " from tcgetattr!" << std::endl;
    return -1;
  }
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // Set parity
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK; // disable break processing
  tty.c_lflag = 0;        // no signaling chars, no echo, no canonical processing
  tty.c_oflag = 0;     // no remapping, no delays
  tty.c_cc[VMIN] = 0;  // read doesn't block
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls, enable reading
  tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(this->connection, TCSANOW, &tty) != 0) {
    std::cout << "error " << errno << " from tcsetattr!" << std::endl;
    return -1;
  }

  return 0;
}

void UART::setBlocking(int should_block) {
  // Setup
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  // Get terminal attributes
  if (tcgetattr(this->connection, &tty) != 0) {
    std::cout << "error " << errno << " from tggetattr!" << std::endl;
    return;
  }

  // Set terminal attributes - blocking
  tty.c_cc[VMIN] = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
  if (tcsetattr(this->connection, TCSANOW, &tty) != 0) {
    std::cout << "error " << errno << " setting term attributes!" << std::endl;
  }
}

} //  namespace prototype
