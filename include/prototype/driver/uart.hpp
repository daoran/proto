/**
 * @file
 * @ingroup driver
 */
#ifndef PROTOTYPE_DRIVER_UART_HPP
#define PROTOTYPE_DRIVER_UART_HPP

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

namespace prototype {
/**
 * @addtogroup driver
 * @{
 */

class UART {
public:
  bool connected = false;
  int connection = -1;
  std::string port;
  int speed;
  int parity = 0;
  int blocking = 1;

  UART();
  UART(const std::string &port, const int speed);

  /**
   * Connect
   */
  int connect();

  /**
   * Disconnect
   */
  int disconnect();

  /**
   * Set interface attributes
   */
  int setInterfaceAttributes(const int speed, const int parity);

  /**
   * Set blocking
   */
  void setBlocking(const int blocking);
};

int set_interface_attribs(int fd, int speed, int parity);
void set_blocking(int fd, int should_block);

/** @} group driver */
} //  namespace prototype
#endif // PROTOTYPE_DRIVER_UART_HPP
