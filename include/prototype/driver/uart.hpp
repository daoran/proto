#ifndef PROTOTYPE_DRIVER_UART_HPP
#define PROTOTYPE_DRIVER_UART_HPP

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

#include "prototype/core.hpp"

namespace prototype {

/**
 * UART
 */
struct uart_t {
  bool connected = false;
  int connection = -1;
  std::string port;

  int speed = 0;
  int parity = 0;
  int blocking = 1;

  uart_t() {}
  uart_t(const std::string &port_, const int speed_)
      : port{port_}, speed{speed_} {}
};

/**
 * Connect
 *
 * @param[in,out] uart UART
 * @returns 0 or -1 for success or failure
 */
int uart_connect(uart_t &uart);

/**
 * Disconnect
 *
 * @param[in,out] uart UART
 * @returns 0 or -1 for success or failure
 */
int uart_disconnect(uart_t &uart);

/**
 * Set interface attributes
 *
 * @param[in,out] uart UART
 * @param[in] speed UART speed
 * @param[in] parity UART parity
 * @returns 0 or -1 for success or failure
 */
int uart_configure(const uart_t &uart, const int speed, const int parity);

/**
 * Set blocking
 *
 * @param[in,out] uart UART
 * @param[in] blocking Blocking
 * @returns 0 or -1 for success or failure
 */
void uart_set_blocking(uart_t &uart, const bool blocking);

} //  namespace prototype
#endif // PROTOTYPE_DRIVER_UART_HPP
