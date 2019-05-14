#ifndef PROTOTYPE_DRIVER_UART_HPP
#define PROTOTYPE_DRIVER_UART_HPP

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

#include "prototype/core/core.hpp"

namespace proto {

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
 * Write packet to UART
 *
 * @param[in] uart UART
 * @param[in] payload Payload
 * @param[in] length Payload length
 * @returns 0 or -1 for success or failure
 */
int uart_write(const uart_t &uart,
               const uint8_t *payload,
               const size_t length);

/**
 * Read packet from UART
 *
 * @param[in] uart UART
 * @param[in,out] payload Payload
 * @param[in] length Payload length
 * @returns 0 or -1 for success or failure
 */
int uart_read(const uart_t &uart, uint8_t *payload, const size_t length);

} //  namespace proto
#endif // PROTOTYPE_DRIVER_UART_HPP
