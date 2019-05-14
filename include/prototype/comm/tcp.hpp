#ifndef PROTOTYPE_COMM_TCP_HPP
#define PROTOTYPE_COMM_TCP_HPP

#include <stdlib.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>

#include <vector>
#include <string>

#include "prototype/core/log.hpp"

namespace proto {

/**
 * TCP server
 */
struct tcp_server {
  int port = 8080;
  int sockfd = -1;
  std::vector<int> conns;

  tcp_server(int port_=8080);
};

/**
 * TCP client
 */
struct tcp_client{
  std::string server_ip;
  int server_port = 8080;
  int sockfd = -1;

  tcp_client(const std::string &server_ip_, int server_port_=8080);
};

/**
 * Configure TCP server
 */
int tcp_server_config(int port=8080);

/**
 * Configure TCP client
 */
int tcp_client_config(const std::string &ip, const int port=8080);

} //  namespace proto
#endif // PROTOTYPE_COMM_TCP_HPP
