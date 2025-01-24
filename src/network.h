#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>

#include "macros.h"
#include "logging.h"
#include "data.h"

/**
 * TCP server
 */
typedef struct tcp_server_t {
  int port;
  int sockfd;
  int conn;
  void *(*conn_handler)(void *);
} tcp_server_t;

/**
 * TCP client
 */
typedef struct tcp_client_t {
  char server_ip[1024];
  int server_port;
  int sockfd;
  int (*loop_cb)(struct tcp_client_t *);
} tcp_client_t;

status_t ip_port_info(const int sockfd, char *ip, int *port);

status_t tcp_server_setup(tcp_server_t *server, const int port);
status_t tcp_server_loop(tcp_server_t *server);

status_t tcp_client_setup(tcp_client_t *client,
                        const char *server_ip,
                        const int server_port);
status_t tcp_client_loop(tcp_client_t *client);
