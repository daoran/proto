#include "prototype/comm/tcp.hpp"

namespace proto {

tcp_server::tcp_server(int port_) : port{port_} {}

tcp_client::tcp_client(const std::string &server_ip_, int server_port_)
    : server_ip{server_ip_}, server_port{server_port_} {}

int tcp_server_config(int port) {
  // Create socket
  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd == -1) {
    LOG_ERROR("Socket creation failed...");
    return -1;
  }

  // Socket options
  const int enable = 1;
  const size_t int_sz = sizeof(int);
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEADDR) failed");
  }
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &enable, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEPORT) failed");
  }

  // Assign IP, PORT
  struct sockaddr_in server;
  bzero(&server, sizeof(server));
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = htonl(INADDR_ANY);
  server.sin_port = htons(port);

  // Bind newly created socket to given IP
  int retval = bind(sockfd,
                    (struct sockaddr *) &server,
                    sizeof(server));
  if (retval != 0) {
    LOG_ERROR("Socket bind failed...");
    LOG_ERROR("%s", strerror(errno));
    return -1;
  }

  // Server is ready to listen
  if ((listen(sockfd, 5)) != 0) {
    LOG_ERROR("Listen failed...");
    return -1;
  } else {
    DEBUG("Server running");
  }

  // Accept the data packet from client and verification
  struct sockaddr_in client;
  socklen_t len = sizeof(client);
  int client_conn = accept(sockfd, (struct sockaddr *) &client, &len);
  if (client_conn < 0) {
    LOG_ERROR("Server acccept failed...");
    return -1;
  }

  return 0;
}

int tcp_client_config(const std::string &ip, const int port) {
  // Create socket
  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd == -1) {
    LOG_ERROR("Socket creation failed!");
    return -1;
  } else {
    DEBUG("Created socket!");
  }

  // Assign IP, PORT
  struct sockaddr_in server;
  bzero(&server, sizeof(server));
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(ip.c_str());
  server.sin_port = htons(port);

  // Connect to server
  int retval = connect(sockfd,
                       (struct sockaddr *) &server,
                       sizeof(server));
  if (retval != 0) {
    LOG_ERROR("Connection with the server failed!");
    return -1;
  } else {
    DEBUG("Connected to the server!");
  }

  return 0;
}

} //  namespace proto
