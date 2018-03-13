#include <iostream>
#include <cstring>
#include <netdb.h>

#include <phil/common/udp.h>

namespace phil {

socklen_t sockaddr_size = sizeof(struct sockaddr_in);

UDPServer::UDPServer(const int16_t port_num) {
  if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    std::cerr << "socket failed: [" << strerror(errno) << "]" << std::endl;
    return;
  }

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port_num);

  if (bind(socket_fd, (struct sockaddr *) &addr, sockaddr_size) < 0) {
    std::cerr << "bind failed: [" << strerror(errno) << "]" << std::endl;
    return;
  }

}

ssize_t UDPServer::Read() {
  uint8_t scrap;
  return recvfrom(socket_fd, &scrap, 1, 0, nullptr, &sockaddr_size);
}

std::pair<ssize_t, struct sockaddr_in> UDPServer::Read(data_t *result) {
  struct sockaddr_in remote_addr{};
  auto recvlen = recvfrom(socket_fd, reinterpret_cast<uint8_t *>(result), data_t_size, 0,
                          reinterpret_cast<sockaddr *>(&remote_addr), &sockaddr_size);
  return {recvlen, remote_addr};
}

ssize_t UDPServer::Reply(struct sockaddr_in client, data_t reply) {
  return sendto(socket_fd, reinterpret_cast<uint8_t *>(&reply), data_t_size, 0,
                reinterpret_cast<const sockaddr *>(&client), sockaddr_size);
}

void UDPServer::SetTimeout(struct timeval timeout) {
  if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    std::cerr << "setting socket timeout failed : [" << strerror(errno) << "]" << std::endl;
  }
}

UDPClient::UDPClient(const std::string &server_hostname, int port_num) : server_hostname(server_hostname) {

  if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    std::cerr << "socket failed: [" << strerror(errno) << "]" << std::endl;
    return;
  }

  struct sockaddr_in client_addr{};
  client_addr.sin_family = AF_INET;
  client_addr.sin_port = htons(0);
  client_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(socket_fd, (struct sockaddr *) &client_addr, sockaddr_size) < 0) {
    std::cerr << "bind failed: [" << strerror(errno) << "]" << std::endl;
    return;
  }

  // look up hostname of the main
  struct hostent *hp;
  hp = gethostbyname(server_hostname.c_str());
  if (hp == nullptr) {
    std::cerr << "gethostbyname of [" << server_hostname << "] failed." << std::endl;
    std::cerr << "Are you *sure* that's the right hostname? Trying pinging it." << std::endl;
    return;
  }

  char *host_ip = hp->h_addr_list[0];
  printf("Found %s at IP %d.%d.%d.%d\n", server_hostname.c_str(), host_ip[0], host_ip[1], host_ip[2], host_ip[3]);

  server_addr = {};
  server_addr.sin_family = AF_INET;
  // server_addr.sin_port = htons(kPort);
  server_addr.sin_port = htons(port_num);
  memcpy((void *) &server_addr.sin_addr, hp->h_addr_list[0], hp->h_length);
}

void UDPClient::SetTimeout(struct timeval timeout) {
  if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    std::cerr << "setting socket timeout failed : [" << strerror(errno) << "]" << std::endl;
  }
}

data_t UDPClient::Transaction(data_t data) {
  struct sockaddr response_addr{};

  if (sendto(socket_fd,
             reinterpret_cast<uint8_t *>(&data),
             data_t_size,
             0,
             (struct sockaddr *) &server_addr,
             sockaddr_size) < 0) {
    std::cerr << "sendto failed: [" << strerror(errno) << "]" << std::endl;
  }

  ssize_t recvlen = recvfrom(socket_fd, (uint8_t *) &data, data_t_size, 0, &response_addr, &sockaddr_size);

  if (recvlen != data_t_size) {
    fprintf(stderr, "received %zd bytes, expected %zu bytes\n", recvlen, data_t_size);
    return data_t{};
  } else {
    return data;
  }
}

ssize_t UDPClient::Read(uint8_t *response, size_t response_size) {
  struct sockaddr response_addr{};
  ssize_t recvlen = recvfrom(socket_fd, response, response_size, 0, &response_addr, &sockaddr_size);
  return recvlen;
}

void UDPClient::RawTransaction(uint8_t *request, size_t request_size, uint8_t *response, size_t response_size) {
  struct sockaddr_in response_addr{};

  if (sendto(socket_fd, request, request_size, 0, (struct sockaddr *) &server_addr, sockaddr_size) < 0) {
    std::cerr << "sendto failed: [" << strerror(errno) << "]" << std::endl;
  }

  // user is not expecting a response, so don't bother.
  if (response == nullptr) {
    return;
  }

  recvfrom(socket_fd, response, response_size, 0, reinterpret_cast<sockaddr *>(&response_addr), &sockaddr_size);
}
} // end namespace
