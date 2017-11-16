#include <phil/udp.h>
#include <cstdlib>
#include <iostream>
#include <cstring>
#include <netdb.h>

namespace phil {

UDPClient::UDPClient(const std::string &tk1_hostname) : tk1_hostname(tk1_hostname) {

  if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    std::cerr << "socket failed: [" << strerror(errno) << "]" << std::endl;
    return;
  }

  struct sockaddr_in client_addr = {0};
  client_addr.sin_family = AF_INET;
  client_addr.sin_port = htons(0);
  client_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(socket_fd, (struct sockaddr *) &client_addr, sockaddr_size) < 0) {
    std::cerr << "bind failed: [" << strerror(errno) << "]" << std::endl;
    return;
  }

  struct timeval tv = {};
  tv.tv_sec = 0;
  tv.tv_usec = 1000000; // 1 second
  if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    std::cerr << "setting socket timeout failed : [" << strerror(errno) << "]" << std::endl;
  }

  // look up hostname of the tk1
  struct hostent *hp;
  hp = gethostbyname(tk1_hostname.c_str());
  if (hp == nullptr) {
    std::cerr << "gethostbyname failed: [" << strerror(errno) << "]" << std::endl;
    return;
  }

  char *host_ip = hp->h_addr_list[0];
  printf("Found %s at IP %d.%d.%d.%d\n", tk1_hostname.c_str(), host_ip[0], host_ip[1], host_ip[2], host_ip[3]);

  struct sockaddr_in server_addr = {0};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(kPort);
  memcpy((void *) &server_addr.sin_addr, hp->h_addr_list[0], hp->h_length);
}

data_t UDPClient::Transaction(data_t data) {
  struct sockaddr_in response_addr = {0};

  if (sendto(socket_fd, (uint8_t *) &data, data_t_size, 0, (struct sockaddr *) &server_addr, sockaddr_size)
      < 0) {
    std::cerr << "sendto failed: [" << strerror(errno) << "]" << std::endl;
  }

  ssize_t recvlen =
      recvfrom(socket_fd, (uint8_t *) &data, data_t_size, 0, (struct sockaddr *) &response_addr,
               const_cast<socklen_t *>(&sockaddr_size));

  if (recvlen != data_t_size) {
    fprintf(stderr, "received %zd bytes, expected %zu bytes\n", recvlen, data_t_size);
    return data_t{0};
  } else {
    return data;
  }
}

void UDPClient::RawTransaction(uint8_t *request, size_t request_size, uint8_t *response, size_t response_size) {
  struct sockaddr_in response_addr = {0};

  if (sendto(socket_fd, request, request_size, 0, (struct sockaddr *) &server_addr, sockaddr_size)
      < 0) {
    std::cerr << "sendto failed: [" << strerror(errno) << "]" << std::endl;
  }

  // user is not expecting a response, so don't bother.
  if (response == nullptr) {
    return;
  }

  ssize_t recvlen =
      recvfrom(socket_fd, response, response_size, 0, (struct sockaddr *) &response_addr,
               const_cast<socklen_t *>(&sockaddr_size));
}

} // end namespace
