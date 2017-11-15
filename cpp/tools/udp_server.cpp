#include <cstdlib>
#include <sys/time.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <netinet/in.h>
#include <cstring>

#include "udp.h"

int main(int argc, char *argv[]) {
  int socket_fd;
  if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    std::cerr << "socket failed: [" << strerror(errno) << "]" << std::endl;
    return EXIT_FAILURE;
  }

  socklen_t address_length = sizeof(struct sockaddr_in);

  struct sockaddr_in addr;
  memset((char *)&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(kPort);

  if (bind(socket_fd, (struct sockaddr *) &addr, address_length) < 0) {
    std::cerr << "bind failed: [" << strerror(errno) << "]" << std::endl;
    return EXIT_FAILURE;
  }

  uint8_t buf[kBufferSize];
  struct sockaddr_in remote_address;
  bool done = false;
  struct timeval server_time;
  while (!done) {
    ssize_t recvlen = recvfrom(socket_fd, buf, kBufferSize, 0, (struct sockaddr *) &remote_address, &address_length);

    printf("received %zd bytes\n", recvlen);
    if (recvlen > 0) {
      gettimeofday(&server_time, nullptr);
      char *time_bytes = reinterpret_cast<char *>(&server_time);
      printf("received message: \"%s\". Responding with server timestamp\n", buf);
      if (sendto(socket_fd, time_bytes, sizeof(struct timeval), 0, (struct sockaddr *) &remote_address, address_length)
          < 0) {
        std::cerr << "sendto failed: [" << strerror(errno) << "]" << std::endl;
      }
    }
  }

  close(socket_fd);

  return EXIT_SUCCESS;
}