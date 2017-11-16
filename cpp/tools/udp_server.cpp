#include <cstdlib>
#include <sys/time.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <netinet/in.h>
#include <cstring>

#include <phil/udp.h>

using namespace phil;

int main(int argc, char *argv[]) {
  int socket_fd;
  if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    std::cerr << "socket failed: [" << strerror(errno) << "]" << std::endl;
    return EXIT_FAILURE;
  }

  struct sockaddr_in addr = {0};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(kPort);

  if (bind(socket_fd, (struct sockaddr *) &addr, sockaddr_size) < 0) {
    std::cerr << "bind failed: [" << strerror(errno) << "]" << std::endl;
    return EXIT_FAILURE;
  }

  struct sockaddr_in remote_addr = {0};
  bool done = false;
  struct timeval server_time = {};
  data_t data;
  while (!done) {
    ssize_t recvlen =
        recvfrom(socket_fd, (uint8_t *) &data, data_t_size, 0, (struct sockaddr *) &remote_addr, &sockaddr_size);

    if (recvlen != data_t_size) {
      printf("received %zd bytes, expected %zu bytes\n", recvlen, data_t_size);
    }

    gettimeofday(&server_time, nullptr);
    data.tk1_recv_time_s = timeval_to_sec(server_time);
    if (sendto(socket_fd, (uint8_t *) &data, data_t_size, 0, (struct sockaddr *) &remote_addr, sockaddr_size)
        < 0) {
      std::cerr << "sendto failed: [" << strerror(errno) << "]" << std::endl;
    }
  }

  close(socket_fd);

  return EXIT_SUCCESS;
}