#include <cstdlib>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <netinet/in.h>
#include <cstring>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/time.h>

#include "udp.h"

void show_help();

int main(int argc, char *argv[]) {

  if (argc != 2) {
    show_help();
    return EXIT_FAILURE;
  }
  char *hostname = argv[1];

  int socket_fd;
  if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    std::cerr << "socket failed: [" << strerror(errno) << "]" << std::endl;
    return EXIT_FAILURE;
  }

  socklen_t address_length = sizeof(struct sockaddr_in);

  struct sockaddr_in client_address = {0};
  client_address.sin_family = AF_INET;
  client_address.sin_port = htons(0);
  client_address.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(socket_fd, (struct sockaddr *) &client_address, address_length) < 0) {
    std::cerr << "bind failed: [" << strerror(errno) << "]" << std::endl;
    return EXIT_FAILURE;
  }

  struct timeval tv = {};
  tv.tv_sec = 0;
  tv.tv_usec = 500000;
  if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    std::cerr << "setting socket timeout failed : [" << strerror(errno) << "]" << std::endl;
  }

  // look up hostname
  struct hostent *hp;
  hp = gethostbyname(hostname);
  if (hp == nullptr) {
    std::cerr << "gethostbyname failed: [" << strerror(errno) << "]" << std::endl;
    return EXIT_FAILURE;
  }

  char *host_ip = hp->h_addr_list[0];
  printf("Found %s at IP %d.%d.%d.%d\n", hostname, host_ip[0], host_ip[1], host_ip[2], host_ip[3]);

  struct sockaddr_in server_address = {0};
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(kPort);
  memcpy((void *) &server_address.sin_addr, hp->h_addr_list[0], hp->h_length);

  struct sockaddr_in response_address = {0};

  for (size_t i = 0; i < 10; ++i) {
    usleep(20000);

    data_t data = {};
    data.left_encoder_rate = 1;
    data.right_encoder_rate = 2;
    struct timeval tv = {};
    gettimeofday(&tv, nullptr);
    data.rio_send_time_s = timeval_to_sec(tv);

    if (sendto(socket_fd, (uint8_t *) &data, data_t_size, 0, (struct sockaddr *) &server_address, address_length) < 0) {
      std::cerr << "sendto failed: [" << strerror(errno) << "]" << std::endl;
    }

    ssize_t recvlen =
        recvfrom(socket_fd, (uint8_t *) &data, data_t_size, 0, (struct sockaddr *) &response_address, &address_length);

    if (recvlen != data_t_size) {
      printf("received %zd bytes, expected %zu bytes\n", recvlen, data_t_size);
    } else {
      print_data_t(data);
    }
  }

  close(socket_fd);

  return EXIT_SUCCESS;
}

void show_help() {
  std::cout << "USAGE: ./udp_client hostname"
            << std::endl
            << std::endl
            << "Example: ./udp_client phil-tk1.local"
            << std::endl;
}