#include <cstdlib>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <netinet/in.h>
#include <cstring>
#include <netdb.h>
#include <arpa/inet.h>

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

  struct sockaddr_in remaddr;
  memset((char *) &remaddr, 0, sizeof(remaddr));
  remaddr.sin_family = AF_INET;
  remaddr.sin_port = htons(kPort);
  if (inet_aton("127.0.0.1", &remaddr.sin_addr)==0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }

  constexpr size_t BUFLEN = 1024;
  socklen_t address_length = sizeof(struct sockaddr_in);
  char buf[BUFLEN];
  ssize_t recvlen;
  for (int i=0; i < 10; i++) {
    sprintf(buf, "This is packet %d", i);
    if (sendto(socket_fd, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, address_length)==-1) {
      perror("sendto");
      exit(1);
    }
    /* now receive an acknowledgement from the server */
    recvlen = recvfrom(socket_fd, buf, BUFLEN, 0, (struct sockaddr *)&remaddr, &address_length);
    if (recvlen >= 0) {
      buf[recvlen] = 0;	/* expect a printable string - terminate it */
      printf("received message: \"%s\"\n", buf);
    }
  }
  close(socket_fd);
  return 0;

}

void show_help() {
  std::cout << "USAGE: ./udp_client hostname"
            << std::endl
            << std::endl
            << "Example: ./udp_client tegra-ubuntu.local"
            << std::endl;
}
