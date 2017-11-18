#include <iostream>

#include <phil/common/udp.h>

int main(int argc, char **argv) {

  phil::UDPClient client("phil-tk1.local");
  uint8_t request = 1;
  client.RawTransaction(&request, 1, nullptr, 1);

  return 0;
}
