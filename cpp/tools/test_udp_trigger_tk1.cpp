#include <iostream>

#include <phil/common/udp.h>
#include <phil/common/args.h>

int main(int argc, const char **argv) {
  args::ArgumentParser parser("This program send a udp test message",
                              "Meant for debugging or triggering mocap_record without the RIO."
                                  " It literally just sends the value 1, and it's a 1 byte message."
                                  "It can be built for the RoboRIO.");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::ValueFlag<std::string>
      server_hostname_flag(parser, "hostname", "hostname of the client to send to", {'o'});

  try
  {
    parser.ParseCLI(argc, argv);
  }
  catch (args::Help &e)
  {
    std::cout << parser;
    return 0;
  }
  std::string hostname = "phil-main.local";
  if (server_hostname_flag) {
    hostname = args::get(server_hostname_flag);
  }

  phil::UDPClient client(hostname);
  uint8_t request = 1;
  client.RawTransaction(&request, 1, nullptr, 1);

  return 0;
}
