#include <iostream>
#include <errno.h>
#include <cstring>
#include <fstream>
#include <cscore.h>
#include <opencv2/highgui.hpp>

#include <phil/common/udp.h>
#include <phil/common/args.h>

int main(int argc, const char **argv) {
  args::ArgumentParser parser("This program records camera frames and their timestamps",
                              "This program is meant to run on TK1 during Motion Capture"
                                  "recording tests. However, it is general purpose and could be used on a laptop."
                                  "It cannot be built for the RoboRIO.");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::ValueFlag<std::string>
      server_hostname_flag(parser, "hostname", "hostname of the server sending start/stop", {'o'});

  try
  {
    parser.ParseCLI(argc, argv);
  }
  catch (args::Help &e)
  {
    std::cout << parser;
    return 0;
  }

  cs::UsbCamera camera{"usbcam", 0};
  camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 30);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);
  cs::CvSink sink{"sink"};
  sink.SetSource(camera);

  cv::Mat frame;
  cv::VideoWriter video("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 28, cv::Size(320, 240));

  std::ofstream time_stamps_file;
  time_stamps_file.open("frame_time_stamps.csv");

  if (!time_stamps_file.good()) {
    std::cerr << "Time stamp file failed to open: " << strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }

  std::string hostname = "phil-main.local";
  if (server_hostname_flag) {
    hostname = args::get(server_hostname_flag);
  }

  // wait for UDP message to start
  phil::UDPServer udp_server;
  uint8_t message = 0;
  udp_server.Read(&message, 1);

  std::cout << "Starting recording" << std::endl;

  struct timeval timeout = {0};
  timeout.tv_usec = 10;
  timeout.tv_sec = 0;
  udp_server.SetTimeout(timeout);

  for (int i = 0; i < 1000; i++) {
    uint64_t time = sink.GrabFrame(frame);
    if (time == 0) {
      std::cout << "error: " << sink.GetError() << std::endl;
      continue;
    }

    video.write(frame);
    time_stamps_file << time << std::endl;

    // check for UDP message to stop
    ssize_t bytes_received = udp_server.Read(&message, 1);
    if (bytes_received > 0) {
      break;
    }
  }

  std::cout << "Stopping recording" << std::endl;
  time_stamps_file.close();

  return EXIT_SUCCESS;
}
