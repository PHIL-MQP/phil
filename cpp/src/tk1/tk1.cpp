#include <iostream>
#include <cstring>
#include <cscore.h>
#include <opencv2/core/mat.hpp>
#include "../../include/phil/common/udp.h"

int main(int argc, char **argv) {
  std::cout << "tk1 main program..." << std::endl;

  // camera server
  cs::UsbCamera camera{"usbcam", 0};
  camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 30);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);
  cs::CvSink sink{"sink"};
  sink.SetSource(camera);

  // comms with the roborio
  phil::UDPServer server;

  cv::Mat frame;
  bool done = false;
  while (!done) {
    // read some sensor data from roborio
    phil::data_t data = {0};
    ssize_t bytes_received = server.Read(reinterpret_cast<uint8_t *>(&data), phil::data_t_size);

    if (bytes_received != phil::data_t_size) {
      std::cerr << "bytes does not match data_t_sze: [" << strerror(errno) << "]" << std::endl;
      continue;
    }

    // grab latest camera frame
    uint64_t time = sink.GrabFrame(frame);
  }

  return 0;
}