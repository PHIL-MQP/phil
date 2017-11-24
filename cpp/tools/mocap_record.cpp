#include <iostream>
#include <stdio.h>
#include <errno.h>
#include <cstring>
#include <fstream>
#include <cscore.h>
#include <opencv2/highgui.hpp>

#include <phil/common/udp.h>

int main(int argc, char **argv) {
  cs::UsbCamera camera{"usbcam", 0};
  camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 30);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);
  cs::CvSink sink{"sink"};
  sink.SetSource(camera);

  cv::Mat frame;
  cv::VideoWriter video("out.mjpeg", CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(320, 240));

  std::ofstream time_stamps_file;
  time_stamps_file.open("frame_time_stamps.csv");

  if (!time_stamps_file.good()) {
    std::cerr << "Time stamp file failed to open: " << strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }

  // wait for UDP message to start
  phil::UDPClient udp_client("phil-tk1.local");
  uint8_t message = 0;
  udp_client.Read(&message, 1);

  struct timeval timeout = {0};
  timeout.tv_usec = 10;
  timeout.tv_sec = 0;
  udp_client.SetTimeout(timeout);

  for (int i=0; i < 1000; i++) {
    uint64_t time = sink.GrabFrame(frame);
    if (time == 0) {
      std::cout << "error: " << sink.GetError() << std::endl;
      continue;
    }

    video.write(frame);
    time_stamps_file << time << std::endl;

    // check for UDP message to stop
    int bytes_received = udp_client.Read(&message, 1);
    if (bytes_received > 0) {
      break;
    }
  }

  time_stamps_file.close();

  return EXIT_SUCCESS;
}
