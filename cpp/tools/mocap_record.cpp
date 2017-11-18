#include <iostream>
#include <stdio.h>
#include <errno.h>
#include <cstring>
#include <fstream>
#include <cscore.h>
#include <opencv2/highgui.hpp>

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

  for (int i=0; i < 300; i++) {
    uint64_t time = sink.GrabFrame(frame);
    if (time == 0) {
      std::cout << "error: " << sink.GetError() << std::endl;
      continue;
    }

    video.write(frame);
    time_stamps_file << time << std::endl;

    // check for UDP message to stop
  }

  time_stamps_file.close();

  return EXIT_SUCCESS;
}
