#include <iostream>
#include <cstring>
#include <cscore.h>
#include <opencv2/opencv.hpp>
#include <networktables/NetworkTableInstance.h>
#include "phil/common/udp.h"
#include "phil/localization/localization.h"

int main(int argc, char **argv) {
  std::cout << "main main program..." << std::endl;

  // camera server
  cs::UsbCamera camera{"usbcam", 0};
  camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 30);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);
  cs::CvSink sink{"sink"};
  sink.SetSource(camera);

  // comms with the roborio
  phil::UDPServer server;

  // network tables
  auto inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<NetworkTable> table;
  table = inst.GetTable(phil::kTableName);

  cv::Mat frame;
  bool done = false;
  while (!done) {
    // read some sensor data from roborio
    phil::data_t rio_data = {0};
    ssize_t bytes_received = server.Read(reinterpret_cast<uint8_t *>(&rio_data), phil::data_t_size);

    if (bytes_received != phil::data_t_size) {
      std::cerr << "bytes does not match data_t_size: [" << strerror(errno) << "]" << std::endl;
      continue;
    }

    // grab latest camera frame
    uint64_t time = sink.GrabFrame(frame);

    // do localization
    double time_s = time / 1e7; // convert 100's of nanoseconds to seconds
    phil::pose_t pose = phil::compute_pose(time_s, frame, rio_data);

    table->PutNumberArray(phil::kPoseKey, llvm::ArrayRef<double>({pose.x, pose.y, pose.theta}));
  }

  return -1;
}
