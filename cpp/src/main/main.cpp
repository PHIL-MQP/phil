#include <iostream>
#include <cstring>
#include <cscore.h>
#include <opencv2/opencv.hpp>
#include <networktables/NetworkTableInstance.h>
#include <marker_mapper/markermapper.h>
#include <aruco/aruco.h>

#include "phil/common/udp.h"
#include "phil/localization/localization.h"

/**
 * The main program that runs on the TK1. Receives sensor data from the camera and the RoboRIO and performs localization
 */
int main(int argc, char **argv) {

  std::cout << "Starting camera sever thread" << std::endl;
  // TODO: abstract away video versus live camera data
  // TODO: add arguments for device/w/h/fps?
  cs::UsbCamera camera{"usbcam", 0};
  camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 30);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);
  cs::CvSink sink{"sink"};
  sink.SetSource(camera);

  // read in the markermapper config yaml file
  std::cout << "Loading MarkerMap from map.yml" << std::endl;
  aruco::MarkerMap mmap;
  mmap.readFromFile("map.yml");
  mmap.setDictionary("ARUCO_MIP_16h3");

  // comms with the roborio
  phil::UDPServer server;
  struct timeval timeout{0};
  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;
  server.SetTimeout(timeout);

  // network tables
  auto inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<NetworkTable> table;
  table = inst.GetTable(phil::kTableName);

  cv::Mat frame;
  bool done = false;
  std::cout << "Beginning Localization loop" << std::endl;
  while (!done) {
    // read some sensor data from roborio
    phil::data_t rio_data = {0};
    ssize_t bytes_received = server.Read(reinterpret_cast<uint8_t *>(&rio_data), phil::data_t_size);

    if (bytes_received != phil::data_t_size) {
      std::cerr << "bytes does not match data_t_size: [" << strerror(errno) << "]" << std::endl;
    }

    // and run update step of EKF with just the camera data

    // only wait for 10ms for camera frame.
    // if it's not available that's fine we'll just not call the EKF update for camera data
    uint64_t time = sink.GrabFrame(frame, 0.010);
    if (time > 0) {
      // update step for camera measurement
    }

    // TODO: read serial data

    // do localization
    double time_s = time / 1e6; // convert micoseconds to seconds
    phil::pose_t pose = phil::compute_pose(time_s, frame, rio_data);

    table->PutNumberArray(phil::kPoseKey, llvm::ArrayRef<double>({pose.x, pose.y, pose.theta}));
  }

  return -1;
}
