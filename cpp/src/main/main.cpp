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
  constexpr int w = 320;
  constexpr int h = 320;
  constexpr int fps = 320;
  camera.SetVideoMode(cs::VideoMode::kMJPEG, w, h, fps);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);
  cs::CvSink sink{"sink"};
  sink.SetSource(camera);

  std::cout << "Go to localhost:8082 to see annotated camera stream" << std::endl;
  cs::CvSink cvsink{"cvsink"};
  cvsink.SetSource(camera);
  cs::CvSource cvsource{"cvsource", cs::VideoMode::kMJPEG, w, h ,fps};
  cs::MjpegServer cvMjpegServer{"cvhttpserver", 8082};
  cvMjpegServer.SetSource(cvsource);

  // read in the markermapper config yaml file
  std::cout << "Loading MarkerMap from map.yml" << std::endl;
  aruco::MarkerMap mmap;
  mmap.readFromFile("map.yml");
  mmap.setDictionary("ARUCO_MIP_16h3");

  // convert to meters if necessary
  if (mmap.isExpressedInPixels()) {
    mmap = mmap.convertToMeters(0.02);
  }

  // marker detector is needed for passing detected markers to the tracker
  aruco::MarkerDetector detector;
  detector.setDictionary("ARUCO_MIP_16h3");

  // load camera params
  std::cout << "Loading Camera Params from cam_params.yml" << std::endl;
  aruco::CameraParameters camera_params;
  camera_params.readFromXMLFile("cam_params.yml");
  camera_params.resize(cv::Size(w, h));

  // create pose tracker
  aruco::MarkerMapPoseTracker tracker;
  tracker.setParams(camera_params, mmap);

  // comms with the roborio
  phil::UDPServer server(phil::kPort);
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

    // only wait for 10ms for camera frame.
    // if it's not available that's fine we'll just not call the EKF update for camera data
    uint64_t time = sink.GrabFrame(frame, 0.010);
    if (time > 0) {
      cv::Mat annotated_frame;
      frame.copyTo(annotated_frame);

      // update step for camera measurement
      std::vector<aruco::Marker> detected_markers = detector.detect(frame);

      // estimate 3d camera pose if possible
      if (tracker.isValid()) {
        // estimate the pose of the camera with respect to the detected markers
        if (tracker.estimatePose(detected_markers)) {
          cv::Mat rt_matrix = tracker.getRTMatrix();
        }

        // annotate the video feed
        for (int idx : mmap.getIndices(detected_markers)) {
          detected_markers[idx].draw(annotated_frame, cv::Scalar(0, 0, 255), 1);
        }
      }

      // and run update step of EKF with just the camera data

      // show annotated frame. It's useful to debugging/visualizing
      cvsource.PutFrame(annotated_frame);
    }

    // TODO: read serial data from PSoC

    // do localization
    double time_s = time / 1e6; // convert micoseconds to seconds
    phil::pose_t pose = phil::compute_pose(time_s, frame, rio_data);

    table->PutNumberArray(phil::kPoseKey, llvm::ArrayRef<double>({pose.x, pose.y, pose.theta}));
  }

  return -1;
}
