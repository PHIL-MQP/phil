#include <cstring>
#include <iostream>

#include <aruco/aruco.h>
#include <cscore.h>
#include <marker_mapper/markermapper.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/opencv.hpp>

#include <phil/common/common.h>
#include <phil/common/udp.h>
#include <phil/localization/ekf.h>

/**
 * The main program that runs on the TK1. Receives sensor data from the camera and the RoboRIO and performs localization
 */
int main(int argc, char **argv) {

  cs::UsbCamera camera{"usbcam", 0};
  constexpr int w = 1280;
  constexpr int h = 720;
  constexpr int fps = 30;
  camera.SetVideoMode(cs::VideoMode::kMJPEG, w, h, fps);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);
  cs::CvSink sink{"sink"};
  sink.SetSource(camera);

  std::cout << phil::cyan << "Go to localhost:8082 to see annotated camera stream" << phil::reset << std::endl;
  cs::CvSink cvsink{"cvsink"};
  cvsink.SetSource(camera);
  cs::CvSource cvsource{"cvsource", cs::VideoMode::kMJPEG, w, h, fps};
  cs::MjpegServer cvMjpegServer{"cvhttpserver", 8082};
  cvMjpegServer.SetSource(cvsource);

  // read in the markermapper config yaml file
  std::cout << "Loading MarkerMap from map.yml" << std::endl;
  aruco::MarkerMap mmap;
  try {
    mmap.readFromFile("map.yml");
  }
  catch (cv::Exception &e) {
    std::cerr << phil::red
              << "Failed to open map.yml"
              << phil::reset
              << "\n"
              << e.what()
              << "\n";
    return EXIT_FAILURE;
  }
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
  try {
    camera_params.readFromXMLFile("cam_params.yml");
  }
  catch (std::runtime_error &e) {
    std::cerr << phil::red
              << "Failed to open cam_params.yml"
              << phil::reset
              << "\n"
              << e.what()
              << "\n";
    return EXIT_FAILURE;
  }
  if (!camera_params.isValid()) {
    std::cout << phil::red
              << "Invalid camera parameters"
              << phil::reset
              << std::endl;
    return EXIT_FAILURE;
  }
  camera_params.resize(cv::Size(w, h));

  // create pose tracker
  aruco::MarkerMapPoseTracker tracker;
  tracker.setParams(camera_params, mmap);

  // communication with the roborio
  phil::UDPServer server(phil::kPort);
  struct timeval timeout{0};
  timeout.tv_sec = 0;
  timeout.tv_usec = 50000; // 20ms
  server.SetTimeout(timeout);

  // network tables
  auto inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<NetworkTable> table;
  table = inst.GetTable(phil::kTableName);

  // Create the EKF
  phil::EKF ekf;

  cv::Mat frame;
  bool done = false;
  std::cout << phil::green << "Beginning Localization loop" << phil::reset << std::endl;
  while (!done) {
    // read some sensor data from roborio
    phil::data_t rio_data = {0};
    ssize_t bytes_received = server.Read(reinterpret_cast<uint8_t *>(&rio_data), phil::data_t_size);

    if (bytes_received == -1) {
      long now = std::chrono::system_clock::now().time_since_epoch().count();
      std::cout << now << " " << phil::yellow << "No data from RoboRIO" << phil::reset << std::endl;
    } else if (bytes_received != phil::data_t_size) {
      std::cerr << phil::red << "bytes does not match data_t_size: [" << strerror(errno) << "]" << phil::reset
                << std::endl;
    } else {
      MatrixWrapper::ColumnVector encoder_input(2);
      encoder_input(1) = rio_data.left_encoder_rate;
      encoder_input(2) = rio_data.right_encoder_rate;

      MatrixWrapper::ColumnVector yaw_measurement(1);
      yaw_measurement << rio_data.yaw;

      MatrixWrapper::ColumnVector acc_measurement(2);
      acc_measurement << rio_data.world_accel_x, rio_data.world_accel_y;

      ekf.filter->Update(ekf.encoder_system_model.get(), encoder_input);
      ekf.filter->Update(ekf.yaw_measurement_model.get(), yaw_measurement);
      ekf.filter->Update(ekf.acc_measurement_model.get(), acc_measurement);
    }

    // only wait briefly for camera frame.
    // if it's not available that's fine we'll just not call the EKF update for camera data
    uint64_t time = sink.GrabFrame(frame, 0.001);
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
          // We got a fully valid pose estimate from our camera frame
          MatrixWrapper::ColumnVector camera_measurement(3);
          auto camera_pose = phil::MatrixTo3Pose(rt_matrix);
          camera_measurement << camera_pose.x, camera_pose.y, camera_pose.theta;

          // perform camera EKF update
          ekf.filter->Update(ekf.camera_measurement_model.get(), camera_measurement);
        }

        // annotate the video feed
        for (int idx : mmap.getIndices(detected_markers)) {
          detected_markers[idx].draw(annotated_frame, cv::Scalar(0, 0, 255), 1);
        }
      } else {
        std::cerr << "Invalid marker map pose tracker\n";
      }

      // and run update step of EKF with just the camera data

      // show annotated frame. It's useful to debugging/visualizing
      cvsource.PutFrame(annotated_frame);
    }

    // TODO: read serial data from PSoC

    // Fill our pose struct from the belief state of the EKF
    phil::pose_t pose{};
    auto estimate = ekf.filter->PostGet()->ExpectedValueGet();
    pose.x = estimate(1);
    pose.y = estimate(2);
    pose.theta = estimate(3);

    table->PutNumberArray(phil::kPoseKey, llvm::ArrayRef<double>({pose.x, pose.y, pose.theta}));
  }

  return EXIT_FAILURE;
}
