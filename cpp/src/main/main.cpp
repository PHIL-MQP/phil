#include <cstring>
#include <iostream>
#include <unistd.h>
#include <algorithm>

#include <aruco/aruco.h>
#include <cscore.h>
#include <eigen3/Eigen/Eigen>
#include <marker_mapper/markermapper.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <phil/common/common.h>
#include <phil/common/udp.h>
#include <phil/localization/ekf.h>
#include <phil/common/args.h>

template<typename T>
T yaml_get(const YAML::Node &node, const std::vector<std::string> &keys) {
  YAML::Node tmp = YAML::Clone(node);
  size_t idx = 0;
  for (auto &key : keys) {
    if (tmp[key]) {
      tmp = tmp[key];
      if (idx == keys.size() - 1) {
        return tmp.as<T>();
      }
    } else {
      std::cerr << phil::red << "Key [" << key << "] not found" << phil::reset << "\n";
      throw YAML::ParserException(node.Mark(), "key mispelled");
    }
    ++idx;
  }

  return {0};
}

/**
 * The main program that runs on the TK1. Receives sensor data from the camera and the RoboRIO and performs localization
 */
int main(int argc, const char **argv) {
  args::ArgumentParser parser("The main program to run on the localization co-processor");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::Positional<std::string> config_filename(parser, "config_filename", "", args::Options::Required);

  try {
    parser.ParseCLI(argc, argv);
  }
  catch (args::Help &e) {
    std::cout << parser;
    return 0;
  }
  catch (args::RequiredError &e) {
    std::cout << parser;
    return 0;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(args::get(config_filename));
  }
  catch (YAML::Exception &e) {
    std::cerr << phil::red << "Failed to open config file." << phil::reset << "\n" << e.what() << "\n";
    return EXIT_FAILURE;
  }

  const auto w = yaml_get<int>(config, {"camera", "w"});
  const auto h = yaml_get<int>(config, {"camera", "h"});
  const auto fps = yaml_get<int>(config, {"camera", "fps"});
  const auto encoding = yaml_get<std::string>(config, {"camera", "encoding"});
  const auto map_filename = yaml_get<std::string>(config, {"aruco", "map"});
  const auto phil_source_url = yaml_get<std::string>(config, {"camera", "source_url"});
  const auto dictionary = yaml_get<std::string>(config, {"aruco", "dictionary"});
  const auto cam_params_file = yaml_get<std::string>(config, {"camera", "params"});

  constexpr auto hostname_length = 10;
  char hostname[hostname_length] = "localhost";
  auto failure = gethostname(hostname, hostname_length);
  if (failure) {
    std::cout << phil::yellow << "Failed to get hostname" << phil::reset << "\n";
  }

  cs::HttpCamera camera("phil/main/camera", phil_source_url);
  cs::CvSink sink("phil/main/sink");
  sink.SetSource(camera);

  constexpr int annotated_stream_port = 8777;

  cs::CvSource cvsource("phil/main/annotated_source", cs::VideoMode::kMJPEG, w, h, fps);
  cs::MjpegServer cvMjpegServer{"phil/main/annotated_mjpeg_server", annotated_stream_port};
  cvMjpegServer.SetSource(cvsource);

  std::cout << phil::cyan << "See annotated camera stream at " << hostname << ":" << annotated_stream_port
            << phil::reset << "\n";

  // read in the markermapper config yaml file
  aruco::MarkerMap mmap;
  try {
    mmap.readFromFile(map_filename);
  }
  catch (cv::Exception &e) {
    std::cerr << phil::red << "Failed to open [" << map_filename << "]" << phil::reset << "\n" << e.what() << "\n";
    return EXIT_FAILURE;
  }
  mmap.setDictionary(dictionary);

  // convert to meters if necessary
  if (mmap.isExpressedInPixels()) {
    mmap = mmap.convertToMeters(0.02);
  }

  // marker detector is needed for passing detected markers to the tracker
  aruco::MarkerDetector detector;
  detector.setDictionary(dictionary);

  // load camera params
  aruco::CameraParameters camera_params;
  try {
    camera_params.readFromXMLFile(cam_params_file);
  }
  catch (cv::Exception &e) {
    std::cerr << phil::red << "Failed to open [" << cam_params_file << "]" << phil::reset << "\n" << e.what() << "\n";
    return EXIT_FAILURE;
  }
  if (!camera_params.isValid()) {
    std::cout << phil::red << "Invalid camera parameters" << phil::reset << std::endl;
    return EXIT_FAILURE;
  }
  camera_params.resize(cv::Size(w, h));

  const auto acc_calib_params = yaml_get<std::vector<double>>(config, {"imu_calibration", "accelerometer"});

  // create pose tracker
  aruco::MarkerMapPoseTracker tracker;
  tracker.setParams(camera_params, mmap);

  // network tables
  auto inst = nt::NetworkTableInstance::GetDefault();
  const auto servername = yaml_get<std::string>(config, {"nt", "server"});
  const auto port = yaml_get<unsigned int>(config, {"nt", "port"});
  inst.StartClient(llvm::StringRef(servername), port);
  usleep(500000); // wait for connection to be established
  auto phil_table = inst.GetTable(phil::kTableName);

  // set the .type entry to ensure the shuffleboard integration works
  auto type_entry = phil_table->GetEntry(".type");
  type_entry.SetString("Phil");
  auto x_entry = phil_table->GetEntry("x");
  auto y_entry = phil_table->GetEntry("y");
  auto yaw_entry = phil_table->GetEntry("yaw");

  // Setup communication with the roborio
  phil::UDPServer server(phil::kPort);

  std::cout << phil::green << "Waiting for data from the RoboRIO" << phil::reset << std::endl;
  server.Read(nullptr, phil::data_t_size);

  std::cout << phil::green << "Collecting Initial Stationary Sample" << phil::reset << std::endl;
  constexpr size_t num_initial_samples = 250;
  Eigen::Matrix<double, num_initial_samples, 2> initial_samples(num_initial_samples, 2);
  for (size_t i = 0; i < num_initial_samples; ++i) {
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
      initial_samples(i, 0) = rio_data.world_accel_x;
      initial_samples(i, 1) = rio_data.world_accel_y;
    }
  }

  // compute the variance of our initial sample
  Eigen::Matrix<double, Eigen::Dynamic, 2> centered = initial_samples.rowwise() - initial_samples.colwise().mean();
  double variance_norm = centered.array().square().matrix().colwise().mean().norm();
  auto static_threshold = std::pow(variance_norm, 6);

  // set a timeout so we can be robust to missing RoboRIO data
  struct timeval timeout{0};
  timeout.tv_sec = 0;
  timeout.tv_usec = 40000;
  server.SetTimeout(timeout);

  phil::EKF ekf;
  cv::Mat frame;
  bool done = false;
  static double accumulated_yaw_rad = 0;
  static double last_yaw_rad = 0;
  constexpr size_t window_size = 20;
  size_t window_idx = 0;
  Eigen::Array<double, window_size, 2> window;
  Eigen::Array<double, 1, 2> biases;
  std::cout << phil::green << "Beginning Localization loop" << phil::reset << std::endl;
  while (!done) {
    // read some sensor data from roborio
    phil::data_t rio_data = {0};
    ssize_t bytes_received = server.Read(reinterpret_cast<uint8_t *>(&rio_data), phil::data_t_size);

    if (bytes_received != -1 && bytes_received != phil::data_t_size) {
      std::cerr << phil::red << "bytes does not match data_t_size: [" << strerror(errno) << "]" << phil::reset
                << std::endl;
    } else {
      // static-interval detection for zero velocity updates
      // FIXME: need to use raw_accel_* here instead, or rotate using current yaw estimate
      window(window_idx, 0) = rio_data.world_accel_x;
      window(window_idx, 1) = rio_data.world_accel_y;

      auto window_mean = window.colwise().mean();
      double window_variance_norm = (window.rowwise() - window_mean).array().square().matrix().colwise().mean().norm();

      if (window_variance_norm > static_threshold) {
        // set the bias in each axis to the current mean of the window
        biases = window_mean;

        // set the current velocity estimate to be 0
        // TODO: consider making this a "measurment" update with near zero variance?
        auto current_state_estimate = ekf.filter->PostGet()->ExpectedValueGet();
        current_state_estimate(4) = 0;
        current_state_estimate(5) = 0;
        ekf.filter->PostGet()->ExpectedValueSet(current_state_estimate);
      }

      // wrap cicular-array index around
      ++window_idx;
      if (window_idx >= window_size) {
        window_idx = 0;
      }

      double adjusted_ax, adjusted_ay;

      // Data structures for EKF Update
      constexpr double meters_per_tick = 0.000357;
      double v_l = -rio_data.left_encoder_rate * meters_per_tick;
      double v_r = -rio_data.right_encoder_rate * meters_per_tick;

      MatrixWrapper::ColumnVector encoder_input(2);
      encoder_input(1) = v_l;
      encoder_input(2) = v_r;

      // The NavX gives us angles (-180/180), we want to unwrap this to (-\infty,\infty)
      double yaw_rad = -rio_data.yaw * M_PI / 180.0;
      auto d_yaw_rad = phil::yaw_diff_rad(yaw_rad, last_yaw_rad);
      last_yaw_rad = yaw_rad;
      accumulated_yaw_rad += d_yaw_rad;

      MatrixWrapper::ColumnVector yaw_measurement(1);
      yaw_measurement << accumulated_yaw_rad;

      MatrixWrapper::ColumnVector acc_measurement(2);
      acc_measurement << adjusted_ax, adjusted_ay;

      // TODO: perform calibration and other operations to accelerometer data
      Eigen::Matrix3d Ta;
      Ta << 1, -acc_calib_params[0], acc_calib_params[1],
          0, 1, -acc_calib_params[2],
          0, 0, 1;
      Eigen::Matrix3d Ka;
      Ka << acc_calib_params[3], 0, 0,
          0, acc_calib_params[4], 0,
          0, 0, acc_calib_params[5];

      Eigen::Vector3d ba;
      ba << acc_calib_params[6], acc_calib_params[7], acc_calib_params[8];

      ekf.filter->Update(ekf.encoder_system_model.get(), encoder_input);
      ekf.filter->Update(ekf.yaw_measurement_model.get(), yaw_measurement);
      ekf.filter->Update(ekf.acc_measurement_model.get(), acc_measurement);
    }

    // only wait briefly for camera frame.
    // if it's not available that's fine we'll just not call the EKF update for camera data
    uint64_t time = sink.GrabFrame(frame, 0.005);
    if (time > 0) {
      if (frame.empty()) {
        std::cerr << phil::yellow << "empty frame" << std::endl;
        break;
      }

      cv::Mat annotated_frame;
      frame.copyTo(annotated_frame);

      // update step for camera measurement
      std::vector<aruco::Marker> detected_markers = detector.detect(frame);

      // show annotated frame. It's useful to debugging/visualizing
      if (detected_markers.empty()) {
        std::cout << phil::cyan << "no tags detected" << phil::reset << "\n";
        cvsource.PutFrame(annotated_frame); // put in the unannotated frame
        continue;
      }

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
        for (auto &marker : detected_markers) {
          marker.draw(annotated_frame, cv::Scalar(0, 0, 255), 2);
          aruco::CvDrawingUtils::draw3dCube(annotated_frame, marker, camera_params);
          aruco::CvDrawingUtils::draw3dAxis(annotated_frame, marker, camera_params);
        }
      } else {
        std::cerr << "Invalid marker map pose tracker\n";
      }

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

    std::cout << pose.x << ", " << pose.y << ", " << pose.theta << "\n";
    x_entry.SetDouble(pose.x);
    y_entry.SetDouble(pose.y);
    yaw_entry.SetDouble(pose.theta);
  }

  return EXIT_FAILURE;
}

