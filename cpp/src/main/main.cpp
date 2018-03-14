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
#include <phil/common/math.h>

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
  args::Flag verbose_flag(parser, "verbose", "Print more information", {'v', "verbose"});
  args::Flag no_camera_flag(parser, "no_camera", "Don't check the camera stream", {"no-camera"});
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

  const bool verbose = args::get(verbose_flag);
  const bool no_camera = args::get(no_camera_flag);

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
  while (server.Read() < 0);

  // Create calibration matrices
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


  /////////////////////////////////////////////
  //// Start of the localization procedure ////
  /////////////////////////////////////////////

  std::cout << phil::green << "Collecting Initial Stationary Sample" << phil::reset << "\n";
  constexpr size_t num_initial_samples = 60;
  Eigen::MatrixX3d initial_samples(num_initial_samples, 3);
  for (size_t i = 0; i < num_initial_samples; ++i) {
    // read some sensor data from roborio
    phil::data_t rio_data = {0};
    ssize_t bytes_received = 0;
    struct sockaddr_in client = {0};
    std::tie(bytes_received, client) = server.Read(&rio_data);
    phil::data_t reply = {0};
    reply.rio_send_time_s = rio_data.rio_send_time_s;
    reply.received_time_s = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    server.Reply(client, reply);

    if (bytes_received != phil::data_t_size) {
      std::cerr << phil::red << "bytes received [" << bytes_received << "] doesn't match data_t size: ["
                << strerror(errno) << "]" << phil::reset << "\n";
      return EXIT_FAILURE;
    } else {
      // the correct amount of data was received so we store it
      initial_samples(i, 0) = rio_data.raw_acc_x;
      initial_samples(i, 1) = rio_data.raw_acc_y;
      initial_samples(i, 2) = rio_data.raw_acc_z;
    }
  }

  // Now that we've collected our initial sample, set a timeout so we can be robust to dropped RoboRIO data
  struct timeval timeout{0};
  timeout.tv_sec = 0;
  timeout.tv_usec = 300000;
  server.SetTimeout(timeout);

  // compute the variance of our initial sample
  Eigen::Matrix<double, 1, 3> initial_static_means = initial_samples.colwise().mean();
  Eigen::MatrixX3d centered = initial_samples.rowwise() - initial_static_means;
  double variance_norm = centered.array().square().matrix().colwise().mean().norm();
  double static_threshold = std::pow(variance_norm, 1.2);

  std::cout << "Using static threshold [" << static_threshold << "]\n";

  // use initial sample to compute base frame rotation
  Eigen::Vector3d calibrated_mean = Ta * Ka * (initial_static_means.transpose() + ba);
  Eigen::Vector3d unit_calibrated_mean = calibrated_mean / calibrated_mean.norm();
  Eigen::Vector3d expected_means{0, 0, 1};
  Eigen::Vector3d v = unit_calibrated_mean.cross(expected_means);
  double c = unit_calibrated_mean.dot(expected_means);
  Eigen::Matrix3d v_x = Eigen::Matrix3d::Zero();
  v_x(0, 1) = -v(2);
  v_x(0, 2) = v(1);
  v_x(1, 0) = v(2);
  v_x(1, 2) = -v(0);
  v_x(2, 0) = -v(1);
  v_x(2, 1) = v(0);
  Eigen::Matrix3d base_rotation = Eigen::Matrix3d::Identity() + v_x + (v_x * v_x) * (1 / (1 + c));
  std::cout << unit_calibrated_mean << "\n";
  std::cout << "Base Rotation Matrix:\n"
            << base_rotation
            << "\n";

  ////////////////////////////////
  //// Start of the main loop ////
  ////////////////////////////////

  phil::EKF ekf;
  cv::Mat frame;
  bool done = false;
  static double accumulated_yaw_rad = 0;
  static double last_yaw_rad = 0;
  constexpr size_t window_size = 60;
  phil::math::Window<window_size, 3> window;
  Eigen::Vector3d latest_static_bias_estimate = calibrated_mean;
  size_t main_loop_idx = 0;
  std::cout << phil::green << "Beginning Localization loop" << phil::reset << std::endl;
  while (!done) {
    // read some sensor data from roborio
    phil::data_t rio_data = {0};
    // FIXME: respond with time stamp or rio time sync won't work and error messages will print in publish_rio_data
    ssize_t bytes_received = 0;
    sockaddr_in client = {0};
    std::tie(bytes_received, client) = server.Read(&rio_data);
    phil::data_t reply = {0};
    reply.rio_send_time_s = rio_data.rio_send_time_s;
    reply.received_time_s = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    server.Reply(client, reply);

    if (bytes_received != -1 && bytes_received != phil::data_t_size) {
      std::cerr << phil::red << "bytes does not match data_t_size: [" << strerror(errno) << "]" << phil::reset
                << std::endl;
    } else {
      Eigen::Vector3d raw_acc{rio_data.raw_acc_x, rio_data.raw_acc_y, rio_data.raw_acc_z};
      window.push(raw_acc);

      if (window.isFull()) {
        auto window_mean = window.colwise().mean();
        auto error = window.rowwise() - window_mean;
        double window_variance_norm = std::pow(error.array().square().matrix().colwise().mean().norm(), 2);
        if (window_variance_norm < static_threshold) {
          //[[9, 104], [539, 760], [776, 965], [1749, 1757], [1763, 2193], [3035, 3061], [3250, 3281],
          // [3337, 3342], [3351, 3771], [5292, 5293], [5303, 5734], [7629, 7637], [7646, 7728]]
          std::cout << "static at idx [" << main_loop_idx << "]\n";
          // set the bias in each axis to the current mean of the window
          latest_static_bias_estimate = window_mean;

          // set the current velocity estimate to be 0
          // TODO: consider making this a "measurement" update with near zero variance?
          auto current_state_estimate = ekf.filter->PostGet()->ExpectedValueGet();
          current_state_estimate(4) = 0;
          current_state_estimate(5) = 0;
          ekf.filter->PostGet()->ExpectedValueSet(current_state_estimate);
        }
      }

      // apply calibration
      Eigen::Vector3d calibrated_acc = Ta * Ka * (raw_acc + ba);

      // rotate into base frame
      Eigen::Vector3d base_frame_acc = base_rotation * calibrated_acc;

      // apply current bias estimate
      Eigen::Vector3d adjusted_acc = base_frame_acc - latest_static_bias_estimate;

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
      acc_measurement << adjusted_acc(0), adjusted_acc(1);

      ekf.filter->Update(ekf.encoder_system_model.get(), encoder_input);
      ekf.filter->Update(ekf.yaw_measurement_model.get(), yaw_measurement);
      ekf.filter->Update(ekf.acc_measurement_model.get(), acc_measurement);
    }

    // only wait briefly for camera frame.
    // if it's not available that's fine, we'll just not call the EKF update for camera data
    uint64_t time = 0;
    if (!no_camera) {
      time = sink.GrabFrame(frame, 0.005);
    }

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

    // Fill our pose struct from the belief state of the EKF
    phil::pose_t pose{};
    auto estimate = ekf.filter->PostGet()->ExpectedValueGet();
    pose.x = estimate(1);
    pose.y = estimate(2);
    pose.theta = estimate(3);

    if (verbose) {
      std::cout << pose.x << ", " << pose.y << ", " << pose.theta << "\n";
    }
    x_entry.SetDouble(pose.x);
    y_entry.SetDouble(pose.y);
    yaw_entry.SetDouble(pose.theta);

    ++main_loop_idx;
  }

  return EXIT_FAILURE;
}

