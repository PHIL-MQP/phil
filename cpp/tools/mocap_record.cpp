#include <iostream>
#include <cstring>
#include <fstream>
#include <cscore.h>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>

#include <phil/common/udp.h>
#include <phil/common/args.h>

struct camera_t {
  int id;
  cs::UsbCamera usb_camera;
  cs::CvSink sink;
  cv::VideoWriter video;
};

int main(int argc, const char **argv) {
  args::ArgumentParser parser("This program records camera frames and their timestamps",
                              "This program is meant to run on TK1 during Motion Capture recording tests."
                                  "However, it is general purpose and could also be used on a laptop."
                                  "It cannot be built for the RoboRIO.");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::Positional<std::string> config_filename(parser, "config_filename", "yaml file of configuration", args::Options::Required);

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

  // Create timestamps file (one column per camera
  time_t now = time(nullptr);
  tm *ltm = localtime(&now);
  std::ofstream time_stamps_file;
  char timestamp_filename[50];
  strftime(timestamp_filename, 50, "timestamps_%m_%d_%H-%M-%S.csv", ltm);
  time_stamps_file.open(timestamp_filename);

  if (!time_stamps_file.good()) {
    std::cerr << "Time stamp file failed to open: " << strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }

  // Read all the camera configs and open cs core streams
  YAML::Node config = YAML::LoadFile(args::get(config_filename));

  std::vector<camera_t> cameras;
  for (auto cam_config : config) {
    const auto device = cam_config["device"].as<int>();
    cs::UsbCamera cam{"usbcam", device};
    const auto w = cam_config["width"].as<int>();
    const auto h = cam_config["height"].as<int>();
    const auto fps = cam_config["fps"].as<int>();
    std::string enc = cam_config["encoding"].as<std::string>();

    if (enc == "MJPG") {
      cam.SetVideoMode(cs::VideoMode::kMJPEG, w, h, fps);
    }
    else if (enc == "YUYV") {
      cam.SetVideoMode(cs::VideoMode::kYUYV, w, h, fps);
    }
    else {
      cam.SetVideoMode(cs::VideoMode::kMJPEG, w, h, fps);
      std::cerr << "Invalid format [" << enc << "]. Defaulting to MJPG" << std::endl;
    }

    cs::CvSink sink{"sink"};
    sink.SetSource(cam);

    char video_filename[50];
    strftime(video_filename, 50, "out_%m_%d_%H-%M-%S.avi", ltm);

    cv::VideoWriter video(video_filename, CV_FOURCC('M', 'J', 'P', 'G'), fps, cv::Size(w, h));

    cameras.push_back({device, cam, sink});
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

  cv::Mat frame;
  while (true) {
    for (camera_t camera : cameras) {
      uint64_t time = camera.sink.GrabFrame(frame);
      if (time == 0) {
        std::cout << "error on camera " << camera.id << " [" << camera.sink.GetError() << "]\n";
        continue;
      }

      time_stamps_file << time << std::endl;
      camera.video.write(frame);
    }

    time_stamps_file << "\n";

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
