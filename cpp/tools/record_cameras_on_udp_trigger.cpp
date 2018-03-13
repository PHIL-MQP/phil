#include <iostream>
#include <cstring>
#include <fstream>
#include <cscore.h>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <phil/common/udp.h>
#include <phil/common/args.h>

int main(int argc, const char **argv) {
  args::ArgumentParser parser("This program records camera frames and their timestamps.");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::Positional<std::string> config_filename(parser, "config_filename", "yaml file of configuration.\n"
      "See the cpp/configs folder for example yaml config files.", args::Options::Required);

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
  // Read all the camera configs and open cs core streams
  YAML::Node config = YAML::LoadFile(args::get(config_filename));

  const auto device = config["device"].as<int>();
  cs::UsbCamera camera{"usbcam", device};
  const auto w = config["width"].as<int>();
  const auto h = config["height"].as<int>();
  const auto fps = config["fps"].as<int>();
  const auto udp_port = config["udp_port"].as<int16_t>();
  std::string enc = config["encoding"].as<std::string>();

  if (enc == "MJPG") {
    camera.SetVideoMode(cs::VideoMode::kMJPEG, w, h, fps);
  } else if (enc == "YUYV") {
    camera.SetVideoMode(cs::VideoMode::kYUYV, w, h, fps);
  } else {
    camera.SetVideoMode(cs::VideoMode::kMJPEG, w, h, fps);
    std::cerr << "Invalid format [" << enc << "]. Defaulting to MJPG" << std::endl;
  }

  cs::CvSink sink{"sink"};
  sink.SetSource(camera);
  cs::MjpegServer mjpeg_server{"httpserver_" + std::to_string(device), 8080 + device};
  mjpeg_server.SetSource(camera);

  // create output folder
  char out_dir[50];
  time_t now = time(nullptr);
  tm *ltm = localtime(&now);
  std::stringstream fmt_ss;
  fmt_ss << "video" << device << "_%m_%d_%H-%M-%S";
  strftime(out_dir, 50, fmt_ss.str().c_str(), ltm);
  mkdir(out_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  // Create timestamps file
  std::ofstream time_stamps_file;
  std::stringstream timestamp_ss;
  timestamp_ss << out_dir << "/" << "timestamps.csv";
  time_stamps_file.open(timestamp_ss.str());

  if (!time_stamps_file.good()) {
    std::cerr << "Time stamp file failed to open: " << strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }

  std::stringstream video_ss;
  video_ss << out_dir << "/" << "out.avi";
  cv::VideoWriter video(video_ss.str(), CV_FOURCC('M', 'J', 'P', 'G'), fps, cv::Size(w, h));

  // wait for UDP message to start
  phil::UDPServer udp_server(udp_port);
  udp_server.Read();

  std::cout << "Starting recording" << std::endl;

  struct timeval timeout = {0};
  timeout.tv_usec = 10;
  timeout.tv_sec = 0;
  udp_server.SetTimeout(timeout);

  cv::Mat frame;
  unsigned long frame_idx = 0;
  while (true) {
    uint64_t time = sink.GrabFrame(frame);
    if (time == 0) {
      std::cout << "error grabbing frame " << sink.GetError() << "]\n";
      continue;
    }
    else {
      time_stamps_file << time << "\n";

      video.write(frame);
      ++frame_idx;
    }

    // check for UDP message to stop
    ssize_t bytes_received;
    bytes_received = udp_server.Read();
    if (bytes_received > 0) {
      break;
    }
  }

  std::cout << "Stopping recording" << std::endl;
  time_stamps_file.close();

  return EXIT_SUCCESS;
}
