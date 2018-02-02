#include <iostream>
#include <errno.h>
#include <cstring>
#include <fstream>
#include <cscore.h>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui.hpp>

// #include <phil/common/udp.h>
// #include <phil/common/args.h>

#define CAM_WIDTH 320
#define CAM_HEIGHT 240
#define FPS 30

int main(int argc, char **argv) {

  if(argc != 2){ return -1;}
  char *input_filename = argv[1];
  char *p;
  // int num_frames = (int) strtol(argv[2], &p, 10);
  int num_frames = 100;
  int idx_frames = 0;
  char video_filename[50];
  char timestamp_filename[50];

  time_t now = time(0);
  tm *ltm = localtime(&now);
  strftime(video_filename, 50, "out_%m_%d_%H-%M-%S.avi", ltm);
  strftime(timestamp_filename, 50, "timestamps_%m_%d_%H-%M-%S.csv", ltm);

  std::ofstream time_stamps_file;
  time_stamps_file.open(timestamp_filename);

  if (!time_stamps_file.good()) {
    std::cerr << "Time stamp file failed to open: " << strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }

  cv::VideoCapture capture(input_filename);
  cv::VideoWriter video(video_filename, CV_FOURCC('M', 'J', 'P', 'G'), FPS, cv::Size(CAM_WIDTH, CAM_HEIGHT));

  capture.set(CV_CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);
  capture.set(CV_CAP_PROP_FPS, FPS);
  
  cv::Mat frames[num_frames];
  
  struct timespec timestamps[num_frames];

  struct timespec ts_start;
  struct timespec ts_current;
  struct timespec relative;
  clock_gettime(CLOCK_MONOTONIC, &ts_start);
  while (capture.isOpened() && idx_frames < num_frames) {
    capture.grab();
    capture.retrieve(frames[idx_frames]);
    clock_gettime(CLOCK_MONOTONIC, &ts_current);
    // timestamps[idx_frames].tv_sec = ts_current.tv_sec - ts_start.tv_sec;
    // timestamps[idx_frames].tv_nsec = ts_current.tv_nsec - ts_start.tv_nsec;
    clock_gettime(CLOCK_MONOTONIC, &timestamps[idx_frames]);
    idx_frames++; 
  }

  for(int i = 0; i < num_frames; i++) {
    video.write(frames[i]);
    time_stamps_file << timestamps[i].tv_sec << "." << timestamps[i].tv_nsec << std::endl;
  }

  time_stamps_file.close();

  // cs::UsbCamera camera{"usbcam", 1};
  // camera.SetVideoMode(cs::VideoMode::kMJPEG, CAM_HEIGHT, CAM_HEIGHT, 30);
  // cs::MjpegServer mjpegServer{"httpserver", 8081};
  // mjpegServer.SetSource(camera);
  // cs::CvSink sink{"sink"};
  // sink.SetSource(camera);std::

  // time_t now = time(0);
  // tm *ltm = localtime(&now);
  // char video_filename[50];
  // char timestamp_filename[50];
  // strftime(video_filename, 50, "out_%m_%d_%H-%M-%S.avi", ltm);
  // strftime(timestamp_filename, 50, "timestamps_%m_%d_%H-%M-%S.csv", ltm);

  // cv::Mat frame;
  // cv::VideoWriter video(video_filename, CV_FOURCC('M', 'J', 'P', 'G'), 28, cv::Size(320, 240));

  // std::ofstream time_stamps_file;
  // time_stamps_file.open(timestamp_filename);

  // if (!time_stamps_file.good()) {
  //   std::cerr << "Time stamp file failed to open: " << strerror(errno) << std::endl;
  //   return EXIT_FAILURE;
  // }

  // std::string hostname = "phil-tk1.local";
  // if (server_hostname_flag) {
  //   hostname = args::get(server_hostname_flag);
  // }

  // // wait for UDP message to start
  // phil::UDPServer udp_server;
  // uint8_t message = 0;
  // udp_server.Read(&message, 1);

  // std::cout << "Starting recording" << std::endl;

  // struct timeval timeout = {0};
  // timeout.tv_usec = 10;
  // timeout.tv_sec = 0;
  // udp_server.SetTimeout(timeout);

  // for (int i = 0; i < 1000; i++) {
  //   uint64_t time = sink.GrabFrame(frame);
  //   if (time == 0) {
  //     std::cout << "error: " << sink.GetError() << std::endl;
  //     continue;
  //   }

  //   video.write(frame);
  //   time_stamps_file << time << std::endl;

  //   // check for UDP message to stop
  //   ssize_t bytes_received = udp_server.Read(&message, 1);
  //   if (bytes_received > 0) {
  //     break;
  //   }
  // }

  // std::cout << "Stopping recording" << std::endl;
  // time_stamps_file.close();

  return EXIT_SUCCESS;
}
