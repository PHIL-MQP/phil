#include <iostream>
#include <errno.h>
#include <cstring>
#include <fstream>
#include <cscore.h>
#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <cscore.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include <algorithm> // for std::copy


// #include <phil/common/udp.h>
// #include <phil/common/args.h>

#define CAM_WIDTH 320
#define CAM_HEIGHT 240
#define FPS 30

int main(int argc, char **argv) {

  char *video_filename = argv[1];
  char *params_filename = argv[3];
  cv::VideoCapture capture(video_filename);
  aruco::CameraParameters params;
  params.readFromXMLFile(params_filename);
  capture.set(CV_CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);
  capture.set(CV_CAP_PROP_FPS, 30);
  params.resize(cv::Size(CAM_WIDTH, CAM_HEIGHT));
  aruco::MarkerDetector MDetector;
  std::map<uint32_t, aruco::MarkerPoseTracker> tracker;
  float MarkerSize = 0.20; // meters

	std::ifstream is(argv[2]);
  std::istream_iterator<double> start(is), end;
  std::vector<double> timestamps(start, end);
  std::cout << "Read " << timestamps.size() << " timestamps" << std::endl;

  std::ofstream poses_stamped;
  time_stamps_file.open(poses_stamped);

  if (!poses_stamped.good()) {
    std::cerr << "Time stamp file failed to open: " << strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }

  // print the numbers to stdout
  std::cout << "timestamps read in:\n";
  std::copy(timestamps.begin(), timestamps.end(), 
            std::ostream_iterator<double>(std::cout, "\n"));
  std::cout << std::endl;

  cv::Mat frame;
  int idx_frames = 0;
  int length = (int) capture.get(CV_CAP_PROP_FRAME_COUNT);
  std::cout << length << std::endl;
  while (capture.isOpened()) {
  	capture.grab();
    capture.retrieve(frame);
    if(frame.empty()) {break;}
		std::vector<aruco::Marker> markers = MDetector.detect(frame);
		if(markers.size() > 0) {
			for (int i = 0; i < markers.size(); i++) {
				tracker[markers[i].id].estimatePose(markers[i], params, MarkerSize);
				std::cout << markers[i].id << ' ' << markers[i].Tvec.reshape(1,1) << ' ' << markers[i].Rvec.reshape(1,1) << ' ';
				poses_stamped << markers[i].id << ' ' << markers[i].Tvec.reshape(1,1) << ' ' << markers[i].Rvec.reshape(1,1) << ' ';
			}
			std::cout << timestamps[idx_frames] << idx_frames <<  std::endl;
			poses_stamped << timestamps[idx_frames] << ' ' << idx_frames <<  std::endl;
		}
    idx_frames++;
  }

}