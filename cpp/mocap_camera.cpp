#include <sys/time.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

double time_to_sec(struct timeval tv) {
  return tv.tv_sec + (float) tv.tv_usec / 1000000.0;
}

struct stamped_frame_t {
  cv::Mat frame;
  double t_sec;
};

stamped_frame_t lastest_frame;

void real_camera_thread() {
  cv::VideoCapture capture;
  capture.open(0);

  if (!capture.isOpened()) {
    std::cerr << "failed to initialized camera" << std::endl;
    return -1;
  }

  for (;;) {
    struct timeval t;
    gettimeofday(&t, nullptr);
    capture >> lastest_frame.frame;
    latest_frame.t_sec = time_to_sec(t);
  }
}

void fake_camera_thread() {
  // open video
  cv::VideoCapture capture;
  capture.open("input.mp4");
  double t = 0; // how to initialize?
  double fps = capture.get(CV_CAP_PROP_FPS);
  for (;;) {
    capture >> lastest_frame.frame;
    latest_frame.t_sec = t;
    t += 1.0/fps;
  }
}

int main(int argc, char **argv) {

  for (;;) {
    // read data from RoboRIO somehow

    // grab latest camera frame
    // lock a mutex?
    cv::Mat frame;
    {
      frame = copy(latest_frame);
    }

    // compute position based on all the data

    // push to network tables

  }

  return 0;
}
