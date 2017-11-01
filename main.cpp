/**
  @file videocapture_basic.cpp
  @brief A very basic sample for using VideoCapture and VideoWriter
  @author PkLab.net
  @date Aug 24, 2016
  */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  if (argc <= 1)
  {
    std::cout << "USAGE: ./main camera_id"
              << std::endl
              << std::endl
              << "EX: ./main 1"
              << std::endl;
    return 0;
  }


  Mat frame;
  //--- INITIALIZE VIDEOCAPTURE
  VideoCapture cap;
  int deviceID = atoi(argv[1]);
  cap.open(deviceID);
  // check if we succeeded
  if (!cap.isOpened()) {
    cerr << "ERROR! Unable to open camera\n";
    return -1;
  }

  //--- GRAB AND WRITE LOOP
  cout << "Start grabbing" << endl
    << "Press any key to terminate" << endl;
  for (;;)
  {
    // wait for a new frame from camera and store it into 'frame'
    cap.read(frame);
    // check if we succeeded
    if (frame.empty()) {
      cerr << "ERROR! blank frame grabbed\n";
      break;
    }
    // show live and wait for a key with timeout long enough to show images
    imshow("Live", frame);
    if (waitKey(5) >= 0)
      break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
