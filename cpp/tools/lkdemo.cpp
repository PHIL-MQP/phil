#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

static void help() {
  cout << "Opens a camera (optional argument) and processes 100 frames with Lucas-Kanade tracking.\n";
}

int main(int argc, char **argv) {
  VideoCapture cap;
  TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
  Size subPixWinSize(10, 10), winSize(31, 31);

  const int MAX_COUNT = 500;
  bool needToInit = true;
  bool nightMode = false;

  help();
  cv::CommandLineParser parser(argc, argv, "{@input|0|}");
  string input = parser.get<string>("@input");

  if (input.size() == 1 && isdigit(input[0]))
    cap.open(input[0] - '0');
  else
    cap.open(input);

  if (!cap.isOpened()) {
    cout << "Could not initialize capturing...\n";
    return 0;
  }

  Mat gray, prevGray, image, frame;
  vector<Point2f> points[2];

  for (size_t i = 0; i < 100; ++i) {
    cap >> frame;
    if (frame.empty())
      break;

    frame.copyTo(image);
    cvtColor(image, gray, COLOR_BGR2GRAY);

    if (needToInit) {
      // automatic initialization
      goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
      cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
    } else if (!points[0].empty()) {
      vector<uchar> status;
      vector<float> err;
      if (prevGray.empty())
        gray.copyTo(prevGray);
      calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
      size_t j, k;
      for (j = k = 0; j < points[1].size(); j++) {
        if (!status[j])
          continue;

        points[1][k++] = points[1][j];
        circle(image, points[1][j], 3, Scalar(0, 255, 0), -1, 8);
      }
      points[1].resize(k);
    }

    needToInit = false;

    std::swap(points[1], points[0]);
    cv::swap(prevGray, gray);
  }

  return 0;
}
