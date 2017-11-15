#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;

void help() {
  printf("\nThis program illustrates Log-Polar image transforms\n"
             "Usage :\n"
             "./polar_transforms [[camera number -- Default 0],[path_to_filename]]\n\n");
}

int main(int argc, char **argv) {
  VideoCapture capture;
  Mat log_polar_img;

  help();

  CommandLineParser parser(argc, argv, "{@input|0|}");
  std::string arg = parser.get<std::string>("@input");

  std::cout << "Press q twice to quit" << std::endl;

  if (arg.size() == 1 && isdigit(arg[0]))
    capture.open(arg[0] - '0');
  else
    capture.open(arg.c_str());

  if (!capture.isOpened()) {
    const char *name = argv[0];
    fprintf(stderr, "Could not initialize capturing...\n");
    fprintf(stderr, "Usage: %s <CAMERA_NUMBER>    , or \n       %s <VIDEO_FILE>\n", name, name);
    return -1;
  }

  namedWindow("Log-Polar", WINDOW_AUTOSIZE);

  for (;;) {
    Mat frame;
    capture >> frame;

    if (frame.empty())
      break;

    Point2f center((float) frame.cols / 2, (float) frame.rows / 2);
    double M = 70;

    logPolar(frame, log_polar_img, center, M, INTER_LINEAR + WARP_FILL_OUTLIERS);

    imshow("Log-Polar", log_polar_img.t());

    if (waitKey(10) == 'q') {
      break;
    }
  }

  waitKey(0);
  return 0;
}
