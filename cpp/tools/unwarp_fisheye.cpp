#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static constexpr unsigned short maxR = 400, maxT = 2250;
static constexpr unsigned short midX = 320, midY = 240;

int main(int argc, const char **argv) {
  cv::VideoCapture capture;
  cv::Mat log_polar_img;

  cv::CommandLineParser parser(argc, argv, "{@input|0|}");
  std::string arg = parser.get<std::string>("@input");

  if (arg.size() == 1 && isdigit(arg[0]))
    capture.open(arg[0] - '0');
  else
    capture.open(arg.c_str());

  bool done = false;

  cv::Mat map_x;
  cv::Mat map_y;

  map_x.create(maxR, maxT, CV_32FC1);
  map_y.create(maxR, maxT, CV_32FC1);

  for (int x = 0; x < maxT; x++) {
    const double radians = static_cast<float>(x) / maxT * 2 * M_PI;
    for (int y = 0; y < maxR; y++) {
      map_y.at<float>(maxR - y - 1, x) = midY + y * static_cast<float>(sin(radians));
      map_x.at<float>(maxR - y - 1, x) = midX + y * static_cast<float>(cos(radians));
    }
  }

  while (!done) {
    cv::Mat frame;
    capture >> frame;

    cv::Mat unwarped_frame;
    cv::remap(frame, unwarped_frame, map_x, map_y, CV_INTER_LINEAR);

    cv::imshow("unwarped image", unwarped_frame);
    cv::imshow("raw image", frame);
    if (cv::waitKey(10) == 'q') {
      break;
    }
  }

  return EXIT_SUCCESS;
}
