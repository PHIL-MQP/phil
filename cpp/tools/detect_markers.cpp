#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include <aruco/aruco.h>

void detectMarkers(cv::VideoCapture capture, aruco::CameraParameters CamParam) {
  cv::Mat frame;
  cv::Mat annotated_frame;

  float MarkerSize = 0.175; // meters

  //Create the detector
  aruco::MarkerDetector MDetector;
  MDetector.setThresholdParams(7, 7);
  MDetector.setThresholdParamRange(2, 0);
  map<uint32_t, aruco::MarkerPoseTracker> tracker;//use a map so that for each id, we use a different pose tracker
  cv::namedWindow("in", 1);

  if (!capture.isOpened()) {
    cout << "Can not load video";
  } else {
    capture >> frame;

    CamParam.resize(frame.size());

    unsigned int frame_idx = 0;
    while (capture.isOpened()) {
      capture.grab();
      capture.retrieve(frame);

      if (frame.empty()) {
        break;
      }

      frame.copyTo(annotated_frame);
      std::cout << "frame: " << frame_idx << std::endl;
      cv::imshow("in", frame);
      cv::waitKey(1);

      /*detect markers in frame*/
      vector<aruco::Marker> markers = MDetector.detect(frame);

      for (auto &marker:markers) {
        tracker[marker.id].estimatePose(marker, CamParam, MarkerSize);
        marker.draw(annotated_frame, cv::Scalar(0, 0, 255), 2);
      }

      if (CamParam.isValid() && MarkerSize != -1) {
        for (auto &marker:markers) {
          aruco::CvDrawingUtils::draw3dCube(annotated_frame, marker, CamParam);
          aruco::CvDrawingUtils::draw3dAxis(annotated_frame, marker, CamParam);
        }
      }

      cv::imshow("annotated", annotated_frame);
      cv::waitKey(1);
      ++frame_idx;
    }
  }
}

void show_help();

int main(int argc, char **argv) {
  if (argc != 3) {
    show_help();
    return EXIT_FAILURE;
  }

  char *video_filename = argv[1];
  char *params_filename = argv[2];

  cv::VideoCapture cap(video_filename);
  aruco::CameraParameters params;
  params.readFromXMLFile(params_filename);

  detectMarkers(cap, params);
}

void show_help() {
  std::cout << "USAGE: ./detect_markers VideoFile CameraParamsFile"
            << std::endl
            << std::endl
            << "Example: ./detect_markers out.avi recorded_sensor_data/camera_calibration/11_12.yml"
            << std::endl;
}