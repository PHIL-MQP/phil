#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>

void detectMarkers(cv::VideoCapture capture, aruco::CameraParameters CamParam, cv::VideoWriter &out_video, bool step, bool quiet) {
  cv::Mat frame;
  cv::Mat annotated_frame;

  float MarkerSize = 0.175; // meters


  //Create the detector
  aruco::MarkerDetector MDetector;
  MDetector.setDetectionMode(aruco::DetectionMode::DM_VIDEO_FAST);
  std::map<uint32_t, aruco::MarkerPoseTracker> tracker; //use a map so that for each id, we use a different pose tracker

  if (!capture.isOpened()) {
    std::cout << "Can not load video";
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

      /*detect markers in frame*/
      std::vector<aruco::Marker> markers = MDetector.detect(frame);

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

      if (!quiet) {
        cv::imshow("annotated", annotated_frame);
      }

      if (step) {
        std::cin.get();
      }
      else if (!quiet) {
        cv::waitKey(10);
      }

      out_video.write(annotated_frame);
      ++frame_idx;
    }
  }
}

void show_help();

int main(int argc, char **argv) {
  if (argc < 3) {
    show_help();
    return EXIT_FAILURE;
  }

  char *video_filename = argv[1];
  char *params_filename = argv[2];

  cv::VideoCapture cap(video_filename);
  const unsigned int w = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  const unsigned int h = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  const unsigned int fps = cap.get(CV_CAP_PROP_FPS);
  cv::Size input_size(w, h);

  bool step = false;
  bool quiet = false;
  cv::VideoWriter out_video;
  std::string outfile;
  if (argc == 4) {
    if (strncmp(argv[3], "-s", 2) == 0) {
      step = true;
    }
    else if (strncmp(argv[3], "-q", 2) == 0) {
      quiet = true;
    }
    else {
      // for output file
      out_video = cv::VideoWriter(argv[3], CV_FOURCC('M', 'J', 'P', 'G'), fps, input_size);
    }
  }
  else if (argc == 5) {
    if (strncmp(argv[3], "-s", 2) == 0) {
      step = true;
    }
    else if (strncmp(argv[3], "-q", 2) == 0) {
      quiet = true;
    }
    // for output file
    out_video = cv::VideoWriter(argv[4], CV_FOURCC('M', 'J', 'P', 'G'), fps, input_size);
  }

  aruco::CameraParameters params;
  params.readFromXMLFile(params_filename);

  detectMarkers(cap, params, out_video, step, quiet);
}

void show_help() {
  std::cout << "USAGE: ./detect_markers VideoFile CameraParamsFile [-s] [outfile.avi]"
            << std::endl
            << std::endl
            << "you can use -s to step through frame by frame"
            << std::endl
            << "Example: ./detect_markers out.avi recorded_sensor_data/camera_calibration/11_12.yml"
            << std::endl
            << "         ./detect_markers out.avi recorded_sensor_data/camera_calibration/11_12.yml -q"
            << std::endl
            << "         ./detect_markers out.avi recorded_sensor_data/camera_calibration/11_12.yml -s out.avi"
            << std::endl;
}
