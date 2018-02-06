#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <unordered_set>

void detectMarkers(cv::VideoCapture capture,
                   const std::vector<unsigned long> &timestamps,
                   aruco::CameraParameters cam_params,
                   bool step) {
  cv::Mat frame;
  cv::Mat annotated_frame;

  float marker_size = 0.158; // meters

  //Create the detector
  aruco::MarkerDetector MDetector;
  MDetector.setDetectionMode(aruco::DetectionMode::DM_VIDEO_FAST);
  std::map<uint32_t, aruco::MarkerPoseTracker> tracker; //use a map so that for each id, we use a different pose tracker

  if (!capture.isOpened()) {
    std::cout << "Can not load video";
  } else {
    capture >> frame;

    cam_params.resize(frame.size());

    size_t frame_idx = 0;
    std::unordered_map<int, unsigned int> detection_counts;
    while (capture.isOpened()) {
      capture >> frame;

      if (frame.empty()) {
        break;
      }

      frame.copyTo(annotated_frame);

      // detect markers in frame
      std::vector<aruco::Marker> markers = MDetector.detect(frame);

      for (auto &marker : markers) {
        tracker[marker.id].estimatePose(marker, cam_params, marker_size);

        // draw the tags that were detected
        marker.draw(annotated_frame, cv::Scalar(0, 0, 255), 2);
        aruco::CvDrawingUtils::draw3dCube(annotated_frame, marker, cam_params);
        aruco::CvDrawingUtils::draw3dAxis(annotated_frame, marker, cam_params);

        // output to std out so one can redirect to any file they want
        std::cout << timestamps[frame_idx] << "," << marker.id << ","
                  << marker.Tvec.at<float>(0) << ","
                  << marker.Tvec.at<float>(1) << ","
                  << marker.Tvec.at<float>(2) << ","
                  << marker.Rvec.at<float>(0) << ","
                  << marker.Rvec.at<float>(1) << ","
                  << marker.Rvec.at<float>(2) << std::endl;
      }

      if (step) {
        cv::imshow("annotated", annotated_frame);
        cv::waitKey(10);
        std::cin.get();
      }

      std::string dt_str;
      ++frame_idx;
    }
  }
}

void show_help();

int main(int argc, char **argv) {
  if (argc < 4) {
    show_help();
    return EXIT_FAILURE;
  }

  char *video_filename = argv[1];
  char *timestamps_filename = argv[2];
  char *params_filename = argv[3];

  cv::VideoCapture cap(video_filename);
  auto w = static_cast<const int>(cap.get(CV_CAP_PROP_FRAME_WIDTH));
  auto h = static_cast<const int>(cap.get(CV_CAP_PROP_FRAME_HEIGHT));
  const double fps = cap.get(CV_CAP_PROP_FPS);
  cv::Size input_size(w, h);

  aruco::CameraParameters params;
  try {
    params.readFromXMLFile(params_filename);
    if (!params.isValid()) {
      std::cout << "Camera parameters are invalid." << std::endl;
      return EXIT_FAILURE;
    }
  }
  catch (cv::Exception &e) {
    std::cout << "Camera parameters are invalid." << std::endl;
    return EXIT_FAILURE;
  }

  // Read and process the time stamps
  std::ifstream timestamps_file(timestamps_filename);

  if (!timestamps_file.good()) {
    std::cout << "Bad file: [" << timestamps_filename << "]. " << std::strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<unsigned long> timestamps;
  while (!timestamps_file.eof()) {
    std::string line;
    timestamps_file >> line;

    if (line.empty()) {
      break;
    }

    size_t pos = 0;
    try {

      unsigned long time = std::stoul(line, &pos);
      if (pos != line.length()) {
        std::cout << "Error parsing whole line: [" << line << "] at " << pos << std::endl;
      } else {
        timestamps.push_back(time);
      }
    }
    catch (std::invalid_argument &e) {
      std::cout << "Error parsing: [" << line << "]" << std::endl;
      std::cout << e.what() << std::endl;
    }
  }

  bool step = false;
  if (argc == 5) {
    step = true;
    std::cout << "Stepping (press enter for next frame)" << std::endl;
  }

  detectMarkers(cap, timestamps, params, step);
}

void show_help() {
  std::cout << "USAGE: ./marker_detection_analysis video_file timer_stamps camera_params [-s]"
            << std::endl
            << std::endl
            << "This program will print detected tag info to standard out."
            << "You can analyze the results by feeding that file into analyze_marker_detection_analysis.py"
            << std::endl
            << std::endl
            << "you can use -s to step through frame by frame"
            << std::endl
            << "Example: ./marker_detection_analysis input.avi timestamps.csv camera_params.yml"
            << std::endl
            << "         ./marker_detection_analysis input.avi timestamps.csv camera_params.yml -s"
            << std::endl;
}
