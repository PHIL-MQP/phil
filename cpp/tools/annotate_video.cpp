#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>

#include <phil/common/args.h>

void annotate_video(cv::VideoCapture capture, aruco::CameraParameters CamParam, cv::VideoWriter &out_video, bool step, bool quiet) {
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

int main(int argc, const char **argv) {
  args::ArgumentParser parser("Read a video and write a new video with all the frames annotated\n");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::Positional<std::string> in_video_param(parser, "in_video_filename", "input video", args::Options::Required);
  args::Positional<std::string> out_video_param(parser, "out_video_filename", "output video", args::Options::Required);
  args::Positional<std::string> camera_config_param(parser, "params_filename", "camera parameters yaml file", args::Options::Required);
  args::Flag quiet_flag(parser, "quiet", "don't show the video frames", {'q', "quiet"});
  args::Flag step_flag(parser, "step", "step the video frame-by-frame", {'s', "step"});

  try {
    parser.ParseCLI(argc, argv);
  }
  catch (args::Help &e) {
    std::cout << parser;
    return 0;
  }
  catch (args::RequiredError &e) {
    std::cout << parser;
    return 0;
  }

  cv::VideoCapture cap(args::get(in_video_param));
  auto w = static_cast<const unsigned int>(cap.get(CV_CAP_PROP_FRAME_WIDTH));
  auto h = static_cast<const unsigned int>(cap.get(CV_CAP_PROP_FRAME_HEIGHT));
  auto fps = static_cast<const unsigned int>(cap.get(CV_CAP_PROP_FPS));
  cv::Size input_size(w, h);

  bool step = false;
  bool quiet = false;
  cv::VideoWriter out_video;
  std::string outfile;

  if (args::get(step_flag)) {
    std::cout << "will step frame-by-frame\n";
    step = true;
  }

  out_video = cv::VideoWriter(args::get(out_video_param), CV_FOURCC('M', 'J', 'P', 'G'), fps, input_size);

  if (args::get(quiet_flag)) {
    quiet = true;
  }

  aruco::CameraParameters params;
  params.readFromXMLFile(args::get(camera_config_param));

  annotate_video(cap, params, out_video, step, quiet);
}
