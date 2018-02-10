#include <iostream>
#include <cstring>
#include <fstream>
#include <cscore.h>
#include <opencv2/opencv.hpp>

#include <phil/common/udp.h>
#include <phil/common/args.h>
#include <marker_mapper/markermapper.h>
#include <aruco/aruco.h>

int main(int argc, const char **argv) {

	time_t now = time(0);
  tm *ltm = localtime(&now);
  char video_filename[50];
  char timestamp_filename[50];
  strftime(video_filename, 50, "out_%m_%d_%H-%M-%S.avi", ltm);
  strftime(timestamp_filename, 50, "timestamps_%m_%d_%H-%M-%S.csv", ltm);
	
	cs::UsbCamera camera{"usbcam", 1};
  constexpr int w = 640;
  constexpr int h = 480;
  constexpr int fps = 60;

  camera.SetVideoMode(cs::VideoMode::kYUYV, w, h, fps);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);
  cs::CvSink sink{"sink"};
  sink.SetSource(camera);

  std::cout << "Go to localhost:8082 to see annotated camera stream" << std::endl;
  cs::CvSink cvsink{"cvsink"};
  cvsink.SetSource(camera);
  cs::CvSource cvsource{"cvsource", cs::VideoMode::kYUYV, w, h ,fps};
  cs::MjpegServer cvMjpegServer{"cvhttpserver", 8082};
  cvMjpegServer.SetSource(cvsource);

  // read in the markermapper config yaml file
	aruco::MarkerMap mmap;
  mmap.readFromFile("/home/kacper/Desktop/2_7/mapper/ps3_b_640px_60fps.yml");
  mmap.setDictionary("ARUCO");
	constexpr int markerMeters = .0183;
  // convert to meters if necessary
  if (mmap.isExpressedInPixels()) {
    mmap = mmap.convertToMeters(markerMeters);
  }

  // marker detector is needed for passing detected markers to the tracker
  aruco::MarkerDetector detector;
  detector.setDictionary("ARUCO");

  // load camera params
  std::cout << "Loading Camera Params from cam_params.yml" << std::endl;
  aruco::CameraParameters camera_params;
  camera_params.readFromXMLFile("/home/kacper/Desktop/2_7/mapper/ps3_b_640px_60fps-cam.yml");
  camera_params.resize(cv::Size(w, h));
  cv::VideoWriter output_cap(video_filename, CV_FOURCC('M', 'P', 'E', 'G'), fps, cv::Size(w, h ));
	if (!output_cap.isOpened())
	{
	        std::cout << "!!! Output video could not be opened" << std::endl;
	        return -1;
	}
  // create pose tracker
  aruco::MarkerMapPoseTracker tracker;
  tracker.setParams(camera_params, mmap);

  bool done = false;
  cv::Mat frame;

  while (!done) {
		uint64_t time = sink.GrabFrame(frame, 0.010);
    if (time > 0) {
    	// double time_s = time / 1e6; // convert micoseconds to seconds
      cv::Mat annotated_frame;
      frame.copyTo(annotated_frame);

      // update step for camera measurement
      std::vector<aruco::Marker> detected_markers = detector.detect(frame);

      // estimate 3d camera pose if possible
      if (tracker.isValid()) {
        // estimate the pose of the camera with respect to the detected markers
        if (tracker.estimatePose(detected_markers)) {
          cv::Mat rt_matrix = tracker.getRTMatrix();
          std::cout << rt_matrix.col(3).t() << ' ' << time << std::endl;
        }

        // annotate the video feed
        for (int idx : mmap.getIndices(detected_markers)) {
          detected_markers[idx].draw(annotated_frame, cv::Scalar(0, 0, 255), 1);
        }
      }

      // show annotated frame. It's useful to debugging/visualizing
      cvsource.PutFrame(annotated_frame);
      output_cap.write(frame);
      
    }

  }
  output_cap.release();
}