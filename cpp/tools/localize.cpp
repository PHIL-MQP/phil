#include <map>
#include <vector>
#include <iostream>

#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>
#include <opencv2/aruco.hpp>
#include <aruco/aruco.h>

#include "localize.h"

void localize(cv::VideoCapture cap, aruco::CameraParameters camParam);
// void localize(cv::VideoCapture cap, cv::Mat intrinsics, cv::Mat distortion);

// void computeCameraPose();

// struct myMarker {
// 	int id;
// 	cv::Mat t;
// 	cv::Mat r;
// };

int main(int argc, char *argv[]) {
  // if(argc != 4) {
  // 	show_help();
  // 	return EXIT_FAILURE;
  // }

  // char *flag = argv[1];
  // char *input = argv[2];
  // char *params_filename = argv[3];

  char *video_filename = argv[1];
  char *params_filename = argv[2];
  cv::VideoCapture cap(video_filename);
  aruco::CameraParameters params;
  params.readFromXMLFile(params_filename);

  cs::MjpegServer mjpegServer("httpserver", 8081);
  cs::CvSink sink("sink");

  // if (strncmp(flag, "-v", 2) == 0) {
  // } else if (strncmp(flag, "-d", 2) == 0) {
  //   int device_number = std::stoi(input);
  //   cs::UsbCamera camera("usbcam", device_number);
  //   camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 30);
  //   mjpegServer.SetSource(camera);
  //   sink.SetSource(camera);
  // } else {
  //   show_help();
  //   return EXIT_FAILURE;
  // }

  localize(cap, params);

  return 0;
}

std::vector<Trans> globalTrans;
std::vector<int> globalIds;
int orig;

cv::Mat cameraT;
cv::Mat cameraR;

//check if some ID pairs are in the transforms
int inTransforms(int toId, int fromId) {
  for (int i = 0; i < globalTrans.size(); i++) {
    if (toId == globalTrans[i].toId && fromId == globalTrans[i].fromId) {
      return 1;
    }
  }
  return 0;
}

int inIds(int id) {
  // for(int i = 0; i < globalIds.size(); i++){
  // 	std::cout << globalIds[i] << std::endl;
  // }
  if (std::find(globalIds.begin(), globalIds.end(), id) != globalIds.end()) {
    return 1;
  }
  return 0;
}

cv::Mat computeTransform(int idTo, int idFrom) {
  if (!inTransforms(idTo, idFrom)) {
    long m[6] = {-999999, -999999, -999999, -999999, -999999, -999999};
    return cv::Mat(2, 3, CV_32F, m);
  }
  cv::Mat m;
  std::vector<Trans>::iterator trans;

  const int n[2] = {idTo, idFrom};
  trans = std::find_if(globalTrans.begin(), globalTrans.end(),
                       [n](const Trans &m) -> bool { return m.toId == n[0] && m.fromId == n[1]; });

  m.push_back(globalTrans[trans - globalTrans.begin()].t.t());
  m.push_back(globalTrans[trans - globalTrans.begin()].r.t());
  return m;
}

std::vector<int> getAdjacentNodes(int id) {
  std::vector<int> ret;
  for (int i = 0; i < globalTrans.size(); i++) {
    if (globalTrans[i].toId == id) {
      ret.push_back(globalTrans[i].fromId);
    }
    if (globalTrans[i].fromId == id) {
      ret.push_back(globalTrans[i].toId);
    }
  }

  return ret;

}

void localize(cv::VideoCapture capture, aruco::CameraParameters CamParam) {
  cv::Mat frame;
  cv::Mat annotated_frame;

  float MarkerSize = 0.200; // meters

  //Create the detector
  aruco::MarkerDetector MDetector;
  MDetector.setDetectionMode(aruco::DetectionMode::DM_VIDEO_FAST);
  std::map<uint32_t, aruco::MarkerPoseTracker> tracker;//use a map so that for each id, we use a different pose tracker
  cv::namedWindow("in", 1);

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
      cv::imshow("in", frame);
      cv::waitKey(1);

      /*detect markers in frame*/
      std::vector<aruco::Marker> markers = MDetector.detect(frame);
      if (markers.size() > 0) {
        // std::cout << "frame: " << frame_idx << std::endl;

        // set the origin to be the first seen tag
        if (globalIds.size() < 1) {
          orig = markers[0].id;
          std::cout << "origin added" << std::endl;
          std::cout << markers[0].id << std::endl;
        }
        //markers loop
        for (int i = 0; i < markers.size(); i++) {
          //estimate pose and draw frame
          tracker[markers[i].id].estimatePose(markers[i], CamParam, MarkerSize);

          if (!inIds(markers[i].id)) {
            globalIds.push_back(markers[i].id);
            std::cout << "added marker\t" << markers[i].id << std::endl;
          }

          markers[i].draw(annotated_frame, cv::Scalar(0, 0, 255), 2);
          if (markers[i].Tvec.at<float>(0, 0) < -999998) continue;
          // std::cout << markers[i].Tvec << '\t';

          if (markers.size() > 1) {

            //transforms loop
            for (int j = 0; j < markers.size(); j++) {

              //add transforms to vector
              //only allow for toID < fromID
              //this way, it avoids duplicates
              if (markers[j].id != markers[i].id && markers[i].id < markers[j].id) {
                // std::cout << markers[i] << std::endl;
                tracker[markers[j].id].estimatePose(markers[j], CamParam, MarkerSize);
                if (markers[j].Tvec.at<float>(0, 0) < -999998) continue;
                //check for transforms
                if (!inTransforms(markers[i].id, markers[j].id)) {
                  Trans t;
                  t.toId = markers[i].id;
                  t.fromId = markers[j].id;
                  t.t = (markers[i].Tvec - markers[j].Tvec);
                  t.r = (markers[i].Rvec - markers[j].Rvec);
                  globalTrans.push_back(t);
                  std::cout << "added transforms" << std::endl;
                  std::cout << t.toId << '\t' << t.fromId << std::endl;
                }
              }
            }
          }

          //if the marker is our origin, just report position away
          if (markers[i].id == orig) {
            cv::Mat pose;
//            std::cout << markers[i].Tvec << std::endl;
//            std::cout << markers[i].Rvec << std::endl;
            continue;
          }
            // cv::Mat m;
            // if(orig < markers[i].id){
            // 	m = computeTransform(orig, markers[i].id);
            // } else {
            // 	m = computeTransform(markers[i].id, orig);
            // }
            // if(m.at<float>(0,0) > -9999.){
            // 	cv::Mat out = (m.row(0).t() + markers[i].Tvec);
            // 	// std::cout << m.row(0).t() << '\n';
            // 	// std::cout << out << endl;
            // }
          else {
            // search though through the transforms
            int pathFound = 0;
            int pathIdx = -1;
            int startId = markers[i].id;
            int goalId = orig;
            std::vector<int> foundPath;
            std::vector<int> start;
            start.push_back(startId);

            std::vector<std::vector<int> > paths;
            paths.push_back(start);

            while (!pathFound) {

              int deadEnds = 0;
              for (int k = 0; k < paths.size(); k++) {
                std::vector<int> allTrans = getAdjacentNodes(paths[k].back());
                if (allTrans.size() == 0) {
                  // std::cout << "rouge tag " << paths[i].back() << std::endl;
                  pathFound = 1;
                  break;
                }
                // build paths from transforms
                std::vector<std::vector<int> > toAppend;
                for (int j = 0; j < allTrans.size(); j++) {

                  //no cyclic paths
                  if (std::find(paths[k].begin(), paths[k].end(), allTrans[j]) == paths[k].end()) {

                    //add to paths
                    std::vector<int> pathCopy(paths[k]);
                    pathCopy.push_back(allTrans[j]);
                    toAppend.push_back(pathCopy);

                    //check if a path from start to goal has been found
                    if ((std::find(pathCopy.begin(), pathCopy.end(), startId) != pathCopy.end())
                        && (std::find(pathCopy.begin(), pathCopy.end(), goalId) != pathCopy.end())) {

                      // foundPath.reserve(pathCopy.size());
                      foundPath.insert(foundPath.end(), pathCopy.begin(), pathCopy.end());
                      pathFound = 1;
                      break;
                    }
                  } else {
                    deadEnds++;
                  }
                }

                if (pathFound) {
                  break;
                }
                paths[k].pop_back();
                // paths.reserve( paths.size() + toAppend.size());
                paths.insert(paths.end(), toAppend.begin(), toAppend.end());

              }
              // if(deadEnds == paths.size()) {
              // 	std::cout << "no transforms" << std::endl;
              // 	break;
              // }
            }
            if (foundPath.size() > 0) {
              std::cout << "using transforms" << std::endl;
              for (int j = 0; j < foundPath.size(); j++) {
                std::cout << foundPath[j] << '\t';
              }
              std::cout << std::endl;
              float a[6] = {0., 0., 0., 0., 0., 0.};
              cv::Mat pose;
              std::cout << markers[i].Tvec << std::endl;
              std::cout << markers[i].Rvec << std::endl;
              cv::vconcat(markers[i].Tvec.t(), markers[i].Rvec.t(), pose);
              cv::Mat r(2, 3, CV_32F, a);
              cv::Mat m;
              for (int j = 0; j < foundPath.size() - 1; j++) {
                if (foundPath[j] < foundPath[j + 1]) {
                  m = computeTransform(foundPath[j], foundPath[j + 1]);

                } else {
                  m = computeTransform(foundPath[j + 1], foundPath[j]);
                  m = -1 * m;
                }
                // std::cout << m << std::endl;

                if (m.at<float>(0, 0) > -9999.) {
                  pose = pose + m;
                  // std::cout << m.row(0).t() << '\n';
                  // std::cout << out << endl;
                } else {
                  std::cout << "error computing transform \t";
                  pose.at<float>(0, 0) = -99999.;
                  break;
                }

              }
              std::cout << pose << std::endl;

            }
          }

        }
        // std::cout << endl;
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

void show_help() {
  std::cout << "USAGE: ./localize [-v video_filename|-d device_number]  params_file"
            << std::endl
            << std::endl
            << "EXAMPLE: ./localize -v 0 params.yml"
            << "         ./localize -d test.avi params.yml"
            << std::endl;
}
