#include <cstdio>
#include <cscore.h>

#include "localize.h"

std::vector<Trans> globalTrans;
int orig;

int main(int argc, char *argv[]) {
  if (argc != 4) {
    show_help();
    return EXIT_FAILURE;
  }

  char *flag = argv[1];
  char *input = argv[2];
  char *params_filename = argv[3];

  aruco::CameraParameters params;
  params.readFromXMLFile(params_filename);

  cs::MjpegServer mjpegServer("httpserver", 8081);
  cs::CvSink sink("sink");

  if (strncmp(flag, "-v", 2) == 0) {
  } else if (strncmp(flag, "-d", 2) == 0) {
    int device_number = std::stoi(input);
    cs::UsbCamera camera("usbcam", device_number);
    camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 30);
    mjpegServer.SetSource(camera);
    sink.SetSource(camera);
  } else {
    show_help();
    return EXIT_FAILURE;
  }

  cs::CvSource annotated_source{"annotated", cs::VideoMode::kMJPEG, 320, 240, 30};
  cs::MjpegServer cvMjpegServer{"annotated_httpserver", 8082};
  cvMjpegServer.SetSource(annotated_source);

  config_t config;
  config.camera_params = params;
  config.marker_size = 0.2;

  cv::Mat frame;
  cv::Mat annotated_frame;

  // Create the detector
  aruco::MarkerDetector MDetector;
  MDetector.setThresholdParams(7, 7);
  MDetector.setThresholdParamRange(2, 0);
  map<int, aruco::MarkerPoseTracker> trackers;// use a map so that for each id, we use a different pose tracker

  sink.GrabFrame(frame);
  config.camera_params.resize(frame.size());

  bool done = false;
  unsigned int frame_idx = 0;
  while (!done) {
    sink.GrabFrame(frame);

    if (frame.empty()) {
      std::cout << "empty frame" << std::endl;
      continue;
    }

    frame.copyTo(annotated_frame);

    vector<aruco::Marker> markers = MDetector.detect(frame);
    if (!markers.empty()) {

      // set the origin to be the first seen tag
      if (trackers.empty()) {
        orig = markers[0].id;
        std::cout << "origin added" << std::endl;
        std::cout << markers[0].id << std::endl;
      }

      cv::Mat robot_pose = use_transforms(markers, trackers, annotated_frame, config);
    }

    if (config.camera_params.isValid()) {
      for (auto &marker:markers) {
        aruco::CvDrawingUtils::draw3dCube(annotated_frame, marker, config.camera_params);
        aruco::CvDrawingUtils::draw3dAxis(annotated_frame, marker, config.camera_params);
      }
    }

    // show annotated frame
    annotated_source.PutFrame(annotated_frame);

    ++frame_idx;
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

// check if some ID pairs are in the transforms
int inTransforms(int toId, int fromId) {
  for (auto &globalTran : globalTrans) {
    if (toId == globalTran.toId && fromId == globalTran.fromId) {
      return 1;
    }
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
  for (auto &globalTran : globalTrans) {
    if (globalTran.toId == id) {
      ret.push_back(globalTran.fromId);
    }
    if (globalTran.fromId == id) {
      ret.push_back(globalTran.toId);
    }
  }

  return ret;

}

cv::Mat use_transforms(std::vector<aruco::Marker> markers, trackers_map_t trackers, cv::Mat annotated_frame, const config_t config) {
  for (auto &marker : markers) {
    // insert the marker and its tracker if the tag is new
    if (trackers.count(marker.id) == 0) {
      aruco::MarkerPoseTracker marker_tracker;
      trackers.insert({marker.id, marker_tracker});
    }

    // draw to indicate detection
    marker.draw(annotated_frame, cv::Scalar(255, 200, 200), 2);

    // estimate pose and draw frame
    bool success = trackers.at(marker.id).estimatePose(marker, config.camera_params, config.marker_size);

    if (!success) {
      std::cout << "tag " << marker.id << " too far from last tracked pose. Ignoring." << std::endl;
      continue;
    }

    // draw to indicate tracking
    marker.draw(annotated_frame, cv::Scalar(0, 0, 255), 2);

    // transforms loop
    for (auto &other_markers : markers) {

      // add transforms to vector
      // only allow for toID < fromID
      // this way, it avoids duplicates
      if (marker.id < other_markers.id) {

        // check for transforms
        if (!inTransforms(marker.id, other_markers.id)) {
          Trans t;
          t.toId = marker.id;
          t.fromId = other_markers.id;
          t.t = (marker.Tvec - other_markers.Tvec);
          t.r = (marker.Rvec - other_markers.Rvec);
          globalTrans.push_back(t);

          std::cout << "added transforms" << endl;
          std::cout << t.toId << '\t' << t.fromId << std::endl;
        }
      }
    }

    // if the marker is our origin, just report position away
    if (marker.id == orig) {
      cv::Mat pose;
      cv::vconcat(marker.Tvec.t(), marker.Rvec.t(), pose);
      std::cout << pose << std::endl;
    } else {
      // search though through the transforms
      int pathFound = 0;
      int startId = marker.id;
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
          if (allTrans.empty()) {
            pathFound = 1;
            break;
          }

          // build paths from transforms
          std::vector<std::vector<int> > toAppend;
          for (int allTran : allTrans) {

            // no cyclic paths
            if (std::find(paths[k].begin(), paths[k].end(), allTran) == paths[k].end()) {

              // add to paths
              std::vector<int> pathCopy(paths[k]);
              pathCopy.push_back(allTran);
              toAppend.push_back(pathCopy);

              // check if a path from start to goal has been found
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
          paths.insert(paths.end(), toAppend.begin(), toAppend.end());

        }
      }

      if (!foundPath.empty()) {
        std::cout << "using transforms" << std::endl;
        for (int j : foundPath) {
          std::cout << j << '\t';
        }
        std::cout << std::endl;
        float a[6] = {0.f};
        cv::Mat pose;
        cv::vconcat(marker.Tvec.t(), marker.Rvec.t(), pose);
        cv::Mat r(2, 3, CV_32F, a);
        cv::Mat m;
        for (int j = 0; j < foundPath.size() - 1; j++) {
          if (foundPath[j] < foundPath[j + 1]) {
            m = computeTransform(foundPath[j], foundPath[j + 1]);

          } else {
            m = computeTransform(foundPath[j + 1], foundPath[j]);
            m = -1 * m;
          }

          if (m.at<float>(0, 0) > -9999.f) {
            pose = pose + m;
          } else {
            std::cout << "error computing transform \t";
            pose.at<float>(0, 0) = -99999.f;
            break;
          }

        }
        std::cout << pose << std::endl;
      }
    }
  }
}

