#pragma once

#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>

struct Trans {
  int toId;
  int fromId;
  cv::Mat t;
  cv::Mat r;
};

struct config_t {
  aruco::CameraParameters camera_params;
  float marker_size = 0.f;
};

void show_help();

typedef std::map<int, aruco::MarkerPoseTracker> trackers_map_t;

cv::Mat use_transforms(std::vector<aruco::Marker> markers, trackers_map_t trackers, cv::Mat annotated_frame, const config_t config) ;

std::vector<int> getAdjacentNodes(int id);
cv::Mat computeTransform(int idTo, int idFrom);
int inTransforms(int toId, int fromId);
