#pragma once

#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <cscore.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

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

typedef std::map<int, aruco::MarkerPoseTracker> trackers_map_t;

void show_help();

std::vector<int> getAdjacentNodes(int id);

cv::Mat computeTransform(int idTo, int idFrom);

int inTransforms(int toId, int fromId);
