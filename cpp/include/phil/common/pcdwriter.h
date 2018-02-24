#pragma once

#include <map>
#include <string>
#include <aruco/markermap.h>

void savePCDFile(std::string fpath, const aruco::MarkerMap& ms, std::map<int, cv::Mat> frame_pose_map);
