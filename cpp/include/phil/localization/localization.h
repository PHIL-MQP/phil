#pragma once

#include <phil/common/common.h>
#include <opencv2/core.hpp>

namespace phil {

pose_t compute_pose(double time_s, cv::Mat frame, phil::data_t rio_data);

}
