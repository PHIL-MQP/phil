#pragma once

#include <cmath>
#include <ostream>

#include <llvm/StringRef.h>

#include <opencv2/core/mat.hpp>
#include <iostream>

namespace phil {

const llvm::StringRef kTableName("phil_table");
const llvm::StringRef kEncodersKey("encoders");
const llvm::StringRef kINSKey("ins");
const llvm::StringRef kPoseKey("pose");
const llvm::StringRef kWheelRadius("wheel_radius");
const llvm::StringRef kTrackWidth("track_width");
const llvm::StringRef kMotor1Inverted("motor_1_inverted");
const llvm::StringRef kMotor2Inverted("motor_2_inverted");

struct pose_t {
  double x;
  double y;
  double theta;
};

/**
 * Computes shortest angle between two angles yaw1 - yaw2 safely, such that yaw_diff_rad(0.1,2*M_PI - 0.1) == 0.2.
 *
 * @param yaw1
 * @param yaw2
 * @return
 */
inline double yaw_diff_rad(double yaw1, double yaw2) {
  double diff = yaw1 - yaw2;
  if (diff > M_PI) { diff -= M_PI * 2; };
  if (diff < -M_PI) { diff += M_PI * 2; };
  return diff;
}

inline double yaw_diff_deg(double yaw1, double yaw2) {
  double diff = yaw1 - yaw2;
  if (diff > 180) { diff -= 360; };
  if (diff < -180) { diff += 360; };
  return diff;
}

inline pose_t MatrixTo3Pose(cv::Mat rt_matrix) {
  pose_t pose{0, 0, 0};
  pose.x = rt_matrix.at<double>(0, 3);
  pose.y = rt_matrix.at<double>(1, 3);
  auto r32 = rt_matrix.at<double>(3, 2);
  auto r33 = rt_matrix.at<double>(3, 3);
  pose.theta = atan2(r32, r33);
  return pose;
}

extern const std::string red;
extern const std::string green;
extern const std::string yellow;
extern const std::string cyan;
extern const std::string magenta;
extern const std::string reset;
}
