#pragma once

#include <llvm/StringRef.h>
#include "udp.h"

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

}
