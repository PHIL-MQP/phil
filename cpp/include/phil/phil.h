#pragma  once

#pragma once

#include <memory>
#include <string>

#include <AHRS.h>
#include <Encoder.h>
#include <llvm/StringRef.h>

namespace phil {

const llvm::StringRef kTableName("phil_table");
const llvm::StringRef kEncodersKey("encoders");
const llvm::StringRef kINSKey("ins");
const llvm::StringRef kPoseKey("pose");
const llvm::StringRef kWheelRadius("wheel_radius");
const llvm::StringRef kTrackWidth("track_width");

struct pose_t {
  double x;
  double y;
  double phi;
};

class Phil {
 private:
  static Phil *instance;

  Phil();

  frc::Encoder *left_encoder;
  frc::Encoder *right_encoder;
  AHRS *ahrs;
  std::shared_ptr<NetworkTable> table;

 public:
  static Phil *GetInstance();

  void GiveSensors(frc::Encoder *left_encoder, frc::Encoder *right_encoder, AHRS *ahrs);

  /**
   * For when we are only doing IMU and Encoders, we will use this method.
   */
  void ReadSensorsAndProcessLocally();

  /**
   * For when we have a camera being processed on a co-processor
   */
  void ReadSensorsAndProcessOnTK1();

  /**
   * Literally just reads the value of network tables and returns it
   */
  pose_t GetPosition();
};

}

