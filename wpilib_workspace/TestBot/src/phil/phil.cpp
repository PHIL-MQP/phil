#include <array>

#include <phil/phil.h>

namespace phil {

Phil *Phil::instance = nullptr;

Phil::Phil() :
    left_encoder(nullptr), right_encoder(nullptr), ahrs(nullptr) {
  table = NetworkTable::GetTable(phil::kTableName);
}

Phil *Phil::GetInstance() {
  if (instance == nullptr) {
    instance = new Phil();
  }

  return instance;
}

void Phil::GiveSensors(Encoder *left_encoder, Encoder *right_encoder, AHRS *ahrs) {
  this->left_encoder = left_encoder;
  this->right_encoder = right_encoder;
  this->ahrs = ahrs;
}

void Phil::ReadSensorsAndProcessLocally() {
  // Read the sensors
  double wl = left_encoder->GetRate();
  double wr = right_encoder->GetRate();
  double ax = ahrs->GetRawAccelX();
  double ay = ahrs->GetRawAccelY();
  double az = ahrs->GetRawAccelZ();
  double gx = ahrs->GetRawGyroX();
  double gy = ahrs->GetRawGyroY();
  double gz = ahrs->GetRawGyroZ();

  // do math here...

  // post to network tables
  std::vector<double> pose = { 0, 0, 0 };
  table->PutNumberArray(llvm::StringRef(phil::kPoseKey), llvm::ArrayRef<double>(pose));
}

void Phil::ReadSensorsAndProcessOnTK1() {
  const std::vector<double> encoders_data = { left_encoder->GetRate(), right_encoder->GetRate(), frc::GetTime() };

  const std::vector<double> ins_data = { ahrs->GetRawAccelX(), ahrs->GetRawAccelY(), ahrs->GetRawAccelZ(),
      ahrs->GetRawGyroX(), ahrs->GetRawGyroY(), ahrs->GetRawGyroZ(), frc::GetTime() };

  table->PutNumberArray(llvm::StringRef(phil::kEncodersKey), llvm::ArrayRef<double>(encoders_data));
  table->PutNumberArray(llvm::StringRef(phil::kINSKey), llvm::ArrayRef<double>(ins_data));
}

phil::pose_t Phil::GetPosition() {
  pose_t pose;

  std::vector<double> defult_value = { 999, 999, 999, };
  std::vector<double> pose_ref = table->GetNumberArray(llvm::StringRef(phil::kPoseKey),
      llvm::ArrayRef<double>(defult_value));

  return pose;
}

}
