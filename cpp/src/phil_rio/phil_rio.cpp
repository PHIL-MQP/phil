#include <array>
#include <iostream>

#include <phil/phil_rio/phil_rio.h>
#include <SmartDashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <phil/localization/localization.h>

namespace phil {

Phil *Phil::instance = nullptr;

// TODO: don't hard code main hostname
Phil::Phil() :
    left_encoder(nullptr), right_encoder(nullptr), ahrs(nullptr), udp_client("phil-tk1.local"), tk1_time_offset(0) {
  auto inst = nt::NetworkTableInstance::GetDefault();
  table = inst.GetTable(phil::kTableName);

  // initialize the table -- THIS IS BAD DON'T DO THIS
  frc::SmartDashboard::PutNumber(phil::kWheelRadius, 0.038);
  frc::SmartDashboard::PutNumber(phil::kTrackWidth, 0.23);
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
  // read configuration values
  double wheel_radius = frc::SmartDashboard::GetNumber(phil::kWheelRadius, -1);
  double track_width = frc::SmartDashboard::GetNumber(phil::kTrackWidth, -1);
  bool motor_1_inverted = frc::SmartDashboard::GetBoolean(phil::kMotor1Inverted, false);
  bool motor_2_inverted = frc::SmartDashboard::GetBoolean(phil::kMotor2Inverted, true);

  // Read the sensors
  phil::data_t rio_data = {0};
  rio_data.raw_accel_x = ahrs->GetRawAccelX();
  rio_data.raw_accel_y = ahrs->GetRawAccelY();
  rio_data.raw_accel_z = ahrs->GetRawAccelZ();
  rio_data.raw_gyro_x = ahrs->GetRawGyroX();
  rio_data.raw_gyro_y = ahrs->GetRawGyroY();
  rio_data.raw_gyro_z = ahrs->GetRawGyroZ();
  rio_data.raw_mag_x = ahrs->GetRawMagX();
  rio_data.raw_mag_y = ahrs->GetRawMagY();
  rio_data.raw_mag_z = ahrs->GetRawMagZ();
  rio_data.navx_x = ahrs->GetDisplacementX();
  rio_data.navx_y = ahrs->GetDisplacementY();
  rio_data.navx_z = ahrs->GetDisplacementZ();
  rio_data.left_encoder_rate = left_encoder->GetRate();
  rio_data.right_encoder_rate = right_encoder->GetRate();
  rio_data.fpga_t = frc::Timer::GetFPGATimestamp();
  rio_data.navx_t = ahrs->GetLastSensorTimestamp();
  rio_data.left_motor = motor_1_inverted * left_motor->Get();
  rio_data.right_motor = motor_2_inverted * right_motor->Get();

  cv::Mat no_image;
  pose_t pose = phil::compute_pose(0, no_image, rio_data);

  // post to network tables
  table->PutNumberArray(phil::kPoseKey, llvm::ArrayRef<double>({pose.x, pose.y, pose.theta}));
}

void Phil::ReadSensorsAndProcessOnTK1() {
  data_t data;
  struct timeval t0 = {};
  gettimeofday(&t0, nullptr);
  data.rio_send_time_s = timeval_to_sec(t0) + tk1_time_offset;
  data.left_encoder_rate = left_encoder->GetRate();
  data.right_encoder_rate = right_encoder->GetRate();

  std::cout << "the co-processor will do math here..." << std::endl;
  data_t response_data = udp_client.Transaction(data);

  struct timeval t1 = {};
  gettimeofday(&t1, nullptr);
  double rio_return_time_s = timeval_to_sec(t1);
  double half_rtt = (rio_return_time_s - response_data.rio_send_time_s) / 2.0;
  tk1_time_offset = rio_return_time_s - (response_data.tk1_recv_time_s + half_rtt);
}

phil::pose_t Phil::GetPosition() {
  pose_t pose = {0};

  std::vector<double> defult_value = {999, 999, 999,};
  std::vector<double> pose_ref = table->GetNumberArray(phil::kPoseKey,
                                                       llvm::ArrayRef<double>(defult_value));

  pose.x = pose_ref.at(0);
  pose.y = pose_ref.at(1);
  pose.theta = pose_ref.at(2);

  return pose;
}

void Phil::SendUDPToTK1(uint8_t *request, size_t request_size, uint8_t *response, size_t response_size) {
  udp_client.RawTransaction(request, request_size, response, response_size);
}

} // end namespace
