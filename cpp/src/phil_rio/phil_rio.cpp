#include <array>
#include <iostream>

#include <phil/phil_rio/phil_rio.h>
#include <SmartDashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

namespace phil {

Phil *Phil::instance = nullptr;

// TODO: don't hard code main hostname
Phil::Phil() :
    left_encoder(nullptr), right_encoder(nullptr), ahrs(nullptr), udp_client("raspberrypi.local", phil::kPort), tk1_time_offset(0) {
  auto inst = nt::NetworkTableInstance::GetDefault();
  table = inst.GetTable(phil::kTableName);

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 100000;
  udp_client.SetTimeout(timeout);
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

void Phil::ReadSensorsAndProcessRemotely() {
  data_t data = {0};
  struct timeval t0 = {};
  gettimeofday(&t0, nullptr);
  data.rio_send_time_s = timeval_to_sec(t0) + tk1_time_offset;
  data.raw_acc_x = ahrs->GetRawAccelX();
  data.raw_acc_y = ahrs->GetRawAccelY();
  data.raw_acc_z = ahrs->GetRawAccelZ();
  data.navx_t = ahrs->GetLastSensorTimestamp();
  data.fpga_t = Timer::GetFPGATimestamp();
  data.yaw = ahrs->GetYaw();
  data.world_acc_x = ahrs->GetWorldLinearAccelX();
  data.world_acc_y = ahrs->GetWorldLinearAccelY();
  data.left_encoder_rate = left_encoder->GetRate();
  data.right_encoder_rate = right_encoder->GetRate();

  data_t response_data = udp_client.Transaction(data);

  struct timeval t1 = {};
  gettimeofday(&t1, nullptr);
  double rio_return_time_s = timeval_to_sec(t1);
  double half_rtt = (rio_return_time_s - response_data.rio_send_time_s) / 2.0;
  tk1_time_offset = rio_return_time_s - (response_data.received_time_s + half_rtt);
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

void Phil::SendUDPTo(std::string hostname,
                     uint8_t *request,
                     size_t request_size,
                     uint8_t *response,
                     size_t response_size,
                     uint16_t udp_port) {
  UDPClient temp_udp_client(hostname, udp_port);
  temp_udp_client.RawTransaction(request, request_size, response, response_size);
}

void Phil::SendUDPToPI(uint8_t *request, size_t request_size, uint8_t *response, size_t response_size) {
  udp_client.RawTransaction(request, request_size, response, response_size);
}

} // end namespace
