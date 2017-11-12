#include <iostream>
#include <unistd.h>

#include <CameraServer.h>
#include <Joystick.h>
#include <Timer.h>
#include <Commands/Scheduler.h>
#include <opencv2/imgproc.hpp>
#include <SmartDashboard/SmartDashboard.h>

#include <RobotMap.h>
#include <MoCapBot.h>


Joystick *Robot::gamepad = nullptr;
DriveBase *Robot::drive_base = nullptr;
frc::SPI *Robot::tk1_spi = nullptr;
AHRS *Robot::ahrs = nullptr;
frc::Encoder *Robot::left_encoder = nullptr;
frc::Encoder *Robot::right_encoder = nullptr;


void Robot::RobotInit() {
  std::cout << "RobotInit" << std::endl;

  gamepad = new frc::Joystick(0);
  drive_base = new DriveBase();
  ahrs = new AHRS(SPI::Port::kMXP);
  tk1_spi = new frc::SPI(frc::SPI::Port::kOnboardCS0);
  left_encoder = new frc::Encoder(RobotMap::kLeftEnocderA, RobotMap::kLeftEnocderB);
  right_encoder = new frc::Encoder(RobotMap::kRightEnocderA, RobotMap::kRightEnocderB);

  tk1_spi->SetClockRate(500000);
  tk1_spi->SetMSBFirst();
  tk1_spi->SetSampleDataOnFalling();
  tk1_spi->SetClockActiveLow();
  tk1_spi->SetChipSelectActiveHigh();

}

void Robot::TeleopInit() {
  std::cout << "TeleopInit" << std::endl;
}

void Robot::TeleopPeriodic() {
  frc::Scheduler::GetInstance()->Run();

  // An important question is what data from the NavX should we bag? (which functions do we call)
  struct data_t {
    double raw_accel_x;
    double raw_accel_y;
    double raw_accel_z;
    double raw_gyro_x;
    double raw_gyro_y;
    double raw_gyro_z;
    double raw_mag_x;
    double raw_mag_y;
    double raw_mag_z;
    double left_encoder_rate;
    double right_encoder_rate;
    double t;
  } data;

  double t0 = frc::Timer::GetFPGATimestamp();
  data.raw_accel_x = ahrs->GetRawAccelX();
  data.raw_accel_y = ahrs->GetRawAccelY();
  data.raw_accel_z = ahrs->GetRawAccelZ();
  data.raw_gyro_x = ahrs->GetRawGyroX();
  data.raw_gyro_y = ahrs->GetRawGyroY();
  data.raw_gyro_z = ahrs->GetRawGyroZ();
  data.raw_mag_x = ahrs->GetRawMagX();
  data.raw_mag_y = ahrs->GetRawMagY();
  data.raw_mag_z = ahrs->GetRawMagZ();
  data.left_encoder_rate = left_encoder->GetRate();
  data.right_encoder_rate = right_encoder->GetRate();
  data.t = frc::Timer::GetFPGATimestamp();


  uint8_t *raw_data = (uint8_t *) &data;
  int size = sizeof(data_t);
  tk1_spi->Write(raw_data, size);
//  uint8_t test = 170;
//  tk1_spi->Write(&test, 1);
}


START_ROBOT_CLASS(Robot)
