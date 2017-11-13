#include <iostream>
#include <iomanip>
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
frc::I2C *Robot::tk1_i2c = nullptr;
AHRS *Robot::ahrs = nullptr;
frc::Encoder *Robot::left_encoder = nullptr;
frc::Encoder *Robot::right_encoder = nullptr;

void Robot::RobotInit() {
  std::cout << "RobotInit" << std::endl;

  gamepad = new frc::Joystick(0);
  drive_base = new DriveBase();
  ahrs = new AHRS(SPI::Port::kMXP);
  tk1_spi = new frc::SPI(frc::SPI::Port::kOnboardCS0);
  tk1_i2c = new frc::I2C(frc::I2C::Port::kOnboard, 0);
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
  log.open("ins_data.csv");
  log << "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,left_encoder_rate,right_encoder_rate,t0,t1" << std::endl;
}

void Robot::DisabledPeriodic() {
  log.close();
}

void Robot::TeleopPeriodic() {
  frc::Scheduler::GetInstance()->Run();

  if (buffer_idx < buff_size) {
    data_t sample;
    sample.raw_accel_x = ahrs->GetRawAccelX();
    sample.raw_accel_y = ahrs->GetRawAccelY();
    sample.raw_accel_z = ahrs->GetRawAccelZ();
    sample.raw_gyro_x = ahrs->GetRawGyroX();
    sample.raw_gyro_y = ahrs->GetRawGyroY();
    sample.raw_gyro_z = ahrs->GetRawGyroZ();
    sample.raw_mag_x = ahrs->GetRawMagX();
    sample.raw_mag_y = ahrs->GetRawMagY();
    sample.raw_mag_z = ahrs->GetRawMagZ();
    sample.left_encoder_rate = left_encoder->GetRate();
    sample.right_encoder_rate = right_encoder->GetRate();
    sample.t = frc::Timer::GetFPGATimestamp();

    buffer[buffer_idx] = sample;

    buffer_idx++;
  } else {
    // copy contents of buffer to disc
    std::cout << "copying to disc..." << std::endl;
    if (!log.good()) {
      std::cout << "fuck" << std::endl;
    }
    buffer_idx = 0;
    for (data_t sample : buffer) {
      std::cout << std::setw(6)
          << sample.raw_accel_x << ","
          << sample.raw_accel_y << ","
          << sample.raw_accel_z << ","
          << sample.raw_gyro_x << ","
          << sample.raw_gyro_y << ","
          << sample.raw_gyro_z << ","
          << sample.t << std::endl;
      log << std::setw(6)
          << sample.raw_accel_x << ","
          << sample.raw_accel_y << ","
          << sample.raw_accel_z << ","
          << sample.raw_gyro_x << ","
          << sample.raw_gyro_y << ","
          << sample.raw_gyro_z << ","
          << sample.t << std::endl;
    }
    log.flush();

  }

//  uint8_t *raw_data = (uint8_t *)&data;
//  tk1_spi->Write(raw_data, 12);
//  tk1_i2c->WriteBulk(raw_data, 12);
//
//  double x = SmartDashboard::GetNumber("x", -1);
//  SmartDashboard::PutNumber("y", 2.1);
//  std::cout << x << std::endl;
}

START_ROBOT_CLASS(Robot)
