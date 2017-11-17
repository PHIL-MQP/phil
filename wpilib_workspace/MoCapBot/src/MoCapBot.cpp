#include <iostream>
#include <iomanip>
#include <unistd.h>

#include <CameraServer.h>
#include <Joystick.h>
#include <Timer.h>
#include <Commands/Scheduler.h>
#include <opencv2/imgproc.hpp>
#include <phil/phil.h>
#include <SmartDashboard/SmartDashboard.h>

#include <RobotMap.h>
#include <MoCapBot.h>
#include <AHRS.h>

Joystick *Robot::gamepad = nullptr;
DriveBase *Robot::drive_base = nullptr;
frc::SPI *Robot::tk1_spi = nullptr;
frc::I2C *Robot::tk1_i2c = nullptr;
AHRS *Robot::ahrs = nullptr;
frc::Encoder *Robot::left_encoder = nullptr;
frc::Encoder *Robot::right_encoder = nullptr;
frc::AnalogOutput *Robot::mocap_start_trigger = nullptr;
frc::AnalogOutput *Robot::mocap_stop_trigger = nullptr;

void Robot::RobotInit() {
  std::cout << "RobotInit" << std::endl;

  gamepad = new frc::Joystick(0);
  drive_base = new DriveBase();
  //ahrs = new AHRS(SPI::Port::kMXP);
  ahrs = new AHRS(SerialPort::kMXP); /* Alternatives:  SPI::kMXP, I2C::kMXP or SerialPort::kUSB */
  tk1_spi = new frc::SPI(frc::SPI::Port::kOnboardCS0);
  tk1_i2c = new frc::I2C(frc::I2C::Port::kOnboard, 0);
  left_encoder = new frc::Encoder(RobotMap::kLeftEnocderA,
      RobotMap::kLeftEnocderB);
  right_encoder = new frc::Encoder(RobotMap::kRightEnocderA,
      RobotMap::kRightEnocderB);
  mocap_stop_trigger = new frc::AnalogOutput(RobotMap::kTriggerStop);
  mocap_start_trigger = new frc::AnalogOutput(RobotMap::kTriggerStart);

  mocap_start_trigger->SetVoltage(5);
  mocap_stop_trigger->SetVoltage(5);
  running = false;

  tk1_spi->SetClockRate(500000);
  tk1_spi->SetMSBFirst();
  tk1_spi->SetSampleDataOnFalling();
  tk1_spi->SetClockActiveLow();
  tk1_spi->SetChipSelectActiveHigh();
}

void Robot::TeleopInit() {
  // create the log to start capturing data
  std::cout << "TeleopInit" << std::endl;
  log.open("/home/lvuser/mocap_imu_encoder_data.csv");
  if(!log) {
    std::cout << strerror(errno) << '\n';
  }

  if (!log.good()) {
    std::cout << "log !good()" << std::endl;
  }

  log
      << "accel_x,accel_y,accel_z,"
      << "gyro_x,gyro_y,gyro_z,"
      << "mag_x,mag_y,mag_z,"
      << "left_encoder_rate,right_encoder_rate,"
      << "t0,t1"
      << std::endl;

  // tell the TK1 to start recording data
  uint8_t data = 1;
  std::cout << "Signaling TK1" << std::endl;
  phil::Phil::GetInstance()->SendUDPToTK1(&data, 1, nullptr, 0);

  // tell the motion capture to start
  std::cout << "Triggering Motion Capture" << std::endl;
  Robot::mocap_start_trigger->SetVoltage(0);
  Robot::mocap_stop_trigger->SetVoltage(5);
  running = true;
}

void Robot::DisabledInit() {
  std::cout << "DisabledInit" << std::endl;
  if (running) {
    Robot::mocap_stop_trigger->SetVoltage(0);
    Robot::mocap_start_trigger->SetVoltage(5);
    running = false;
  }

  if (log.is_open()) {
    log.close();
  }
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

    buffer_idx = 0;
    for (data_t sample : buffer) {
      log << std::setw(6) << sample.raw_accel_x << "," << sample.raw_accel_y
          << "," << sample.raw_accel_z << "," << sample.raw_gyro_x << ","
          << sample.raw_gyro_y << "," << sample.raw_gyro_z << "," << sample.t
          << std::endl;
    }
    log.flush();

  }
}

START_ROBOT_CLASS(Robot)
