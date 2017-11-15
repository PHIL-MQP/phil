#pragma once

#include <fstream>

#include <Subsystems/DriveBase.h>
#include <IterativeRobot.h>
#include <Joystick.h>

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
};

class Robot: public frc::IterativeRobot {

public:
  static void VisionThread();

  void RobotInit() override;

  void TeleopInit() override;

  void DisabledPeriodic() override;

  void TeleopPeriodic() override;

  static constexpr size_t buff_size = 500;
  data_t buffer[buff_size];
  size_t buffer_idx = 0;
  std::ofstream log;

  static frc::Joystick *gamepad;
  static DriveBase *drive_base;
  static frc::SPI *tk1_spi;
  static frc::I2C *tk1_i2c;
	static AHRS *ahrs;
	static frc::Encoder *left_encoder;
	static frc::Encoder *right_encoder;
	static frc::DigitalOutput *mocap_start_trigger;
	static frc::DigitalOutput *mocap_stop_trigger;
};
