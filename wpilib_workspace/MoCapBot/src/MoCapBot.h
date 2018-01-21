#pragma once

#include <fstream>

#include <Subsystems/DriveBase.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <AnalogOutput.h>

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
  double x;
  double y;
  double z;
  double left_input;
  double right_input;
  double left_encoder_rate;
  double right_encoder_rate;
  double left_motor;
  double right_motor;
  double fpga_t;
  long navx_t;
};

class Robot: public frc::IterativeRobot {

public:
  static void VisionThread();

  void RobotInit() override;

  void TeleopInit() override;

  void DisabledInit() override;

  void TeleopPeriodic() override;

  void TestPeriodic() override;

  bool running = false;
  std::ofstream log;

  static frc::Joystick *gamepad;
  static DriveBase *drive_base;
	static AHRS *ahrs;
	static frc::AnalogOutput *mocap_start_trigger;
	static frc::AnalogOutput *mocap_stop_trigger;
};
