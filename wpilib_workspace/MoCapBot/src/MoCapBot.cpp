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
AHRS *Robot::ahrs = nullptr;
frc::Encoder *Robot::left_encoder = nullptr;
frc::Encoder *Robot::right_encoder = nullptr;
frc::AnalogOutput *Robot::mocap_start_trigger = nullptr;
frc::AnalogOutput *Robot::mocap_stop_trigger = nullptr;

void Robot::RobotInit() {
  std::cout << "RobotInit" << std::endl;

  gamepad = new frc::Joystick(0);
  drive_base = new DriveBase();
  ahrs = new AHRS(frc::I2C::kMXP);
  left_encoder = new frc::Encoder(RobotMap::kLeftEnocderA,
      RobotMap::kLeftEnocderB);
  right_encoder = new frc::Encoder(RobotMap::kRightEnocderA,
      RobotMap::kRightEnocderB);
  mocap_stop_trigger = new frc::AnalogOutput(RobotMap::kTriggerStop);
  mocap_start_trigger = new frc::AnalogOutput(RobotMap::kTriggerStart);

  mocap_start_trigger->SetVoltage(5);
  mocap_stop_trigger->SetVoltage(5);
  running = false;
}

void Robot::TeleopInit() {
  // create the log to start capturing data
  std::cout << "TeleopInit" << std::endl;

  std::ostringstream filename;
  filename << "/home/lvuser/mocap_data-" << frc::Timer::GetFPGATimestamp() << ".csv";
  log.open(filename.str());

  if (!log) {
    std::cout << strerror(errno) << '\n';
  }

  if (!log.good()) {
    std::cout << "log !good()" << std::endl;
  }

  log << "accel_x,accel_y,accel_z,"
      << "gyro_x,gyro_y,gyro_z,"
      << "x,y,z,"
      << "left_encoder_rate,right_encoder_rate,"
      << "left_input,right_input,"
      << "fpga time,navx time"
      << std::endl;

  // tell the TK1 to start recording data
  uint8_t data = 1;
  std::cout << "Starting TK1" << std::endl;
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

  // tell the TK1 to stop recording data
  uint8_t data = 0;
  std::cout << "Stopping TK1" << std::endl;
  phil::Phil::GetInstance()->SendUDPToTK1(&data, 1, nullptr, 0);
}

void Robot::TeleopPeriodic() {
  frc::Scheduler::GetInstance()->Run();

  data_t sample = {0};
  sample.raw_accel_x = ahrs->GetRawAccelX();
  sample.raw_accel_y = ahrs->GetRawAccelY();
  sample.raw_accel_z = ahrs->GetRawAccelZ();
  sample.raw_gyro_x = ahrs->GetRawGyroX();
  sample.raw_gyro_y = ahrs->GetRawGyroY();
  sample.raw_gyro_z = ahrs->GetRawGyroZ();
  sample.raw_mag_x = ahrs->GetRawMagX();
  sample.raw_mag_y = ahrs->GetRawMagY();
  sample.raw_mag_z = ahrs->GetRawMagZ();
  sample.x = ahrs->GetDisplacementX();
  sample.y = ahrs->GetDisplacementY();
  sample.z = ahrs->GetDisplacementZ();
  sample.left_encoder_rate = left_encoder->GetRate();
  sample.right_encoder_rate = right_encoder->GetRate();
  sample.left_input = gamepad->GetRawAxis(1);
  sample.right_input = -gamepad->GetRawAxis(5);
  sample.fpga_t = frc::Timer::GetFPGATimestamp();
  sample.navx_t = ahrs->GetLastSensorTimestamp();
  sample.left_motor = drive_base->left_motor->Get();
  sample.right_motor = drive_base->right_motor->GetInverted();

  std::cout
	  << "["
	  << std::setw(6)
      << sample.raw_accel_x << "," << sample.raw_accel_y << "," << sample.raw_accel_z
      << "," << sample.raw_gyro_x << "," << sample.raw_gyro_y << "," << sample.raw_gyro_z
      << "," << sample.x << "," << sample.y << "," << sample.z
      << "," << sample.left_encoder_rate << "," << sample.right_encoder_rate
	  << "," << sample.left_input << "," << sample.right_input
      << "," << sample.fpga_t
      << "," << sample.navx_t
	  << "]"
      << std::endl;

  log << std::setw(6)
      << sample.raw_accel_x << "," << sample.raw_accel_y << "," << sample.raw_accel_z
      << "," << sample.raw_gyro_x << "," << sample.raw_gyro_y << "," << sample.raw_gyro_z
      << "," << sample.x << "," << sample.y << "," << sample.z
      << "," << sample.left_encoder_rate << "," << sample.right_encoder_rate
	  << "," << sample.left_input << "," << sample.right_input
      << "," << sample.fpga_t
      << "," << sample.navx_t
      << std::endl;
}

START_ROBOT_CLASS(Robot)
