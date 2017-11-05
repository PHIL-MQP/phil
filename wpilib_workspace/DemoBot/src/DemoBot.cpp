#include <iostream>

#include <Commands/Scheduler.h>
#include <phil/phil.h>

#include <DemoBot.h>

frc::Joystick *Robot::gamepad = nullptr;
DriveBase *Robot::drive_base = nullptr;

void Robot::RobotInit() {
  std::cout << "RobotInit" << std::endl;

  gamepad = new frc::Joystick(0);
  drive_base = new DriveBase();
}

void Robot::TeleopInit() {
  std::cout << "TeleopInit" << std::endl;
}

void Robot::TeleopPeriodic() {
  frc::Scheduler::GetInstance()->Run();

  phil::Phil::GetInstance()->ReadSensorsAndProcessLocally();
//  phil::Phil::GetInstance()->ReadSensorsAndProcessOnTK1();

  phil::pose_t pose = phil::Phil::GetInstance()->GetPosition();
}


START_ROBOT_CLASS(Robot)
