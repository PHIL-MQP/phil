#include <iostream>

#include <WPILib.h>
#include <phil/phil.h>

class Robot : public frc::IterativeRobot {
  void RobotInit() override {
    std::cout << "RobotInit" << std::endl;
  }

  void TeleopInit() override {
    std::cout << "TeleopInit" << std::endl;
  }

  void TeleopPeriodic() override {
    frc::Scheduler::GetInstance()->Run();
    phil::pose_t pose = phil::Phil::GetInstance()->GetPosition();
  }
};

START_ROBOT_CLASS(Robot)
