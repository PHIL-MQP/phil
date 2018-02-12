#include <iostream>

#include <WPILib.h>
#include <phil/phil_rio/phil_rio.h>

class Robot : public frc::IterativeRobot {

  phil::Phil *phil = phil::Phil::GetInstance();

  void RobotInit() override {
    std::cout << "RobotInit" << std::endl;
  }

  void TeleopInit() override {
    std::cout << "TeleopInit" << std::endl;
  }

  void TeleopPeriodic() override {
    frc::Scheduler::GetInstance()->Run();
    uint8_t data = 0;
    phil->SendUDPTo("fake.local", &data, 1, nullptr, 0);
    phil::pose_t pose = phil::Phil::GetInstance()->GetPosition();
  }
};

START_ROBOT_CLASS(Robot)
