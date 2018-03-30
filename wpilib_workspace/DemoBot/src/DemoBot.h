#include <Subsystems/DriveBase.h>
#include <IterativeRobot.h>
#include <Joystick.h>

class Robot: public frc::IterativeRobot {

public:
  void RobotInit() override;

  void TeleopInit() override;

  void TeleopPeriodic() override;

  void DisabledInit() override;

  static frc::Joystick *gamepad;
  static DriveBase *drive_base;
};
