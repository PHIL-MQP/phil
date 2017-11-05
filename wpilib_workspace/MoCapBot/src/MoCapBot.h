#include <Subsystems/DriveBase.h>
#include <IterativeRobot.h>
#include <Joystick.h>

class Robot: public frc::IterativeRobot {

public:
  static void VisionThread();

  void RobotInit() override;

  void TeleopInit() override;

  void TeleopPeriodic() override;

  static frc::Joystick *gamepad;
  static DriveBase *drive_base;
};
