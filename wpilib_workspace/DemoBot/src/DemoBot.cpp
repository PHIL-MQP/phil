/**
 * This program lets your drive the robot around and publish the required encoder data to the co-processor.
 *  If you also run phil_main on your co-processor,
 *  you will get a position estimate which you can view in Shuffleboard (hypothetically).
 */
#include <iostream>

#include <Commands/Scheduler.h>
#include <phil/phil_rio/phil_rio.h>

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

	phil::Phil::GetInstance()->ReadSensorsAndProcessRemotely();

	phil::pose_t pose = phil::Phil::GetInstance()->GetPosition();

	std::cout  << phil::Phil::GetInstance()->ahrs->GetRawAccelX() << ", "
			<< phil::Phil::GetInstance()->ahrs->GetRawAccelY() << ", "
			<< phil::Phil::GetInstance()->ahrs->GetYaw() << ", "
			<< phil::Phil::GetInstance()->left_encoder->GetRate() << ", "
			<< phil::Phil::GetInstance()->right_encoder->GetRate();
}

START_ROBOT_CLASS(Robot)
