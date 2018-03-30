/**
 * This program lets your drive the robot around and publish the required encoder data to the co-processor.
 *  If you also run phil_main on your co-processor,
 *  you will get a position estimate which you can view in Shuffleboard (hypothetically).
 */
#include <iostream>

#include <Commands/Scheduler.h>
#include <AnalogOutput.h>
#include <phil/phil_rio/phil_rio.h>

#include <DemoBot.h>

frc::Joystick *Robot::gamepad = nullptr;
DriveBase *Robot::drive_base = nullptr;
frc::AnalogOutput *mocap_start_trigger = nullptr;
frc::AnalogOutput *mocap_stop_trigger = nullptr;
constexpr int kTriggerStart = 0;
constexpr int kTriggerStop = 1;

void Robot::RobotInit() {
	std::cout << "RobotInit" << std::endl;

	gamepad = new frc::Joystick(0);
	drive_base = new DriveBase();

	mocap_stop_trigger = new frc::AnalogOutput(kTriggerStop);
	mocap_start_trigger = new frc::AnalogOutput(kTriggerStart);

	mocap_start_trigger->SetVoltage(5);
	mocap_stop_trigger->SetVoltage(5);

}

void Robot::TeleopInit() {
	std::cout << "TeleopInit" << std::endl;

	std::cout << "Starting Motion Capture" << std::endl;
	mocap_start_trigger->SetVoltage(0);
	mocap_stop_trigger->SetVoltage(5);
}

void Robot::TeleopPeriodic() {
	frc::Scheduler::GetInstance()->Run();

	phil::Phil::GetInstance()->ReadSensorsAndProcessRemotely();

//	phil::pose_t pose = phil::Phil::GetInstance()->GetPosition();

	std::cout  << phil::Phil::GetInstance()->ahrs->GetRawAccelX() << ", "
			<< phil::Phil::GetInstance()->ahrs->GetRawAccelY() << ", "
			<< phil::Phil::GetInstance()->ahrs->GetYaw() << ", "
			<< phil::Phil::GetInstance()->left_encoder->GetRate() << ", "
			<< phil::Phil::GetInstance()->right_encoder->GetRate() << std::endl;
}

void Robot::DisabledInit() {
	std::cout << "Stopping Motion Capture" << std::endl;
	mocap_start_trigger->SetVoltage(5);
	mocap_stop_trigger->SetVoltage(0);
}

START_ROBOT_CLASS(Robot)
