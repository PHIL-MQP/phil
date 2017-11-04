#include <Subsystems/DriveBase.h>
#include <Victor.h>
#include <RobotMap.h>

DriveBase::DriveBase() :
		frc::Subsystem("DriveBase") {
	left_motor = new frc::Victor(RobotMap::kLeftMotor);
	right_motor = new frc::Victor(RobotMap::kRightMotor);

	left_encoder = new frc::Encoder(RobotMap::kLeftEnocderA, RobotMap::kLeftEnocderB);
	right_encoder = new frc::Encoder(RobotMap::kRightEnocderA, RobotMap::kRightEnocderB);
}

void DriveBase::InitDefaultCommand() {
}

void DriveBase::Stop() {
	left_motor->StopMotor();
	right_motor->StopMotor();
}

void DriveBase::SetSpeed(double left_speed, double right_speed) {
	left_motor->Set(left_speed);
	right_motor->Set(left_speed);
}

