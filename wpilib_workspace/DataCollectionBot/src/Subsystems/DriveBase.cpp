#include <Victor.h>
#include <HAL/HAL.h>

#include <RobotMap.h>
#include <Commands/JoystickTeleop.h>
#include <Subsystems/DriveBase.h>

DriveBase::DriveBase() :
		frc::Subsystem("DriveBase") {
}

void DriveBase::Init() {
	left_motor = new frc::Victor(RobotMap::kLeftMotor);
	right_motor = new frc::Victor(RobotMap::kRightMotor);

	left_encoder = new frc::Encoder(RobotMap::kLeftEnocderA, RobotMap::kLeftEnocderB);
	right_encoder = new frc::Encoder(RobotMap::kRightEnocderA, RobotMap::kRightEnocderB);
}

void DriveBase::InitDefaultCommand() {
  SetDefaultCommand(new JoystickTeleop());
}

void DriveBase::Stop() {
	left_motor->StopMotor();
	right_motor->StopMotor();
}

void DriveBase::SetSpeed(double left_speed, double right_speed) {
	left_motor->Set(left_speed);
	right_motor->Set(right_speed);
}

