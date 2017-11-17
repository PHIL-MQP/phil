#include <Commands/JoystickTeleop.h>
#include <RobotMap.h>
#include <SPI.h>
#include <Subsystems/DriveBase.h>
#include <Victor.h>

DriveBase::DriveBase() :
    frc::Subsystem("DriveBase"), left_motor(nullptr), right_motor(nullptr) {
  left_motor = new frc::Victor(RobotMap::kLeftMotor);
  right_motor = new frc::Victor(RobotMap::kRightMotor);
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
  right_motor->Set(-right_speed);
}

