#include <Commands/JoystickTeleop.h>
#include <RobotMap.h>
#include <SPI.h>
#include <Subsystems/DriveBase.h>
#include <Victor.h>
#include <RobotBase.h>

Wheel::Wheel(const std::string& name, int motor_port, int encoder_port_a, int encoder_port_b) : frc::PIDSubsystem(name, 1, 0, 0) {
  motor = new frc::Talon(motor_port);
  encoder = new frc::Encoder(encoder_port_a, encoder_port_b);
  SetPIDSourceType(frc::PIDSourceType::kRate);
  SmartDashboard::PutData(name, GetPIDController().get());
  LiveWindow::GetInstance()->AddActuator(name, "pid", GetPIDController());
  encoder->SetDistancePerPulse(0.000357);
  Enable();
}

double Wheel::GetRate() {
  return encoder->GetRate();
}

double Wheel::GetMotor() {
  return motor->Get();
}

double Wheel::GetMotorInverted() {
  return motor->GetInverted();
}

void Wheel::InitDefaultCommand() {}

void Wheel::UsePIDOutput(double output) {
  motor->PIDWrite(output);
}

double Wheel::ReturnPIDInput() {
  return encoder->GetRate();
}

DriveBase::DriveBase() : frc::Subsystem("DriveBase") {
  left_encoder = new frc::Encoder(RobotMap::kLeftEnocderA, RobotMap::kLeftEnocderB);
  right_encoder = new frc::Encoder(RobotMap::kRightEnocderA, RobotMap::kRightEnocderB);
  left_motor = new frc::Talon(RobotMap::kLeftMotor);
  right_motor = new frc::Talon(RobotMap::kRightMotor);
  left_pid = new frc::PIDController(1, 0, 0, 1, left_encoder, left_motor);
  left_pid->SetContinuous(true);
  left_pid->SetPIDSourceType(frc::PIDSourceType::kRate);
//  left_pid->Enable();
  right_pid = new frc::PIDController(1, 0, 0, 1, right_encoder, right_motor);
  right_pid->SetContinuous(true);
  right_pid->SetPIDSourceType(frc::PIDSourceType::kRate);
//  right_pid->Enable();
}

void DriveBase::InitDefaultCommand() {
  SetDefaultCommand(new JoystickTeleop());
}

void DriveBase::Stop() {
  left_motor->StopMotor();
  right_motor->StopMotor();
}

void DriveBase::SetSpeed(double left_speed, double right_speed) {
//  left_pid->Disable();
//  right_pid->Disable();
  left_motor->Set(left_speed);
  right_motor->Set(-right_speed);
}
