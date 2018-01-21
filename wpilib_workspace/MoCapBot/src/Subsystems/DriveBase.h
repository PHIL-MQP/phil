#pragma once

#include <string>

#include <PIDController.h>
#include <Talon.h>
#include <Encoder.h>
#include <Commands/Subsystem.h>
#include <Commands/PIDSubsystem.h>
#include <AHRS.h>

class Wheel: public frc::PIDSubsystem {
public:
  Wheel(const std::string& name, int motor_port, int encoder_port_a, int encoder_port_b);
  void InitDefaultCommand() override;

  double GetRate();
  double GetMotor();
  double GetMotorInverted();

  frc::Talon *motor;
  frc::Encoder *encoder;

protected:
  double ReturnPIDInput() override;
  void UsePIDOutput(double output) override;
};

class DriveBase: public frc::Subsystem {
public:
  DriveBase();
  void InitDefaultCommand() override;
  void Stop();
  void SetSpeed(double left_speed, double right_speed);
  void Init();

  frc::Talon *left_motor;
  frc::Encoder *left_encoder;
  frc::Talon *right_motor;
  frc::Encoder *right_encoder;

  frc::PIDController *left_pid;
  frc::PIDController *right_pid;
};
