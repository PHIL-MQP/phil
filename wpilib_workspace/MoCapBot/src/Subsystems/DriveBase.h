#pragma once

#include <SpeedController.h>
#include <Encoder.h>
#include <Commands/Subsystem.h>
#include <AHRS.h>

class DriveBase: public frc::Subsystem {
public:
	DriveBase();
	void InitDefaultCommand() override;
	void Stop();
	void SetSpeed(double left_speed, double right_speed);
	void Init();

	frc::SpeedController *left_motor;
	frc::SpeedController *right_motor;
};
