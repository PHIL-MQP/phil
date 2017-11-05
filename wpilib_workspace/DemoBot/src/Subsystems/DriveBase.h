#pragma once

#include <AHRS.h>
#include <Commands/Subsystem.h>
#include <Encoder.h>
#include <SpeedController.h>

class DriveBase: public frc::Subsystem {
public:
	DriveBase();
	void InitDefaultCommand() override;
	void Stop();
	void SetSpeed(double left_speed, double right_speed);

private:
	frc::SpeedController *left_motor;
	frc::SpeedController *right_motor;
	frc::Encoder *left_encoder;
	frc::Encoder *right_encoder;
	AHRS *ahrs;

};
