#pragma once

#include <Commands/Command.h>

class Circle: public Command {
public:
	Circle(double radius_m);
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
	double left_speed, right_speed, duration;
};
