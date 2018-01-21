#pragma once

#include <Commands/Command.h>

class Forward: public Command {
public:
	Forward(double dist_m);
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	double duration;
};
