#pragma once

#include <Commands/Command.h>

class Turn: public Command {
public:
	Turn(int degrees);
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	double radians;
	double duration;
};
