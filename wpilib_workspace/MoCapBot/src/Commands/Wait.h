#pragma once

#include <Commands/Command.h>

class Wait : public Command {
public:
	Wait(int seconds);
	void Initialize();
	bool IsFinished();
private:
	int seconds;
};
