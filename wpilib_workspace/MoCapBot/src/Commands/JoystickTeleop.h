#pragma once

#include <Commands/Command.h>

class JoystickTeleop : public Command {
public:
	JoystickTeleop();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
