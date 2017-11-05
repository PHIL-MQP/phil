#pragma once

#include <CommandBase.h>

class JoystickTeleop : public CommandBase {
public:
	JoystickTeleop();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
