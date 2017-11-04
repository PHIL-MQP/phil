#pragma once

#include "../CommandBase.h"

class Forward: public CommandBase {
public:
	Forward();
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};
