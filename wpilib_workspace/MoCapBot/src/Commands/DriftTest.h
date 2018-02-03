#pragma once

#include <Commands/CommandGroup.h>

class DriftTest : public CommandGroup {
public:
	DriftTest(int n);
private:
	int n;
};
