#include "Wait.h"
#include "MoCapBot.h"

Wait::Wait(int seconds) : Command("Wait"), seconds(seconds){
    Requires(Robot::drive_base);
}

void Wait::Initialize() {
	SetTimeout(seconds);
}

bool Wait::IsFinished() {
	return IsTimedOut();
}
