#include <Commands/Forward.h>
#include <DemoBot.h>

Forward::Forward() : Command("Forward") {
}

void Forward::Initialize() {
	SetTimeout(1.0);
	Robot::drive_base->SetSpeed(0.1, 0.1);
}

void Forward::Execute() {
}

bool Forward::IsFinished() {
	return false;
}

void Forward::End() {
	Robot::drive_base->Stop();
}

void Forward::Interrupted() {
	End();
}
