#include <Commands/Forward.h>

Forward::Forward() : CommandBase("Forward") {
}

void Forward::Initialize() {
	SetTimeout(1.0);
	drive_base->SetSpeed(0.1, 0.1);
}

void Forward::Execute() {
}

bool Forward::IsFinished() {
	return false;
}

void Forward::End() {
	drive_base->Stop();
}

void Forward::Interrupted() {
	End();
}
