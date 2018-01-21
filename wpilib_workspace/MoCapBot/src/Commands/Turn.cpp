#include <Commands/Turn.h>
#include <MoCapBot.h>
#include <math.h>

Turn::Turn(int degrees) : Command("Turn") {
	radians = ((float) degrees) * M_PI / 180.0;
	duration = 0; // TODO:
}

void Turn::Initialize() {
	SetTimeout(duration);
	Robot::drive_base->SetSpeed(0.1, 0.1);
}

void Turn::Execute() {
}

bool Turn::IsFinished() {
	return false;
}

void Turn::End() {
	Robot::drive_base->Stop();
}

void Turn::Interrupted() {
	End();
}
