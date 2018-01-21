#include <Commands/Turn.h>
#include <MoCapBot.h>
#include <math.h>

Turn::Turn(int degrees) : Command("Turn") {
  Requires(Robot::drive_base);
	radians = ((float) degrees) * M_PI / 180.0;
	duration = degrees * 0.017; // TODO:
}

void Turn::Initialize() {
	SetTimeout(duration);
	Robot::drive_base->SetSpeed(-0.5, 0.5);
}

void Turn::Execute() {
}

bool Turn::IsFinished() {
	return IsTimedOut();
}

void Turn::End() {
	Robot::drive_base->Stop();
}

void Turn::Interrupted() {
	End();
}
