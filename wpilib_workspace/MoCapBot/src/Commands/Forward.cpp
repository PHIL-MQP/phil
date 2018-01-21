#include <iostream>
#include <Commands/Forward.h>
#include <MoCapBot.h>

Forward::Forward(double dist_m) : Command("Forward") {
	const double SPEED_MPS = 1;
	duration = dist_m / SPEED_MPS;
}

void Forward::Initialize() {
	std::cout << "fwd" << std::endl;
	SetTimeout(duration);
	Robot::drive_base->SetSpeed(0.3, 0.3);
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
