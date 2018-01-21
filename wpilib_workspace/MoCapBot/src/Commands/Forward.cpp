#include <iostream>
#include <Commands/Forward.h>
#include <MoCapBot.h>

Forward::Forward(double dist_m) : Command("Forward") {
  Requires(Robot::drive_base);
	const double SPEED_MPS = 0.5;
	duration = dist_m / SPEED_MPS;
}

void Forward::Initialize() {
	std::cout << "fwd " << duration << std::endl;
	SetTimeout(duration);
	Robot::drive_base->SetSpeed(0.3, 0.3);
}

void Forward::Execute() {
}

bool Forward::IsFinished() {
	return IsTimedOut();
}

void Forward::End() {
	Robot::drive_base->Stop();
}

void Forward::Interrupted() {
	End();
}
