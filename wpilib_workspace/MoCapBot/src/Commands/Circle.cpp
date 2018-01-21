#include <iostream>

#include <Commands/Circle.h>
#include <MoCapBot.h>

Circle::Circle(double radius_m) : Command("Circle") {
  Requires(Robot::drive_base);
	const float TRACK_WIDTH_M = 0.5254625;
	right_speed = 0.2;
	left_speed = right_speed * (radius_m + TRACK_WIDTH_M/2) / (radius_m - TRACK_WIDTH_M/2);
	duration = right_speed * (radius_m + TRACK_WIDTH_M/2);
}

void Circle::Initialize() {
	std::cout << "circle " << duration << std::endl;
	SetTimeout(5);
	Robot::drive_base->SetSpeed(0.3, 0.5);
}

void Circle::Execute() {
}

bool Circle::IsFinished() {
	return IsTimedOut();
}

void Circle::End() {
	Robot::drive_base->Stop();
}

void Circle::Interrupted() {
	End();
}
