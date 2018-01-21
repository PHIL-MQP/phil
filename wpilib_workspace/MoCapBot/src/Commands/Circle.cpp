#include <iostream>

#include <Commands/Circle.h>
#include <MoCapBot.h>

Circle::Circle(double radius_m) : Command("Circle") {
	const float TRACK_WIDTH_M = 0.5254625;
	right_speed = 0.2;
	left_speed = right_speed * (radius_m + TRACK_WIDTH_M/2) / (radius_m - TRACK_WIDTH_M/2);
	duration = right_speed * (radius_m + TRACK_WIDTH_M/2);
}

void Circle::Initialize() {
	std::cout << "running for " << duration << " seconds";
	SetTimeout(duration);
	Robot::drive_base->SetSpeed(left_speed, right_speed);
}

void Circle::Execute() {
}

bool Circle::IsFinished() {
	return false;
}

void Circle::End() {
	Robot::drive_base->Stop();
}

void Circle::Interrupted() {
	End();
}
