#include <iostream>

#include <Commands/JoystickTeleop.h>
#include <MoCapBot.h>

JoystickTeleop::JoystickTeleop() : Command("JoystickTeleop") {
  Requires(Robot::drive_base);
}

void JoystickTeleop::Initialize() {

}

void JoystickTeleop::Execute() {
   double l = -Robot::gamepad->GetRawAxis(1); // left up/down
   double r = -Robot::gamepad->GetRawAxis(3); // right up/down
   Robot::drive_base->SetSpeed(l, r);
}

bool JoystickTeleop::IsFinished() {
	return false;
}

void JoystickTeleop::End() {
  Robot::drive_base->Stop();
}

void JoystickTeleop::Interrupted() {
  End();
}
