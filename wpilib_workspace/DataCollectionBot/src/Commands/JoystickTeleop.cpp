#include <iostream>

#include <Commands/JoystickTeleop.h>

JoystickTeleop::JoystickTeleop() {
  Requires(drive_base.get());
}

void JoystickTeleop::Initialize() {

}

void JoystickTeleop::Execute() {
   double l = -oi->gamepad->GetX();
   double r = -oi->gamepad->GetY();
   drive_base->SetSpeed(l, r);
}

bool JoystickTeleop::IsFinished() {
	return false;
}

void JoystickTeleop::End() {
  drive_base->Stop();
}

void JoystickTeleop::Interrupted() {
  End();
}
