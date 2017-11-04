#pragma once

#include <memory>
#include <string>

#include <Commands/Command.h>
#include <Subsystems/DriveBase.h>

#include <OI.h>

class CommandBase: public frc::Command {
public:
	CommandBase(const std::string& name);
	CommandBase() = default;

	static std::unique_ptr<DriveBase> drive_base;
	static std::unique_ptr<OI> oi;
};
