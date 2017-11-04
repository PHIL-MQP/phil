#include <Commands/Scheduler.h>

#include <CommandBase.h>
#include <Subsystems/DriveBase.h>

std::unique_ptr<DriveBase> CommandBase::drive_base = std::make_unique<DriveBase>();
std::unique_ptr<OI> CommandBase::oi = std::make_unique<OI>();

CommandBase::CommandBase(const std::string &name) :
		frc::Command(name) {

}
