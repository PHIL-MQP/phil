#include <iostream>
#include <unistd.h>

#include <CameraServer.h>
#include <Commands/Scheduler.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc.hpp>
#include <phil/phil.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot : public frc::IterativeRobot {

public:
	void RobotInit() override {
		frc::SmartDashboard::PutString("debug", "hello");
		std::cout << "RobotInit" << std::endl;
	}

	void TeleopInit() override {
		std::cout << "TeleopInit" << std::endl;
	}

	void TeleopPeriodic() override {
		frc::Scheduler::GetInstance()->Run();

		 phil::Phil::GetInstance()->ReadSensorsAndProcessLocally();
//		 phil::Phil::GetInstance()->ReadSensorsAndProcessOnTK1();

		 phil::pose_t pose = phil::Phil::GetInstance()->GetPosition();
	}
};

START_ROBOT_CLASS(Robot)
