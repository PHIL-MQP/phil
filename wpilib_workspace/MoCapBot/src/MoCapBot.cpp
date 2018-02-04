#include <iostream>
#include <iomanip>
#include <unistd.h>

#include <CameraServer.h>
#include <Joystick.h>
#include <Buttons/JoystickButton.h>
#include <Timer.h>
#include <Commands/Scheduler.h>
#include <opencv2/imgproc.hpp>
#include <phil/phil_rio/phil_rio.h>
#include <SmartDashboard/SmartDashboard.h>

#include <Commands/Forward.h>
#include <Commands/DriftTest.h>
#include <Commands/Turn.h>
#include <Commands/Circle.h>
#include <Commands/Square.h>
#include <RobotMap.h>
#include <MoCapBot.h>
#include <AHRS.h>

Joystick *Robot::gamepad = nullptr;
frc::JoystickButton *square = nullptr;
frc::JoystickButton *circle = nullptr;
frc::JoystickButton *fwd = nullptr;
frc::JoystickButton *turn = nullptr;
frc::JoystickButton *drift = nullptr;
DriveBase *Robot::drive_base = nullptr;
AHRS *Robot::ahrs = nullptr;
frc::AnalogOutput *Robot::mocap_start_trigger = nullptr;
frc::AnalogOutput *Robot::mocap_stop_trigger = nullptr;

void Robot::RobotInit() {
	std::cout << "RobotInit" << std::endl;

	gamepad = new frc::Joystick(0);
	drive_base = new DriveBase();
	ahrs = new AHRS(frc::I2C::kMXP);
	mocap_stop_trigger = new frc::AnalogOutput(RobotMap::kTriggerStop);
	mocap_start_trigger = new frc::AnalogOutput(RobotMap::kTriggerStart);

	mocap_start_trigger->SetVoltage(5);
	mocap_stop_trigger->SetVoltage(5);
	running = false;

	square = new frc::JoystickButton(gamepad, 1);
	square->WhenReleased(new Square());
	circle = new frc::JoystickButton(gamepad, 2);
	circle->WhenReleased(new Circle(1));
	fwd = new frc::JoystickButton(gamepad, 3);
	fwd->WhenReleased(new Forward(1));
	turn = new frc::JoystickButton(gamepad, 4);
	turn->WhenReleased(new Turn(90));
	drift = new frc::JoystickButton(gamepad, 5);
	drift->WhenReleased(new DriftTest(5));

}

void Robot::TeleopInit() {
	SmartDashboard::PutBoolean("ready", false);
	// create the log to start capturing data
	std::cout << "TeleopInit" << std::endl;

	std::ostringstream filename;
	filename << "/home/lvuser/mocap_data-" << frc::Timer::GetFPGATimestamp()
			<< ".csv";
	log.open(filename.str());

	if (!log) {
		std::cout << "bad log!" << strerror(errno) << '\n';
	}

	if (!log.good()) {
		std::cout << "log !good()" << std::endl;
	}

	log << "raw_accel_x,raw_accel_y,raw_accel_z,"
			<< "world_accel_x,world_accel_y," << "yaw,fused_heading," << "x,y,z,"
			<< "left_encoder_rate,right_encoder_rate,"
			<< "left_input,right_input," << "fpga time,navx time" << std::endl;

	// tell the TK1 to start recording data
	uint8_t data = 1;
	std::cout << "Starting TK1" << std::endl;
	phil::Phil::GetInstance()->SendUDPToTK1(&data, 1, nullptr, 0);
	std::cout << "Starting PI" << std::endl;
	phil::Phil::GetInstance()->SendUDPTo("raspberrypi.local", &data, 1, nullptr, 0);

	// tell the motion capture to start
	std::cout << "Triggering Motion Capture" << std::endl;
	Robot::mocap_start_trigger->SetVoltage(0);
	Robot::mocap_stop_trigger->SetVoltage(5);
	running = true;
	SmartDashboard::PutBoolean("ready", true);

	ahrs->Reset();
	ahrs->ResetDisplacement();
	std::cout << "Status Is Fatal: " << ahrs->StatusIsFatal() << std::endl;
}

void Robot::DisabledInit() {
	SmartDashboard::PutBoolean("ready", false);
	std::cout << "DisabledInit" << std::endl;
	if (running) {
		Robot::mocap_stop_trigger->SetVoltage(0);
		Robot::mocap_start_trigger->SetVoltage(5);
		running = false;
	}

	if (log.is_open()) {
		log.close();
	}

//   tell the TK1 to stop recording data
	uint8_t data = 0;
	std::cout << "Stopping TK1" << std::endl;
	phil::Phil::GetInstance()->SendUDPToTK1(&data, 1, nullptr, 0);
	std::cout << "Stopping PI" << std::endl;
	phil::Phil::GetInstance()->SendUDPTo("raspberrypi.local", &data, 1, nullptr, 0);
}

void Robot::TeleopPeriodic() {
	frc::Scheduler::GetInstance()->Run();

	data_t sample = { 0 };
	sample.raw_accel_x = ahrs->GetRawAccelX();
	sample.raw_accel_y = ahrs->GetRawAccelY();
	sample.raw_accel_z = ahrs->GetRawAccelZ();
	sample.world_accel_x = ahrs->GetWorldLinearAccelX();
	sample.world_accel_y = ahrs->GetWorldLinearAccelY();
	sample.yaw = ahrs->GetYaw();
	sample.fused_heading = ahrs->GetFusedHeading();
	sample.x = ahrs->GetDisplacementX();
	sample.y = ahrs->GetDisplacementY();
	sample.z = ahrs->GetDisplacementZ();
	sample.left_encoder_rate = drive_base->left_encoder->GetRate();
	sample.right_encoder_rate = drive_base->right_encoder->GetRate();
	sample.left_input = gamepad->GetRawAxis(1);
	sample.right_input = -gamepad->GetRawAxis(5);
	sample.fpga_t = frc::Timer::GetFPGATimestamp();
	sample.navx_t = ahrs->GetLastSensorTimestamp();
	sample.left_motor = drive_base->left_motor->Get();
	sample.right_motor = drive_base->right_motor->GetInverted();

	SmartDashboard::PutBoolean("ready", true);
	SmartDashboard::PutData("left pid", drive_base->left_pid);
	SmartDashboard::PutData("right pid", drive_base->right_pid);
	SmartDashboard::PutNumber("left encoder rate",
			drive_base->left_encoder->GetRate());
	SmartDashboard::PutNumber("right encoder rate",
			drive_base->right_encoder->GetRate());

	std::cout << std::setw(6) << sample.raw_accel_x << "," << sample.raw_accel_y
			<< "," << sample.raw_accel_z << "," << sample.world_accel_x << ","
			<< sample.world_accel_y << "," << sample.yaw << ","
			<< sample.fused_heading << "," << "," << sample.x << "," << sample.y
			<< "," << sample.z << "," << sample.left_encoder_rate << ","
			<< sample.right_encoder_rate << "," << sample.left_input << ","
			<< sample.right_input << "," << sample.fpga_t << ","
			<< sample.navx_t << std::endl;

	log << std::setw(6) << sample.raw_accel_x << "," << sample.raw_accel_y
			<< "," << sample.raw_accel_z << "," << sample.world_accel_x << ","
			<< sample.world_accel_y << "," << sample.yaw << ","
			<< sample.fused_heading << "," << sample.x << "," << sample.y << ","
			<< sample.z << "," << sample.left_encoder_rate << ","
			<< sample.right_encoder_rate << "," << sample.left_input << ","
			<< sample.right_input << "," << sample.fpga_t << ","
			<< sample.navx_t << std::endl;
}

void Robot::TestPeriodic() {
	LiveWindow::GetInstance()->AddActuator("drive base", "left pid",
			drive_base->left_pid);
	LiveWindow::GetInstance()->AddActuator("drive base", "right pid",
			drive_base->right_pid);
	LiveWindow::GetInstance()->Run();
}

START_ROBOT_CLASS(Robot)
