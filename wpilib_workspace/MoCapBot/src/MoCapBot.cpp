#include <iostream>
#include <unistd.h>

#include <CameraServer.h>
#include <Joystick.h>
#include <Timer.h>
#include <Commands/Scheduler.h>
#include <opencv2/imgproc.hpp>
#include <SmartDashboard/SmartDashboard.h>

#include <MoCapBot.h>


Joystick *Robot::gamepad = nullptr;
DriveBase *Robot::drive_base = nullptr;

void Robot::VisionThread() {
  cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
  camera.SetResolution(640, 480);
  cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
  cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("gray", 640, 480);
  cv::Mat source;
  cv::Mat output;

  while (true) {
    cvSink.GrabFrame(source);
    // Somehow we want to time-stamp each frame and save it as a video
    double t = frc::GetTime();
    if (!source.empty()) {
      cvtColor(source, output, cv::COLOR_BGR2GRAY);
      outputStreamStd.PutFrame(output);
    }
    else {
      std::cout << "empty source!" << std::endl;
    }
  }
}


void Robot::RobotInit() {
  std::cout << "RobotInit" << std::endl;

  gamepad = new Joystick(0);
  drive_base = new DriveBase();
}

void Robot::TeleopInit() {
  std::cout << "TeleopInit" << std::endl;
}

void Robot::TeleopPeriodic() {
  frc::Scheduler::GetInstance()->Run();

  // read encoders + NavX and bag
  // An important question is what data from the NavX should we bag? (which functions do we call)
}


START_ROBOT_CLASS(Robot)
