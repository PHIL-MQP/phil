#include <unistd.h>
#include <iostream>

#include <Timer.h>
#include <Joystick.h>
#include <CameraServer.h>
#include <opencv2/imgproc.hpp>
#include <IterativeRobot.h>
#include <Commands/Scheduler.h>
#include <SmartDashboard/SmartDashboard.h>

#include <CommandBase.h>

class Robot: public frc::IterativeRobot {

private:
  static void VisionThread() {
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

public:

  Robot() : IterativeRobot() {
    joystick = new frc::Joystick(0);
  }

  void RobotInit() override {
    CommandBase::drive_base->Init();
    CommandBase::oi->Init();

    std::cout << "RobotInit" << std::endl;

    std::thread visionThread(VisionThread);
    visionThread.detach();
  }

  void TeleopInit() override {
    std::cout << "TeleopInit" << std::endl;
  }

  void TeleopPeriodic() override {
    frc::Scheduler::GetInstance()->Run();

    // read encoders + NavX and bag
    // An important question is what data from the NavX should we bag? (which functions do we call)
  }

private:
  frc::Joystick *joystick;
};

START_ROBOT_CLASS(Robot)
