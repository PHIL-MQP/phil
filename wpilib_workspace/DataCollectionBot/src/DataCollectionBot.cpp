#include <unistd.h>
#include <iostream>

#include <CameraServer.h>
#include <opencv2/imgproc.hpp>
#include <IterativeRobot.h>
#include <Commands/Scheduler.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public frc::IterativeRobot {

private:
  // Somehow we want to time-stamp each frame and save it as a video
  static void VisionThread() {
    cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
    camera.SetResolution(640, 480);
    cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
    cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
    cv::Mat source;
    cv::Mat output;

    while (true) {
      cvSink.GrabFrame(source);
      cvtColor(source, output, cv::COLOR_BGR2GRAY);
      outputStreamStd.PutFrame(output);
    }
  }

public:
  void RobotInit() override {
    frc::SmartDashboard::PutString("debug", "hello");
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
    // the real question is what data from the NavX should we bag? Can we do all of it?
  }
};

START_ROBOT_CLASS(Robot)
