#include <aruco/aruco.h>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <map>
#include <stdexcept>

using namespace cv;
using namespace aruco;
using namespace std;

string TheMarkerMapConfigFile;
bool The3DInfoAvailable = false;
float TheMarkerSize = -1;
VideoCapture TheVideoCapturer;
Mat TheInputImage, TheInputImageCopy;
CameraParameters TheCameraParameters;
MarkerMap TheMarkerMapConfig;
MarkerDetector TheMarkerDetector;
MarkerMapPoseTracker TheMSPoseTracker;
int waitTime = 0;

std::map<int, cv::Mat> frame_pose_map;  // set of poses and the frames they were detected
class CmdLineParser {
  int argc;
  char **argv;
 public:
  CmdLineParser(int _argc, char **_argv) : argc(_argc), argv(_argv) {}

  //is the param?
  bool operator[](string param) {
    int idx = -1;
    for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;
    return (idx != -1);
  }

  //return the value of a param using a default value if it is not present
  string operator()(string param, string defvalue = "-1") {
    int idx = -1;
    for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param) idx = i;
    if (idx == -1) return defvalue; else return (argv[idx + 1]);
  }
};

void savePCDFile(string fpath, const aruco::MarkerMap &ms,
                 const std::map<int, cv::Mat> frame_pose_map);

void savePosesToFile(string filename, const std::map<int, cv::Mat> &fmp);

/************************************
 *
 *
 *
 *
 ************************************/
int main(int argc, char **argv) {
  try {
    CmdLineParser cml(argc, argv);
    if (argc < 4 || cml["-h"]) {
      cerr << "Invalid number of arguments" << endl;
      cerr
          << "Usage: (in.avi|live[:camera_index(e.g 0 or 1)])) marksetconfig.yml camera_intrinsics.yml [optional_arguments]  "
              "\n\t[-s marker_size] \n\t[-pcd out_pcd_file_with_camera_poses] \n\t[-poses out_file_with_poses] "
              "\n\t[-mti value: minimum value in range (0,1) for the size of the detected markers. If 0, ] "
          << endl;
      return false;
    }
    TheMarkerMapConfig.readFromFile(argv[2]);

    TheMarkerMapConfigFile = argv[2];
    TheMarkerSize = stof(cml("-s", "1"));
    // read from camera or from  file
    string TheInputVideo = string(argv[1]);
    if (TheInputVideo.find("live") != std::string::npos) {
      int vIdx = 0;
      // check if the :idx is here
      char cad[100];
      if (TheInputVideo.find(":") != string::npos) {
        std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
        sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
      }
      cout << "Opening camera index " << vIdx << endl;
      TheVideoCapturer.open(vIdx);
      waitTime = 10;
    } else
      TheVideoCapturer.open(argv[1]);        // check video is open
    if (!TheVideoCapturer.isOpened())
      throw std::runtime_error("Could not open video");

    // read first image to get the dimensions
    TheVideoCapturer >> TheInputImage;

    // read camera parameters if passed
    TheCameraParameters.readFromXMLFile(argv[3]);
    TheCameraParameters.resize(TheInputImage.size());
    // prepare the detector
    TheMarkerDetector.setDictionary(TheMarkerMapConfig.getDictionary());
//        TheMarkerDetector.setDetectionMode(aruco::DM_VIDEO_FAST); //use this mode if you know is better
    // prepare the pose tracker if possible
    // if the camera parameers are avaiable, and the markerset can be expressed in meters, then go

    if (TheMarkerMapConfig.isExpressedInPixels() && TheMarkerSize > 0)
      TheMarkerMapConfig = TheMarkerMapConfig.convertToMeters(TheMarkerSize);

    cout << "TheCameraParameters.isValid()=" << TheCameraParameters.isValid() << " "
         << TheMarkerMapConfig.isExpressedInMeters() << endl;

    if (TheCameraParameters.isValid() && TheMarkerMapConfig.isExpressedInMeters())
      TheMSPoseTracker.setParams(TheCameraParameters, TheMarkerMapConfig);

    // Create gui


    char key = 0;
    int index = 0;
    // capture until press ESC or until the end of the video
    cout << "Press 's' to start/stop video" << endl;
    do {
      TheVideoCapturer.retrieve(TheInputImage);
      TheInputImage.copyTo(TheInputImageCopy);
      index++;  // number of images captured
      // Detection of the markers
      vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(TheInputImage);
      // estimate 3d camera pose if possible
      if (TheMSPoseTracker.isValid())
        if (TheMSPoseTracker.estimatePose(detected_markers))
          frame_pose_map.insert(make_pair(index,
                                          TheMSPoseTracker.getRTMatrix()));
      // print the markers detected that belongs to the markerset
      for (auto idx : TheMarkerMapConfig.getIndices(detected_markers))
        detected_markers[idx].draw(TheInputImageCopy, Scalar(0, 0, 255), 1);

      // show  informatino

    } while (key != 27 && TheVideoCapturer.grab());

    // save a beatiful pcd file (pcl library) showing the results (you can use pcl_viewer to see it)
    if (cml["-pcd"])
      savePCDFile(cml("-pcd"), TheMarkerMapConfig, frame_pose_map);

    // save the poses to a file in tum rgbd data format
    if (cml["-poses"])
      savePosesToFile(cml("-poses"), frame_pose_map);
  }
  catch (std::exception &ex) {
    cout << "Exception :" << ex.what() << endl;
  }
}

void getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in, float &qx, float &qy, float &qz, float &qw, float &tx,
                                             float &ty, float &tz) {

  auto SIGN = [](float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
  };

  auto NORM = [](double a, double b, double c, double d) {
    return sqrt(a * a + b * b + c * c + d * d);
  };
  // get the 3d part of matrix and get quaternion
  assert(M_in.total() == 16);
  cv::Mat M;
  M_in.convertTo(M, CV_32F);
  // use now eigen
  float r11 = M.at<float>(0, 0);
  float r12 = M.at<float>(0, 1);
  float r13 = M.at<float>(0, 2);
  float r21 = M.at<float>(1, 0);
  float r22 = M.at<float>(1, 1);
  float r23 = M.at<float>(1, 2);
  float r31 = M.at<float>(2, 0);
  float r32 = M.at<float>(2, 1);
  float r33 = M.at<float>(2, 2);

  double q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
  double q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
  double q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
  double q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
  if (q0 < 0.0f)
    q0 = 0.0f;
  if (q1 < 0.0f)
    q1 = 0.0f;
  if (q2 < 0.0f)
    q2 = 0.0f;
  if (q3 < 0.0f)
    q3 = 0.0f;
  q0 = sqrt(q0);
  q1 = sqrt(q1);
  q2 = sqrt(q2);
  q3 = sqrt(q3);
  if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
    q0 *= +1.0f;
    q1 *= SIGN(r32 - r23);
    q2 *= SIGN(r13 - r31);
    q3 *= SIGN(r21 - r12);
  } else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
    q0 *= SIGN(r32 - r23);
    q1 *= +1.0f;
    q2 *= SIGN(r21 + r12);
    q3 *= SIGN(r13 + r31);
  } else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
    q0 *= SIGN(r13 - r31);
    q1 *= SIGN(r21 + r12);
    q2 *= +1.0f;
    q3 *= SIGN(r32 + r23);
  } else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
    q0 *= SIGN(r21 - r12);
    q1 *= SIGN(r31 + r13);
    q2 *= SIGN(r32 + r23);
    q3 *= +1.0f;
  } else {
    cerr << "Coding error" << endl;
  }
  double r = NORM(q0, q1, q2, q3);
  qx = static_cast<float>(q0 / r);
  qy = static_cast<float>(q1 / r);
  qz = static_cast<float>(q2 / r);
  qw = static_cast<float>(q3 / r);

  tx = M.at<float>(0, 3);
  ty = M.at<float>(1, 3);
  tz = M.at<float>(2, 3);
}

void savePosesToFile(string filename, const std::map<int, cv::Mat> &fmp) {
  std::ofstream file(filename);
  float qx, qy, qz, qw, tx, ty, tz;
  for (auto frame : fmp) {
    if (!frame.second.empty()) {
      getQuaternionAndTranslationfromMatrix44(frame.second, qx, qy, qz, qw, tx, ty, tz);
      file << frame.first << " " << tx << " " << ty << " " << tz << " " << qx << " " << qy << " " << qz << " "
           << qw << endl;
    }
  }
}
