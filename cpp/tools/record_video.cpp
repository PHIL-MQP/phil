#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <chrono>
#include <thread>
#include <unistd.h>

using namespace cv;
using namespace std;
using namespace aruco;

VideoCapture initCapture()
{
    /*
    fancy initialization using v4l2
    doesnt work (on Kacper's laptop) now, not sure why.
    http://opencv-users.1802565.n2.nabble.com/CvGetCaptureProperty-returns-0-for-frame-count-and-time-MSEC-td7583259.html
    possible explanation?
    */

    // string cameraPipeline;
    // cameraPipeline ="v4l2src device=/dev/video0 extra-controls=\"c,exposure_auto=1,exposure_absolute=500\" ! ";
    // cameraPipeline+="video/x-raw, format=BGR, framerate=30/1, width=(int)1280,height=(int)720 ! ";
    // cameraPipeline+="appsink";

    // open the default camera, default settings
    // 0 --> /dev/video0
    VideoCapture cap(0);
    CameraParameters CamParam;

    CamParam.readFromXMLFile("/home/kacper/Desktop/camera_example_11_12.yml");


    cap.open(0);

    if(!cap.isOpened()) { // check if we succeeded
        cout << "Cap failed to open" << endl;
        return -1;
    }

    return cap;

}

int getv4LTimestamps()
{

    VideoCapture cap = initCapture();

    Mat frame;
    namedWindow("frame",1);

    while(cap.isOpened())
    {


    /*
    read a frame
    under the hood, this will call both grab and retrieve
    you might not want to do this if you have many camera streams (combine grabs & retrieves)
    */
    // cap.grab();
    // cap.retrieve(frame);
    cap >> frame;

    /*
    print the data to the console
    */
    printf( "CV_CAP_PROP_POS_MSEC:   %ld   \n", (long) cap.get( CV_CAP_PROP_POS_MSEC) );
    printf( "CV_CAP_PROP_POS_FRAMES:  %ld \n", ((long) cap.get( CV_CAP_PROP_POS_FRAMES) ));  // <-- the v4l2 'sequence' field
    printf( "CV_CAP_PROP_FPS:  %ld \n", ((long) cap.get( CV_CAP_PROP_FPS) ));  // <-- the v4l2 'sequence' field

    /*
    show frame
    */
    imshow("frame", frame);

    if(waitKey(1) >= 0) break;
    // if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

int getTimestampsFromClock()
{

    /*
    open the default camera, default settings
    0 --> /dev/video0
    */
    VideoCapture cap = initCapture();


    Mat frame;
    namedWindow("frame",1);

    #if __cplusplus >= 201103L || (__cplusplus < 200000 && __cplusplus > 199711L)
        std::chrono::steady_clock::time_point tf1;
        std::chrono::steady_clock::time_point tf2;
    #else
        std::chrono::monotonic_clock::time_point tf1;
        std::chrono::monotonic_clock::time_point tf2;
    #endif

    /*
    loop
    */
    while(cap.isOpened())
    {
     //get the time right now
    #if __cplusplus >= 201103L || (__cplusplus < 200000 && __cplusplus > 199711L)
        tf1 = std::chrono::steady_clock::now();
    #else
        tf1 = std::chrono::monotonic_clock::now();
    #endif

    //query frame
    cap >> frame;

    //get the time after querying
    #if __cplusplus >= 201103L || (__cplusplus < 200000 && __cplusplus > 199711L)
        tf2 = std::chrono::steady_clock::now();
    #else
        tf2 = std::chrono::monotonic_clock::now();
    #endif

    /* find difference */
    double timestamp = std::chrono::duration_cast<std::chrono::duration<double> >(tf2 - tf1).count();
    /* get system-wide clock */
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);

    cout << "Timestamp: " << timestamp << endl;
    printf("%lld.%.9ld", (long long)now.tv_sec, now.tv_nsec);

    /*
    show frame
    */
    imshow("frame", frame);

    if(waitKey(1) >= 0) break;
    // if(waitKey(30) >= 0) break;
    }

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

int recordStream()
{
    // 9000
    VideoCapture cap = initCapture();
    int numFrames = 30;
    Mat frame[numFrames];
    struct timespec stamps[numFrames];

    for(int i = 0; i < numFrames; i++)
    {
        cap >> frame[i];
        clock_gettime(CLOCK_REALTIME, &stamps[i]);
    }





    /*
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    for(int i = 0; i < numFrames; i++)
    {
        string pathname = "/home/kacper/Desktop/ocv_sample/test/test" + to_string(i) + ".png";
        cout << pathname << endl;
        try {
            imwrite(pathname, frame[i], compression_params);
        }
        catch (runtime_error& ex) {
            fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
            return 1;
        }
    }
    */

    int frame_width =   cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height =   cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);

    FILE * pFile;

    pFile = fopen ("myfile.txt","w");
    for(int i = 0; i < numFrames; i++)
    {
        video.write(frame[i]);
        fprintf(pFile,"%lld.%.9ld\n", (long long)stamps[i].tv_sec, stamps[i].tv_nsec);
    }

    fclose (pFile);

    cout << "saved" << endl;

}
/* display one image */
int displayImage(string img)
{


    Mat image;
    image = imread( img, 1 );

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);

    waitKey(0);

    return 0;
}

void detectMarkers(VideoCapture capture, CameraParameters CamParam)
{
    Mat frame;

    float MarkerSize = 0.175; // meters

    //Create the detector
    MarkerDetector MDetector;
    MDetector.setThresholdParams(7, 7);
    MDetector.setThresholdParamRange(2, 0);
    map<uint32_t,MarkerPoseTracker> MTracker;//use a map so that for each id, we use a different pose tracker
    cv::namedWindow("in", 1);

    if(!capture.isOpened())
    {
        //error!
        cout << "Can not load video";
    }

    else
    {
        //new frame resize based on frame
        // capture.grab();
        capture >> frame;
        CamParam.resize(frame.size());


        while(capture.isOpened())
        {
            /*read frame*/
            capture.grab();
            capture.retrieve(frame);

            /*detect markers in frame*/
            vector< Marker >  Markers=MDetector.detect(frame);

            for(auto & marker:Markers)
                MTracker[marker.id].estimatePose(marker,CamParam,MarkerSize);//


            for (unsigned int i = 0; i < Markers.size(); i++) {
                cout << Markers[i] << '\t';
                Markers[i].draw(frame, Scalar(0, 0, 255), 2);
            }
            if (CamParam.isValid() && MarkerSize != -1){
                for (unsigned int i = 0; i < Markers.size(); i++) {
                    CvDrawingUtils::draw3dCube(frame, Markers[i], CamParam);
                    CvDrawingUtils::draw3dAxis(frame, Markers[i], CamParam);
                }
            }
            cv::namedWindow("in", 1);
            cv::imshow("in", frame);


        }
    }
}

int main(int argc, char** argv )
{
    // getv4LTimestamps();
    // getTimestampsFromClock();
    recordStream();
}
