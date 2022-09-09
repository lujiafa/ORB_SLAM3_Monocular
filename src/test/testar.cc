
#include <iostream>
#include <exception>
#include <sys/timeb.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "QueueProc.cc"

using namespace std;
using namespace cv;


time_t getCurrTimestamp() {
    timeb t;
    ftime(&t);
    return (t.time * 1000 + t.millitm);
}

int main()
{
//    VideoCapture capCam = cv::VideoCapture(0);
//    if (!capCam.isOpened())
//    {
//        cout << "camera->open faild." << std::endl;
//        exit(-1);
//    }
//    capCam.set(CV_CAP_PROP_FRAME_WIDTH, 640);
//    capCam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
//    capCam.set(CV_CAP_PROP_FRAME_COUNT, 30);

    VideoCapture capCam = cv::VideoCapture("/home/jon/slamtest.mp4");

    // init 系统的构造函数，将会启动其他的线程
    ORB_SLAM3::System SLAM("res/ORBvoc.bin","res/TUM1.yaml",ORB_SLAM3::System::MONOCULAR,false);

    QueueProcess::QueueProc* qproc = new QueueProcess::QueueProc(&SLAM);
    //qproc->start();
    while (true) {
        cv::Mat im;
        if (!capCam.read(im)) {
            cout << "读取摄像头帧信息失败->退出" << endl;
            break;
        }
        Sophus::SE3f& se3f = qproc->track(im);
    }
    // Stop all threads
    SLAM.Shutdown();
    return 0;
}


