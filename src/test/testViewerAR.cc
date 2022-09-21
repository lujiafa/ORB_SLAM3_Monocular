
#include <iostream>
#include <exception>
#include <sys/timeb.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "ViewerAR.h"

using namespace std;
using namespace cv;


ORB_SLAM3::ViewerAR viewerAR;

cv::Mat K;
cv::Mat DistCoef;

int main()
{
    // init 系统的构造函数，将会启动其他的线程
    ORB_SLAM3::System SLAM("/home/jon/workspace2/ORB_SLAM3_Monocular/output/res/ORBvoc.bin","/home/jon/workspace2/ORB_SLAM3_Monocular/output/res/TUM1.yaml",ORB_SLAM3::System::MONOCULAR,false);
    viewerAR.SetSLAM(&SLAM);
    cv::FileStorage fSettings("/home/jon/workspace2/ORB_SLAM3_Monocular/output/res/TUM1.yaml", cv::FileStorage::READ);
    bool bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
    float fps = fSettings["Camera.fps"];
    viewerAR.SetFPS(fps);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    viewerAR.SetCameraCalibration(fx,fy,cx,cy);

    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    DistCoef = cv::Mat::zeros(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    viewerAR.Run();
    // Stop all threads
    SLAM.Shutdown();
    return 0;
}


