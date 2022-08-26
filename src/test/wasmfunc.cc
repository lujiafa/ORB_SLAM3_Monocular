
#include <iostream>
#include <algorithm>
#include <time.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "System.h"
#include "QueueProc.cc"


using namespace std;
using namespace cv;

#ifdef __cplusplus
extern "C"
{
#endif


// init 系统的构造函数，将会启动其他的线程
ORB_SLAM3::System *pSLAM;
QueueProcess::QueueProc* qproc;

//从世界坐标系到相机坐标系的转化
cv::Mat Tcw;
vector<ORB_SLAM3::MapPoint*> vMPs;
vector<cv::KeyPoint> vKeys;

int track(int* ptr, int w, int h)
{
    try {
        if (pSLAM == 0) {
            pSLAM = new ORB_SLAM3::System("res/ORBvoc.bin", "res/TUM1.yaml", ORB_SLAM3::System::MONOCULAR, false);
            qproc = new QueueProcess::QueueProc(pSLAM);
        }
        Mat im = Mat(h, w, CV_8UC4, ptr);
        //cv::cvtColor(tim, im, CV_RGB2BGR);
        if (im.empty()) {
            cout << "invalid image data." << endl;
            return 0;
        }
        qproc->track(im);
        vKeys = (*pSLAM).GetTrackedKeyPointsUn();
    } catch (std::exception e) {
        cout << "exception e====>" << e.what() << endl;
    } catch(char *str) {
        cout << "exception str====>" << *str << endl;
    }
    return vKeys.size();
}

void resetTrack() {
    if (pSLAM != 0) {
        (*pSLAM).Reset();
    }
}

//申请内存
int* _malloc(int bytesLen) {
    return (int *) std::malloc(bytesLen);
}
//释放内存
void _free(int *ptr) {
    std::free(ptr);
    cout << "release success..." << endl;
}

#ifdef __cplusplus
}
#endif
