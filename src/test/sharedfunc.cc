
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

// init 系统的构造函数，将会启动其他的线程
ORB_SLAM3::System *pSLAM;
QueueProcess::QueueProc* qproc;

//从世界坐标系到相机坐标系的转化
cv::Mat Tcw;
vector<ORB_SLAM3::MapPoint*> vMPs;


//process_Image
//track
extern "C"
{
int process_Image(uchar ptr[], int w, int h, float position[], float rotation[]) {
    cout << "in slam. " << endl;
    try {
        if (pSLAM == 0) {
            pSLAM = new ORB_SLAM3::System("res/ORBvoc.bin", "res/TUM1.yaml", ORB_SLAM3::System::MONOCULAR, false);
            qproc = new QueueProcess::QueueProc(pSLAM);
        }
        uchar *s = ptr;
        Mat im = Mat(h, w, CV_8UC4, s);
        //cv::cvtColor(tim, im, CV_RGB2BGR);
        if (im.empty()) {
            cout << "invalid image data." << endl;
            return 0;
        }
        Sophus::SE3f &se3f = qproc->track(im);
        cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(se3f.matrix());

        rotation[0] = se3f.angleX();
        rotation[1] = -se3f.angleY();
        rotation[2] = se3f.angleZ();

        Mat t;
        float T[3];
        Tcw.rowRange(0, 3).col(3).copyTo(t);
        memcpy(T, t.data, t.cols * t.rows * sizeof(float));
        position[0] = T[0];
        position[1] = -T[2];
        position[2] = T[1];
    } catch (std::exception e) {
        cout << "exception e====>" << e.what() << endl;
    } catch (char *str) {
        cout << "exception str====>" << *str << endl;
    }
    return 0;
}
}

void resetTrack() {
    if (pSLAM != 0) {
        (*pSLAM).Reset();
    }
}

//申请内存
int *_malloc(int bytesLen) {
    return (int *) std::malloc(bytesLen);
}
//释放内存
void _free(int *ptr) {
    std::free(ptr);
    cout << "release success..." << endl;
}



