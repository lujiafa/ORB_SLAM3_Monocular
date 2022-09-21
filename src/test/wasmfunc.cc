
#include <iostream>
#include <algorithm>
#include <time.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <emscripten.h>

#include "System.h"
#include "QueueProc.cc"


using namespace std;
using namespace cv;

#ifdef __cplusplus
extern "C"
{
#endif
EM_JS(void, trackEMJS, (float a0, float a1, float a2, float a3,
        float a4, float a5, float a6, float a7,
        float a8, float a9, float a10, float a11,
        float a12, float a13, float a14, float a15), {
    var array = [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15];
    trackCallback(array);
});

EM_JS(void, trackEMJS2, (float px, float py, float pz,
                        float rx, float ry, float rz), {
    var array = [px, py, pz, rx, ry, rz];
    trackCallback2(array);
});

// init 系统的构造函数，将会启动其他的线程
ORB_SLAM3::System *pSLAM;
QueueProcess::QueueProc* qproc;

void init() {
    if (pSLAM != 0 && !pSLAM->isShutDown()) {
        pSLAM->Shutdown();
        qproc->shutdownAsync();
    }
    pSLAM = new ORB_SLAM3::System("res/ORBvoc.bin", "res/TUM1.yaml", ORB_SLAM3::System::MONOCULAR, false);
    qproc = new QueueProcess::QueueProc(pSLAM);
}

int track(int* ptr, int w, int h, bool stype = false)
{
    try {
        Mat im = Mat(h, w, CV_8UC4, ptr);
        //cv::cvtColor(tim, im, CV_RGB2BGR);
        if (im.empty()) {
            cout << "invalid image data." << endl;
            return 0;
        }
        Sophus::SE3f &se3f = qproc->track(im);
        cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(se3f.matrix());
        if (stype) {
            trackEMJS2(Tcw.at<float>(0,3), Tcw.at<float>(1,3), Tcw.at<float>(2,3), se3f.angleX(), se3f.angleY(), se3f.angleZ());
        } else {
            trackEMJS(Tcw.at<float>(0,0), Tcw.at<float>(1,0), Tcw.at<float>(2,0), Tcw.at<float>(3,0),
                      Tcw.at<float>(0,1), Tcw.at<float>(1,1), Tcw.at<float>(2,1), Tcw.at<float>(3,1),
                      Tcw.at<float>(0,2), Tcw.at<float>(1,2), Tcw.at<float>(2,2), Tcw.at<float>(3,2),
                      Tcw.at<float>(0,3), Tcw.at<float>(1,3), Tcw.at<float>(2,3), Tcw.at<float>(3,3));
        }
        return 1;
    } catch (std::exception e) {
        cout << "exception e====>" << e.what() << endl;
    } catch(char *str) {
        cout << "exception str====>" << *str << endl;
    }
    return 0;
}

void arackCallback(Sophus::SE3f& se3f) {
    cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(se3f.matrix());
    trackEMJS(Tcw.at<float>(0,0), Tcw.at<float>(1,0), Tcw.at<float>(2,0), 0.0,
              Tcw.at<float>(0,1), Tcw.at<float>(1,1), Tcw.at<float>(2,1), 0.0,
              Tcw.at<float>(0,2), Tcw.at<float>(1,2), Tcw.at<float>(2,2), 0.0,
              Tcw.at<float>(0,3), Tcw.at<float>(1,3), Tcw.at<float>(2,3), 0.0);
}

int atrack(int* ptr, int w, int h) {
    try{
        Mat im = Mat(h, w, CV_8UC4, ptr);
        //cv::cvtColor(tim, im, CV_RGB2BGR);
        if (im.empty()) {
            cout << "invalid image data." << endl;
            return 0;
        }
        if (qproc->push(im, arackCallback)) {
            return 1;
        }
    } catch (std::exception e) {
        cout << "exception e====>" << e.what() << endl;
    } catch(char *str) {
        cout << "exception str====>" << *str << endl;
    }
    return 0;
}

//重置
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

void shutdown() {
    if (pSLAM != 0 && !pSLAM->isShutDown()) {
        pSLAM->Shutdown();
        qproc->shutdownAsync();
    }
}

#ifdef __cplusplus
}
#endif

