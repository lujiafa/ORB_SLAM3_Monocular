
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
EM_JS(void, trackEMJS, (float a0, float a1, float a2,
                        float a3, float a4, float a5, float a6), {
    let p = [a0, a1, a2];
    let r = [a3, a4, a5, a6];
    trackCallback(p, r);
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

int track(int* ptr, int w, int h)
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
        if ((*pSLAM).GetTrackingState() != 2) {
            return 0;
        }

        float transform_ygx[] =
                {
                        1, 0, 0, 0,
                        0, -1, 0, 0,
                        0, 0, -1, 0,
                        0, 0, 0, 1
                };
        Mat_<float> transformM(4, 4, transform_ygx);
        Mat r;
        Tcw.rowRange(0,3).colRange(0,3).copyTo(r);
        float R[9];
        memcpy(R, r.data, r.cols*r.rows*sizeof(float));
        float qw = sqrtf(1.0f + R[0] + R[4] + R[8]) / 2.0f;
        float qx = -(R[7] - R[5]) / (4*qw) ;
        float qy = (R[3] - R[1]) / (4*qw) ;
        float qz = -(R[2] - R[6]) / (4*qw) ;
        float rotate_ygx[] =
                {
                        1 - 2 * qy * qy - 2 * qz * qz, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw, 0,
                        2 * qx * qy + 2 * qz * qw, 1 - 2 * qx * qx - 2 * qz * qz, 2 * qy * qz - 2 * qx * qw, 0,
                        2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx * qx - 2 * qy * qy, 0,
                        0, 0, 0, 1
                };
        Mat_<float> rotateM(4, 4, rotate_ygx);
        rotateM = transformM * rotateM;

        Mat t = Tcw.col(3);
        float tf = t.at<float>(1,0);
        t.at<float>(1,0) = -t.at<float>(2,0);
        t.at<float>(1,0) = tf;
        t = rotateM * t;

        vector<float> q = ORB_SLAM3::Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3).t());
        trackEMJS(Tcw.at<float>(0,3), Tcw.at<float>(1,3), Tcw.at<float>(2,3),
                   q[0], q[1], q[2],q[3]);

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
//    trackEMJS(Tcw.at<float>(0,0), Tcw.at<float>(1,0), Tcw.at<float>(2,0),
//              Tcw.at<float>(0,1), Tcw.at<float>(1,1), Tcw.at<float>(2,1),
//              Tcw.at<float>(0,2), Tcw.at<float>(1,2), Tcw.at<float>(2,2),
//              Tcw.at<float>(0,3), Tcw.at<float>(1,3), Tcw.at<float>(2,3));
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

void shutdown() {
    if (pSLAM != 0 && !pSLAM->isShutDown()) {
        pSLAM->Shutdown();
        qproc->shutdownAsync();
    }
}

#ifdef __cplusplus
}
#endif

