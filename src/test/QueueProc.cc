#include <iostream>
#include <queue>
#include <unistd.h>
#include <sys/timeb.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "System.h"

using namespace std;
using namespace cv;

namespace QueueProcess {
    class QueueProc {
    private:
        ORB_SLAM3::System* SLAM;
        bool startedThread = false;
        std::thread* ithread;
        std::queue<Mat> pqueue;

        bool showKeys = false;
        bool showFps = true;
    public:
        QueueProc(ORB_SLAM3::System* pSLAM) {
           SLAM = pSLAM;
        }

        time_t getCurrTimestamp() {
            timeb t;
            ftime(&t);
            return (t.time * 1000 + t.millitm);
        }

        Sophus::SE3f& track(Mat& im) {
            return proc(im);
        }

        bool push(Mat& im, void rc(Sophus::SE3f& se3f)) {
            if (pqueue.size() < 10) {
                pqueue.push(im);
                return true;
            }
            if (!startedThread) {
                startedThread = true;
                ithread = new thread(&QueueProcess::QueueProc::procTask, this, rc);
            }
            return false;
        }

        void shutdownAsync() {
            if (!startedThread)
                return;
            startedThread = false;
            ithread->join();
            ithread = nullptr;
            while (!pqueue.empty()) {
                pqueue.pop();
            }
        }
    private:
        time_t lastProcSec;
        int fpsNum;

        void procTask(void rc(Sophus::SE3f& se3f)) {
            cv::Mat im;
            vector<ORB_SLAM3::MapPoint*> vMPs;
            vector<cv::KeyPoint> vKeys;
            while (true) {
                if (!startedThread)
                    break;
                try {
                    if (pqueue.empty()) {
                        cout << "queue empty." << endl;
                        usleep(1);
                        continue;
                    }
                    im = pqueue.front();
                    pqueue.pop();
                    if (im.empty()) {
                        continue;
                    }
                    Sophus::SE3f& se3f = proc(im);
                    rc(se3f);
                } catch (exception e) {
                    cout << e.what() << endl;
                }
            }
        }

        Sophus::SE3f& proc(Mat& im) {
            //获取时间戳
            time_t timestamp = getCurrTimestamp();
            double timep = (double) timestamp;
            //输入帧图信息跟踪获取相机位姿的估计结果
            Sophus::SE3f se3f = (*SLAM).TrackMonocular(im, timep);
            // attach
            if (showFps) {
                time_t procSec = timestamp / 1000;
                if (lastProcSec != procSec) {
                    cout << "FPS(" << lastProcSec << "): " << fpsNum << endl;
                    lastProcSec = procSec;
                    fpsNum = 0;
                }
                ++fpsNum;
            }
            if (showKeys) {
//                //获取追踪器状态
//                int status = (*SLAM).GetTrackingState();
//                //获取追踪到的地图点（其实实际上得到的是一个指针）
//                vector<ORB_SLAM3::MapPoint*> vMPs = (*SLAM).GetTrackedMapPoints();
//                //获取追踪到的关键帧的点
//                vector<cv::KeyPoint> vKeys = (*SLAM).GetTrackedKeyPointsUn();
//
//                //画出与地图点于特征点匹配信息
//                const int N = vKeys.size();
//                vector<cv::KeyPoint> vMapKeys;
//                cv::Mat mapKeysShowMat;
//                for (int i = 0; i < N; i++) {
//                    if (vMPs[i]) {
//                        vMapKeys.insert(vMapKeys.begin(), vKeys[i]);
//                    }
//                }
//                char *txt = "UNKOWN";
//                if (status == 1) {
//                    txt = "SLAM NOT INITIALIZED";
//                } else if (status == 2) {
//                    txt = "SLAM ON";
//                } else if (status == 3) {
//                    txt = "SLAM LOST";
//                }
//                //cout << "---->" << vMapKeys.size() << endl;
//                cv::drawKeypoints(im, vMapKeys, mapKeysShowMat, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
//                cv::putText(mapKeysShowMat, txt, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2, 8);
//                cv::imshow("和地图点匹配特征点信息", mapKeysShowMat);
//                waitKey(1);


//                if (se3f.empty()) {
//                    cout << "warning: Tcw is empty." << endl;
//                }
            }
            return se3f;
        }
    };
}