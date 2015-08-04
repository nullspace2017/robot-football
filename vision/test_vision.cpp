#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "vision.h"

using namespace std;
using namespace cv;

#if 1
void on_mouse(int event, int x, int y, int, void *) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        cout << x << '\t' << y << endl;
    }
}

int main() {
    VideoCapture cap(1);
    if (!cap.isOpened())
      return -1;
    Transform trans(1);
    Vision *vis = new Vision(cvRound(cap.get(CV_CAP_PROP_FRAME_HEIGHT)),
                             cvRound(cap.get(CV_CAP_PROP_FRAME_WIDTH)), &trans);
    while (1) {
        Mat frame;
        cap >> frame;
        vis->input(frame);
        line(frame, Point(1, 240), Point(638, 240), Scalar(255, 0, 0));
        line(frame, Point(320, 230), Point(320, 250), Scalar(255, 0, 0));
        imshow("window0", frame);
        imshow("platform", vis->gen_platform());
        cvSetMouseCallback("window0", on_mouse, 0);
        cvSetMouseCallback("platform", on_mouse, 0);
        if (waitKey(1) >= 0) break;
    }
    delete vis;
    return 0;
}
#else
int main() {
    Mat frame = imread("../vision/5.jpg");
    Transform trans(1);
    Vision *vis = new Vision(frame.rows, frame.cols, &trans);
    vis->input(frame);
    imshow("platform", vis->gen_platform());
    waitKey();
    delete vis;
    return 0;
}
#endif
