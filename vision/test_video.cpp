#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "vision.h"
#include "transform.h"

using namespace std;
using namespace cv;

#if 0
void on_mouse(int event, int x, int y, int, void *) {
    if(event == CV_EVENT_LBUTTONDOWN) {
        cout << y << '\t' << x << endl;
    }
}

int main() {
    VideoCapture cap(1);
    if (!cap.isOpened())
      return -1;
    Mat frame;
    Transform trans;
    Vision *vis = 0;
    while (1) {
        cap >> frame;
        if (!vis) vis = new Vision(frame.rows, frame.cols, &trans);
        vis->input(frame);
        vis->get_edge_white();
        line(frame, Point(1, 240), Point(638, 240), Scalar(255, 0, 0));
        line(frame, Point(320, 230), Point(320, 250), Scalar(255, 0, 0));
        imshow("window0", frame);
        imshow("window", vis->gen_as_pic());
        imshow("platform", vis->gen_planform());
        cvSetMouseCallback("window0", on_mouse, 0);
        cvSetMouseCallback("window", on_mouse, 0);
        cvSetMouseCallback("platform", on_mouse, 0);
        if (waitKey(30) >= 0) break;
    }
    delete vis;
    return 0;
}
#else
int main() {
    Mat frame = imread("../vision/1.jpg");
    Transform trans;
    Vision *vis = new Vision(frame.rows, frame.cols, &trans);
    vis->input(frame);
    vis->get_edge_white();
    imshow("window", vis->gen_as_pic());
    imshow("platform", vis->gen_planform());
    waitKey();
    delete vis;
    return 0;
}
#endif
