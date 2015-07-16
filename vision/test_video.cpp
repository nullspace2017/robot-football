#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "vision.h"

using namespace std;
using namespace cv;

#if 0
int main() {
    VideoCapture cap(1);
    if (!cap.isOpened())
      return -1;
    Mat frame;
    Vision *vis = 0;
    while (1) {
        cap >> frame;
        imshow("window0", frame);
        if (!vis) vis = new Vision(frame.rows, frame.cols);
        vis->input(frame);
        vis->get_edge_white();
        imshow("window", vis->gen_as_pic());
        if (waitKey(30) >= 0) break;
    }
    delete vis;
    return 0;
}
#else
int main() {
    Mat frame = imread("../vision/1.jpg");
    Vision *vis = new Vision(frame.rows, frame.cols);
    vis->input(frame);
    vis->get_edge_white();
    imshow("window", vis->gen_as_pic());
    waitKey();
    delete vis;
    return 0;
}
#endif
