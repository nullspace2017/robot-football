/*
+ 调整摄像头使得恢复到原来标定状态
*/

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

void onMouse(int event, int x, int y, int, void* _img) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        cout << x << ", " << y << endl;
    }
}

void drawLine(Mat& img) {
    // left camera
    line(img, Point(558, 185), Point(89, 182), Scalar(0, 0, 255), 2);
    line(img, Point(558, 185), Point(417, 129), Scalar(0, 0, 255), 2);
    // right camera
    line(img, Point(602, 108), Point(368, 110), Scalar(255, 255, 0), 2);
    line(img, Point(602, 108), Point(484, 67), Scalar(255, 255, 0), 2);
}


int main(int argc, char** argv) {
    VideoCapture cap(1);
    Mat img;
    namedWindow("img");
    while (true) {
        cap >> img;
        drawLine(img);
        imshow("img", img);
        setMouseCallback("img", onMouse, &img);
        if (waitKey(30) >= 0)
            break;
    }
    return 0;
}

