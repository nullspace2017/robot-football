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
    Mat* img = (Mat*)_img;
    if (event == CV_EVENT_LBUTTONDOWN) {
        cout << x << ", " << y << endl;
    }
}

void drawLine(Mat& img) {
    line(img, Point(15, 252), Point(228, 211), Scalar(0, 0, 255), 2);
    line(img, Point(62, 242), Point(628, 260), Scalar(0, 0, 255), 2);
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
        waitKey(30);
    }
    return 0;
}

