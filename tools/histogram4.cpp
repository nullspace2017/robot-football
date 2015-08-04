/*
+ 把RGB改成HSV，也许会好一点
*/
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int point_time = 0;
Point point1, point2;
const int channels0[1] = {0};
const int channels1[1] = {1};
const int channels2[1] = {2};
const int histSize[1] = {256};
float hranges[2] = {0, 255};    // 难不成这是均匀直方图的意思？
const float* ranges[1] = {hranges};

Mat getHistImg(const Mat& hist, const Scalar& s) {
    double maxVal = 0;
    double minVal = 0;
    
    minMaxLoc(hist, &minVal, &maxVal, 0, 0);
    int histSize = hist.rows;
    Mat histImg(histSize, histSize, CV_8UC3, Scalar(255, 255, 255));
    
    int hpt = int(0.9 * histSize);

    for (int h = 0; h < histSize; h ++) {
        float binVal = hist.at<float>(h);
        int intensity = int(binVal*hpt/maxVal);
        line(histImg, Point(h, histSize), Point(h, histSize-intensity), s);
    }
    return histImg;
}

void pointMouse(int event, int x, int y, int, void*) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        cout << x << ", " << y << endl;
    }
}


void trigger(Mat& image) {
    Mat image2 = image.clone();
    Rect rect(point1, point2);
    rectangle(image2, rect, 255);
    Mat mask(image.size(), CV_8U, Scalar::all(0));
    rectangle(mask, rect, 255, -1);
    Mat imgHSV(image.rows, image.cols, CV_32F);
    cvtColor(image, imgHSV, CV_BGR2HSV_FULL);

    Mat hist0, hist1, hist2;
    calcHist(&imgHSV, 1, channels0, mask, hist0, 1, histSize, ranges);
    calcHist(&imgHSV, 1, channels1, mask, hist1, 1, histSize, ranges);
    calcHist(&imgHSV, 1, channels2, mask, hist2, 1, histSize, ranges);

    Mat histImg0 = getHistImg(hist0, Scalar(255, 0, 0));
    Mat histImg1 = getHistImg(hist1, Scalar(0, 255, 0));
    Mat histImg2 = getHistImg(hist2, Scalar(0, 0, 255));

    imshow("H", histImg0);
    imshow("S", histImg1);
    imshow("V", histImg2);
    
    setMouseCallback("H", pointMouse);
    setMouseCallback("S", pointMouse);
    setMouseCallback("V", pointMouse);

    imshow("image", image2);
    waitKey(30);
}

void onMouse(int event, int x, int y, int, void* _image) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        cout << x << ", " << y << endl;
        if (point_time == 0 || point_time == 2) {
            point_time ++;
            point1.x = x;
            point1.y = y;
        }
        else if (point_time == 1 || point_time == 3) {
            point_time = 2;
            point2.x = x;
            point2.y = y;
            Mat* image = (Mat*)_image;
            trigger(*image);
        }
    }
}

#if 0
int main(int argc, char** argv) {
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " [filename]" << endl;
        return 1;
    }
    Mat image = imread(argv[1]);
    cout << image.type() << endl;
    imshow("image", image);
    setMouseCallback("image", onMouse, &image);
    waitKey(0);
    return 0;
}
#else
int main() {
    VideoCapture cap(1);
    if (!cap.isOpened())
      return -1;
    Mat image;
    while (waitKey(30) < 0) {
        cap >> image;
        if (point_time >= 2) {
            trigger(image);
        } else {
            imshow("image", image);
        }
        setMouseCallback("image", onMouse, &image);
    }
    return 0;
}
#endif
