#include <iostream>
#include <unistd.h>
#include "../motor/motor.h"
#include "../location/location.h"

using namespace std;
using namespace cv;

int main() {
    Motor *motor = Motor::get_instance();
    VideoCapture capture1(1);
    Location location(motor);
    Server server;
    location.add_server(&server);
    while (1) {
        Mat frame;
        capture1 >> frame;
        int iLowH = 160, iHighH = 179, iLowS = 60, iHighS = 255, iLowV = 0, iHighV = 255;
        Mat imgHSV;
        cvtColor(frame, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        Mat imgThresholded;
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
        imshow("thres", imgThresholded);
        int sum_u = 0;
        int red_cnt = 0;
        for (int i = 0; i < imgThresholded.rows; i++) {
            uchar *p = imgThresholded.ptr<uchar>(i);
            for (int j = 0; j < imgThresholded.cols; j++) {
                if (p[j] == 255) {
                    sum_u += j;
                    red_cnt++;
                }
            }
        }
        double red_avg = (double)sum_u / red_cnt;
        cout << red_cnt << '\t' << red_avg << endl;
        if (red_cnt > 1400) {
            if (red_avg < 310) motor->go(INFINITY, 0.6);
            else if (red_avg > 400) motor->go(INFINITY, -0.6);
            else motor->stop();
        } else {
            motor->stop();
        }
        waitKey(20);
    }
    Motor::destroy_instance();
    return 0;
}
