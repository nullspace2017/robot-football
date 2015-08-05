#include <iostream>
#include <unistd.h>
#include "../motor/motor.h"
#include "../location/location.h"

using namespace std;
using namespace cv;

int main() {
    Vec2d origin(900, 200);
    Motor *motor = Motor::get_instance();
    VideoCapture capture2(1);
    Transform trans2(2);
    Location location(motor);
    location.add_camera(&capture2, &trans2, true);
    location.set_current_location(origin, cv::Vec2d(-1, 0));

    double speed = 0.5, thresh = 500;
    int dir = 0;
    Vec2d last_ball_loc(0,0);

    while (1) {
        Vec2d loc = location.get_location().first;
        cout << "machine:" << loc << endl;
        pair<Location::BALLSTATE, cv::Vec2d> ball_pair = location.get_ball();
        int ballstate = ball_pair.first;
        Vec2d ball_loc = ball_pair.second;
        cout << "ball:" << ball_loc << endl;
        cout << "lastball:" << last_ball_loc << endl;
        cout << "dir:" << dir << endl;
        cout << endl;
        imshow("location", location.gen_ground_view());
/*        if (ballstate != Location::BALL_HAS) {
            if (loc[0] - origin[0] > thresh || loc[0] - origin[0] < -thresh) motor->stop();
            else {
                motor->go(5000, dir*speed);
            }
        } else */  {
            if (ball_loc[0] > last_ball_loc[0]) dir = -1;
            else if (ball_loc[0] < last_ball_loc[0]) dir = 1;
            else dir = 0;
            last_ball_loc = ball_loc;
            if (ball_loc[0] == 0 && ball_loc[1] == 0) {
                motor->stop();
            } else if (abs(ball_loc[0] - loc[0]) > 1500 || abs(ball_loc[1] - loc[1]) > 1500) {
                motor->stop();
            } else if ((loc[0] - ball_loc[0]) < 50 && (loc[0] - ball_loc[0]) > -100) {
                motor->stop();
            } else if (loc[0] - ball_loc[0] < 0) {
                if (loc[0] - origin[0] > thresh) motor->stop();
                else {
                    motor->go(5000, -speed);
                }
            } else if (loc[0] - ball_loc[0] > 0) {
                if (loc[0] - origin[0] < -thresh) motor->stop();
                else {
                    motor->go(5000, speed);
                }
            } else {
                motor->stop();
            }
        }
        waitKey(50);
    }
    Motor::destroy_instance();
    return 0;

//    clock_t start = clock();
//    Mat img = imread("6.jpg");
//    Mat newimg(580, 640, CV_8UC1), gray;
//    cvtColor(img, gray, CV_BGR2GRAY);
//    cout << clock() - start << endl;
//    for (int i = 0; i < 480; i ++) {
//        for (int j = 0; j < 640; j ++) {
//            newimg.at<uchar>(i,j) = gray.at<uchar>(i,j);
//        }
//    }
//    for (int i = 480; i < 580; i ++) {
//        for (int j = 0; j < 640; j ++) {
//            newimg.at<uchar>(i,j) = 0;
//        }
//    }
//    cout << clock() - start << endl;
//    imshow("gray", gray);
//    imshow("newimg", newimg);
//    cout << clock() - start << endl;
//    waitKey();
}
