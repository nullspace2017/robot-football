#include <iostream>
#include <unistd.h>
#include "../motor/motor.h"
#include "../location/location.h"

using namespace std;
using namespace cv;

int main() {
    Motor *motor = Motor::get_instance();
    VideoCapture capture2(1);
    Transform trans2(2);
    Location location(motor);
    location.add_camera(&capture2, &trans2, true);
    location.set_current_location(cv::Vec2d(0, 0), cv::Vec2d(-1, 0));
    while (1) {
        Vec2d loc = location.get_location().first;
        cout << "machine:" << loc << endl;
        Vec2d ball_loc = location.get_ball().second;
        cout << "ball:" << ball_loc << endl;
        cout << endl;
        imshow("location", location.gen_ground_view());
        if (ball_loc[0] == 0 && ball_loc[1] == 0) {
            motor->stop();
        } else if (abs(ball_loc[0] - loc[0]) > 2000 || abs(ball_loc[1] - loc[1]) > 2000) {
            motor->stop();
        } else if ((loc[0] - ball_loc[0]) < 50 && (loc[0] - ball_loc[0]) > -50) {
            motor->stop();
        } else if (loc[0] - ball_loc[0] < 0) {
            if (loc[0] > 700) motor->stop();
            else motor->go(5000, -0.5);
        } else if (loc[0] - ball_loc[0] > 0) {
            if (loc[0] < -700) motor->stop();
            else motor->go(5000, 0.5);
        } else {
            motor->stop();
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
