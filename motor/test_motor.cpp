#include <iostream>
#include <cmath>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "motor.h"

using namespace std;
using namespace cv;

#if 0
int main() {
    Motor motor;
    double sum_distance = 0.0, sum_angle = 0.0;
    while (sum_distance < 1000 && sum_angle < CV_PI) {
        motor.go(1 / 0.0, 1.0);
        std::vector<std::pair<cv::Vec2d, cv::Vec2d> > delta = motor.get_delta();
        for (auto it = delta.begin(); it != delta.end(); it++) {
            Vec2d dist = it->first;
            Vec2d angle = it->second;
            sum_distance += sqrt(dist.dot(dist));
            sum_angle += abs(asin(angle[0]));
        }
    }
    return 0;
}
#else
int main() {
    Motor *motor = Motor::get_instance();
    motor->go(-2000, 0.5);
    usleep(1000000);
    motor->stop();
    Motor::destroy_instance();
    return 0;
}

#endif
