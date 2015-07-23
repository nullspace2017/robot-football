#include "motor.h"

using namespace std;
using namespace cv;

Motor::Motor() { }

void Motor::go(double, double) {
    history.push_back(make_pair(Vec2d(1.0, 1.0), Vec2d(sin(CV_PI / 180), cos(CV_PI / 180))));
}

std::vector<std::pair<cv::Vec2d, cv::Vec2d> > Motor::get_delta() {
    auto res = history;
    history.clear();
    return res;
}
