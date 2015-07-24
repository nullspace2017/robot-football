#include "motor.h"
#include "signal.h"

using namespace std;
using namespace cv;

double const Motor::const_speed[Motor::SPEED_SELECTION_COUNT] = {
    0.0, 0.3, 0.6, 0.9, 1.2, 1.5
};


Motor::Motor() {
    static bool exit_bind = false;
    if (!exit_bind) {
        signal(SIGINT, Motor::ctrl_stop);
        exit_bind = true;
    }
}

Motor::~Motor() {
    stop();
}

void Motor::go(double, double) {
    history.push_back(make_pair(Vec2d(1.0, 1.0), Vec2d(sin(CV_PI / 180), cos(CV_PI / 180))));
}

void Motor::stop() {
    go(1.0 / 0.0, 0.0);
}

std::vector<std::pair<cv::Vec2d, cv::Vec2d> > Motor::get_delta() {
    auto res = history;
    history.clear();
    return res;
}

void Motor::ctrl_send(int speed_left, int speed_right) {
    // ...
}

void Motor::ctrl_stop(int signo) {
    // ...
    if (signo != 0) {
        signal(signo, SIG_DFL);
        raise(signo);
    }
}
