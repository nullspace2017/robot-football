#include "location.h"
#include <iostream>
#include <stdexcept>

using namespace std;
using namespace cv;

Location::Location(Motor *motor): motor(motor) { }

std::pair<cv::Vec2d, cv::Vec2d> Location::get_location() {
    vector<pair<Vec2d, Vec2d> > delta = motor->get_delta();
    // ...
    return make_pair(position, direction);
}

void Location::set_current_location(cv::Vec2d position, cv::Vec2d direction) {
    this->position = position;
    this->direction = direction;
}

void Location::try_vision_correct() {
    throw invalid_argument("not implemented method Location::try_vision_correct");
}
