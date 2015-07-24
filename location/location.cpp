#include "location.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

using namespace std;
using namespace cv;

Location::Location(Motor *motor): motor(motor) { }

std::pair<cv::Vec2d, cv::Vec2d> Location::get_location() {
    vector<pair<Vec2d, Vec2d> > v_delta = motor->get_delta();
    for (auto it = v_delta.begin(); it != v_delta.end(); it++) {
        Vec2d off = it->first, delta = it->second;
        Vec2d n(direction[1], -direction[0]);
        position += off[0] * n + off[1] * direction;
        direction = delta[0] * n + delta[1] * direction;
        direction /= sqrt(direction.dot(direction));
    }
    return make_pair(position, direction);
}

void Location::set_current_location(cv::Vec2d position, cv::Vec2d direction) {
    this->position = position;
    double m = direction.dot(direction);
    if (m <= 0)
        throw invalid_argument("Location::set_current_location: direction == Vec2d(0.0, 0.0)");
    direction /= sqrt(m);
    this->direction = direction;
}

void Location::try_vision_correct() {
    throw invalid_argument("not implemented method Location::try_vision_correct");
}
