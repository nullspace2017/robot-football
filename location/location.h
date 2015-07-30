#ifndef LOCATION_H
#define LOCATION_H

#include <opencv2/opencv.hpp>
#include <utility>
#include "motor.h"
#include "vision.h"

class Location {
public:
    Location(Motor *motor);
    std::pair<cv::Vec2d, cv::Vec2d> get_location();
    void set_current_location(cv::Vec2d position, cv::Vec2d direction);
    void try_vision_correct();
    float get_radius();

private:
    Motor *const motor;
    cv::Vec2d position;
    cv::Vec2d direction;
};

#endif // LOCATION_H
