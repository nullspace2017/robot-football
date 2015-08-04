#ifndef LOCATION_H
#define LOCATION_H

#include <vector>
#include <utility>
#include "../motor/motor.h"
#include "../vision/vision.h"
#include "capture.hpp"

class Location {
public:
    Location(Motor *motor);
    ~Location();
    void add_camera(cv::VideoCapture *capture, Transform *trans);
    std::pair<cv::Vec2d, cv::Vec2d> get_location();
    enum BALLSTATE { BALL_NO, BALL_LAST, BALL_HAS };
    std::pair<Location::BALLSTATE, cv::Vec2d> get_ball(); // should get_location first
    void set_current_location(cv::Vec2d position, cv::Vec2d direction);
    cv::Mat gen_ground_view(); // should get_location, get_ball first
    double get_radius(cv::Vec2d, cv::Vec2d);
private:
    Motor *const motor;
    Ground ground;
    std::vector<Capture *> v_capture;
    std::vector<Vision *> v_vision;
    cv::Vec2d position;
    cv::Vec2d direction;
    BALLSTATE ball_state;
    cv::Vec2d ball_pos;
    void try_vision_correct();
};

#endif // LOCATION_H
