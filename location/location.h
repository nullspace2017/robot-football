#ifndef LOCATION_H
#define LOCATION_H

#include <vector>
#include <utility>
#include <opencv2/opencv.hpp>
#include "motor.h"
#include "vision.h"
#include "capture.hpp"

class Location {
public:
    Location(Motor *motor);
    ~Location();
    void add_camera(cv::VideoCapture *capture, Transform *trans);
    std::pair<cv::Vec2d, cv::Vec2d> get_location();
    void set_current_location(cv::Vec2d position, cv::Vec2d direction);
    void try_vision_correct();
    cv::Mat gen_ground_view(double mm_per_pixel = 8.0);
    float get_radius();
private:
    Motor *const motor;
    Ground const ground;
    enum { E_CAMEAR_COUNT = 2 };
    std::vector<Capture *> v_capture;
    std::vector<Vision *> v_vision;
    cv::Vec2d position;
    cv::Vec2d direction;
};

#endif // LOCATION_H
