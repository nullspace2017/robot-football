#ifndef MOTOR_H
#define MOTOR_H

#include <vector>
#include <opencv2/opencv.hpp>

class Motor
{
public:
    Motor();
    void go(double radius, double speed);
    std::vector<std::pair<cv::Vec2d, cv::Vec2d> > get_delta();
private:
    std::vector<std::pair<cv::Vec2d, cv::Vec2d> > history;
};

#endif // MOTOR_H
