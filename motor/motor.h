#ifndef MOTOR_H
#define MOTOR_H

#include <vector>
#include <opencv2/opencv.hpp>

class Motor
{
public:
    Motor();
    ~Motor();
    void go(double radius, double speed);
    void stop();
    std::vector<std::pair<cv::Vec2d, cv::Vec2d> > get_delta();
private:
    std::vector<std::pair<cv::Vec2d, cv::Vec2d> > history;
    enum { SPEED_SELECTION_COUNT = 6 };
    static double const const_speed[SPEED_SELECTION_COUNT]; // order: asc
    static void ctrl_send(int speed_left, int speed_right);
    static void ctrl_stop(int signo = 0);
};

#endif // MOTOR_H
