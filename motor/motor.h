#ifndef MOTOR_H
#define MOTOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <sys/time.h>

class Motor {
public:
    static Motor *get_instance(bool offline = false);
    static void destroy_instance();
    static bool has_instance();
    void go(double radius, double speed);
    void stop();
    std::vector<std::pair<cv::Vec2d, cv::Vec2d> > get_delta();
private:
    Motor(bool offline);
    ~Motor();
    static Motor *m_instance;
    bool offline;
    bool initialized;
    timeval dw_time_start;
    //double last_radius, last_speed;
    int last_vl, last_vr;
    std::vector<std::pair<cv::Vec2d, cv::Vec2d> > history;
    enum { MAX_SPEED_SELECTION = 50 };
    double get_speed(int speed_level, bool is_left_wheel);
    void update_location();
private:
    int m_motor_fd;
    void tty_init();
    void ctrl_init();
    void ctrl_send(void *buf, int len);
    void ctrl_move(int speed_left, int speed_right);
    void ctrl_stop();
};

#endif // MOTOR_H
