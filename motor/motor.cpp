#include "motor.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <csignal>
#include <string>
#include <stdexcept>

using namespace std;
using namespace cv;

Motor *Motor::m_instance = 0;

Motor::Motor() {
    tty_init();
    ctrl_init();
    static bool exit_binded = false;
    if (!exit_binded) {
        signal(SIGINT, [](int signo) {
            Motor::get_instance()->stop();
            signal(signo, SIG_DFL);
            raise(signo);
        });
        exit_binded = true;
    }
}

Motor::~Motor() {
    stop();
    close(m_motor_fd);
}

Motor *Motor::get_instance() {
    if (!m_instance)
        m_instance = new Motor;
    return m_instance;
}

void Motor::destroy_instance() {
    if (m_instance) {
        delete m_instance;
        m_instance = 0;
    }
}

static double const d_wheel = 310.0;

void Motor::go(double radius, double speed) {
    update_location();
    auto select_best_speed = [&](double expected, bool is_left_wheel = false) {
        double min_delta = 1e20;
        int ind = 0;
        for (int i = -MAX_SPEED_SELECTION; i <= MAX_SPEED_SELECTION; i++) {
            double delta = fabs(get_speed(i, is_left_wheel) - expected);
            if (delta < min_delta) {
                min_delta = delta;
                ind = i;
            }
        }
        return ind;
    };
    auto update_last_record = [&](int vl_ind, int vr_ind) {
        double vl = get_speed(vl_ind);
        double vr = get_speed(vr_ind);
    };
    if (std::isinf(radius)) {
        int vl = select_best_speed(speed, true);
        int vr = select_best_speed(speed, false);
        update_last_record(vl, vr);
        ctrl_move(vl, vr);
    }
}

void Motor::stop() {
    ctrl_stop();
}

std::vector<std::pair<cv::Vec2d, cv::Vec2d> > Motor::get_delta() {
    update_location();
    auto res = history;
    history.clear();
    return res;
}

double Motor::get_speed(int speed_level, bool) { // mm/s
    return speed_level * 10.0;
}

void Motor::update_location() {
    struct timeval dw_time_end;
    gettimeofday(&dw_time_end, 0);
    time_t delta_time_1e6 = 1000000 * (dw_time_end.tv_sec - dw_time_start.tv_sec) +
            (dw_time_end.tv_usec - dw_time_start.tv_usec);
    if (delta_time_1e6 == 0) return;
    double delta_time = delta_time_1e6 / 1000000.0;
    dw_time_start = dw_time_end;
    if (last_speed == 0) return;
    Vec2d move, direct;
    if (std::isinf(last_radius)) {
        move = Vec2d(0, last_speed * delta_time);
        direct = Vec2d(0, 1);
    } else if (last_radius == 0.0 || last_radius == -0.0) {
        double theta = last_speed * delta_time / (d_wheel / 2);
        move = Vec2d(0, 0);
        direct = Vec2d(-sin(theta), cos(theta));
    } else if (last_radius >= 0) {
        double theta = last_speed * delta_time / (last_radius + d_wheel / 2);
        move = Vec2d(sin(theta), cos(theta) - 1) * last_radius;
        direct = Vec2d(-sin(theta), cos(theta));
    } else {
        double theta = last_speed * delta_time / (-last_radius + d_wheel / 2);
        move = Vec2d(sin(theta), 1 - cos(theta)) * last_radius;
        direct = Vec2d(sin(theta), cos(theta));
    }
    history.push_back(make_pair(move, direct));
}

#define STD_IN 0    // stdin fd
#define STOP 100    // stop motor now
#define VER 126     // read version (stable)
#define CLR 127     // clear sn, len_cmd, t_cur, t_end (stable)

#define BASIC_SPEED 5
#define ANGLE_4WAY 90
#define ONE_STEP_DIST 50
#define HEAD3 102           // cmd head, len = 3
#define AA_ANGLE -100       // acc with angle set, len = 3
#define AA_DISTANCE -101
#define AA -102             // acc without late echo, len = 2
#define AA_ONLY -103        // acc only, len = 2


class MotorException: public std::exception {
public:
    MotorException(std::string msg): msg(msg) { }
private:
    char const *what() const throw() {
        return msg.c_str();
    }
    std::string msg;
};

void Motor::tty_init() {
    static char const dev[] = "/dev/ttyUSB0";
    if ((m_motor_fd = open(dev, O_RDWR | O_NOCTTY)) < 0)
        throw MotorException(string("Motor::tty_init(): perror: ") + dev);
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tio.c_iflag = INPCK;                                  // 输入模式
    tio.c_cflag = B19200 | CS8 | CLOCAL | CREAD | PARENB; // 控制模式
    tio.c_cc[VMIN] = 1;                                   // 特殊控制模式
    if (tcsetattr(m_motor_fd, TCSANOW, &tio) < 0)         // TCSANOW: set attr now
        throw MotorException("Motor::tty_init(): perror: tcsetattr");
}

void Motor::ctrl_init() {
    cout << "Motor::ctrl_init ...";
    char buf[4] = {CLR, VER, };
    if (write(m_motor_fd, buf, 2) != 2)
        throw MotorException("Motor::ctrl_init(): write fd error");
    if (read(m_motor_fd, buf, sizeof(buf)) < 0)
        throw MotorException("Motor::ctrl_init(): read fd error");
    ctrl_stop();
    cout << "    OK" << endl;
}

void Motor::ctrl_send(void *buf, int len) {
    if (write(m_motor_fd, buf, len) != len)
        throw MotorException("Motor::ctrl_send(void *, in): write error");
}

void Motor::ctrl_move(int speed_left, int speed_right) {
    char buf_ctrl2motor[5];
    buf_ctrl2motor[0] = HEAD3;
    buf_ctrl2motor[1] = speed_left;
    buf_ctrl2motor[2] = speed_right;
    buf_ctrl2motor[3] = 0; // what's this?
    buf_ctrl2motor[4] = AA_DISTANCE;
    ctrl_send(buf_ctrl2motor, 5);
}

void Motor::ctrl_stop() {
    char buf_ctrl2motor[1];
    buf_ctrl2motor[0] = STOP;
    ctrl_send(buf_ctrl2motor, 1);
}
