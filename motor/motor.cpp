#include "motor.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <csignal>
#include <string>
#include <stdexcept>

using namespace std;
using namespace cv;

class MotorException: public std::exception {
public:
    MotorException(std::string msg): msg(msg) { }
private:
    char const *what() const throw() {
        return msg.c_str();
    }
    std::string msg;
};

Motor *Motor::m_instance = 0;

Motor::Motor(bool offline) {
    this->offline = offline;
    initialized = false;
    if (!offline) {
        tty_init();
        ctrl_init();
        static bool exit_binded = false;
        if (!exit_binded) {
            signal(SIGINT, [](int signo) {
                if (Motor::has_instance())
                    Motor::destroy_instance();
                signal(signo, SIG_DFL);
                raise(signo);
            });
            exit_binded = true;
        }
    }
    initialized = true;
    ctrl_stop();
}

Motor::~Motor() {
    stop();
    if (!offline) {
        close(m_motor_fd);
    }
}

Motor *Motor::get_instance(bool offline) {
    if (!m_instance)
        m_instance = new Motor(offline);
    if (offline != m_instance->offline)
        throw MotorException("Motor::get_instance(bool): offline flag does not match");
    return m_instance;
}

void Motor::destroy_instance() {
    if (m_instance) {
        delete m_instance;
        m_instance = 0;
    }
}

bool Motor::has_instance() {
    return m_instance != 0;
}

static double const d_wheel = 310.0;

void Motor::go(double radius, double speed) {
    update_location();
    auto select_best_speed = [&](double expected, bool is_left_wheel) {
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
    speed *= std::min(get_speed(MAX_SPEED_SELECTION, false), get_speed(MAX_SPEED_SELECTION, true));
    static double const eps = 1e-6;
    if (fabs(speed) < eps) {
        ctrl_stop();
    } else if (std::isinf(radius)) {
        int vl = select_best_speed(speed, true);
        int vr = select_best_speed(speed, false);
        ctrl_move(vl, vr);
    } else if (radius >= 0) {
        int vr = select_best_speed(speed, false);
        double vl_expected = get_speed(vr, false) * (radius - d_wheel / 2) / (radius + d_wheel / 2);
        int vl = select_best_speed(vl_expected, true);
        ctrl_move(vl, vr);
    } else { // radius < 0
        int vl = select_best_speed(speed, true);
        double vr_expected = get_speed(vl, true) * (radius + d_wheel / 2) / (radius - d_wheel / 2);
        int vr = select_best_speed(vr_expected, false);
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
    static double const sp10[12] = {0.0, 25.0, 50.0, 75.5, 101.0, 127.0, 154.0, 180.0, 208.5, 232.0, 257.5, 281.5};
    int level = speed_level < 0 ? -speed_level : speed_level;
    double speed;
    if (level > MAX_SPEED_SELECTION)
        level = MAX_SPEED_SELECTION;
    int a = level / 5;
    int b = level % 5;
    if (b == 0) {
        speed = sp10[a] * 2.0;
    } else {
        double speed0 = sp10[a] * 2.0;
        double speed1 = sp10[a + 1] * 2.0;
        speed = speed0 + (speed1 - speed0) * b / 5;
    }
    return speed_level < 0 ? -speed : speed;
}

void Motor::update_location() {
    timeval dw_time_end;
    gettimeofday(&dw_time_end, 0);
    time_t delta_time_1e6 = 1000000 * (dw_time_end.tv_sec - dw_time_start.tv_sec) +
            (dw_time_end.tv_usec - dw_time_start.tv_usec);
    if (delta_time_1e6 == 0) return;
    double delta_time = delta_time_1e6 / 1000000.0;
    dw_time_start = dw_time_end;

    double radius, speed;
    double vl = get_speed(last_vl, true);
    double vr = get_speed(last_vr, false);
    if (fabs(vl) > fabs(vr)) speed = vl;
    else speed = vr;
    radius = (vl + vr) / (vr - vl) * (d_wheel / 2);
    if (speed == 0) return;
    Vec2d move, direct;
    if (std::isinf(radius)) {
        move = Vec2d(0, speed * delta_time);
        direct = Vec2d(0, 1);
    } else if (radius >= 0) {
        double theta = speed * delta_time / (radius + d_wheel / 2);
        move = Vec2d(cos(theta) - 1, sin(theta)) * radius;
        direct = Vec2d(-sin(theta), cos(theta));
    } else {
        double theta = speed * delta_time / (-radius + d_wheel / 2);
        move = Vec2d(1 - cos(theta), sin(theta)) * -radius;
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
    cout << "    OK" << endl;
}

void Motor::ctrl_send(void *buf, int len) {
    if (!initialized) {
        throw MotorException("Motor::ctrl_send(void *, int): not initialized");
    }
    if (offline)
        return;
    if (write(m_motor_fd, buf, len) != len)
        throw MotorException("Motor::ctrl_send(void *, in): write error");
}

void Motor::ctrl_move(int speed_left, int speed_right) {
    last_vl = speed_left;
    last_vr = speed_right;
    char buf_ctrl2motor[5];
    buf_ctrl2motor[0] = HEAD3;
    buf_ctrl2motor[1] = speed_left;
    buf_ctrl2motor[2] = speed_right;
    buf_ctrl2motor[3] = 0; // what's this?
    buf_ctrl2motor[4] = AA_DISTANCE;
    ctrl_send(buf_ctrl2motor, 5);
}

void Motor::ctrl_stop() {
    last_vl = last_vr = 0;
    char buf_ctrl2motor[1];
    buf_ctrl2motor[0] = STOP;
    ctrl_send(buf_ctrl2motor, 1);
}
