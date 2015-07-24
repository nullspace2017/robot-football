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

double const Motor::const_speed[Motor::SPEED_SELECTION_COUNT] = {
    0.0, 0.3, 0.6, 0.9, 1.2, 1.5
};

class MotorException: public std::exception {
public:
    MotorException(std::string msg): msg(msg) { }
private:
    char const *what() const throw() {
        return msg.c_str();
    }
    std::string msg;
};

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

static void sig_ctrl_stop(int signo) {
    Motor::get_instance()->stop();
    signal(signo, SIG_DFL);
    raise(signo);
}

Motor::Motor() {
    tty_init();
    ctrl_init();
    static bool exit_binded = false;
    if (!exit_binded) {
        signal(SIGINT, sig_ctrl_stop);
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

void Motor::go(double, double) {
    history.push_back(make_pair(Vec2d(1.0, 0.01), Vec2d(sin(CV_PI / 180), cos(CV_PI / 180))));
}

void Motor::stop() {
    ctrl_stop();
}

std::vector<std::pair<cv::Vec2d, cv::Vec2d> > Motor::get_delta() {
    auto res = history;
    history.clear();
    return res;
}

void Motor::tty_init() {
    char const dev[] = "/dev/ttyUSB0";
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
