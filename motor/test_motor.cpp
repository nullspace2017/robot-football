#include <iostream>
#include <cmath>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "motor.h"

using namespace std;
using namespace cv;

int main() {
    Motor *motor = Motor::get_instance();
    motor->go(-2000, 0.5);
    usleep(1000000);
    motor->stop();
    Motor::destroy_instance();
    return 0;
}
