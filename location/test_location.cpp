#include <iostream>
#include "motor.h"
#include "location.h"

using namespace std;
using namespace cv;

int main() {
    Motor motor;
    Location location(&motor);
    location.set_current_location(cv::Vec2d(0, 0), cv::Vec2d(0, 1));
    while (1) {
        Vec2d loc = location.get_location().first;
        if (loc.dot(loc) > 500 * 500) {
            motor.stop();
            break;
        }
        motor.go(-2, 1.0);
    }
    return 0;
}
