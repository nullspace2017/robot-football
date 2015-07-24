#include <iostream>
#include "motor.h"
#include "location.h"

using namespace std;
using namespace cv;

int main() {
    Motor *motor = Motor::get_instance();
    Location location(motor);
    location.set_current_location(cv::Vec2d(0, 0), cv::Vec2d(0, 1));
    while (1) {
        Vec2d loc = location.get_location().first;
        cout << loc << endl;
        if (loc.dot(loc) > 100 * 100) {
            motor->stop();
            break;
        }
        motor->go(-2, 1.0);
    }
    Motor::destroy_instance();
    return 0;
}
