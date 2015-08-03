#include <iostream>
#include <unistd.h>
#include "../motor/motor.h"
#include "../location/location.h"

using namespace std;
using namespace cv;

int main() {
    Motor *motor = Motor::get_instance();
    VideoCapture capture1(1);
    Transform trans1(1);
    Location location(motor);
    location.add_camera(&capture1, &trans1);
    location.set_current_location(cv::Vec2d(1500, 2200), cv::Vec2d(0, 1));
    while (1) {
        auto loc = location.get_location();
        imshow("location", location.gen_ground_view());
        cout << loc.first << endl;
        if ((loc.first - Vec2d(900, 2200)).dot(loc.second) > 0) {
            motor->go(300, .9);
        } else {
            motor->go(450, .9);
        }
        waitKey(20);
    }
    Motor::destroy_instance();
    return 0;
}
