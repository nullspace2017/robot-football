#include <iostream>
#include <unistd.h>
#include "../motor/motor.h"
#include "location.h"

using namespace std;
using namespace cv;

int main() {
    Motor *motor = Motor::get_instance();
    VideoCapture capture1(1);
    Transform trans1(1);
    Location location(motor);
    location.add_camera(&capture1, &trans1);
    location.set_current_location(cv::Vec2d(0, 0), cv::Vec2d(0, 1));
    Server server;
    location.add_server(&server);
    while (1) {
        Vec2d loc = location.get_location().first;
        imshow("location", location.gen_ground_view());
        cout << loc << endl;
        if (loc.dot(loc) > 4000 * 4000) {
            motor->stop();
            break;
        }
        motor->go(-5000, 0.4);
        waitKey(50);
    }
    Motor::destroy_instance();
    return 0;
}
