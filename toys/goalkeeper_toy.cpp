#include <iostream>
#include <unistd.h>
#include "../motor/motor.h"
#include "../location/location.h"

using namespace std;
using namespace cv;

int main() {
    Motor *motor = Motor::get_instance();
    VideoCapture capture2(1);
    Transform trans2(2);
    Location location(motor);
    location.add_camera(&capture2, &trans2);
    location.set_current_location(cv::Vec2d(0, 0), cv::Vec2d(-1, 0));
    while (1) {
        Vec2d loc = location.get_location().first;
        cout << "machine:" << loc << endl;
        Vec2d ball_loc = location.get_ball().second;
        cout << "ball:" << ball_loc << endl;
        imshow("location", location.gen_ground_view());
        /*if (ball_loc[0] <= 350 || ball_loc[0] >= 1350) {
            motor->stop();
        } else */if (loc[0] - ball_loc[0] < 0) {
            motor->go(5000, -0.5);
        } else if (loc[0] - ball_loc[0] > 0) {
            motor->go(5000, 0.5);
        } else {
            motor->stop();
        }
        waitKey(50);
    }
    Motor::destroy_instance();
    return 0;
}
