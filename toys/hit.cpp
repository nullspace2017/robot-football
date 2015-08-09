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
    location.set_current_location(cv::Vec2d(666, 1000), cv::Vec2d(-0.1, 0.9));

    pair<Location::BALLSTATE, Vec2d> ball;
    bool hold_ball = false, adjust_dir = false;
    while (1) {
        imshow("location", location.gen_ground_view());
        Vec2d loc = location.get_location().first, dir = location.get_location().second;
#if 1
        pair<Location::BALLSTATE, Vec2d> ball_cur = location.get_ball();
        if (ball_cur.first == Location::BALL_HAS) {
            ball = ball_cur;
        }
        if (ball.first == Location::BALL_NO) {
            motor->go(200, 0.2);
            waitKey(50);
            continue;
        }
#else
        ball.second = Vec2d(1096, 1440);
#endif
        Vec2d des_dir = Vec2d(900, 4400) - ball.second, dist = loc - ball.second;
        //des_dir = Vec2d(-0.01, 0.99);
        cout << sqrt(dist.ddot(dist)) << endl;
        double dist_val = sqrt(dist.ddot(dist));
        if (dist_val < 200 || hold_ball) {
            hold_ball = true;
            double cosa = dir.ddot(des_dir) / sqrt(dir.ddot(dir)) / sqrt(des_dir.ddot(des_dir));
            //cosa = 1;
            if (cosa < 0.99 && !adjust_dir) {
                if (dir.ddot(Vec2d(des_dir[1], -des_dir[0])) < 0)
                    motor->go(-200, 0.1);
                else
                    motor->go(200, 0.1);
                waitKey(50);
            } else {
                adjust_dir = true;
                Vec2d door_dist = Vec2d(900, 4400) - loc;
                if (sqrt(door_dist.ddot(door_dist)) > 100) {
                    if (door_dist.ddot(Vec2d(dir[1], -dir[0])) < 0)
                        motor->go(1500, 0.2);
                    else 
                        motor->go(-1500, 0.2);
                } else 
                    motor->stop();
                waitKey(50);
            }
        } else {
            double r = location.get_radius(loc, dir, ball.second, des_dir);
            cout << loc << dir << ball.second << des_dir << r << endl;
            motor->go(r, 0.2);
            waitKey(50);
        }
    }
    Motor::destroy_instance();
    return 0;
}
