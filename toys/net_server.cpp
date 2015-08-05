#include <iostream>
#include <cstdio>
#include <deque>
#include <utility>
#include <mutex>
#include <unistd.h>
#include "../motor/motor.h"
#include "../location/location.h"

using namespace std;
using namespace cv;

mutex dest_mutex;
deque<pair<Vec2d, Vec2d> > v_dest;

void on_instruction(int, void const *buf, int) {
    double x, y, dx, dy;
    sscanf((char const *)buf, "%lf%lf%lf%lf", &x, &y, &dx, &dy);
    dest_mutex.lock();
    v_dest.clear();
    v_dest.push_back(make_pair(Vec2d(x, y), Vec2d(dx, dy)));
    dest_mutex.unlock();
}

int main() {
    Motor *motor = Motor::get_instance(true);
    //VideoCapture capture1(1);
    //Transform trans1(1);
    Location location(motor);
    Server server;
    server.add_on_receive_hook(on_instruction);
    //location.add_camera(&capture1, &trans1);
    location.add_server(&server);
    location.set_current_location(cv::Vec2d(1000, 300), cv::Vec2d(0, 1));
    int keep_cnt = 0;
    while (1) {
        double const motor_speed = 0.5;
        double const threshold = 100.0;
        pair<Vec2d, Vec2d> loc = location.get_location();
        imshow("location", location.gen_ground_view());
        if (keep_cnt > 0) {
            keep_cnt--;
        } else {
            pair<Vec2d, Vec2d> next_dest;
            bool has_next = false;
            dest_mutex.lock();
            while (!v_dest.empty()) {
                next_dest = v_dest.front();
                Vec2d delta_s = loc.first - next_dest.first;
                if (delta_s.dot(delta_s) > threshold * threshold) {
                    has_next = true;
                    break;
                }
                v_dest.pop_front();
            }
            dest_mutex.unlock();
            if (has_next) {
                double r = location.get_radius(loc.first, loc.second, next_dest.first, next_dest.second);
                motor->go(r, motor_speed);
            } else {
                motor->stop();
            }
        }
        waitKey(20);
    }
    Motor::destroy_instance();
    return 0;
}

