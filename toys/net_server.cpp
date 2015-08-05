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

int main(int argc, char *argv[]) {
    bool simulate = true;
    if (argc != 2) {
        cout << "Usage: " << argv[0] << "[s/r]" << endl;
        cout << "  s: simulate only" << endl;
        cout << "  r: run with motor" << endl;
        cout << "  default to simulate" << endl;
    } else {
        if (argv[1][0] == 'r')
            simulate = false;
    }
    Motor *motor = Motor::get_instance(simulate);
    Location location(motor);
    Server server;
    server.add_on_receive_hook(on_instruction);
    VideoCapture *capture1;
    Transform *trans1;
    if (!simulate) {
        capture1 = new VideoCapture(1);
        trans1 = new Transform(1);
        location.add_camera(capture1, trans1);
    }
    location.add_server(&server);
    location.set_current_location(cv::Vec2d(1000, 300), cv::Vec2d(0, 1));
    int keep_cnt = 0;
    while (1) {
        double const motor_speed = 0.85;
        double const threshold = 200.0;
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
        waitKey(50);
    }
    if (!simulate) {
        delete capture1;
        delete trans1;
    }
    Motor::destroy_instance();
    return 0;
}

