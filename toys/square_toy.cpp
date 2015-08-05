#include <iostream>
#include <unistd.h>
#include <cmath>
#include "../motor/motor.h"
#include "../location/location.h"

using namespace std;
using namespace cv;

const int Dir_Num = 4;
const int edges[Dir_Num] = {200, 2100, 1600, 200};
const int edge_axis[Dir_Num] = {0, 1, 0, 1};
const double inf = 1 / 0.0;
#define Angle_Eps1 0.9
#define Angle_Eps2 0.999
#define Dist_Eps1 200
#define Dist_Eps2 50

Vec2d preDir(0, 0);
int num = -1;

Vec2d check_dir(Vec2d pos) {
    double min_dist = 2200;
    int min_no = -1;
    bool flag = false;
    Vec2d res;
    for (int i = 0; i < Dir_Num; i ++) {
        double dist = fabs(pos[edge_axis[i]] - edges[i]);
        if (dist < min_dist) {
            min_dist = dist;
            min_no = i;
        }
        if (dist < Dist_Eps1) {
            int next = (i + 1) % 4;
            flag = true;
            dist = fabs(pos[edge_axis[next]] - edges[next]);
            if (dist < Dist_Eps2) {
                min_dist = dist;
                min_no = next;
            }
            if (i == 0) {
               dist = fabs(pos[edge_axis[Dir_Num-1]] - edges[Dir_Num-1]);
               if (dist < Dist_Eps2) {
                   min_dist = dist;
                   min_no = 0;
               }
            }
            break;
        }
    }
    if (flag) {
        switch (min_no) {
        case 0: res = Vec2d(0, 1); break;
        case 1: res = Vec2d(1, 0); break;
        case 2: res = Vec2d(0, -1); break;
        case 3: res = Vec2d(-1, 0); break;
        default: res = Vec2d(0, 0); break;   // impossible
        }
    }
    else {
        double half_radical_2 = pow(2, 0.5) / 2;
        switch (min_no) {
        case 0: res = Vec2d(-half_radical_2, half_radical_2); break;
        case 1: res = Vec2d(half_radical_2, half_radical_2); break;
        case 2: res = Vec2d(half_radical_2, -half_radical_2); break;
        case 3: res = Vec2d(-half_radical_2, -half_radical_2); break;
        default: res = Vec2d(0, 0);
        }
    }
    
    double dist = fabs(pos[edge_axis[min_no]] - edges[min_no]);
    if (num != -1) {
        if (preDir != res) {
            if (flag && (dist > Dist_Eps2 || num < 5))
                res = preDir;
        }
    }
    if (preDir == res) {
        num ++;
    }
    else {
        num = 0;
        preDir = res;
    }

    return res;
}


bool deviate_to_left(Vec2d _right_dir, Vec2d _cur_dir) {
    Vec3d right_dir(_right_dir[0], _right_dir[1], 0);
    Vec3d cur_dir(_cur_dir[0], _cur_dir[1], 0);
    return right_dir.cross(cur_dir)[2] > 0;
}


int main() {
    Motor *motor = Motor::get_instance();
    VideoCapture capture1(1);
    Transform trans1(1);
    Location location(motor);
    location.add_camera(&capture1, &trans1);
    
    location.set_current_location(cv::Vec2d(900, 400), Vec2d(-1, 0));
    
    while (1) {
        auto loc = location.get_location();
        imshow("location", location.gen_ground_view());
        cout << loc.first << endl;
        Vec2d right_dir = check_dir(loc.first);
        if (right_dir.dot(loc.second) < Angle_Eps1) {
            bool turn_left = deviate_to_left(right_dir, loc.second);
            if (turn_left)
                motor->go(0, -.1);
            else
                motor->go(0, .1);
        }
        else if (right_dir.dot(loc.second) < Angle_Eps2) {
            bool turn_left = deviate_to_left(right_dir, loc.second);
            if (turn_left)
                motor->go(0, -.04);
            else
                motor->go(0, .04);
        }
        else {
            motor->go(inf, .4);
        }
        waitKey(10);
    }
    Motor::destroy_instance();
    return 0;
}


