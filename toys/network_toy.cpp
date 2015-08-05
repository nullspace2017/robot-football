#include <iostream>
#include <mutex>
#include <unistd.h>
#include "../motor/motor.h"
#include "../location/location.h"
#include "../network/client.hpp"

using namespace std;
using namespace cv;

Location *location;
mutex frame_mutex;
Mat frame(200, 200, CV_8UC3, cv::Scalar::all(0));

void on_recieve(int, void *buf, int) {
    double x, y, dx, dy;
    sscanf((char const *)buf, "%lf%lf%lf%lf", &x, &y, &dx, &dy);
    Vec2d pos(x, y), cur(dx, dy);
    location->set_current_location(pos, cur);
    frame_mutex.lock();
    frame = location->gen_ground_view();
    frame_mutex.unlock();
}

int main() {
    Motor *motor = Motor::get_instance(true);
    location = new Location(motor);
    Client client("127.0.0.1");
    client.add_on_receive_hook(on_recieve);
    while (1) {
        frame_mutex.lock();
        imshow("client", frame);
        frame_mutex.unlock();
        waitKey(20);
    }
    Motor::destroy_instance();
    return 0;
}

