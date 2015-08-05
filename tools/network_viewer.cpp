#include <iostream>
#include <mutex>
#include <unistd.h>
#include "../vision/ground.hpp"
#include "../network/client.hpp"

using namespace std;
using namespace cv;

Ground ground;
mutex frame_mutex;
Mat frame(200, 200, CV_8UC3, cv::Scalar::all(0));

void on_recieve(int, void const *buf, int) {
    double x, y, dx, dy, ballx, bally;
    int ball_state;
    sscanf((char const *)buf, "%lf%lf%lf%lf%d%lf%lf", &x, &y, &dx, &dy, &ball_state, &ballx, &bally);
    Vec2d position(x, y), direction(dx, dy);
    frame_mutex.lock();
    frame = ground.gen_ground_view();
    ground.draw_robot(frame, position, direction);
    if (ball_state == 1)
        ground.draw_ball(frame, Vec2d(ballx, bally));
    frame_mutex.unlock();
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " server_ip" << endl;
        return 1;
    }
    Client client((char const *)(void *)argv[1]);
    client.add_on_receive_hook(on_recieve);
    while (1) {
        frame_mutex.lock();
        imshow("client", frame);
        frame_mutex.unlock();
        waitKey(20);
    }
    return 0;
}

