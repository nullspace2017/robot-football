#include <iostream>
#include <cmath>
#include <mutex>
#include <unistd.h>
#include "../vision/ground.hpp"
#include "../network/client.hpp"

using namespace std;
using namespace cv;

Ground ground;
mutex frame_mutex;
Mat frame(200, 200, CV_8UC3, cv::Scalar::all(0));

bool has_dest = false;
Vec2d dest_pos, dest_direct;

void on_recieve(int, void const *buf, int) {
    double x, y, dx, dy, ballx, bally;
    int ball_state;
    sscanf((char const *)buf, "%lf%lf%lf%lf%d%lf%lf", &x, &y, &dx, &dy, &ball_state, &ballx, &bally);
    Vec2d position(x, y), direction(dx, dy);
    frame_mutex.lock();
    frame = ground.gen_ground_view();
    ground.draw_robot(frame, position, direction);
    if (has_dest)
        ground.draw_robot(frame, dest_pos, dest_direct, Scalar(255, 0, 0));
    if (ball_state == 1)
        ground.draw_ball(frame, Vec2d(ballx, bally));
    frame_mutex.unlock();
}

void onMouse(int event, int x, int y, int, void *p_client) {
    Client *client = (Client *)p_client;
    static Point mousedown;
    if (event == CV_EVENT_LBUTTONDOWN) {
        mousedown = Point(x, y);
    } else if (event == CV_EVENT_LBUTTONUP) {
        Point mouseup = Point(x, y);
        if (mouseup == mousedown) {
            cout << "Drag mouse to decide a direction" << endl;
        } else {
            Vec2d pos = ground.uv_to_xy(mousedown.x, mousedown.y);
            Vec2d direct = ground.uv_to_xy(mouseup.x, mouseup.y) - pos;
            direct /= sqrt(direct.dot(direct));
            char buf[128];
            snprintf(buf, 128, "%f %f %f %f", pos[0], pos[1], direct[0], direct[1]);
            client->send_to_server((void *)buf, strnlen(buf, 128));
            has_dest = true;
            dest_pos = pos;
            dest_direct = direct;
        }
    }
}

int main(int argc, char *argv[]) {
    char const *server_ip;
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " server_ip" << endl;
        cout << "  default to 127.0.0.1" << endl;
        server_ip = "127.0.0.1";
    } else {
        server_ip = (char const *)(void *)argv[1];
    }
    Client client(server_ip);
    client.add_on_receive_hook(on_recieve);
    while (1) {
        frame_mutex.lock();
        imshow("client", frame);
        setMouseCallback("client", onMouse, &client);
        frame_mutex.unlock();
        waitKey(20);
    }
    return 0;
}

