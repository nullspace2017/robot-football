#include "location.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

using namespace std;
using namespace cv;

Location::Location(Motor *motor): motor(motor) { }

void Location::add_camera(VideoCapture *capture, Transform *trans) {
    v_capture.push_back(capture);
    v_vision.push_back(new Vision(cvRound(capture->get(CAP_PROP_FRAME_HEIGHT)),
                                  cvRound(capture->get(CAP_PROP_FRAME_WIDTH)), trans));
}

std::pair<cv::Vec2d, cv::Vec2d> Location::get_location() {
    vector<pair<Vec2d, Vec2d> > v_delta = motor->get_delta();
    for (auto it = v_delta.begin(); it != v_delta.end(); it++) {
        Vec2d off = it->first, delta = it->second;
        Vec2d n(direction[1], -direction[0]);
        position += off[0] * n + off[1] * direction;
        direction = delta[0] * n + delta[1] * direction;
        direction /= sqrt(direction.dot(direction));
    }
    try_vision_correct();
    return make_pair(position, direction);
}

void Location::set_current_location(cv::Vec2d position, cv::Vec2d direction) {
    this->position = position;
    double m = direction.dot(direction);
    if (m <= 0)
        throw invalid_argument("Location::set_current_location: direction == Vec2d(0.0, 0.0)");
    direction /= sqrt(m);
    this->direction = direction;
}

void Location::try_vision_correct() {
    for (size_t i = 0; i < v_capture.size(); i++) {
        Mat frame;
        *v_capture[i] >> frame;
        imshow("frame", frame);
        v_vision[i]->input(frame);
        Vec2f pos;
        Vec2f direct;
        double location_confidence;
        v_vision[i]->get_location(pos, direct, location_confidence);
        if (location_confidence > 0.5) {
            if (direct.dot(direction) < 0) {
                direct = -direct;
                pos = Vec2f(ground.width - pos[0], ground.height - pos[1]);
            }
            position = pos;
            direction = direct;
            break;
        }
    }
}

cv::Mat Location::gen_ground_view(double mm_per_pixel) {
    auto xy_to_ground_point = [&](double xw, double yw) {
        return Point(xw / mm_per_pixel, (ground.height - yw) / mm_per_pixel);
    };
    Mat view_ground(ground.height / mm_per_pixel, ground.width / mm_per_pixel, CV_8UC3, Scalar::all(0));
    for (size_t i = 0; i < ground.lines.size(); i++) {
        Vec4f const &l(ground.lines[i]);
        line(view_ground, xy_to_ground_point(l[0], l[1]), xy_to_ground_point(l[2], l[3]), Scalar::all(255), 2);
    }
    line(view_ground, xy_to_ground_point(position[0], position[1]), xy_to_ground_point(position[0], position[1]), Scalar(0, 255, 0), 5);
    line(view_ground, xy_to_ground_point(position[0], position[1]),
         xy_to_ground_point(position[0] + 10000 * direction[0], position[1] + 10000 * direction[1]), Scalar(0, 255, 0), 1);
    return view_ground;
}
