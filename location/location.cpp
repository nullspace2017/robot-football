#include "location.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

using namespace std;
using namespace cv;

Location::Location(Motor *motor): motor(motor) { }

Location::~Location() {
    for (size_t i = 0; i < v_capture.size(); i++)
        delete v_capture[i];
    for (size_t i = 0; i < v_vision.size(); i++)
        delete v_vision[i];
}

void Location::add_camera(cv::VideoCapture *capture, Transform *trans) {
    v_capture.push_back(new Capture(capture));
    v_vision.push_back(new Vision(cvRound(capture->get(CV_CAP_PROP_FRAME_HEIGHT)),
                                  cvRound(capture->get(CV_CAP_PROP_FRAME_WIDTH)), trans));
}

std::pair<cv::Vec2d, cv::Vec2d> Location::get_location() {
    std::vector<std::pair<cv::Vec2d, cv::Vec2d> > v_delta = motor->get_delta();
    for (auto it = v_delta.begin(); it != v_delta.end(); it++) {
        cv::Vec2d off = it->first, delta = it->second;
        cv::Vec2d n(direction[1], -direction[0]);
        position += off[0] * n + off[1] * direction;
        direction = delta[0] * n + delta[1] * direction;
        direction /= sqrt(direction.dot(direction));
    }
    try_vision_correct();
    return std::make_pair(position, direction);
}

void Location::set_current_location(cv::Vec2d position, cv::Vec2d direction) {
    this->position = position;
    double m = direction.dot(direction);
    if (m <= 0)
        throw std::invalid_argument("Location::set_current_location: direction == cv::Vec2d(0.0, 0.0)");
    direction /= sqrt(m);
    this->direction = direction;
}

void Location::try_vision_correct() {
    for (size_t i = 0; i < v_capture.size(); i++) {
        cv::Mat frame;
        *v_capture[i] >> frame;
        imshow("frame", frame);
        v_vision[i]->input(frame);
        cv::Vec2f pos;
        cv::Vec2f direct;
        double location_confidence;
        v_vision[i]->get_location(pos, direct, location_confidence);
        if (location_confidence > 0.5) {
            if (direct.dot(direction) < 0) {
                direct = -direct;
                pos = cv::Vec2f(ground.width - pos[0], ground.height - pos[1]);
            }
            position = pos;
            direction = direct;
            break;
        }
    }
}

cv::Mat Location::gen_ground_view(double mm_per_pixel) {
    auto xy_to_ground_point = [&](double xw, double yw) {
        return cv::Point(xw / mm_per_pixel, (ground.height - yw) / mm_per_pixel);
    };
    cv::Mat view_ground(ground.height / mm_per_pixel, ground.width / mm_per_pixel, CV_8UC3, cv::Scalar::all(0));
    for (size_t i = 0; i < ground.lines.size(); i++) {
        cv::Vec4f const &l(ground.lines[i]);
        line(view_ground, xy_to_ground_point(l[0], l[1]), xy_to_ground_point(l[2], l[3]), cv::Scalar::all(255), 2);
    }
    line(view_ground, xy_to_ground_point(position[0], position[1]), xy_to_ground_point(position[0], position[1]), cv::Scalar(0, 255, 0), 5);
    line(view_ground, xy_to_ground_point(position[0], position[1]),
         xy_to_ground_point(position[0] + 10000 * direction[0], position[1] + 10000 * direction[1]), cv::Scalar(0, 255, 0), 1);
    return view_ground;
}

template <typename T>
static inline T sqr(T x) {
    return x * x;
}

double Location::get_radius(Vec2d cur_pos, Vec2d cur_dir) {
    Vec2d /*cur_pos(0, 0), cur_dir(0, 1), */des_pos(900, 900), des_dir(0, 1);
    Vec2d rdir, ori, norm; // direction of radius, origin of circle;
    Vec2d mov = (cur_pos - des_pos) / 2;
    double r;
    if (cur_dir.ddot(des_dir) > 0) {
        if (abs(des_dir[0]) < 1e-6) rdir = Vec2d(1, 0);
        else if (abs(des_dir[1]) < 1e-6) rdir = Vec2d(0, 1);
        else  {
            rdir = Vec2d(1, des_dir[1] / des_dir[0]);
            rdir /= sqrt(1 + sqr(rdir[1]));
        }
        if (rdir.ddot(Vec2d(des_dir[1], -des_dir[0])) < 0)
            rdir = -rdir;
        if (abs(mov.ddot(rdir)) < 1e-6) {
            if (abs(cur_dir.ddot(rdir)) < 1e-6)
                return INFINITY;
            else {
                norm = rdir;
            }
        } else {
            r = (sqr(mov[0]) + sqr(mov[1])) / mov.ddot(rdir);
            ori = des_pos + r * rdir;
            norm = cur_pos - ori;
        }
    } else {
        if (abs(cur_dir[0]) < 1e-6) rdir = Vec2d(1, 0);
        else if (abs(cur_dir[1]) < 1e-6) rdir = Vec2d(0, 1);
        else  {
            rdir = Vec2d(1, cur_dir[1] / cur_dir[0]);
            rdir /= sqrt(1 + sqr(rdir[1]));
        }
        if (rdir.ddot(Vec2d(cur_dir[1], -cur_dir[0])) < 0)
            rdir = -rdir;
        if (abs(mov.ddot(rdir)) < 1e-6) {
            norm = rdir;
        } else {
            r = (sqr(mov[0]) + sqr(mov[1])) / mov.ddot(rdir);
            ori = cur_pos + r * rdir;
            norm = des_pos - ori;
        }
    }
    if (norm.ddot(cur_dir) < 0) {
        return -r / 3.0;
    } else {
        return r / 3.0;
    }
}
