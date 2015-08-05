#include "location.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

using namespace std;
using namespace cv;

Location::Location(Motor *motor): motor(motor),
    ball_state(Location::BALL_NO) { }

Location::~Location() {
    for (size_t i = 0; i < v_capture.size(); i++)
        delete v_capture[i];
    for (size_t i = 0; i < v_vision.size(); i++)
        delete v_vision[i];
}

void Location::add_camera(cv::VideoCapture *capture, Transform *trans, bool disable_locate) {
    v_capture.push_back(new Capture(capture));
    v_vision.push_back(new Vision(cvRound(capture->get(CV_CAP_PROP_FRAME_HEIGHT)),
                                  cvRound(capture->get(CV_CAP_PROP_FRAME_WIDTH)), trans));
    v_vision_conf.push_back(disable_locate ? 1 : 0);
}

void Location::add_server(Server *server) {
    v_server.push_back(server);
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
    for (size_t i = 0; i < v_server.size(); i++) {
        char buf[128];
        snprintf(buf, 128, "%f %f %f %f %d %f %f", position[0], position[1], direction[0], direction[1], ball_state == BALL_NO ? 0 : 1, ball_pos[0], ball_pos[1]);
        v_server[i]->send_broadcast((void *)buf, strnlen(buf, 128));
    }
    return std::make_pair(position, direction);
}

std::pair<Location::BALLSTATE, cv::Vec2d> Location::get_ball() {
    return make_pair(ball_state, ball_pos);
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
        char buf[32] = {'f', 'r', 'a', 'm', 'e', (char)('0' + i), '\0'};
        imshow(buf, frame);
        v_vision[i]->input(frame);
        if ((v_vision_conf[i] & 0x00000001) == 0) { // disable_locate == false
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
    if (ball_state != BALL_NO)
        ball_state = BALL_LAST;
    for (size_t i = 0; i < v_vision.size(); i++) {
        Vision::BALLSTATE ball_state;
        cv::Vec2f ball_pos;
        v_vision[i]->get_ball_pos(ball_pos, ball_state);
        if (ball_state == Vision::BALL_HAS) {
            this->ball_state = BALL_HAS;
            cv::Vec2d n(direction[1], -direction[0]);
            this->ball_pos = position + ball_pos[0] * n + ball_pos[1] * direction;
            break;
        }
    }
}

cv::Mat Location::gen_ground_view() {
    Mat ground_view = ground.gen_ground_view();
    ground.draw_robot(ground_view, position, direction);
    if (ball_state != BALL_NO)
        ground.draw_ball(ground_view, ball_pos);
    return ground_view;
}

template <typename T>
static inline T sqr(T x) {
    return x * x;
}

double Location::get_radius(Vec2d cur_pos, Vec2d cur_dir, Vec2d des_pos, Vec2d des_dir) {
    cur_dir /= sqrt(cur_dir.dot(cur_dir));
    des_dir /= sqrt(des_dir.dot(des_dir));
    double board = 200.0;
    double board_r = 260.0;
    if (cur_pos[0] < board && cur_dir[0] < 0) {
        if (cur_dir[1] > 0) return -board_r;
        else return board_r;
    } else if (cur_pos[0] > ground.width - board && cur_dir[0] > 0) {
        if (cur_dir[1] > 0) return board_r;
        else return -board_r;
    } else if (cur_pos[1] < board && cur_dir[1] < 0) {
        if (cur_dir[0] > 0) return board_r;
        else return -board_r;
    } else if (cur_pos[1] > ground.height - board && cur_dir[1] > 0) {
        if (cur_dir[0] > 0) return -board_r;
        else return board_r;
    }
    Vec2d rdir, ori, norm; // direction of radius, origin of circle;
    Vec2d mov = (cur_pos - des_pos) / 2;
    double r;
    if (cur_dir.ddot(des_dir) > 0) {
        if (abs(des_dir[0]) < 1e-6) rdir = Vec2d(1, 0);
        else if (abs(des_dir[1]) < 1e-6) rdir = Vec2d(0, 1);
        else  {
            rdir = Vec2d(1, -des_dir[0] / des_dir[1]);
            rdir /= sqrt(1 + sqr(rdir[1]));
        }        
        if (rdir.ddot(Vec2d(des_dir[1], -des_dir[0])) < 0) {
            rdir = -rdir;
        }
        if (abs(mov.ddot(rdir)) < 1e-6) {

            if (abs(cur_dir.ddot(rdir)) < 1e-6)
                return INFINITY;
            else {
                norm = rdir;
                r = sqrt(mov.ddot(mov)) * 2;
            }
        } else {            
            r = (sqr(mov[0]) + sqr(mov[1])) / mov.ddot(rdir);
            ori = des_pos + r * rdir;
            norm = cur_pos - ori;
        }        
        if (abs(norm.ddot(cur_dir)) < 1e-6) {
            return -r;
        } else if (norm.ddot(cur_dir) < 0) {
            if (norm.ddot(cur_dir) > -0.4)
            return r * 0.7;
            else return r * 0.4;
        } else {
            if (norm.ddot(cur_dir) < 0.2) return INFINITY;
            else if (norm.ddot(cur_dir) < 0.4) return -r * 1.5;
            else return -r * 0.7;
        }
    } else {
        if (mov.ddot(des_dir) < 0) {
            if (abs(des_dir[0]) < 1e-6) rdir = Vec2d(1, 0);
            else if (abs(des_dir[1]) < 1e-6) rdir = Vec2d(0, 1);
            else  {
                rdir = Vec2d(1, -des_dir[0] / des_dir[1]);
                rdir /= sqrt(1 + sqr(rdir[1]));
            }
            if (rdir.ddot(Vec2d(des_dir[1], -des_dir[0])) < 0) {
                rdir = -rdir;
            }
            if (abs(mov.ddot(rdir)) < 1e-6) {

                if (abs(cur_dir.ddot(rdir)) < 1e-6)
                    return INFINITY;
                else {
                    norm = rdir;
                    r = sqrt(mov.ddot(mov)) * 2;
                }
            } else {
                r = (sqr(mov[0]) + sqr(mov[1])) / mov.ddot(rdir);
                ori = des_pos + r * rdir;
                norm = cur_pos - ori;
            }
            if (norm.ddot(cur_dir) < 0) {
                return 200 - (r <= 0) * 400;
            } else {
                return -200 + (r <= 0) * 400;
            }
        }
        if (abs(cur_dir[0]) < 1e-6) rdir = Vec2d(1, 0);
        else if (abs(cur_dir[1]) < 1e-6) rdir = Vec2d(0, 1);
        else  {
            rdir = Vec2d(1, -cur_dir[0] / cur_dir[1]);
            rdir /= sqrt(1 + sqr(rdir[1]));            
        }
        if (rdir.ddot(Vec2d(cur_dir[1], -cur_dir[0])) < 0)
            rdir = -rdir;
        if (abs(mov.ddot(rdir)) < 1e-6) {
            norm = rdir;
            r = sqrt(mov.ddot(mov)) * 2;
        } else {
            r = (sqr(mov[0]) + sqr(mov[1])) / -mov.ddot(rdir);
            ori = cur_pos + r * rdir;
            norm = des_pos - ori;
        }

        r *= 2.0;
        return -r;
    }
}
