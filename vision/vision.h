#ifndef VISION_VISION_H
#define VISION_VISION_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "transform.hpp"
#include "ground.hpp"

class Vision {
public:
    Vision(int height, int width, Transform *trans);
    ~Vision();
    void input(cv::Mat const &in);
    void get_location(cv::Vec2f &pos, cv::Vec2f &direct, double &location_confidence);
    enum BALLSTATE { BALL_NO, BALL_HAS };
    void get_ball_pos(cv::Vec2f &pos, BALLSTATE &ball_state);
    cv::Mat gen_as_pic();
    cv::Mat gen_platform();
private:
    enum VCOLOR { VCOLOR_WHITE = 0, VCOLOR_GREEN, VCOLOR_BACKGROUND, VCOLOR_EDGE,
                    VCOLOR_EDGE_POSSIBLE, VCOLOR_OUT_OF_RANGE, VCOLOR_BALL, VCOLOR_BALL_POSSIBLE, VCOLOR_COUNT };
    enum { VPLAT_HEIGHT = 500, VPLAT_WIDTH = 480, VPLAT_MM_PER_PIXEL = 5 };
    static cv::Vec3b const const_vcolors[VCOLOR_COUNT];
    cv::Mat pic;
    int height, width;
    uchar **v_pic;
    uchar *v_pic_pool;
    uchar **v_plat;
    uchar *v_plat_pool;
    std::vector<cv::Vec2f> white_lines;
    Ground ground;
    cv::Vec2f robot_pos;
    cv::Vec2f robot_direct;
    double robot_location_confidence;
    BALLSTATE ball_state;
    int ballx, bally, ballr;
    Transform *trans;
private:
    void pre_copy();
    void get_edge_white();
    void update_plat();
    void get_white_lines();
    void match_robot_pos();
    void get_ball_color();
    void get_ball_hough();
private: // helper functions
    bool in_rect(int x, int y) { return x >= 0 && x < height && y >= 0 && y < width; }
    bool cut_to_rect(int &x, int &y) {
        bool flag = false;
        if (x < 0) { x = 0; flag = true; }
        else if (x >= height) { x = height - 1; flag = true; }
        if (y < 0) { y = 0; flag = true; }
        else if (y >= width) { y = width - 1; flag = true; }
        return flag;
    }
    void expand_to_ball(int x, int y); //huanglj
};

#endif
