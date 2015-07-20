#ifndef VISION_VISION_H
#define VISION_VISION_H

#include <opencv2/opencv.hpp>
#include "transform.h"

class Vision {
public:
    Vision(int height, int width, Transform *trans);
    ~Vision();
    void input(cv::Mat const &in);
    cv::Mat gen_as_pic();
    cv::Mat gen_platform();
private:
    enum VCOLOR { VCOLOR_WHITE = 0, VCOLOR_GREEN, VCOLOR_BACKGROUND, VCOLOR_EDGE, VCOLOR_EDGE_POSSIBLE, VCOLOR_OUT_OF_RANGE, VCOLOR_COUNT };
    enum { VPLAT_HEIGHT = 800, VPLAT_WIDTH = 800, VPLAT_MM_PER_PIXEL = 5 };
    static cv::Vec3b const const_vcolors[VCOLOR_COUNT];
    cv::Mat pic;
    int height, width;
    uchar **v_pic;
    uchar *v_pic_pool;
    uchar **v_plat;
    uchar *v_plat_pool;
    Transform *trans;
private:
    void pre_copy();
    void get_edge_white();
    void update_plat();
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
};

#endif
