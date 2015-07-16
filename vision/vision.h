#ifndef VISION_VISION_H
#define VISION_VISION_H

#include <opencv2/opencv.hpp>

class Vision {
public:
    Vision(int height, int width);
    ~Vision();
    void input(cv::Mat const &in);
    void get_edge_white();
    cv::Mat gen_as_pic();
private:
    enum VCOLOR { VCOLOR_WHITE = 0, VCOLOR_GREEN, VCOLOR_BACKGROUND, VCOLOR_EDGE, VCOLOR_EDGE_POSSIBLE };
    int height, width;
    uchar **v;
    uchar *v_pool;
private:
    void expand_to_white(int x, int y);
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
