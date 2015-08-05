#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <opencv2/opencv.hpp>

class Transform {
public:
    Transform(int _camera_no);
    ~Transform();
    cv::Vec2d uv_to_xy(int u, int v);
    cv::Vec2d xy_to_uv(double x, double y);
private:
    const double left_camera[8], right_camera[8];
    int camera_no;
    static const int LEFT, RIGHT;
};

#endif // TRANSFORM_H
