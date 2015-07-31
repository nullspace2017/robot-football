#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <opencv2/opencv.hpp>

class Transform {
public:
    Transform(int camera_number);
    ~Transform();
    cv::Vec2d uv_to_xy(int u, int v);
    cv::Vec2d xy_to_uv(double x, double y);
private:
    const double m[8];
};

#endif // TRANSFORM_H
