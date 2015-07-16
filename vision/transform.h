#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <opencv2/opencv.hpp>

class Transform
{
public:
    Transform();
    ~Transform();
    cv::Vec2d uv_to_xy(int u, int v);
private:
    cv::Vec3d camera_pos;
    cv::Vec3d axis_i, axis_j, axis_k;
private:
    cv::Vec2d get_delta_to_center_in_scale(int u, int v);
};

#endif // TRANSFORM_H
