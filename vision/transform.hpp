#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include <opencv2/opencv.hpp>

class Transform {
public:
    Transform(int /*camera_number*/):
        m{85.4018, 37.925, 1125.62, 0.712637, 8.59611, 2234.42, 0.00100575, 0.119256} { }
    ~Transform() { }
    cv::Vec2d uv_to_xy(int u, int v) {
        double tmp1 = (m[4] - m[7] * v) * (m[2] - u);
        double tmp2 = (m[1] - m[7] * u) * (m[5] - v);
        double x = tmp1 - tmp2;
        tmp1 = (m[4] - m[7] * v) * (m[0] - m[6] * u);
        tmp2 = (m[1] - m[7] * u) * (m[3] - m[6] * v);
        x = - x / (tmp1 - tmp2);

        tmp1 = (m[3] - m[6] * v) * (m[2] - u);
        tmp2 = (m[0] - m[6] * u) * (m[5] - v);
        double y = tmp1 - tmp2;
        tmp1 = (m[3] - m[6] * v) * (m[1] - m[7] * u);
        tmp2 = (m[0] - m[6] * u) * (m[4] - m[7] * v);
        y = - y / (tmp1 - tmp2);
        return cv::Vec2d(x, y) * 10;
    }
    cv::Vec2d xy_to_uv(double x, double y) {
        x /= 10;
        y /= 10;
        double u = (m[0]*x + m[1]*y + m[2]) / (1 + m[6]*x + m[7]*y);
        double v = (m[3]*x + m[4]*y + m[5]) / (1 + m[6]*x + m[7]*y);
        return cv::Vec2d(u, v);
    }
private:
    double const m[8];
};

#endif // TRANSFORM_HPP
