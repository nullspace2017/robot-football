#include "transform.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>

using namespace std;
using namespace cv;

Transform::Transform(int): m{88.4118, 33.2783, 1054.05, 3.50322, 9.87371, 2300.52, 0.00842589, 0.121405} { }

Transform::~Transform() { }

Vec2d Transform::uv_to_xy(int u, int v) {
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
    return Vec2d(x, y);
}

Vec2d Transform::xy_to_uv(double x, double y) {
    double u = (m[0]*x + m[1]*y + m[2]) / (1 + m[6]*x + m[7]*y);
    double v = (m[3]*x + m[4]*y + m[5]) / (1 + m[6]*x + m[7]*y);
    return Vec2d(u, v);
}
