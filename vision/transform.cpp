#include "transform.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>

using namespace std;
using namespace cv;

Transform::Transform() {
    camera_pos = Vec3d(-90.0, -0.0, 240.0);
    Vec3d center_pos(0, 1000, 0);
    axis_k = center_pos - camera_pos;
    axis_k /= sqrt(axis_k.dot(axis_k));
    Vec3d tmp(1, 0, 0);
    axis_i = tmp.cross(axis_k);
    axis_j = axis_k.cross(axis_i);
}

Transform::~Transform() { }

Vec2d Transform::uv_to_xy(int u, int v) {
    Vec2d xy_scale = get_delta_to_center_in_scale(u, v);
    Vec3d direction = axis_k + xy_scale[0] * axis_i + xy_scale[1] * axis_j;
    double lambda = -camera_pos[2] / direction[2];
    return Vec2d(camera_pos[0] + lambda * direction[0], camera_pos[1] + lambda * direction[1]);
}

Vec2d Transform::get_delta_to_center_in_scale(int u, int v) {
    int du = u - 240;
    int dv = v - 320;
    double const per_pixel = 1.527 / 1000.0;
    return Vec2d(-du * per_pixel, dv * per_pixel);
}
