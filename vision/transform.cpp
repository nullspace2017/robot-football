#include "transform.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>

using namespace std;
using namespace cv;

Transform::Transform() {
    camera_pos = Vec3d(90.0, 0.0, 243.0);
    Vec3d center_pos(0, 1000, 0);
    axis_k = center_pos - camera_pos;
    axis_k /= sqrt(axis_k.dot(axis_k));
    Vec3d tmp(1, 0, 0);
    axis_i = tmp.cross(axis_k);
    axis_i /= sqrt(axis_i.dot(axis_i));
    axis_j = axis_k.cross(axis_i);
}

Transform::~Transform() { }

Vec2d Transform::uv_to_xy(int u, int v) {
    Vec2d xy_scale = get_delta_to_center_in_scale(u, v);
    Vec3d direction = axis_k + xy_scale[0] * axis_i + xy_scale[1] * axis_j;
    double lambda = -camera_pos[2] / direction[2];
    return Vec2d(camera_pos[0] + lambda * direction[0], camera_pos[1] + lambda * direction[1]);
}

Vec2d Transform::xy_to_uv(double x, double y) {
    Vec3d direction = Vec3d(x, y, 0) - camera_pos;
    double weight_i = axis_i.dot(direction);
    double weight_j = axis_j.dot(direction);
    double weight_k = axis_k.dot(direction);
    return get_uv_through_scale(weight_i / weight_k, weight_j / weight_k);
}

static double const per_pixel = (400.0 / 268) / 1000.0;

Vec2d Transform::get_delta_to_center_in_scale(int u, int v) {
    int du = u - 320;
    int dv = v - 240;
    return Vec2d(-dv * per_pixel, du * per_pixel);
}

Vec2d Transform::get_uv_through_scale(double weight_i, double weight_j) {
    double du = weight_j / per_pixel;
    double dv = weight_i / per_pixel;
    return Vec2d(320 + du, 240 - dv);
}
