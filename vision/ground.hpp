#ifndef GROUND_HPP
#define GROUND_HPP

#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <initializer_list>

class Ground {
public:
    inline Ground(): lines({cv::Vec4f(   0,    0, 1800,    0),
                           cv::Vec4f( 350,  640, 1450,  640),
                           cv::Vec4f(   0, 2200, 1800, 2200),
                           cv::Vec4f( 350, 3760, 1450, 3760),
                           cv::Vec4f(   0, 4400, 1800, 4400),
                           cv::Vec4f( 350,    0,  350,  640),
                           cv::Vec4f(1450,    0, 1450,  640),
                           cv::Vec4f( 350, 3760,  350, 4400),
                           cv::Vec4f(1450, 3760, 1450, 4400),
                           cv::Vec4f(   0,    0,    0, 4400),
                           cv::Vec4f(1800,    0, 1800, 4400)}),
        width(1800.0), height(4400.0) { }
    cv::Mat gen_ground_view() {
        cv::Mat ground_view(GROUND_HEIGHT / GROUND_MM_PER_PIXEL, GROUND_WIDTH / GROUND_MM_PER_PIXEL, CV_8UC3, cv::Scalar::all(0));
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4f const &l(lines[i]);
            cv::line(ground_view, xy_to_uv(l[0], l[1]), xy_to_uv(l[2], l[3]), cv::Scalar::all(255), 2);
        }
        return ground_view;
    }
    cv::Point xy_to_uv(double xw, double yw) {
        return cv::Point(xw / GROUND_MM_PER_PIXEL, (GROUND_HEIGHT - yw) / GROUND_MM_PER_PIXEL);
    }
    cv::Vec2d uv_to_xy(int u, int v) {
        return cv::Vec2d(u * GROUND_MM_PER_PIXEL, GROUND_HEIGHT - v * GROUND_MM_PER_PIXEL);
    }
    void draw_robot(cv::Mat &ground, cv::Vec2f pos, cv::Vec2f direct, cv::Scalar color = cv::Scalar(0, 255, 0)) {
        direct /= std::sqrt(direct.dot(direct));
        cv::line(ground, xy_to_uv(pos[0], pos[1]), xy_to_uv(pos[0], pos[1]), color, 5);
        cv::line(ground, xy_to_uv(pos[0], pos[1]),
             xy_to_uv(pos[0] + 10000 * direct[0], pos[1] + 10000 * direct[1]), color, 1);
    }
    void draw_ball(cv::Mat &ground, cv::Vec2f ball_pos) {
        cv::line(ground, xy_to_uv(ball_pos[0], ball_pos[1]),
                xy_to_uv(ball_pos[0], ball_pos[1]), cv::Scalar(0, 0, 255), 8);
    }
    std::vector<cv::Vec4f> const lines;
    double const width;
    double const height;
private:
    enum {GROUND_WIDTH = 1800, GROUND_HEIGHT = 4400, GROUND_MM_PER_PIXEL = 8};
};

#endif // GROUND_HPP
