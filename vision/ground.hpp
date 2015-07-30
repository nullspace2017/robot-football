#ifndef GROUND_HPP
#define GROUND_HPP

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
    std::vector<cv::Vec4f> const lines;
    double const width;
    double const height;
};

#endif // GROUND_HPP
