#ifndef CAPTURE_HPP
#define CAPTURE_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <opencv2/opencv.hpp>

class Capture {
public:
    Capture(cv::VideoCapture *cap): cap(cap),
        frame(cvRound(cap->get(CV_CAP_PROP_FRAME_HEIGHT)),
              cvRound(cap->get(CV_CAP_PROP_FRAME_WIDTH)),
              CV_8UC3, cv::Scalar::all(0)),
        pre_read(false),
        pre_close(false),
        th(capture_thread, this) { }
    ~Capture() {
        pre_close = true;
        th.join();
    }
    Capture &operator >>(cv::Mat &mat) {
        mu.lock();
        mat = frame.clone();
        mu.unlock();
        usleep(0);
        return *this;
    }
    static void capture_thread(Capture *cap) {
        while (!cap->pre_close) {
            cap->mu.lock();
            *cap->cap >> cap->frame;
            cap->mu.unlock();
            usleep(0);
        }
    }
private:
    cv::VideoCapture *cap;
    cv::Mat frame;
    bool pre_read;
    bool pre_close;
    std::thread th;
    std::mutex mu;
};

#endif // CAPTURE_HPP
