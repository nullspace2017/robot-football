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
        captured(false),
        pre_read(false),
        pre_close(false),
        th(capture_thread, this) { }
    ~Capture() {
        pre_close = true;
        th.join();
    }
    Capture &operator >>(cv::Mat &mat) {
        while (!captured)
            usleep(0);
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
            cap->captured = true;
            cap->mu.unlock();
            usleep(0);
        }
    }
private:
    cv::VideoCapture *cap;
    cv::Mat frame;
    bool captured;
    bool pre_read;
    bool pre_close;
    std::thread th;
    std::mutex mu;
};

#endif // CAPTURE_HPP
