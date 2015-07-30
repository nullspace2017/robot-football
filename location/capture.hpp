#ifndef CAPTURE_HPP
#define CAPTURE_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>

class Capture {
public:
    Capture(cv::VideoCapture *cap): cap(cap),
        frame(cvRound(cap->get(cv::CAP_PROP_FRAME_HEIGHT)),
              cvRound(cap->get(cv::CAP_PROP_FRAME_WIDTH)),
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
        return *this;
    }
    static void capture_thread(Capture *cap) {
        while (!cap->pre_close) {
            cap->mu.lock();
            *cap->cap >> cap->frame;
            cap->cv.notify_all();
            cap->mu.unlock();
        }
    }
private:
    cv::VideoCapture *cap;
    cv::Mat frame;
    bool pre_read;
    bool pre_close;
    std::thread th;
    std::mutex mu;
    std::condition_variable cv;
};

#endif // CAPTURE_HPP
