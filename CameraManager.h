#pragma once
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

// 웹캠 연결 및 프레임 입력
class CameraManager {
public:
    ~CameraManager();
    bool open(int index = 0, int width = 1280, int height = 720);
    bool read(cv::Mat& frame);
    bool isOpened() const { return cap_.isOpened(); }
    void release();
    cv::Size frameSize() const;

private:
    cv::VideoCapture cap_;
    int maxSide_ = 1280;   // 이보다 큰 화면은 줄여서 처리
};
