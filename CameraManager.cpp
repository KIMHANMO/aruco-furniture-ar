#include "CameraManager.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <iostream>

CameraManager::~CameraManager() { release(); }

bool CameraManager::open(int index, int width, int height) {
#ifdef _WIN32
    // Windows에서는 DirectShow 백엔드가 카메라 열기/해상도 설정이 빠르고 안정적
    cap_.open(index, cv::CAP_DSHOW);
    if (!cap_.isOpened()) cap_.open(index);
#else
    cap_.open(index);
#endif
    if (!cap_.isOpened()) {
        std::cerr << "[CameraManager] Cannot open camera " << index << "\n";
        return false;
    }
    maxSide_ = std::max(width, height);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    std::cout << "[CameraManager] Camera " << index << " opened: "
              << frameSize().width << "x" << frameSize().height << "\n";
    return true;
}

bool CameraManager::read(cv::Mat& frame) {
    if (!cap_.isOpened()) return false;
    if (!cap_.read(frame) || frame.empty()) return false;
    // 핸드폰 카메라는 요청을 무시하고 1080p/4K를 보내기도 해서, 느려지지 않게 줄인다
    const int side = std::max(frame.cols, frame.rows);
    if (side > maxSide_) {
        const double k = (double)maxSide_ / side;
        cv::resize(frame, frame, cv::Size(), k, k, cv::INTER_AREA);
    }
    return true;
}

void CameraManager::release() {
    if (cap_.isOpened()) cap_.release();
}

cv::Size CameraManager::frameSize() const {
    return { static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
             static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)) };
}
