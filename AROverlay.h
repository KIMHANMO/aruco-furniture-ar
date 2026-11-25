#pragma once
#include "MarkerInfo.h"
#include "Mesh.h"
#include <opencv2/core.hpp>
#include <map>
#include <vector>

// 사용자가 돌린 각도 (라디안)
struct Rotation {
    float yaw = 0.f;    // 좌우: 팽이처럼 돌리기
    float pitch = 0.f;  // 상하: 앞뒤로 기울이기
};

// 마커 위치/기울기를 계산해 3D 모델을 그 위에 합성
class AROverlay {
public:
    struct Item {
        MarkerInfo marker;
        const Mesh* mesh;
        Rotation rotation;
        float size = 1.f;   // 모델의 가장 긴 변 / 마커 한 변 (실제 길이의 비)
    };

    // scale: 사용자 확대 배율 (1.0 = 실제 크기)
    // focalRatio: 카메라 초점거리 / 화면 너비
    explicit AROverlay(float scale = 1.0f, float focalRatio = 0.9f);

    // 한 번에 모두 그려야 가까운 가구가 먼 가구를 올바르게 가린다
    void draw(cv::Mat& frame, const std::vector<Item>& items);
    void setScale(float s) { scale_ = s; }
    float scale() const { return scale_; }
    void setFocalRatio(float f) { focalRatio_ = f; }

private:
    struct Pose {
        std::vector<cv::Point2f> corners;
        cv::Matx33d R;
        cv::Vec3d t;
    };
    bool updatePose(const MarkerInfo& marker, const cv::Matx33d& K);
    std::map<int, Pose> poses_;     // 마커 ID -> 부드럽게 다듬은 자세
    float scale_;
    float focalRatio_;
    cv::Mat depthBuf_, idBuf_;   // 프레임마다 다시 할당하지 않도록 보관
};
