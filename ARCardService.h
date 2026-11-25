#pragma once
#include "AROverlay.h"
#include "CameraManager.h"
#include "ContentManager.h"
#include "MarkerDetector.h"
#include "UIManager.h"
#include <map>
#include <string>
#include <utility>
#include <vector>

struct AppConfig {
    int cameraIndex = 0;
    int width = 1280;
    int height = 720;
    std::string contentDir = "content";
    std::string captureDir = "captures";
    float overlayScale = 1.0f;
    float focalRatio = 0.9f;      // 초점거리/화면 너비. 'f' 키로 잰 값을 여기에 적어 둔다
    float markerMm = 180.f;       // 인쇄한 마커의 검은 테두리 한 변을 자로 잰 길이
    float calibrationMm = 1000.f; // 'f' 키를 누를 때 카메라와 마커 사이의 거리
};

// 전체 흐름 제어: 영상 입력 → 마커 인식 → 콘텐츠 검색 → AR 합성 → 결과 출력
class ARCardService {
public:
    explicit ARCardService(const AppConfig& cfg = AppConfig());
    bool initialize();
    void run();

private:
    void processFrame(cv::Mat& frame);
    bool handleAction(UIAction action, const cv::Mat& frame);
    void handleMouse();
    void rotateSelected(float yaw, float pitch);
    void calibrateFocal(const cv::Mat& frame);
    void saveCapture(const cv::Mat& frame);

    AppConfig cfg_;
    CameraManager camera_;
    MarkerDetector detector_;
    ContentManager contents_;
    AROverlay overlay_;
    UIManager ui_;
    double fps_ = 0.0;
    std::vector<std::pair<MarkerInfo, const CardContent*>> cards_;   // 현재 화면에 보이는 카드
    std::map<int, Rotation> rotations_;                              // 마커 ID -> 돌린 각도
    int selectedId_ = -1;                                            // 지금 돌릴 가구의 마커 ID
};
