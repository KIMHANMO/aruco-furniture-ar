#pragma once
#include "AROverlay.h"
#include "CameraManager.h"
#include "ContentManager.h"
#include "MarkerDetector.h"
#include "Settings.h"
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
    std::string settingsFile = "settings.txt";   // 여기 적힌 값이 아래 기본값보다 먼저다
    float overlayScale = 1.0f;
    float focalRatio = 0.9f;      // 초점거리/화면 너비. 'f' 키로 재면 설정 파일에 저장된다
    float markerMm = 180.f;       // 인쇄한 마커의 검은 테두리 한 변을 자로 잰 길이
    int markerCount = 4;          // 사용할 마커 수 (ID 0~3 = 화면의 1~4번 마커)
    float calibrationMm = 1000.f; // 'f' 키를 누를 때 카메라와 마커 사이의 거리
};

// 전체 흐름 제어: 영상 입력 → 마커 인식 → 고른 가구 찾기 → AR 합성 → 결과 출력
class ARCardService {
public:
    explicit ARCardService(const AppConfig& cfg = AppConfig());
    bool initialize();
    void run();

private:
    void processFrame(cv::Mat& frame);
    bool handleAction(UIAction action, const cv::Mat& frame);
    void handleMouse();
    void chooseFurniture(int number);
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
    std::vector<std::pair<MarkerInfo, const CardContent*>> cards_;   // 현재 화면에 보이는 마커와 그 위의 가구 (nullptr = 아직 안 고름)
    std::map<int, int> chosen_;                                      // 마커 ID -> 고른 가구 번호 (0 = 없음)
    std::map<int, Rotation> rotations_;                              // 마커 ID -> 돌린 각도
    int selectedId_ = -1;                                            // 지금 고르거나 돌릴 마커 ID
};
