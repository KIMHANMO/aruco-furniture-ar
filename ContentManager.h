#pragma once
#include "Mesh.h"
#include "Settings.h"
#include <opencv2/core.hpp>
#include <string>
#include <utility>
#include <vector>

// 마커 위에 띄울 수 있는 가구 1개
struct CardContent {
    std::string name;      // 화면 표시용 이름 (cv::putText는 한글 미지원 → 영문)
    std::string modelFile; // 3D 모델 파일명 (content/models 안의 .obj)
    float longestMm;       // 실제 가구의 가장 긴 변 (mm)
    std::vector<std::pair<std::string, std::string>> specs; // 제원 (항목, 값)
    cv::Scalar color;      // UI 강조색 (BGR)
    Mesh mesh;
};

// 가구 목록 관리. 어느 마커에 무엇을 띄울지는 사용자가 실행 중에 고른다
class ContentManager {
public:
    bool load(const std::string& contentDir, const Settings& settings);   // 3D 모델 로드 (없으면 상자로 대체)
    const std::vector<CardContent>& all() const { return contents_; }

private:
    void registerDefaults();
    std::vector<CardContent> contents_;   // 순서 = 화면의 숫자 키 순서 (첫 번째가 1번 키)
};
