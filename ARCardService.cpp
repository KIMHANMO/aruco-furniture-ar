#include "ARCardService.h"
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

ARCardService::ARCardService(const AppConfig& cfg)
    : cfg_(cfg), overlay_(cfg.overlayScale, cfg.focalRatio) {}

bool ARCardService::initialize() {
    const Settings settings = loadSettings(cfg_.settingsFile);
    cfg_.markerMm = settingOr(settings, "markerMm", cfg_.markerMm, 10.f, 2000.f);
    overlay_.setFocalRatio(settingOr(settings, "focalRatio", cfg_.focalRatio, 0.4f, 2.5f));
    contents_.load(cfg_.contentDir, settings);   // 모델이 없어도 상자로 계속 진행
    if (!camera_.open(cfg_.cameraIndex, cfg_.width, cfg_.height)) return false;
    return true;
}

void ARCardService::processFrame(cv::Mat& frame) {
    // 1) 마커 인식
    std::vector<MarkerInfo> markers = detector_.detect(frame);

    // 2) 마커마다 사용자가 고른 가구 찾기 (사용하지 않는 ID는 무시)
    cards_.clear();
    for (const auto& m : markers) {
        if (m.id() >= cfg_.markerCount) continue;
        // 처음 보는 마커는 바로 선택해서 가구를 고를 수 있게
        if (chosen_.try_emplace(m.id(), 0).second) selectedId_ = m.id();
        const int number = chosen_[m.id()];
        cards_.emplace_back(m, number > 0 ? &contents_.all()[number - 1] : nullptr);
    }

    // ID 순으로 정렬해서 패널 순서가 흔들리지 않게
    std::sort(cards_.begin(), cards_.end(),
              [](const auto& a, const auto& b) { return a.first.id() < b.first.id(); });

    // 선택한 마커가 화면에서 사라졌으면 보이는 첫 번째 마커를 선택
    const bool selectedVisible = std::any_of(cards_.begin(), cards_.end(),
        [this](const auto& card) { return card.first.id() == selectedId_; });
    if (!selectedVisible && !cards_.empty()) selectedId_ = cards_.front().first.id();

    // 3) AR 합성
    std::vector<AROverlay::Item> items;
    for (const auto& [m, c] : cards_)
        if (c) items.push_back({ m, &c->mesh, rotations_[m.id()], c->longestMm / cfg_.markerMm });
    overlay_.draw(frame, items);

    if (ui_.showDebug) detector_.drawDebug(frame);

    // 4) 정보 표시
    ui_.drawOverlayInfo(frame, cards_, contents_.all(), selectedId_, overlay_.scale(), fps_);
}

// 선택한 마커 위에 number번 가구를 띄운다 (0 = 치우기)
void ARCardService::chooseFurniture(int number) {
    if (cards_.empty() || number > (int)contents_.all().size()) return;
    chosen_[selectedId_] = number;
    rotations_.erase(selectedId_);   // 새 가구는 처음 방향으로
}

void ARCardService::rotateSelected(float yaw, float pitch) {
    if (cards_.empty()) return;
    Rotation& r = rotations_[selectedId_];
    r.yaw += yaw;
    r.pitch += pitch;
}

// 누른 곳에서 가장 가까운 마커를 선택하고, 끌면 그 위의 가구를 돌린다
void ARCardService::handleMouse() {
    const MouseDrag drag = ui_.takeDrag();
    if (drag.pressed && !cards_.empty()) {
        auto dist = [&](const auto& card) {
            const cv::Point2f d = card.first.center() - cv::Point2f(drag.start);
            return d.dot(d);
        };
        selectedId_ = std::min_element(cards_.begin(), cards_.end(),
            [&](const auto& a, const auto& b) { return dist(a) < dist(b); })->first.id();
    }
    const float radPerPixel = 0.01f;
    rotateSelected(drag.delta.x * radPerPixel, drag.delta.y * radPerPixel);
}

// 마커 1장을 카메라 정면, 정해진 거리(1m)에 두고 부르면 카메라의 초점거리를 알아낸다
void ARCardService::calibrateFocal(const cv::Mat& frame) {
    char msg[96];
    const float ratio = cards_.size() == 1
        ? cards_[0].first.sideLength() * cfg_.calibrationMm / (cfg_.markerMm * frame.cols) : 0.f;
    if (ratio < 0.4f || ratio > 2.5f) {   // 흔한 카메라 범위를 벗어나면 거리나 마커가 잘못된 것
        std::snprintf(msg, sizeof(msg), "Show ONE marker, facing camera, at %.0f mm", cfg_.calibrationMm);
    } else {
        overlay_.setFocalRatio(ratio);
        const bool saved = saveSetting(cfg_.settingsFile, "focalRatio", ratio);
        std::snprintf(msg, sizeof(msg), "focalRatio = %.3f (%s)", ratio, saved ? "saved" : "NOT saved");
        std::cout << "[ARCardService] " << msg << "\n";
    }
    ui_.showMessage(msg);
}

void ARCardService::saveCapture(const cv::Mat& frame) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(cfg_.captureDir, ec);

    std::time_t t = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream name;
    name << cfg_.captureDir << "/capture_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".png";

    if (cv::imwrite(name.str(), frame)) {
        std::cout << "[ARCardService] Saved " << name.str() << "\n";
        ui_.showMessage("Saved: " + name.str());
    } else {
        ui_.showMessage("Capture failed");
    }
}

bool ARCardService::handleAction(UIAction action, const cv::Mat& frame) {
    if (action >= UIAction::Furniture0 && action <= UIAction::Furniture9) {
        chooseFurniture((int)action - (int)UIAction::Furniture0);
        return true;
    }
    const float step = 0.1f;   // 방향키 한 번에 약 6도
    switch (action) {
    case UIAction::Quit:        return false;
    case UIAction::Capture:     saveCapture(frame); break;
    case UIAction::ToggleInfo:  ui_.showInfo = !ui_.showInfo; break;
    case UIAction::ToggleSpecs: ui_.showSpecs = !ui_.showSpecs; break;
    case UIAction::ToggleDebug: ui_.showDebug = !ui_.showDebug; break;
    case UIAction::ScaleUp:     overlay_.setScale(std::min(3.0f, overlay_.scale() + 0.1f)); break;
    case UIAction::ScaleDown:   overlay_.setScale(std::max(0.5f, overlay_.scale() - 0.1f)); break;
    case UIAction::RotateLeft:  rotateSelected(-step, 0.f); break;
    case UIAction::RotateRight: rotateSelected(step, 0.f); break;
    case UIAction::RotateUp:    rotateSelected(0.f, -step); break;
    case UIAction::RotateDown:  rotateSelected(0.f, step); break;
    case UIAction::CalibrateFocal: calibrateFocal(frame); break;
    case UIAction::ResetRotation: rotations_.clear(); break;
    default: break;
    }
    return true;
}

void ARCardService::run() {
    cv::Mat frame;
    int failCount = 0;
    auto prev = std::chrono::steady_clock::now();

    while (true) {
        // 0) 영상 입력 (일시적 실패는 몇 번 재시도)
        if (!camera_.read(frame)) {
            if (++failCount > 30) {
                std::cerr << "[ARCardService] Camera read failed, exiting\n";
                break;
            }
            if (ui_.pollKey(10) == UIAction::Quit) break;
            continue;
        }
        failCount = 0;

        processFrame(frame);

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - prev).count();
        prev = now;
        if (dt > 0) fps_ = fps_ * 0.9 + (1.0 / dt) * 0.1;   // 지수 이동평균

        ui_.show(frame);
        if (!handleAction(ui_.pollKey(1), frame)) break;
        handleMouse();
    }
    camera_.release();
}
