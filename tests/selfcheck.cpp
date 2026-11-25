// 카메라 없이 모델 읽기/합성을 확인하는 자가 점검 (Visual Studio 프로젝트에는 포함되지 않음)
// 인자: 임시 폴더 — 점검용 모델과 결과 그림을 여기에 쓴다
#include "../AROverlay.h"
#include "../MarkerDetector.h"
#include "../Settings.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry.hpp>
#endif
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>

#define CHECK(x) do { if (!(x)) { std::cerr << "FAIL line " << __LINE__ << ": " #x "\n"; return 1; } } while (0)

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: selfcheck <temp dir>\n"; return 2; }
    const std::string dir = std::string(argv[1]) + "/";

    // 1) OBJ 읽기: 사각형 면, 음수 번호, 범위를 벗어난 면, 재질 색, 텍스처(다른 컴퓨터 경로 + 공백 이름)
    cv::Mat tex(64, 64, CV_8UC3, cv::Scalar(0, 0, 255));
    tex.colRange(32, 64).setTo(cv::Scalar(255, 0, 0));          // 왼쪽 빨강, 오른쪽 파랑
    cv::imwrite(dir + "my tex.png", tex);
    std::ofstream(dir + "t.mtl") << "newmtl green\nKd 0 1 0\nnewmtl wood\nmap_Kd C:\\other\\my tex.png\n";
    std::ofstream(dir + "t.obj")
        << "mtllib t.mtl\nv -2 0 2\nv 2 0 2\nv 2 0 -2\nv -2 0 -2\n"
        << "vt 0 0\nvt 3 0\nvt 3 1\nvt 0 1\n"
        << "usemtl wood\nf 1/1 2/2 3/3 4/4\n"                     // 바닥 (무늬 3번 반복)
        << "v 0 4 0\nusemtl green\nf -5//1 -4//1 -1//1\nf 1 2 99\n";
    Mesh m = Mesh::loadObj(dir + "t.obj");
    CHECK(m.triangles.size() == 3);                             // 사각형 2 + 삼각형 1, 잘못된 면은 버림
    CHECK(m.textures.size() == 1 && m.triangles[0].texture == 0);
    CHECK(m.triangles[2].texture == -1 && m.triangles[2].color == cv::Vec3f(0, 255, 0));
    CHECK(m.triangles[2].v[2] == cv::Point3f(0, 0, 1));         // y가 위 → z가 위, 가장 긴 변 = 1
    CHECK(Mesh::loadObj(dir + "none.obj").triangles.empty());

    // 설정 파일: BOM, 설명, 잘못된 값, 저장할 때 다른 줄 보존
    const std::string cfg = dir + "settings.txt";
    std::ofstream(cfg) << "\xEF\xBB\xBF" << "chair = 900 # mine\n# note = 5\n sofa=abc\nbed = -3\nfocalRatio = 0.9 # cam\n";
    Settings s = loadSettings(cfg);
    CHECK(settingOr(s, "chair", 1, 10, 20000) == 900 && !s.count("note") && !s.count("# note"));
    CHECK(settingOr(s, "sofa", 7, 10, 20000) == 7 && settingOr(s, "bed", 8, 10, 20000) == 8);
    CHECK(settingOr(s, "table", 9, 10, 20000) == 9 && loadSettings(dir + "none.txt").empty());
    CHECK(saveSetting(cfg, "focalRatio", 0.75f) && saveSetting(cfg, "markerMm", 175.f));
    s = loadSettings(cfg);
    CHECK(s["focalRatio"] == 0.75f && s["markerMm"] == 175.f && s["chair"] == 900);
    std::ifstream saved(cfg);
    const std::string text((std::istreambuf_iterator<char>(saved)), {});
    CHECK(text.find("# mine") != std::string::npos && text.find("# note = 5") != std::string::npos && text.find("# cam") != std::string::npos);

    // 2) 기울어진 가짜 카메라 화면에서 마커 검출 → 합성
    cv::Mat marker, frame(720, 1280, CV_8UC3, cv::Scalar(200, 200, 200));
    cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), 0, 200, marker, 1);
    cv::cvtColor(marker, marker, cv::COLOR_GRAY2BGR);
    const std::vector<cv::Point2f> from = { {0,0}, {200,0}, {200,200}, {0,200} },
                                   to = { {540,400}, {740,400}, {790,560}, {490,560} };
    cv::warpPerspective(marker, frame, cv::getPerspectiveTransform(from, to), frame.size(),
                        cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
    const auto markers = MarkerDetector().detect(frame);
    CHECK(markers.size() == 1 && markers[0].id() == 0);

    auto render = [&](const Mesh& mesh, Rotation r) {
        cv::Mat out = frame.clone();
        AROverlay().draw(out, { { markers[0], &mesh, r } });
        return out;
    };
    auto changed = [&](const cv::Mat& a, const cv::Mat& b) {
        cv::Mat d;
        cv::absdiff(a, b, d);
        cv::cvtColor(d, d, cv::COLOR_BGR2GRAY);
        return cv::boundingRect(d > 8);
    };
    const Mesh box = Mesh::makeBox({ 200, 110, 52 });
    const cv::Mat plain = render(box, {}), yawed = render(box, { 0.8f, 0.f }), tilted = render(box, { 0.f, 0.8f });
    const cv::Rect area = changed(frame, plain);
    CHECK(area.x > 450 && area.br().x < 830);                   // 마커 너비 안
    CHECK(area.y < 400 && area.br().y > 480);                   // 마커에서 위로 솟음
    CHECK(changed(plain, yawed).area() > 0 && changed(plain, tilted).area() > 0);

    // 반복 텍스처의 빨강/파랑이 모두 보여야 함
    const cv::Mat textured = render(m, { 3.14f, 0.f });
    int red = 0, blue = 0;
    for (const auto& p : cv::Mat_<cv::Vec3b>(textured)) {
        red += p[2] > 90 && p[0] < 20 && p[1] < 20;
        blue += p[0] > 90 && p[2] < 20 && p[1] < 20;
    }
    CHECK(red > 1000 && blue > 1000);

    // 3) 실제 크기: 1000mm 상자가 180mm 마커의 1000/180 배로, 정면 3m 거리에서 정확한 픽셀 수로 보여야 함
    const float f = 0.9f * 1280, px = f * 180 / 3000 / 2;       // 마커 반 변 (px)
    const MarkerInfo far3m(0, { {640 - px, 360 - px}, {640 + px, 360 - px}, {640 + px, 360 + px}, {640 - px, 360 + px} });
    cv::Mat canvas(720, 1280, CV_8UC3, cv::Scalar::all(200)), real = canvas.clone();
    AROverlay().draw(real, { { far3m, &box, {}, 1000.f / 180.f } });
    // 윗면(높이 600mm)이 카메라에 가장 가까우므로 가로 1000mm 는 거리 2400mm 에서 보임
    CHECK(std::abs(changed(canvas, real).width - f * 1000 / 2400) < 6);

    // 4) 모서리가 0.3px 흔들려도 가구 높이는 거의 흔들리지 않아야 하고, 카메라 뒤로 넘어가도 멈추지 않아야 함
    AROverlay smooth;
    cv::RNG rng(1);
    int lo = 9999, hi = 0;
    for (int i = 0; i < 60; ++i) {
        std::vector<cv::Point2f> c = { {600,400}, {660,400}, {668,440}, {592,440} };
        for (auto& p : c) p += cv::Point2f((float)rng.gaussian(0.3), (float)rng.gaussian(0.3));
        cv::Mat out = canvas.clone();
        smooth.draw(out, { { MarkerInfo(0, c), &box, {}, 5.f } });
        const int h = changed(canvas, out).height;
        if (i >= 20) { lo = std::min(lo, h); hi = std::max(hi, h); }
    }
    std::cout << "height jitter: " << lo << ".." << hi << " px\n";
    CHECK(hi - lo < 25);
    cv::Mat inside = frame.clone();
    AROverlay().draw(inside, { { markers[0], &box, {}, 12.f } });
    cv::imwrite(dir + "out_inside.png", inside);

    cv::imwrite(dir + "out_plain.png", plain);
    cv::imwrite(dir + "out_yaw.png", yawed);
    cv::imwrite(dir + "out_tilt.png", tilted);
    cv::imwrite(dir + "out_texture.png", textured);
    std::cout << "OK\n";
    return 0;
}
