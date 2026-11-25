#include "ContentManager.h"
#include <iostream>

// 카드(가구) 데이터 등록 — 가구를 추가하려면 여기에 한 줄 추가 + content/models/ 에 OBJ 추가
// 파일 이름 다음 숫자 = 가장 긴 변(mm) 기본값. settings.txt 에 적은 길이가 먼저다. 나머지 정보는 예시값
void ContentManager::registerDefaults() {
    contents_.clear();
    contents_[0] = { 0, "CHAIR", "chair.obj", 850.f,
        { {"Material", "Wood"}, {"Seats", "1"} },
        cv::Scalar(200, 110, 52), Mesh() };
    contents_[1] = { 1, "TABLE", "table.obj", 1200.f,
        { {"Material", "Wood"}, {"Seats", "4"} },
        cv::Scalar(100, 150, 40), Mesh() };
    contents_[2] = { 2, "SOFA", "sofa.obj", 1900.f,
        { {"Material", "Fabric"}, {"Seats", "3"} },
        cv::Scalar(50, 50, 210), Mesh() };
    contents_[3] = { 3, "BED", "bed.obj", 2000.f,
        { {"Material", "Wood + Fabric"}, {"Type", "Queen"} },
        cv::Scalar(30, 150, 230), Mesh() };
}

bool ContentManager::load(const std::string& contentDir, const Settings& settings) {
    registerDefaults();
    std::string dir = contentDir;
    if (!dir.empty() && dir.back() != '/' && dir.back() != '\\') dir += '/';
    dir += "models/";

    bool allLoaded = true;
    for (auto& [id, c] : contents_) {
        c.longestMm = settingOr(settings, c.modelFile.substr(0, c.modelFile.find('.')), c.longestMm, 10.f, 20000.f);
        c.mesh = Mesh::loadObj(dir + c.modelFile);
        if (c.mesh.triangles.empty()) {
            std::cerr << "[ContentManager] Cannot load model: " << dir + c.modelFile
                      << " -> using a box\n";
            c.mesh = Mesh::makeBox(c.color);
            allLoaded = false;
        } else {
            std::cout << "[ContentManager] " << c.modelFile << ": "
                      << c.mesh.triangles.size() << " triangles\n";
        }
        auto mm = [&](float v) { return std::to_string(cvRound(v * c.longestMm)); };
        const cv::Point3f& s = c.mesh.size;
        c.specs.insert(c.specs.begin(), { "Size WxDxH", mm(s.x) + "x" + mm(s.y) + "x" + mm(s.z) + " mm" });
    }
    std::cout << "[ContentManager] " << contents_.size() << " cards registered\n";
    return allLoaded;
}

const CardContent* ContentManager::find(int id) const {
    auto it = contents_.find(id);
    return it == contents_.end() ? nullptr : &it->second;
}
