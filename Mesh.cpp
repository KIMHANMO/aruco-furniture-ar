#include "Mesh.h"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cfloat>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

namespace {
struct Material {
    cv::Vec3f color = { 180.f, 180.f, 180.f };
    int texture = -1;
};

std::string restOfLine(std::istringstream& ss) {
    std::string s;
    std::getline(ss >> std::ws, s);
    s.erase(s.find_last_not_of(" \t\r") + 1);
    return s;
}

// 모델 제작자의 컴퓨터 경로가 적혀 있는 경우가 많아, 그 경로에 없으면 OBJ 옆에서 파일 이름만으로 찾는다
std::string baseName(const std::string& p) { return p.substr(p.find_last_of("/\\") + 1); }

void loadMtl(const std::string& dir, const std::string& name,
             std::map<std::string, Material>& materials, std::vector<cv::Mat>& textures) {
    std::ifstream in(dir + name);
    if (!in) in.open(dir + baseName(name));
    if (!in) {
        std::cerr << "[Mesh] Missing material file: " << dir + name << "\n";
        return;
    }
    std::map<std::string, int> loaded;
    Material* cur = nullptr;
    for (std::string line; std::getline(in, line);) {
        std::istringstream ss(line);
        std::string key;
        ss >> key;
        if (key == "newmtl") {
            cur = &materials[restOfLine(ss)];
        } else if (cur && key == "Kd") {
            float r, g, b;
            if (ss >> r >> g >> b) cur->color = { b * 255.f, g * 255.f, r * 255.f };
        } else if (cur && key == "map_Kd") {
            std::string file = restOfLine(ss);
            if (!file.empty() && file[0] == '-')   // "-s 1 1 1 file.png" 같은 옵션은 건너뜀
                file = file.substr(file.find_last_of(" \t") + 1);
            if (file.empty()) continue;
            if (!loaded.count(file)) {
                cv::Mat img = cv::imread(dir + file, cv::IMREAD_COLOR);
                if (img.empty()) img = cv::imread(dir + baseName(file), cv::IMREAD_COLOR);
                if (img.empty()) std::cerr << "[Mesh] Missing texture: " << dir + file << "\n";
                loaded[file] = img.empty() ? -1 : (int)textures.size();
                if (!img.empty()) textures.push_back(img);
            }
            cur->texture = loaded[file];
        }
    }
}

// OBJ 번호(1부터, 음수는 뒤에서부터)를 배열 번호로. 범위를 벗어나면 -1
int toIndex(const std::string& s, size_t count) {
    int i = std::atoi(s.c_str());
    i = i < 0 ? (int)count + i : i - 1;
    return i >= 0 && i < (int)count ? i : -1;
}

// 가운데 정렬 + 바닥을 z=0 으로 + 가장 긴 변을 1로
void normalize(Mesh& mesh) {
    auto& tris = mesh.triangles;
    cv::Point3f lo(FLT_MAX, FLT_MAX, FLT_MAX), hi(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    for (const auto& t : tris)
        for (const auto& p : t.v) {
            lo = { std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z) };
            hi = { std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z) };
        }
    const float size = std::max({ hi.x - lo.x, hi.y - lo.y, hi.z - lo.z });
    if (!(size > 0)) { tris.clear(); return; }
    mesh.size = (hi - lo) * (1.f / size);
    const cv::Point3f origin((lo.x + hi.x) / 2, (lo.y + hi.y) / 2, lo.z);
    for (auto& t : tris)
        for (auto& p : t.v) p = (p - origin) * (1.f / size);
}
} // namespace

Mesh Mesh::loadObj(const std::string& path) {
    Mesh mesh;
    std::ifstream in(path);
    if (!in) return mesh;
    const std::string dir = path.substr(0, path.find_last_of("/\\") + 1);

    std::vector<cv::Point3f> points;
    std::vector<cv::Point2f> uvs;
    std::map<std::string, Material> materials;
    Material material;

    for (std::string line; std::getline(in, line);) {
        std::istringstream ss(line);
        std::string key;
        ss >> key;
        if (key == "v") {
            float x = 0, y = 0, z = 0;
            ss >> x >> y >> z;
            points.emplace_back(x, -z, y);   // OBJ는 보통 y가 위 → 마커는 z가 위
        } else if (key == "vt") {
            float u = 0, v = 0;
            ss >> u >> v;
            uvs.emplace_back(u, 1.f - v);    // 그림 파일은 맨 위가 0
        } else if (key == "mtllib") {
            loadMtl(dir, restOfLine(ss), materials, mesh.textures);
        } else if (key == "usemtl") {
            material = materials[restOfLine(ss)];
        } else if (key == "f") {
            // "v", "v/vt", "v//vn", "v/vt/vn" 형식
            std::vector<std::pair<int, int>> face;
            for (std::string tok; ss >> tok;) {
                const size_t slash = tok.find('/');
                const int vt = slash == std::string::npos ? -1 : toIndex(tok.substr(slash + 1), uvs.size());
                face.emplace_back(toIndex(tok, points.size()), vt);
            }
            // shortcut: 다각형을 부채꼴로 나눠 오목한 면은 어긋남, 그런 모델은 Blender에서 Triangulate 후 내보내기
            for (size_t i = 2; i < face.size(); ++i) {
                const std::pair<int, int> corner[3] = { face[0], face[i - 1], face[i] };
                Triangle t;
                t.color = material.color;
                t.texture = material.texture;
                bool ok = true;
                for (int k = 0; k < 3; ++k) {
                    if (corner[k].first < 0) { ok = false; break; }
                    t.v[k] = points[corner[k].first];
                    if (corner[k].second < 0) t.texture = -1;
                    else t.uv[k] = uvs[corner[k].second];
                }
                if (ok) mesh.triangles.push_back(t);
            }
        }
    }
    normalize(mesh);
    return mesh;
}

Mesh Mesh::makeBox(const cv::Scalar& color) {
    static const int quads[6][4] = { {0,1,3,2}, {4,5,7,6}, {0,1,5,4}, {2,3,7,6}, {0,2,6,4}, {1,3,7,5} };
    auto corner = [](int i) {
        return cv::Point3f((i & 1) ? .5f : -.5f, (i & 2) ? .35f : -.35f, (i & 4) ? .6f : 0.f);
    };
    Mesh mesh;
    for (const auto& q : quads)
        for (int i = 2; i < 4; ++i) {
            Triangle t;
            t.color = cv::Vec3f((float)color[0], (float)color[1], (float)color[2]);
            t.v[0] = corner(q[0]);
            t.v[1] = corner(q[i - 1]);
            t.v[2] = corner(q[i]);
            mesh.triangles.push_back(t);
        }
    normalize(mesh);
    return mesh;
}
