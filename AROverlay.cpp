#include "AROverlay.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>
#include <algorithm>
#include <cmath>
#include <iterator>

namespace {
const float NEAR_Z = 0.1f;   // 이보다 카메라에 가까운 부분은 자른다 (단위: 마커 한 변)

struct Vertex { cv::Vec3f p; cv::Point2f uv; };

// 화면에 투영된 삼각형. 화면 위 어디서든 a*x + b*y + c 로 값을 구한다
struct Face {
    cv::Vec3d edge[3];       // 세 값이 모두 0 이상이면 삼각형 안
    cv::Vec3d invZ, uz, vz;  // 1/거리, 텍스처 좌표/거리
    int x0, y0, x1, y1;      // 차지하는 픽셀 범위
    cv::Vec3f color;         // 밝기를 곱한 색
    float light;
    const cv::Mat* texture;
};

// 카메라 앞쪽만 남기고 자른다. 남은 꼭짓점 수(0, 3, 4)를 돌려준다
int clipNear(const Vertex in[3], Vertex out[4]) {
    int n = 0;
    for (int i = 0; i < 3; ++i) {
        const Vertex& a = in[i];
        const Vertex& b = in[(i + 1) % 3];
        const bool aIn = a.p[2] >= NEAR_Z, bIn = b.p[2] >= NEAR_Z;
        if (aIn) out[n++] = a;
        if (aIn != bIn) {
            const float t = (NEAR_Z - a.p[2]) / (b.p[2] - a.p[2]);
            out[n++] = { a.p + (b.p - a.p) * t, a.uv + (b.uv - a.uv) * t };
        }
    }
    return n;
}

// 화면 밖이거나 넓이가 없으면 false
bool project(const Vertex& a, const Vertex& b, const Vertex& c, float focal, cv::Size size, Face& f) {
    const Vertex* v[3] = { &a, &b, &c };
    double x[3], y[3];
    for (int i = 0; i < 3; ++i) {
        x[i] = (double)focal * v[i]->p[0] / v[i]->p[2] + size.width / 2.0;
        y[i] = (double)focal * v[i]->p[1] / v[i]->p[2] + size.height / 2.0;
    }
    double area = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0]);
    if (!(std::abs(area) > 1e-9)) return false;
    if (area < 0) {
        std::swap(v[1], v[2]);
        std::swap(x[1], x[2]);
        std::swap(y[1], y[2]);
        area = -area;
    }
    // 정수로 바꾸기 전에 화면 안으로 제한해야 아주 큰 값에서도 안전하다
    auto pixel = [](double value, int limit) { return (int)std::floor(std::clamp(value, 0.0, (double)limit)); };
    f.x0 = pixel(std::min({ x[0], x[1], x[2] }), size.width);
    f.y0 = pixel(std::min({ y[0], y[1], y[2] }), size.height);
    f.x1 = pixel(std::max({ x[0], x[1], x[2] }) + 1, size.width);
    f.y1 = pixel(std::max({ y[0], y[1], y[2] }) + 1, size.height);
    if (f.x0 >= f.x1 || f.y0 >= f.y1) return false;

    for (int i = 0; i < 3; ++i) {
        const int j = (i + 1) % 3;
        f.edge[i] = { y[i] - y[j], x[j] - x[i], x[i] * y[j] - x[j] * y[i] };
    }
    auto plane = [&](double f0, double f1, double f2) {
        const double ka = ((f1 - f0) * (y[2] - y[0]) - (f2 - f0) * (y[1] - y[0])) / area;
        const double kb = ((f2 - f0) * (x[1] - x[0]) - (f1 - f0) * (x[2] - x[0])) / area;
        return cv::Vec3d(ka, kb, f0 - ka * x[0] - kb * y[0]);
    };
    const double w[3] = { 1.0 / v[0]->p[2], 1.0 / v[1]->p[2], 1.0 / v[2]->p[2] };
    f.invZ = plane(w[0], w[1], w[2]);
    if (f.texture) {
        f.uz = plane(v[0]->uv.x * w[0], v[1]->uv.x * w[1], v[2]->uv.x * w[2]);
        f.vz = plane(v[0]->uv.y * w[0], v[1]->uv.y * w[1], v[2]->uv.y * w[2]);
    }
    return true;
}

// y번째 줄에서 이 삼각형이 덮는 픽셀 중 지금까지보다 가까운 곳에 번호를 적는다
void rasterRow(const Face& f, int id, int y, float* depth, int* ids) {
    const double yc = y + 0.5;
    double lo = f.x0, hi = f.x1;   // lo <= x < hi
    for (const auto& e : f.edge) {
        const double rest = e[1] * yc + e[2];
        if (e[0] > 0) lo = std::max(lo, std::ceil(-rest / e[0] - 0.5));
        else if (e[0] < 0) hi = std::min(hi, std::floor(-rest / e[0] - 0.5) + 1);
        else if (rest < 0) return;
    }
    if (!(lo < hi)) return;
    const int end = (int)hi;
    const float step = (float)f.invZ[0];
    int x = (int)lo;
    const float first = (float)(f.invZ[0] * (x + 0.5) + f.invZ[1] * yc + f.invZ[2]);
    for (int i = 0; x + i < end; ++i) {
        const float w = first + step * i;
        if (w > depth[x + i]) {
            depth[x + i] = w;
            ids[x + i] = id;
        }
    }
}
} // namespace

AROverlay::AROverlay(float scale, float focalRatio) : scale_(scale), focalRatio_(focalRatio) {}

// 가구가 마커보다 훨씬 커서 마커의 작은 떨림도 크게 보인다 → 이전 자세를 이용해 다듬는다
bool AROverlay::updatePose(const MarkerInfo& marker, const cv::Matx33d& K) {
    static const std::vector<cv::Point3f> square = {   // MarkerInfo 모서리 순서와 동일, 한 변 = 1
        { -.5f, .5f, 0.f }, { .5f, .5f, 0.f }, { .5f, -.5f, 0.f }, { -.5f, -.5f, 0.f } };
    if (marker.corners().size() != 4) return false;
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<double> errors;
    if (cv::solvePnPGeneric(square, marker.corners(), K, cv::noArray(), rvecs, tvecs, false,
                            cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(), errors) < 1) return false;
    // 마커가 카메라와 정확히 나란하면 IPPE가 깨진 답(뒤집힘, NaN)을 준다 → 오차가 크면 반복법으로 다시 푼다
    if (!(errors[0] < 5.0)) {
        rvecs.resize(1);
        tvecs.resize(1);
        if (!cv::solvePnP(square, marker.corners(), K, cv::noArray(), rvecs[0], tvecs[0])) return false;
    }
    std::vector<cv::Matx33d> Rs(rvecs.size());
    for (size_t i = 0; i < rvecs.size(); ++i) cv::Rodrigues(rvecs[i], Rs[i]);

    const auto it = poses_.find(marker.id());
    if (it == poses_.end()) {
        poses_[marker.id()] = { marker.corners(), Rs[0], tvecs[0] };
        return true;
    }
    Pose& prev = it->second;
    // 사각형 하나로는 기울기 해답이 두 개 나온다. 둘 다 그럴듯하면 직전과 비슷한 쪽을 골라 뒤집힘을 막는다
    size_t best = 0;
    if (Rs.size() > 1 && errors[1] < errors[0] * 2 + 0.1 &&
        cv::norm(Rs[1] - prev.R) < cv::norm(Rs[0] - prev.R)) best = 1;

    // 거의 안 움직이면 조금만, 많이 움직이면 바로 따라간다
    float move = 0.f;
    for (int i = 0; i < 4; ++i) move += (float)cv::norm(marker.corners()[i] - prev.corners[i]) / 4.f;
    const double a = std::clamp(move / 4.0, 0.1, 1.0);
    cv::Matx33d U, Vt;
    cv::Matx31d w;
    cv::SVD::compute(prev.R * (1 - a) + Rs[best] * a, w, U, Vt);   // 섞은 뒤 다시 회전 행렬로
    prev = { marker.corners(), U * Vt, prev.t * (1 - a) + tvecs[best] * a };
    return true;
}

void AROverlay::draw(cv::Mat& frame, const std::vector<Item>& items) {
    // 화면에서 사라진 마커의 자세는 잊는다
    for (auto it = poses_.begin(); it != poses_.end();) {
        const int id = it->first;
        const bool seen = std::any_of(items.begin(), items.end(), [id](const Item& i) { return i.marker.id() == id; });
        it = seen ? std::next(it) : poses_.erase(it);
    }
    if (frame.empty() || items.empty()) return;
    const float focal = focalRatio_ * frame.cols;
    const cv::Matx33d K(focal, 0, frame.cols / 2.0, 0, focal, frame.rows / 2.0, 0, 0, 1);

    // 1) 모든 삼각형을 화면에 투영한다 (CPU 코어 수만큼 나눠서)
    const int parts = std::max(1, cv::getNumThreads());
    std::vector<std::vector<Face>> projected;
    const cv::Vec3f lightDir = cv::normalize(cv::Vec3f(0.4f, -0.8f, -0.6f));   // 오른쪽 위, 카메라 쪽

    for (const auto& item : items) {
        if (!item.mesh || !updatePose(item.marker, K)) continue;
        const cv::Matx33d markerR = poses_[item.marker.id()].R;
        const cv::Vec3d tvec = poses_[item.marker.id()].t;
        const float scale = scale_ * item.size;
        const double cy = std::cos(item.rotation.yaw), sy = std::sin(item.rotation.yaw);
        const double cp = std::cos(item.rotation.pitch), sp = std::sin(item.rotation.pitch);
        const cv::Matx33d yaw(cy, -sy, 0, sy, cy, 0, 0, 0, 1), pitch(1, 0, 0, 0, cp, -sp, 0, sp, cp);
        const cv::Matx33f R(markerR * pitch * yaw);

        // 기울일 때 제자리에서 돌도록 모델의 가운데 높이를 회전축으로 쓴다
        const cv::Vec3f pivot(0.f, 0.f, item.mesh->size.z / 2 * scale);
        const cv::Vec3f T = cv::Matx33f(markerR) * pivot + cv::Vec3f(tvec);

        const size_t firstPart = projected.size();
        projected.resize(firstPart + parts);
        const auto& triangles = item.mesh->triangles;
        cv::parallel_for_(cv::Range(0, parts), [&](const cv::Range& range) {
            for (int part = range.start; part < range.end; ++part) {
                std::vector<Face>& faces = projected[firstPart + part];
                const size_t begin = triangles.size() * part / parts, end = triangles.size() * (part + 1) / parts;
                for (size_t i = begin; i < end; ++i) {
                    const Triangle& t = triangles[i];
                    Vertex v[3];
                    for (int k = 0; k < 3; ++k)
                        v[k] = { R * (cv::Vec3f(t.v[k]) * scale - pivot) + T, t.uv[k] };
                    Vertex clipped[4];
                    const int n = clipNear(v, clipped);
                    if (n < 3) continue;

                    const cv::Vec3f normal = (v[1].p - v[0].p).cross(v[2].p - v[0].p);
                    const float len = (float)cv::norm(normal);
                    // 외부 모델은 면의 앞뒤가 뒤섞인 경우가 많아, 항상 카메라를 향한 쪽을 앞면으로 본다
                    float lit = len > 0 ? normal.dot(lightDir) / len : 0.f;
                    if (normal.dot(v[0].p) > 0) lit = -lit;

                    Face f;
                    f.light = 0.6f + 0.4f * std::max(0.f, lit);
                    f.color = t.color * f.light;
                    f.texture = t.texture < 0 ? nullptr : &item.mesh->textures[t.texture];
                    for (int k = 2; k < n; ++k)
                        if (project(clipped[0], clipped[k - 1], clipped[k], focal, frame.size(), f)) faces.push_back(f);
                }
            }
        });
    }

    // 2) 화면을 가로 띠로 나누고, 띠마다 걸치는 삼각형을 모은다
    const int bandRows = 16, bandCount = (frame.rows + bandRows - 1) / bandRows;
    std::vector<const Face*> faces;
    std::vector<std::vector<int>> bands(bandCount);
    for (const auto& part : projected)
        for (const auto& f : part) {
            for (int b = f.y0 / bandRows; b <= (f.y1 - 1) / bandRows; ++b) bands[b].push_back((int)faces.size());
            faces.push_back(&f);
        }

    // 3) 띠마다 픽셀별로 가장 가까운 삼각형을 찾고, 그 색(또는 텍스처)을 칠한다
    depthBuf_.create(frame.size(), CV_32F);
    idBuf_.create(frame.size(), CV_32S);
    cv::parallel_for_(cv::Range(0, bandCount), [&](const cv::Range& range) {
        for (int b = range.start; b < range.end; ++b) {
            if (bands[b].empty()) continue;
            const int top = b * bandRows, bottom = std::min(top + bandRows, frame.rows);
            for (int y = top; y < bottom; ++y) {
                std::fill_n(depthBuf_.ptr<float>(y), frame.cols, 0.f);   // 1/거리: 0 = 아주 멂
                std::fill_n(idBuf_.ptr<int>(y), frame.cols, -1);
            }
            for (int id : bands[b]) {
                const Face& f = *faces[id];
                const int y1 = std::min(f.y1, bottom);
                for (int y = std::max(f.y0, top); y < y1; ++y)
                    rasterRow(f, id, y, depthBuf_.ptr<float>(y), idBuf_.ptr<int>(y));
            }
            for (int y = top; y < bottom; ++y) {
                const int* ids = idBuf_.ptr<int>(y);
                const float* depth = depthBuf_.ptr<float>(y);
                cv::Vec3b* out = frame.ptr<cv::Vec3b>(y);
                for (int x = 0; x < frame.cols; ++x) {
                    if (ids[x] < 0) continue;
                    const Face& f = *faces[ids[x]];
                    cv::Vec3f color = f.color;
                    if (f.texture) {
                        const double z = 1.0 / depth[x], xc = x + 0.5, yc = y + 0.5;
                        double u = (f.uz[0] * xc + f.uz[1] * yc + f.uz[2]) * z;
                        double v = (f.vz[0] * xc + f.vz[1] * yc + f.vz[2]) * z;
                        u -= std::floor(u);   // 1을 넘으면 무늬 반복
                        v -= std::floor(v);
                        const cv::Vec3b& c = f.texture->at<cv::Vec3b>(
                            std::min((int)(v * f.texture->rows), f.texture->rows - 1),
                            std::min((int)(u * f.texture->cols), f.texture->cols - 1));
                        color = cv::Vec3f(c[0] * f.light, c[1] * f.light, c[2] * f.light);
                    }
                    for (int c = 0; c < 3; ++c) out[x][c] = cv::saturate_cast<uchar>(color[c]);
                }
            }
        }
    });
}
