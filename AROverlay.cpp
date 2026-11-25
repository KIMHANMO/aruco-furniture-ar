#include "AROverlay.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ptcloud.hpp>
#include <algorithm>
#include <cmath>
#include <iterator>

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

    // 1) 모든 모델의 꼭짓점을 카메라 기준 좌표로 옮긴다
    struct Paint { cv::Vec3f color; float light; const cv::Mat* texture; };
    std::vector<cv::Vec3f> vertices, attributes;   // attributes = (u, v, 삼각형 번호)
    std::vector<cv::Vec3i> indices;
    std::vector<Paint> paints;
    const float zNear = 0.1f, zFar = 1000.f;       // 단위: 마커 한 변
    cv::Point2f lo((float)frame.cols, (float)frame.rows), hi(0.f, 0.f);   // 가구가 차지하는 화면 영역
    const cv::Vec3f lightDir = cv::normalize(cv::Vec3f(0.4f, 0.8f, 0.6f));   // 오른쪽 위, 카메라 쪽

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

        for (const auto& t : item.mesh->triangles) {
            cv::Vec3f cam[3];
            for (int k = 0; k < 3; ++k) {
                const cv::Vec3f p = R * (cv::Vec3f(t.v[k]) * scale - pivot) + T;
                cam[k] = { p[0], -p[1], -p[2] };   // triangleRasterize는 y=위, z=카메라 쪽
                if (p[2] < zNear) {             // 카메라 뒤로 넘어가면 영역을 계산할 수 없어 전체 화면을 그린다
                    lo = { 0.f, 0.f };
                    hi = { (float)frame.cols, (float)frame.rows };
                } else {
                    const float x = focal * p[0] / p[2] + frame.cols / 2.f, y = focal * p[1] / p[2] + frame.rows / 2.f;
                    lo = { std::min(lo.x, x), std::min(lo.y, y) };
                    hi = { std::max(hi.x, x), std::max(hi.y, y) };
                }
            }
            const cv::Vec3f n = (cam[1] - cam[0]).cross(cam[2] - cam[0]);
            const float len = (float)cv::norm(n);
            // 외부 모델은 면의 앞뒤가 뒤섞인 경우가 많아, 항상 카메라를 향한 쪽을 앞면으로 본다
            float lit = len > 0 ? n.dot(lightDir) / len : 0.f;
            if (n.dot(cam[0]) > 0) lit = -lit;

            const int base = (int)vertices.size();
            const float id = (float)paints.size();
            for (int k = 0; k < 3; ++k) {
                vertices.push_back(cam[k]);
                attributes.push_back({ t.uv[k].x, t.uv[k].y, id });
            }
            indices.push_back({ base, base + 1, base + 2 });
            paints.push_back({ t.color, 0.6f + 0.4f * std::max(0.f, lit),
                               t.texture < 0 ? nullptr : &item.mesh->textures[t.texture] });
        }
    }
    if (indices.empty()) return;

    // 프레임 전체가 아닌, 가구가 차지하는 영역(ROI)만 그려서 속도 향상
    lo = { std::max(lo.x, 0.f), std::max(lo.y, 0.f) };
    hi = { std::min(hi.x, (float)frame.cols), std::min(hi.y, (float)frame.rows) };
    if (hi.x <= lo.x || hi.y <= lo.y) return;
    const cv::Rect roi = cv::Rect(cv::Point(cvFloor(lo.x), cvFloor(lo.y)), cv::Point(cvCeil(hi.x), cvCeil(hi.y)));
    // shortcut: 가구가 화면을 크게 덮으면 반 해상도로 칠해 테두리가 2px 단위, 더 매끈해야 하면 GPU(OpenGL)로 교체
    const int step = roi.area() > 200000 ? 2 : 1;
    const cv::Size buf((roi.width + step - 1) / step, (roi.height + step - 1) / step);
    // triangleRasterize는 화면 중심을 광축으로 보므로, ROI 중심이 광축에 오도록 장면을 민다
    const float shiftX = (roi.x + buf.width * step / 2.f - frame.cols / 2.f) / focal;
    const float shiftY = (roi.y + buf.height * step / 2.f - frame.rows / 2.f) / focal;
    for (auto& p : vertices) {
        p[0] += shiftX * p[2];
        p[1] -= shiftY * p[2];
    }

    // 2) 픽셀마다 가장 가까운 삼각형을 찾는다
    colorBuf_.create(buf, CV_32FC3);
    depthBuf_.create(buf, CV_32F);
    colorBuf_.setTo(cv::Scalar(0, 0, -1));
    depthBuf_.setTo(zFar);
    cv::triangleRasterize(vertices, indices, attributes, colorBuf_, depthBuf_, cv::Matx44f::eye(),
                          2 * std::atan(buf.height * step / (2.0 * focal)), zNear, zFar,
                          cv::TriangleRasterizeSettings().setCullingMode(cv::RASTERIZE_CULLING_NONE));

    // 3) 찾은 삼각형의 색(또는 텍스처)에 밝기를 곱해 칠한다
    cv::Mat target = frame(roi);
    for (int y = 0; y < roi.height; ++y) {
        const cv::Vec3f* hit = colorBuf_.ptr<cv::Vec3f>(y / step);
        cv::Vec3b* out = target.ptr<cv::Vec3b>(y);
        for (int x = 0; x < roi.width; ++x) {
            const cv::Vec3f& h = hit[x / step];
            if (h[2] < 0) continue;
            const Paint& paint = paints[std::min(cvRound(h[2]), (int)paints.size() - 1)];
            cv::Vec3f color = paint.color;
            if (const cv::Mat* tex = paint.texture) {
                const float u = h[0] - std::floor(h[0]);   // 1을 넘으면 무늬 반복
                const float v = h[1] - std::floor(h[1]);
                color = tex->at<cv::Vec3b>(std::min((int)(v * tex->rows), tex->rows - 1),
                                           std::min((int)(u * tex->cols), tex->cols - 1));
            }
            for (int c = 0; c < 3; ++c) out[x][c] = cv::saturate_cast<uchar>(color[c] * paint.light);
        }
    }
}
