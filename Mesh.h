#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>

// 3D 모델을 이루는 삼각형 1개
struct Triangle {
    cv::Point3f v[3];
    cv::Point2f uv[3];                          // 텍스처(무늬 그림) 좌표
    cv::Vec3f color = { 180.f, 180.f, 180.f };  // BGR, 텍스처가 없을 때의 색
    int texture = -1;                           // Mesh::textures 번호, -1 = 없음
};

// 외부에서 가져온 3D 모델. 파일마다 단위가 달라서 크기를 1로 통일해 둔다.
// 좌표계: x=오른쪽, y=마커 위쪽, z=마커에서 솟아오르는 방향 / 바닥 z=0, 가장 긴 변=1
struct Mesh {
    std::vector<Triangle> triangles;
    std::vector<cv::Mat> textures;
    cv::Point3f size;   // 가로, 깊이, 높이 (가장 긴 변 = 1)

    static Mesh loadObj(const std::string& path);   // 실패하면 빈 Mesh
    static Mesh makeBox(const cv::Scalar& color);   // 모델 파일이 없을 때 쓰는 상자
};
