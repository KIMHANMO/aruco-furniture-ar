#pragma once
#include <map>
#include <string>

// "이름 = 숫자" 형식의 설정 파일 (# 뒤는 설명). 빌드하지 않고 값을 바꾸기 위해 사용
using Settings = std::map<std::string, float>;

Settings loadSettings(const std::string& path);   // 파일이 없으면 빈 값
// 설정이 있고 lo~hi 안이면 그 값, 아니면 fallback
float settingOr(const Settings& s, const std::string& key, float fallback, float lo, float hi);
// 다른 줄과 설명은 그대로 두고 key 하나만 바꿔 저장
bool saveSetting(const std::string& path, const std::string& key, float value);
