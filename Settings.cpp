#include "Settings.h"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

namespace {
std::string trim(const std::string& s) {
    const size_t a = s.find_first_not_of(" \t\r");
    return a == std::string::npos ? "" : s.substr(a, s.find_last_not_of(" \t\r") - a + 1);
}

// 설정 줄이면 이름, 아니면 빈 문자열
std::string keyOf(const std::string& line) {
    const std::string body = line.substr(0, line.find('#'));
    const size_t eq = body.find('=');
    return eq == std::string::npos ? "" : trim(body.substr(0, eq));
}

std::vector<std::string> readLines(const std::string& path) {
    std::vector<std::string> lines;
    std::ifstream in(path);
    for (std::string line; std::getline(in, line);) {
        if (lines.empty() && line.rfind("\xEF\xBB\xBF", 0) == 0) line.erase(0, 3);   // 메모장이 붙이는 BOM
        lines.push_back(line);
    }
    return lines;
}
} // namespace

Settings loadSettings(const std::string& path) {
    Settings s;
    for (const auto& line : readLines(path)) {
        const std::string key = keyOf(line);
        if (!key.empty()) s[key] = std::strtof(line.c_str() + line.find('=') + 1, nullptr);
    }
    return s;
}

float settingOr(const Settings& s, const std::string& key, float fallback, float lo, float hi) {
    const auto it = s.find(key);
    if (it == s.end()) return fallback;
    if (it->second >= lo && it->second <= hi) return it->second;
    std::cerr << "[Settings] " << key << " must be " << lo << " to " << hi << " -> using " << fallback << "\n";
    return fallback;
}

bool saveSetting(const std::string& path, const std::string& key, float value) {
    std::error_code ec;
    // 있는데 못 여는 파일을 빈 파일로 착각해 덮어쓰지 않도록
    if (std::filesystem::exists(path, ec) && !std::ifstream(path)) return false;
    std::vector<std::string> lines = readLines(path);
    const std::string entry = key + " = " + std::to_string(value);
    bool found = false;
    for (auto& line : lines)
        if (keyOf(line) == key) {
            const size_t comment = line.find('#');
            line = entry + (comment == std::string::npos ? "" : "   " + line.substr(comment));
            found = true;
        }
    if (!found) lines.push_back(entry);

    // 저장 도중 꺼져도 원래 파일이 남도록 임시 파일에 쓰고 바꿔치기
    const std::string temp = path + ".tmp";
    {
        std::ofstream out(temp);
        for (const auto& line : lines) out << line << "\n";
        out.close();
        if (!out) return false;
    }
    std::filesystem::rename(temp, path, ec);
    return !ec;
}
