#pragma once
// Release 0.2.30+Alf.1.0.0 (tag alf-v1.0.0): its flight logic and its shipped [tuning], both
// frozen in alf_1_0_0/, so the column stays that release whatever the working tree becomes.
#include "controller.h"
#include "alf_1_0_0/flight_logic.h"
#include <fstream>
#include <map>
#include <sstream>
#include <string>

namespace bench {
inline alf_1_0_0::Tuning alf_1_0_0_tuning(const std::string& folder) {
    const std::string path = folder + "/controllers/alf_1_0_0/config.ini";
    const std::wstring wide(path.begin(), path.end());
    wchar_t full[MAX_PATH]{};
    GetFullPathNameW(wide.c_str(), MAX_PATH, full, nullptr);
    return alf_1_0_0::read_tuning(full);
}

// Whether the working tree is still this release: the same flight logic (the frozen copy less its
// two header lines and its namespace) and the same [tuning]. Then the release needs no column of
// its own: the working tree's column is it.
inline std::string alf_1_0_0_text(const std::string& path, int skip_lines) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream all; all << in.rdbuf();
    std::string text = all.str(), out;
    if (text.rfind("\xEF\xBB\xBF", 0) == 0) text.erase(0, 3);
    for (char c : text) if (c != '\r') out += c;
    for (int i = 0; i < skip_lines && out.find('\n') != std::string::npos; ++i) out.erase(0, out.find('\n') + 1);
    return out;
}
inline std::map<std::string, std::string> alf_1_0_0_tuning_keys(const std::string& path) {
    std::map<std::string, std::string> keys;
    std::stringstream in(alf_1_0_0_text(path, 0));
    bool on = false;
    for (std::string line; std::getline(in, line);) {
        while (!line.empty() && line.back() == ' ') line.pop_back();
        if (!line.empty() && line[0] == '[') { on = line == "[tuning]"; continue; }
        const auto eq = line.find('=');
        if (!on || line.empty() || line[0] == ';' || eq == std::string::npos) continue;
        keys[line.substr(0, eq)] = line.substr(eq + 1);
    }
    return keys;
}
inline bool alf_1_0_0_is_working_tree(const std::string& folder, const std::string& config) {
    for (const char* f : {"flight_math.h", "maneuver.h", "flight_logic.h"}) {
        std::string frozen = alf_1_0_0_text(folder + "/controllers/alf_1_0_0/" + f, 2);
        for (size_t at = frozen.find("namespace alf_1_0_0 {"); at != std::string::npos; at = frozen.find("namespace alf_1_0_0 {", at))
            frozen.replace(at, 21, "namespace flight {");
        if (frozen != alf_1_0_0_text(std::string("src/") + f, 0)) return false;
    }
    return alf_1_0_0_tuning_keys(folder + "/controllers/alf_1_0_0/config.ini") == alf_1_0_0_tuning_keys(config);
}

struct Alf_1_0_0 : Controller {
    alf_1_0_0::Tuning tuning;
    std::unique_ptr<alf_1_0_0::LogicState> state = std::make_unique<alf_1_0_0::LogicState>();
    alf_1_0_0::LogicOutput out{};
    explicit Alf_1_0_0(const alf_1_0_0::Tuning& t) : tuning(t) {}
    void reset() override { state = std::make_unique<alf_1_0_0::LogicState>(); }
    Stick step(const Sense& s) override {
        const alf_1_0_0::LogicInput in{s.pitch, s.yaw, s.roll, s.aim[0], s.aim[1], s.aim[2],
                                       s.pitch_rate, s.yaw_rate, s.roll_rate, s.dt, 0, 1, s.throttle, s.brake,
                                       s.vel[0], s.vel[1], s.vel[2], s.acc[0], s.acc[1], s.acc[2], s.altitude};
        alf_1_0_0::logic_step(tuning, *state, in, out);
        return {out.pitch, out.roll, out.yaw};
    }
    const char* trace() const override { return out.trace; }
};
}
