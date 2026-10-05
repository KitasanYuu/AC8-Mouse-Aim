#pragma once
// This repository's flight logic (src/flight_logic.h, working tree), with the [tuning]
// of the shipped Payload config.ini plus optional overrides ("pitch_kd=0.1;...").
#include "controller.h"
#include "../../../src/flight_logic.h"
#include <fstream>
#include <sstream>

namespace bench {
// Reads [tuning] the way the mod does (flight::read_tuning), with the override keys
// placed first in the section, where GetPrivateProfileString finds them first.
inline flight::Tuning ours_tuning(const std::string& config, const std::string& overrides, const std::string& scratch) {
    std::ifstream in(config);
    std::stringstream text; text << in.rdbuf();
    std::string ini = text.str(), extra;
    std::stringstream list(overrides);
    for (std::string item; std::getline(list, item, ';');) if (!item.empty()) extra += item + "\n";
    const auto at = ini.find("[tuning]");
    if (at == std::string::npos) ini += "\n[tuning]\n" + extra;
    else ini.insert(ini.find('\n', at) + 1, extra);
    std::ofstream(scratch, std::ios::binary) << ini;
    const std::wstring path(scratch.begin(), scratch.end());
    wchar_t full[MAX_PATH]{};
    GetFullPathNameW(path.c_str(), MAX_PATH, full, nullptr);
    return flight::read_tuning(full);
}

struct Ours : Controller {
    flight::Tuning tuning;
    std::unique_ptr<flight::LogicState> state = std::make_unique<flight::LogicState>();
    flight::LogicOutput out{};
    explicit Ours(const flight::Tuning& t) : tuning(t) {}
    void reset() override { state = std::make_unique<flight::LogicState>(); }
    Stick step(const Sense& s) override {
        const flight::LogicInput in{s.pitch, s.yaw, s.roll, s.aim[0], s.aim[1], s.aim[2],
                                    s.pitch_rate, s.yaw_rate, s.roll_rate, s.dt, 0, 1, s.throttle, s.brake,
                                    s.vel[0], s.vel[1], s.vel[2], s.acc[0], s.acc[1], s.acc[2], s.altitude};
        flight::logic_step(tuning, *state, in, out);
        return {out.pitch, out.roll, out.yaw};
    }
    const char* trace() const override { return out.trace; }
};
}
