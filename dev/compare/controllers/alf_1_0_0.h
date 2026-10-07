#pragma once
// Release 0.2.30+Alf.1.0.0 (tag alf-v1.0.0): its flight logic and its shipped [tuning], both
// frozen in alf_1_0_0/, so the column stays that release whatever the working tree becomes.
#include "controller.h"
#include "alf_1_0_0/flight_logic.h"
#include <string>

namespace bench {
inline alf_1_0_0::Tuning alf_1_0_0_tuning(const std::string& folder) {
    const std::string path = folder + "/controllers/alf_1_0_0/config.ini";
    const std::wstring wide(path.begin(), path.end());
    wchar_t full[MAX_PATH]{};
    GetFullPathNameW(wide.c_str(), MAX_PATH, full, nullptr);
    return alf_1_0_0::read_tuning(full);
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
