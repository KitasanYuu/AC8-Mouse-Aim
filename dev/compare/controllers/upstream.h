#pragma once
// FletcherMiya/AC8-Mouse-Aim (052cd6a), both controllers in its src/mouse_aim.cpp
// update_commands(), called the way it calls them:
//   controller_mode=0  0.2.30 arrival-command PD (what its current release ships);
//   controller_mode=1  0.2.35 coordinated guidance (in source, release reverted it).
#include "controller.h"
#include "upstream/flight_math.h"

namespace bench {
struct UpstreamCoordinated : Controller {
    upstream::CoordinatedGuidance guidance;
    void reset() override { guidance.reset(); }
    Stick step(const Sense& s) override {
        const upstream::Basis b = upstream::basis(s.pitch, s.yaw, s.roll);
        const upstream::V aim{s.aim[0], s.aim[1], s.aim[2]};
        const auto g = guidance.step(b, aim, s.pitch_rate, s.yaw_rate, s.roll_rate, s.dt);
        return {g.pitch, g.roll, g.yaw};
    }
};

struct UpstreamLegacy : Controller {
    upstream::LevelBlend roll_level_blend;
    void reset() override { roll_level_blend.reset(); }
    Stick step(const Sense& s) override {
        using namespace upstream;
        const Basis b = basis(s.pitch, s.yaw, s.roll);
        const V aim{s.aim[0], s.aim[1], s.aim[2]};
        const float f = dot(aim, b.f), right = dot(aim, b.r), up = dot(aim, b.u);
        const float angle = std::acos(std::clamp(f, -1.0f, 1.0f)) / rad;
        const float pitch_error = std::atan2(up, std::max(0.02f, f)) / rad;
        const float yaw_error = std::atan2(right, std::max(0.02f, f)) / rad;
        const float level_error = std::atan2(b.r.z, b.u.z) / rad;
        const float near_blend = tracking_weight(angle);
        const float turn_bank_error = std::clamp(std::atan2(right, std::max(0.12f, up)) / rad, -90.0f, 90.0f);
        const float roll_blend = roll_level_blend.step(angle, s.dt);
        const float roll_error = level_error * (1 - roll_blend) + turn_bank_error * roll_blend;
        const float roll_speed = 70.0f + 70.0f * roll_blend;
        const float rcmd = arrival_command(roll_error, s.roll_rate, roll_speed, 240.0f, 3.5f, 1.0f, 170.0f, 1.0f);
        const float final_gain = 1.0f - near_blend;
        const float pcmd = arrival_command(pitch_error, s.pitch_rate, 45.0f, 90.0f, 1.8f + 0.8f * final_gain, 0.2f, 55.0f, 0.85f);
        const float yaw_rate = std::clamp(dead(yaw_error, 0.2f) * (1.2f + 0.6f * final_gain), -7.0f, 7.0f);
        const float ycmd = std::clamp((yaw_rate - s.yaw_rate * 0.6f) / 10.0f, -0.7f, 0.7f);
        return {pcmd, rcmd, ycmd};
    }
};
}
