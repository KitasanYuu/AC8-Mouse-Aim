#pragma once
// xsd467/AC8-Mouse-Aim pw5-flight-control (821f442), called exactly as its
// update_commands() does, with its shipped config (the ResponseSettings defaults).
#include "controller.h"
#include "pw5/turn_guidance.h"

namespace bench {
struct Pw5 : Controller {
    pw5::BankGuidance bank;
    pw5::ResponseSettings response;
    pw5::BankSettings bank_settings;
    void reset() override { bank = {}; }
    Stick step(const Sense& s) override {
        using namespace pw5;
        response.high_g_requested = s.throttle > 0.5f && s.brake > 0.5f;
        const Basis b = basis(s.pitch, s.yaw, s.roll);
        const V aim{s.aim[0], s.aim[1], s.aim[2]};
        const float f = dot(aim, b.f), right = dot(aim, b.r), up = dot(aim, b.u);
        const float angle = std::acos(std::clamp(f, -1.0f, 1.0f)) / rad;
        const float pitch_error = std::atan2(up, std::max(0.02f, f)) / rad;
        const float yaw_error = std::atan2(right, std::max(0.02f, f)) / rad;
        const float near_blend = tracking_weight(angle);
        const auto bk = bank.step(b, aim, angle, s.dt, bank_settings, response);
        const float roll_drift = horizon_roll_drift(b, s.pitch_rate, s.yaw_rate);
        const float roll_cmd = roll_command(bk, s.roll_rate, response, roll_drift);
        const float final_gain = 1.0f - near_blend;
        const auto turn = coordinated_turn(b, aim, bk, s.pitch_rate, s.yaw_rate, s.roll_rate + roll_drift, response);
        const float pitch_cmd = bk.diving ? dive_pitch_command(pitch_error, s.pitch_rate, final_gain, response) : turn.pitch_command;
        const float wanted_yaw = std::clamp(dead(yaw_error, 0.2f) * (1.2f + 0.6f * final_gain), -7.0f, 7.0f);
        const float yaw_cmd = bk.diving ? std::clamp((wanted_yaw - s.yaw_rate * 0.6f) / 10.0f, -0.7f, 0.7f) : turn.yaw_command;
        return {pitch_cmd, roll_cmd, yaw_cmd};
    }
};
}
