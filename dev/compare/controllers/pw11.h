#pragma once
// xsd467/AC8-Mouse-Aim pw5-flight-control @ 05d7f48 (pw.11), called exactly as its
// update_commands() does (angle control with rollout coordination and terminal braking), with its
// shipped config.ini values where they differ from the ResponseSettings/BankSettings defaults
// (max_bank 89, roll_rate_scale 1.35, roll_lookahead 0.12, high_g_yaw_boost on).
#include "controller.h"
#include "pw11/terminal_braking.h"
#include "pw11/rollout_coordination.h"

namespace bench {
struct Pw11 : Controller {
    pw11::BankGuidance bank;
    pw11::HighGBraking high_g_braking;
    pw11::RolloutCoordinator rollout_coordinator;
    pw11::TerminalBraking terminal_braking;
    pw11::ResponseSettings config;
    pw11::BankSettings bank_settings;
    Pw11() {
        bank_settings.max_bank = 89;
        config.roll_rate_scale = 1.35f;
        config.roll_lookahead = 0.12f;
        config.high_g_yaw_boost = true;
    }
    void reset() override { bank = {}; high_g_braking.reset(); rollout_coordinator.reset(); terminal_braking.reset(); }
    Stick step(const Sense& s) override {
        using namespace pw11;
        const Basis b = basis(s.pitch, s.yaw, s.roll);
        const V aim{s.aim[0], s.aim[1], s.aim[2]};
        const float angle = std::acos(std::clamp(dot(aim, b.f), -1.0f, 1.0f)) / rad;
        ResponseSettings response = config;
        response.high_g_requested = config.high_g_enabled && s.throttle > 0.5f && s.brake > 0.5f;
        response.high_g_brake_weight = high_g_braking.step(response.high_g_requested, s.dt, response.high_g_brake_hold, angle, s.pitch_rate);
        response.high_g_settling = high_g_braking.settling();
        const auto bk = bank.step(b, aim, angle, s.dt, bank_settings, response);
        const auto rollout = rollout_coordinator.step(b, aim, bk, s.pitch_rate, s.yaw_rate, s.dt, true);
        const float roll_drift = horizon_roll_drift(b, s.pitch_rate, s.yaw_rate);
        const auto turn = angle_turn(b, aim, bk, s.pitch_rate, s.yaw_rate, s.roll_rate + roll_drift, response);
        const auto roll = angle_roll(rollout.bank, s.roll_rate, response, roll_drift);
        const float pitch_cmd = terminal_braking.step(b, aim, bk, s.pitch_rate, turn, response, s.dt);
        return {pitch_cmd, roll.command, turn.yaw.command};
    }
};
}
