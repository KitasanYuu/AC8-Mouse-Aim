// Flight controller comparison bench.
//
// Every registered controller flies the same scenarios on the same whole-aircraft model
// with the same sensor noise, so the differences are the controllers' alone:
//   - attitude response fitted from game telemetry on the game clock (2026-10-04: a 16
//     min mouse mission and a 15 min gamepad run): input delay and smoothing, response
//     curve, pull/push authority, roll acceleration and limit, high-G pull;
//   - point-mass flight path fitted from the same recordings (dev/compare/fit_airframe.py):
//     the velocity follows the nose with a 0.2 s lag (AoA ~ 0.2 s x pitch rate), gravity,
//     and speed relaxing toward ~190 m/s at neutral throttle, bleeding in high-G turns.
// Scenarios: captures of a fixed direction (the mouse moved and held), chained
// captures, precision holds (level flight, small mouse nudges, slow pans) and pursuits
// of a scripted enemy aircraft (aim = the line of sight to it).
//
//   dev\compare\compare.cmd                      build, run, open the viewer
//   harness.exe variant="label:pitch_kd=0.1;level_per_deg=10" ...   extra builds of ours
//
// Writes dev/compare/scorecard.txt (kept in git: a diff shows what a change did) and
// dev/compare/results/*.js for dev/compare/index.html.
#include "controllers/ours.h"
#include "controllers/upstream.h"
#include "controllers/pw5.h"
#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <map>
#include <random>
#include <string>
#include <vector>

namespace bench {
constexpr float rad = 0.017453292519943295f, g0 = 9.81f;
struct V3 {
    float x{}, y{}, z{};
    V3 operator+(V3 b) const { return {x + b.x, y + b.y, z + b.z}; }
    V3 operator-(V3 b) const { return {x - b.x, y - b.y, z - b.z}; }
    V3 operator*(float k) const { return {x * k, y * k, z * k}; }
};
float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float len(V3 a) { return std::sqrt(dot(a, a)); }
V3 unit(V3 a) { const float n = len(a); return n > 1e-6f ? a * (1 / n) : V3{1, 0, 0}; }
struct Frame3 { V3 f, r, u; };
// Same as flight::basis: X north, Y east, Z up; roll + lowers the right wing.
Frame3 basis(float pitch, float yaw, float roll) {
    const float p = pitch * rad, y = yaw * rad, r = roll * rad;
    const V3 f{std::cos(p) * std::cos(y), std::cos(p) * std::sin(y), std::sin(p)};
    const V3 right{-std::sin(y), std::cos(y), 0}, up = cross(f, right);
    return {f, right * std::cos(r) - up * std::sin(r), up * std::cos(r) + right * std::sin(r)};
}
V3 direction(float pitch, float yaw) { return basis(pitch, yaw, 0).f; }
V3 turn(V3 v, V3 toward, float a) { return v * std::cos(a) + toward * std::sin(a); }
void euler(const Frame3& b, float& pitch, float& yaw, float& roll) {
    pitch = std::asin(std::clamp(b.f.z, -1.0f, 1.0f)) / rad;
    yaw = std::atan2(b.f.y, b.f.x) / rad;
    const V3 right0{-std::sin(yaw * rad), std::cos(yaw * rad), 0}, up0 = cross(b.f, right0);
    roll = std::atan2(dot(b.u, right0), dot(b.u, up0)) / rad;
}
// Bank against the horizon, + right wing down.
float bank_of(const Frame3& b) { return std::atan2(-b.r.z, b.u.z) / rad; }
// Rotate the body by pitch, roll and yaw rates (deg/s) over dt.
void rotate_body(Frame3& b, float q, float p, float r, float dt) {
    V3 nf = turn(b.f, b.u, q * dt * rad), nu = turn(b.u, b.f * -1, q * dt * rad);
    b.f = nf; b.u = nu;
    V3 nu2 = turn(b.u, b.r, p * dt * rad), nr2 = turn(b.r, b.u * -1, p * dt * rad);
    b.u = nu2; b.r = nr2;
    V3 nf2 = turn(b.f, b.r, r * dt * rad), nr3 = turn(b.r, b.f * -1, r * dt * rad);
    b.f = unit(nf2); b.r = unit(nr3 - b.f * dot(nr3, b.f)); b.u = cross(b.f, b.r);
}

// ---------------------------------------------------------------- aircraft model
struct Plant {
    const char* id; const char* title;
    // attitude (same structure as tests/flight_sim.cpp)
    float pitch_delay, pitch_input, pitch_release, pitch_tau, pitch_pull, pitch_push, pitch_gravity, pitch_curve;
    float roll_delay, roll_input, roll_accel, roll_curve, roll_max, roll_tau;
    float yaw_delay, yaw_max, yaw_tau;   // rudder: full stick held ~5.4 deg/s (logged)
    float highg_pull;                     // full-pull rate multiplier, throttle+brake held
    // flight path
    float aoa_lag;                        // s: AoA = lag x path rate
    float cruise, cruise_rate;            // neutral throttle: speed relaxes to cruise (m/s, 1/s)
    float turn_drag;                      // m/s^2 per g above 1
};
// Fitted on the game clock (see tests/flight_sim.cpp `measured`, fit_airframe.py):
// full pull ~50 deg/s at any speed up to 700 m/s (arcade: no corner speed), x1.4-1.6 in
// a high-G turn; rolls top out near 105 deg/s.
constexpr Plant measured{"measured", "实测机体",
    0.12f, 0.20f, 0.10f, 0.6f, 50, 25, 0.3f, 1.5f,
    0.14f, 0.25f, 220, 1.3f, 105, 0.6f,
    0.15f, 5.4f, 0.35f, 1.5f,
    0.20f, 187, 0.0825f, 0.4f};
// Robustness: 30% more delay and smoothing, less authority (a heavier aircraft, or the
// fit being optimistic). A controller tuned to the edge of the measured model shows it here.
constexpr Plant sluggish{"sluggish", "迟钝机体（延迟 ×1.3，权限 ×0.75）",
    0.16f, 0.26f, 0.13f, 0.8f, 38, 19, 0.3f, 1.5f,
    0.18f, 0.33f, 165, 1.3f, 80, 0.7f,
    0.2f, 4.0f, 0.45f, 1.5f,
    0.25f, 187, 0.0825f, 0.4f};

float input_smooth(float f, float u, float dt, float build, float release) {
    const float tc = (u * f < 0 || std::abs(u) < std::abs(f)) ? release : build;
    return f + (u - f) * (1 - std::exp(-dt / tc));
}

struct Aircraft {
    Frame3 b;
    float q = 0, p = 0, r = 0, sq = 0, sr = 0;
    std::deque<float> dq, dr, dy;
    V3 pos, vel, acc;
    float nz = 1;
    void start(float pitch, float yaw, float roll, float speed, V3 at) {
        b = basis(pitch, yaw, roll); pos = at; vel = b.f * speed; acc = {};
    }
    void step(const Plant& m, Stick u, float throttle, float brake, float dt) {
        // attitude: dead time, the game's input smoothing, first-order response
        auto delayed = [dt](std::deque<float>& line, float value, float delay) {
            line.push_back(value);
            const size_t n = size_t(delay / dt + 0.5f);
            if (line.size() <= n) return 0.0f;
            const float out = line.front(); line.pop_front(); return out;
        };
        const float uq = delayed(dq, u.pitch, m.pitch_delay), ur = delayed(dr, u.roll, m.roll_delay), uy = delayed(dy, u.yaw, m.yaw_delay);
        sq = input_smooth(sq, uq, dt, m.pitch_input, m.pitch_release);
        sr += (ur - sr) * (1 - std::exp(-dt / m.roll_input));
        const bool high_g = throttle > 0.5f && brake > 0.5f;
        const float aq = std::pow(std::min(std::abs(sq), 1.0f), m.pitch_curve);
        const float pull = m.pitch_pull * (high_g ? m.highg_pull : 1.0f);
        const float steady = (sq >= 0 ? pull * aq : -m.pitch_push * aq) - m.pitch_gravity * b.u.z;
        q += dt * (steady - q) / m.pitch_tau;
        p = std::clamp(p + dt * (std::copysign(m.roll_accel * std::pow(std::min(std::abs(sr), 1.0f), m.roll_curve), sr) - p / m.roll_tau),
                       -m.roll_max, m.roll_max);
        r += dt * (m.yaw_max * std::clamp(uy, -1.0f, 1.0f) - r) / m.yaw_tau;
        rotate_body(b, q, p, r, dt);
        // flight path: lift turns the velocity toward the nose; gravity; speed
        const float speed = std::max(len(vel), 30.0f);
        const V3 vh = unit(vel);
        const V3 lift = (b.f - vh * dot(b.f, vh)) * (speed / m.aoa_lag);
        const V3 gravity{0, 0, -g0};
        nz = len(lift) / g0;
        float dv;
        if (high_g) dv = -28 - 0.08f * speed;                       // fitted: high-G bleeds speed
        else if (brake > 0.5f) dv = -12 - 0.135f * speed;
        else dv = (1 - throttle) * m.cruise_rate * (m.cruise - speed) + throttle * 0.125f * (750 - speed);
        dv -= m.turn_drag * std::max(0.0f, nz - 1);
        const V3 a = lift + gravity + vh * dv;
        acc = a;
        vel = vel + a * dt;
        pos = pos + vel * dt;
    }
};

// ---------------------------------------------------------------- the enemy
// Flies a scripted program: each segment holds a bank (deg against the horizon, NAN =
// keep the current body roll) and a pull rate (deg/s in its lift plane).
struct Segment { float until, bank, pull; };
struct Enemy {
    Frame3 b; V3 pos; float speed = 220;
    void step(const std::vector<Segment>& program, float t, float dt) {
        const Segment* s = nullptr;
        for (const auto& seg : program) if (t < seg.until) { s = &seg; break; }
        float roll_rate = 0, pull = 0;
        if (s) {
            pull = s->pull;
            if (!std::isnan(s->bank) && std::abs(b.f.z) < 0.95f)
                roll_rate = std::clamp(std::remainder(s->bank - bank_of(b), 360.0f) * 4, -150.0f, 150.0f);
        }
        rotate_body(b, pull, roll_rate, 0, dt);
        pos = pos + b.f * (speed * dt);
    }
};

// ---------------------------------------------------------------- scenarios
enum Kind { Capture, Chain, Hold, Pursuit };
const char* kind_name(Kind k) { return k == Capture ? "capture" : k == Chain ? "chain" : k == Hold ? "hold" : "pursuit"; }
struct Scenario {
    std::string id, title, note;
    Kind kind;
    float seconds = 8;
    float start_roll = 0;
    V3 first{}, next{}; float switch_at = -1;          // capture / chain
    std::function<V3(float)> aim;                        // hold: world aim over time
    std::vector<Segment> program;                        // pursuit: enemy program
    V3 enemy_offset{}; float enemy_yaw = 0, enemy_speed = 220;   // enemy start, relative to us (m, deg)
    bool high_g = false;                                 // the player holds throttle+brake
};
std::vector<Scenario> scenarios() {
    std::vector<Scenario> s;
    auto cap = [&](std::string id, std::string title, V3 dir, float roll = 0, std::string note = "") {
        Scenario c; c.id = id; c.title = title; c.kind = Capture; c.first = unit(dir); c.start_roll = roll; c.note = note; s.push_back(c);
    };
    cap("cap_r3", "右 3°", direction(0, 3), 0, "小角度修正");
    cap("cap_r10", "右 10°", direction(0, 10));
    cap("cap_r30", "右 30°", direction(0, 30));
    cap("cap_r90", "右 90°", direction(0, 90));
    cap("cap_r150", "右后 150°", direction(0, 150), 0, "大角度：方向选择");
    cap("cap_u20", "上 20°", direction(20, 0));
    cap("cap_d15", "下 15°", direction(-15, 0), 0, "小幅下压：不应翻成倒飞");
    cap("cap_d90", "正下 90°", V3{0.0001f, 0, -1});
    cap("cap_dl", "左下 30°/40°", direction(-30, -40));
    cap("cap_r60_lo12", "右 60° 低 12°", direction(-12, 60));
    cap("cap_l100_lo25", "左 100° 低 25°", direction(-25, -100));
    cap("cap_r40_lo50", "右 40° 低 50°", direction(-50, 40));
    cap("inv_u10", "倒飞 · 上 10°", direction(10, 0), 180, "倒飞起始");
    cap("inv_u60", "倒飞 · 上 60°", direction(60, 0), 180);
    cap("inv_r15", "倒飞 · 右 15°", direction(0, 15), 180, "推杆与拉杆距离相近");
    cap("inv_dr12", "倒飞 · 右下 12°/12°", direction(-12, 12), 180);
    cap("bank100_ul", "坡度 100° · 左上 10°/15°", direction(10, -15), 100);
    auto chain = [&](std::string id, std::string title, V3 a, float at, V3 b, std::string note) {
        Scenario c; c.id = id; c.title = title; c.kind = Chain; c.first = unit(a); c.next = unit(b); c.switch_at = at; c.seconds = 10; c.note = note; s.push_back(c);
    };
    chain("chain_up_right", "上 60° → 0.8 s → 右 40°", direction(60, 0), 0.8f, direction(30, 40), "机动中途换目标；指标从切换时刻起算");
    chain("chain_right_dl", "右 90° → 2.2 s → 左下", direction(0, 90), 2.2f, direction(-30, 50), "到达附近时换目标");
    chain("chain_down_up", "下 50° → 1.2 s → 上 20°", direction(-50, 0), 1.2f, direction(20, 10), "反向");
    chain("chain_r_l", "右 30° → 1.0 s → 左 20°", direction(0, 30), 1.0f, direction(0, -20), "左右反向");
    auto hold = [&](std::string id, std::string title, float seconds, std::function<V3(float)> aim, std::string note) {
        Scenario c; c.id = id; c.title = title; c.kind = Hold; c.seconds = seconds; c.aim = aim; c.note = note; s.push_back(c);
    };
    hold("hold_level", "平飞保持", 10, [](float) { return direction(0, 0); }, "鼠标不动，只有角速度测量噪声：晃不晃");
    hold("hold_nudge", "鼠标微调 1°/0.7 s", 10, [](float t) {
        float p = 0, y = 2;
        for (int k = 1; k <= int(t / 0.7f); ++k) {
            const unsigned h = static_cast<unsigned>(k) * 2654435761u;
            const float a = (h % 3600) * 0.1f * rad;
            p += std::sin(a); y += std::cos(a);
        }
        return direction(p, y);
    }, "已对准后每 0.7 s 小幅移动鼠标（实测会引起 ±20° 摇翼）");
    hold("hold_pan", "匀速拖动 4°/s", 10, [](float t) { return direction(0, 2 + 4 * t); }, "缓慢平移鼠标");
    hold("hold_weave", "正弦拖动 ±5°", 10, [](float t) {
        const float w = 2 * 3.14159265f * t / 2.5f; return direction(2 * std::sin(w + 1), 5 * std::sin(w));
    }, "左右来回 2.5 s 一周");
    auto pursuit = [&](std::string id, std::string title, V3 offset, float yaw, std::vector<Segment> program, std::string note, float seconds = 14, bool high_g = false) {
        Scenario c; c.id = id; c.title = title; c.kind = Pursuit; c.enemy_offset = offset; c.enemy_yaw = yaw; c.program = program;
        c.note = note; c.seconds = seconds; c.high_g = high_g; s.push_back(c);
    };
    pursuit("pur_turn", "追击 · 敌机持续右转", {700, 120, 0}, 0, {{1, 65, 0}, {99, 65, 16}}, "敌机 65° 坡度稳定盘旋（约 16°/s）");
    pursuit("pur_break", "追击 · 敌机急转脱离", {600, -80, 20}, 0, {{1.5f, 0, 0}, {2.2f, -80, 0}, {99, -80, 30}}, "1.5 s 后左压 80° 急拉 30°/s");
    pursuit("pur_break_hg", "追击 · 急转 + 玩家高G", {600, -80, 20}, 0, {{1.5f, 0, 0}, {2.2f, -80, 0}, {99, -80, 30}}, "同上，玩家同时按住油门和减速（高G转弯）", 14, true);
    pursuit("pur_scissors", "追击 · 剪刀机动", {500, 0, 0}, 0,
            {{0.5f, 70, 0}, {3, 70, 20}, {3.5f, -70, 0}, {6, -70, 20}, {6.5f, 70, 0}, {9, 70, 20}, {9.5f, -70, 0}, {99, -70, 20}}, "每 3 s 反向一次");
    pursuit("pur_weave", "追击 · 爬升蛇行", {600, 50, -30}, 10,
            {{2, 35, 8}, {4, -35, 8}, {6, 35, 8}, {8, -35, 8}, {10, 35, 8}, {12, -35, 8}, {99, 35, 8}}, "±35° 坡度小幅拉起交替");
    pursuit("pur_jink", "尾追 · 小幅抖动", {450, 0, 0}, 0,
            {{1.0f, 25, 4}, {2.2f, -25, 4}, {3.0f, 20, 3}, {4.5f, -30, 5}, {5.5f, 25, 4}, {7, -20, 3}, {8, 30, 5}, {9.5f, -25, 4}, {11, 20, 3}, {99, -25, 4}},
            "敌机在机头前方左右小幅抖动（实测会满杆左右滚）");
    pursuit("pur_split_s", "追击 · 敌机半滚倒转", {600, 40, 0}, 0, {{1.0f, 0, 0}, {2.2f, 180, 0}, {8.5f, NAN, 28}, {99, 0, 0}}, "滚到倒飞后拉杆向下脱离");
    pursuit("pur_headon", "对头 · 交错后反转", {1800, 150, 60}, 180, {{99, 0, 0}}, "对头交错，之后需要掉头 180°（方向选择）", 16);
    return s;
}

// ---------------------------------------------------------------- metrics
struct Metrics {
    float to5 = -1, to2 = -1, settle = -1, level = -1, overshoot = 0;
    int reversals = 0, chatter = 0;
    float inverted = 0, speed_loss = 0, alt_change = 0, peak_nz = 0;
    float mean_angle = -1, rms_angle = -1, within2 = -1, within5 = -1, bank_rms = -1;
    float pitch_sat = 0, roll_sat = 0, roll_effort = 0;
    float cost = 0;
};
struct Sample { float v[26]; int n; };

struct Run { Metrics m; std::vector<Sample> frames; };

// Measurement conditions: the aircraft model plus how the host senses body rates.
// In game, rates come from the pose change between frames divided by the frame time.
// On the game clock (this repository since 2026-10-04) the filtered rates are quiet:
// 0.1-0.2 deg/s rms in level flight (logged). Hosts timing frames with the wall clock
// (upstream and pw5 builds, and ours before) divide by a time that jitters 9-18 ms around
// the ~12.5 ms frame, a multiplicative error of roughly 20%.
struct Condition { const char* id; const char* title; const Plant* plant; float white; float jitter; };
const Condition conditions[] = {
    {"measured", "实测机体 · 游戏时钟", &measured, 0.15f, 0.0f},
    {"wallclock", "实测机体 · 墙钟计时（角速度噪声大）", &measured, 0.15f, 0.2f},
    {"sluggish", "迟钝机体（延迟 ×1.3，权限 ×0.75）· 游戏时钟", &sluggish, 0.15f, 0.0f},
};

Run fly(const Condition& cond, const Scenario& sc, Controller& ctl, bool keep_frames) {
    const Plant& plant = *cond.plant;
    const float dt = 1.0f / 80;   // logged median frame 12.5-13.3 ms
    const float speed0 = 220;
    Aircraft me; me.start(0, 0, sc.start_roll, speed0, {0, 0, 3000});
    Enemy enemy;
    if (sc.kind == Pursuit) {
        const Frame3 h = basis(0, 0, 0);
        enemy.pos = me.pos + h.f * sc.enemy_offset.x + h.r * sc.enemy_offset.y + h.u * sc.enemy_offset.z;
        enemy.b = basis(0, sc.enemy_yaw, 0); enemy.speed = sc.enemy_speed;
    }
    ctl.reset();
    std::mt19937 rng(1234);
    std::normal_distribution<float> unit_noise(0.0f, 1.0f);
    float fq = 0, fp = 0, fr = 0;
    Run run; Metrics& m = run.m;
    float start_t = 0, min_angle = 180, held = 0, held_level = 0, last_q = 0, last_r = 0;
    int last_sign = 0;
    double sum_angle = 0, sum_angle2 = 0, sum_bank2 = 0, track_time = 0, in2 = 0, in5 = 0;
    double sat_q = 0, sat_r = 0, effort = 0, total = 0;
    const float track_from = sc.kind == Pursuit ? 3.0f : sc.kind == Hold ? 1.0f : 1e9f;
    V3 target = sc.first;
    const int steps = int(sc.seconds / dt);
    const float alt0 = me.pos.z;
    const bool started_inverted = std::abs(sc.start_roll) > 100;
    for (int k = 0; k < steps; ++k) {
        const float t = k * dt;
        if (sc.kind == Chain && k == int(sc.switch_at / dt)) {
            target = sc.next; start_t = t;
            m.to5 = m.to2 = m.settle = m.level = -1; m.overshoot = 0; m.reversals = 0;
            min_angle = 180; held = held_level = 0; last_sign = 0;
        }
        if (sc.kind == Hold) target = unit(sc.aim(t));
        if (sc.kind == Pursuit) { enemy.step(sc.program, t, dt); target = unit(enemy.pos - me.pos); }
        float pitch, yaw, roll; euler(me.b, pitch, yaw, roll);
        // the host's view of this frame: measured frame time, rates from the pose change
        const float seen_dt = dt * std::clamp(1 + cond.jitter * unit_noise(rng), 0.5f, 1.6f);
        {
            const float scale = dt / seen_dt, a = 1 - std::exp(-12 * seen_dt);
            fq += (me.q * scale + cond.white * unit_noise(rng) - fq) * a;
            fp += (me.p * scale + cond.white * unit_noise(rng) - fp) * a;
            fr += (me.r * scale + cond.white * unit_noise(rng) - fr) * a;
        }
        Sense s{pitch, yaw, roll, {target.x, target.y, target.z}, fq, fr, fp, seen_dt,
                sc.high_g ? 1.0f : 0.0f, sc.high_g ? 1.0f : 0.0f,
                {me.vel.x, me.vel.y, me.vel.z}, {me.acc.x, me.acc.y, me.acc.z}, me.pos.z};
        Stick u = ctl.step(s);
        u.pitch = std::clamp(u.pitch, -1.0f, 1.0f); u.roll = std::clamp(u.roll, -1.0f, 1.0f); u.yaw = std::clamp(u.yaw, -1.0f, 1.0f);
        me.step(plant, u, s.throttle, s.brake, dt);

        const float angle = std::acos(std::clamp(dot(me.b.f, target), -1.0f, 1.0f)) / rad;
        const float bank = bank_of(me.b), tt = t - start_t;
        if (t > 1.0f) {   // stick reversals between beyond +0.3 and beyond -0.3
            if (std::abs(u.pitch) > 0.3f) { const float sg = u.pitch > 0 ? 1.0f : -1.0f; if (last_q * sg < 0) ++m.chatter; last_q = sg; }
            if (std::abs(u.roll) > 0.3f) { const float sg = u.roll > 0 ? 1.0f : -1.0f; if (last_r * sg < 0) ++m.chatter; last_r = sg; }
        }
        if (std::abs(me.p) > 15) { const int sg = me.p > 0 ? 1 : -1; if (last_sign && sg != last_sign) ++m.reversals; last_sign = sg; }
        if (std::abs(bank) > 100) m.inverted += dt;
        m.peak_nz = std::max(m.peak_nz, me.nz);
        sat_q += std::abs(u.pitch) > 0.95f; sat_r += std::abs(u.roll) > 0.95f; effort += std::abs(u.roll); total += 1;
        if (m.to5 < 0 && angle < 5) m.to5 = tt;
        if (m.to2 < 0 && angle < 2) m.to2 = tt;
        if (m.to2 >= 0) { min_angle = std::min(min_angle, angle); m.overshoot = std::max(m.overshoot, angle - min_angle); }
        held = angle < 1.0f ? held + dt : 0;
        if (m.settle < 0 && held >= 0.5f) m.settle = tt - 0.5f;
        held_level = (angle < 1.0f && std::abs(bank) < 5.0f) ? held_level + dt : 0;
        if (m.level < 0 && held_level >= 0.5f) m.level = tt - 0.5f;
        if (t >= track_from) {
            sum_angle += angle * dt; sum_angle2 += angle * angle * dt; sum_bank2 += bank * bank * dt; track_time += dt;
            in2 += angle < 2 ? dt : 0; in5 += angle < 5 ? dt : 0;
        }
        if (keep_frames && k % 4 == 0) {
            float ep = 0, ey = 0, er = 0;
            if (sc.kind == Pursuit) euler(enemy.b, ep, ey, er);
            Sample f{{t, me.pos.x, me.pos.y, me.pos.z, pitch, yaw, roll, target.x, target.y, target.z, angle,
                      me.q, me.p, me.r, u.pitch, u.roll, u.yaw, len(me.vel), me.nz,
                      std::atan2(-dot(me.vel, me.b.u), dot(me.vel, me.b.f)) / rad,
                      enemy.pos.x, enemy.pos.y, enemy.pos.z, ep, ey, er}, sc.kind == Pursuit ? 26 : 20};
            run.frames.push_back(f);
        }
    }
    m.speed_loss = speed0 - len(me.vel);
    m.alt_change = me.pos.z - alt0;
    m.pitch_sat = float(sat_q / total); m.roll_sat = float(sat_r / total); m.roll_effort = float(effort / total);
    if (track_time > 0) {
        m.mean_angle = float(sum_angle / track_time); m.rms_angle = float(std::sqrt(sum_angle2 / track_time));
        m.bank_rms = float(std::sqrt(sum_bank2 / track_time));
        m.within2 = float(in2 / track_time); m.within5 = float(in5 / track_time);
    }
    // One number per scenario, lower is better. Rough weights: responsiveness first, then
    // clear overshoot, roll reversals and stick chatter; tracking error for holds/pursuits.
    const float span = sc.kind == Chain ? sc.seconds - sc.switch_at : sc.seconds;
    if (sc.kind == Capture || sc.kind == Chain)
        m.cost = (m.to2 < 0 ? span : m.to2) + 0.5f * (m.settle < 0 ? span : m.settle) + 2 * std::max(0.0f, m.overshoot - 1.5f) +
                 0.5f * std::max(0, m.reversals - 1) + 0.2f * m.chatter + (started_inverted ? 0 : 0.5f * m.inverted);
    else if (sc.kind == Hold)
        m.cost = m.rms_angle + 0.05f * m.bank_rms + 0.3f * m.reversals + 0.1f * m.chatter;
    else
        m.cost = 0.5f * m.mean_angle + 3 * (1 - m.within5) + 0.2f * m.reversals + 0.05f * m.chatter;
    return run;
}

// ---------------------------------------------------------------- registry and output
struct Entry { std::string id, label, source; std::function<std::unique_ptr<Controller>()> make; };

void json_metrics(std::string& out, const Metrics& m) {
    char b[700];
    std::snprintf(b, sizeof(b),
        "{\"to5\":%.2f,\"to2\":%.2f,\"settle\":%.2f,\"level\":%.2f,\"overshoot\":%.2f,\"reversals\":%d,\"chatter\":%d,"
        "\"inverted\":%.2f,\"speed_loss\":%.1f,\"alt_change\":%.0f,\"peak_nz\":%.1f,\"mean_angle\":%.2f,\"rms_angle\":%.2f,"
        "\"within2\":%.3f,\"within5\":%.3f,\"bank_rms\":%.1f,\"pitch_sat\":%.3f,\"roll_sat\":%.3f,\"roll_effort\":%.3f,\"cost\":%.2f}",
        m.to5, m.to2, m.settle, m.level, m.overshoot, m.reversals, m.chatter, m.inverted, m.speed_loss, m.alt_change, m.peak_nz,
        m.mean_angle, m.rms_angle, m.within2, m.within5, m.bank_rms, m.pitch_sat, m.roll_sat, m.roll_effort, m.cost);
    out += b;
}
std::string esc(const std::string& s) {
    std::string o;
    for (char c : s) { if (c == '"' || c == '\\') o += '\\'; o += c; }
    return o;
}
}  // namespace bench

int main(int argc, char** argv) {
    using namespace bench;
    std::string out_dir = "dev/compare", config = "Payload/Game/Binaries/Win64/UE4SS/Mods/AC8MouseAim/config.ini";
    std::vector<std::pair<std::string, std::string>> variants;
    std::string only;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto eq = arg.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = arg.substr(0, eq), value = arg.substr(eq + 1);
        if (key == "out") out_dir = value;
        else if (key == "config") config = value;
        else if (key == "only") only = value;
        else if (key == "variant") {   // "label:key=value;key=value"
            const auto colon = value.find(':');
            variants.push_back({value.substr(0, colon), colon == std::string::npos ? "" : value.substr(colon + 1)});
        }
    }
    const std::string scratch = out_dir + "/results/tuning.ini";
    CreateDirectoryA((out_dir + "/results").c_str(), nullptr);
    std::vector<Entry> entries;
    const flight::Tuning base = ours_tuning(config, "", scratch);
    entries.push_back({"ours", "本仓库（当前工作区）", "src/flight_logic.h + Payload config.ini", [base] { return std::make_unique<Ours>(base); }});
    for (size_t i = 0; i < variants.size(); ++i) {
        const flight::Tuning t = ours_tuning(config, variants[i].second, scratch);
        entries.push_back({"ours_v" + std::to_string(i + 1), "本仓库 · " + variants[i].first, variants[i].second, [t] { return std::make_unique<Ours>(t); }});
    }
    entries.push_back({"upstream_legacy", "主仓库 0.2.30（发布版）", "FletcherMiya 052cd6a controller_mode=0", [] { return std::make_unique<UpstreamLegacy>(); }});
    entries.push_back({"upstream_coord", "主仓库 0.2.35 协调制导", "FletcherMiya 052cd6a controller_mode=1", [] { return std::make_unique<UpstreamCoordinated>(); }});
    entries.push_back({"pw5", "xsd467 pw5", "xsd467 821f442 pw5-flight-control", [] { return std::make_unique<Pw5>(); }});

    const auto list = scenarios();
    std::string index = "window.COMPARE_INDEX={\"controllers\":[";
    for (size_t i = 0; i < entries.size(); ++i)
        index += std::string(i ? "," : "") + "{\"id\":\"" + entries[i].id + "\",\"label\":\"" + esc(entries[i].label) + "\",\"source\":\"" + esc(entries[i].source) + "\"}";
    index += "],\"plants\":[";
    for (int p = 0; p < 3; ++p) index += std::string(p ? "," : "") + "{\"id\":\"" + conditions[p].id + "\",\"title\":\"" + conditions[p].title + "\"}";
    index += "],\"scenarios\":[";
    for (size_t i = 0; i < list.size(); ++i)
        index += std::string(i ? "," : "") + "{\"id\":\"" + list[i].id + "\",\"title\":\"" + esc(list[i].title) + "\",\"note\":\"" + esc(list[i].note) +
                 "\",\"kind\":\"" + kind_name(list[i].kind) + "\",\"seconds\":" + std::to_string(list[i].seconds) +
                 ",\"switch_at\":" + std::to_string(list[i].switch_at) + "}";
    index += "],\"metrics\":{";

    std::string card;
    char line[512];
    for (int p = 0; p < 3; ++p) {
        const Condition& plant = conditions[p];
        std::string data = std::string("(window.COMPARE_DATA=window.COMPARE_DATA||{})[\"") + plant.id + "\"]={";
        index += std::string(p ? "," : "") + "\"" + plant.id + "\":{";
        std::snprintf(line, sizeof(line), "\n== %s (%s) ==  cost per scenario, lower is better\n%-26s", plant.id, plant.title, "scenario");
        card += line;
        for (const auto& e : entries) { std::snprintf(line, sizeof(line), " %15s", e.id.c_str()); card += line; }
        card += "\n";
        std::map<std::string, std::vector<float>> group_cost;
        for (size_t si = 0; si < list.size(); ++si) {
            const Scenario& sc = list[si];
            if (!only.empty() && sc.id.find(only) == std::string::npos) continue;
            data += std::string(si ? "," : "") + "\"" + sc.id + "\":{";
            index += std::string(si ? "," : "") + "\"" + sc.id + "\":{";
            std::snprintf(line, sizeof(line), "%-26s", sc.id.c_str()); card += line;
            for (size_t ci = 0; ci < entries.size(); ++ci) {
                auto ctl = entries[ci].make();
                const Run run = fly(plant, sc, *ctl, true);
                group_cost[std::string(kind_name(sc.kind)) + "#" + entries[ci].id].push_back(run.m.cost);
                std::snprintf(line, sizeof(line), " %15.2f", run.m.cost); card += line;
                index += std::string(ci ? "," : "") + "\"" + entries[ci].id + "\":"; json_metrics(index, run.m);
                data += std::string(ci ? "," : "") + "\"" + entries[ci].id + "\":[";
                for (size_t k = 0; k < run.frames.size(); ++k) {
                    const Sample& f = run.frames[k];
                    data += k ? ",[" : "[";
                    for (int j = 0; j < f.n; ++j) {
                        char num[32];
                        const bool coarse = (j >= 1 && j <= 3) || (j >= 20 && j <= 22);   // positions
                        std::snprintf(num, sizeof(num), coarse ? "%s%.1f" : (j >= 7 && j <= 9) ? "%s%.4f" : "%s%.2f", j ? "," : "", f.v[j]);
                        data += num;
                    }
                    data += "]";
                }
                data += "]";
            }
            card += "\n";
            data += "}";
            index += "}";
        }
        data += "};\n";
        index += "}";
        std::ofstream(out_dir + "/results/data_" + plant.id + ".js", std::ios::binary) << data;
        std::snprintf(line, sizeof(line), "%-26s", "-- total by kind --"); card += line; card += "\n";
        for (const char* kind : {"capture", "chain", "hold", "pursuit"}) {
            std::snprintf(line, sizeof(line), "%-26s", kind); card += line;
            for (const auto& e : entries) {
                float sum = 0; for (float c : group_cost[std::string(kind) + "#" + e.id]) sum += c;
                std::snprintf(line, sizeof(line), " %15.2f", sum); card += line;
            }
            card += "\n";
        }
        std::snprintf(line, sizeof(line), "%-26s", "ALL"); card += line;
        for (const auto& e : entries) {
            float sum = 0;
            for (const char* kind : {"capture", "chain", "hold", "pursuit"}) for (float c : group_cost[std::string(kind) + "#" + e.id]) sum += c;
            std::snprintf(line, sizeof(line), " %15.2f", sum); card += line;
        }
        card += "\n";
    }
    index += "}};\n";
    std::ofstream(out_dir + "/results/index.js", std::ios::binary) << index;
    std::string head = "Flight controller comparison (dev/compare). Controllers:\n";
    for (const auto& e : entries) head += "  " + e.id + ": " + e.label + "  [" + e.source + "]\n";
    card = head + card;
    std::fputs(card.c_str(), stdout);
    if (only.empty() && variants.empty()) std::ofstream(out_dir + "/scorecard.txt", std::ios::binary) << card;
    return 0;
}
