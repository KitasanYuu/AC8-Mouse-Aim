// Flight controller comparison bench: the same decisions against the same enemies.
//
// Every registered controller flies the same scenes on the same whole-aircraft model with the
// same sensor noise and, in scenes with enemies, the same simulated player making the same
// decisions (target choice, aim on the gun lead point, throttle, firing), so the differences
// are the controllers' alone. Every scene runs on every condition (aircraft model).
//   - attitude response fitted from game telemetry on the game clock: input delay and
//     smoothing, response curve, pull/push authority, roll acceleration and limit, high-G;
//     authority and full-throttle acceleration by speed, per aircraft (weak jet, Su-35);
//   - point-mass flight path (dev/compare/fit_airframe.py): the velocity follows the nose with
//     a 0.2 s lag, gravity, speed.
// Scenes: captures, target switches while rolling, chained captures, holds, scripted pursuits,
// and recorded enemies (one, several, a boss) flying their recorded paths, met from a
// standard start (dev/compare/scenes.txt).
//
//   dev\compare\compare.cmd                      build, run, open the viewer
//   harness.exe variant="label:pitch_kd=0.1;level_per_deg=10" ...   extra builds of ours
//
// Writes dev/compare/scorecard.txt (kept in git: a diff shows what a change did) and
// dev/compare/results/*.js for dev/compare/index.html.
#include "controllers/ours.h"
#include "controllers/upstream.h"
#include "controllers/pw5.h"
#include <shellapi.h>
#pragma comment(lib, "shell32.lib")
#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <sstream>
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
    // Authority by speed, per 100 m/s from 0 (interpolated): factors on the pull and push
    // and on the roll acceleration and limit. From full-stick p90 rates in each speed band.
    std::array<float, 9> pull_by_speed{1, 1, 1, 1, 1, 1, 1, 1, 1};
    std::array<float, 9> roll_by_speed{1, 1, 1, 1, 1, 1, 1, 1, 1};
    // Full throttle: dV/dt + g sin(climb) by speed (m/s^2, per 100 m/s, nz < 3, logged).
    std::array<float, 9> thrust_by_speed{43, 43, 35, 29, 22, 8.7f, 3.6f, -3, -10};
};
float by_speed(const std::array<float, 9>& table, float speed) {
    const float x = std::clamp(speed / 100.0f, 0.0f, 8.0f);
    const int i = std::min(int(x), 7);
    return table[i] + (table[i + 1] - table[i]) * (x - i);
}
// The weaker aircraft flown 2026-10-04/05 (game clock): full pull ~43 deg/s p90 from 200 to
// 500 m/s, falling to 6-10 at 600-700 m/s; rolls ~105 deg/s, 55-73 above 500 m/s.
constexpr std::array<float, 9> weak_pull{0.88f, 0.88f, 1.0f, 1.0f, 0.95f, 0.75f, 0.2f, 0.15f, 0.1f};
constexpr std::array<float, 9> weak_roll{0.85f, 0.85f, 1.0f, 0.97f, 0.7f, 0.55f, 0.62f, 0.58f, 0.55f};
// Su-35 (LADON mission, 2026-10-05): about the same at low speed, but keeps 26-28 deg/s of
// pull at 600-800 m/s and rolls 80-100 deg/s up to 700 m/s.
constexpr std::array<float, 9> su35_pull{0.95f, 0.95f, 1.0f, 0.8f, 0.82f, 0.65f, 0.66f, 0.68f, 0.42f};
constexpr std::array<float, 9> su35_roll{0.72f, 0.72f, 0.92f, 0.92f, 1.0f, 0.94f, 0.85f, 0.68f, 0.73f};
// Full throttle: the weak aircraft tops out near 640 m/s, the Su-35 near 795 m/s.
constexpr std::array<float, 9> weak_thrust{43, 43, 35, 29, 22, 8.7f, 3.6f, -3, -10};
constexpr std::array<float, 9> su35_thrust{62, 62, 65, 45, 42, 33, 15, 5.2f, -0.5f};
// Fitted on the game clock (see tests/flight_sim.cpp `measured`, fit_airframe.py): full pull
// ~50 deg/s at 200-500 m/s, x1.4-1.6 in a high-G turn; rolls top out near 105 deg/s.
constexpr Plant measured{"measured", "实测机体",
    0.12f, 0.20f, 0.10f, 0.6f, 50, 25, 0.3f, 1.5f,
    0.14f, 0.25f, 220, 1.3f, 105, 0.6f,
    0.15f, 5.4f, 0.35f, 1.5f,
    0.20f, 187, 0.0825f, 0.4f, weak_pull, weak_roll, weak_thrust};
constexpr Plant su35{"su35", "Su-35",
    0.12f, 0.20f, 0.10f, 0.6f, 50, 25, 0.3f, 1.5f,
    0.14f, 0.25f, 220, 1.3f, 105, 0.6f,
    0.15f, 5.4f, 0.35f, 1.5f,
    0.20f, 187, 0.0825f, 0.4f, su35_pull, su35_roll, su35_thrust};
// Robustness: 30% more delay and smoothing, less authority (a heavier aircraft, or the
// fit being optimistic). A controller tuned to the edge of the measured model shows it here.
constexpr Plant sluggish{"sluggish", "迟钝机体（延迟 ×1.3，权限 ×0.75）",
    0.16f, 0.26f, 0.13f, 0.8f, 38, 19, 0.3f, 1.5f,
    0.18f, 0.33f, 165, 1.3f, 80, 0.7f,
    0.2f, 4.0f, 0.45f, 1.5f,
    0.25f, 187, 0.0825f, 0.4f, weak_pull, weak_roll, weak_thrust};

float input_smooth(float f, float u, float dt, float build, float release) {
    const float tc = (u * f < 0 || std::abs(u) < std::abs(f)) ? release : build;
    return f + (u - f) * (1 - std::exp(-dt / tc));
}

bool trace_on = false;   // BENCH_TRACE=<scene>:<controller>: plant internals every 0.1 s
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
        const float speed_now = len(vel), pitch_k = by_speed(m.pull_by_speed, speed_now), roll_k = by_speed(m.roll_by_speed, speed_now);
        const float pull = m.pitch_pull * pitch_k * (high_g ? m.highg_pull : 1.0f);
        const float steady = (sq >= 0 ? pull * aq : -m.pitch_push * pitch_k * aq) - m.pitch_gravity * b.u.z;
        q += dt * (steady - q) / m.pitch_tau;
        if (trace_on) std::printf("  sq %.2f aq %.2f k %.2f hg %d steady %.1f q %.1f speed %.0f\n", sq, aq, pitch_k, high_g ? 1 : 0, steady, q, speed_now);
        p = std::clamp(p + dt * (std::copysign(m.roll_accel * roll_k * std::pow(std::min(std::abs(sr), 1.0f), m.roll_curve), sr) - p / m.roll_tau),
                       -m.roll_max * roll_k, m.roll_max * roll_k);
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
        else dv = (1 - throttle) * m.cruise_rate * (m.cruise - speed) + throttle * by_speed(m.thrust_by_speed, speed);
        dv -= m.turn_drag * std::max(0.0f, nz - 1);
        const V3 a = lift + gravity + vh * dv;
        acc = a;
        vel = vel + a * dt;
        if (len(vel) < 60) vel = unit(vel) * 60;   // no stall model: the game keeps aircraft flying
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


struct Pose { float t; V3 pos; float pitch, yaw, roll; };
Pose pose_at(const std::vector<Pose>& path, float t) {
    if (path.empty()) return {};
    if (t <= path.front().t) return path.front();
    for (size_t i = 1; i < path.size(); ++i) if (path[i].t >= t) {
        const Pose& a = path[i - 1]; const Pose& b = path[i];
        const float k = (t - a.t) / std::max(b.t - a.t, 1e-3f);
        auto mix = [k](float x, float y) { return x + std::remainder(y - x, 360.0f) * k; };
        return {t, a.pos + (b.pos - a.pos) * k, a.pitch + (b.pitch - a.pitch) * k, mix(a.yaw, b.yaw), mix(a.roll, b.roll)};
    }
    return path.back();
}
// Where a gun must point from `from` to hit an aircraft flying `path`: its position after
// the bullet's flight (~1000 m/s), the way a lead indicator shows it.
V3 lead_point(const std::vector<Pose>& path, float t, V3 from) {
    const Pose e = pose_at(path, t);
    const V3 vel = (pose_at(path, t + 0.1f).pos - pose_at(path, t - 0.1f).pos) * 5.0f;
    return unit(e.pos + vel * (len(e.pos - from) / 1000.0f) - from);
}
// Recorded enemies carry only a heading (the actor's pitch and roll read 0): attitude is
// rebuilt from the path, the nose along the velocity and the bank from the turn.
void orient_from_path(std::vector<Pose>& path) {
    const size_t n = path.size();
    if (n < 3) return;
    std::vector<V3> vel(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t a = i ? i - 1 : 0, b = std::min(i + 1, n - 1);
        vel[i] = (path[b].pos - path[a].pos) * (1.0f / std::max(path[b].t - path[a].t, 1e-3f));
    }
    for (size_t i = 0; i < n; ++i) {
        if (len(vel[i]) < 5) continue;
        const V3 f = unit(vel[i]);
        const size_t a = i ? i - 1 : 0, b = std::min(i + 1, n - 1);
        const V3 acc = (vel[b] - vel[a]) * (1.0f / std::max(path[b].t - path[a].t, 1e-3f));
        const V3 lift = acc - f * dot(acc, f) + V3{0, 0, g0};   // what the wings must supply
        path[i].pitch = std::asin(std::clamp(f.z, -1.0f, 1.0f)) / rad;
        path[i].yaw = std::atan2(f.y, f.x) / rad;
        const V3 right{-std::sin(path[i].yaw * rad), std::cos(path[i].yaw * rad), 0}, up = cross(f, right);
        path[i].roll = std::atan2(dot(lift, right), dot(lift, up)) / rad;
    }
}

// ---------------------------------------------------------------- recorded enemies
// Enemies cut from recordings (extract_tracks.py: one enemy of a gun attack; extract_battles.py:
// every enemy of a battle). Only the enemies are used: they fly their recorded paths and do
// not react to the simulated aircraft, so every controller meets exactly the same enemies.
// last: the end of the recorded path (for one the player shot down, when that happened).
// boss: a big aircraft that is not brought down here (LADON); the score is how well it is
// kept in the sights.
struct Foe { std::string cls; float first = 0, last = 0; bool elite = false, boss = false; std::vector<Pose> path; };
struct Encounter {
    std::vector<Foe> foes;
    bool guns_only = false;   // a single enemy: gun tracking for the whole path (never brought down)
    // the standard start, the same for every controller and condition (see standard_start)
    V3 start; float pitch = 0, yaw = 0, speed = 250;
};
bool elite_class(const std::string& cls) {
    std::string c = cls; for (char& ch : c) ch = char(std::tolower(static_cast<unsigned char>(ch)));
    return c.find("shadow") != std::string::npos || c.find("named") != std::string::npos ||
           c.find("boss") != std::string::npos || c.find("_ladn") != std::string::npos;
}
// The player starts behind the enemy nearest the middle of those flying at the start (1.5 km;
// a single enemy, fought with the gun, 1 km),
// on its heading (climb or dive limited to 20 deg), at its speed (200-650 m/s), no lower
// than 600 m. Not where the player happened to be in the recording: the start must not carry
// one flight's luck into every controller's run.
void standard_start(Encounter& e, float behind) {
    float t0 = 1e9f; for (const Foe& f : e.foes) t0 = std::min(t0, f.first);
    std::vector<int> early;
    for (int i = 0; i < int(e.foes.size()); ++i) if (e.foes[i].first <= t0 + 0.5f) early.push_back(i);
    V3 mid{};
    for (int i : early) mid = mid + pose_at(e.foes[i].path, e.foes[i].first).pos * (1.0f / early.size());
    int ref = early[0];
    for (int i : early) if (len(pose_at(e.foes[i].path, e.foes[i].first).pos - mid) < len(pose_at(e.foes[ref].path, e.foes[ref].first).pos - mid)) ref = i;
    const Foe& f = e.foes[ref];
    const float t = std::min(f.first + 0.5f, f.last);
    const V3 p = pose_at(f.path, t).pos, v = (pose_at(f.path, t + 0.5f).pos - pose_at(f.path, t - 0.5f).pos);
    const float speed = len(v);
    const V3 dir = speed > 15 ? unit(v) : V3{1, 0, 0};
    e.pitch = std::clamp(std::asin(std::clamp(dir.z, -1.0f, 1.0f)) / rad, -20.0f, 20.0f);
    e.yaw = std::atan2(dir.y, dir.x) / rad;
    e.start = p - direction(e.pitch, e.yaw) * behind;
    e.start.z = std::max(e.start.z, 600.0f);
    e.speed = std::clamp(speed, 200.0f, 900.0f);
}
// tracks/<file>: "enemy t x y z pitch yaw roll" (one enemy; its class in the header line)
// battles/<file>: "enemy i class first last down elite by_player boss" and "e i t x y z p y r"
std::shared_ptr<Encounter> load_encounter(const std::string& path, bool single) {
    std::ifstream in(path);
    if (!in) return nullptr;
    auto e = std::make_shared<Encounter>();
    for (std::string line; std::getline(in, line);) {
        std::istringstream ss(line);
        std::string tag; ss >> tag;
        if (single) {
            if (tag == "#" && line.find("enemy ") != std::string::npos && e->foes.empty()) {
                Foe f; const auto a = line.find("enemy ") + 6, b = line.find(',', a);
                f.cls = line.substr(a, b == std::string::npos ? std::string::npos : b - a);
                e->foes.push_back(f);
            } else if (tag == "enemy") {
                if (e->foes.empty()) e->foes.push_back(Foe{});
                Pose p; ss >> p.t >> p.pos.x >> p.pos.y >> p.pos.z >> p.pitch >> p.yaw >> p.roll; e->foes[0].path.push_back(p);
            }
        } else if (tag == "enemy") {
            size_t i; Foe f; int down = 0, elite = 0, by_player = 0, boss = 0;
            ss >> i >> f.cls >> f.first >> f.last >> down >> elite >> by_player >> boss;
            f.elite = elite != 0; f.boss = boss != 0;
            if (e->foes.size() <= i) e->foes.resize(i + 1);
            e->foes[i] = f;
        } else if (tag == "e") {
            size_t i; Pose p; ss >> i >> p.t >> p.pos.x >> p.pos.y >> p.pos.z >> p.pitch >> p.yaw >> p.roll;
            if (i < e->foes.size()) e->foes[i].path.push_back(p);
        }
    }
    e->foes.erase(std::remove_if(e->foes.begin(), e->foes.end(), [](const Foe& f) { return f.path.size() < 5; }), e->foes.end());
    if (e->foes.empty()) return nullptr;
    e->guns_only = single;
    for (Foe& f : e->foes) {
        orient_from_path(f.path);
        if (single) { f.first = f.path.front().t; f.last = f.path.back().t; f.elite = elite_class(f.cls); }
    }
    // a boss (supersonic, ECM: lock only within 1 km) is met 800 m behind at its own speed
    const bool boss = std::any_of(e->foes.begin(), e->foes.end(), [](const Foe& f) { return f.boss; });
    standard_start(*e, single ? 1000.0f : boss ? 800.0f : 1500.0f);
    return e;
}

// ---------------------------------------------------------------- scenarios
enum Kind { Capture, Switch, Chain, Hold, Pursuit, Single, Multi, Boss };
const Kind kinds[] = {Capture, Switch, Chain, Hold, Pursuit, Single, Multi, Boss};
const char* kind_name(Kind k) {
    return k == Capture ? "capture" : k == Switch ? "switch" : k == Chain ? "chain" : k == Hold ? "hold" :
           k == Pursuit ? "pursuit" : k == Single ? "single" : k == Multi ? "multi" : "boss";
}
bool encounter_kind(Kind k) { return k == Single || k == Multi || k == Boss; }
struct Scenario {
    std::string id, title, note;
    Kind kind;
    float seconds = 8;
    float start_roll = 0;
    float start_pitch = 0, start_p = 0;                 // nose pitch and roll rate (deg/s) at the start
    V3 first{}, next{}; float switch_at = -1;          // capture / chain
    std::function<V3(float)> aim;                        // hold: world aim over time
    std::vector<Segment> program;                        // pursuit: enemy program
    V3 enemy_offset{}; float enemy_yaw = 0, enemy_speed = 220;   // enemy start, relative to us (m, deg)
    std::shared_ptr<Encounter> encounter;                // recorded enemies
};
// The scenes, named "<category> · <what>" with ids "<category>_<nn>". Scripted ones are defined
// here; those with recorded enemies are listed in dev/compare/scenes.txt.
std::vector<Scenario> scenarios(const std::string& dir) {
    std::vector<Scenario> s;
    auto add = [&](Scenario c) { s.push_back(c); };
    // Captures: the mouse moved to a direction and held.
    auto cap = [&](std::string id, std::string title, V3 dir, float roll = 0, std::string note = "") {
        Scenario c; c.id = id; c.title = "捕获 · " + title; c.kind = Capture; c.first = unit(dir); c.start_roll = roll; c.note = note; add(c);
    };
    cap("cap_01", "右 3°", direction(0, 3), 0, "小角度修正");
    cap("cap_02", "右 10°", direction(0, 10));
    cap("cap_03", "右 90°", direction(0, 90));
    cap("cap_04", "右后 150°", direction(0, 150), 0, "大角度：方向选择");
    cap("cap_05", "上 20°", direction(20, 0));
    cap("cap_06", "下 15°", direction(-15, 0), 0, "小幅下压：不应翻成倒飞");
    cap("cap_07", "右 60° 低 12°", direction(-12, 60), 0, "目标在地平线下方的转弯");
    cap("cap_08", "倒飞起始右 15°", direction(0, 15), 180, "推杆与拉杆距离相近");
    // A new target picked while the aircraft is still rolling from the last one (switches
    // against the roll cost ~1.5 s in recorded battles). Nose 30 deg up, 60 deg left bank,
    // rolling right at 45 deg/s; the target 35 deg off at a clock position around the nose.
    auto sw = [&](std::string id, std::string title, int clock) {
        const Frame3 b0 = basis(30, 0, -60);
        const float a = 35 * rad, c = clock * rad;
        Scenario x; x.kind = Switch; x.start_pitch = 30; x.start_roll = -60; x.start_p = 45;
        x.first = unit(b0.f * std::cos(a) + (b0.u * std::cos(c) + b0.r * std::sin(c)) * std::sin(a));
        x.id = id; x.title = "换目标 · " + title;
        x.note = "坡度左 60°、机头上仰 30°、正以 45°/s 向右滚时，新目标出现在偏 35° 处";
        add(x);
    };
    sw("sw_01", "座舱方向 35°", 0);
    sw("sw_02", "顺滚转方向 90°", 90);
    sw("sw_03", "逆滚转方向 90°", -90);
    sw("sw_04", "机腹方向 35°", 175);
    auto chain = [&](std::string id, std::string title, V3 a, float at, V3 b, std::string note) {
        Scenario c; c.id = id; c.title = "连续 · " + title; c.kind = Chain; c.first = unit(a); c.next = unit(b); c.switch_at = at; c.seconds = 10; c.note = note; add(c);
    };
    chain("seq_01", "右 90° 途中转左下", direction(0, 90), 2.2f, direction(-30, 50), "到达附近时换目标，指标从切换时刻起算");
    chain("seq_02", "右 30° 途中转左 20°", direction(0, 30), 1.0f, direction(0, -20), "左右反向，指标从切换时刻起算");
    auto hold = [&](std::string id, std::string title, std::function<V3(float)> aim, std::string note) {
        Scenario c; c.id = id; c.title = "保持 · " + title; c.kind = Hold; c.seconds = 10; c.aim = aim; c.note = note; add(c);
    };
    hold("hold_01", "平飞", [](float) { return direction(0, 0); }, "鼠标不动，只有角速度测量噪声：晃不晃");
    hold("hold_02", "鼠标微调", [](float t) {
        float p = 0, y = 2;
        for (int k = 1; k <= int(t / 0.7f); ++k) {
            const unsigned h = static_cast<unsigned>(k) * 2654435761u;
            const float a = (h % 3600) * 0.1f * rad;
            p += std::sin(a); y += std::cos(a);
        }
        return direction(p, y);
    }, "已对准后每 0.7 s 移动鼠标约 1°（实测会引起 ±20° 摇翼）");
    hold("hold_03", "慢拖鼠标", [](float t) { return direction(0, 2 + 4 * t); }, "鼠标匀速平移 4°/s");
    auto pursuit = [&](std::string id, std::string title, V3 offset, float yaw, std::vector<Segment> program, std::string note, float seconds = 14) {
        Scenario c; c.id = id; c.title = "脚本追击 · " + title; c.kind = Pursuit; c.enemy_offset = offset; c.enemy_yaw = yaw; c.program = program;
        c.note = note; c.seconds = seconds; add(c);
    };
    pursuit("pur_01", "持续转弯", {700, 120, 0}, 0, {{1, 65, 0}, {99, 65, 16}}, "敌机 65° 坡度稳定盘旋（约 16°/s）");
    pursuit("pur_02", "急转脱离", {600, -80, 20}, 0, {{1.5f, 0, 0}, {2.2f, -80, 0}, {99, -80, 30}}, "1.5 s 后左压 80° 急拉 30°/s");
    pursuit("pur_03", "剪刀", {500, 0, 0}, 0,
            {{0.5f, 70, 0}, {3, 70, 20}, {3.5f, -70, 0}, {6, -70, 20}, {6.5f, 70, 0}, {9, 70, 20}, {9.5f, -70, 0}, {99, -70, 20}}, "每 3 s 反向一次");
    pursuit("pur_04", "小幅抖动", {450, 0, 0}, 0,
            {{1.0f, 25, 4}, {2.2f, -25, 4}, {3.0f, 20, 3}, {4.5f, -30, 5}, {5.5f, 25, 4}, {7, -20, 3}, {8, 30, 5}, {9.5f, -25, 4}, {11, 20, 3}, {99, -25, 4}},
            "敌机在机头前方左右小幅抖动（实测会满杆左右滚）");
    pursuit("pur_05", "对头后反转", {1800, 150, 60}, 180, {{99, 0, 0}}, "对头交错后掉头 180°（方向选择）", 16);
    // Recorded enemies, from the list: "<kind> <file> <id> <title>", kind single / multi / boss.
    std::ifstream list(dir + "/scenes.txt");
    for (std::string line; std::getline(list, line);) {
        std::istringstream ss(line);
        std::string kind, file, id;
        if (!(ss >> kind >> file >> id) || kind[0] == '#') continue;
        std::string title; std::getline(ss >> std::ws, title);
        Scenario c; c.id = id; c.title = title;
        c.kind = kind == "single" ? Single : kind == "multi" ? Multi : kind == "boss" ? Boss : Capture;
        if (c.kind == Capture) continue;
        const std::string path = dir + (c.kind == Single ? "/tracks/" : "/battles/") + file;
        c.encounter = load_encounter(path, c.kind == Single);
        if (!c.encounter) { std::fprintf(stderr, "scenes.txt: cannot read %s\n", path.c_str()); continue; }
        float end = 0; for (const Foe& f : c.encounter->foes) end = std::max(end, f.last);
        c.seconds = std::floor(end * 4) / 4;
        int bosses = 0, elites = 0; for (const Foe& f : c.encounter->foes) { bosses += f.boss; elites += f.elite; }
        char note[200];
        if (c.kind == Single) std::snprintf(note, sizeof(note), "敌机轨迹取自录像（%s）；我方从敌机后方 1 km 出发，全程用机炮跟踪（不计击落）。", elites ? "精英" : "普通");
        else std::snprintf(note, sizeof(note), "敌机 %d 架（精英 %d、头目 %d），轨迹取自录像；我方从敌机后方 %s 的标准位置出发。",
                           int(c.encounter->foes.size()), elites, bosses, bosses ? "800 m（取头目当时的速度）" : "1.5 km");
        c.note = note;
        add(c);
    }
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
    // encounters
    int kills = -1, elite_kills = 0, foes = 0, gun_kills = 0, missiles = 0;
    float first_kill = -1, on_lead = -1, close_time = 0, to_shot = -1;
    float lock_frac = -1;   // share of the time the target was within missile lock (15 deg, 0.3-2.5 km; a boss 1 km)
    float crashed = -1;     // time the aircraft went below 50 m (the ground is near 0-200 m)
    float clear = -1;       // time the last enemy came down (the run ends there)
};
struct Sample { float v[28]; int n; };
struct Run { Metrics m; std::vector<Sample> frames; std::vector<std::pair<float, int>> kills; };

struct Scorer {
    const Scenario& sc;
    Metrics& m;
    float start_t = 0, min_angle = 180, held = 0, held_level = 0, last_q = 0, last_r = 0;
    int last_sign = 0;
    double sum_angle = 0, sum_angle2 = 0, sum_bank2 = 0, track_time = 0, in2 = 0, in5 = 0;
    double sat_q = 0, sat_r = 0, effort = 0, total = 0;
    float track_from;
    Scorer(const Scenario& s, Metrics& out) : sc(s), m(out) {
        track_from = sc.kind == Pursuit ? 3.0f : sc.kind == Hold || encounter_kind(sc.kind) ? 1.0f : 1e9f;
    }
    void restart(float t) {   // chained target: the capture metrics start again
        start_t = t; m.to5 = m.to2 = m.settle = m.level = -1; m.overshoot = 0; m.reversals = 0;
        min_angle = 180; held = held_level = 0; last_sign = 0;
    }
    void add(float t, float dt, float angle, float bank, float roll_rate, Stick u, float nz) {
        const float tt = t - start_t;
        if (t > 1.0f) {   // stick reversals between beyond +0.3 and beyond -0.3
            if (std::abs(u.pitch) > 0.3f) { const float sg = u.pitch > 0 ? 1.0f : -1.0f; if (last_q * sg < 0) ++m.chatter; last_q = sg; }
            if (std::abs(u.roll) > 0.3f) { const float sg = u.roll > 0 ? 1.0f : -1.0f; if (last_r * sg < 0) ++m.chatter; last_r = sg; }
        }
        if (std::abs(roll_rate) > 15) { const int sg = roll_rate > 0 ? 1 : -1; if (last_sign && sg != last_sign) ++m.reversals; last_sign = sg; }
        if (std::abs(bank) > 100) m.inverted += dt;
        m.peak_nz = std::max(m.peak_nz, nz);
        sat_q += (std::abs(u.pitch) > 0.95f) * dt; sat_r += (std::abs(u.roll) > 0.95f) * dt; effort += std::abs(u.roll) * dt; total += dt;
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
    }
    void finish(float speed_loss, float alt_change) {
        m.speed_loss = speed_loss; m.alt_change = alt_change;
        m.pitch_sat = float(sat_q / total); m.roll_sat = float(sat_r / total); m.roll_effort = float(effort / total);
        if (track_time > 0) {
            m.mean_angle = float(sum_angle / track_time); m.rms_angle = float(std::sqrt(sum_angle2 / track_time));
            m.bank_rms = float(std::sqrt(sum_bank2 / track_time));
            m.within2 = float(in2 / track_time); m.within5 = float(in5 / track_time);
        }
        const bool started_inverted = std::abs(sc.start_roll) > 100;
        const float span = sc.kind == Chain ? sc.seconds - sc.switch_at : sc.seconds;
        // Crashed: the rest of the scene counts as completely off target.
        if (m.crashed >= 0 && track_time > 0) {
            const float tracked_span = std::max(span - track_from, 1.0f), flown = float(track_time), k = std::min(flown / tracked_span, 1.0f);
            m.mean_angle = (m.mean_angle * flown + 90 * (tracked_span - flown)) / tracked_span;
            m.within2 *= k; m.within5 *= k;
            if (m.lock_frac >= 0) m.lock_frac *= k;
            if (m.on_lead >= 0) m.on_lead *= k;
        }
        // Long scenes count roll reversals and stick chatter per minute.
        const float minutes = std::max(span / 60.0f, 0.25f);
        const float rev_rate = m.reversals / minutes, chatter_rate = m.chatter / minutes;
        // One number per scenario, lower is better (rough weights; the page shows the parts).
        if (sc.kind == Capture || sc.kind == Switch || sc.kind == Chain)
            m.cost = (m.to2 < 0 ? span : m.to2) + 0.5f * (m.settle < 0 ? span : m.settle) + 2 * std::max(0.0f, m.overshoot - 1.5f) +
                     0.5f * std::max(0, m.reversals - 1) + 0.2f * m.chatter + (started_inverted ? 0 : 0.5f * m.inverted);
        else if (sc.kind == Hold)
            m.cost = m.rms_angle + 0.05f * m.bank_rms + 0.3f * m.reversals + 0.1f * m.chatter;
        else if (sc.kind == Pursuit)
            m.cost = 0.5f * m.mean_angle + 3 * (1 - m.within5) + 0.2f * m.reversals + 0.05f * m.chatter;
        else if (sc.kind == Boss)   // kept in missile lock, on the gun lead point, the nose near the aim
            m.cost = 10 * (1 - std::max(m.lock_frac, 0.0f)) + 10 * (1 - std::max(m.on_lead, 0.0f)) + 0.2f * m.mean_angle +
                     0.05f * rev_rate + 0.01f * chatter_rate;
        else if (sc.kind == Single)   // gun tracking of one enemy: on the lead point, steadily
            m.cost = 0.5f * m.mean_angle + 2 * (1 - m.within5) + 2 * (1 - m.within2) + 0.05f * rev_rate + 0.01f * chatter_rate;
        else   // enemies brought down, how soon all of them, how quickly a picked enemy is fired at, gun aim
            m.cost = 20 * (1 - float(std::max(m.kills, 0)) / std::max(m.foes, 1)) + 10 * (m.clear < 0 ? 1 : m.clear / span) +
                     (m.to_shot < 0 ? 15 : m.to_shot) + 5 * (1 - std::max(m.on_lead, 0.0f)) + 0.2f * m.mean_angle;
        if (m.crashed >= 0) m.cost += 20;
    }
};

// Conditions: the aircraft model plus how the host senses body rates (on the game clock the
// filtered rates are quiet, 0.1-0.2 deg/s rms in level flight, logged). Every scene runs on
// every condition.
struct Condition { const char* id; const char* title; const Plant* plant; float white; float jitter; };
const Condition conditions[] = {
    {"measured", "差机体（实测）", &measured, 0.15f, 0.0f},
    {"su35", "Su-35 高机动（实测）", &su35, 0.15f, 0.0f},
    {"sluggish", "迟钝机体（差机体延迟 ×1.3、权限 ×0.75）", &sluggish, 0.15f, 0.0f},
};
constexpr int condition_count = int(sizeof(conditions) / sizeof(conditions[0]));

// The simulated player: the same decisions for every controller. Picks an enemy the way the
// game's target selection tends to (nearest the nose, distance counting), keeps it until it is
// down or over 5 km away, puts the mouse on its gun lead point, and works the throttle:
// throttle+brake (a high-G turn) while the enemy is more than 40 deg off the nose, full
// throttle while it is beyond 1.5 km (a boss: 800 m) and not closing at 50 m/s, brake inside
// 600 m (a boss: 400 m) when closing fast. Low over the ground it does not follow the aim into
// it: within ~4 s of 700 m the mouse is held at least level, higher the lower it gets.
// It fires:
//   missiles: the enemy within 15 deg of the nose at 300-2500 m (a boss with ECM: within 1 km)
//     for 0.8 s locks it; one missile every 1.5 s at most, ~700 m/s. It hits an ordinary enemy
//     always; an elite dodges unless launched within 1000 m and 5 deg, and takes two hits;
//     a boss's CIWS stops them (a boss is scored on the lock time, not brought down).
//   gun: within 1 km, a hit while the nose is within a wingspan (~9 m) of the lead point;
//     0.5 s of hits bring an enemy down, 1.5 s an elite.
struct Pilot {
    const Encounter& e;
    std::vector<char> killed;
    std::vector<float> hits;
    std::vector<int> missile_hits, on_the_way;
    std::vector<std::pair<float, int>> flying;   // missile impact time, enemy
    float lock = 0, next_launch = 0;
    int missiles = 0, gun_kills = 0;
    float picked_at = 0; bool shot = false;
    double to_shot = 0; int shots_timed = 0;
    int target = -1;
    V3 aim{1, 0, 0};
    float range = 0;
    std::vector<std::pair<float, int>> kill_log;
    double close_time = 0, on_lead = 0, lock_time = 0, target_time = 0;
    int elite_kills = 0;
    float throttle = 0, brake = 0, last_range = -1;
    explicit Pilot(const Encounter& enc, V3 forward)
        : e(enc), killed(enc.foes.size(), 0), hits(enc.foes.size(), 0), missile_hits(enc.foes.size(), 0),
          on_the_way(enc.foes.size(), 0), aim(forward) {}
    bool alive(int i, float t) const { const Foe& f = e.foes[i]; return !killed[i] && t >= f.first && t <= f.last; }
    int needed(int i) const { return e.foes[i].boss ? 1000000 : e.foes[i].elite ? 2 : 1; }
    bool all_down() const { for (char k : killed) if (!k) return false; return true; }
    float lock_range(int i) const { return e.foes[i].boss ? 1000.0f : 2500.0f; }
    bool doomed(int i) const { return missile_hits[i] + on_the_way[i] >= needed(i); }
    void kill(float t, int i) {
        killed[i] = 1; kill_log.push_back({t, i});
        elite_kills += e.foes[i].elite;
        if (target == i) target = -1;
    }
    void before(float t, float dt, const Aircraft& me) {
        if (target >= 0 && (!alive(target, t) || doomed(target) || len(pose_at(e.foes[target].path, t).pos - me.pos) > 5000)) target = -1;
        if (target < 0) {
            float best = 1e9f;
            for (int i = 0; i < int(e.foes.size()); ++i) {
                if (!alive(i, t) || doomed(i)) continue;
                const V3 to = pose_at(e.foes[i].path, t).pos - me.pos;
                const float d = len(to);
                if (d > 8000) continue;
                const float off = std::acos(std::clamp(dot(me.b.f, unit(to)), -1.0f, 1.0f)) / rad;
                const float c = off / 20 + d / 1000;
                if (c < best) { best = c; target = i; }
            }
            if (target >= 0) { picked_at = t; shot = false; }
        }
        if (target >= 0) {
            aim = lead_point(e.foes[target].path, t, me.pos);
            range = len(pose_at(e.foes[target].path, t).pos - me.pos);
        } else {   // nobody to attack: level flight on the current heading
            const V3 flat{me.b.f.x, me.b.f.y, 0};
            aim = len(flat) > 0.1f ? unit(flat) : V3{1, 0, 0};
        }
        // low over the ground: the aim no lower than a floor that rises as the ground nears
        const float low = std::min(me.pos.z, me.pos.z + me.vel.z * 4);
        if (low < 700) {
            const float floor_el = low < 200 ? 20.0f : low < 400 ? 10.0f : 0.0f;
            const float el = std::asin(std::clamp(aim.z, -1.0f, 1.0f)) / rad;
            if (el < floor_el) aim = direction(floor_el, std::atan2(aim.y, aim.x) / rad);
        }
        const float off = std::acos(std::clamp(dot(me.b.f, aim), -1.0f, 1.0f)) / rad;
        const float closing = target >= 0 && last_range >= 0 && dt > 0 ? (last_range - range) / dt : 0;
        last_range = target >= 0 ? range : -1;
        throttle = brake = 0;
        const bool boss = target >= 0 && e.foes[target].boss;
        const float far_range = boss ? 800.0f : 1500.0f, near_range = boss ? 400.0f : 600.0f;
        if (target >= 0 && off > 40 && len(me.vel) > 150) throttle = brake = 1;
        else if (target >= 0 && range > far_range && closing < 50) throttle = 1;
        else if (target >= 0 && range < near_range && closing > (boss ? 30.0f : 80.0f)) brake = 1;
    }
    void after(float t, float dt, const Aircraft& me) {
        for (size_t k = 0; k < flying.size();) {   // missiles arriving
            if (t < flying[k].first) { ++k; continue; }
            const int i = flying[k].second;
            --on_the_way[i];
            if (alive(i, t) && ++missile_hits[i] >= needed(i)) kill(t, i);
            flying.erase(flying.begin() + k);
        }
        if (target < 0) { lock = 0; return; }
        const V3 to = pose_at(e.foes[target].path, t).pos - me.pos;
        const float seen = std::acos(std::clamp(dot(me.b.f, unit(to)), -1.0f, 1.0f)) / rad;
        const bool in_lock = seen < 15 && range > 300 && range < lock_range(target);
        lock = in_lock ? lock + dt : 0;
        target_time += dt; if (in_lock) lock_time += dt;
        if (!e.guns_only && lock >= 0.8f && t >= next_launch && !doomed(target)) {
            if (!shot) { shot = true; to_shot += t - picked_at; ++shots_timed; }
            const Foe& f = e.foes[target];
            const bool good = !f.boss && (!f.elite || (range < 1000 && seen < 5));
            if (good) { flying.push_back({t + range / 700.0f, target}); ++on_the_way[target]; }
            ++missiles;
            next_launch = t + 1.5f;
            if (doomed(target)) { target = -1; lock = 0; return; }
        }
        if (range > 1000) return;
        const float off = std::acos(std::clamp(dot(me.b.f, aim), -1.0f, 1.0f)) / rad;
        close_time += dt;
        if (off < 2) on_lead += dt;
        if (off < std::max(0.5f, std::atan(9.0f / std::max(range, 1.0f)) / rad)) {
            if (!shot) { shot = true; to_shot += t - picked_at; ++shots_timed; }
            hits[target] += dt;
            if (!e.guns_only && !e.foes[target].boss && hits[target] >= (e.foes[target].elite ? 1.5f : 0.5f)) { ++gun_kills; kill(t, target); }
        }
    }
};

std::string trace_scene;   // BENCH_TRACE=<scene>:<controller>@<condition>: plant internals, first 4 s
Run fly(const Condition& cond, const Scenario& sc, Controller& ctl, bool keep_frames, const std::string& ctl_id = "") {
    const bool trace_this = !trace_scene.empty() && trace_scene == sc.id + ":" + ctl_id + "@" + cond.id;
    const Plant& plant = *cond.plant;
    const float dt = 1.0f / 80;   // logged median frame 12.5-13.3 ms
    const Encounter* enc = sc.encounter.get();
    const float speed0 = enc ? enc->speed : 220;
    Aircraft me;
    if (enc) me.start(enc->pitch, enc->yaw, 0, speed0, enc->start);
    else me.start(sc.start_pitch, 0, sc.start_roll, speed0, {0, 0, 3000});
    me.p = sc.start_p;
    std::unique_ptr<Pilot> pilot;
    if (enc) pilot = std::make_unique<Pilot>(*enc, me.b.f);
    Enemy enemy;
    if (sc.kind == Pursuit) {
        const Frame3 h = basis(0, 0, 0);
        enemy.pos = me.pos + h.f * sc.enemy_offset.x + h.r * sc.enemy_offset.y + h.u * sc.enemy_offset.z;
        enemy.b = basis(0, sc.enemy_yaw, 0); enemy.speed = sc.enemy_speed;
    }
    ctl.reset();
    std::mt19937 rng(1234);
    std::normal_distribution<float> unit_noise(0.0f, 1.0f);
    float fq = 0, fp = sc.start_p, fr = 0;
    Run run;
    Scorer score(sc, run.m);
    V3 target = sc.first;
    const int steps = int(sc.seconds / dt);
    const float alt0 = me.pos.z;
    const int every = sc.seconds > 200 ? 16 : enc ? 8 : 4;   // stored frames: 5, 10 or 20 a second
    for (int k = 0; k < steps; ++k) {
        const float t = k * dt;
        if (sc.kind == Chain && k == int(sc.switch_at / dt)) { target = sc.next; score.restart(t); }
        if (sc.kind == Hold) target = unit(sc.aim(t));
        if (sc.kind == Pursuit) { enemy.step(sc.program, t, dt); target = unit(enemy.pos - me.pos); }
        if (pilot) {
            pilot->before(t, dt, me);
            target = pilot->aim;
            if (pilot->target >= 0) {
                const Pose p = pose_at(enc->foes[pilot->target].path, t);
                enemy.pos = p.pos; enemy.b = basis(p.pitch, p.yaw, p.roll);
            }
        }
        float pitch, yaw, roll; euler(me.b, pitch, yaw, roll);
        // the host's view of this frame: measured frame time, rates from the pose change
        const float seen_dt = dt * std::clamp(1 + cond.jitter * unit_noise(rng), 0.5f, 1.6f);
        {
            const float scale = dt / seen_dt, a = 1 - std::exp(-12 * seen_dt);
            fq += (me.q * scale + cond.white * unit_noise(rng) - fq) * a;
            fp += (me.p * scale + cond.white * unit_noise(rng) - fp) * a;
            fr += (me.r * scale + cond.white * unit_noise(rng) - fr) * a;
        }
        const float throttle = pilot ? pilot->throttle : 0.0f, brake = pilot ? pilot->brake : 0.0f;
        Sense s{pitch, yaw, roll, {target.x, target.y, target.z}, fq, fr, fp, seen_dt, throttle, brake,
                {me.vel.x, me.vel.y, me.vel.z}, {me.acc.x, me.acc.y, me.acc.z}, me.pos.z};
        Stick u = ctl.step(s);
        u.pitch = std::clamp(u.pitch, -1.0f, 1.0f); u.roll = std::clamp(u.roll, -1.0f, 1.0f); u.yaw = std::clamp(u.yaw, -1.0f, 1.0f);
        trace_on = trace_this && k % 8 == 0 && t < 4;
        if (trace_on) std::printf("t %.2f stick %.2f thr %.0f brk %.0f pitch %.1f aim_el %.1f\n", t, u.pitch, throttle, brake, pitch, std::asin(target.z) / rad);
        me.step(plant, u, throttle, brake, dt);
        trace_on = false;
        if (me.pos.z < 50) { run.m.crashed = t; break; }

        const float angle = std::acos(std::clamp(dot(me.b.f, target), -1.0f, 1.0f)) / rad;
        score.add(t, dt, angle, bank_of(me.b), me.p, u, me.nz);
        if (pilot) pilot->after(t, dt, me);
        const bool cleared = pilot && !enc->guns_only && pilot->all_down();
        const bool has_enemy = sc.kind == Pursuit || (pilot && pilot->target >= 0);
        if (keep_frames && k % every == 0) {
            float ep = 0, ey = 0, er = 0;
            if (has_enemy) euler(enemy.b, ep, ey, er);
            Sample f{{t, me.pos.x, me.pos.y, me.pos.z, pitch, yaw, roll, target.x, target.y, target.z, angle,
                      me.q, me.p, me.r, u.pitch, u.roll, u.yaw, len(me.vel), me.nz,
                      std::atan2(-dot(me.vel, me.b.u), dot(me.vel, me.b.f)) / rad,
                      has_enemy ? enemy.pos.x : 0, has_enemy ? enemy.pos.y : 0, has_enemy ? enemy.pos.z : 0, ep, ey, er,
                      pilot ? float(pilot->target) : -1, pilot ? float(pilot->kill_log.size()) : 0},
                     enc ? 28 : sc.kind == Pursuit ? 26 : 20};
            run.frames.push_back(f);
        }
        if (cleared) { run.m.clear = t; break; }
    }
    if (pilot) {
        Metrics& m = run.m;
        m.foes = int(enc->foes.size()); m.kills = int(pilot->kill_log.size()); m.elite_kills = pilot->elite_kills;
        m.gun_kills = pilot->gun_kills; m.missiles = pilot->missiles;
        m.to_shot = pilot->shots_timed ? float(pilot->to_shot / pilot->shots_timed) : -1;
        m.lock_frac = pilot->target_time > 0 ? float(pilot->lock_time / pilot->target_time) : 0;
        m.first_kill = pilot->kill_log.empty() ? -1 : pilot->kill_log.front().first;
        m.close_time = float(pilot->close_time); m.on_lead = pilot->close_time > 0 ? float(pilot->on_lead / pilot->close_time) : 0;
        run.kills = pilot->kill_log;
    }
    score.finish(speed0 - len(me.vel), me.pos.z - alt0);
    return run;
}

// ---------------------------------------------------------------- registry and output
struct Entry { std::string id, label, source; std::function<std::unique_ptr<Controller>()> make; };

void json_metrics(std::string& out, const Metrics& m) {
    char b[900];
    std::snprintf(b, sizeof(b),
        "{\"to5\":%.2f,\"to2\":%.2f,\"settle\":%.2f,\"level\":%.2f,\"overshoot\":%.2f,\"reversals\":%d,\"chatter\":%d,"
        "\"inverted\":%.2f,\"speed_loss\":%.1f,\"alt_change\":%.0f,\"peak_nz\":%.1f,\"mean_angle\":%.2f,\"rms_angle\":%.2f,"
        "\"within2\":%.3f,\"within5\":%.3f,\"bank_rms\":%.1f,\"pitch_sat\":%.3f,\"roll_sat\":%.3f,\"roll_effort\":%.3f,\"cost\":%.2f,"
        "\"kills\":%d,\"elite_kills\":%d,\"foes\":%d,\"gun_kills\":%d,\"missiles\":%d,\"first_kill\":%.1f,\"on_lead\":%.3f,"
        "\"close_time\":%.1f,\"to_shot\":%.2f,\"crashed\":%.1f,\"lock_frac\":%.3f,\"clear\":%.1f}",
        m.to5, m.to2, m.settle, m.level, m.overshoot, m.reversals, m.chatter, m.inverted, m.speed_loss, m.alt_change, m.peak_nz,
        m.mean_angle, m.rms_angle, m.within2, m.within5, m.bank_rms, m.pitch_sat, m.roll_sat, m.roll_effort, m.cost,
        m.kills, m.elite_kills, m.foes, m.gun_kills, m.missiles, m.first_kill, m.on_lead, m.close_time, m.to_shot, m.crashed, m.lock_frac, m.clear);
    out += b;
}
std::string esc(const std::string& s) {
    std::string o;
    for (char c : s) { if (c == '"' || c == '\\') o += '\\'; o += c; }
    return o;
}
}  // namespace bench

int main() {
    using namespace bench;
    // Arguments as UTF-8 (variant labels may be Chinese; the narrow argv is the ANSI code page).
    int argc = 0;
    LPWSTR* wide = CommandLineToArgvW(GetCommandLineW(), &argc);
    std::vector<std::string> args(argc);
    for (int i = 0; i < argc; ++i) {
        const int n = WideCharToMultiByte(CP_UTF8, 0, wide[i], -1, nullptr, 0, nullptr, nullptr);
        args[i].resize(n > 0 ? n - 1 : 0);
        if (n > 1) WideCharToMultiByte(CP_UTF8, 0, wide[i], -1, args[i].data(), n, nullptr, nullptr);
    }
    LocalFree(wide);
    std::string out_dir = "dev/compare", config = "Payload/Game/Binaries/Win64/UE4SS/Mods/AC8MouseAim/config.ini";
    std::vector<std::pair<std::string, std::string>> variants;
    std::string only;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = args[i];
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
    if (const char* tr = std::getenv("BENCH_TRACE")) trace_scene = tr;
    const std::string scratch = out_dir + "/results/tuning.ini";
    CreateDirectoryA((out_dir + "/results").c_str(), nullptr);
    std::vector<Entry> entries;
    const flight::Tuning base = ours_tuning(config, "", scratch);
    entries.push_back({"ours", "本仓库 · 默认配置", "src/flight_logic.h + Payload config.ini", [base] { return std::make_unique<Ours>(base); }});
    // the configuration installed in the game (dev\Dev-Deploy remembers the game folder)
    {
        std::ifstream game(".dev-game-path");
        std::string root; std::getline(game, root);
        if (root.rfind("\xEF\xBB\xBF", 0) == 0) root.erase(0, 3);   // written by PowerShell with a BOM
        while (!root.empty() && (root.back() == '\r' || root.back() == ' ')) root.pop_back();
        const std::string installed = root + "\\Game\\Binaries\\Win64\\UE4SS\\Mods\\AC8MouseAim\\config.ini";
        if (!root.empty() && std::ifstream(installed)) {
            const flight::Tuning t = ours_tuning(installed, "", scratch);
            entries.push_back({"ours_game", "本仓库 · 游戏现配置", installed, [t] { return std::make_unique<Ours>(t); }});
        }
    }
    for (size_t i = 0; i < variants.size(); ++i) {
        const flight::Tuning t = ours_tuning(config, variants[i].second, scratch);
        entries.push_back({"ours_v" + std::to_string(i + 1), "本仓库 · " + variants[i].first, variants[i].second, [t] { return std::make_unique<Ours>(t); }});
    }
    entries.push_back({"upstream_legacy", "主仓库 0.2.30", "FletcherMiya 052cd6a controller_mode=0", [] { return std::make_unique<UpstreamLegacy>(); }});
    entries.push_back({"upstream_coord", "主仓库 0.2.35 协调制导", "FletcherMiya 052cd6a controller_mode=1", [] { return std::make_unique<UpstreamCoordinated>(); }});
    entries.push_back({"pw5", "xsd467 pw5", "xsd467 821f442 pw5-flight-control", [] { return std::make_unique<Pw5>(); }});

    const auto list = scenarios(out_dir);
    std::string index = "window.COMPARE_INDEX={\"controllers\":[";
    for (size_t i = 0; i < entries.size(); ++i)
        index += std::string(i ? "," : "") + "{\"id\":\"" + entries[i].id + "\",\"label\":\"" + esc(entries[i].label) + "\",\"source\":\"" + esc(entries[i].source) + "\"}";
    index += "],\"plants\":[";
    for (int p = 0; p < condition_count; ++p) index += std::string(p ? "," : "") + "{\"id\":\"" + conditions[p].id + "\",\"title\":\"" + conditions[p].title + "\"}";
    index += "],\"scenarios\":[";
    bool first_listed = true;
    for (const Scenario& sc : list) {
        if (!only.empty() && sc.id.find(only) == std::string::npos) continue;
        index += std::string(first_listed ? "" : ",") + "{\"id\":\"" + sc.id + "\",\"title\":\"" + esc(sc.title) + "\",\"note\":\"" + esc(sc.note) +
                 "\",\"kind\":\"" + kind_name(sc.kind) + "\",\"seconds\":" + std::to_string(sc.seconds) +
                 ",\"switch_at\":" + std::to_string(sc.switch_at) + "}";
        first_listed = false;
    }
    index += "],\"metrics\":{";

    // cost[scenario][condition][controller], for the scorecard
    std::map<std::string, std::vector<std::vector<float>>> cost;
    for (int p = 0; p < condition_count; ++p) {
        const Condition& cond = conditions[p];
        std::string data = std::string("(window.COMPARE_DATA=window.COMPARE_DATA||{})[\"") + cond.id + "\"]={";
        index += std::string(p ? "," : "") + "\"" + cond.id + "\":{";
        bool first_scenario = true;
        for (const Scenario& sc : list) {
            if (!only.empty() && sc.id.find(only) == std::string::npos) continue;
            data += std::string(first_scenario ? "" : ",") + "\"" + sc.id + "\":{";
            index += std::string(first_scenario ? "" : ",") + "\"" + sc.id + "\":{";
            first_scenario = false;
            auto& row = cost[sc.id];
            row.resize(condition_count);
            std::string kills_json = "{";
            for (size_t ci = 0; ci < entries.size(); ++ci) {
                auto ctl = entries[ci].make();
                const Run run = fly(cond, sc, *ctl, true, entries[ci].id);
                row[p].push_back(run.m.cost);
                const char* sep = ci ? "," : "";
                if (sc.encounter) {   // kill times per controller, for the viewer
                    kills_json += std::string(sep) + "\"" + entries[ci].id + "\":[";
                    for (size_t k = 0; k < run.kills.size(); ++k) {
                        char e[48]; std::snprintf(e, sizeof(e), "%s[%.2f,%d]", k ? "," : "", run.kills[k].first, run.kills[k].second);
                        kills_json += e;
                    }
                    kills_json += "]";
                }
                index += std::string(sep) + "\"" + entries[ci].id + "\":"; json_metrics(index, run.m);
                data += std::string(sep) + "\"" + entries[ci].id + "\":[";
                for (size_t k = 0; k < run.frames.size(); ++k) {
                    const Sample& f = run.frames[k];
                    data += k ? ",[" : "[";
                    for (int j = 0; j < f.n; ++j) {
                        char num[32];
                        const bool coarse = (j >= 1 && j <= 3) || (j >= 20 && j <= 22);   // positions
                        std::snprintf(num, sizeof(num), coarse ? "%s%.1f" : (j >= 7 && j <= 9) ? "%s%.4f" : j >= 26 ? "%s%.0f" : "%s%.2f", j ? "," : "", f.v[j]);
                        data += num;
                    }
                    data += "]";
                }
                data += "]";
            }
            if (sc.encounter) {
                // every enemy's path at 2 Hz and the kill times, for drawing the encounter
                data += ",\"_kills\":" + kills_json + "},\"_foes\":[";
                for (size_t i = 0; i < sc.encounter->foes.size(); ++i) {
                    const Foe& f = sc.encounter->foes[i];
                    char head[200];
                    std::snprintf(head, sizeof(head), "%s[\"%s\",%d,%.1f,%.1f,[", i ? "," : "", esc(f.cls).c_str(), f.boss ? 2 : f.elite ? 1 : 0, f.first, f.last);
                    data += head;
                    bool first_point = true;
                    for (float t = f.first; t <= f.last; t += 0.5f) {
                        const Pose e = pose_at(f.path, t);
                        char pt[96];
                        std::snprintf(pt, sizeof(pt), "%s[%.1f,%.0f,%.0f,%.0f]", first_point ? "" : ",", t, e.pos.x, e.pos.y, e.pos.z);
                        data += pt; first_point = false;
                    }
                    data += "]]";
                }
                data += "]";
            }
            data += "}";
            index += "}";
        }
        data += "};\n";
        index += "}";
        std::ofstream(out_dir + "/results/data_" + cond.id + ".js", std::ios::binary) << data;
    }
    index += "}};\n";
    std::ofstream(out_dir + "/results/index.js", std::ios::binary) << index;

    // Scorecard: per scene, one row per condition, one column per controller (the cost, lower
    // is better), then each controller's rank count and total per condition.
    std::string card = "Flight controller comparison (dev/compare): the same decisions against the same enemies.\n"
                       "Cost per scene (lower is better), one row per condition.\n\nControllers:\n";
    for (const auto& e : entries) card += "  " + e.id + "  " + e.label + "  [" + e.source + "]\n";
    char line[512];
    auto header = [&](const char* first) {
        std::snprintf(line, sizeof(line), "\n%-12s", first); card += line;
        for (const auto& e : entries) { std::snprintf(line, sizeof(line), " %16s", e.id.c_str()); card += line; }
        card += "\n";
    };
    header("scene");
    std::vector<std::vector<float>> total(condition_count, std::vector<float>(entries.size(), 0));
    std::vector<std::vector<int>> wins(condition_count, std::vector<int>(entries.size(), 0));
    for (const Scenario& sc : list) {
        if (!cost.count(sc.id)) continue;
        card += sc.id + "  " + sc.title + "\n";
        for (int p = 0; p < condition_count; ++p) {
            const auto& r = cost[sc.id][p];
            std::snprintf(line, sizeof(line), "  %-10s", conditions[p].id); card += line;
            const float best = *std::min_element(r.begin(), r.end());
            for (size_t ci = 0; ci < r.size(); ++ci) {
                std::snprintf(line, sizeof(line), " %15.2f%s", r[ci], r[ci] <= best + 1e-3f ? "*" : " "); card += line;
                total[p][ci] += r[ci];
                if (r[ci] <= best + 1e-3f) ++wins[p][ci];
            }
            card += "\n";
        }
    }
    header("total");
    for (int p = 0; p < condition_count; ++p) {
        std::snprintf(line, sizeof(line), "  %-10s", conditions[p].id); card += line;
        for (float v : total[p]) { std::snprintf(line, sizeof(line), " %16.1f", v); card += line; }
        card += "\n";
    }
    header("best (*)");
    for (int p = 0; p < condition_count; ++p) {
        std::snprintf(line, sizeof(line), "  %-10s", conditions[p].id); card += line;
        for (int v : wins[p]) { std::snprintf(line, sizeof(line), " %16d", v); card += line; }
        card += "\n";
    }
    std::fputs(card.c_str(), stdout);
    if (only.empty() && variants.empty()) std::ofstream(out_dir + "/scorecard.txt", std::ios::binary) << card;
    return 0;
}
