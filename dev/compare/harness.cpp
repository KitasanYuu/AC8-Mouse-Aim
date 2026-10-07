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
#include "controllers/pw11.h"
#include <shellapi.h>
#pragma comment(lib, "shell32.lib")
#include <cmath>
#include <cstdio>
#include <atomic>
#include <deque>
#include <mutex>
#include <thread>
#include <functional>
#include <algorithm>
#include <array>
#include <map>
#include <cstring>
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
    float highg_pull;                     // unused since 2026-10-06 (see highg_rate: the same for all)
    // flight path
    float aoa_lag;                        // s: AoA = lag x path rate
    float cruise, cruise_rate;            // unused: the neutral throttle is game_neutral below (until 2026-10-05)
    float turn_drag;                      // unused: the turn's cost is turn_cost below (until 2026-10-06)
    // Authority by speed, per 100 m/s from 0 (interpolated): factors on the pull and push
    // and on the roll acceleration and limit. From full-stick p90 rates in each speed band.
    std::array<float, 9> pull_by_speed{1, 1, 1, 1, 1, 1, 1, 1, 1};
    std::array<float, 9> roll_by_speed{1, 1, 1, 1, 1, 1, 1, 1, 1};
    // Full throttle: dV/dt + g sin(climb) by speed (m/s^2, per 100 m/s, nz < 3, logged).
    std::array<float, 9> thrust_by_speed{43, 43, 35, 29, 22, 8.7f, 3.6f, -3, -10};
    // Brake held (throttle not): dV/dt + g sin(climb) by speed (m/s^2, per 100 m/s, nz < 1.5,
    // logged). The FA-36's unless the aircraft's own was measured.
    std::array<float, 9> brake_by_speed{-25, -25, -59, -85, -122, -132, -126, -112, -83};
    float stall_speed = 62.5f;            // m/s: 225 km/h logged on the FA-36 (the game's SpeedStall per aircraft)
    // m/s, the game's SpeedMax: no scene starts or is held faster (scenes set from an FA-36's
    // flight or a boss's speed started an A-10C at 3190 km/h, its top speed being 1890)
    float speed_max = 850;
};
// Neutral throttle, the same for every aircraft logged (FA-36, Su-57, Su-35, 2026-10-05,
// dV/dt + g sin(climb), median, nz < 1.5): the speed holds near 150-200 m/s and above that
// falls 14 m/s^2 at 250, 23 at 350, 28 at 450, ~30 at 550-750. The bench had relaxed toward
// 187 m/s at 0.0825/s instead: 5 m/s^2 at 250 and 13 at 350, a glide that kept a fight's speed
// two to three times as long as the game does.
constexpr std::array<float, 9> game_neutral{2, 2, -14, -23, -28, -30, -30, -27, -15};
// Entry k is measured over the band k*100 to k*100+99 m/s, so it holds at the band's centre
// (k*100+50); read at the band's start, every table ran half a band early (the ADF-X02's ~0 at
// 800-899 m/s made 800 its top speed, while it held 845 behind LADON).
// What turning costs in speed (m/s^2), by the path's turn rate (deg/s): the same at every speed from
// 200 to 600 m/s (every recording, 2026-10-06: dV/dt + g sin(climb) less that of flying straight
// at the same speed and controls). Nothing to ~50 deg/s, then fast; held high-G costs from 20 deg/s
// on, more at the same rate. It was turn_drag (nz - 1) for every pull, from fits at low speed:
// a 35 g pull at 2000 km/h (50 deg/s, as the player's in the M28 opening) cost 140 m/s^2, where
// the recording shows 6-30, and a FA-36 diving in at 2300 km/h was down to 600 in 7 s (the player
// kept 1600). High-G held was -28 - 0.08 v on top of it.
constexpr float turn_rate_at[12] = {5, 15, 25, 35, 45, 55, 65, 75, 85, 95, 110, 135};
constexpr float turn_cost_free[12] = {0, 0, 0, -3, -4, -12, -42, -52, -60, -68, -82, -110};    // throttle, neutral, brake
// High-G held: a speed part and a turn-rate part, fitted together (additive, least squares over
// 74 speed x turn-rate cells, every recording with the nose within 30 deg of the path, i.e. no
// post-stall: FA-36, Su-57, ADF-X02, Su-35, MiG-21 alike; median residual 5 m/s^2, 2026-10-06).
// Was {0,0,-4,-11,-20,-33,-57,-73,-88,-100,-120,-155} with a straight part of -8 to -11 at 100-600
// m/s: a 52 deg/s high-G turn at 200-500 m/s lost ~39 m/s^2 where the recordings lose 50-53, and
// the speed came back sooner than the game gives it (thrust is right: see thrust_by_speed).
constexpr float turn_cost_highg[12] = {0, -3, -8, -18, -27, -38, -51, -65, -79, -92, -110, -140};
// by speed per 100 m/s from 50; below 100 m/s the game holds the speed up (its minimum, 150 km/h)
constexpr std::array<float, 9> highg_straight{20, -1, -20, -19, -18, -10, -7, -16, -17};
// High-G full-pull pitch rate (deg/s) by speed per 100 m/s from 50, the same for every aircraft:
// median with the full pull held 0.5 s and more (every recording, 34,000 frames; 2026-10-06). The
// game's high-G does not follow the aircraft's pitch table: the MiG-21bis, whose table is 0.6 of
// the FA-36's, turned 55 deg/s at 1000 km/h as the FA-36 did (gamepad steps), and the Su-57, with
// the FA-36's very table, a median 10-20 deg/s more in combat. It was each aircraft's full pull x
// 2.2: 110 deg/s for the FA-36, 66 for the MiG-21.
constexpr std::array<float, 9> highg_rate{50, 63, 63, 66, 67, 61, 57, 55, 47};
float turn_cost(const float (&cost)[12], float rate) {
    if (rate <= turn_rate_at[0]) return cost[0];
    for (int k = 1; k < 12; ++k)
        if (rate <= turn_rate_at[k]) return cost[k - 1] + (cost[k] - cost[k - 1]) * (rate - turn_rate_at[k - 1]) / (turn_rate_at[k] - turn_rate_at[k - 1]);
    return cost[11] + (cost[11] - cost[10]) * (rate - turn_rate_at[11]) / (turn_rate_at[11] - turn_rate_at[10]);
}
float by_speed(const std::array<float, 9>& table, float speed) {
    const float x = std::clamp((speed - 50.0f) / 100.0f, 0.0f, 8.0f);
    const int i = std::min(int(x), 7);
    return table[i] + (table[i + 1] - table[i]) * (x - i);
}
// MiG-21bis, flown 2026-10-04/05 (game clock; the recordings did not yet name the aircraft, the
// player did later): full pull ~43 deg/s p90 from 200 to
// 500 m/s, falling to 6-10 at 600-700 m/s; rolls ~105 deg/s, 55-73 above 500 m/s.
constexpr std::array<float, 9> weak_pull{0.88f, 0.88f, 1.0f, 1.0f, 0.95f, 0.75f, 0.2f, 0.15f, 0.1f};
constexpr std::array<float, 9> weak_roll{0.85f, 0.85f, 1.0f, 0.97f, 0.7f, 0.55f, 0.62f, 0.58f, 0.55f};
// Su-35 (LADON mission, 2026-10-05): about the same at low speed, but keeps 26-28 deg/s of
// pull at 600-800 m/s and rolls 80-100 deg/s up to 700 m/s.
constexpr std::array<float, 9> su35_pull{0.95f, 0.95f, 1.0f, 0.8f, 0.82f, 0.65f, 0.66f, 0.68f, 0.42f};
constexpr std::array<float, 9> su35_roll{0.72f, 0.72f, 0.92f, 0.92f, 1.0f, 0.94f, 0.85f, 0.68f, 0.73f};
// Full throttle: the MiG-21bis tops out near 640 m/s, the Su-35 near 795 m/s.
constexpr std::array<float, 9> weak_thrust{43, 43, 35, 29, 22, 8.7f, 3.6f, -3, -10};
constexpr std::array<float, 9> su35_thrust{62, 62, 65, 45, 42, 33, 15, 5.2f, -0.5f};
// Brake held, logged 2026-10-05 (m/s^2): the Su-35 sheds speed hardest, the Su-57 least.
constexpr std::array<float, 9> su35_brake{-32, -32, -99, -166, -173, -143, -169, -176, -176};
constexpr std::array<float, 9> su57_brake{-16, -24, -42, -59, -80, -106, -110, -110, -110};
// Fitted on the game clock (see tests/flight_sim.cpp `measured`, fit_airframe.py): full pull
// ~50 deg/s at 200-500 m/s, x1.4-1.6 in a high-G turn; rolls top out near 105 deg/s.
constexpr Plant mig21bis{"mig21bis", "MiG-21bis",
    0.12f, 0.20f, 0.10f, 0.6f, 50, 25, 0.3f, 1.5f,
    0.14f, 0.25f, 220, 1.3f, 105, 0.6f,
    0.15f, 5.4f, 0.35f, 1.5f,
    0.20f, 187, 0.0825f, 0.4f, weak_pull, weak_roll, weak_thrust};
constexpr Plant su35{"su35", "Su-35",
    0.12f, 0.20f, 0.10f, 0.6f, 50, 25, 0.3f, 1.5f,
    0.14f, 0.25f, 220, 1.3f, 105, 0.6f,
    0.15f, 5.4f, 0.35f, 1.5f,
    0.20f, 187, 0.0825f, 0.4f, su35_pull, su35_roll, su35_thrust, su35_brake};
// Su-57 (two Shadow-boss missions, 2026-10-05, attitude differences at 75 Hz, p90): full pull
// 50-53 deg/s at 100-500 m/s, 44 at 600, 33 at 700, 11 at 800; high-G (throttle and brake held)
// ~2.5 times that; full roll 110-116 deg/s at 100-400, 92-97 at 500-600, 78 at 700, 49 at 800;
// full throttle dV/dt 68 m/s^2 at 100, 45 at 400, 14 at 600, ~5 at 700-800.
constexpr std::array<float, 9> su57_pull{0.92f, 1.0f, 1.04f, 1.06f, 1.04f, 0.96f, 0.89f, 0.66f, 0.22f};
constexpr std::array<float, 9> su57_roll{0.54f, 1.05f, 1.04f, 1.1f, 1.05f, 0.93f, 0.88f, 0.75f, 0.46f};
constexpr std::array<float, 9> su57_thrust{60, 68, 62, 53, 45, 36, 14, 4.8f, 0};
constexpr Plant su57{"su57", "Su-57",
    0.12f, 0.20f, 0.10f, 0.6f, 50, 25, 0.3f, 1.5f,
    0.14f, 0.25f, 220, 1.3f, 105, 0.6f,
    0.15f, 5.4f, 0.35f, 2.5f,
    0.20f, 187, 0.0825f, 0.4f, su57_pull, su57_roll, su57_thrust, su57_brake};
// ADF-X02 (LADON missions, 2026-10-05, dev/compare/fit_speed_tables.py, p90): full pull 56 deg/s
// below 300 m/s, 38-39 at 300-500, 35 at 500, 23-24 at 600-800, 17 above 800; high-G about 2.7
// times that; full roll ~100 deg/s to 400, 87 at 400-600, 70 at 600-800, 60 above; full-throttle
// dV/dt 51 at 300, 37 at 400, 34 at 500, 15 at 600, 6 at 700, ~0 at 850: it keeps up with LADON's
// 850 m/s first phase, which the Su-35 and Su-57 (~800 m/s at most) cannot.
constexpr std::array<float, 9> adfx02_pull{1.1f, 1.12f, 1.13f, 0.78f, 0.77f, 0.7f, 0.47f, 0.47f, 0.34f};
constexpr std::array<float, 9> adfx02_roll{0.95f, 0.95f, 0.93f, 0.98f, 0.83f, 0.82f, 0.68f, 0.65f, 0.57f};
constexpr std::array<float, 9> adfx02_thrust{42, 42, 50, 51, 37, 34, 15.5f, 5.8f, -0.1f};
constexpr Plant adfx02{"adfx02", "ADF-X02",
    0.12f, 0.20f, 0.10f, 0.6f, 50, 25, 0.3f, 1.5f,
    0.14f, 0.25f, 220, 1.3f, 105, 0.6f,
    0.15f, 5.4f, 0.35f, 2.7f,
    0.20f, 187, 0.0825f, 0.4f, adfx02_pull, adfx02_roll, adfx02_thrust};
// FA-36 (Selene and LADON missions and M28, 2026-10-05, dev/compare/fit_speed_tables.py, p90):
// full pull 49-51 deg/s at 100-500 m/s, 43 at 500, 36 at 600, 28 at 700, 23 above 800; high-G
// about 2.1 times that; full roll 106-116 deg/s at 100-500, 99-106 at 500-700, 77-79 above;
// full-throttle dV/dt 68 at 100, 73 at 200, 59 at 300, 50 at 400, 38 at 500, 18 at 600, ~5 at
// 700-900: about twice the weak aircraft's. Below 100 m/s the frames are post-stall manoeuvres
// (an 88 deg/s pull, 12 m/s^2): the 100-199 band is used there.
constexpr std::array<float, 9> fa36_pull{1.02f, 1.02f, 1.02f, 1.02f, 0.98f, 0.85f, 0.72f, 0.57f, 0.46f};
constexpr std::array<float, 9> fa36_roll{1.01f, 1.01f, 1.08f, 1.1f, 1.1f, 1.0f, 0.94f, 0.73f, 0.76f};
constexpr std::array<float, 9> fa36_thrust{68, 68, 73, 59, 50, 38, 18, 4.5f, -7.7f};   // past 790 m/s (2850 km/h, its SpeedMax): slowing
constexpr Plant fa36{"fa36", "FA-36",
    0.12f, 0.20f, 0.10f, 0.6f, 50, 25, 0.3f, 1.5f,
    0.14f, 0.25f, 220, 1.3f, 105, 0.6f,
    0.15f, 5.4f, 0.35f, 2.1f,
    0.20f, 187, 0.0825f, 0.4f, fa36_pull, fa36_roll, fa36_thrust};

float input_smooth(float f, float u, float dt, float build, float release) {
    const float tc = (u * f < 0 || std::abs(u) < std::abs(f)) ? release : build;
    return f + (u - f) * (1 - std::exp(-dt / tc));
}

// Fights are gun only: missiles lock from far and almost anything on screen (the game's 40 deg,
// 2.85 km), so they told the controllers apart in nothing, and brought down 99.7% of the enemies
// (2026-10-06), with no turn limits or lost locks modelled. Off everywhere, the fixed orders of
// attack included; missiles=1 brings the old rules back, everywhere. A scene in scenes.txt can
// have missiles on its own: missiles=1 the old rules, missiles=long long-range missiles flown (see LongRange).
bool with_missiles = false;
// The pilot's ground floor: the share of the full pull counted on, and the reaction before it
// (seconds, plus the bank at 90 deg/s); BENCH_FLOOR=<share>,<react> to try others.
float floor_pull_share = 0.5f, floor_react = 2.0f;
// The share of the height the floor lets the mouse's path use (the low takeover acts on the
// aircraft's own path at all of it): at the whole, a path a little past the mouse's tripped the
// takeover, let go, and the mouse went back down to the limit, every 5 s (LADON's second phase).
float floor_margin = 0.6f;
// The player takes over low (BENCH_LOWTAKE=<m>,<deg>; 0 off): the present path going into the crash
// line before it could level out (by the floor's reckoning), or inverted (bank past 120) within
// low_take_m of it, the mouse at least low_take_deg above the horizon until the sinking stops.
bool low_take = true; float low_take_m = 300, low_take_deg = 25;
enum MissileKind { NoMissiles = 0, RuleMissiles = 1, LongRangeMissiles = 2 };
// A long-range missile, flown frame by frame instead of hitting by rule. Its figures were first
// taken from the game's long-range missile class (BP_plwp_laam_e0, read in game 2026-10-06; UE cm
// and cm/s converted), but it does not fly as the game's does: called a long-range missile, not by
// the game's name for it. It leaves at the aircraft's velocity plus 83 m/s,
// coasts 0.4 s (AccelerationDelayFromStart) and then gains 333 m/s^2 up to 1389 m/s; it does
// not steer for 0.4 s (HomingDelayFromStart), then 1.2 s at 20 deg/s at most
// (ReducedHomingDuration, ReducedHomingRotationAngle), then 195 deg/s (MaxRotationAngle),
// always at where the enemy is now (HomingForesightAmount 0: the game's missiles chase the
// tail). An enemy more than 117 deg off its heading (MaxHomingAngle) is lost for good; 25 s
// and it is gone (LifeTime). It hits when it passes within the enemy's size (no proximity fuse
// in the class: ProximityFuseRadius 0). Locked within 27.5 deg of the nose and 10 km
// (LockonAngle, LockonRange; held 0.8 s, as for the other missiles); two rails, each reloading
// 6.5 s after it fires (MaxLoadedCount 2, LoadTime). 12 aboard, the bench's own choice (the
// game's tables give the J-39E 26 or 14). So at close range it rarely turns in time: in its first
// 1.6 s it covers ~800 m nearly straight.
struct LongRange {
    static constexpr float ignition = 83.3f, accel = 333.3f, top = 1388.9f, accel_delay = 0.4f;
    static constexpr float homing_delay = 0.4f, reduced_for = 1.2f, reduced_rate = 20, rate = 195;
    static constexpr float max_homing = 117, life = 25, lock_angle = 27.5f, lock_range = 10000;
    static constexpr float reload = 6.5f;
    static constexpr int aboard = 12;
    // kept for the far ones (the player, 2026-10-06): not fired within 2 km, where the gun reaches
    // soon; fired at anything in reach, the 12 were gone in the first minutes
    static constexpr float min_range = 2000;
};
thread_local bool trace_on = false;   // BENCH_TRACE=<scene>:<controller>: plant internals every 0.1 s
struct Aircraft {
    Frame3 b;
    float q = 0, p = 0, r = 0, sq = 0, sr = 0;
    std::deque<float> dq, dr, dy;
    V3 pos, vel, acc;
    float nz = 1;
    float stalled = 0;   // s spent below the stall speed
    // Stall, as the game has it: below 225 km/h (62.5 m/s) the nose and the flight path fall
    // toward the ground whatever the stick. Fitted to 766 stalled frames logged on 2026-10-05:
    // beyond what the controls and gravity do, the nose fell a median 22 deg/s and the path
    // 25 deg/s (34 in all), the speed held near 60 until the nose was down and it built again.
    static constexpr float stall_nose = 22, stall_path = 25;   // the speed is the plant's stall_speed
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
        const float pull = high_g ? by_speed(highg_rate, speed_now) : m.pitch_pull * pitch_k;
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
        const float turn_rate = len(lift) / speed / rad;   // deg/s
        if (high_g) dv = by_speed(highg_straight, speed) + turn_cost(turn_cost_highg, turn_rate);
        else {
            if (brake > 0.5f) dv = by_speed(m.brake_by_speed, speed);   // -12 - 0.135 v until 2026-10-05
            else dv = throttle > 0.5f ? by_speed(m.thrust_by_speed, speed) : by_speed(game_neutral, speed);
            dv += turn_cost(turn_cost_free, turn_rate);
        }
        const V3 a = lift + gravity + vh * dv;
        acc = a;
        vel = vel + a * dt;
        if (len(vel) < m.stall_speed) {
            stalled += dt;
            // nose: toward straight down, as fast as it is still off it (none pointing down)
            const V3 down = V3{0, 0, -1} + b.f * b.f.z;
            if (len(down) > 1e-3f) rotate_body(b, stall_nose * dot(down, b.u), 0, stall_nose * dot(down, b.r), dt);
            const V3 vd = unit(vel), pd = V3{0, 0, -1} + vd * vd.z;
            if (len(pd) > 1e-3f) vel = turn(vd, unit(pd), stall_path * len(pd) * dt * rad) * len(vel);
        }
        if (len(vel) < 45) vel = unit(vel) * 45;   // the lowest logged in a stall: ~52
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


struct Pose { float t; V3 pos; float pitch, yaw, roll; float sweep = -1, fold = -1; };   // sweep, fold: a boss's variable wings (deg), -1 unknown
Pose pose_at(const std::vector<Pose>& path, float t) {
    if (path.empty()) return {};
    if (t <= path.front().t) return path.front();
    for (size_t i = 1; i < path.size(); ++i) if (path[i].t >= t) {
        const Pose& a = path[i - 1]; const Pose& b = path[i];
        const float span = std::max(b.t - a.t, 1e-3f), k = (t - a.t) / span;
        auto mix = [k](float x, float y) { return x + std::remainder(y - x, 360.0f) * k; };
        V3 pos = a.pos + (b.pos - a.pos) * k;
        // A gap in the recording (the enemy off the contact list: LADON for 9 s in its first phase,
        // back 3.5 km off the straight line): a curve meeting both ends at their own velocities,
        // not a straight line that turns sharply where the samples resume.
        if (span > 0.5f && i >= 2 && i + 1 < path.size()) {
            const Pose& a0 = path[i - 2]; const Pose& b1 = path[i + 1];
            if (a.t - a0.t < 0.5f && b1.t - b.t < 0.5f) {
                const V3 va = (a.pos - a0.pos) * (1 / std::max(a.t - a0.t, 1e-3f)), vb = (b1.pos - b.pos) * (1 / std::max(b1.t - b.t, 1e-3f));
                const float k2 = k * k, k3 = k2 * k;
                pos = a.pos * (2 * k3 - 3 * k2 + 1) + va * (span * (k3 - 2 * k2 + k)) + b.pos * (-2 * k3 + 3 * k2) + vb * (span * (k3 - k2));
            }
        }
        return {t, pos, a.pitch + (b.pitch - a.pitch) * k, mix(a.yaw, b.yaw), mix(a.roll, b.roll),
                a.sweep + (b.sweep - a.sweep) * k, a.fold + (b.fold - a.fold) * k};
    }
    return path.back();
}
// Where rounds must go from `from` to meet an aircraft flying `path`: where it will be after the
// rounds' flight (`bullet` m/s over the ground) if it keeps its present velocity, as seen up to
// now (it read 0.1 s of the path ahead until 2026-10-06). The game shows no such point: only where
// the rounds go (the gun marker, within 1 km); where the enemy will be is the player's judgement.
V3 lead_point(const std::vector<Pose>& path, float t, V3 from, float bullet = 1000) {
    const Pose e = pose_at(path, t);
    const V3 vel = (e.pos - pose_at(path, t - 0.2f).pos) * 5.0f;
    // the flight time to where it will be, not to where it is (it opens or closes the range meanwhile)
    float tof = len(e.pos - from) / std::max(bullet, 300.0f);
    for (int k = 0; k < 2; ++k) tof = len(e.pos + vel * tof - from) / std::max(bullet, 300.0f);
    return unit(e.pos + vel * tof + V3{0, 0, 0.5f * 9.8f * tof * tof} - from);   // aimed above by the rounds' fall
}
// Recorded enemies carry only a heading (the actor's pitch and roll read 0): attitude is
// rebuilt from the path, the nose along the velocity and the bank from the turn.
// Attitude for a recorded path. The recordings carry the game's own yaw and roll (contacts.lua
// read pitch under the wrong spelling until 2026-10-05, so it is 0 there): kept when present,
// pitch then from the climb angle. Without recorded rotation, all three from the path (yaw
// along it, roll from the lift a coordinated turn needs).
void orient_from_path(std::vector<Pose>& path) {
    const size_t n = path.size();
    if (n < 3) return;
    bool recorded = false;
    for (const Pose& p : path) if (std::abs(p.roll) > 0.5f || std::abs(p.pitch) > 0.5f) { recorded = true; break; }
    bool recorded_pitch = false;
    for (const Pose& p : path) if (std::abs(p.pitch) > 0.5f) { recorded_pitch = true; break; }
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
        if (recorded) {
            if (!recorded_pitch) path[i].pitch = std::asin(std::clamp(f.z, -1.0f, 1.0f)) / rad;
            continue;
        }
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
// ecm: missiles lock only within 1 km (LADON); escort: a boss's shield drone (Moon 11's uavn):
// while two or more stay within 150 m of their boss, each projects a field around itself and
// neither they nor the boss can be harmed (the player's account of the Expert battle)
// down: brought down in the recording (by the player or anyone): its path ends there
struct Foe { std::string cls; float first = 0, last = 0; bool elite = false, boss = false, ecm = false, escort = false, down = false; std::vector<Pose> path; };
struct Encounter {
    std::vector<Foe> foes;
    bool guns_only = false;   // a single enemy: gun tracking for the whole path (never brought down)
    // the standard start, the same for every controller and condition (see standard_start)
    V3 start; float pitch = 0, yaw = 0, roll = 0, speed = 250;
    // A fixed order of attack (enemy indices), the same for every controller: the simulated
    // player takes the best placed of the next three in it still flying (order_window) instead of
    // choosing among all by its own position. In a crowd (78 Tu-95s) choosing among all split the
    // controllers onto different targets within ~10 s, and the kill counts measured the luck of
    // the draw as much as the flying. The first in it alone (until 2026-10-06) was often behind
    // the simulated player or far off (the order is the player's, flown elsewhere), and with one
    // missile on its way the next was taken at once: three picks in a second, the last behind.
    std::vector<int> order;
    std::vector<int> game_order;   // the player's own selections in the recording, in order
    // target=game (scenes.txt): the enemy the player had locked at each moment (time, index; -1
    // none), followed instead of the pilot's own choice while one is locked. In LADON's first
    // phase the player went between the two (the second 56 s, the first 9 s); its own choice kept
    // the first all 235 s (a boss is never let go).
    std::vector<std::pair<float, int>> selections; bool follow_selection = false;
    bool has_player = false; V3 player_pos; float player_pitch = 0, player_yaw = 0, player_roll = 0, player_speed = 0;   // the recorded start
    int missiles = NoMissiles;   // this scene's missiles (scenes.txt missiles=1|long)
    // Below this the aircraft has hit the ground (m): 50, or 20 m under the lowest any recorded
    // enemy flew. The ground is unknown (0-200 m); over the sea LADON came down to 40 m and Moon
    // 11 to 43, and those following them at 45 m were counted crashed (2026-10-06).
    float crash_z = 50;
};
bool elite_class(const std::string& cls) {
    std::string c = cls; for (char& ch : c) ch = char(std::tolower(static_cast<unsigned char>(ch)));
    return c.find("shadow") != std::string::npos || c.find("named") != std::string::npos ||
           c.find("boss") != std::string::npos || c.find("_ladn") != std::string::npos || c.find("tewy") != std::string::npos ||
           c.find("toplak") != std::string::npos;
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
std::shared_ptr<Encounter> load_encounter(const std::string& path, bool single, float behind = -1, float from = 0) {
    std::ifstream in(path);
    if (!in) return nullptr;
    auto e = std::make_shared<Encounter>();
    struct Own { float t; V3 pos; float pitch, yaw, roll, speed; };
    std::vector<Own> own;   // the player's path, for a scene cut later than the file (from=)
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
            f.elite = elite != 0; f.boss = boss != 0; f.down = down != 0;
            f.ecm = f.boss && f.cls.find("_ladn") != std::string::npos;
            f.escort = f.cls.find("uavn") != std::string::npos;
            if (e->foes.size() <= i) e->foes.resize(i + 1);
            e->foes[i] = f;
        } else if (tag == "player" && !single) {   // "player x y z pitch yaw roll speed"
            if (ss >> e->player_pos.x >> e->player_pos.y >> e->player_pos.z >> e->player_pitch >> e->player_yaw >> e->player_roll >> e->player_speed)
                e->has_player = true;
        } else if (tag == "own" && !single) {   // "own t x y z p y r aim_p aim_y speed q r p sp sr sy selected ctl"
            // the player's attacks: an enemy counts once it was selected within 3 km (selections
            // passing over far ones while cycling targets are not attacks: they sent every
            // controller on 15 km chases), in the order they were first attacked
            float v[18]; int k = 0;
            while (k < 18 && ss >> v[k]) ++k;
            const int sel = k == 18 ? int(v[16]) : -1;
            if (k >= 10) own.push_back({v[0], V3{v[1], v[2], v[3]}, v[4], v[5], v[6], v[9]});
            if (k == 18 && (e->selections.empty() || e->selections.back().second != sel)) e->selections.push_back({v[0], sel});
            if (sel >= 0 && sel < int(e->foes.size()) && !e->foes[sel].path.empty() &&
                std::find(e->game_order.begin(), e->game_order.end(), sel) == e->game_order.end() &&
                len(pose_at(e->foes[sel].path, v[0]).pos - V3{v[1], v[2], v[3]}) < 3000)
                e->game_order.push_back(sel);
        } else if (tag == "e") {
            size_t i; Pose p; ss >> i >> p.t >> p.pos.x >> p.pos.y >> p.pos.z >> p.pitch >> p.yaw >> p.roll;
            if (!(ss >> p.sweep >> p.fold)) p.sweep = p.fold = -1;   // a boss's wings, when recorded
            if (i < e->foes.size()) e->foes[i].path.push_back(p);
        }
    }
    // from=<s> (scenes.txt): the scene starts that far into the file, the player as recorded then
    // (LADON's first phase from 61.8 s: right behind both at 2.6-2.9 km, after the head-on meeting
    // the recording has gaps of 12-15 s in)
    if (from > 0) {
        for (Foe& f : e->foes) {
            size_t k = 0; while (k < f.path.size() && f.path[k].t < from) ++k;
            f.path.erase(f.path.begin(), f.path.begin() + (k > 0 ? k - 1 : 0));   // one sample before, to interpolate from
            for (Pose& p : f.path) p.t -= from;
            f.first = std::max(f.first - from, 0.0f); f.last -= from;
        }
        for (Foe& f : e->foes) if (f.last <= 0) f.path.clear();   // over before the cut: dropped below
        int last_sel = -1; std::vector<std::pair<float, int>> sel;
        for (const auto& [ts, i] : e->selections) { if (ts <= from) last_sel = i; else sel.push_back({ts - from, i}); }
        sel.insert(sel.begin(), {0.0f, last_sel}); e->selections = sel;
        const auto o = std::find_if(own.begin(), own.end(), [from](const Own& x) { return x.t >= from; });
        if (o != own.end()) {
            e->has_player = true; e->player_pos = o->pos; e->player_pitch = o->pitch; e->player_yaw = o->yaw;
            e->player_roll = o->roll; e->player_speed = o->speed;
        }
    }
    {   // drop enemies with too few samples, keeping the player's order on the new indices
        std::vector<int> index(e->foes.size(), -1); int kept = 0;
        for (size_t i = 0; i < e->foes.size(); ++i) if (e->foes[i].path.size() >= 5) index[i] = kept++;
        std::vector<int> order;
        for (int i : e->game_order) if (index[i] >= 0) order.push_back(index[i]);
        e->game_order = order;
        for (auto& [ts, i] : e->selections) i = i >= 0 && i < int(index.size()) ? index[i] : -1;
    }
    e->foes.erase(std::remove_if(e->foes.begin(), e->foes.end(), [](const Foe& f) { return f.path.size() < 5; }), e->foes.end());
    if (e->foes.empty()) return nullptr;
    e->guns_only = single;
    // A big aircraft brought down keeps flying in the recording (its wreck stays on the contact
    // list, never marked down): LADON's second in its first phase, shot down by the player at
    // 216 s, then on a frozen heading, slowing and falling to the end. Down from where its heading
    // froze, if it stayed within 0.3 deg for the last 5 s or more.
    for (Foe& f : e->foes) {
        if (!f.boss || f.path.size() < 10 || f.down) continue;
        const float yaw_end = f.path.back().yaw;
        size_t k = f.path.size();
        while (k > 0 && std::abs(std::remainder(f.path[k - 1].yaw - yaw_end, 360.0f)) < 0.3f) --k;
        if (k < f.path.size() && f.path.back().t - f.path[k].t >= 5) { f.down = true; f.last = f.path[k].t; }
    }
    for (Foe& f : e->foes) {
        orient_from_path(f.path);
        if (single) { f.first = f.path.front().t; f.last = f.path.back().t; f.elite = elite_class(f.cls); }
    }
    // a boss (supersonic, ECM: lock only within 1 km) is met 800 m behind at its own speed
    const bool boss = std::any_of(e->foes.begin(), e->foes.end(), [](const Foe& f) { return f.boss; });
    standard_start(*e, behind > 0 ? behind : single ? 1000.0f : boss ? 800.0f : 1500.0f);
    for (const Foe& f : e->foes) for (const Pose& p : f.path) e->crash_z = std::min(e->crash_z, std::max(0.0f, p.pos.z - 20));
    return e;
}

// ---------------------------------------------------------------- scenarios
enum Kind { Capture, Switch, Chain, Hold, Landing, Pursuit, Single, Multi, Boss };
const Kind kinds[] = {Capture, Switch, Chain, Hold, Landing, Pursuit, Single, Multi, Boss};
const char* kind_name(Kind k) {
    return k == Capture ? "capture" : k == Switch ? "switch" : k == Chain ? "chain" : k == Hold ? "hold" : k == Landing ? "landing" :
           k == Pursuit ? "pursuit" : k == Single ? "single" : k == Multi ? "multi" : "boss";
}
bool encounter_kind(Kind k) { return k == Single || k == Multi || k == Boss; }

// A recorded landing (landings/<file>, cut by extract_landings.py): the runway, the state the
// approach starts from, the player's touchdown, and the player's aim, throttle, brake, speed and
// position over it. Every controller starts from the same state and is aimed along the player's
// path, at where the player was 1.5 s later (the recorded aim replayed as it was steered no
// controller onto the runway: the bench's flight path drifts tens of metres from the game's in
// 20 s, and the player had corrected for the game's own; following the path, every controller
// meets the runway where the player did). The speed follows the recording (the bench's idle
// model pulls toward cruise speed, ~190 m/s: on the runway the aircraft would never slow down).
struct Approach {
    float cx = 0, cy = 0, heading = 0, half_length = 0, half_width = 0, ground = 0;   // runway
    V3 pos; float pitch = 0, yaw = 0, roll = 0, speed = 0;                             // start
    float touch_t = 0, touch_sink = 0, touch_bank = 0, touch_pitch = 0;               // as recorded
    struct Sample { float t, aim_p, aim_y, throttle, brake, speed; V3 pos; };
    std::vector<Sample> r;
    // After the player's touchdown the path carries on along the runway at the touchdown speed,
    // on the ground: a controller still in the air is led down onto the runway, flying, instead
    // of after the player braking to a stop (the scene runs 8 s past the touchdown)
    V3 touch_pos, touch_dir{1, 0, 0}; float touch_speed = 0;
    Sample at(float t) const {
        if (t > touch_t && touch_speed > 0) {
            Sample s = recorded(touch_t);
            s.t = t; s.throttle = s.brake = 0; s.speed = touch_speed;
            s.pos = touch_pos + touch_dir * (touch_speed * (t - touch_t)); s.pos.z = ground;
            return s;
        }
        return recorded(t);
    }
    Sample recorded(float t) const {
        size_t k = 0;
        while (k + 1 < r.size() && r[k + 1].t <= t) ++k;
        if (k + 1 >= r.size()) return r.back();
        const Sample& a = r[k]; const Sample& b = r[k + 1];
        const float u = std::clamp((t - a.t) / std::max(b.t - a.t, 1e-3f), 0.0f, 1.0f);
        const float dy = std::remainder(b.aim_y - a.aim_y, 360.0f);
        return {t, a.aim_p + (b.aim_p - a.aim_p) * u, a.aim_y + dy * u, u < 0.5f ? a.throttle : b.throttle, u < 0.5f ? a.brake : b.brake,
                a.speed + (b.speed - a.speed) * u, a.pos + (b.pos - a.pos) * u};
    }
    static constexpr float lookahead = 1.5f;   // s along the player's path
};
// What counts as a landing: the envelope of the recorded wartime landings the game accepted
// (M28, 2026-10-05, at touchdown: sink 3.6 / 14.5 / 4.7 m/s, bank 2 / -15 / 3 deg, pitch
// 2.1 / -3.1 / 0.2 deg, within 8 m of the centre line, heading within 1 deg either way).
constexpr float land_sink = 15, land_bank = 15, land_pitch = -3.5f, land_side = 40, land_heading = 10;
std::shared_ptr<Approach> load_approach(const std::string& path) {
    std::ifstream in(path);
    if (!in) return nullptr;
    auto a = std::make_shared<Approach>();
    for (std::string line; std::getline(in, line);) {
        std::istringstream ss(line);
        std::string tag; ss >> tag;
        if (tag == "runway") ss >> a->cx >> a->cy >> a->heading >> a->half_length >> a->half_width >> a->ground;
        else if (tag == "start") ss >> a->pos.x >> a->pos.y >> a->pos.z >> a->pitch >> a->yaw >> a->roll >> a->speed;
        else if (tag == "touch") ss >> a->touch_t >> a->touch_sink >> a->touch_bank >> a->touch_pitch;
        else if (tag == "r") {
            Approach::Sample s;
            if (ss >> s.t >> s.aim_p >> s.aim_y >> s.throttle >> s.brake >> s.speed >> s.pos.x >> s.pos.y >> s.pos.z) a->r.push_back(s);
        }
    }
    if (a->r.size() <= 10) return nullptr;
    const Approach::Sample td = a->recorded(a->touch_t), before = a->recorded(a->touch_t - 0.5f);
    const V3 flat{td.pos.x - before.pos.x, td.pos.y - before.pos.y, 0};
    a->touch_pos = td.pos; a->touch_speed = td.speed;
    if (len(flat) > 1) a->touch_dir = unit(flat);
    return a;
}
struct Scenario {
    std::string id, title, note;
    Kind kind;
    float seconds = 8;
    float start_roll = 0;
    float start_pitch = 0, start_p = 0;                 // nose pitch and roll rate (deg/s) at the start
    float start_alt = 3000, start_speed = 220;           // m, m/s (scripted scenes)
    V3 first{}, next{}; float switch_at = -1;          // capture / chain
    std::function<V3(float)> aim;                        // hold: world aim over time
    std::vector<Segment> program;                        // pursuit: enemy program
    V3 enemy_offset{}; float enemy_yaw = 0, enemy_speed = 220;   // enemy start, relative to us (m, deg)
    std::shared_ptr<Encounter> encounter;                // recorded enemies
    std::shared_ptr<Approach> approach;                  // a recorded landing
    std::string group;   // scenes of one battle (M28 in four orders) share it: counted once in the summaries
};
std::vector<int> reference_order(const Encounter& e, float seconds, int prefer, const std::vector<int>& first = {});
// The scenes, named "<category> · <what>" with ids "<category>_<nn>". Scripted ones are defined
// here; those with recorded enemies are listed in dev/compare/scenes.txt.
std::vector<Scenario> scenarios(const std::string& dir) {
    std::vector<Scenario> s;
    auto add = [&](Scenario c) { s.push_back(c); };
    // General manoeuvres, as few as cover the distinct behaviours (scenes whose controller
    // rankings matched others, or that the recorded battles cover, were dropped 2026-10-05):
    // small, large and rear captures, a slight push, switches against the roll and to the belly,
    // holding still and with mouse nudges, and the low dive recovery.
    auto cap = [&](std::string id, std::string title, V3 dir, float roll = 0, std::string note = "") {
        Scenario c; c.id = id; c.title = "通用机动 · 捕获 " + title; c.kind = Capture; c.first = unit(dir); c.start_roll = roll; c.note = note; add(c);
    };
    cap("man_01", "右 3°", direction(0, 3), 0, "小角度修正");
    cap("man_02", "右 90°", direction(0, 90));
    cap("man_03", "右后 150°", direction(0, 150), 0, "大角度：方向选择");
    cap("man_04", "下 15°", direction(-15, 0), 0, "小幅下压：不应翻成倒飞");
    // A new target picked while the aircraft is still rolling from the last one (switches
    // against the roll cost ~1.5 s in recorded battles). Nose 30 deg up, 60 deg left bank,
    // rolling right at 45 deg/s; the target 35 deg off at a clock position around the nose.
    auto sw = [&](std::string id, std::string title, int clock) {
        const Frame3 b0 = basis(30, 0, -60);
        const float a = 35 * rad, c = clock * rad;
        Scenario x; x.kind = Switch; x.start_pitch = 30; x.start_roll = -60; x.start_p = 45;
        x.first = unit(b0.f * std::cos(a) + (b0.u * std::cos(c) + b0.r * std::sin(c)) * std::sin(a));
        x.id = id; x.title = "通用机动 · 换目标 " + title;
        x.note = "坡度左 60°、机头上仰 30°、正以 45°/s 向右滚时，新目标出现在偏 35° 处";
        add(x);
    };
    sw("man_05", "逆滚转方向 90°", -90);
    sw("man_06", "机腹方向 35°", 175);
    auto hold = [&](std::string id, std::string title, std::function<V3(float)> aim, std::string note) {
        Scenario c; c.id = id; c.title = "通用机动 · 保持 " + title; c.kind = Hold; c.seconds = 10; c.aim = aim; c.note = note; add(c);
    };
    hold("man_07", "平飞", [](float) { return direction(0, 0); }, "鼠标不动，只有角速度测量噪声：晃不晃");
    hold("man_08", "鼠标微调", [](float t) {
        float p = 0, y = 2;
        for (int k = 1; k <= int(t / 0.7f); ++k) {
            const unsigned h = static_cast<unsigned>(k) * 2654435761u;
            const float a = (h % 3600) * 0.1f * rad;
            p += std::sin(a); y += std::cos(a);
        }
        return direction(p, y);
    }, "已对准后每 0.7 s 移动鼠标约 1°（实测会引起 ±20° 摇翼）");
    // Near the sea, sinking fast, the aim raised above the horizon on the side away from the bank:
    // the way the Moon 11 crash happened (2026-10-05 11:33: 650 m/s, 480 m, -280 m/s, 80 deg of
    // bank, the aim rising and swinging sideways; recorded, it was too late for any controller,
    // so this starts higher). Crash = below 50 m.
    {
        Scenario c; c.id = "man_09"; c.title = "通用机动 · 低空 俯冲中瞄准点抬到左上"; c.kind = Hold; c.seconds = 6;
        c.start_alt = 900; c.start_speed = 650; c.start_pitch = -25; c.start_roll = 80;
        c.aim = [](float) { return direction(5, -30); };
        c.note = "900 m、650 m/s、下沉约 275 m/s、右坡度 80° 起，瞄准点在地平线上 5°、左侧 30°（坡度另一侧，Moon 11 撞海时的情形）";
        add(c);
    }
    // Recorded enemies, from the list: "<kind> <file> <id> <title>", kind single / multi / boss.
    std::ifstream list(dir + "/scenes.txt");
    for (std::string line; std::getline(list, line);) {
        std::istringstream ss(line);
        std::string kind, file, id;
        if (!(ss >> kind >> file >> id) || kind[0] == '#') continue;
        std::string title; std::getline(ss >> std::ws, title);
        // options before the title: "start=<m>" begins that far behind the enemy instead of the
        // standard distance; "order=game|near|nose|mixed" fixes the order of attack (the player's
        // own selections, then as the mixed reference; or a reference flight's choosing nearest
        // first, nearest the nose first, or mixed: see reference_order)
        float behind = -1, from = 0; std::string order; int missiles = NoMissiles; bool from_player = false, follow_selection = false;
        for (;;) {
            const size_t sp = title.find(' ');
            const std::string opt = title.substr(0, sp);
            if (opt == "start=player") from_player = true;
            else if (opt == "target=game") follow_selection = true;
            else if (opt.rfind("from=", 0) == 0) from = std::stof(opt.substr(5));
            else if (opt.rfind("start=", 0) == 0) behind = std::stof(opt.substr(6));
            else if (opt.rfind("order=", 0) == 0) order = opt.substr(6);
            else if (opt == "missiles=1") missiles = RuleMissiles;
            else if (opt == "missiles=long") missiles = LongRangeMissiles;
            else break;
            title = sp == std::string::npos ? "" : title.substr(sp + 1);
        }
        Scenario c; c.id = id; c.title = title; c.group = file;
        c.kind = kind == "single" ? Single : kind == "multi" ? Multi : kind == "boss" ? Boss : kind == "landing" ? Landing : Capture;
        if (c.kind == Capture) continue;
        if (c.kind == Landing) {   // landings/<file>: until 8 s after the recorded touchdown
            c.approach = load_approach(dir + "/landings/" + file);
            if (!c.approach) { std::fprintf(stderr, "scenes.txt: cannot read landings/%s\n", file.c_str()); continue; }
            c.seconds = c.approach->touch_t + 8;
            add(c);
            continue;
        }
        const std::string path = dir + (c.kind == Single ? "/tracks/" : "/battles/") + file;
        c.encounter = load_encounter(path, c.kind == Single, behind, from);
        if (!c.encounter) { std::fprintf(stderr, "scenes.txt: cannot read %s\n", path.c_str()); continue; }
        c.encounter->missiles = missiles;
        c.encounter->follow_selection = follow_selection;
        // The player's locks smoothed: a lock counts only once held 3 s, and a moment with none
        // keeps the last. Wearing both LADONs down in its first phase, the player went between them
        // and between their parts, a second or two each, and the pilot turned after each at once.
        if (follow_selection) {
            auto& sel = c.encounter->selections;
            std::vector<std::pair<float, int>> kept;
            for (size_t k = 0; k < sel.size(); ++k) {
                const float until = k + 1 < sel.size() ? sel[k + 1].first : 1e9f;
                if (sel[k].second < 0 || until - sel[k].first < 3) continue;
                if (kept.empty() || kept.back().second != sel[k].second) kept.push_back(sel[k]);
            }
            sel = kept;
        }
        // start=player: as the player was at the cut, attitude and all (a phase following another
        // without a break: LADON's second phase opens in a 50-70 deg dive at ~800 m/s, which the
        // player was already in; the standard start, level within 20 deg 800 m behind, left every
        // controller to push over at full speed close behind it)
        if (from_player && c.encounter->has_player) {
            Encounter& e = *c.encounter;
            e.start = e.player_pos; e.pitch = e.player_pitch; e.yaw = e.player_yaw; e.roll = e.player_roll;
            e.speed = std::clamp(e.player_speed, 200.0f, 900.0f);
        }
        float end = 0; for (const Foe& f : c.encounter->foes) end = std::max(end, f.last);
        c.seconds = std::floor(end * 4) / 4;
        if (!order.empty()) {
            Encounter& e = *c.encounter;
            // the player's order belongs to the player's own path: that run starts where the
            // player was (from the standard start the first targets were kilometres off and each
            // kill took ~13 s of transit)
            if (order == "game" && e.has_player) {
                e.start = e.player_pos; e.start.z = std::max(e.start.z, 600.0f);
                e.pitch = std::clamp(e.player_pitch, -20.0f, 20.0f); e.yaw = e.player_yaw;
                e.speed = std::clamp(e.player_speed, 200.0f, 900.0f);
            }
            e.order = order == "game" ? reference_order(e, c.seconds, 0, e.game_order)
                    : reference_order(e, c.seconds, order == "near" ? 1 : order == "nose" ? 2 : 0);
        }
        int bosses = 0, elites = 0; for (const Foe& f : c.encounter->foes) { bosses += f.boss; elites += f.elite; }
        char note[600];
        if (c.kind == Single) std::snprintf(note, sizeof(note), "敌机轨迹取自录像（%s）；我方从敌机后方 1 km 出发，全程用机炮跟踪（不计击落）。", elites ? "精英" : "普通");
        else {
            char dist[64];
            if (behind > 0) std::snprintf(dist, sizeof(dist), "%.0f m", behind);
            else std::snprintf(dist, sizeof(dist), "%s", bosses ? "800 m" : "1.5 km");
            const char* how = order == "game" ? "按实战锁定顺序（%d 架，之后按综合参考顺序），每次在接下来的 3 架里挑位置最好的一架击杀"
                            : order == "near" ? "按参考飞行“距离优先”得出的固定顺序击杀"
                            : order == "nose" ? "按参考飞行“偏角优先”得出的固定顺序击杀"
                            : order == "mixed" ? "按参考飞行“综合”得出的固定顺序击杀" : "";
            char fixed[160] = "";
            if (*how) { std::snprintf(fixed, sizeof(fixed), how, int(c.encounter->game_order.size())); }
            char from[96];
            if (order == "game" && c.encounter->has_player) std::snprintf(from, sizeof(from), "我方从实战中玩家的起点出发");
            else std::snprintf(from, sizeof(from), "我方从敌机后方 %s 出发", dist);
            std::snprintf(note, sizeof(note), "敌机 %d 架（精英 %d、头目 %d），轨迹取自录像；%s%s%s%s%s。",
                          int(c.encounter->foes.size()), elites, bosses, from, bosses ? "，速度取头目当时的速度" : "",
                          *fixed ? "；所有飞控" : "", fixed,
                          with_missiles || missiles == RuleMissiles ? "；机炮和导弹" : missiles == LongRangeMissiles ? "；机炮和 12 发远程导弹（逐帧飞行，尾追）" : "；只用机炮");
        }
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
    // gun: the share of the time with a target that the trigger was held with the nose on the lead
    // point (what can hit), and the unbroken spells on it (within 1 km, 2 deg): longest, median (s)
    float gun_usable = -1, gun_window_max = 0, gun_window_med = 0;
    float gun_hits = 0, gun_hits_max = 0;   // hit-seconds credited by the rounds: all enemies, and the most on one
    float first_kill = -1, on_lead = -1, close_time = 0, to_shot = -1;
    float lock_frac = -1;   // share of the time the target was within missile lock (15 deg, 0.3-2.5 km; a boss 1 km)
    float crashed = -1;     // time the aircraft went below 50 m (the ground is near 0-200 m)
    // landings: 1 down within the limits on the runway, 0 not (crashed, or never down); at touchdown
    int landed = -1;
    float td_sink = -1, td_bank = 0, td_pitch = 0, td_side = 0, td_late = 0;   // td_late: s after the player
    float clear = -1;       // time the last enemy came down (the run ends there)
    float stall = 0;        // s below the stall speed (225 km/h)
};
struct Sample { float v[40]; int n; };
// One long-range missile: launched at t0 at an enemy `range` m away and `off` deg off the nose; how it ended
// (0 still flying at the end, 1 hit, 2 lost: the enemy beyond 117 deg of its heading, 3 its 25 s
// out, 4 the enemy gone meanwhile, 5 a boss's CIWS) and when; its path, t x y z every 0.1 s.
struct Shot { float t0, range, off, t_end = -1; int target, outcome = 0; std::vector<float> track; };
// the bench's launch-range bins for the long-range missile log (m): 0-500, 500-800, 800-1200, 1200-2000, 2000-4000, 4000+
constexpr float shot_bins[] = {500, 800, 1200, 2000, 4000, 1e9f};
constexpr int n_shot_bins = sizeof(shot_bins) / sizeof(shot_bins[0]);
int shot_bin(float range) { int b = 0; while (range >= shot_bins[b]) ++b; return b; }
struct Run { Metrics m; std::vector<Sample> frames; std::vector<std::pair<float, int>> kills; std::vector<char> kill_gun; std::vector<Shot> shots; };

struct Scorer {
    const Scenario& sc;
    Metrics& m;
    float start_t = 0, min_angle = 180, held = 0, held_level = 0, last_q = 0, last_r = 0;
    int last_sign = 0;
    double sum_angle = 0, sum_angle2 = 0, sum_bank2 = 0, track_time = 0, in2 = 0, in5 = 0;
    double sat_q = 0, sat_r = 0, effort = 0, total = 0;
    float track_from;
    Scorer(const Scenario& s, Metrics& out) : sc(s), m(out) {
        track_from = sc.kind == Pursuit ? 3.0f : sc.kind == Hold || sc.kind == Landing || encounter_kind(sc.kind) ? 1.0f : 1e9f;
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
            if (m.gun_usable >= 0) m.gun_usable *= k;
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
        else if (sc.kind == Landing)   // on the aim, a smooth stick, a soft touchdown; down within the limits
            m.cost = m.rms_angle + 0.1f * m.chatter + 0.3f * std::max(m.td_sink, 0.0f) + 0.5f * std::max(m.td_late, 0.0f) +
                     (m.landed == 1 || m.crashed >= 0 ? 0.0f : 20.0f);
        else if (sc.kind == Pursuit)
            m.cost = 0.5f * m.mean_angle + 3 * (1 - m.within5) + 0.2f * m.reversals + 0.05f * m.chatter;
        else if (sc.kind == Boss)   // kept in missile lock, on the gun lead point, the nose near the aim
            m.cost = (with_missiles ? 10 * (1 - std::max(m.lock_frac, 0.0f)) + 10 * (1 - std::max(m.gun_usable, 0.0f))
                                    : 20 * (1 - std::max(m.gun_usable, 0.0f))) + 0.2f * m.mean_angle +
                     0.05f * rev_rate + 0.01f * chatter_rate;   // gun only: the usable gun time in place of the lock
        else if (sc.kind == Single)   // gun tracking of one enemy: on the lead point, steadily
            m.cost = 0.5f * m.mean_angle + 2 * (1 - m.within5) + 2 * (1 - m.within2) + 0.05f * rev_rate + 0.01f * chatter_rate;
        else   // enemies brought down, how soon all of them, how quickly a picked enemy is fired at, gun aim
            m.cost = 20 * (1 - float(std::max(m.kills, 0)) / std::max(m.foes, 1)) + 10 * (m.clear < 0 ? 1 : m.clear / span) +
                     (m.to_shot < 0 ? 15 : m.to_shot) + 5 * (1 - std::max(m.gun_usable, 0.0f)) + 0.2f * m.mean_angle;
        if (m.crashed >= 0) m.cost += 20;
    }
};

// Conditions: the aircraft model plus how the host senses body rates (on the game clock the
// filtered rates are quiet, 0.1-0.2 deg/s rms in level flight, logged). Every scene runs on
// every condition.
struct Condition { std::string id, title; const Plant* plant; float white; float jitter; std::array<int, 3> bars{}; };
// The aircraft fitted from flights, used when there is no dev/compare/aircraft/aircraft.txt.
const Condition fitted_conditions[] = {
    {"mig21bis", "MiG-21bis", &mig21bis, 0.15f, 0.0f},
    {"su35", "Su-35", &su35, 0.15f, 0.0f},
    {"su57", "Su-57", &su57, 0.15f, 0.0f},
    {"adfx02", "ADF-X02", &adfx02, 0.15f, 0.0f},
    {"fa36", "FA-36", &fa36, 0.15f, 0.0f},
};
std::vector<Condition> conditions;
int condition_count = 0;
std::deque<Plant> imported_plants;
std::deque<std::string> imported_names;
// Every player plane from the game's own flight parameters (import_aircraft.py writes the file
// from the spec probe; it is game data and stays out of the repository): the FA-36's fitted
// dynamics (delays, smoothing, flight path) with each aircraft's pull, roll, high-G, thrust,
// brake and stall speed. Sorted by the hangar's Speed + Mobility. (A made-up sluggish aircraft,
// a MiG-21bis with 30% more delay and 25% less authority, followed until 2026-10-06: the game's
// own slow aircraft cover it now.)
void load_conditions(const std::string& dir) {
    std::ifstream in(dir + "/aircraft/aircraft.txt");
    for (std::string line; std::getline(in, line);) {
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> f;
        std::stringstream ss(line);
        for (std::string part; std::getline(ss, part, '|');) f.push_back(part);
        if (f.size() < 9) continue;
        auto nums = [](const std::string& text) { std::vector<float> v; std::stringstream t(text); for (float x; t >> x;) v.push_back(x); return v; };
        const auto bars = nums(f[2]), pull = nums(f[5]), roll = nums(f[6]), thrust = nums(f[7]), brake = nums(f[8]);
        if (bars.size() < 3 || pull.size() < 9 || roll.size() < 9 || thrust.size() < 9 || brake.size() < 9) continue;
        Plant plant = fa36;
        imported_names.push_back(f[0]); plant.id = imported_names.back().c_str();
        imported_names.push_back(f[1]); plant.title = imported_names.back().c_str();
        plant.stall_speed = std::stof(f[3]);
        plant.highg_pull = std::stof(f[4]);
        if (f.size() > 9 && !f[9].empty()) plant.speed_max = std::stof(f[9]);
        for (int k = 0; k < 9; ++k) {
            plant.pull_by_speed[k] = pull[k]; plant.roll_by_speed[k] = roll[k];
            plant.thrust_by_speed[k] = thrust[k]; plant.brake_by_speed[k] = brake[k];
        }
        imported_plants.push_back(plant);
        conditions.push_back({f[0], f[1], &imported_plants.back(), 0.15f, 0.0f, {int(bars[0]), int(bars[1]), int(bars[2])}});
    }
    if (conditions.empty()) for (const Condition& c : fitted_conditions) conditions.push_back(c);
    condition_count = int(conditions.size());
}

// The simulated player's habits, one set per pass over every scene and aircraft. The standard
// one (seed 0) is what the controllers were tuned on; the others took no part in tuning and show
// whether a ranking holds for other players and other luck (robust=0 skips them).
struct PilotProfile {
    const char* id; const char* title;
    float react = 1, highg_off = 40;   // reaction x, high-G beyond (deg)
    unsigned seed = 0;
};
const PilotProfile pilot_profiles[] = {
    {"std", "标准（调参用）"},
    {"std_s1", "标准 · 种子 1", 1, 40, 1},
    {"std_s2", "标准 · 种子 2", 1, 40, 2},
    // slow (reaction and scatter x1.5), sharp (x0.75, x0.7, high-G from 25 deg) and few high-G
    // turns (from 70 deg) until 2026-10-07: they ranked as the standard (0.874-0.895 against its
    // 0.887) and crowded the page; the two seeds stay, to tell a change from the forks.
};
PilotProfile pilot_profile = pilot_profiles[0];   // read-only while a pass runs

// No simulated hand on the mouse (removed 2026-10-07): the mouse is where the pilot wants it at
// once. Strokes, pauses, reaction and scatter modelled after the player's mouse (cddf6e3 on) moved
// the results more than any controller change did, and each fix brought new artefacts (the mouse
// still while a fighter moved away, stepping on a crossing target).

// The simulated player: the same decisions for every controller. Picks an enemy the way the
// game's target selection tends to (nearest the nose, distance counting), keeps it until it is
// down or over 5 km away, puts the mouse on it (a little ahead of its motion across the view),
// and works the throttle:
// throttle+brake (a high-G turn) while the enemy is more than 40 deg off the nose, full
// throttle while it is beyond 1.5 km (a boss: 800 m) and not closing at 50 m/s, brake inside
// 600 m (a boss: 400 m) when closing fast. Low over the ground it does not follow the aim into
// it: within ~4 s of 700 m the mouse is held at least level, higher the lower it gets.
// It fires:
//   missiles: the enemy within 15 deg of the nose at 300-2500 m (a boss with ECM: within 1 km)
//     for 0.8 s locks it; two rails, each reloading 4.1 s after it fires, ~700 m/s. It hits an ordinary enemy,
//     except that the first missile at every fourth one (by its index: the same enemies for every
//     controller) misses (the player's missiles: a fighter gone within the flight time after 75%
//     of the presses, near and far alike, logged); an elite (or a boss's escort drone) dodges unless launched within 1000 m and 5 deg, and takes two hits;
//     a boss's CIWS stops them (a boss is scored on the lock time, not brought down).
//     So by the old rules (missiles=1); a scene with missiles=long flies long-range missiles instead (see LongRange).
//   gun: rounds at 1389 m/s plus the aircraft's velocity, falling at 9.8 m/s^2, gone after 1.08 s
//     (the game's BP_plwp_gun_x1). The game's ring (the locked target within 1.1 km, with the range
//     part) shows where the rounds are now at the enemy's distance, trailing the nose in a turn;
//     the player fires while it covers the enemy. The player sees no lead point (the game shows
//     none): the mouse goes on the enemy (see before()); beyond the ring's range only a large
//     target is fired on, blind, the nose on it. Hits and the gun measures go by where the enemy really is (the program's window:
//     rounds fired then would meet it, within its size); the player's window is the ring shown
//     and on the enemy. The trigger, a player's, not a script's, on the game's gun ring
//     (4.4 deg around the marker): the ring sweeping onto the judged point
//     fast enough to be on it within a reaction time, it fires ahead (a dogfight's snap shot);
//     coming on slowly (chasing a bomber), it fires once on it (2 deg) for a reaction time
//     (0.25 s, scaled by the profile). Either way it keeps firing until the nose is 3 deg off
//     or the enemy beyond 1 km. (Fired the frame the nose came within 2 deg until 2026-10-05.)
//     Each frame's rounds
//     fly at 1000 m/s on top of the aircraft's speed along the nose and are judged where they
//     arrive, against where the enemy really is then (its recorded path), with a 0.35 deg
//     spread: the share that falls within the enemy's size (a fighter 6 m, a bomber 20 m, a boss
//     30 m) counts as hits, 150 a second of them on a bomber, 100 on a fighter (gun_damage_rate). Down when
//     they reach its health (health(); the game's DT_NPC_Model, 2026-10-06: a MiG-29 60, a Tu-95
//     150, a Shadow MiG-29 60 as any). The round's 5 m burst
//     reaches more than the part it strikes: a Tu-95 shot at its wing went down
//     with its body's 150 (the player: 2.2 s of fire at 450-580 m), not with the wing's 75; a
//     missile on a wing does not bring it down. (1 s of hits for all until then; 0.5 s, and 1.5 s
//     an elite, before 2026-10-06.) (Judged at the
//     moment of firing, with the nose within a wingspan of a perfect lead point, every
//     controller shot fighters down from 1 km in half a second; the player, firing a median
//     920 m out, brought down fighters with the gun about twice in five recordings.)
struct Pilot {
    const Encounter& e;
    std::vector<char> killed;
    std::vector<float> hits;
    std::vector<int> missile_hits, on_the_way;
    std::vector<std::pair<float, int>> flying;   // missile impact time, enemy
    // The game's gun marker, a ring around where the rounds go: at 500 m it just frames LADON's
    // body (~77 m), 4.4 deg in radius. The player fires when the enemy, where they judge it will
    // be, is inside it.
    static constexpr float gun_ring = 4.4f;
    // The player's rounds (BP_plwp_gun_x1, read in game 2026-10-06): 1389 m/s from the muzzle
    // (5000 km/h, +-5% a round) on top of the aircraft's velocity, falling at 9.8 m/s^2, gone
    // after 1.08 s (1.5 km and the aircraft's own speed). 1000 m/s, no fall and no end until then.
    static constexpr float muzzle_speed = 1389, round_life = 1.08f, round_fall = 9.8f;
    // Rounds fly until they pass the enemy (or their life ends) and count by the closest they came,
    // within the enemy's size plus the round's 5 m burst (the game's RayCastRadius, 500 cm: area
    // damage, a proximity hit). Judged at a fixed time, when they had covered the range the enemy
    // was at when fired, those at an enemy opening the range fell 35-47 m short and nothing hit.
    static constexpr float round_burst = 5;
    // No draw onto the enemy beyond its size and the burst: the hits the game reports (contacts.lua
    // "hit" packets, every one with the victim's health after it; dev/compare/gun_hits.py) came
    // from rounds that would have passed it, flown straight along the nose, a median 8-34 m off
    // a bomber and 7-19 m off a fighter at every range from 0 to 1.7 km (with and without the gun
    // parts, 2026-10-07): a distance, its size and burst, not an angle (3.5 deg until then, from
    // kills that took in wrecks and missiles: 61 m at 1 km, 92 at 1.5). BENCH_ASSIST=<deg> to try.
    static inline float gun_assist_deg = 0;
    struct Round { float t0, dt; V3 p0, v; int target; float spread, best, assist; };
    std::vector<Round> rounds;                   // gun rounds on their way (see the gun above)
    // the size a round must fall within: a fighter, a bomber (or its wing), a boss
    // By the aircraft, not by its being a boss: Shadow 22 (a Su-57) and Moon 11 (an X-40A) are
    // fighters, and took LADON's 30 m until 2026-10-06.
    // Health, by the aircraft (the game's DT_NPC_Model MaxHealth, 2026-10-06; the same as live
    // enemies' MaxHealth in a mission); a fighter not listed as a MiG-29
    float health(int i) const {
        static const std::pair<const char*, float> table[] = {
            {"tu95", 150}, {"t160", 125}, {"b01b", 130}, {"ladn", 1200}, {"x40a", 300}, {"su57", 115}, {"f22a", 110},
            {"f35c", 70}, {"su35", 65}, {"typn", 60}, {"m29a", 60}, {"f04e", 60}, {"su25", 60}, {"uavn", 60},
            {"mr2k", 55}, {"f16c", 55}, {"a06e", 55}, {"m21b", 46}, {"mi24", 35}};
        for (const auto& [name, hp] : table) if (e.foes[i].cls.find(name) != std::string::npos) return hp;
        return 60;
    }
    // A second of hits: 150 on a bomber, 100 on a fighter. The game reports a hit about every 0.05 s
    // while the rounds keep on an enemy (20 a second, unbroken runs to a kill, with or without the
    // gun parts), 7.5 off a Tu-95's or a Mirage's health from the player's FA-36 with the damage
    // part, 5 from an X-40A without (2026-10-07). The rate differs by attacker and target type (the
    // player's account); this version takes 150 a second on bombers and 100 on fighters for all
    // (the player's choice). The player's aircraft carry the gun parts (range, grouping, damage) in
    // every recording: they are the standard. Was 100 a second, 150% on bombers and 85% on fighters.
    float gun_damage_rate(int i) const {
        const std::string& c = e.foes[i].cls;
        const bool large = c.find("tu95") != std::string::npos || c.find("t160") != std::string::npos || c.find("b01b") != std::string::npos;
        return large ? 150.0f : 100.0f;
    }
    float hit_radius(int i) const {
        const std::string& c = e.foes[i].cls;
        if (c.find("ladn") != std::string::npos) return 30;
        if (c.find("tu95") != std::string::npos || c.find("t160") != std::string::npos || c.find("b01b") != std::string::npos) return 20;
        return 6;
    }
    float lock = 0;
    // The standard missile's two rails (the game's MaxLoadedCount 2): each reloads 4.1 s after
    // it fires (LoadTime, DT_WeaponSettings "C.Story", the campaign's row). Two can go at once,
    // then about one in 2 s. (One every 1.5 s, without end, until 2026-10-06.)
    static constexpr float missile_reload = 4.1f;
    float rail_ready[2] = {0, 0};
    int missiles = 0, gun_kills = 0;
    float picked_at = 0; bool shot = false;
    double to_shot = 0; int shots_timed = 0;
    int target = -1;
    V3 aim{1, 0, 0};       // where the mouse is (what the controller flies to)
    V3 lead{1, 0, 0};      // where the enemy will be when rounds fired now arrive (the gun measures)
    V3 impact{1, 0, 0};    // where rounds fired now go: the nose plus the aircraft's own velocity
    // The game's gun ring: where the rounds are now at the enemy's distance, those fired a flight
    // time ago along the nose and velocity the aircraft had then. In a hard turn the aircraft
    // turns on while they fly straight, and the ring is left well behind the nose (in game, far
    // from the screen's centre; drawn at the nose until 2026-10-06, as if the rounds were lasers).
    V3 ring{1, 0, 0};
    std::deque<std::pair<float, std::pair<V3, V3>>> muzzle;   // (t, (position, rounds' velocity)), the last 1.5 s
    V3 seen_lead{1, 0, 0}; // where the player puts the mouse: on the enemy (the game shows no lead point)
    // The game shows the gun ring with the locked target within 1.1 km with the range part (the
    // player's account, 2026-10-07; taken as nothing else). Beyond it the player fires only at a large target (a bomber, a boss), blind,
    // with the nose within blind_deg of it (reported: a bomber sprayed without the ring).
    static constexpr float ring_range = 1100, blind_deg = 2;
    // A fighter followed at 700 m as anything else: closer, the bench pilot held its nose worse (M28,
    // the mean of 35 aircraft: 700 m -> 14 deg off a fighter within 1.5 km, 350 m 20, 250 m 25) and
    // killed no more (2026-10-07). BENCH_FOLLOW=<m> to try.
    static inline float fighter_follow = 700;
    // The mouse ahead of the enemy by its motion across the view in this many seconds: the player,
    // not reckoning a lead (their account), kept the mouse a median 2.0 deg ahead of a fighter along
    // its motion (quartiles -0.7 to +5.3; X-40A, 2026-10-07), the nose 7.8 deg off it, 9.9 behind
    // the mouse as the bench's. The bench pilot, the mouse on the enemy: 2.9 deg behind it (the eye's
    // delay and the hand's drag), the nose 14 off. M28, 35 aircraft (anticipation: mouse ahead, nose
    // off, gun kills): 0 -2.9, 14.2, 10.5; 0.15 s -0.3, 11.0, 11.0; 0.3 s +1.8, 9.2, 11.6; 0.5 s
    // +4.7, 7.9, 13.2; 0.8 s +8.2, 9.0, 13.7. BENCH_ANTICIPATE=<s> to try.
    static inline float anticipate = 0.3f;
    V3 los_prev{1, 0, 0}, los_rate{0, 0, 0}; int los_target = -1;
    bool ring_shown = false, player_window = false;   // the player's window: the ring shown and on the enemy
    std::mt19937 judge_rng{7};
    std::normal_distribution<float> judge_n{0.0f, 1.0f};
    float range = 0;
    bool taking_over = false;   // the low takeover held (see the floor)
    std::vector<std::pair<float, int>> kill_log;
    std::vector<char> kill_gun;                  // per kill: brought down by the gun
    std::vector<int> launched;                   // missiles launched at each enemy
    double close_time = 0, on_lead = 0, lock_time = 0, target_time = 0;
    int elite_kills = 0;
    float throttle = 0, brake = 0, last_range = -1;
    // free choice (no fixed order): 0 mixed (angle off the nose and distance), 1 nearest first,
    // 2 nearest the nose first
    int prefer = 0;
    static constexpr int order_window = 3;   // with a fixed order: picked among the next this many
    float pull_rate = 20;
    bool extending = false;   // flying out after an overshoot (see before())   // deg/s: the aircraft's full pull at its present speed (fly() sets it: a player knows its aircraft)
    int missile_kind = with_missiles ? RuleMissiles : e.missiles;
    bool use_missiles = missile_kind != NoMissiles;
    // long-range missiles in flight (missile_kind LongRangeMissiles)
    struct Flying { V3 pos, vel; float t0; int target, shot; };
    std::vector<Flying> long_range;
    std::vector<Shot> shots;   // every long-range missile launched, for the log and the viewer
    int long_left = LongRange::aboard;
    std::vector<std::pair<float, float>> target_speed;   // the target's speed lately (time, m/s), for its braking
    explicit Pilot(const Encounter& enc, V3 forward)
        : e(enc), killed(enc.foes.size(), 0), hits(enc.foes.size(), 0), missile_hits(enc.foes.size(), 0),
          on_the_way(enc.foes.size(), 0), launched(enc.foes.size(), 0), aim(forward) {
        const PilotProfile& pp = pilot_profile;
        judge_rng.seed(7u + pp.seed);
    }
    // Time from picking an enemy to the first shot at it. A pick never fired at counts until it
    // is dropped (gone, out of reach, the next one picked or the scene's end): averaged over the
    // shots alone, an enemy picked and never fired at took no part (the measure favoured giving up).
    bool pick_open = false;
    // where the rounds are now at `dist` m: fired a flight time ago from where the aircraft was,
    // along its velocity then
    V3 ring_at(float t, float dist) const {
        if (muzzle.size() < 2) return impact;
        const V3 now = muzzle.back().second.first;
        for (size_t k = muzzle.size(); k-- > 0;) {
            const float age = t - muzzle[k].first;
            const V3 at = muzzle[k].second.first + muzzle[k].second.second * age + V3{0, 0, -0.5f * round_fall * age * age};
            if (len(at - now) >= dist || k == 0) return unit(at - now);
        }
        return impact;
    }
    void open_pick(float t) {
        close_pick(t); end_window(); picked_at = t; shot = false; pick_open = true; extending = false;
    }
    // the trigger and the unbroken spells on the lead point
    bool trigger = false;
    float window = 0, seen_on = 0, last_off = -1, off_rate = 0;
    double usable = 0;
    std::vector<float> windows;
    void end_window() { if (window > 0) windows.push_back(window); window = 0; seen_on = 0; trigger = false; last_off = -1; off_rate = 0; player_window = false; }
    void close_pick(float t) { if (pick_open && !shot) { to_shot += t - picked_at; ++shots_timed; } pick_open = false; }
    bool alive(int i, float t) const { const Foe& f = e.foes[i]; return !killed[i] && t >= f.first && t <= f.last; }
    // An enemy brought down in the recording within 30 s (by the player then, most of them: the
    // order of attack is the player's) is not picked: its path ends there and it vanishes, and the
    // simulated player, a little behind the recorded one, chased three in a row that vanished as
    // it closed (2026-10-06). 30 s: with 15, a tenth of the picks still vanished while chased, a
    // median 23 s after being picked. It is the recording's own end of it, not a made-up one, the
    // same for every controller.
    static constexpr float vanish_horizon = 30;
    bool vanishing(int i, float t) const { const Foe& f = e.foes[i]; return f.down && f.last - t < vanish_horizon; }
    // tough: an elite, or a boss's escort (Moon 11's drones: the player shot none down at Expert)
    bool tough(int i) const { return e.foes[i].elite || e.foes[i].escort; }
    int needed(int i) const { return e.foes[i].boss ? 1000000 : tough(i) ? 2 : 1; }
    bool all_down() const { for (char k : killed) if (!k) return false; return true; }
    float lock_range(int i) const { return e.foes[i].ecm ? 1000.0f : 2500.0f; }
    // a boss with two or more of its escorts within 150 m: shielded, shots do not reach it
    bool shielded(int i, float t) const {
        if (!e.foes[i].boss) return false;
        const V3 p = pose_at(e.foes[i].path, t).pos;
        int guarding = 0;   // ("near" is a Windows macro)
        for (int k = 0; k < int(e.foes.size()); ++k)
            if (e.foes[k].escort && alive(k, t) && len(pose_at(e.foes[k].path, t).pos - p) < 150) ++guarding;
        return guarding >= 2;
    }
    // an escort inside its boss's shield (the boss shielded, this drone within 150 m of it)
    bool in_field(int k, float t) const {
        const V3 p = pose_at(e.foes[k].path, t).pos;
        for (int i = 0; i < int(e.foes.size()); ++i)
            if (e.foes[i].boss && alive(i, t) && shielded(i, t) && len(pose_at(e.foes[i].path, t).pos - p) < 150) return true;
        return false;
    }
    bool protected_now(int i, float t) const { return shielded(i, t) || (e.foes[i].escort && in_field(i, t)); }
    // Brought down already, or (the old rules: they hit for certain) a missile on its way. A long-range missile is
    // flown and may miss: the player stays on the enemy until it is down (the player, 2026-10-06;
    // with a long-range missile out the next was taken at once until then).
    bool doomed(int i) const { return missile_hits[i] + (missile_kind == LongRangeMissiles ? 0 : on_the_way[i]) >= needed(i); }
    void kill(float t, int i, bool gun = false) {
        killed[i] = 1; kill_log.push_back({t, i}); kill_gun.push_back(gun);
        elite_kills += e.foes[i].elite;
        if (target == i) target = -1;
    }
    void before(float t, float dt, const Aircraft& me) {
        // a fighter more than 5 km away is let go; a boss never (a player chases it whatever the
        // distance: dropping LADON at 5 km in its 850 m/s phase left the aircraft circling, 60 km behind)
        if (target >= 0 && (!alive(target, t) || doomed(target) ||
                            (e.order.empty() && !e.foes[target].boss && len(pose_at(e.foes[target].path, t).pos - me.pos) > 5000))) target = -1;
        // Moon 11's drones while they shield her cannot be harmed: no point in chasing them
        if (target >= 0 && e.foes[target].escort && in_field(target, t)) target = -1;
        if (e.follow_selection && !e.selections.empty()) {   // the player's lock at this moment, if any
            int sel = -1;
            for (const auto& [ts, i] : e.selections) { if (ts > t) break; sel = i; }
            if (sel >= 0 && sel != target && alive(sel, t) && !doomed(sel)) { target = sel; open_pick(t); }
        }
        if (target < 0 && !e.order.empty()) {   // a fixed order: the best placed of the next three still flying
            float best = 1e9f; int seen = 0;
            for (int i : e.order) {
                if (!alive(i, t) || doomed(i) || vanishing(i, t) || (e.foes[i].escort && in_field(i, t))) continue;
                const V3 to = pose_at(e.foes[i].path, t).pos - me.pos;
                const float off = std::acos(std::clamp(dot(me.b.f, unit(to)), -1.0f, 1.0f)) / rad;
                const float c = off / 20 + len(to) / 1000;   // as the free choice, mixed
                if (c < best) { best = c; target = i; }
                if (++seen == order_window) break;
            }
            if (target >= 0) open_pick(t);
        }
        if (target < 0) {
            float best = 1e9f;
            for (int i = 0; i < int(e.foes.size()); ++i) {
                if (!alive(i, t) || doomed(i) || vanishing(i, t) || (e.foes[i].escort && in_field(i, t))) continue;
                const V3 to = pose_at(e.foes[i].path, t).pos - me.pos;
                const float d = len(to);
                if (d > 8000 && !e.foes[i].boss) continue;
                const float off = std::acos(std::clamp(dot(me.b.f, unit(to)), -1.0f, 1.0f)) / rad;
                const float c = (prefer == 1 ? off / 180 + d / 1000 : prefer == 2 ? off / 5 + d / 4000 : off / 20 + d / 1000)
                                - (e.foes[i].boss && !e.foes[i].ecm ? 3.0f : 0.0f);   // Moon 11 before her drones
                if (c < best) { best = c; target = i; }
            }
            if (target >= 0) open_pick(t);
        }
        V3 mouse_to;
        muzzle.push_back({t, {me.pos, me.vel + me.b.f * muzzle_speed}});
        while (muzzle.size() > 2 && muzzle.front().first < t - 1.5f) muzzle.pop_front();
        if (target >= 0) {
            const V3 v = me.vel + me.b.f * muzzle_speed;
            const V3 epos = pose_at(e.foes[target].path, t).pos;
            impact = unit(v);
            lead = lead_point(e.foes[target].path, t, me.pos, len(v));
            range = len(epos - me.pos);
            ring = ring_at(t, range);
            // What the player sees: the enemy, and within 1 km the game's gun ring. The mouse goes on
            // the enemy and the trigger waits for the ring to cover it. No lead point: the game shows
            // none (until 2026-10-06 the bench pilot judged one from the enemy's true future, 0.4-1.6
            // times it with a 15% wander, which no player can see). Nor ahead of it by the ring's
            // trail behind the nose: that fed back (the harder the turn, the further the mouse, the
            // harder the turn): the nose swung 30-50 deg about a fighter within 1 km every ~3 s. M28,
            // the mean of 35 aircraft (gain on the trail, its time constant): 0 -> 30.1 kills, 18.7 by
            // the gun, the ring on a fighter within 1 km 10% of the time; 0.5, 1 s -> 26.9, 16.0, 8%;
            // 1, 2 s -> 27.5, 16.9, 8%; 1, 0.25 s -> 15.6, 6.3, 4%.
            // The eye follows the enemy's motion across the view a little ahead of it (anticipate s of
            // its angular motion, seen over ~0.2 s): no lead point, what is on the screen moving.
            const V3 seen = unit(epos - me.pos);
            if (target == los_target && dt > 0) los_rate = los_rate + ((seen - los_prev) * (1 / dt) - los_rate) * (1 - std::exp(-dt / 0.2f));
            else los_rate = V3{0, 0, 0};
            los_prev = seen; los_target = target;
            seen_lead = unit(seen + los_rate * anticipate);
            ring_shown = range <= ring_range;
            mouse_to = seen_lead;
        } else {   // nobody to attack: level flight on the current heading
            const V3 flat{me.b.f.x, me.b.f.y, 0};
            lead = len(flat) > 0.1f ? unit(flat) : V3{1, 0, 0};
            seen_lead = impact = ring = mouse_to = lead;
            ring_shown = false;
        }
        // Overshot (the enemy within 300 m and behind the wing line): fly out on the present heading
        // at full throttle and come back from 800 m, a new pass, instead of turning back at once.
        // Faster than a slow bomber (the 540 km/h floor below), the bench pilot circled one at
        // 35-300 m for 38 s, its nose never on (looping, 15-135 deg off), not a round fired; the
        // player, after a bomber, spent 9% of the time within 300 m (a median 1.9 s a pass) and 20%
        // at 300-700 m, the bench pilot 21% and 9% (2026-10-06).
        if (target >= 0) {
            const V3 to = pose_at(e.foes[target].path, t).pos - me.pos;
            const float behind = std::acos(std::clamp(dot(me.b.f, unit(to)), -1.0f, 1.0f)) / rad;
            if (!extending && range < 300 && behind > 90) extending = true;
            else if (extending && range > 800) extending = false;
            if (extending) {
                const float path_el = std::asin(std::clamp(me.vel.z / std::max(len(me.vel), 1.0f), -1.0f, 1.0f)) / rad;
                mouse_to = direction(std::clamp(path_el, -10.0f, 20.0f), std::atan2(me.vel.y, me.vel.x) / rad);
            }
        } else extending = false;
        aim = mouse_to;
        // Low over the ground: the aim no steeper down than a path that can still be levelled above
        // 150 m (crash below 50 m): time to roll upright and start the pull (2 s and the bank at
        // 90 deg/s: inverted 4 s), then half the aircraft's own full pull at this speed: from a path
        // angle g at speed v it loses v sin(-g) meanwhile and (v / pull) (1 - cos g) pulling out.
        // Tried over the boss fights (BENCH_FLOOR, 2026-10-06): 0.5 s and the full pull let the
        // dives after Moon 11 from 2.8 km go into the ground (24 and 19 of 36); 2 s and half, 14
        // and 9, with LADON's low windows kept (63%); a high-G pull when held up cost LADON's windows (at 15 deg/s for every aircraft, the weak
        // ones at speed, 6-10 deg/s, went into the sea after Moon 11). Below 150 m, up, by as much as a path climbing
        // back within 4 s (at most 25 deg). By the aim and the height, so that a dive already too
        // steep is pulled out of (the aim above the path), and no feedback from the climb.
        // Until 2026-10-06 a floor set by the height 4 s ahead at the present climb fed back on
        // itself at speed (diving after LADON over the sea it came up 25 deg, the climb took it
        // away, once a second; rolled inverted to pull down, told up while inverted, into the sea:
        // 20 of 36 runs in its second phase); then, by a 4 s straight path above 150 m, it allowed
        // 3.6 deg down 150 m up at 2200 km/h, and the aircraft took 12 s to come down to LADON
        // after its dive to the sea, no firing window meanwhile.
        {
            const float v = std::max(len(me.vel), 100.0f), room = me.pos.z - (e.crash_z + 100);   // 100 m over the ground
            // half the full pull: the path follows the nose late, and the controller is still lining
            // the lift up (after Moon 11's dive from 2.8 km the path turned ~10 deg/s on a ~20 pull)
            const float radius = v / (std::max(floor_pull_share * pull_rate, 3.0f) * rad);
            const float cos_pitch = std::max(std::sqrt(std::max(0.0f, 1 - me.b.f.z * me.b.f.z)), 0.1f);
            const float bank = std::acos(std::clamp(me.b.u.z / cos_pitch, -1.0f, 1.0f)) / rad;   // 0 level, 180 inverted
            const float react = floor_react + bank / 90;
            float floor_el = std::asin(std::clamp(-room / (v * 4), 0.0f, 0.42f)) / rad;   // below 150 m
            if (room >= 0) {
                floor_el = 0;
                for (float g = -1; g >= -90; g -= 1)
                    if (v * react * std::sin(-g * rad) + radius * (1 - std::cos(g * rad)) <= floor_margin * room) floor_el = g; else break;
            }
            const float el = std::asin(std::clamp(aim.z, -1.0f, 1.0f)) / rad;
            if (el < floor_el) aim = direction(floor_el, std::atan2(aim.y, aim.x) / rad);
            // A player over the sea does not wait for the controller: the aircraft's own path going
            // into the sea before it could level out (the floor bounds the mouse, not where the
            // controller has the aircraft), or inverted near it, the mouse goes up whatever the
            // target does, and stays up until the sinking stops or the pull-out needs under half the
            // height left. Against the floor's own line (100 m over the crash line) it fired every
            // few seconds following LADON at 150 m. The floor alone counted on a pull-out the controller may not give (upstream's,
            // rolling inverted over LADON low over the sea). Taken over when the height ran out
            // within 5 s at the present sink and let go as soon as not, it pumped the mouse between
            // +25 and -35 deg every 4-5 s from 2.8 km down (LADON's second phase, 2026-10-07).
            if (low_take) {
                const float path = std::asin(std::clamp(me.vel.z / v, -1.0f, 1.0f)) / rad;
                const float need = path < 0 ? v * react * std::sin(-path * rad) + radius * (1 - std::cos(path * rad)) : 0;
                const float height = me.pos.z - e.crash_z;
                if (need > height || (height < low_take_m && me.b.u.z < -0.5f)) taking_over = true;   // into the crash line itself (the floor keeps 100 m over it)   // inverted: bank past 120 (90-100 is a hard turn low over the sea)
                else if (taking_over && (me.vel.z >= 0 || need < 0.5f * height)) taking_over = false;
                const float el_now = std::asin(std::clamp(aim.z, -1.0f, 1.0f)) / rad;
                if (taking_over && el_now < low_take_deg) aim = direction(low_take_deg, std::atan2(aim.y, aim.x) / rad);
            }
        }
        const float off = std::acos(std::clamp(dot(me.b.f, aim), -1.0f, 1.0f)) / rad;
        const float closing = target >= 0 && last_range >= 0 && dt > 0 ? (last_range - range) / dt : 0;
        last_range = target >= 0 ? range : -1;
        throttle = brake = 0;
        // Speed, as a player flies it: the throttle open unless closing on the target faster than
        // a following distance wants (a boss 400 m, others 700 m, closed at a third of the excess
        // a second), then neutral; the brake when near and still closing much faster. Logged
        // 2026-10-05, four sorties: throttle 48-58% of the time, neutral 21-39%, brake 3-8%,
        // high-G 8-21%, a median 1300-1950 km/h. (Throttle only when closing too slowly, neutral
        // otherwise, left the bench pilot near the neutral cruise, ~750 km/h in fights.)
        // Beyond the following distance, as fast as the throttle let off sheds by then: closing
        // at sqrt(2 a d), a the neutral throttle's drag at speed (25 m/s^2: the game's neutral, 26-28
        // at 500-600 m/s). Capped at 200 m/s until 2026-10-06: after a far enemy, a 300 km/h Tu-95
        // say, the pilot let the throttle go at 800-900 km/h and coasted the kilometres in.
        const bool boss = target >= 0 && e.foes[target].boss;
        const std::string& tcls = target >= 0 ? e.foes[target].cls : std::string();
        const bool fighter = target >= 0 && !boss && tcls.find("tu95") == std::string::npos && tcls.find("t160") == std::string::npos &&
                             tcls.find("b01b") == std::string::npos;
        const float follow = boss ? 400.0f : fighter ? fighter_follow : 700.0f;
        // The target braking hard (more than ~6 g over the last 0.3 s, as a pilot sees it slow):
        // throttle off and brake at once while near and closing, instead of running into it.
        // LADON alone (no other enemy flies so) slows hard from 630 to 380 m/s in its first phase:
        // its slowing over the last second taken off what letting the throttle go sheds, and the
        // brake beyond the following distance while it slows. Before, it was closed at 250-300 m/s
        // with the throttle let go from 1.4 km and the brake only within 400 m, run past at 250 m,
        // and circled for a minute (FA-36, 141-152 s). Applied to every enemy, it cost M28 kills.
        float decel = 0, slowing = 0;
        if (target >= 0) {
            const float v_now = len(pose_at(e.foes[target].path, t).pos - pose_at(e.foes[target].path, t - 0.1f).pos) / 0.1f;
            target_speed.push_back({t, v_now});
            while (!target_speed.empty() && target_speed.front().first < t - 1.0f) target_speed.erase(target_speed.begin());
            for (const auto& [ts, vs] : target_speed)
                if (ts >= t - 0.3f) { if (t - ts > 0.2f) decel = (vs - v_now) / (t - ts); break; }
            if (tcls.find("ladn") != std::string::npos && t - target_speed.front().first > 0.8f) slowing = std::max(0.0f, (target_speed.front().second - v_now) / (t - target_speed.front().first));
        } else target_speed.clear();
        const float shed = std::max(25.0f - slowing, 5.0f);   // m/s^2 the gap closes slower by, the throttle let go
        const float want = range > follow ? std::sqrt(2 * shed * (range - follow)) : std::max(-60.0f, (range - follow) / 3);
        if (target >= 0 && decel > 60 && range < 2 * follow && closing > 0) brake = 1;
        else if (target >= 0 && off > pilot_profile.highg_off && len(me.vel) > 150) throttle = brake = 1;
        else if (target >= 0 && (range < follow || slowing > 5) && closing > want + 60) brake = 1;
        else if (target < 0 || closing <= want) throttle = 1;
        // Not slowed below 540 km/h to sit behind a slower enemy: the player, within 700 m of a
        // bomber, flew a median 650 km/h (quartiles 520-1000), of a fighter 970; the bench pilot,
        // matching the enemy's speed, sat behind Tu-95s at 300 km/h, near the stall (2026-10-06).
        if (!(throttle > 0.5f && brake > 0.5f) && len(me.vel) < 150) { throttle = 1; brake = 0; }
        if (extending) { throttle = 1; brake = 0; }
    }
    void after(float t, float dt, const Aircraft& me) {
        for (size_t k = 0; k < long_range.size();) {   // long-range missiles: flown, steering at the enemy's tail
            Flying& m = long_range[k];
            const int i = m.target;
            const float age = t - m.t0;
            bool gone = age > LongRange::life || !alive(i, t);
            if (!gone) {
                float speed = len(m.vel);
                if (age > LongRange::accel_delay) speed = std::min(LongRange::top, speed + LongRange::accel * dt);
                V3 dir = unit(m.vel);
                const V3 was = pose_at(e.foes[i].path, t - dt).pos, now = pose_at(e.foes[i].path, t).pos;
                if (age > LongRange::homing_delay) {
                    const V3 los = unit(now - m.pos);
                    const float ang = std::acos(std::clamp(dot(dir, los), -1.0f, 1.0f));
                    if (ang > LongRange::max_homing * rad) gone = true;   // lost: it flies on, harming nothing
                    else if (ang > 1e-4f) {
                        const float turn = (age < LongRange::homing_delay + LongRange::reduced_for ? LongRange::reduced_rate : LongRange::rate) * rad * dt;
                        const V3 side = unit(los - dir * dot(dir, los));
                        const float a = std::min(ang, turn);
                        dir = unit(dir * std::cos(a) + side * std::sin(a));
                    }
                }
                if (!gone) {
                    const V3 from = m.pos;
                    m.vel = dir * speed; m.pos = m.pos + m.vel * dt;
                    std::vector<float>& tr = shots[m.shot].track;
                    if (t - tr[tr.size() - 4] >= 0.1f) tr.insert(tr.end(), {t, m.pos.x, m.pos.y, m.pos.z});
                    // the closest it came to the enemy over the step (both moving)
                    const V3 r0 = was - from, r1 = now - m.pos, dr = r1 - r0;
                    const float u = std::clamp(-dot(r0, dr) / std::max(dot(dr, dr), 1e-6f), 0.0f, 1.0f);
                    if (len(r0 + dr * u) < hit_radius(i)) {
                        shots[m.shot].outcome = 1; shots[m.shot].t_end = t;
                        shots[m.shot].track.insert(shots[m.shot].track.end(), {t, m.pos.x, m.pos.y, m.pos.z});
                        --on_the_way[i];
                        if (!protected_now(i, t) && ++missile_hits[i] >= needed(i)) kill(t, i);
                        long_range.erase(long_range.begin() + k);
                        continue;
                    }
                }
            }
            if (gone) {
                Shot& sh = shots[m.shot];
                sh.outcome = age > LongRange::life ? 3 : !alive(i, t) ? 4 : 2; sh.t_end = t;
                --on_the_way[i]; long_range.erase(long_range.begin() + k); continue;
            }
            ++k;
        }
        for (size_t k = 0; k < flying.size();) {   // missiles arriving
            if (t < flying[k].first) { ++k; continue; }
            const int i = flying[k].second;
            --on_the_way[i];
            if (alive(i, t) && !protected_now(i, t) && ++missile_hits[i] >= needed(i)) kill(t, i);
            flying.erase(flying.begin() + k);
        }
        for (size_t k = 0; k < rounds.size();) {   // gun rounds on their way: the closest each comes to its enemy
            Round& r = rounds[k];
            const int i = r.target;
            const float age = t - r.t0;
            bool done = age > round_life || !alive(i, t);
            if (!done) {
                const V3 pos = r.p0 + r.v * age + V3{0, 0, -0.5f * round_fall * age * age};
                const float d = len(pose_at(e.foes[i].path, t).pos - pos);
                if (d < r.best) r.best = d;
                else done = true;   // past it: the closest was the last
            }
            if (!done) { ++k; continue; }
            if (r.best < 1e8f && alive(i, t) && !protected_now(i, t)) {
                const float R = hit_radius(i) + round_burst + r.assist, w = R * R + 2 * r.spread * r.spread;
                hits[i] += r.dt * (R * R / w) * std::exp(-r.best * r.best / w);   // the share of a spread burst within R
                if (!e.guns_only && !e.foes[i].boss && hits[i] * gun_damage_rate(i) >= health(i)) { ++gun_kills; kill(t, i, true); }
            }
            rounds.erase(rounds.begin() + k);
        }
        if (target < 0) { lock = 0; end_window(); return; }
        const V3 to = pose_at(e.foes[target].path, t).pos - me.pos;
        const float seen = std::acos(std::clamp(dot(me.b.f, unit(to)), -1.0f, 1.0f)) / rad;
        const bool in_lock = missile_kind == LongRangeMissiles
                                 ? seen < LongRange::lock_angle && range < (e.foes[target].ecm ? 1000.0f : LongRange::lock_range)
                                 : seen < 15 && range > 300 && range < lock_range(target);
        lock = in_lock ? lock + dt : 0;
        target_time += dt; if (in_lock) lock_time += dt;
        float* rail = rail_ready[0] <= rail_ready[1] ? &rail_ready[0] : &rail_ready[1];
        // one long-range missile at a time at an enemy: the second rail waits to see the first hit or miss
        if (missile_kind == LongRangeMissiles && !e.guns_only && lock >= 0.8f && t >= *rail && long_left > 0 && range >= LongRange::min_range &&
            on_the_way[target] == 0 && !doomed(target)) {
            if (!shot) { shot = true; to_shot += t - picked_at; ++shots_timed; }
            // a boss's CIWS stops it, as the other missiles
            shots.push_back({t, range, seen, -1, target, 0, {t, me.pos.x, me.pos.y, me.pos.z}});
            if (!e.foes[target].boss) { long_range.push_back({me.pos, me.vel + me.b.f * LongRange::ignition, t, target, int(shots.size()) - 1}); ++on_the_way[target]; }
            else { shots.back().outcome = 5; shots.back().t_end = t; }
            ++launched[target]; ++missiles; --long_left;
            *rail = t + LongRange::reload;
            if (doomed(target)) { target = -1; lock = 0; return; }
        }
        if (missile_kind == RuleMissiles && !e.guns_only && lock >= 0.8f && t >= *rail && !doomed(target)) {
            if (!shot) { shot = true; to_shot += t - picked_at; ++shots_timed; }
            const Foe& f = e.foes[target];
            const bool first_misses = !tough(target) && target % 4 == 3 && launched[target] == 0;
            ++launched[target];
            const bool good = !f.boss && !first_misses && (!tough(target) || (range < 1000 && seen < 5));
            if (good) { flying.push_back({t + range / 700.0f, target}); ++on_the_way[target]; }
            ++missiles;
            *rail = t + missile_reload;
            if (doomed(target)) { target = -1; lock = 0; return; }
        }
        // the gun within the rounds' reach in their life (muzzle and own speed, 1.08 s: ~1.9 km at
        // 1800 km/h); 1 km until 2026-10-06, where the player's gun kills were a median 1.0-1.15 km out
        if (range > (muzzle_speed + len(me.vel)) * round_life) { end_window(); return; }
        // rounds fired now against where the enemy will really be (what hits: the measures), and
        // the game's ring against the enemy (what the player fires on: the rounds in flight are
        // reaching it; kept on it in a steady turn, the nose is on the lead)
        impact = unit(me.vel + me.b.f * muzzle_speed);
        const float off = std::acos(std::clamp(dot(impact, lead), -1.0f, 1.0f)) / rad;
        // what the player fires on: the ring on the enemy when the game shows it; beyond, at a large
        // target only, the nose on it (blind); a fighter not at all
        const bool blind = !ring_shown && hit_radius(target) >= 20;
        const float judged = std::acos(std::clamp(dot(ring_shown ? ring : me.b.f, unit(to)), -1.0f, 1.0f)) / rad;
        const float cover = ring_shown ? gun_ring : blind_deg;
        player_window = ring_shown && judged < gun_ring;
        // a window: rounds fired now would meet the enemy (within its size, as hits are judged,
        // plus the rounds' spread); 2 deg for everything until 2026-10-06
        const float hit_deg = std::atan2(hit_radius(target) + round_burst, std::max(range, 1.0f)) / rad + gun_assist_deg + 0.35f;
        if (last_off >= 0 && dt > 0) off_rate += ((judged - last_off) / dt - off_rate) * (1 - std::exp(-dt / 0.1f));   // deg/s, seen over ~0.1 s
        last_off = judged;
        close_time += dt;
        // a window is a moment rounds would hurt it: not while it is shielded (Moon 11 with her
        // drones near), when they meet it and count nothing
        const bool hittable = off < hit_deg && !protected_now(target, t);
        if (hittable) { on_lead += dt; window += dt; }
        else if (window > 0) { windows.push_back(window); window = 0; }
        seen_on = judged < cover ? seen_on + dt : 0;
        const float react = 0.25f * pilot_profile.react;
        if (!ring_shown && !blind) trigger = false;
        else if (!trigger) trigger = (judged < 2 * cover && off_rate < -5 && judged + off_rate * react < cover) || seen_on >= react;
        else if (judged > cover + 1) trigger = false;
        if (trigger) {   // the trigger: rounds along the nose, judged on arrival
            if (hittable) usable += dt;
            if (!shot) { shot = true; to_shot += t - picked_at; ++shots_timed; }
            const V3 v = me.vel + me.b.f * muzzle_speed;
            rounds.push_back({t, dt, me.pos, v, target, range * std::tan(0.35f * rad), 1e9f, range * std::tan(gun_assist_deg * rad)});
        }
    }
};

// A fixed order of attack, made without any of the controllers under test: an idealised fighter
// (no attitude dynamics: the flight path turns straight toward the aim at 12 deg/s, speed by
// the same throttle and brake, 150-800 m/s) flies the scene with the simulated player choosing
// freely by `prefer`; the order it brought them down in, then those it did not reach (by
// first appearance). `first`, when given, goes in front (the player's own order). The slow
// turn keeps the order's pace one every controller can hold: made at 35 deg/s, a controller a
// few seconds late on one target found the next ones kilometres further on and fell further
// behind with each (21 kills against 45 in the same order).
std::vector<int> reference_order(const Encounter& e, float seconds, int prefer, const std::vector<int>& first) {
    Encounter free = e; free.order.clear();
    Aircraft me; me.start(e.pitch, e.yaw, e.roll, e.speed, e.start);
    Pilot pilot(free, me.b.f); pilot.prefer = prefer;
    const float dt = 1.0f / 40;
    for (float t = 0; t < seconds && !pilot.all_down(); t += dt) {
        pilot.before(t, dt, me);
        const V3 v = unit(me.vel), a = pilot.aim;
        const float ang = std::acos(std::clamp(dot(v, a), -1.0f, 1.0f)), step = std::min(ang, 12 * rad * dt);
        V3 dir = v;
        if (ang > 1e-4f) { const V3 side = unit(a - v * dot(v, a)); dir = unit(v * std::cos(step) + side * std::sin(step)); }
        float speed = len(me.vel);
        speed += (pilot.throttle > 0.5f && pilot.brake > 0.5f ? -30.0f : pilot.brake > 0.5f ? -60.0f : pilot.throttle > 0.5f ? 40.0f : 0.0f) * dt;
        speed = std::clamp(speed, 150.0f, 800.0f);
        me.vel = dir * speed; me.pos = me.pos + me.vel * dt;
        me.pos.z = std::max(me.pos.z, 300.0f);
        me.b = basis(std::asin(std::clamp(dir.z, -1.0f, 1.0f)) / rad, std::atan2(dir.y, dir.x) / rad, 0);
        pilot.after(t, dt, me);
    }
    std::vector<int> order;
    auto put = [&](int i) { if (i >= 0 && i < int(e.foes.size()) && std::find(order.begin(), order.end(), i) == order.end()) order.push_back(i); };
    for (int i : first) put(i);
    for (const auto& k : pilot.kill_log) put(k.second);
    std::vector<int> rest(e.foes.size());
    for (int i = 0; i < int(rest.size()); ++i) rest[i] = i;
    std::stable_sort(rest.begin(), rest.end(), [&](int a, int b) { return e.foes[a].first < e.foes[b].first; });
    for (int i : rest) put(i);
    return order;
}

// Roll against the guidance: time within 20 deg of the aim (outside a tail or push) that the roll
// stick is set against a roll error of more than 30 deg the guidance asks for (controllers that
// report it in trace(): ours and the archived configurations). Per controller and condition.
std::map<std::string, std::array<double, 4>> roll_fight;
std::map<std::string, double> stall_time;
// The simulated pilot's speed in fights: s with throttle, neutral, brake, high-G, s at 1000 km/h or more,
// and the speeds (km/h) for the median. Per controller and condition.
struct SpeedLog { double mode[4] = {}, fast = 0, total = 0; std::vector<float> kmh; };
std::map<std::string, SpeedLog> pilot_speed;
// long-range missiles of the standard pass, per controller: [launch-range bin][outcome 0-5]
std::map<std::string, std::array<std::array<int, 6>, n_shot_bins>> long_range_log;
std::mutex stats_mutex;   // the maps above: each run adds its own totals at its end
std::string trace_scene;   // BENCH_TRACE=<scene>:<controller>@<condition>[,<from s>]: the controller's internals for 6 s
float trace_from = 0;
Run fly(const Condition& cond, const Scenario& sc, Controller& ctl, bool keep_frames, const std::string& ctl_id = "") {
    const bool trace_this = !trace_scene.empty() && trace_scene == sc.id + ":" + ctl_id + "@" + cond.id;
    const Plant& plant = *cond.plant;
    const float dt = 1.0f / 80;   // logged median frame 12.5-13.3 ms
    const Encounter* enc = sc.encounter.get();
    const Approach* app = sc.approach.get();
    const float speed0 = std::min(enc ? enc->speed : app ? app->speed : sc.start_speed, plant.speed_max);
    Aircraft me;
    if (enc) me.start(enc->pitch, enc->yaw, enc->roll, speed0, enc->start);
    else if (app) me.start(app->pitch, app->yaw, app->roll, speed0, app->pos);
    else me.start(sc.start_pitch, 0, sc.start_roll, speed0, {0, 0, sc.start_alt});
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
    std::mt19937 rng(1234 + pilot_profile.seed);
    std::normal_distribution<float> unit_noise(0.0f, 1.0f);
    float fq = 0, fp = sc.start_p, fr = 0;
    Run run;
    std::array<double, 4> fight{};   // roll_fight for this run
    SpeedLog speed_log;              // pilot_speed for this run
    Scorer score(sc, run.m);
    V3 target = sc.first;
    const int steps = int(sc.seconds / dt);
    const float alt0 = me.pos.z;
    const int every = sc.seconds > 200 ? 16 : enc ? 8 : 4;   // stored frames: 5, 10 or 20 a second
    float t_end = 0;
    bool fired = false;   // the trigger held at some moment since the last stored frame (frames are 5-20 a second)
    for (int k = 0; k < steps; ++k) {
        const float t = k * dt;
        t_end = t;
        if (sc.kind == Chain && k == int(sc.switch_at / dt)) { target = sc.next; score.restart(t); }
        if (sc.kind == Hold) target = unit(sc.aim(t));
        if (sc.kind == Pursuit) { enemy.step(sc.program, t, dt); target = unit(enemy.pos - me.pos); }
        const Approach::Sample rec = app ? app->at(t) : Approach::Sample{};
        if (app) {   // along the player's path (see Approach)
            const V3 ahead = app->at(t + Approach::lookahead).pos - me.pos;
            target = len(ahead) > 1 ? unit(ahead) : direction(rec.aim_p, rec.aim_y);
            // no steeper than the path itself less 20 deg: a slow aircraft fallen behind and above
            // the path was aimed 70 deg down at 600 m, as no one would, and one flying the aim
            // exactly dived into the sea
            const V3 p0 = app->at(t).pos, p1 = app->at(t + 0.5f).pos, d = p1 - p0;
            const float path_el = len(d) > 1 ? std::asin(std::clamp(d.z / len(d), -1.0f, 1.0f)) / rad : 0;
            const float el = std::asin(std::clamp(target.z, -1.0f, 1.0f)) / rad;
            if (el < path_el - 20) target = direction(path_el - 20, std::atan2(target.y, target.x) / rad);
        }
        if (pilot) {
            pilot->pull_rate = plant.pitch_pull * by_speed(plant.pull_by_speed, len(me.vel));
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
        const float throttle = pilot ? pilot->throttle : app ? rec.throttle : 0.0f, brake = pilot ? pilot->brake : app ? rec.brake : 0.0f;
        if (pilot) {
            SpeedLog& sl = speed_log;
            sl.mode[throttle > 0.5f ? (brake > 0.5f ? 3 : 0) : brake > 0.5f ? 2 : 1] += dt;
            const float kmh = len(me.vel) * 3.6f;
            sl.total += dt; if (kmh >= 1000) sl.fast += dt;
            if (k % 8 == 0) sl.kmh.push_back(kmh);
        }
        Sense s{pitch, yaw, roll, {target.x, target.y, target.z}, fq, fr, fp, seen_dt, throttle, brake,
                {me.vel.x, me.vel.y, me.vel.z}, {me.acc.x, me.acc.y, me.acc.z}, me.pos.z};
        Stick u = ctl.step(s);
        u.pitch = std::clamp(u.pitch, -1.0f, 1.0f); u.roll = std::clamp(u.roll, -1.0f, 1.0f); u.yaw = std::clamp(u.yaw, -1.0f, 1.0f);
        {
            float angle = 0, ep = 0, ey = 0, er = 0; int tail = 0, push = 0;
            const char* tr = ctl.trace(); const char* at = std::strstr(tr, "angle=");
            if (at && std::sscanf(at, "angle=%f w=%*f tail=%d push=%d err=(p%f,y%f,r%f)", &angle, &tail, &push, &ep, &ey, &er) == 6 &&
                angle < 20 && angle > 0.5f && !tail && !push) {
                auto& f = fight;
                const int i = angle < 5 ? 0 : 2;
                f[i + 1] += dt;
                if (std::abs(er) > 30 && u.roll * er < 0) f[i] += dt;
            }
        }
        trace_on = trace_this && k % 8 == 0 && t >= trace_from && t < trace_from + 6;
        if (trace_on) std::printf("t %.2f alt %.0f vz %.0f stick %.2f %.2f thr %.0f brk %.0f pitch %.1f roll %.0f aim_el %.1f | %s\n", t, me.pos.z, me.vel.z,
                                  u.pitch, u.roll, throttle, brake, pitch, roll, std::asin(target.z) / rad, ctl.trace());
        me.step(plant, u, throttle, brake, dt);
        trace_on = false;
        if (app) {
            if (rec.speed > 1) me.vel = unit(me.vel) * std::min(rec.speed, plant.speed_max);   // the recorded speed (see Approach)
            if (me.pos.z <= app->ground + 1) {   // touchdown: held to the recorded landings' limits
                Metrics& m = run.m;
                const float h = app->heading * rad;
                const V3 d = me.pos - V3{app->cx, app->cy, 0};
                const float along = d.x * std::cos(h) + d.y * std::sin(h), side = -d.x * std::sin(h) + d.y * std::cos(h);
                const float track = std::atan2(me.vel.y, me.vel.x) / rad;
                const float off_heading = std::abs(std::remainder(track - app->heading, 180.0f));   // either way along it
                float tp, ty, tr; euler(me.b, tp, ty, tr);
                m.td_sink = -me.vel.z; m.td_bank = tr; m.td_pitch = tp; m.td_side = side; m.td_late = t - app->touch_t;
                m.landed = (m.td_sink <= land_sink && std::abs(tr) <= land_bank && tp >= land_pitch && std::abs(side) <= land_side &&
                            std::abs(along) <= app->half_length && off_heading <= land_heading) ? 1 : 0;
                if (!m.landed) m.crashed = t;
                break;
            }
        } else if (me.pos.z < (enc ? enc->crash_z : 50.0f)) { run.m.crashed = t; break; }

        const float angle = std::acos(std::clamp(dot(me.b.f, target), -1.0f, 1.0f)) / rad;
        score.add(t, dt, angle, bank_of(me.b), me.p, u, me.nz);
        if (pilot) { pilot->after(t, dt, me); fired = fired || pilot->trigger; }
        const bool cleared = pilot && !enc->guns_only && pilot->all_down();
        const bool has_enemy = sc.kind == Pursuit || (pilot && pilot->target >= 0);
        if (keep_frames && k % every == 0) {
            float ep = 0, ey = 0, er = 0;
            if (has_enemy) euler(enemy.b, ep, ey, er);
            Sample f{{t, me.pos.x, me.pos.y, me.pos.z, pitch, yaw, roll, target.x, target.y, target.z, angle,
                      me.q, me.p, me.r, u.pitch, u.roll, u.yaw, len(me.vel), me.nz,
                      std::atan2(-dot(me.vel, me.b.u), dot(me.vel, me.b.f)) / rad,
                      has_enemy ? enemy.pos.x : 0, has_enemy ? enemy.pos.y : 0, has_enemy ? enemy.pos.z : 0, ep, ey, er,
                      pilot ? float(pilot->target) : -1, pilot ? float(pilot->kill_log.size()) : 0,
                      // the gun lead point (world direction) and the trigger, for the viewer's HUD
                      pilot ? pilot->lead.x : 0, pilot ? pilot->lead.y : 0, pilot ? pilot->lead.z : 0, fired ? 1.0f : 0.0f,
                      pilot ? pilot->ring.x : 0, pilot ? pilot->ring.y : 0, pilot ? pilot->ring.z : 0,
                      pilot ? pilot->seen_lead.x : 0, pilot ? pilot->seen_lead.y : 0, pilot ? pilot->seen_lead.z : 0,
                      pilot && pilot->window > 0 ? 1.0f : 0.0f,
                      // the player's window: the ring shown (within 1 km) and on the enemy
                      pilot && pilot->player_window ? 1.0f : 0.0f},
                     enc ? 40 : sc.kind == Pursuit ? 26 : 20};
            run.frames.push_back(f);
            fired = false;
        }
        if (cleared) { run.m.clear = t; break; }
    }
    if (app && run.m.landed < 0) run.m.landed = 0;   // never down (still flying 8 s after the player was)
    run.m.stall = me.stalled;
    {
        std::lock_guard<std::mutex> lock(stats_mutex);
        const std::string key = ctl_id + "@" + cond.id;
        if (keep_frames && pilot)
            for (const Shot& sh : pilot->shots) ++long_range_log[ctl_id][shot_bin(sh.range)][sh.outcome];
        stall_time[key] += me.stalled;
        auto& f = roll_fight[key];
        for (int i = 0; i < 4; ++i) f[i] += fight[i];
        if (speed_log.total > 0) {
            SpeedLog& sl = pilot_speed[key];
            for (int i = 0; i < 4; ++i) sl.mode[i] += speed_log.mode[i];
            sl.fast += speed_log.fast; sl.total += speed_log.total;
            sl.kmh.insert(sl.kmh.end(), speed_log.kmh.begin(), speed_log.kmh.end());
        }
    }
    if (pilot) {
        Metrics& m = run.m;
        m.foes = int(enc->foes.size()); m.kills = int(pilot->kill_log.size()); m.elite_kills = pilot->elite_kills;
        m.gun_kills = pilot->gun_kills; m.missiles = pilot->missiles;
        pilot->close_pick(t_end);
        pilot->end_window();
        m.gun_usable = pilot->target_time > 0 ? float(pilot->usable / pilot->target_time) : 0;
        for (float h : pilot->hits) { m.gun_hits += h; m.gun_hits_max = std::max(m.gun_hits_max, h); }
        if (!pilot->windows.empty()) {
            std::vector<float> w;   // passes of under 0.05 s (the nose sweeping through) are not spells
            for (float x : pilot->windows) if (x >= 0.05f) w.push_back(x);
            std::sort(w.begin(), w.end());
            if (!w.empty()) { m.gun_window_max = w.back(); m.gun_window_med = w[w.size() / 2]; }
        }
        m.to_shot = pilot->shots_timed ? float(pilot->to_shot / pilot->shots_timed) : -1;
        m.lock_frac = pilot->target_time > 0 ? float(pilot->lock_time / pilot->target_time) : 0;
        m.first_kill = pilot->kill_log.empty() ? -1 : pilot->kill_log.front().first;
        // the share of the time with a target that the gun was on it: over the time within 1 km it
        // favoured closing in seldom (2 s near with 1 s on beat 60 s near with 24 s on)
        m.close_time = float(pilot->close_time); m.on_lead = pilot->target_time > 0 ? float(pilot->on_lead / pilot->target_time) : 0;
        run.kills = pilot->kill_log; run.kill_gun = pilot->kill_gun; run.shots = pilot->shots;
    }
    score.finish(speed0 - len(me.vel), me.pos.z - alt0);
    return run;
}

// ---------------------------------------------------------------- registry and output
struct Entry { std::string id, label, source; std::function<std::unique_ptr<Controller>()> make; };

void json_metrics(std::string& out, const Metrics& m) {
    char b[1600];
    std::snprintf(b, sizeof(b),
        "{\"to5\":%.2f,\"to2\":%.2f,\"settle\":%.2f,\"level\":%.2f,\"overshoot\":%.2f,\"reversals\":%d,\"chatter\":%d,"
        "\"inverted\":%.2f,\"speed_loss\":%.1f,\"alt_change\":%.0f,\"peak_nz\":%.1f,\"mean_angle\":%.2f,\"rms_angle\":%.2f,"
        "\"within2\":%.3f,\"within5\":%.3f,\"bank_rms\":%.1f,\"pitch_sat\":%.3f,\"roll_sat\":%.3f,\"roll_effort\":%.3f,\"cost\":%.2f,"
        "\"kills\":%d,\"elite_kills\":%d,\"foes\":%d,\"gun_kills\":%d,\"missiles\":%d,\"first_kill\":%.1f,\"on_lead\":%.3f,"
        "\"close_time\":%.1f,\"to_shot\":%.2f,\"crashed\":%.1f,\"lock_frac\":%.3f,\"clear\":%.1f,"
        "\"landed\":%d,\"td_sink\":%.1f,\"td_bank\":%.1f,\"td_pitch\":%.1f,\"td_side\":%.0f,\"td_late\":%.1f,\"stall\":%.2f,\"gun_usable\":%.3f,\"gun_window_max\":%.2f,\"gun_window_med\":%.2f,\"gun_hits\":%.2f,\"gun_hits_max\":%.2f}",
        m.to5, m.to2, m.settle, m.level, m.overshoot, m.reversals, m.chatter, m.inverted, m.speed_loss, m.alt_change, m.peak_nz,
        m.mean_angle, m.rms_angle, m.within2, m.within5, m.bank_rms, m.pitch_sat, m.roll_sat, m.roll_effort, m.cost,
        m.kills, m.elite_kills, m.foes, m.gun_kills, m.missiles, m.first_kill, m.on_lead, m.close_time, m.to_shot, m.crashed, m.lock_frac, m.clear,
        m.landed, m.td_sink, m.td_bank, m.td_pitch, m.td_side, m.td_late, m.stall, m.gun_usable, m.gun_window_max, m.gun_window_med, m.gun_hits, m.gun_hits_max);
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
    bool all_refs = false, robust = true;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = args[i];
        const auto eq = arg.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = arg.substr(0, eq), value = arg.substr(eq + 1);
        if (key == "out") out_dir = value;
        else if (key == "config") config = value;
        else if (key == "only") only = value;
        else if (key == "refs") all_refs = value == "all";
        else if (key == "robust") robust = value != "0";
        else if (key == "missiles") with_missiles = value == "1";
        else if (key == "variant") {   // "label:key=value;key=value"
            const auto colon = value.find(':');
            variants.push_back({value.substr(0, colon), colon == std::string::npos ? "" : value.substr(colon + 1)});
        }
    }
    if (const char* as = std::getenv("BENCH_ASSIST")) Pilot::gun_assist_deg = std::stof(as);
    if (const char* fl = std::getenv("BENCH_FLOOR")) std::sscanf(fl, "%f,%f", &floor_pull_share, &floor_react);
    if (const char* fo = std::getenv("BENCH_FOLLOW")) Pilot::fighter_follow = std::stof(fo);
    if (const char* an = std::getenv("BENCH_ANTICIPATE")) Pilot::anticipate = std::stof(an);
    if (const char* lt = std::getenv("BENCH_LOWTAKE")) std::sscanf(lt, "%f,%f", &low_take_m, &low_take_deg), low_take = low_take_m > 0;
    if (const char* tr = std::getenv("BENCH_TRACE")) {
        trace_scene = tr;
        const auto comma = trace_scene.find(',');
        if (comma != std::string::npos) { trace_from = std::stof(trace_scene.substr(comma + 1)); trace_scene.erase(comma); }
    }
    const std::string scratch = out_dir + "/results/tuning.ini";
    CreateDirectoryA((out_dir + "/results").c_str(), nullptr);
    std::vector<Entry> entries;
    const flight::Tuning base = ours_tuning(config, "", scratch);
    entries.push_back({"ours", "当前版本", "src/flight_logic.h + Payload config.ini", [base] { return std::make_unique<Ours>(base); }});
    // The game's installed [tuning] is the development configuration (tuned there, live): it
    // is not a column of its own but must equal the repository's. Any key that differs is
    // reported here and at the top of the scorecard, to be brought into the repository.
    // (dev\Dev-Deploy remembers the game folder.)
    std::string drift;
    {
        std::ifstream game(".dev-game-path");
        std::string root; std::getline(game, root);
        if (root.rfind("\xEF\xBB\xBF", 0) == 0) root.erase(0, 3);   // written by PowerShell with a BOM
        while (!root.empty() && (root.back() == '\r' || root.back() == ' ')) root.pop_back();
        const std::string installed = root + "\\Game\\Binaries\\Win64\\UE4SS\\Mods\\AC8MouseAim\\config.ini";
        auto tuning_keys = [](const std::string& path) {
            std::map<std::string, std::string> keys;
            std::ifstream in(path);
            bool on = false;
            for (std::string line; std::getline(in, line);) {
                while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
                if (!line.empty() && line[0] == '[') { on = line == "[tuning]"; continue; }
                const auto eq = line.find('=');
                if (!on || line.empty() || line[0] == ';' || eq == std::string::npos) continue;
                keys[line.substr(0, eq)] = line.substr(eq + 1);
            }
            return keys;
        };
        if (!root.empty() && std::ifstream(installed)) {
            const auto repo = tuning_keys(config), game_keys = tuning_keys(installed);
            for (const auto& [k, v] : game_keys) {
                const auto it = repo.find(k);
                if (it == repo.end() || it->second != v)
                    drift += "  " + k + ": game " + v + ", repository " + (it == repo.end() ? std::string("(unset)") : it->second) + "\n";
            }
            for (const auto& [k, v] : repo) if (!game_keys.count(k)) drift += "  " + k + ": game (unset), repository " + v + "\n";
            if (!drift.empty()) std::fprintf(stderr, "The game's [tuning] differs from the repository's:\n%s", drift.c_str());
        }
    }
    // archived configurations (dev/compare/configs/*.ini, "; label: <name>" on the first line):
    // fixed columns to compare later changes against. Keys a file leaves out take the code's
    // defaults of the day, so an archive writes out what it must keep.
    {
        std::vector<std::string> files;
        WIN32_FIND_DATAA fd;
        const HANDLE h = FindFirstFileA((out_dir + "/configs/*.ini").c_str(), &fd);
        if (h != INVALID_HANDLE_VALUE) {
            do files.push_back(fd.cFileName); while (FindNextFileA(h, &fd));
            FindClose(h);
        }
        std::sort(files.begin(), files.end());
        for (const std::string& f : files) {
            const std::string path = out_dir + "/configs/" + f;
            std::ifstream in(path); std::string first; std::getline(in, first);
            const auto at = first.find("label:");
            std::string label = at == std::string::npos ? f : first.substr(at + 6);
            while (!label.empty() && (label.front() == ' ')) label.erase(0, 1);
            while (!label.empty() && (label.back() == '\r' || label.back() == ' ')) label.pop_back();
            std::string id = "arch_" + f.substr(0, f.rfind('.'));
            for (char& c : id) if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
            const flight::Tuning t = ours_tuning(path, "", scratch);
            entries.push_back({id, label, "dev/compare/configs/" + f, [t] { return std::make_unique<Ours>(t); }});
        }
    }
    for (size_t i = 0; i < variants.size(); ++i) {
        const flight::Tuning t = ours_tuning(config, variants[i].second, scratch);
        entries.push_back({"ours_v" + std::to_string(i + 1), "试 · " + variants[i].first, variants[i].second, [t] { return std::make_unique<Ours>(t); }});
    }
    // outside references: the upstream release by default; refs=all adds upstream's 0.2.35
    // (which upstream itself rolled back) and xsd467's older pw5
    entries.push_back({"upstream_legacy", "上游 0.2.30", "FletcherMiya 052cd6a controller_mode=0", [] { return std::make_unique<UpstreamLegacy>(); }});
    // xsd467's latest, shown by default: the newest outside work to measure against
    entries.push_back({"pw11", "pw.11 · xsd467", "xsd467 05d7f48 pw5-flight-control", [] { return std::make_unique<Pw11>(); }});
    if (all_refs) {
        entries.push_back({"upstream_coord", "上游 0.2.35", "FletcherMiya 052cd6a controller_mode=1", [] { return std::make_unique<UpstreamCoordinated>(); }});
        entries.push_back({"pw5", "pw5 · xsd467", "xsd467 821f442 pw5-flight-control", [] { return std::make_unique<Pw5>(); }});
    }

    load_conditions(out_dir);
    const auto list = scenarios(out_dir);
    std::string index = "window.COMPARE_INDEX={\"controllers\":[";
    for (size_t i = 0; i < entries.size(); ++i)
        index += std::string(i ? "," : "") + "{\"id\":\"" + entries[i].id + "\",\"label\":\"" + esc(entries[i].label) + "\",\"source\":\"" + esc(entries[i].source) + "\"}";
    index += "],\"plants\":[";
    for (int p = 0; p < condition_count; ++p) {
        const auto& b = conditions[p].bars;
        index += std::string(p ? "," : "") + "{\"id\":\"" + conditions[p].id + "\",\"title\":\"" + esc(conditions[p].title) + "\",\"bars\":[" +
                 std::to_string(b[0]) + "," + std::to_string(b[1]) + "," + std::to_string(b[2]) + "]}";
    }
    index += "],\"scenarios\":[";
    bool first_listed = true;
    for (const Scenario& sc : list) {
        if (!only.empty() && sc.id.find(only) == std::string::npos) continue;
        index += std::string(first_listed ? "" : ",") + "{\"id\":\"" + sc.id + "\",\"title\":\"" + esc(sc.title) + "\",\"note\":\"" + esc(sc.note) +
                 "\",\"kind\":\"" + kind_name(sc.kind) + "\",\"seconds\":" + std::to_string(sc.seconds) +
                 ",\"switch_at\":" + std::to_string(sc.switch_at);
        if (const Approach* a = sc.approach.get()) {   // the runway, for the view: centre x, y, heading, half length, half width, ground
            char rw[160];
            std::snprintf(rw, sizeof(rw), ",\"runway\":[%.1f,%.1f,%.2f,%.0f,%.0f,%.1f]", a->cx, a->cy, a->heading, a->half_length, std::min(a->half_width, land_side), a->ground);
            index += rw;
        }
        index += "}";
        first_listed = false;
    }
    index += "],\"metrics\":{";

    // cost[scenario][condition][controller], for the scorecard
    std::map<std::string, std::vector<std::vector<float>>> cost;
    std::vector<std::string> index_parts(condition_count);
    std::vector<std::map<std::string, std::vector<float>>> cost_parts(condition_count);
    auto run_condition = [&](int p) {
        const Condition& cond = conditions[p];
        std::string& index = index_parts[p];
        std::string data = std::string("(window.COMPARE_DATA=window.COMPARE_DATA||{})[\"") + cond.id + "\"]={";
        index += std::string(p ? "," : "") + "\"" + cond.id + "\":{";
        bool first_scenario = true;
        for (const Scenario& sc : list) {
            if (!only.empty() && sc.id.find(only) == std::string::npos) continue;
            data += std::string(first_scenario ? "" : ",") + "\"" + sc.id + "\":{";
            index += std::string(first_scenario ? "" : ",") + "\"" + sc.id + "\":{";
            first_scenario = false;
            auto& row = cost_parts[p][sc.id];
            std::string kills_json = "{", shots_json = "{";
            for (size_t ci = 0; ci < entries.size(); ++ci) {
                auto ctl = entries[ci].make();
                const Run run = fly(cond, sc, *ctl, true, entries[ci].id);
                row.push_back(run.m.cost);
                const char* sep = ci ? "," : "";
                if (sc.encounter) {   // kill times per controller, for the viewer
                    kills_json += std::string(sep) + "\"" + entries[ci].id + "\":[";
                    for (size_t k = 0; k < run.kills.size(); ++k) {
                        char e[48]; std::snprintf(e, sizeof(e), "%s[%.2f,%d,%d]", k ? "," : "", run.kills[k].first, run.kills[k].second,
                                                  k < run.kill_gun.size() && run.kill_gun[k] ? 1 : 0);   // [time, enemy, by the gun]
                        kills_json += e;
                    }
                    kills_json += "]";
                    // long-range missiles: [t0, enemy, range, off, outcome, t_end, [t, x, y, z, ...]]
                    shots_json += std::string(sep) + "\"" + entries[ci].id + "\":[";
                    for (size_t k = 0; k < run.shots.size(); ++k) {
                        const Shot& sh = run.shots[k];
                        char e[96]; std::snprintf(e, sizeof(e), "%s[%.2f,%d,%.0f,%.1f,%d,%.2f,[", k ? "," : "", sh.t0, sh.target, sh.range, sh.off, sh.outcome, sh.t_end);
                        shots_json += e;
                        for (size_t j = 0; j < sh.track.size(); ++j) {
                            char num[24]; std::snprintf(num, sizeof(num), j % 4 ? "%s%.0f" : "%s%.1f", j ? "," : "", sh.track[j]);
                            shots_json += num;
                        }
                        shots_json += "]]";
                    }
                    shots_json += "]";
                }
                index += std::string(sep) + "\"" + entries[ci].id + "\":"; json_metrics(index, run.m);
                data += std::string(sep) + "\"" + entries[ci].id + "\":[";
                for (size_t k = 0; k < run.frames.size(); ++k) {
                    const Sample& f = run.frames[k];
                    data += k ? ",[" : "[";
                    for (int j = 0; j < f.n; ++j) {
                        char num[32];
                        const bool coarse = (j >= 1 && j <= 3) || (j >= 20 && j <= 22);   // positions
                        std::snprintf(num, sizeof(num), coarse ? "%s%.1f" : (j >= 7 && j <= 9) || (j >= 28 && j <= 30) || (j >= 32 && j <= 37) ? "%s%.4f" : j >= 26 ? "%s%.0f" : "%s%.2f", j ? "," : "", f.v[j]);
                        data += num;
                    }
                    data += "]";
                }
                data += "]";
            }
            if (sc.encounter) {
                // every enemy's path at 2 Hz, a boss at 10 Hz (t, x, y, z, roll, pitch, yaw, and a boss's wing sweep and fold
                // when recorded; attitude as recorded, pitch from the climb in recordings before 2026-10-05) and the kill times
                data += ",\"_kills\":" + kills_json + "},\"_long_range\":" + shots_json + "},\"_foes\":[";
                for (size_t i = 0; i < sc.encounter->foes.size(); ++i) {
                    const Foe& f = sc.encounter->foes[i];
                    char head[200];
                    std::snprintf(head, sizeof(head), "%s[\"%s\",%d,%.1f,%.1f,[", i ? "," : "", esc(f.cls).c_str(), f.boss ? 2 : f.elite ? 1 : 0, f.first, f.last);
                    data += head;
                    bool first_point = true;
                    for (float t = f.first; t <= f.last; t += f.boss ? 0.1f : 0.5f) {   // a boss at 10 Hz (its manoeuvres are seconds long)
                        const Pose e = pose_at(f.path, t);
                        char pt[128];
                        if (e.sweep >= 0)
                            std::snprintf(pt, sizeof(pt), "%s[%.1f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]", first_point ? "" : ",", t, e.pos.x, e.pos.y, e.pos.z, e.roll, e.pitch, e.yaw, e.sweep, e.fold);
                        else
                            std::snprintf(pt, sizeof(pt), "%s[%.1f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]", first_point ? "" : ",", t, e.pos.x, e.pos.y, e.pos.z, e.roll, e.pitch, e.yaw);
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
    };
    {
        std::atomic<int> next{0};
        const int workers = std::max(1, std::min<int>(condition_count, int(std::thread::hardware_concurrency())));
        std::vector<std::thread> pool;
        for (int w = 0; w < workers; ++w)
            pool.emplace_back([&] { for (int p; (p = next++) < condition_count;) run_condition(p); });
        for (auto& th : pool) th.join();
    }
    for (int p = 0; p < condition_count; ++p) {
        index += index_parts[p];
        for (auto& [scene, row] : cost_parts[p]) {
            auto& all = cost[scene];
            all.resize(condition_count);
            all[p] = row;
        }
    }
    index += "}};\n";
    std::ofstream(out_dir + "/results/index.js", std::ios::binary) << index;

    // Scorecard: per scene, one row per condition, one column per controller (the cost, lower
    // is better), then each controller's rank count and total per condition.
    std::string card = "Flight controller comparison (dev/compare): the same decisions against the same enemies.\n"
                       "Cost per scene (lower is better), one row per condition.\n\nControllers:\n";
    if (!drift.empty()) card = "WARNING: the game's [tuning] differs from the repository's (bring it in):\n" + drift + "\n" + card;
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
            std::snprintf(line, sizeof(line), "  %-10s", conditions[p].id.c_str()); card += line;
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
        std::snprintf(line, sizeof(line), "  %-10s", conditions[p].id.c_str()); card += line;
        for (float v : total[p]) { std::snprintf(line, sizeof(line), " %16.1f", v); card += line; }
        card += "\n";
    }
    header("best (*)");
    for (int p = 0; p < condition_count; ++p) {
        std::snprintf(line, sizeof(line), "  %-10s", conditions[p].id.c_str()); card += line;
        for (int v : wins[p]) { std::snprintf(line, sizeof(line), " %16d", v); card += line; }
        card += "\n";
    }
    header("roll against guidance (%, within 5 / 5-20 deg)");
    for (int p = 0; p < condition_count; ++p) {
        std::snprintf(line, sizeof(line), "  %-10s", conditions[p].id.c_str()); card += line;
        for (const Entry& e : entries) {
            const auto it = roll_fight.find(e.id + "@" + conditions[p].id);
            if (it == roll_fight.end() || it->second[1] <= 0 || it->second[3] <= 0) std::snprintf(line, sizeof(line), " %16s", "-");
            else std::snprintf(line, sizeof(line), " %9.1f /%5.1f", 100 * it->second[0] / it->second[1], 100 * it->second[2] / it->second[3]);
            card += line;
        }
        card += "\n";
    }
    header("pilot speed in fights (throttle/neutral/brake/high-G %, median km/h)");
    for (int p = 0; p < condition_count; ++p) {
        std::snprintf(line, sizeof(line), "  %-10s", conditions[p].id.c_str()); card += line;
        for (const Entry& e : entries) {
            auto it = pilot_speed.find(e.id + "@" + conditions[p].id);
            if (it == pilot_speed.end() || it->second.total <= 0) { std::snprintf(line, sizeof(line), " %26s", "-"); card += line; continue; }
            SpeedLog& sl = it->second;
            std::nth_element(sl.kmh.begin(), sl.kmh.begin() + sl.kmh.size() / 2, sl.kmh.end());
            const double T = sl.total / 100;
            std::snprintf(line, sizeof(line), " %3.0f/%3.0f/%2.0f/%2.0f %5.0f", sl.mode[0] / T, sl.mode[1] / T, sl.mode[2] / T, sl.mode[3] / T,
                          sl.kmh.empty() ? 0.0f : sl.kmh[sl.kmh.size() / 2]);
            card += line;
        }
        card += "\n";
    }
    header("stall (s, all scenes)");
    for (int p = 0; p < condition_count; ++p) {
        std::snprintf(line, sizeof(line), "  %-10s", conditions[p].id.c_str()); card += line;
        for (const Entry& e : entries) {
            const auto it = stall_time.find(e.id + "@" + conditions[p].id);
            std::snprintf(line, sizeof(line), " %16.1f", it == stall_time.end() ? 0.0 : it->second); card += line;
        }
        card += "\n";
    }
    // Summaries. Scene costs mix seconds, degrees, kill shares and stick reversals with rough
    // weights, so their plain sum is no verdict: each controller is put against the 10-05 baseline
    // scene by scene, (cost + 1) / (baseline + 1), and those ratios are averaged geometrically
    // (< 1 better), a battle flown in several orders (M28) counted once. Shown three ways (every
    // scene alike; every kind of scene alike; the mean rank) for every aircraft and overall.
    struct Summary { std::vector<double> per_scene, per_kind, mean_rank; std::vector<std::vector<double>> per_plant; };
    const size_t n_ctl = entries.size();
    size_t base_ctl = 0;
    for (size_t i = 0; i < n_ctl; ++i) if (entries[i].id == "arch_baseline_20261005") base_ctl = i;
    auto summarize = [&](const std::map<std::string, std::vector<std::vector<float>>>& costs) {
        Summary S;
        S.per_plant.assign(condition_count, std::vector<double>(n_ctl, 1.0));
        std::vector<double> log_scene(n_ctl, 0), log_kind(n_ctl, 0), rank_sum(n_ctl, 0);
        int plants = 0, ranked = 0;
        for (int p = 0; p < condition_count; ++p) {
            std::map<std::string, std::pair<Kind, std::vector<double>>> groups;
            std::map<std::string, int> counts;
            for (const Scenario& sc : list) {
                const auto it = costs.find(sc.id);
                if (it == costs.end() || int(it->second.size()) <= p || it->second[p].size() < n_ctl) continue;
                const std::string g = sc.group.empty() ? sc.id : sc.group;
                auto& gr = groups[g];
                gr.first = sc.kind; gr.second.resize(n_ctl, 0);
                for (size_t i = 0; i < n_ctl; ++i) gr.second[i] += it->second[p][i];
                ++counts[g];
            }
            if (groups.empty()) continue;
            std::vector<double> lp(n_ctl, 0);
            std::map<Kind, std::pair<std::vector<double>, int>> kinds;
            for (auto& [g, gr] : groups) {
                auto& v = gr.second;
                for (auto& x : v) x /= counts[g];
                auto& kk = kinds[gr.first];
                kk.first.resize(n_ctl, 0); ++kk.second;
                for (size_t i = 0; i < n_ctl; ++i) {
                    const double r = std::log((v[i] + 1) / (v[base_ctl] + 1));
                    lp[i] += r; kk.first[i] += r;
                    int better = 0;
                    for (size_t j = 0; j < n_ctl; ++j) if (v[j] < v[i] - 1e-6) ++better;
                    rank_sum[i] += better + 1;
                }
                ++ranked;
            }
            for (size_t i = 0; i < n_ctl; ++i) {
                S.per_plant[p][i] = std::exp(lp[i] / groups.size());
                log_scene[i] += lp[i] / groups.size();
                double lk = 0;
                for (auto& [k, kk] : kinds) lk += kk.first[i] / kk.second;
                log_kind[i] += lk / kinds.size();
            }
            ++plants;
        }
        S.per_scene.resize(n_ctl); S.per_kind.resize(n_ctl); S.mean_rank.resize(n_ctl);
        for (size_t i = 0; i < n_ctl; ++i) {
            S.per_scene[i] = std::exp(log_scene[i] / std::max(plants, 1));
            S.per_kind[i] = std::exp(log_kind[i] / std::max(plants, 1));
            S.mean_rank[i] = rank_sum[i] / std::max(ranked, 1);
        }
        return S;
    };
    // the cost of every scene in a pass with another player profile (no frames kept), aircraft in parallel
    auto pass_costs = [&]() {
        std::vector<std::map<std::string, std::vector<float>>> parts(condition_count);
        std::atomic<int> next{0};
        const int workers = std::max(1, std::min<int>(condition_count, int(std::thread::hardware_concurrency())));
        std::vector<std::thread> pool;
        for (int w = 0; w < workers; ++w)
            pool.emplace_back([&] {
                for (int p; (p = next++) < condition_count;)
                    for (const Scenario& sc : list) {
                        if (!only.empty() && sc.id.find(only) == std::string::npos) continue;
                        auto& row = parts[p][sc.id];
                        for (size_t ci = 0; ci < n_ctl; ++ci) {
                            auto ctl = entries[ci].make();
                            row.push_back(fly(conditions[p], sc, *ctl, false, entries[ci].id).m.cost);
                        }
                    }
            });
        for (auto& th : pool) th.join();
        std::map<std::string, std::vector<std::vector<float>>> out;
        for (int p = 0; p < condition_count; ++p)
            for (auto& [scene, row] : parts[p]) { auto& all = out[scene]; all.resize(condition_count); all[p] = row; }
        return out;
    };
    const Summary main_summary = summarize(cost);
    auto row_of = [&](const char* label, const std::vector<double>& v, const char* fmt) {
        std::snprintf(line, sizeof(line), "  %-10s", label); card += line;
        for (double x : v) { std::snprintf(line, sizeof(line), fmt, x); card += line; }
        card += "\n";
    };
    header("summary (vs 10-05 baseline, geometric mean of (cost+1) ratios, < 1 better)");
    row_of("per scene", main_summary.per_scene, " %16.3f");
    row_of("per kind", main_summary.per_kind, " %16.3f");
    row_of("mean rank", main_summary.mean_rank, " %16.2f");
    header("per aircraft (per scene, vs baseline)");
    for (int p = 0; p < condition_count; ++p) row_of(conditions[p].id.c_str(), main_summary.per_plant[p], " %16.3f");
    std::string summary_js = "window.COMPARE_SUMMARY={\"controllers\":[";
    for (size_t i = 0; i < n_ctl; ++i) summary_js += std::string(i ? "," : "") + "\"" + entries[i].id + "\"";
    auto arr = [](const std::vector<double>& v) {
        std::string a = "[";
        for (size_t i = 0; i < v.size(); ++i) { char b[32]; std::snprintf(b, sizeof(b), "%s%.4f", i ? "," : "", v[i]); a += b; }
        return a + "]";
    };
    summary_js += "],\"per_scene\":" + arr(main_summary.per_scene) + ",\"per_kind\":" + arr(main_summary.per_kind) +
                  ",\"mean_rank\":" + arr(main_summary.mean_rank) + ",\"per_plant\":{";
    for (int p = 0; p < condition_count; ++p) summary_js += std::string(p ? "," : "") + "\"" + conditions[p].id + "\":" + arr(main_summary.per_plant[p]);
    if (!long_range_log.empty()) {
        // long-range missiles by launch range, all aircraft, the standard pass: launched, and the share that hit
        // (the rest lost beyond 117 deg, out of time, or the enemy gone meanwhile)
        header("Long-range missiles by launch range (all aircraft; launched, hit %, lost %)");
        static const char* bins[n_shot_bins] = {"<500 m", "500-800", "800-1200", "1.2-2 km", "2-4 km", "4 km+"};
        summary_js += "},\"long_range\":{";
        bool first_ctl = true;
        for (size_t i = 0; i < n_ctl; ++i) {
            auto it = long_range_log.find(entries[i].id);
            if (it == long_range_log.end()) continue;
            summary_js += std::string(first_ctl ? "" : ",") + "\"" + entries[i].id + "\":["; first_ctl = false;
            std::snprintf(line, sizeof(line), "  %-24s", entries[i].id.c_str()); card += line;
            for (int b = 0; b < n_shot_bins; ++b) {
                const auto& o = it->second[b];
                int n = 0; for (int x : o) n += x;
                std::snprintf(line, sizeof(line), " %s %4d %3.0f%% %3.0f%% |", bins[b], n, n ? 100.0 * o[1] / n : 0.0, n ? 100.0 * o[2] / n : 0.0);
                card += line;
                char a[96]; std::snprintf(a, sizeof(a), "%s[%d,%d,%d,%d,%d,%d]", b ? "," : "", o[0], o[1], o[2], o[3], o[4], o[5]);
                summary_js += a;
            }
            card += "\n"; summary_js += "]";
        }
    }
    summary_js += "},\"profiles\":[";
    if (robust && only.empty()) {
        // other players, other luck: the ranking should hold (the standard profile is the one tuned on)
        header("robustness (per scene vs baseline, by player profile; tuned on std only)");
        row_of(pilot_profiles[0].id, main_summary.per_scene, " %16.3f");
        summary_js += std::string("{\"id\":\"") + pilot_profiles[0].id + "\",\"title\":\"" + esc(pilot_profiles[0].title) + "\",\"per_scene\":" + arr(main_summary.per_scene) +
                      ",\"mean_rank\":" + arr(main_summary.mean_rank) + "}";
        for (size_t k = 1; k < sizeof(pilot_profiles) / sizeof(pilot_profiles[0]); ++k) {
            pilot_profile = pilot_profiles[k];
            const Summary S = summarize(pass_costs());
            row_of(pilot_profiles[k].id, S.per_scene, " %16.3f");
            summary_js += std::string(",{\"id\":\"") + pilot_profiles[k].id + "\",\"title\":\"" + esc(pilot_profiles[k].title) + "\",\"per_scene\":" + arr(S.per_scene) +
                          ",\"mean_rank\":" + arr(S.mean_rank) + "}";
        }
        pilot_profile = pilot_profiles[0];
        card += "  profiles:\n";
        for (const PilotProfile& pp : pilot_profiles) card += std::string("    ") + pp.id + "  " + pp.title + "\n";
    }
    summary_js += "]};\n";
    std::ofstream(out_dir + "/results/summary.js", std::ios::binary) << summary_js;
    std::fputs(card.c_str(), stdout);
    if (only.empty() && variants.empty()) std::ofstream(out_dir + "/scorecard.txt", std::ios::binary) << card;
    return 0;
}
