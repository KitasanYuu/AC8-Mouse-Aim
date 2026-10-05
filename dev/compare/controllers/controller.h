#pragma once
// The interface every compared flight controller is adapted to. A controller sees what
// the mod host gives it in game, once per frame, and returns stick values.
#include <memory>
#include <string>

namespace bench {
struct Sense {
    float pitch, yaw, roll;                   // attitude, degrees (Unreal: X north, Y east, Z up)
    float aim[3];                             // world direction the mouse points at (unit)
    float pitch_rate, yaw_rate, roll_rate;    // body rates, deg/s, filtered as the host does
    float dt;
    float throttle, brake;                    // the game's own inputs (0..1); both = high-G turn
    float vel[3], acc[3], altitude;           // flight path (m/s, m/s^2, m)
};
// Controller convention: pitch + pulls the nose up, roll + rolls right, yaw + yaws right.
struct Stick { float pitch, roll, yaw; };
struct Controller {
    virtual ~Controller() = default;
    virtual void reset() = 0;                 // new flight: forget everything
    virtual Stick step(const Sense&) = 0;
    virtual const char* trace() const { return ""; }   // the last step's internals, for BENCH_TRACE
};
}
