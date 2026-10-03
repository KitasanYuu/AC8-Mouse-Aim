#pragma once
#include "flight_math.h"
// Maneuver paradigm; see docs/maneuver-spec.md. War Thunder style mouse aim: bring
// the nose to the target by the quickest roll/pitch combination the aircraft can fly.
// No protective limits: the player's aim is executed as given.
namespace flight {
inline float smoothstep(float edge0,float edge1,float x) {
    const float t=std::clamp((x-edge0)/(edge1-edge0),0.0f,1.0f);
    return t*t*(3-2*t);
}
// Roll (degrees, positive right) that places body-plane vector (x right, y up) overhead.
inline float roll_toward(float x,float y) {
    return x*x+y*y<1e-8f ? 0.0f : std::atan2(x,y)/rad;
}
// Near +-180 degrees either roll direction is equivalent; keep the chosen one
// until the remaining roll is unambiguous again.
inline float latch_roll(float error,int& side,float limit) {
    if(std::abs(error)<=limit) {
        if(std::abs(error)>0.5f) side=error>0?1:-1;
        return error;
    }
    if((error>0?1:-1)!=side) error+=360.0f*side;
    return error;
}
// Measured aircraft response (calibration 2026-10-03, build 25201480, level flight).
// The stick passes a pure delay, then the game's own input smoothing (first order,
// *_input seconds), then the aircraft.
// Pitch: the rate approaches its steady value with time constant tau. Steady rate is
// pull*|f|^curve or push*|f|^curve (f = smoothed input), minus gravity*upright, where
// upright is the canopy's world-up component (gravity drops the nose when upright).
// Roll: the rate accelerates at accel*|f|^curve against damping rate/tau, up to
// max_rate (refit on in-game rolls 2026-10-03: sustained fast rolls need near-full stick).
struct Plant {
    float pitch_delay=0.24f, pitch_input=0.36f, pitch_tau=0.40f, pitch_curve=1.1f;
    float pitch_pull=61.0f, pitch_push=26.0f, pitch_gravity=9.0f; // deg/s
    float roll_delay=0.16f, roll_input=0.06f, roll_accel=220.0f, roll_curve=1.3f;
    float roll_max_rate=150.0f, roll_tau=0.8f;
    float pull_rate(float upright) const { return std::max(5.0f,pitch_pull-pitch_gravity*upright); }
    float push_rate(float upright) const { return std::max(5.0f,pitch_push+pitch_gravity*upright); }
    // Time to roll through deg from rest: accelerate, cruise at max rate, brake.
    float roll_time(float deg) const {
        deg=std::abs(deg);
        const float cap_distance=roll_max_rate*roll_max_rate/roll_accel;
        return roll_delay+roll_input+(deg<cap_distance ? 2*std::sqrt(deg/roll_accel)
                                            : 2*roll_max_rate/roll_accel+(deg-cap_distance)/roll_max_rate);
    }
    // Time to pitch through deg at a steady rate, including the build-up lag.
    float pitch_time(float deg,float rate) const { return pitch_delay+pitch_input+pitch_tau+std::abs(deg)/rate; }
};
struct Guidance {
    float angle=0;        // nose-to-target, degrees
    float roll_error=0;   // positive rolls right
    float pitch_error=0;  // positive raises the nose
    float yaw_error=0;    // positive yaws right
    float turn_weight=0;  // 0 fine tracking, 1 lift-vector turn
    bool tail=false;      // committed rear-target reversal
    bool pushing=false;   // aligning the floor, not the canopy, with the target
};
// Thresholds in degrees (choice_margin in seconds); documented in
// docs/maneuver-spec.md and tunable via config.ini.
struct ManeuverTuning {
    float near_end=3.0f, far_start=20.0f;          // near-field banking -> lift alignment
    float near_bank_gain=3.0f, near_bank_max=60.0f; // bank degrees per degree of offset
    float push_enter=120.0f, push_exit=135.0f;     // largest errors always roll and pull
    float choice_margin=0.25f;                     // seconds saved before switching pull/push
    float pull_full=60.0f, pull_none=120.0f;
    float tail_enter=160.0f, tail_exit=145.0f, tail_bank=50.0f;
    float latch_limit=150.0f;
    // Lead the roll-out: start leveling when the time left to reach the target falls
    // below this multiple of the time the roll-out needs (0 = level after arrival).
    // 1.5 finished lateral turns (on target and wings level) up to 1.7 s sooner in sim.
    float rollout_lead=1.5f;
};
struct Maneuver {
    ManeuverTuning t;
    bool tail=false, pushing=false, committed=false;
    int side=1, tail_side=1, roll_side=1;
    void reset() { tail=false; pushing=false; committed=false; side=1; tail_side=1; roll_side=1; }
    float pull_gate(float roll_remaining) const {
        // Roll first, then pull (or push); never pitch away from the target.
        return 1.0f-smoothstep(t.pull_full,t.pull_none,std::abs(roll_remaining));
    }
    // pitch_rate: measured body pitch rate (deg/s), used to time the roll-out.
    Guidance step(const Basis& b,V aim,const Plant& plant=Plant{},float pitch_rate=0) {
        Guidance g;
        const float fx=std::clamp(dot(aim,b.f),-1.0f,1.0f), rx=dot(aim,b.r), ux=dot(aim,b.u);
        g.angle=std::acos(fx)/rad;
        if(std::abs(rx)>0.02f) side=rx>0?1:-1;
        // World up seen from the cockpit. Its length is cos(nose pitch), so the
        // horizon reference fades out as the nose approaches vertical.
        const float hx=b.r.z, hy=b.u.z, hn=std::sqrt(hx*hx+hy*hy);
        const float horizon=smoothstep(0.2f,0.5f,hn);
        if(!tail && g.angle>t.tail_enter) { tail=true; tail_side=side; }
        else if(tail && g.angle<t.tail_exit) tail=false;
        g.tail=tail;
        if(tail) {
            pushing=false;
            committed=true;
            // Target behind the tail: commit to an upward oblique reversal.
            const float bank=std::atan2(hx,hy)+tail_side*t.tail_bank*rad;
            g.turn_weight=1;
            g.roll_error=latch_roll(roll_toward(std::sin(bank)*horizon,std::cos(bank)*horizon),
                                    roll_side,t.latch_limit);
            g.pitch_error=g.angle*pull_gate(g.roll_error);
            return g;
        }
        const float pull_roll=roll_toward(rx,ux), push_roll=roll_toward(-rx,-ux);
        // Canopy (pull) or floor (push) toward the target, whichever the measured
        // aircraft finishes sooner: roll time plus pitch time at the rate available
        // in the attitude after the roll. Pushing is ~2/3 of pulling upright and
        // weaker still inverted, where gravity works against it.
        auto upright_after=[&](float roll) {
            const float a=roll*rad;
            return b.u.z*std::cos(a)+b.r.z*std::sin(a);
        };
        const float pull_time=plant.roll_time(pull_roll)+
            plant.pitch_time(g.angle,plant.pull_rate(upright_after(pull_roll)));
        const float push_time=plant.roll_time(push_roll)+
            plant.pitch_time(g.angle,plant.push_rate(upright_after(push_roll)));
        // Push only toward targets below the wing plane: never roll away from a
        // target beside the aircraft in order to push (logged as rocking).
        if(g.angle>t.push_exit || std::abs(push_roll)>85.0f) pushing=false;
        else if(pushing) { if(pull_time+t.choice_margin<push_time) pushing=false; }
        else if(g.angle<t.push_enter && std::abs(push_roll)<70.0f &&
                push_time+t.choice_margin<pull_time) pushing=true;
        const float s=pushing?-1.0f:1.0f, lift=pushing?push_roll:pull_roll;
        // Near field: bank in proportion to the target's offset across the horizon.
        // It is measured in world terms, so banking cannot cancel its own demand,
        // and the wings come level as the offset closes; pitch and rudder finish.
        float lateral=std::atan2(aim.y,aim.x)-std::atan2(b.f.y,b.f.x);
        lateral=std::atan2(std::sin(lateral),std::cos(lateral))*std::sqrt(std::max(0.0f,1-aim.z*aim.z))/rad;
        const float bank=std::clamp(lateral*t.near_bank_gain,-t.near_bank_max,t.near_bank_max);
        const float near_roll=std::atan2(hx,hy)+bank*rad;
        // A maneuver begun from a large error keeps its pull/push alignment all the way
        // in and levels only once on target; leveling on the way in rolls the lift off
        // the target and costs a second correction (reported). The same applies when
        // inverted, where leveling would be a half roll before arrival. Near-field
        // banking is for small corrections that start close to the target.
        // On arrival, hold the bank while pitch is still moving: rolling then would turn
        // the remaining pitch motion into a sideways drift off the target (simulated and
        // logged). Once pitch has stopped, rolling level does not move the nose.
        const float moving=smoothstep(4.0f,12.0f,std::abs(pitch_rate));
        if(g.angle>t.far_start) committed=true;
        else if(g.angle<1.0f && moving<0.01f) committed=false;
        const float direct=std::max(smoothstep(0.3f,-0.3f,hy),committed?1.0f:0.0f);
        float w=smoothstep(t.near_end,t.far_start,g.angle)*(1-direct)+
                smoothstep(1.0f,t.near_end,g.angle)*direct;
        const float ln=std::sqrt(rx*rx+ux*ux);
        if(committed && t.rollout_lead>0) {
            // Time to arrive at the current pitch rate vs time to roll level from here.
            const float arrive=g.angle/std::max(std::abs(pitch_rate),1.0f);
            const float rollout=plant.roll_time(std::atan2(hx,hy)/rad)*t.rollout_lead;
            w=std::min(w,smoothstep(0.5f*rollout,rollout,arrive));
        }
        const float hold=(1-w)*direct*moving;
        // Blend near-field, hold-bank and lift-alignment roll targets as directions, so
        // the blend cannot wrap through the opposite roll.
        float x=std::sin(near_roll)*horizon*(1-w-hold), y=std::cos(near_roll)*horizon*(1-w-hold)+hold;
        if(ln>1e-6f) { x+=s*rx/ln*w; y+=s*ux/ln*w; }
        g.turn_weight=w;
        g.pushing=pushing;
        g.roll_error=latch_roll(roll_toward(x,y),roll_side,t.latch_limit);
        const float plane=std::atan2(ux,fx)/rad;
        // Away from the target, pitch only in the chosen direction while the roll lines up.
        const float along=pushing?std::min(plane,0.0f):std::max(plane,0.0f);
        g.pitch_error=plane*(1-w)+along*w*pull_gate(lift);
        g.yaw_error=std::atan2(rx,fx)/rad*(1-w);
        return g;
    }
};
}
