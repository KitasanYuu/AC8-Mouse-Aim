#pragma once
#include "flight_math.h"
// Maneuver paradigm; see docs/maneuver-spec.md. War Thunder style mouse aim: bring
// the nose to the target by the quickest roll/pitch combination the aircraft can fly.
// No protective limits: the player's aim is executed as given.
namespace flight {
#ifdef MANEUVER_DEBUG
inline bool maneuver_debug=false;
#endif
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
    // The game smooths a stick moving back toward or past centre much faster than one
    // building up: 120 logged full-push stops covered 0.72 of the distance the build-up
    // smoothing predicts; a release time constant of ~0.1 s fits them.
    float pitch_release=0.10f;
    // Gravity was first guessed at 9 deg/s, then 1.5 from regressions. Upright with the
    // stick centred the game holds the nose (median -0.2 deg/s over ~1900 logged frames),
    // and 1.5 left the nose settled 0.4-0.7 deg above the target: the model expected it
    // to sink and held a little pull against a sink that never came.
    float pitch_pull=61.0f, pitch_push=17.0f, pitch_gravity=0.3f; // deg/s
    // Push authority depends strongly on attitude (negative-G limited): full push held
    // a median 27 deg/s upright, 20 on the side, 14 at 120 deg bank and ~0 inverted in
    // combat logs, while pull changed only 36-48. Push authority = pitch_push +
    // push_gravity * upright, on top of pitch_gravity.
    float push_gravity=6.5f;
    float roll_delay=0.16f, roll_input=0.06f, roll_accel=220.0f, roll_curve=1.3f;
    float roll_max_rate=150.0f, roll_tau=0.8f;
    float pull_rate(float upright) const { return std::max(5.0f,pitch_pull-pitch_gravity*upright); }
    float push_authority(float upright) const { return std::max(2.0f,pitch_push+push_gravity*upright); }
    float push_rate(float upright) const { return std::max(2.0f,push_authority(upright)+pitch_gravity*upright); }
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
    float yaw_weight=0;   // how much rudder to use (near field, or a bank-limited turn)
    float turn_weight=0;  // 0 fine tracking, 1 lift-vector turn
    bool tail=false;      // committed rear-target reversal
    bool pushing=false;   // aligning the floor, not the canopy, with the target
};
// Thresholds in degrees (choice_margin in seconds); documented in
// docs/maneuver-spec.md and tunable via config.ini.
struct ManeuverTuning {
    // Pitch and rudder: from far_start the pull follows the lift (along it, in proportion
    // to how well the roll has aligned it); inside near_end, the target's offset along
    // the canopy and the wing. Blended in between.
    float near_end=3.0f, far_start=20.0f;
    // Wings level on arrival: the bank may be at most level_per_deg per degree still to
    // go, and no more than can be rolled off at rollout_rate by the time the nose
    // arrives (after rollout_lag).
    // Inside level_inside deg the wings are simply held level: the direction to a target
    // that close swings from side to side and the bank with it.
    float level_inside=1.0f;
    float level_per_deg=20.0f, rollout_rate=0.0f, rollout_lag=0.25f;
    // Roll toward where a moving target will be this far ahead (s).
    float pursuit_ahead=0.0f;
    // A target moving faster than pursuit_speed deg/s (fully at twice it) is chased: the
    // pull may slice (chase_slice = 1) and is not traded for a push. A chased target has
    // no arrival to level the wings for: chase_level = 1 lifts the level_per_deg limit for
    // it (logged gun attacks: held off the lift by that limit, the pull dragged the nose
    // past the enemy, which slid to the side of the canopy and stayed there).
    float pursuit_speed=4.0f, chase_slice=0.0f, chase_level=0.0f;
    float push_enter=120.0f, push_exit=135.0f;     // largest errors always roll and pull
    // A push is used only for targets close to straight below the floor: at most this
    // much roll to bring the floor onto the target (enter / give up).
    float push_roll_enter=30.0f, push_roll_exit=50.0f;
    // A target more than push_below deg beyond the bank a pull may use (exit at
    // push_below_exit) is pushed for, wings toward level.
    float push_below=60.0f, push_below_exit=45.0f;
    // Near the ground, sinking: within ground_guard seconds of reaching sea level at the present
    // sink rate, no push unless its force points up (belly-up); the target is reached by roll
    // and pull. The aim stays the player's; only the way to it changes. Logged 2026-10-05:
    // at 650 m/s, 480 m and -280 m/s the aim rose while 80 deg banked, and the push chosen to
    // reach it sideways cost the 0.5 s that would have pulled out. 0 = off. (Altitude is above
    // sea level: over high ground it does not help, and does not get in the way.)
    float ground_guard=5.0f;
    float choice_margin=0.6f;                      // seconds a push must lose by to switch to a pull
    float push_bias=0.3f;                          // seconds a push may be slower and still be chosen
    float tail_enter=160.0f, tail_exit=145.0f, tail_bank=50.0f;
    float latch_limit=150.0f;
    // Pitch starts before the roll is done: the roll expected over the pitch lag is
    // credited, at most overlap_max degrees, fading out within 15 deg of the target
    // (none within 5), where a misaligned pull swings the target around the nose.
    float overlap_max=30.0f;
    float upright=1;                          // upright preference (0 = exact lift alignment)
    // Most bank for a pull toward a target below the turn: slice_bank for small
    // corrections (no belly-up for a slight down-left adjustment), slice_deep from twice
    // slice_from (a nose-low slice in a big turn), and fully inverted only for targets
    // from invert_min_angle (all the way 30 deg further), behind and low.
    float slice_bank=90.0f, slice_deep=135.0f, slice_from=20.0f;
    float invert_min_angle=120.0f;
    float slice_ease=10.0f, slice_hold=135.0f;
};
struct Maneuver {
    ManeuverTuning t;
    bool tail=false, pushing=false;
    int limit_side=0;                      // side of a bank-limited lift, held near straight below
    int side=1, tail_side=1, roll_side=1;
    float last_angle=-1, closing=0;        // closing rate on the target, deg/s
    void reset() { tail=false; pushing=false; limit_side=0; side=1; tail_side=1; roll_side=1; last_angle=-1; closing=0; }
    // One rule for every distance, decided each frame from the current geometry: roll
    // the canopy (or, for targets straight below, the floor) toward the target within the
    // bank limits, pitch along it, rudder for the rest. The only memory is hysteresis on
    // two-way choices (push or pull, roll direction near 180 deg, the side of a limited
    // lift, the tail reversal side); nothing latches a maneuver, so a new target takes
    // effect at once (reported: a change of intent had to wait until the previous
    // maneuver was finished).
    // pitch_coming: nose-up pitch (deg) still certain to happen from commands in flight
    // plus the shortest stop; roll_rate: measured (deg/s); lead_u/lead_r: how fast the
    // target itself moves across the nose, along the canopy and right wing (deg/s).
    Guidance step(const Basis& b,V aim,const Plant& plant=Plant{},float pitch_coming=0,float roll_rate=0,
                  float lead_u=0,float lead_r=0,float dt=0,float ground_time=1e9f) {
        (void)pitch_coming;
        Guidance g;
        const float fx=std::clamp(dot(aim,b.f),-1.0f,1.0f), rx=dot(aim,b.r), ux=dot(aim,b.u);
        g.angle=std::acos(fx)/rad;
        // Closing rate on the target (deg/s), low-passed.
        if(dt>0 && last_angle>=0)
            closing+=((last_angle-g.angle)/dt-closing)*(1-std::exp(-dt/0.15f));
        last_angle=g.angle;
        if(std::abs(rx)>0.02f) side=rx>0?1:-1;
        // World up seen from the cockpit. Its length is cos(nose pitch), so the
        // horizon reference fades out as the nose approaches vertical.
        const float hx=b.r.z, hy=b.u.z, hn=std::sqrt(hx*hx+hy*hy);
        const float horizon=smoothstep(0.2f,0.5f,hn);
        const float up=std::atan2(hx,hy);
        const float lag=plant.pitch_delay+plant.pitch_input;
        g.yaw_weight=1;
        if(!tail && g.angle>t.tail_enter) { tail=true; tail_side=side; }
        else if(tail && g.angle<t.tail_exit) tail=false;
        g.tail=tail;
        if(tail) {
            // Target behind the tail: an upward oblique reversal on the side it was on.
            pushing=false;
            const float bank=up+tail_side*t.tail_bank*rad;
            g.turn_weight=1;
            g.roll_error=latch_roll(roll_toward(std::sin(bank)*horizon,std::cos(bank)*horizon),
                                    roll_side,t.latch_limit);
            const float credit=std::min(std::max(0.0f,roll_rate*(g.roll_error>=0?1.0f:-1.0f))*lag,t.overlap_max);
            g.pitch_error=g.angle*std::max(0.0f,std::cos(std::max(0.0f,std::abs(g.roll_error)-credit)*rad));
            return g;
        }
        const float target_speed=std::hypot(lead_u,lead_r);
        const float chasing=smoothstep(t.pursuit_speed,2*t.pursuit_speed,target_speed);
        // Lift toward the target, or toward where a moving one will be pursuit_ahead from now.
        const float ahead=t.pursuit_ahead*rad;
        const float rxa=rx+lead_r*ahead, uxa=ux+lead_u*ahead, lna=std::sqrt(rxa*rxa+uxa*uxa);
        float lx=0, ly=1;
        if(lna>1e-6f) { lx=rxa/lna; ly=uxa/lna; }
        // Canopy (pull) or floor (push) toward the target, whichever the measured aircraft
        // finishes sooner, counting the roll back upright if it ends belly-up. A push only
        // for targets close to straight below the floor.
        auto upright_after=[&](float roll) {
            const float a=roll*rad;
            return b.u.z*std::cos(a)+b.r.z*std::sin(a);
        };
        auto recover=[&](float roll) {
            const float bank=std::acos(std::clamp(upright_after(roll)/std::max(hn,1e-3f),-1.0f,1.0f))/rad;
            return plant.roll_time(bank)*smoothstep(60.0f,120.0f,bank)*horizon;
        };
        const float pull_roll=roll_toward(lx,ly), push_roll=roll_toward(-lx,-ly);
        const float pull_time=plant.roll_time(pull_roll)+
            plant.pitch_time(g.angle,plant.pull_rate(upright_after(pull_roll)))+recover(pull_roll);
        const float push_time=plant.roll_time(push_roll)+
            plant.pitch_time(g.angle,plant.push_rate(upright_after(push_roll)))+recover(push_roll);
        // Upright preference for the pull (see slice_*); near vertical the horizon means
        // nothing and the limit fades out.
        float pull_limit=180;
        if(t.upright>0) {
            // A moving target is pursued with the full slice at any distance: a descending
            // turning target needs more than 90 deg of bank to keep the lift on it.
            const float turn=t.slice_bank+(t.slice_deep-t.slice_bank)*
                std::max(smoothstep(t.slice_from,2*t.slice_from,g.angle),chasing*t.chase_slice);
            const float deep=turn+(180-turn)*smoothstep(t.invert_min_angle,t.invert_min_angle+30,g.angle);
            pull_limit=deep+(180-deep)*(1-horizon);
        }
        // Bank the canopy would need to face the target.
        float lift_bank=std::atan2(lx,ly)-up;
        lift_bank=std::atan2(std::sin(lift_bank),std::cos(lift_bank))/rad;
        // A slice already flown is eased off, not cut: the limit by angle drops as the
        // nose closes in, and cutting to it rolled a 130 deg slice back level and pushed
        // halfway through the turn (sim: stopped ~14 deg out for a second). The bank
        // already held, less slice_ease, stays allowed (less, so a roll overshoot past the
        // limit cannot ratchet itself further), up to slice_deep.
        if(t.upright>0) {
            float bank_now=-up/rad;
            if(bank_now*lift_bank>0)
                pull_limit=std::max(pull_limit,std::min(std::abs(bank_now),t.slice_hold)-t.slice_ease);
        }
        // Keep the side of a limited lift while the target is near straight below it,
        // where the sign of the bank flips back and forth.
        if(!pushing && (std::abs(lift_bank)<150.0f || !limit_side)) limit_side=lift_bank>=0?1:-1;
        // How far the target is from the lift the pull may use (on the side kept).
        float off_lift=0;
        if(std::abs(lift_bank)>pull_limit) {
            off_lift=lift_bank-limit_side*pull_limit;
            off_lift=std::abs(std::atan2(std::sin(off_lift*rad),std::cos(off_lift*rad)))/rad;
        }
        // A target well off the lift the pull may use is pushed for: wings toward level and
        // the floor toward it. Holding the limited lift there left the target off it with
        // only the rudder to move the nose (sim: stuck ~6 deg out at 90 deg of bank, and
        // 30 deg out for 2 s in a slice with the target passing straight below). Otherwise
        // a push only for targets close to straight below the floor, when quicker counting
        // the roll back upright.
        const bool below=horizon>0.5f && chasing<0.5f && off_lift>(pushing?t.push_below_exit:t.push_below);
        const bool may_push=g.angle<t.push_enter && std::abs(push_roll)<t.push_roll_enter;
        if(g.angle>t.push_exit) pushing=false;
        else if(below && g.angle<(pushing?t.push_exit:t.push_enter)) pushing=true;
        else if(std::abs(push_roll)>t.push_roll_exit) pushing=false;
        else if(pushing) { if(pull_time+t.choice_margin<push_time) pushing=false; }
        else pushing=may_push && push_time<pull_time+t.push_bias;
        if(pushing && t.ground_guard>0 && ground_time<t.ground_guard && upright_after(push_roll)>-0.2f) pushing=false;
        // Direction the canopy should point (the floor faces the target when pushing).
        float cx=pushing?-lx:lx, cy=pushing?-ly:ly;
        if(horizon>0) {
            float bank=std::atan2(cx,cy)-up;
            bank=std::atan2(std::sin(bank),std::cos(bank))/rad;
            float limit=pushing?180.0f:pull_limit;
            // Level on arrival: within the last few degrees the bank still follows the
            // target's offset, so a sideways offset is closed by bank and pull rather than
            // by the weak rudder alone (logged: 5 deg sideways took 2-3 s with the wings
            // nearly level).
            if(t.level_per_deg>0) {
                const float level=t.level_per_deg*std::max(0.0f,g.angle-t.level_inside)/std::max(horizon,1e-3f);
                limit=std::min(limit,level+std::max(0.0f,180.0f-level)*chasing*t.chase_level);
            }
            const float to_go=g.angle/std::max(closing,1.0f);
            if(t.rollout_rate>0) limit=std::min(limit,t.rollout_rate*std::max(0.0f,to_go-t.rollout_lag)/std::max(horizon,1e-3f));
            if(std::abs(bank)>limit) {
                const int clamp_side=pushing?(bank>=0?1:-1):limit_side;
                const float a=up+clamp_side*limit*rad;
                cx=std::sin(a); cy=std::cos(a);
            }
        }
        g.pushing=pushing;
        g.roll_error=latch_roll(roll_toward(cx,cy),roll_side,t.latch_limit);
#ifdef MANEUVER_DEBUG
        if(maneuver_debug) std::printf("  dbg ang=%.1f horizon=%.2f bank_now=%.1f lift_bank=%.1f pull_limit=%.1f limit_side=%d push=%d push_roll=%.1f c=(%.2f,%.2f) l=(%.2f,%.2f)\n",
            g.angle,horizon,-up/rad,lift_bank,pull_limit,limit_side,pushing?1:0,push_roll,cx,cy,lx,ly);
#endif
        const float w=smoothstep(t.near_end,t.far_start,g.angle);
        g.turn_weight=w;
        // Pitch, roll and rudder act together from the first frame. Pitch is judged in the
        // attitude the roll will have reached when the stick reaches the aircraft (the
        // pitch lag is longer than the roll's), so the pull starts while the roll is
        // still under way (reported: roll, then pull, then yaw, one after another).
        const float toward=roll_rate*(g.roll_error>=0?1.0f:-1.0f);
        const float credit=std::min({std::max(0.0f,toward)*lag,t.overlap_max,std::abs(g.roll_error)})*
                           smoothstep(5.0f,15.0f,g.angle);
        const float a=std::copysign(credit,g.roll_error)*rad;
        // Canopy after that roll (roll moves u toward r), in body coordinates.
        const float ax=std::sin(a), ay=std::cos(a);
        const float plane=std::atan2(rx*ax+ux*ay,fx)/rad, beside=std::atan2(rx*ay-ux*ax,fx)/rad;
        // Far: the whole angle lies in the pitch plane once the roll is done; pitch for it in
        // proportion to how far the (limited) lift is on it already. A push only when pushing.
        const float s=pushing?-1.0f:1.0f;
        const float along=std::max(0.0f,s*std::atan2(rx*cx+ux*cy,fx)/rad);
        const float lift_left=std::max(0.0f,std::abs(g.roll_error)-credit);
        g.pitch_error=plane*(1-w)+s*along*w*std::max(0.0f,std::cos(lift_left*rad));
        // Near the ground, sinking, the aim above the nose: pull while the roll is still under
        // way, as much as the canopy already faces up (the pull above waits for the lift to be
        // within 90 deg of the target: bench, 650 m/s and 275 m/s down, 80 deg banked, the
        // first 0.75 s went to the roll alone, 270 m). Nothing when the aim is below the nose.
        if(!pushing && t.ground_guard>0 && ground_time<t.ground_guard && dot(aim,{0,0,1})>b.f.z) {
            const float urgency=1-smoothstep(0.4f*t.ground_guard,t.ground_guard,ground_time);
            g.pitch_error=std::max(g.pitch_error,60.0f*urgency*std::max(0.0f,b.u.z));
        }
        // The rudder takes the part of the target off the pitch plane: beside the nose,
        // or across the lift in a turn (a bank-limited turn toward a target below it:
        // roll, pull and rudder, as a pilot would).
        const float across=std::atan2(rx*cy-ux*cx,fx)/rad;
        g.yaw_error=beside*(1-w)+across*cy*w;
        return g;
    }
};
}
