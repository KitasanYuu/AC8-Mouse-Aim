#pragma once
// Flight logic: maneuver guidance plus stick shaping. Built into the main DLL and,
// for development, also as ac8_flight_logic.dll, which the main DLL hot-swaps while
// the game runs. Everything crossing that boundary is plain data behind a fixed ABI.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <new>
#include "maneuver.h"
namespace flight {
// Bump when LogicInput, LogicOutput or the exported functions change.
constexpr int logic_abi=7;
// Controller convention: positive pitch raises the nose, roll rolls right, yaw yaws right.
struct LogicInput {
    float pitch,yaw,roll;              // aircraft attitude, degrees
    float aim_x,aim_y,aim_z;           // world target direction (unit)
    float pitch_rate,yaw_rate,roll_rate; // filtered body rates, degrees/second
    float dt;
    int keyboard;                      // axes flown by keyboard: 1 pitch, 2 roll, 4 yaw
    int aircraft;                      // changes whenever the player's aircraft changes
    float throttle,brake;              // the game's own InputThrottle / InputBrake (0..1)
    // Flight path from the actor position: world velocity (m/s), acceleration (m/s^2)
    // and altitude (m). Recorded for the whole-aircraft model; not used for control yet.
    float vel_x,vel_y,vel_z;
    float acc_x,acc_y,acc_z;
    float altitude;
    int request;                       // the player's requests by key: 1 post-stall (key held)
    // 1: the aim is attached to the aircraft (the cockpit's mouse joystick: an offset off the nose
    // that turns with it), so its motion is the aircraft's own, not a target's: no lead. Read as
    // a target's, the aircraft's yaw kept itself going with the ring on the nose (reported
    // 2026-10-07: rudder held at -0.6, 5 deg/s, the aim 0.03 deg off the nose).
    int aim_attached;
};
// Per-frame internal state for the telemetry stream (LogicOutput::diag).
enum Diag { DiagAngle, DiagPitchError, DiagYawError, DiagRollError, DiagTurnWeight, DiagPushing, DiagTail,
            DiagPitchWant, DiagRollWant, DiagPitchPred, DiagRollPred, DiagPitchComing,
            DiagSpeed, DiagAoa, DiagBeta, DiagNz, DiagCount };
struct LogicOutput {
    float pitch,roll,yaw;              // axis values to write (before input-sign config)
    char trace[400];                   // per-frame diagnostic text
    char event[128];                   // non-empty when something notable happened
    float diag[16];                    // see Diag
};
static_assert(DiagCount==16);
struct Tuning {
    ManeuverTuning maneuver;
    Plant plant;
    // Rate control: stick = steady input for the desired rate (measured curve) plus a
    // moderate correction toward it. Earlier full model inversion multiplied the gain by
    // the lag ratios and chattered at ~2.5 Hz in game. Brake with this fraction of the
    // opposite authority, and close the last degrees at gain (1/s). Tuned in the
    // closed-loop simulation (tests/flight_sim.cpp).
    float pitch_kd=0.1f;               // stick per deg/s of predicted rate error (0.25 until 2026-10-05)
    float roll_kv=1.5f;                // roll acceleration per deg/s of rate error (1/s)
    float pitch_brake=0.45f, roll_brake=0.45f;
    float pitch_unload=0.3f;           // opposite authority used to slow a wanted rotation (0 = full; 0 until 2026-10-05)
    // pitch_gain 2.5: 2 began easing off ~40 deg out and crept in (logged); 3 overshot.
    float pitch_gain=2.5f, roll_gain=2.0f;
    // Lead: the target's own motion across the nose is flown as a rate on top of closing
    // the error, so a moving target (a turning enemy) is followed, not trailed. Without
    // it a 20 deg/s turner was trailed by a steady 13 deg (sim), and turns in combat
    // stalled half way (reported). Gain, low-pass (s), limit (deg/s).
    // lead_gate: while the roll must come first, only the part that hurries the coming pitch
    // (1; 0 = the whole lead, as before 2026-10-05).
    float lead_gain=1.0f, lead_filter=0.2f, lead_max=60.0f, lead_gate=1.0f;
    // Extra gain fraction inside the last few degrees: the nose crept the final 1-2 deg
    // into the ~1.3 deg aim circle (logged); 0.4 settled faster in sim, 0.8 overshot.
    float pitch_gain_near=0.4f;
    // Integral on small pitch error. 1.5/8 wound up and oscillated through the vertical
    // in sim (weak jet, straight down); 1.0/4 with the trim kept through maneuvers.
    float pitch_trim=1.0f, pitch_trim_limit=4.0f;
    float pitch_trim_keep=1;           // keep the trim through maneuvers (0 = decay)
    // High-G turn (throttle and brake held together, or the single high-G button, which
    // sets both): the pull is this much stronger. Logged: full pull held 33-38 deg/s
    // outside it and 45-55 inside, with 77-90 for the first second; the model and the
    // authority estimate (1-2 s to follow) under-predicted the turn, braking late.
    float highg_pull=1.4f;
    // High-G (throttle and brake held) is the player asking for all the G there is: beyond
    // highg_full_from deg of the aim, with the lift on the target's side (a pull the guidance
    // wants), the stick goes full back, without waiting for the roll to finish or braking
    // early for the stop (it stops as late as the hardest stop allows); inside it the usual law takes over. (Logged, LADON in an FA-36: in
    // high-G turns the stick sat short of full 44% of the time, 15% waiting on a roll the
    // game turns at ~20 deg/s at 600 m/s, 18% braking early; "no high-G feel", missiles not
    // dodged.) 0 = off.
    float highg_full_from=8.0f;
    // Stick slew (full scale per second). The game smooths stick input, so faster
    // swings are averaged away and only make the model-based loop chatter.
    float pitch_slew=20.0f, roll_slew=20.0f;
    // Rate observer (1/s): the model carries the rate between frames and the noisy
    // measurement corrects it at this rate. Raw measured rates fed straight into the
    // predictor made the stick chatter ~100 reversals/min in combat (logged).
    float rate_observer=3.0f;
    float yaw_dead=0.2f, yaw_gain=1.5f, yaw_max_rate=6.0f, yaw_damping=0.45f, yaw_limit=0.6f;
    float yaw_slew=5.0f;
    // Fine search (the Gaijin patent's scored look-ahead, US8770979B2 process 303C): within
    // fine_angle deg of the aim, a few sticks around the rules' own (and no roll at all) are
    // each flown fine_horizon s ahead on this logic's own aircraft model, from where the
    // commands in flight leave it, and the one with the least squared aim error, roll
    // activity (fine_roll), bank (fine_level) and stick change (fine_smooth) is flown. So a
    // degree off is closed by rudder and pitch rather than a roll in and out (bench, mouse
    // nudges: 7 deg of rocking cut to 0.7, roll reversals in fights down a third). 0 = off.
    float fine_search=1, fine_angle=20, fine_horizon=1.0f, fine_smooth=2.0f, fine_roll=0.002f, fine_level=0.003f;
    float fine_yaw_rate=6.0f;   // deg/s a full rudder gives (logged ~5.4)
    // What is left after the horizon: the aim error there, held for as long as the aircraft model
    // needs to remove it from that attitude (roll the lift onto it and pull, roll the floor onto it
    // and push, or rudder), and closing evenly. Without it a roll that pays off after more than
    // fine_horizon was never chosen: the search held the bank against the guidance's roll ~20% of
    // its time in game (2-3% before it was widened), the target circled the nose in a corkscrew
    // (logged, FA-36 against LADON); the bench halved it. Weight (1 = the remaining error closing
    // evenly); 0 = off (until 2026-10-05).
    float fine_to_go=3;
    // The pitch in the look-ahead follows the rules' pitch law: from the rules' stick now, changed
    // as the aim's offset in the pitch plane changes (a roll that brings the lift onto the aim is
    // followed by the pull the rules will then give). Held fixed, a roll never paid off within the
    // horizon. 0 = held (until 2026-10-05).
    float fine_follow=1;
    // 0: the pitch stays the rules' and only roll and rudder are searched. Searching the pitch
    // too stopped the rocking but settled later and overshot more (bench: the rules' braking
    // is fitted to the game; the search trusts a model that is not quite the aircraft).
    float fine_pitch=0;
    float calibrate=0;                 // change to a new positive value to run input calibration
    float maneuver_test=0;             // change to a new positive value to arm the post-stall test plan
    float auto_trace=0;                // record each maneuver automatically (compact AT lines); 0 = off
    // Online identification of the current aircraft: memory of the estimate (s) and
    // seconds of informative data before it is fully trusted. 0 memory disables it.
    float ident_memory=15.0f, ident_trust=3.0f;
    // Fast control-authority estimate (memory s; 0 = off): the response to the stick
    // changes within seconds with speed (full pull held a median 37-40 deg/s in combat vs
    // 61 at cruise in calibration), and an over-strong model predicts too much pitch from
    // the commands in flight, brakes with full push mid-turn and stutters (logged).
    float authority_memory=2.0f, authority_filter=1.0f, authority_trust=0.6f;
    float authority_min=0.35f, authority_max=2.0f;  // high-G turns pitched ~2x the model (logged)
};
// Recent commands, newest at head, for predicting what is still in flight.
struct AxisHistory {
    static constexpr int size=128;
    float u[size]{}, dt[size]{};
    int head=0;
    void push(float value,float step) { head=(head+1)%size; u[head]=value; dt[head]=step; }
};
// Open-loop input calibration: holds fixed raw axis values and reports the
// aircraft's response, so the game's input curve can be measured and inverted.
// Each test starts only after normal control has settled the aircraft (rates near
// zero, nose on target, wings level), so every step begins from the same state.
struct CalibrationStep { int axis; float value; float hold; }; // axis 0 pitch, 1 roll
inline const CalibrationStep* calibration_plan(int& count) {
    static CalibrationStep plan[64];
    static int n=0;
    if(!n) {
        // Alternate signs so the attitude stays near the start between tests.
        const float roll[]={0.1f,0.2f,0.3f,0.4f,0.5f,0.6f,0.7f,0.8f,0.9f,1.0f};
        for(int i=0;i<10;++i) plan[n++]={1,(i%2?-1.0f:1.0f)*roll[i],1.0f};
        for(float l:{0.1f,0.2f,0.3f,0.45f,0.6f,0.8f,1.0f}) { plan[n++]={0,l,2.0f}; plan[n++]={0,-l,2.0f}; }
    }
    count=n;
    return plan;
}
struct Calibration {
    int step=-1;                       // -1 when idle
    bool holding=false, announce=false;
    float time=0, settled=0, peak=0, onset=-1, t63=-1, end_sum=0;
    int end_count=0;
    float samples[24]{};               // rate every 0.1 s during the hold
};
// Scripted post-stall test (maneuver_test): what a hand on a gamepad cannot time. The player
// slows below 500 km/h and presses the high-G turn (throttle and brake); the script takes the
// pitch only, centred and then full, a set delay after the high-G began, and lets go at a set
// angle between the nose and the flight path (or holds); roll and rudder stay the flight
// logic's, so the turn goes where the pointer is. One step of the plan per high-G press. After
// letting go it watches until the game has swung the flight path onto the nose, then hands the
// pitch back. Found so far (FA-36, 2026-10-06): the post-stall maneuver starts when the pitch
// input is not already pulling at the moment the high-G is pressed (20 of 20; with the stick
// already pulling it is an ordinary high-G turn), below 500 km/h, the pull following within at
// least 0.42 s; let go at 60-121 deg off the path with the nose at 116-140 deg/s, the nose
// carries on another 50-55 deg over ~0.6 s, and the path is on the nose 1.0-1.7 s after letting
// go. Now measured: smaller let-go angles, and how late the pull may come.
// Telemetry diag[DiagTail] is 10 x step (from 1) + phase during a step.
struct TestStep { float delay; float release; };   // s after high-G; deg off the path, 0 = held
inline const TestStep* test_plan(int& count) {
    static const TestStep plan[]={{0.3f,30},{0.3f,75},{0.3f,105},{0.6f,0},{0.9f,0},{1.2f,0}};
    count=int(sizeof(plan)/sizeof(plan[0]));
    return plan;
}
struct ManeuverTest {
    static constexpr unsigned valid_tag=0x54455354u;   // kept across resets only when intact
    unsigned tag=valid_tag;
    int step=-1;                       // -1 idle; otherwise the step that runs on the next press
    int phase=0;                       // 0 waiting for high-G, 1 delay (centred), 2 pulling, 3 let go (centred)
    float time=0, off_max=0;
    bool high_g=false;
};
constexpr float test_hold=3.0f;        // pull held at most when the step lets go at no angle (s)
constexpr float test_watch=2.5f;       // centred stick watched after letting go, at most (s)
// Online identification. Aircraft differ widely (logged: roll 95 vs 238 deg/s^2 at
// full stick, pull 28 vs 47 deg/s), so each aircraft's authority and response time are
// identified in flight. A bank of candidate models runs in parallel on the stick actually
// sent, through the same delay, game smoothing and rate filter as the measurement, and
// the one that has tracked the measured rate best recently is used (output error; no
// differentiation, so filter lag and noise do not bias it). Least squares on rate
// derivatives was tried first and drifted badly under timing mismatch in simulation.
// The game's input pipeline (delay, smoothing, curve exponents) stays from calibration.
struct ModelBank {
    static constexpr int delays=3, pitch_scales=7, pitch_taus=6, roll_scales=7, roll_taus=4;
    static constexpr float delay_extra[delays]={0.0f,0.06f,0.12f};  // seconds beyond calibration
    static constexpr float pitch_scale[pitch_scales]={0.35f,0.45f,0.58f,0.72f,0.86f,1.0f,1.2f};
    static constexpr float pitch_tau[pitch_taus]={0.25f,0.35f,0.5f,0.7f,0.95f,1.3f};
    static constexpr float roll_scale[roll_scales]={0.3f,0.4f,0.52f,0.66f,0.82f,1.0f,1.3f};
    static constexpr float roll_tau[roll_taus]={0.45f,0.65f,0.9f,1.3f};
    static constexpr int np=delays*pitch_scales*pitch_taus, nr=delays*roll_scales*roll_taus;
    float pf[delays]{}, rf[delays]{};   // game smoothing state per delay candidate
    float pr[np]{}, pm[np]{}, pj[np]{}; // model rate, filtered as measured, tracking cost
    float rr[nr]{}, rm[nr]{}, rj[nr]{};
};
// The game's input smoothing: build-up time constant, faster release toward or past 0.
inline float input_smooth(float f,float u,float dt,float build,float release) {
    const float tc=(u*f<0 || std::abs(u)<std::abs(f)) ? release : build;
    return f+(u-f)*(1-std::exp(-dt/tc));
}
// Stick command leaving a delay window this frame (the one that now reaches the game's
// smoothing), matching predict(): the newest entry is this frame's command.
inline bool leaving_command(const AxisHistory& h,float delay,float& u,float& dt) {
    constexpr int size=AxisHistory::size;
    int n=0;
    float back=0;
    while(n<size-1 && back<delay) {
        const float step=h.dt[(h.head-n+size)%size];
        if(step<=0) return false;
        back+=step; ++n;
    }
    const int i=(h.head-n+size)%size;
    u=h.u[i]; dt=h.dt[i];
    return dt>0;
}
struct Identification {
    static constexpr unsigned valid_tag=0x1D3A0004u;  // layout version, survives resets only if it matches
    unsigned tag=0;
    int aircraft=-1;
    ModelBank bank;
    float pitch_seen=0, roll_seen=0;    // seconds of informative data
    float report=0;
    bool primed=false;
    void reset(int id,const Plant&) { *this=Identification{}; tag=valid_tag; aircraft=id; }
    // Cost-weighted best model; near-ties average (log space for scale and tau) so the
    // estimate moves smoothly instead of jumping between grid points.
    static void best(const float* cost,int n_scale,int n_tau,const float* scales,const float* taus,
                     float& extra,float& scale,float& tau) {
        const int n=ModelBank::delays*n_scale*n_tau;
        float lowest=cost[0];
        for(int i=1;i<n;++i) lowest=std::min(lowest,cost[i]);
        const float spread=0.15f*lowest+1e-3f;
        float wsum=0, ld=0, ls=0, lt=0;
        for(int d=0;d<ModelBank::delays;++d) for(int i=0;i<n_scale;++i) for(int j=0;j<n_tau;++j) {
            const float w=std::exp(-(cost[(d*n_scale+i)*n_tau+j]-lowest)/spread);
            wsum+=w; ld+=w*ModelBank::delay_extra[d]; ls+=w*std::log(scales[i]); lt+=w*std::log(taus[j]);
        }
        extra=ld/wsum; scale=std::exp(ls/wsum); tau=std::exp(lt/wsum);
    }
    // Calibrated values until enough informative data, then the identified ones.
    Plant adapt(const Plant& prior,float trust_after) const {
        Plant e=prior;
        if(tag!=valid_tag || trust_after<=0) return e;
        const float pw=smoothstep(0,trust_after,pitch_seen), rw=smoothstep(0,trust_after,roll_seen);
        float pd=0, ps=1, pt=prior.pitch_tau, rd=0, rs=1, rt=prior.roll_tau;
        best(bank.pj,ModelBank::pitch_scales,ModelBank::pitch_taus,ModelBank::pitch_scale,ModelBank::pitch_tau,pd,ps,pt);
        best(bank.rj,ModelBank::roll_scales,ModelBank::roll_taus,ModelBank::roll_scale,ModelBank::roll_tau,rd,rs,rt);
        const float pscale=1+(ps-1)*pw, rscale=1+(rs-1)*rw;
        e.pitch_delay=prior.pitch_delay+pd*pw;
        e.pitch_pull=prior.pitch_pull*pscale;
        e.pitch_push=prior.pitch_push*pscale;
        e.pitch_tau=prior.pitch_tau+(pt-prior.pitch_tau)*pw;
        e.roll_delay=prior.roll_delay+rd*rw;
        e.roll_accel=prior.roll_accel*rscale;
        e.roll_tau=prior.roll_tau+(rt-prior.roll_tau)*rw;
        // Keep the calibrated ratio of usable maximum rate to full-stick steady rate.
        const float ratio=prior.roll_max_rate/(prior.roll_accel*prior.roll_tau);
        e.roll_max_rate=std::clamp(ratio*e.roll_accel*e.roll_tau,30.0f,250.0f);
        return e;
    }
    // Feed one frame after this frame's commands were pushed: q/r are the measured
    // (host-filtered) rates and up the canopy's world-up component.
    void observe(const Plant& p,const AxisHistory& pitch_history,const AxisHistory& roll_history,
                 float q,float r,float up,float dt,bool usable,float memory,LogicOutput& out) {
        ModelBank& m=bank;
        constexpr int ps=ModelBank::pitch_scales, pt=ModelBank::pitch_taus;
        constexpr int rs=ModelBank::roll_scales, rt=ModelBank::roll_taus;
        if(!primed || dt<=0 || dt>0.05f) {
            // Start (or resync after a gap) every candidate from the measured rate.
            for(int i=0;i<ModelBank::np;++i) m.pr[i]=m.pm[i]=q;
            for(int i=0;i<ModelBank::nr;++i) m.rr[i]=m.rm[i]=r;
            primed=true;
            return;
        }
        const float filter=1-std::exp(-12*dt);  // the host's rate filter
        const float forget=memory>0?std::exp(-dt/memory):1;
        const float gravity=p.pitch_gravity*up, cap=1.5f*p.roll_max_rate;
        bool pitch_info=false, roll_info=false;
        for(int d=0;d<ModelBank::delays;++d) {
            float u=0, udt=0;
            if(leaving_command(pitch_history,p.pitch_delay+ModelBank::delay_extra[d],u,udt))
                m.pf[d]=input_smooth(m.pf[d],u,udt,p.pitch_input,p.pitch_release);
            if(leaving_command(roll_history,p.roll_delay+ModelBank::delay_extra[d],u,udt))
                m.rf[d]+=(u-m.rf[d])*(1-std::exp(-udt/p.roll_input));
            const float pa=std::pow(std::min(std::abs(m.pf[d]),1.0f),p.pitch_curve);
            const float drive=m.pf[d]>=0?p.pitch_pull*pa:-p.push_authority(up)*pa;
            const bool pinfo=usable && (std::abs(m.pf[0])>0.1f || std::abs(q)>5);
            pitch_info|=pinfo;
            for(int i=0;i<ps;++i) for(int j=0;j<pt;++j) {
                const int k=(d*ps+i)*pt+j;
                m.pr[k]+=dt*(ModelBank::pitch_scale[i]*drive-gravity-m.pr[k])/ModelBank::pitch_tau[j];
                m.pm[k]+=(m.pr[k]-m.pm[k])*filter;
                const float e=q-m.pm[k];
                m.pj[k]=m.pj[k]*forget+(pinfo?e*e*dt:0);
            }
            const float ra=std::copysign(p.roll_accel*std::pow(std::min(std::abs(m.rf[d]),1.0f),p.roll_curve),m.rf[d]);
            const bool rinfo=usable && (std::abs(m.rf[0])>0.1f || std::abs(r)>10);
            roll_info|=rinfo;
            for(int i=0;i<rs;++i) for(int j=0;j<rt;++j) {
                const int k=(d*rs+i)*rt+j;
                m.rr[k]=std::clamp(m.rr[k]+dt*(ModelBank::roll_scale[i]*ra-m.rr[k]/ModelBank::roll_tau[j]),-cap,cap);
                m.rm[k]+=(m.rr[k]-m.rm[k])*filter;
                const float e=r-m.rm[k];
                m.rj[k]=m.rj[k]*forget+(rinfo?e*e*dt:0);
            }
        }
        if(!usable) {
            // Keyboard flying: keep candidates synced to reality, but learn nothing.
            for(int i=0;i<ModelBank::np;++i) m.pr[i]=m.pm[i]=q;
            for(int i=0;i<ModelBank::nr;++i) m.rr[i]=m.rm[i]=r;
        }
        if(pitch_info) pitch_seen+=dt;
        if(roll_info) roll_seen+=dt;
        report+=dt;
        if(report>5 && !out.event[0]) {
            report=0;
            const Plant e=adapt(p,1);
            snprintf(out.event,sizeof(out.event),"ident: pitch %.0f/%.0f tau %.2f +%.2fs | roll %.0f tau %.2f +%.2fs | data %.0f/%.0fs",
                     e.pitch_pull,e.pitch_push,e.pitch_tau,e.pitch_delay-p.pitch_delay,e.roll_accel,e.roll_tau,
                     e.roll_delay-p.roll_delay,pitch_seen,roll_seen);
        }
    }
};
struct LogicState {
    Identification ident;              // kept across resets; reset only for a new aircraft
    Maneuver maneuver;
    CommandSlew yaw, pitch_out, roll_out;
    AxisHistory pitch_history, roll_history;
    float pitch_trim=0;
    float pitch_smoothed=0, roll_smoothed=0; // game input smoothing state, as of one delay ago
    float pitch_rate_est=0, roll_rate_est=0;  // observer estimates, in measured-rate terms
    bool observer_primed=false;
    bool tail_logged=false;
    bool high_g_logged=false;
    bool recording=false;              // automatic per-maneuver trace
    float settled_for=0;
    Calibration cal;
    ManeuverTest test;
    bool post_stall_high_g=false;      // high-G as of the previous frame, for the post-stall request
    float unmodelled=0;                // s left in which the measured rates are not the model's (post-stall)
    // The game's post-stall entry as the logic sent it: the high-G pressed below 500 km/h with the
    // pitch not pulling (see post_stall_override); how long since, and the nose's most off the path.
    bool psm_entered=false;
    float psm_time=0, psm_off_max=0;
    float post_stall_hold=0;           // pitch held centred for the post-stall entry (s left)
    // Authority estimate: responses to the delayed stick at unit authority (and pitch's
    // gravity part), and their decayed correlation with the measured rates.
    // Pull and push are estimated separately: inverted, a full push gave 2-10 deg/s
    // against ~21 upright (logged), far from what one shared scale can describe.
    float pull_unit=0, push_unit=0, pitch_free=0, roll_unit=0;
    float pull_f=0, push_f=0, pitch_yf=0, roll_f=0, roll_yf=0;   // the same, low-passed
    float s_uu=0, s_dd=0, s_ud=0, s_uy=0, s_dy=0, roll_zz=0, roll_zy=0;
    float pitch_scale=1, push_scale=1, roll_scale=1;
    float aim_prev[3]{}, lead_pitch=0, lead_yaw=0;  // target motion across the nose (deg/s)
    bool aim_primed=false;
};
// Returns true while a test input is being held (output already written).
inline bool calibration_hold(LogicState& s,const LogicInput& in,LogicOutput& out) {
    int count=0;
    const CalibrationStep* plan=calibration_plan(count);
    Calibration& c=s.cal;
    if(in.keyboard) {
        c=Calibration{};
        snprintf(out.event,sizeof(out.event),"CAL aborted: keyboard input");
        return false;
    }
    if(c.announce) {
        c.announce=false;
        snprintf(out.event,sizeof(out.event),"CAL start: %d steps, about 2 min; hands off mouse and keys",count);
    }
    if(!c.holding) return false;
    const CalibrationStep& step=plan[c.step];
    const float sign=step.value>0?1.0f:-1.0f;
    const float rate=(step.axis?in.roll_rate:in.pitch_rate)*sign;
    const int sample=static_cast<int>(c.time*10.0f);
    c.time+=in.dt;
    if(sample<24) c.samples[sample]=rate;
    c.peak=std::max(c.peak,rate);
    if(c.onset<0 && rate>3) c.onset=c.time;
    if(c.time>step.hold-0.2f) { c.end_sum+=rate; ++c.end_count; }
    if(c.time<step.hold) {
        s.yaw.reset();
        out.pitch=out.roll=out.yaw=0;
        (step.axis?out.roll:out.pitch)=step.value;
        s.pitch_out.value=out.pitch; s.roll_out.value=out.roll;
        s.pitch_history.push(out.pitch,in.dt);
        s.roll_history.push(out.roll,in.dt);
        snprintf(out.trace,sizeof(out.trace),"cal=%d axis=%s out=%+.2f t=%.2f rate=(%.1f,%.1f,%.1f) roll=%.1f",
                 c.step,step.axis?"roll":"pitch",step.value,c.time,in.pitch_rate,in.yaw_rate,in.roll_rate,in.roll);
        return true;
    }
    const float end=c.end_count?c.end_sum/c.end_count:0;
    for(int i=0;i<24 && c.t63<0;++i) if(end>1 && c.samples[i]>=0.63f*end) c.t63=0.1f*(i+1);
    snprintf(out.event,sizeof(out.event),"CAL %s out=%+.2f end=%+.1f peak=%+.1f onset=%.2fs t63=%.1fs",
             step.axis?"roll":"pitch",step.value,sign*end,sign*c.peak,c.onset,c.t63);
    const int next=c.step+1;
    c=Calibration{};
    if(next<count) c.step=next;
    else snprintf(out.event+std::strlen(out.event),sizeof(out.event)-std::strlen(out.event)," | CAL done");
    return false;
}
// Recovery between tests runs normal control; begin the next test once settled.
inline void calibration_settle(Calibration& c,const LogicInput& in,const Guidance& g) {
    if(c.step<0 || c.holding) return;
    const float bank=std::abs(std::atan2(std::sin(in.roll*rad),std::cos(in.roll*rad))/rad);
    const bool steady=std::abs(in.pitch_rate)<3 && std::abs(in.roll_rate)<4 && g.angle<4 && bank<10;
    c.time+=in.dt;
    c.settled=steady?c.settled+in.dt:0;
    if((c.settled>0.4f && c.time>0.8f) || c.time>6.0f) {
        const int step=c.step;
        c=Calibration{};
        c.step=step;
        c.holding=true;
    }
}

// Reads [tuning] from config.ini; missing keys keep the defaults above.
inline Tuning read_tuning(const wchar_t* path) {
    Tuning t;
    auto get=[&](const wchar_t* key,float& value) {
        wchar_t text[64]{};
        GetPrivateProfileStringW(L"tuning",key,L"",text,64,path);
        wchar_t* end{};
        const float parsed=wcstof(text,&end);
        if(end!=text && std::isfinite(parsed)) value=parsed;
    };
    auto& m=t.maneuver;
    get(L"near_end",m.near_end); get(L"far_start",m.far_start); get(L"level_per_deg",m.level_per_deg); get(L"rollout_rate",m.rollout_rate);
    get(L"rollout_lag",m.rollout_lag); get(L"pursuit_ahead",m.pursuit_ahead); get(L"push_enter",m.push_enter); get(L"push_exit",m.push_exit);
    get(L"push_roll_enter",m.push_roll_enter); get(L"push_roll_exit",m.push_roll_exit); get(L"push_below",m.push_below); get(L"pursuit_speed",m.pursuit_speed); get(L"level_inside",m.level_inside); get(L"slice_ease",m.slice_ease); get(L"slice_hold",m.slice_hold); get(L"chase_slice",m.chase_slice); get(L"chase_level",m.chase_level); get(L"push_below_exit",m.push_below_exit); get(L"choice_margin",m.choice_margin); get(L"push_bias",m.push_bias); get(L"ground_guard",m.ground_guard); get(L"escape_bank",m.escape_bank); get(L"escape_aim_down",m.escape_aim_down);
    get(L"tail_enter",m.tail_enter); get(L"tail_exit",m.tail_exit); get(L"tail_bank",m.tail_bank); get(L"latch_limit",m.latch_limit);
    get(L"overlap_max",m.overlap_max); get(L"upright",m.upright); get(L"slice_bank",m.slice_bank); get(L"slice_deep",m.slice_deep);
    get(L"slice_from",m.slice_from); get(L"invert_min_angle",m.invert_min_angle);
    auto& p=t.plant;
    get(L"pitch_delay",p.pitch_delay); get(L"pitch_input",p.pitch_input);
    get(L"pitch_release",p.pitch_release);
    get(L"pitch_tau",p.pitch_tau); get(L"pitch_curve",p.pitch_curve);
    get(L"pitch_pull",p.pitch_pull); get(L"pitch_push",p.pitch_push); get(L"pitch_gravity",p.pitch_gravity);
    get(L"push_gravity",p.push_gravity);
    get(L"roll_delay",p.roll_delay); get(L"roll_input",p.roll_input);
    get(L"roll_accel",p.roll_accel); get(L"roll_curve",p.roll_curve);
    get(L"roll_max_rate",p.roll_max_rate); get(L"roll_tau",p.roll_tau);
    get(L"pitch_kd",t.pitch_kd); get(L"roll_kv",t.roll_kv);
    get(L"pitch_brake",t.pitch_brake); get(L"roll_brake",t.roll_brake); get(L"pitch_unload",t.pitch_unload); get(L"pitch_trim_keep",t.pitch_trim_keep); get(L"highg_pull",t.highg_pull); get(L"highg_full_from",t.highg_full_from); get(L"pitch_gain_near",t.pitch_gain_near);
    get(L"lead_gain",t.lead_gain); get(L"lead_filter",t.lead_filter); get(L"lead_max",t.lead_max); get(L"lead_gate",t.lead_gate);
    get(L"pitch_gain",t.pitch_gain); get(L"roll_gain",t.roll_gain);
    get(L"pitch_trim",t.pitch_trim); get(L"pitch_trim_limit",t.pitch_trim_limit);
    get(L"pitch_slew",t.pitch_slew); get(L"roll_slew",t.roll_slew);
    get(L"rate_observer",t.rate_observer);
    get(L"yaw_dead",t.yaw_dead); get(L"yaw_gain",t.yaw_gain); get(L"yaw_max_rate",t.yaw_max_rate);
    get(L"yaw_damping",t.yaw_damping); get(L"yaw_limit",t.yaw_limit); get(L"yaw_slew",t.yaw_slew);
    get(L"fine_search",t.fine_search); get(L"fine_angle",t.fine_angle); get(L"fine_horizon",t.fine_horizon);
    get(L"fine_smooth",t.fine_smooth); get(L"fine_roll",t.fine_roll); get(L"fine_level",t.fine_level); get(L"fine_yaw_rate",t.fine_yaw_rate); get(L"fine_pitch",t.fine_pitch); get(L"fine_to_go",t.fine_to_go); get(L"fine_follow",t.fine_follow);
    get(L"calibrate",t.calibrate);
    get(L"maneuver_test",t.maneuver_test);
    get(L"auto_trace",t.auto_trace);
    get(L"ident_memory",t.ident_memory); get(L"ident_trust",t.ident_trust);
    get(L"authority_memory",t.authority_memory); get(L"authority_min",t.authority_min);
    get(L"authority_max",t.authority_max); get(L"authority_filter",t.authority_filter);
    get(L"authority_trust",t.authority_trust);
    return t;
}

// Plant steps, inverses and stopping distances from the measured model.
inline float pitch_steady(const Plant& p,float u,float upright) {
    const float a=std::pow(std::min(std::abs(u),1.0f),p.pitch_curve);
    return (u>=0 ? p.pitch_pull*a : -p.push_authority(upright)*a)-p.pitch_gravity*upright;
}
inline float pitch_input_for(const Plant& p,float aero,float upright=0) {
    return aero>=0 ? std::min(1.0f,std::pow(aero/p.pitch_pull,1/p.pitch_curve))
                   : -std::min(1.0f,std::pow(-aero/p.push_authority(upright),1/p.pitch_curve));
}
inline float roll_accel_for(const Plant& p,float u) {
    return std::copysign(p.roll_accel*std::pow(std::min(std::abs(u),1.0f),p.roll_curve),u);
}
inline float roll_input_for(const Plant& p,float accel) {
    return std::copysign(std::min(1.0f,std::pow(std::abs(accel)/p.roll_accel,1/p.roll_curve)),accel);
}
// Rate and attitude change still to come from commands already sent but not yet
// visible in the measured rate: replay them through the game's input smoothing and
// the model (Smith predictor). smoothed is the smoothing state one delay ago; it is
// advanced here by the command leaving the delay window.
template<class Step>
inline void predict(const AxisHistory& h,float delay,float smoothing,float release,float rate,float& smoothed,Step step,
                    float& predicted,float& filtered,float& travel,bool advance=true) {
    constexpr int size=AxisHistory::size;
    int n=0;
    float back=0;
    while(n<size-1 && back<delay) {
        const float dt=h.dt[(h.head-n+size)%size];
        if(dt<=0) break;
        back+=dt; ++n;
    }
    const int leaving=(h.head-n+size)%size;
    if(advance && h.dt[leaving]>0) smoothed=input_smooth(smoothed,h.u[leaving],h.dt[leaving],smoothing,release);
    predicted=rate; filtered=smoothed; travel=0;
    for(int k=n-1;k>=0;--k) {
        const int i=(h.head-k+size)%size;
        filtered=input_smooth(filtered,h.u[i],h.dt[i],smoothing,release);
        predicted=step(predicted,filtered,h.dt[i]);
        travel+=predicted*h.dt[i];
    }
}
// Largest rate that can still be stopped within distance: first-order braking toward
// -opposing at time constant tau covers tau*(r - P*ln(1 + r/P)).
inline float stoppable_first_order(float distance,float opposing,float tau,float cap) {
    float lo=0, hi=cap;
    for(int i=0;i<16;++i) {
        const float r=0.5f*(lo+hi);
        (tau*(r-opposing*std::log1p(r/opposing))<=distance ? lo : hi)=r;
    }
    return lo;
}
inline float desired_rate(float error,float cap,float stoppable,float gain) {
    return std::copysign(std::min({cap,stoppable,gain*std::abs(error)}),error);
}

// Speed (m/s), angle of attack and sideslip (deg, nose above / right of the flight path)
// and normal load factor (g along the canopy, 1 in level flight).
struct FlightPath { float speed=0, aoa=0, beta=0, nz=0; };
inline FlightPath flight_path(const Basis& b,const LogicInput& in) {
    FlightPath p;
    const V v{in.vel_x,in.vel_y,in.vel_z};
    p.speed=std::sqrt(dot(v,v));
    if(p.speed>1) {
        p.aoa=std::atan2(-dot(v,b.u),dot(v,b.f))/rad;
        p.beta=std::atan2(dot(v,b.r),dot(v,b.f))/rad;
    }
    p.nz=(in.acc_x*b.u.x+in.acc_y*b.u.y+(in.acc_z+9.81f)*b.u.z)/9.81f;
    return p;
}

// Post-stall on request (a key; the player still presses the high-G): the game enters its
// post-stall maneuver only when the pitch input is not already pulling at the moment the
// high-G is pressed (FA-36, 20 of 20; pulling, it is an ordinary high-G turn), below 500 km/h,
// and only if the pull then comes while the high-G is still held: tapped high-G (0.1-0.2 s)
// with the pull after it, 17 of 17 were ordinary turns; with the pull inside it, the longer the
// overlap the surer (0.08 s and more nearly always; logged 2026-10-06). The flight logic pulls
// whenever the aim is off the nose, so the pitch is held centred for post_stall_entry from the
// high-G (0.1 s and the stick's slew put the full pull 0.11-0.15 s after it, often past a tap),
// then pulled full at once if the logic pulls at all, while the high-G lasts.
constexpr float post_stall_entry=0.04f;
constexpr float post_stall_pull=0.3f;  // the full pull's window from the high-G (s)
constexpr float post_stall_speed=500/3.6f;
inline void post_stall_override(LogicState& s,const LogicInput& in,LogicOutput& out) {
    const bool high_g=in.throttle>0.5f && in.brake>0.5f;
    const bool pressed=high_g && !s.post_stall_high_g;
    s.post_stall_high_g=high_g;
    if(!high_g) { s.post_stall_hold=0; return; }
    const float speed=std::sqrt(in.vel_x*in.vel_x+in.vel_y*in.vel_y+in.vel_z*in.vel_z);
    if(pressed && (in.request&1)) {
        if(speed<post_stall_speed) s.post_stall_hold=post_stall_pull;
        if(!out.event[0]) snprintf(out.event,sizeof(out.event),speed<post_stall_speed?"post-stall: entry at %.0f km/h"
                                   :"post-stall: %.0f km/h, over 500 (an ordinary high-G turn)",speed*3.6f);
    }
    if(s.post_stall_hold>0) {
        const bool centred=s.post_stall_hold>post_stall_pull-post_stall_entry;
        s.post_stall_hold-=in.dt;
        if(out.pitch>0) {
            out.pitch=centred?0.0f:1.0f;
            s.pitch_out.value=out.pitch;
            s.pitch_history.u[s.pitch_history.head]=out.pitch;   // what was sent, for the model
        }
    }
    // the game's rule, with what was sent: a post-stall may begin (requested or not)
    if(pressed && speed<post_stall_speed && out.pitch<=0.05f) { s.psm_entered=true; s.psm_time=0; s.psm_off_max=0; }
}
constexpr float unmodelled_off=30;     // deg between the nose and the flight path: post-stall
// The nose in post-stall (FA-36, logged 2026-10-06): centred it slows by about 160 deg/s^2
// (125-176 at 330-370 km/h), a full push stops it at about 350.
constexpr float post_stall_coast=160, post_stall_push=350;
// After the flight logic's own step: the test's pitch over it while a step runs.
inline void test_override(LogicState& s,const LogicInput& in,LogicOutput& out) {
    int count=0;
    const TestStep* plan=test_plan(count);
    ManeuverTest& m=s.test;
    const bool high_g=in.throttle>0.5f && in.brake>0.5f;
    const bool pressed=high_g && !m.high_g;
    m.high_g=high_g;
    if(in.keyboard) {
        m=ManeuverTest{};
        snprintf(out.event,sizeof(out.event),"TEST aborted: keyboard input");
        return;
    }
    const Basis b=basis(in.pitch,in.yaw,in.roll);
    const FlightPath path=flight_path(b,in);
    const V vel{in.vel_x,in.vel_y,in.vel_z};
    const float off=path.speed>1 ? std::acos(std::clamp(dot(b.f,vel)/path.speed,-1.0f,1.0f))/rad : 0;
    const TestStep& step=plan[m.step];
    if(m.phase==0) {
        if(!pressed) return;
        if(path.speed>=500/3.6f || in.altitude<1500) {
            snprintf(out.event,sizeof(out.event),"TEST %d/%d not started: %.0f km/h, %.0f m (needs under 500 km/h, over 1500 m)",
                     m.step+1,count,path.speed*3.6f,in.altitude);
            return;
        }
        m.phase=1; m.time=0; m.off_max=0;
        snprintf(out.event,sizeof(out.event),"TEST %d/%d start: %.0f km/h, pull after %.2f s, let go at %.0f deg (0 = held)",
                 m.step+1,count,path.speed*3.6f,step.delay,step.release);
    }
    m.time+=in.dt;
    m.off_max=std::max(m.off_max,off);
    if(m.phase==1 && m.time>=step.delay) { m.phase=2; m.time=0; }
    // Held: until the flight path has come round onto the nose (it swings within ~0.5 s once
    // the nose is well past 90 deg) or the time is up.
    const bool swung=m.off_max>60 && off<20;
    if(m.phase==2 && ((step.release>0 && off>=step.release) || (step.release<=0 && swung) || m.time>=test_hold)) {
        snprintf(out.event,sizeof(out.event),"TEST %d let go: %.0f deg off the path after %.2f s, %.0f km/h, pitch %.0f deg/s",
                 m.step+1,off,m.time,path.speed*3.6f,in.pitch_rate);
        m.phase=3; m.time=0;
    }
    if(m.phase==3 && (swung || (m.time>0.8f && m.off_max<45) || m.time>=test_watch)) {   // <45 after 0.8 s: no post-stall
        const int next=m.step+1;
        snprintf(out.event,sizeof(out.event),"TEST %d done: nose %.0f deg off the path at most%s",
                 next,m.off_max,next<count?"":" | TEST plan done");
        m=ManeuverTest{};
        m.high_g=high_g;
        if(next<count) m.step=next;
        return;
    }
    out.pitch=m.phase==2?1.0f:0.0f;
    s.pitch_out.value=out.pitch;
    s.pitch_history.u[s.pitch_history.head]=out.pitch;   // what was sent, for the model
    out.diag[DiagTail]=float(10*(m.step+1)+m.phase);
}

inline void logic_step(const Tuning& t,LogicState& s,const LogicInput& in,LogicOutput& out) {
    out=LogicOutput{};
    if(s.cal.step>=0 && calibration_hold(s,in,out)) return;
    s.maneuver.t=t.maneuver;
    if(s.ident.tag!=Identification::valid_tag || s.ident.aircraft!=in.aircraft) s.ident.reset(in.aircraft,t.plant);
    Plant base=t.ident_memory>0 ? s.ident.adapt(t.plant,t.ident_trust) : t.plant;
    // Applied to the model before the authority estimate, so the estimate does not absorb
    // the high-G turn and over-predict the pull for seconds after it ends.
    const bool high_g=in.throttle>0.5f && in.brake>0.5f;
    if(high_g) base.pitch_pull*=std::max(t.highg_pull,0.1f);
    if(high_g!=s.high_g_logged && !out.event[0]) {
        s.high_g_logged=high_g;
        snprintf(out.event,sizeof(out.event),"high-G turn %s",high_g?"on":"off");
    }
    const Basis b=basis(in.pitch,in.yaw,in.roll);
    const float upright=b.u.z;
    // Advance the game's input smoothing by the commands now reaching the aircraft.
    {
        float unused_pred=0, unused_filtered=0, unused_travel=0;
        auto hold=[](float r,float,float) { return r; };
        predict(s.pitch_history,base.pitch_delay,base.pitch_input,base.pitch_release,0,s.pitch_smoothed,hold,
                unused_pred,unused_filtered,unused_travel);
        predict(s.roll_history,base.roll_delay,base.roll_input,base.roll_input,0,s.roll_smoothed,hold,
                unused_pred,unused_filtered,unused_travel);
    }
    // Authority: least squares of the measured rate on the unit-authority response, both
    // low-passed over authority_filter so a modest lag mismatch is not read as authority,
    // with exponential forgetting, a prior of 1 worth ~0.3 s of 20 deg/s response, and
    // only authority_trust of the estimated change applied (a slower-than-modelled
    // aircraft still reads somewhat weak, and full trust overshot it in sim).
    // Post-stall (the nose more than unmodelled_off from the flight path; a high-G turn peaks
    // near 20): the game turns the nose at 110-190 deg/s whatever the stick, and then swings
    // the flight path onto it, nothing the model describes. Learning from it, the pull
    // authority read 76 against 58-61 before (the full-stick rate predicted 45-50 deg/s after
    // the swing, measured 10-15: the pull eased off and the nose stalled on its way to the
    // pointer), then ran below the aircraft (braked late, overshot, a full push and a roll
    // back: a Z at the pointer; logged, FA-36, 2026-10-06). So the authority, the online
    // identification and the trim learn nothing then, nor for the filters' time after.
    // Post-stall only after the game's entry (the high-G pressed below 500 km/h with the pitch not
    // pulling, as sent): the nose off the path alone also took the X-40A's ordinary high-G turns,
    // up to 58 deg off (25 spells, 12 s in one sortie, the learning held and the pitch switched to
    // the post-stall rule; logged 2026-10-07). Over when the path has come round onto the nose
    // (within 20 deg after more than unmodelled_off), or no swing within 1.5 s, or after 6 s.
    bool post_stall=false;
    {
        const float speed=std::sqrt(in.vel_x*in.vel_x+in.vel_y*in.vel_y+in.vel_z*in.vel_z);
        const float along=speed>1?(in.vel_x*b.f.x+in.vel_y*b.f.y+in.vel_z*b.f.z)/speed:1;
        const float off=std::acos(std::clamp(along,-1.0f,1.0f))/rad;
        if(s.psm_entered) {
            s.psm_time+=in.dt;
            s.psm_off_max=std::max(s.psm_off_max,off);
            if((s.psm_off_max>unmodelled_off && off<20) || (s.psm_time>1.5f && s.psm_off_max<unmodelled_off) || s.psm_time>6)
                s.psm_entered=false;
        }
        post_stall=s.psm_entered && speed>1 && off>unmodelled_off;
        if(post_stall) s.unmodelled=t.authority_filter+0.3f;
        else s.unmodelled=std::max(0.0f,s.unmodelled-in.dt);
    }
    const bool modelled=s.unmodelled<=0;
    Plant p=base;
    if(t.authority_memory>0 && in.dt>0) {
        const float kp=1-std::exp(-in.dt/base.pitch_tau), forget=std::exp(-in.dt/t.authority_memory);
        s.pull_unit+=(pitch_steady(base,std::max(s.pitch_smoothed,0.0f),0)-s.pull_unit)*kp;
        s.push_unit+=(pitch_steady(base,std::min(s.pitch_smoothed,0.0f),upright)+base.pitch_gravity*upright-s.push_unit)*kp;
        s.pitch_free+=(-base.pitch_gravity*upright-s.pitch_free)*kp;
        s.roll_unit=std::clamp(s.roll_unit+in.dt*(roll_accel_for(base,s.roll_smoothed)-s.roll_unit/base.roll_tau),
                               -base.roll_max_rate,base.roll_max_rate);
        const float kf=t.authority_filter>0?1-std::exp(-in.dt/t.authority_filter):1.0f;
        s.pull_f+=(s.pull_unit-s.pull_f)*kf; s.push_f+=(s.push_unit-s.push_f)*kf;
        s.pitch_yf+=(in.pitch_rate-s.pitch_free-s.pitch_yf)*kf;
        s.roll_f+=(s.roll_unit-s.roll_f)*kf; s.roll_yf+=(in.roll_rate-s.roll_yf)*kf;
        auto decay=[&](float& sum,float add) { if(modelled) sum=sum*forget+add*in.dt; };
        decay(s.s_uu,s.pull_f*s.pull_f); decay(s.s_dd,s.push_f*s.push_f); decay(s.s_ud,s.pull_f*s.push_f);
        decay(s.s_uy,s.pull_f*s.pitch_yf); decay(s.s_dy,s.push_f*s.pitch_yf);
        decay(s.roll_zz,s.roll_f*s.roll_f); decay(s.roll_zy,s.roll_f*s.roll_yf);
        constexpr float prior=20.0f*20.0f*0.3f;
        auto applied=[&](float estimate) {
            return std::clamp(1+t.authority_trust*(estimate-1),t.authority_min,t.authority_max);
        };
        // Two-parameter least squares (pull, push), each with the prior toward 1.
        const float a00=s.s_uu+prior, a11=s.s_dd+prior, a01=s.s_ud, b0=s.s_uy+prior, b1=s.s_dy+prior;
        const float det=a00*a11-a01*a01;
        s.pitch_scale=applied((b0*a11-a01*b1)/det);
        s.push_scale=applied((a00*b1-a01*b0)/det);
        s.roll_scale=applied((s.roll_zy+prior)/(s.roll_zz+prior));
        p.pitch_pull*=s.pitch_scale; p.pitch_push*=s.push_scale;
        p.roll_accel*=s.roll_scale; p.roll_max_rate*=s.roll_scale;
    }

    // Pitch: first-order plant with gravity. Predict through the delay, pick the
    // fastest rate that can still be stopped on the target with the opposite
    // authority (push is weaker than pull), then invert the plant for the input.
    auto pitch_step=[&](float r,float f,float dt) { return r+dt*(pitch_steady(p,f,upright)-r)/p.pitch_tau; };
    auto roll_step=[&](float r,float f,float dt) {
        return std::clamp(r+dt*(roll_accel_for(p,f)-r/p.roll_tau),-p.roll_max_rate,p.roll_max_rate);
    };
    // Observer: carry the estimated rates with the model (smoothing already advanced
    // above), then pull them toward the measurement.
    {
        if(!s.observer_primed || t.rate_observer<=0) {
            s.pitch_rate_est=in.pitch_rate; s.roll_rate_est=in.roll_rate; s.observer_primed=true;
        } else {
            const float k=1-std::exp(-t.rate_observer*in.dt);
            s.pitch_rate_est=pitch_step(s.pitch_rate_est,s.pitch_smoothed,in.dt);
            s.roll_rate_est=roll_step(s.roll_rate_est,s.roll_smoothed,in.dt);
            s.pitch_rate_est+=(in.pitch_rate-s.pitch_rate_est)*k;
            s.roll_rate_est+=(in.roll_rate-s.roll_rate_est)*k;
        }
    }
    float pitch_pred=0, pitch_filtered=0, pitch_travel=0;
    predict(s.pitch_history,p.pitch_delay,p.pitch_input,p.pitch_release,s.pitch_rate_est,s.pitch_smoothed,pitch_step,
            pitch_pred,pitch_filtered,pitch_travel,false);
    // Pitch still certain to come: in flight, then the shortest stop at full authority.
    const float stop_authority=pitch_pred>=0?p.push_rate(upright):p.pull_rate(upright);
    const float pitch_coming=pitch_travel+std::copysign((p.pitch_tau+p.pitch_release)*
        (std::abs(pitch_pred)-stop_authority*std::log1p(std::abs(pitch_pred)/stop_authority)),pitch_pred);

    // Target motion across the nose's pitch and yaw axes, low-passed; a jump (view snap,
    // re-anchor) is far faster than any real motion and is ignored.
    {
        const V aim{in.aim_x,in.aim_y,in.aim_z};
        if(in.aim_attached) { s.lead_pitch=0; s.lead_yaw=0; }
        else if(s.aim_primed && in.dt>0 && t.lead_gain>0) {
            const V d{(aim.x-s.aim_prev[0])/in.dt,(aim.y-s.aim_prev[1])/in.dt,(aim.z-s.aim_prev[2])/in.dt};
            if(std::sqrt(dot(d,d))/rad<3*t.lead_max) {
                const float k=1-std::exp(-in.dt/std::max(t.lead_filter,0.01f));
                s.lead_pitch+=(dot(d,b.u)/rad-s.lead_pitch)*k;
                s.lead_yaw+=(dot(d,b.r)/rad-s.lead_yaw)*k;
            }
        }
        s.aim_prev[0]=aim.x; s.aim_prev[1]=aim.y; s.aim_prev[2]=aim.z; s.aim_primed=true;
    }
    const float lead_pitch_raw=t.lead_gain*std::clamp(s.lead_pitch,-t.lead_max,t.lead_max);
    const float lead_yaw=t.lead_gain*std::clamp(s.lead_yaw,-t.lead_max,t.lead_max);
    // Seconds to sea level at the present sink rate (the push guard near the ground).
    const float ground_time=in.vel_z<-5.0f?std::max(0.0f,in.altitude)/-in.vel_z:1e9f;
    const Guidance g=s.maneuver.step(b,{in.aim_x,in.aim_y,in.aim_z},p,pitch_coming,in.roll_rate,
                                     s.lead_pitch,s.lead_yaw,in.dt,ground_time);
    calibration_settle(s.cal,in,g);
    if(g.tail!=s.tail_logged && !out.event[0]) {
        s.tail_logged=g.tail;
        snprintf(out.event,sizeof(out.event),"maneuver: %s angle=%.1f",
                 g.tail?"rear reversal committed":"rear reversal complete",g.angle);
    }
    const float w=g.turn_weight;
    // While the roll must come first (the lift more than 90 deg from the target) the target's
    // motion may hurry the pitch that is coming, never oppose it: the aim moving toward the
    // floor read as a full push mid-roll (logged, FA-36 at 550-700 m/s: 16 s in one sortie,
    // 2.8 s of it in high-G turns, the negative G undone after the roll).
    const float lead_along=g.pushing?std::min(lead_pitch_raw,0.0f):std::max(lead_pitch_raw,0.0f);
    const float lead_pitch=lead_pitch_raw+t.lead_gate*(1-g.pitch_gate)*(lead_along-lead_pitch_raw);
    const float pitch_left=g.pitch_error-pitch_travel;
    // The trim learns a steady model offset near the target and keeps it through
    // maneuvers (it is a few deg/s against rates of tens). Decaying it in every turn
    // relearned it from zero at each arrival, and the nose sat ~0.7 deg high (logged).
    if(std::abs(g.pitch_error)<3 && w<0.5f && modelled)
        s.pitch_trim=std::clamp(s.pitch_trim+t.pitch_trim*g.pitch_error*in.dt,-t.pitch_trim_limit,t.pitch_trim_limit);
    else if(t.pitch_trim_keep<=0) s.pitch_trim*=std::max(0.0f,1-2*in.dt);
    const bool nose_up=pitch_left>=0;
    const float pitch_cap=0.9f*(nose_up?p.pull_rate(upright):p.push_rate(upright));
    const float pitch_opposing=t.pitch_brake*(nose_up?p.push_rate(upright):p.pull_rate(upright));
    // The game's input smoothing lengthens the stop on top of the aircraft's own lag.
    const float pitch_want=desired_rate(pitch_left,pitch_cap,
        stoppable_first_order(std::abs(pitch_left),pitch_opposing,p.pitch_tau+p.pitch_release,pitch_cap),
        t.pitch_gain*(1+t.pitch_gain_near*smoothstep(4.0f,1.0f,std::abs(pitch_left))))+s.pitch_trim+lead_pitch;
    const float pitch_hold=pitch_input_for(p,pitch_want+p.pitch_gravity*upright,upright);
    // Slowing a rotation that is still wanted in its direction uses the planned braking
    // authority, not the full opposite stick: full push to shed a pull mid-turn read as
    // a dive and a stutter in game. Reversing (want of the other sign) may use it all.
    float pitch_low=-1, pitch_high=1;
    if(t.pitch_unload>0) {
        if(pitch_want>=0 && pitch_pred>pitch_want)
            pitch_low=pitch_input_for(p,-t.pitch_unload*p.push_rate(upright)+p.pitch_gravity*upright,upright);
        else if(pitch_want<=0 && pitch_pred<pitch_want)
            pitch_high=pitch_input_for(p,t.pitch_unload*p.pull_rate(upright)+p.pitch_gravity*upright,upright);
    }
    // Far from the target, when the plan would still take more than the modelled full
    // rate, the stick goes all the way: tracking 0.9 of the model held ~0.9 stick in
    // big turns (logged), giving away the aircraft's real, often higher, top rate.
    const float pitch_full=nose_up?p.pull_rate(upright):p.push_rate(upright);
    const bool pitch_all_out=std::min(t.pitch_gain*std::abs(pitch_left),
        stoppable_first_order(std::abs(pitch_left),pitch_opposing,p.pitch_tau+p.pitch_release,3*pitch_full))>=pitch_full;
    float pitch_cmd=pitch_all_out ? (nose_up?1.0f:-1.0f)
        : std::clamp(pitch_hold+t.pitch_kd*(pitch_want-pitch_pred),pitch_low,pitch_high);
    // ...but no further than the pitch already certain to come with the hardest stop: past that
    // the full pull carries the nose through the aim (bench, every aircraft from the game's own
    // parameters: a Su-35 in a high-G pull at 125 deg/s went 40 deg past an aim 10 deg off,
    // into a vertical dive and the ground, where the FA-36's slower pull had stopped in time).
    if(high_g && t.highg_full_from>0 && g.angle>t.highg_full_from && !g.pushing && g.pitch_error>std::max(0.5f,pitch_coming)) pitch_cmd=1;

    // Roll: acceleration-limited plant. Same prediction; braking distance r^2/(2a).
    float roll_pred=0, roll_filtered=0, roll_travel=0;
    predict(s.roll_history,p.roll_delay,p.roll_input,p.roll_input,s.roll_rate_est,s.roll_smoothed,roll_step,
            roll_pred,roll_filtered,roll_travel,false);
    const float roll_left=g.roll_error-roll_travel;
    const float roll_brake=t.roll_brake*p.roll_accel;
    const float roll_want=desired_rate(roll_left,0.95f*p.roll_max_rate,
        std::sqrt(2*roll_brake*std::abs(roll_left)),t.roll_gain);
    const bool roll_all_out=std::min(t.roll_gain*std::abs(roll_left),
        std::sqrt(2*roll_brake*std::abs(roll_left)))>=p.roll_max_rate;
    float roll_cmd=roll_all_out ? (roll_left>=0?1.0f:-1.0f)
        : roll_input_for(p,t.roll_kv*(roll_want-roll_pred)+roll_want/p.roll_tau);

    // Rudder only trims small lateral errors and fades out entirely in turns.
    const float yaw_rate=std::clamp(dead(g.yaw_error,t.yaw_dead)*t.yaw_gain+lead_yaw,-t.yaw_max_rate,t.yaw_max_rate);
    float ycmd=g.yaw_weight*std::clamp((yaw_rate-in.yaw_rate*t.yaw_damping)/10.0f,-t.yaw_limit,t.yaw_limit);
    // (not while the low-altitude guard is on: the look-ahead knows nothing of the ground, and
    // charging the roll rate it held back the roll upright the guard wanted, into the sea)
    if(t.fine_search>0 && g.angle<t.fine_angle && !g.tail && !g.pushing && in.keyboard==0 &&
       !(t.maneuver.ground_guard>0 && ground_time<t.maneuver.ground_guard)) {
        // From where the commands in flight leave the aircraft: attitude advanced by the
        // travel still to come, rates and input smoothing as they will then be.
        auto turn=[](V& a,V& c,float deg) { const float k=deg*rad, cs=std::cos(k), sn=std::sin(k); const V a0=a; a=a*cs+c*sn; c=c*cs-a0*sn; };
        Basis b0=b;
        turn(b0.f,b0.u,pitch_travel); turn(b0.u,b0.r,roll_travel);
        const V aim0{in.aim_x,in.aim_y,in.aim_z};
        const float start=p.pitch_delay, dts=0.05f;
        const int steps=std::max(1,int(t.fine_horizon/dts+0.5f));
        const float still=1-smoothstep(t.maneuver.pursuit_speed,2*t.maneuver.pursuit_speed,std::hypot(s.lead_pitch,s.lead_yaw));
        auto aim_at=[&](float tt) { return unit(aim0+(b.u*s.lead_pitch+b.r*s.lead_yaw)*(tt*rad)); };
        auto law=[&](const Basis& k,const V& aim) {
            const float ep=std::atan2(dot(aim,k.u),dot(aim,k.f))/rad, upr=k.u.z;
            return pitch_input_for(p,std::clamp(t.pitch_gain*ep,-0.9f*p.push_rate(upr),0.9f*p.pull_rate(upr))+p.pitch_gravity*upr,upr);
        };
        const float law0=law(b0,aim_at(start));
        auto cost=[&](float up,float ur,float uy) {
            Basis k=b0;
            float q=pitch_pred, fq=pitch_filtered, pr=roll_pred, fr=roll_filtered, yr=in.yaw_rate, c=0;
            for(int i=0;i<steps;++i) {
                const float upi=t.fine_follow>0&&i>0 ? std::clamp(up+t.fine_follow*(law(k,aim_at(start+i*dts))-law0),-1.0f,1.0f) : up;
                fq=input_smooth(fq,upi,dts,p.pitch_input,p.pitch_release);
                // less the trim: the steady model offset the rules have learned and fly against
                q+=dts*(pitch_steady(p,fq,k.u.z)-s.pitch_trim-q)/p.pitch_tau;
                fr=input_smooth(fr,ur,dts,p.roll_input,p.roll_input);
                pr=std::clamp(pr+dts*(roll_accel_for(p,fr)-pr/p.roll_tau),-p.roll_max_rate,p.roll_max_rate);
                yr+=dts*(uy*t.fine_yaw_rate-yr)/0.35f;
                turn(k.f,k.u,q*dts); turn(k.u,k.r,pr*dts); turn(k.f,k.r,yr*dts);
                // the aim keeps moving across the nose as it has been (low-passed, deg/s)
                const float tt=start+(i+1)*dts;
                const V aim=unit(aim0+(b.u*s.lead_pitch+b.r*s.lead_yaw)*(tt*rad));
                const float e=std::acos(std::clamp(dot(k.f,aim),-1.0f,1.0f))/rad;
                c+=(e*e+e)*dts+t.fine_roll*pr*pr*dts;   // the linear part keeps the last degree worth closing
            }
            if(t.fine_to_go>0) {
                const V aim=aim_at(start+steps*dts);
                const float e=std::acos(std::clamp(dot(k.f,aim),-1.0f,1.0f))/rad;
                // the target's direction around the nose: 0 over the canopy, 180 below the floor
                const float x=dot(aim,k.r), y=dot(aim,k.u), phi=std::atan2(std::abs(x),y)/rad, n=std::hypot(x,y);
                // how fast the aim moves away from the nose (deg/s): a rudder slower than the
                // target never catches it, however well it does over the horizon
                const V lead=b.u*s.lead_pitch+b.r*s.lead_yaw;
                const float away=n>1e-6f?(dot(lead,k.r)*x+dot(lead,k.u)*y)/n:0.0f;
                auto way=[&](float lag,float rate) { return lag+(e+std::max(0.0f,away)*lag)/std::max(rate-away,0.1f*rate); };
                const float lag=p.pitch_delay+p.pitch_input+p.pitch_tau;
                const float go=std::min({way(p.roll_time(phi)+lag,p.pull_rate(k.u.z)),
                                         way(p.roll_time(180-phi)+lag,p.push_rate(k.u.z)),
                                         way(0.35f,std::max(t.fine_yaw_rate,0.1f))});
                { const float beyond=std::max(0.0f,e-t.maneuver.near_end); c+=t.fine_to_go*beyond*beyond/3*go; }   // closing evenly over go; inside near_end the fine tracking's
            }
            const float bank=std::atan2(k.r.z,k.u.z)/rad;
            // the bank counts as much as the horizon is there to bank against (nose straight up or
            // down it is undefined and swung about, and the search rolled after it: sim, 90 deg
            // dive), and only for an aim that is not moving: a chased one has no arrival to level
            // the wings for (as chase_level; charged anyway, the bank held for a circling target
            // was pulled level and back every few seconds: bench, Selene's shield phase)
            c+=t.fine_level*bank*bank*(k.r.z*k.r.z+k.u.z*k.u.z)*still;
            c+=t.fine_smooth*((up-s.pitch_out.value)*(up-s.pitch_out.value)+(ur-s.roll_out.value)*(ur-s.roll_out.value)+
                              (uy-s.yaw.value)*(uy-s.yaw.value));
            return c;
        };
        // coordinate search: roll, then pitch, then rudder, twice; the rules' sticks always a candidate
        float bp=pitch_cmd, br=roll_cmd, by=ycmd, best=cost(bp,br,by);
        auto tryit=[&](float up,float ur,float uy) {
            up=std::clamp(up,-1.0f,1.0f); ur=std::clamp(ur,-1.0f,1.0f); uy=std::clamp(uy,-t.yaw_limit,t.yaw_limit);
            const float c=cost(up,ur,uy);
            if(c<best) { best=c; bp=up; br=ur; by=uy; }
        };
        for(int pass=0;pass<2;++pass) {
            const float r0=br, p0=bp, y0=by;
            for(float ur:{0.0f,0.5f*r0,r0-0.15f,r0+0.15f,-0.3f,0.3f}) tryit(p0,ur,y0);
            const float r1=br;
            if(t.fine_pitch>0) for(float up:{p0-0.2f,p0-0.08f,p0+0.08f,p0+0.2f,pitch_hold}) tryit(up,r1,y0);
            const float p1=bp;
            for(float uy:{0.0f,y0-0.2f,y0+0.2f,-t.yaw_limit,t.yaw_limit}) tryit(p1,r1,uy);
        }
        pitch_cmd=bp; roll_cmd=br; ycmd=by;
    }
    out.pitch=s.pitch_out.step(pitch_cmd,in.dt,t.pitch_slew);
    // Post-stall: while the nose is swinging, a full pull until what is left is what it carries
    // on by itself centred, then centred, and a push only for the part of the stop the centred
    // stick does not make (a share of it) or once past the pointer. Centred, the nose carries on
    // by itself (let go at 60-121 deg off the path at 116-140 deg/s it went another 50-55 deg);
    // the logic's own braking ended the swing short (a push of -0.33 at 44 deg to go and 130
    // deg/s: 15 deg/s 0.3 s later, stopped 37 deg short, then held still 1.4 s while the game
    // swung the path onto it), and its full push 1-4 deg past the pointer sent the nose back at
    // 35 deg/s and a full pull after it (logged, FA-36, 2026-10-06). Once the swing is over
    // (under 20 deg/s) the game is bringing the path round and the nose hardly answers: a pull
    // for what is still far, centred near.
    if(post_stall) {
        const float q=in.pitch_rate, e=g.pitch_error;
        if(q>20) {
            const float coast=q*q/(2*post_stall_coast);
            if(e>coast) out.pitch=1.0f;
            else if(e>0.5f) out.pitch=-std::clamp((q*q/(2*e)-post_stall_coast)/(post_stall_push-post_stall_coast),0.0f,1.0f);
            else out.pitch=-1.0f;
        } else if(std::abs(e)<10) out.pitch=0.0f;
        else if(e>0) out.pitch=1.0f;
        s.pitch_out.value=out.pitch;
    }
    out.roll=s.roll_out.step(roll_cmd,in.dt,t.roll_slew);
    out.yaw=s.yaw.step(ycmd,in.dt,t.yaw_slew);
    s.pitch_history.push(out.pitch,in.dt);
    s.roll_history.push(out.roll,in.dt);
    s.ident.observe(t.plant,s.pitch_history,s.roll_history,in.pitch_rate,in.roll_rate,upright,in.dt,
                    in.keyboard==0 && modelled,t.ident_memory,out);
    // want: desired rates; pred: rates predicted after the delay; abi marks controller-convention signs.
    snprintf(out.trace,sizeof(out.trace),
        "dt=%.4f angle=%.2f w=%.2f tail=%d push=%d err=(p%.2f,y%.2f,r%.1f) roll=%.1f "
        "rate=(%.1f,%.1f,%.1f) cmd=(%.3f,%.3f,%.3f) out=(%.3f,%.3f) kb=%d abi=%d "
        "want=(%.1f,%.1f) pred=(%.1f,%.1f) trim=%.2f est=(%.0f,%.0f,%.2f,%.0f,%.2f)",
        in.dt,g.angle,w,g.tail?1:0,g.pushing?1:0,g.pitch_error,g.yaw_error,g.roll_error,in.roll,
        in.pitch_rate,in.yaw_rate,in.roll_rate,out.pitch,out.yaw,out.roll,out.pitch,out.roll,
        in.keyboard,logic_abi,pitch_want,roll_want,pitch_pred,roll_pred,s.pitch_trim,
        p.pitch_pull,p.pitch_push,p.pitch_tau,p.roll_accel,p.roll_tau);
    const FlightPath path=flight_path(b,in);
    {
        const float values[DiagCount]={g.angle,g.pitch_error,g.yaw_error,g.roll_error,g.turn_weight,g.pushing?1.0f:0.0f,
            g.tail?1.0f:0.0f,pitch_want,roll_want,pitch_pred,roll_pred,pitch_coming,path.speed,path.aoa,path.beta,path.nz};
        for(int i=0;i<DiagCount;++i) out.diag[i]=values[i];
    }
    // Automatic maneuver trace: starts when the nose is more than 4 deg off target and
    // stops after 2 s within 1 deg, so every maneuver is recorded without pressing F4.
    // Compact enough for the event line; fields documented in docs/maneuver-spec.md.
    if(t.auto_trace>0) {
        // auto_trace=2 records every frame (a whole mission), 1 only maneuvers.
        if(g.angle>4 || t.auto_trace>=2) { s.recording=true; s.settled_for=0; }
        else if(s.recording) {
            s.settled_for=g.angle<1 ? s.settled_for+in.dt : 0;
            if(s.settled_for>2) s.recording=false;
        }
        if(s.recording && !out.event[0])
            snprintf(out.event,sizeof(out.event),"AT %.3f %.1f %.2f %d %d %.1f %.1f %.1f %.1f %.1f %.3f %.3f %.0f %.0f %.0f %.0f %d %.0f %.2f %.2f %.2f %d %.1f %.1f %.1f %.2f %.0f",
                     in.dt,g.angle,w,g.tail?1:0,g.pushing?1:0,g.pitch_error,g.roll_error,in.roll,
                     in.pitch_rate,in.roll_rate,out.pitch,out.roll,pitch_want,roll_want,pitch_pred,roll_pred,in.keyboard,
                     pitch_coming,s.pitch_scale,s.roll_scale,s.push_scale,high_g?1:0,
                     path.speed,path.aoa,path.beta,path.nz,in.altitude);
    }
}

// Exported surface, identical for the built-in copy and the hot-swappable DLL.
namespace logic_api {
inline Tuning tuning;
inline int abi(unsigned* input_size,unsigned* output_size,unsigned* state_size) {
    *input_size=sizeof(LogicInput); *output_size=sizeof(LogicOutput); *state_size=sizeof(LogicState);
    return logic_abi;
}
inline float calibrate_seen=0, maneuver_test_seen=0;
inline bool calibrate_request=false, maneuver_test_request=false;
inline void configure(const wchar_t* config_path) {
    tuning=read_tuning(config_path);
    if(tuning.calibrate!=calibrate_seen) {
        calibrate_seen=tuning.calibrate;
        calibrate_request=tuning.calibrate>0;
    }
    if(tuning.maneuver_test!=maneuver_test_seen) {
        maneuver_test_seen=tuning.maneuver_test;
        maneuver_test_request=tuning.maneuver_test>0;
    }
}
// Pauses, recenters and hitches reset control state but keep what was learned about
// the aircraft; a stale or foreign buffer (tag mismatch) starts fresh.
inline void reset_keeping_aircraft(LogicState& s) {
    const Identification keep=s.ident;
    const ManeuverTest test=s.test;
    new(&s) LogicState();
    if(keep.tag==Identification::valid_tag) { s.ident=keep; s.ident.primed=false; }
    // An armed test plan stays armed (a pause or F9 mid-step); a step interrupted after the
    // stick was let go counts as done, one interrupted before runs again on the next press.
    int count=0;
    test_plan(count);
    if(keep.tag==Identification::valid_tag && test.tag==ManeuverTest::valid_tag && test.step>=0 && test.step<count) {
        s.test.step=test.phase>=3 ? test.step+1 : test.step;
        if(s.test.step>=count) s.test.step=-1;
    }
}
inline void reset(void* state) { reset_keeping_aircraft(*static_cast<LogicState*>(state)); }
inline void step(void* state,const LogicInput* in,LogicOutput* out) {
    LogicState& s=*static_cast<LogicState*>(state);
    if(calibrate_request) {
        calibrate_request=false;
        s.cal=Calibration{};
        s.cal.step=0;
        s.cal.announce=true;
    }
    if(maneuver_test_request) {
        maneuver_test_request=false;
        s.test=ManeuverTest{};
        s.test.step=0;
        int count=0;
        test_plan(count);
        snprintf(out->event,sizeof(out->event),"TEST armed: %d steps; each below 500 km/h with high-G held runs one",count);
        LogicOutput first{};
        logic_step(tuning,s,*in,first);
        std::memcpy(first.event,out->event,sizeof(first.event));
        *out=first;
        return;
    }
    logic_step(tuning,s,*in,*out);
    post_stall_override(s,*in,*out);
    if(s.test.step>=0) test_override(s,*in,*out);
}
}
}
