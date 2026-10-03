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
constexpr int logic_abi=1;
// Controller convention: positive pitch raises the nose, roll rolls right, yaw yaws right.
struct LogicInput {
    float pitch,yaw,roll;              // aircraft attitude, degrees
    float aim_x,aim_y,aim_z;           // world target direction (unit)
    float pitch_rate,yaw_rate,roll_rate; // filtered body rates, degrees/second
    float dt;
    int keyboard;                      // axes flown by keyboard: 1 pitch, 2 roll, 4 yaw
};
struct LogicOutput {
    float pitch,roll,yaw;              // axis values to write (before input-sign config)
    char trace[320];                   // per-frame diagnostic text
    char event[128];                   // non-empty when something notable happened
};
struct Tuning {
    ManeuverTuning maneuver;
    Plant plant;
    // Rate control: stick = steady input for the desired rate (measured curve) plus a
    // moderate correction toward it. Earlier full model inversion multiplied the gain by
    // the lag ratios and chattered at ~2.5 Hz in game. Brake with this fraction of the
    // opposite authority, and close the last degrees at gain (1/s). Tuned in the
    // closed-loop simulation (tests/flight_sim.cpp).
    float pitch_kd=0.25f;              // stick per deg/s of predicted rate error
    float roll_kv=1.5f;                // roll acceleration per deg/s of rate error (1/s)
    float pitch_brake=0.45f, roll_brake=0.45f;
    float pitch_gain=2.0f, roll_gain=2.0f;
    float pitch_trim=0.5f, pitch_trim_limit=4.0f; // slow integral on small pitch error
    // Stick slew (full scale per second). The game smooths stick input, so faster
    // swings are averaged away and only make the model-based loop chatter.
    float pitch_slew=20.0f, roll_slew=20.0f;
    float yaw_dead=0.2f, yaw_gain=1.5f, yaw_max_rate=6.0f, yaw_damping=0.45f, yaw_limit=0.6f;
    float yaw_slew=5.0f;
    float calibrate=0;                 // change to a new positive value to run input calibration
    float auto_trace=1;                // record each maneuver automatically (compact AT lines)
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
struct LogicState {
    Maneuver maneuver;
    CommandSlew yaw, pitch_out, roll_out;
    AxisHistory pitch_history, roll_history;
    float pitch_trim=0;
    float pitch_smoothed=0, roll_smoothed=0; // game input smoothing state, as of one delay ago
    bool tail_logged=false;
    bool recording=false;              // automatic per-maneuver trace
    float settled_for=0;
    Calibration cal;
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
    get(L"near_end",m.near_end); get(L"far_start",m.far_start);
    get(L"near_bank_gain",m.near_bank_gain); get(L"near_bank_max",m.near_bank_max);
    get(L"push_enter",m.push_enter); get(L"push_exit",m.push_exit); get(L"choice_margin",m.choice_margin);
    get(L"pull_full",m.pull_full); get(L"pull_none",m.pull_none);
    get(L"tail_enter",m.tail_enter); get(L"tail_exit",m.tail_exit); get(L"tail_bank",m.tail_bank);
    get(L"latch_limit",m.latch_limit); get(L"rollout_lead",m.rollout_lead);
    auto& p=t.plant;
    get(L"pitch_delay",p.pitch_delay); get(L"pitch_input",p.pitch_input);
    get(L"pitch_tau",p.pitch_tau); get(L"pitch_curve",p.pitch_curve);
    get(L"pitch_pull",p.pitch_pull); get(L"pitch_push",p.pitch_push); get(L"pitch_gravity",p.pitch_gravity);
    get(L"roll_delay",p.roll_delay); get(L"roll_input",p.roll_input);
    get(L"roll_accel",p.roll_accel); get(L"roll_curve",p.roll_curve);
    get(L"roll_max_rate",p.roll_max_rate); get(L"roll_tau",p.roll_tau);
    get(L"pitch_kd",t.pitch_kd); get(L"roll_kv",t.roll_kv);
    get(L"pitch_brake",t.pitch_brake); get(L"roll_brake",t.roll_brake);
    get(L"pitch_gain",t.pitch_gain); get(L"roll_gain",t.roll_gain);
    get(L"pitch_trim",t.pitch_trim); get(L"pitch_trim_limit",t.pitch_trim_limit);
    get(L"pitch_slew",t.pitch_slew); get(L"roll_slew",t.roll_slew);
    get(L"yaw_dead",t.yaw_dead); get(L"yaw_gain",t.yaw_gain); get(L"yaw_max_rate",t.yaw_max_rate);
    get(L"yaw_damping",t.yaw_damping); get(L"yaw_limit",t.yaw_limit); get(L"yaw_slew",t.yaw_slew);
    get(L"calibrate",t.calibrate);
    get(L"auto_trace",t.auto_trace);
    return t;
}

// Plant steps, inverses and stopping distances from the measured model.
inline float pitch_steady(const Plant& p,float u,float upright) {
    const float a=std::pow(std::min(std::abs(u),1.0f),p.pitch_curve);
    return (u>=0 ? p.pitch_pull*a : -p.pitch_push*a)-p.pitch_gravity*upright;
}
inline float pitch_input_for(const Plant& p,float aero) {
    return aero>=0 ? std::min(1.0f,std::pow(aero/p.pitch_pull,1/p.pitch_curve))
                   : -std::min(1.0f,std::pow(-aero/p.pitch_push,1/p.pitch_curve));
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
inline void predict(const AxisHistory& h,float delay,float smoothing,float rate,float& smoothed,Step step,
                    float& predicted,float& filtered,float& travel) {
    constexpr int size=AxisHistory::size;
    int n=0;
    float back=0;
    while(n<size-1 && back<delay) {
        const float dt=h.dt[(h.head-n+size)%size];
        if(dt<=0) break;
        back+=dt; ++n;
    }
    const int leaving=(h.head-n+size)%size;
    if(h.dt[leaving]>0) smoothed+=(h.u[leaving]-smoothed)*(1-std::exp(-h.dt[leaving]/smoothing));
    predicted=rate; filtered=smoothed; travel=0;
    for(int k=n-1;k>=0;--k) {
        const int i=(h.head-k+size)%size;
        filtered+=(h.u[i]-filtered)*(1-std::exp(-h.dt[i]/smoothing));
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

inline void logic_step(const Tuning& t,LogicState& s,const LogicInput& in,LogicOutput& out) {
    out=LogicOutput{};
    if(s.cal.step>=0 && calibration_hold(s,in,out)) return;
    s.maneuver.t=t.maneuver;
    const Plant& p=t.plant;
    const Basis b=basis(in.pitch,in.yaw,in.roll);
    const Guidance g=s.maneuver.step(b,{in.aim_x,in.aim_y,in.aim_z},p,in.pitch_rate);
    calibration_settle(s.cal,in,g);
    if(g.tail!=s.tail_logged && !out.event[0]) {
        s.tail_logged=g.tail;
        snprintf(out.event,sizeof(out.event),"maneuver: %s angle=%.1f",
                 g.tail?"rear reversal committed":"rear reversal complete",g.angle);
    }
    const float w=g.turn_weight, upright=b.u.z;

    // Pitch: first-order plant with gravity. Predict through the delay, pick the
    // fastest rate that can still be stopped on the target with the opposite
    // authority (push is weaker than pull), then invert the plant for the input.
    float pitch_pred=0, pitch_filtered=0, pitch_travel=0;
    predict(s.pitch_history,p.pitch_delay,p.pitch_input,in.pitch_rate,s.pitch_smoothed,[&](float r,float f,float dt) {
        return r+dt*(pitch_steady(p,f,upright)-r)/p.pitch_tau;
    },pitch_pred,pitch_filtered,pitch_travel);
    const float pitch_left=g.pitch_error-pitch_travel;
    if(std::abs(g.pitch_error)<3 && w<0.5f)
        s.pitch_trim=std::clamp(s.pitch_trim+t.pitch_trim*g.pitch_error*in.dt,-t.pitch_trim_limit,t.pitch_trim_limit);
    else s.pitch_trim*=std::max(0.0f,1-2*in.dt);
    const bool nose_up=pitch_left>=0;
    const float pitch_cap=0.9f*(nose_up?p.pull_rate(upright):p.push_rate(upright));
    const float pitch_opposing=t.pitch_brake*(nose_up?p.push_rate(upright):p.pull_rate(upright));
    // The game's input smoothing lengthens the stop on top of the aircraft's own lag.
    const float pitch_want=desired_rate(pitch_left,pitch_cap,
        stoppable_first_order(std::abs(pitch_left),pitch_opposing,p.pitch_tau+p.pitch_input,pitch_cap),
        t.pitch_gain)+s.pitch_trim;
    const float pitch_hold=pitch_input_for(p,pitch_want+p.pitch_gravity*upright);
    out.pitch=s.pitch_out.step(std::clamp(pitch_hold+t.pitch_kd*(pitch_want-pitch_pred),-1.0f,1.0f),
                               in.dt,t.pitch_slew);

    // Roll: acceleration-limited plant. Same prediction; braking distance r^2/(2a).
    float roll_pred=0, roll_filtered=0, roll_travel=0;
    predict(s.roll_history,p.roll_delay,p.roll_input,in.roll_rate,s.roll_smoothed,[&](float r,float f,float dt) {
        return std::clamp(r+dt*(roll_accel_for(p,f)-r/p.roll_tau),-p.roll_max_rate,p.roll_max_rate);
    },roll_pred,roll_filtered,roll_travel);
    const float roll_left=g.roll_error-roll_travel;
    const float roll_brake=t.roll_brake*p.roll_accel;
    const float roll_want=desired_rate(roll_left,0.95f*p.roll_max_rate,
        std::sqrt(2*roll_brake*std::abs(roll_left)),t.roll_gain);
    out.roll=s.roll_out.step(roll_input_for(p,t.roll_kv*(roll_want-roll_pred)+roll_want/p.roll_tau),
                             in.dt,t.roll_slew);

    // Rudder only trims small lateral errors and fades out entirely in turns.
    const float yaw_rate=std::clamp(dead(g.yaw_error,t.yaw_dead)*t.yaw_gain,-t.yaw_max_rate,t.yaw_max_rate);
    const float ycmd=(1-w)*std::clamp((yaw_rate-in.yaw_rate*t.yaw_damping)/10.0f,-t.yaw_limit,t.yaw_limit);
    out.yaw=s.yaw.step(ycmd,in.dt,t.yaw_slew);
    s.pitch_history.push(out.pitch,in.dt);
    s.roll_history.push(out.roll,in.dt);
    // want: desired rates; pred: rates predicted after the delay; abi marks controller-convention signs.
    snprintf(out.trace,sizeof(out.trace),
        "dt=%.4f angle=%.2f w=%.2f tail=%d push=%d err=(p%.2f,y%.2f,r%.1f) roll=%.1f "
        "rate=(%.1f,%.1f,%.1f) cmd=(%.3f,%.3f,%.3f) out=(%.3f,%.3f) kb=%d abi=%d "
        "want=(%.1f,%.1f) pred=(%.1f,%.1f) trim=%.2f",
        in.dt,g.angle,w,g.tail?1:0,g.pushing?1:0,g.pitch_error,g.yaw_error,g.roll_error,in.roll,
        in.pitch_rate,in.yaw_rate,in.roll_rate,out.pitch,out.yaw,out.roll,out.pitch,out.roll,
        in.keyboard,logic_abi,pitch_want,roll_want,pitch_pred,roll_pred,s.pitch_trim);
    // Automatic maneuver trace: starts when the nose is more than 4 deg off target and
    // stops after 2 s within 1 deg, so every maneuver is recorded without pressing F4.
    // Compact enough for the event line; fields documented in docs/maneuver-spec.md.
    if(t.auto_trace>0) {
        if(g.angle>4) { s.recording=true; s.settled_for=0; }
        else if(s.recording) {
            s.settled_for=g.angle<1 ? s.settled_for+in.dt : 0;
            if(s.settled_for>2) s.recording=false;
        }
        if(s.recording && !out.event[0])
            snprintf(out.event,sizeof(out.event),"AT %.4f %.2f %.2f %d %d %.2f %.1f %.1f %.1f %.1f %.3f %.3f %.1f %.1f %.1f %.1f %d",
                     in.dt,g.angle,w,g.tail?1:0,g.pushing?1:0,g.pitch_error,g.roll_error,in.roll,
                     in.pitch_rate,in.roll_rate,out.pitch,out.roll,pitch_want,roll_want,pitch_pred,roll_pred,in.keyboard);
    }
}

// Exported surface, identical for the built-in copy and the hot-swappable DLL.
namespace logic_api {
inline Tuning tuning;
inline int abi(unsigned* input_size,unsigned* output_size,unsigned* state_size) {
    *input_size=sizeof(LogicInput); *output_size=sizeof(LogicOutput); *state_size=sizeof(LogicState);
    return logic_abi;
}
inline float calibrate_seen=0;
inline bool calibrate_request=false;
inline void configure(const wchar_t* config_path) {
    tuning=read_tuning(config_path);
    if(tuning.calibrate!=calibrate_seen) {
        calibrate_seen=tuning.calibrate;
        calibrate_request=tuning.calibrate>0;
    }
}
inline void reset(void* state) { new(state) LogicState(); }
inline void step(void* state,const LogicInput* in,LogicOutput* out) {
    LogicState& s=*static_cast<LogicState*>(state);
    if(calibrate_request) {
        calibrate_request=false;
        s.cal=Calibration{};
        s.cal.step=0;
        s.cal.announce=true;
    }
    logic_step(tuning,s,*in,*out);
}
}
}
