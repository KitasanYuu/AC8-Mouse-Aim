// Closed-loop simulation of the real flight logic against the measured plant model.
// Reports time to target, overshoot and roll reversals for standard maneuvers, on the
// nominal model and on a deliberately slower/weaker one (model error robustness).
#define MANEUVER_DEBUG
#include <cstdio>
#include "../src/flight_logic.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include <random>
#include <string>

using namespace flight;

namespace {
// The game smooths stick input before the aircraft responds; in game, short full-stick
// pulses were averaged away (logged ~2.5 Hz full-stick chatter with ~1 deg/s response).
// Part of the measured onset delay is therefore modelled as an input filter.
struct Truth {  // plant actually simulated
    float pitch_delay, pitch_input, pitch_tau, pitch_pull, pitch_push, pitch_gravity, pitch_curve;
    float roll_delay, roll_input, roll_accel, roll_curve, roll_max, roll_tau;
    float push_gravity=0;  // push authority added per unit of upright (negative-G limit)
    float pitch_release=0.10f;  // input smoothing toward or past centre (logged stops)
};
// Nominal = calibration fit (delay, input smoothing, response); sluggish is ~25% slower
// everywhere with weaker authority, to check robustness to model error.
// Strong jets as identified in game (pitch tau ~0.27 s, roll ~286 deg/s^2, tau ~0.6 s).
constexpr Truth nominal{0.24f,0.36f,0.27f,61,26,0.3f,1.1f, 0.16f,0.06f,286,1.3f,170,0.6f};
constexpr Truth sluggish{0.30f,0.45f,0.50f,56,22,3,1.1f, 0.20f,0.08f,190,1.4f,140,0.7f};
// A weaker jet as identified offline from game logs by the model bank (pitch x0.65,
// slower response; roll ~121 deg/s^2 at full stick with more delay).
constexpr Truth weak{0.24f,0.36f,0.45f,40,17,0.3f,1.1f, 0.22f,0.06f,121,1.3f,97,0.8f};
// The nominal jet after bleeding speed in combat: full pull held a median 37-40 deg/s
// in game logs vs 61 at cruise, with the same timing; full push ~27 deg/s upright,
// 20 on the side and far less inverted (logged).
constexpr Truth combat{0.24f,0.36f,0.27f,40,17,0.3f,1.1f, 0.16f,0.06f,200,1.3f,150,0.6f, 8};

V turn(V v,V toward,float a) { return v*std::cos(a)+toward*std::sin(a); }

// dips: pitch rate falling away mid-maneuver and picking up again (reported as a
// stuttering pull).
// inverted: seconds with the canopy more than 100 deg from upright.
// half: time until the angle first falls to half of where it started ("roughly there").
// track: mean angle to a moving target after the first 3 s (chases).
struct Result { float to2=-1, settle=-1, done=-1, overshoot=0, inverted=0, half=-1, track=-1; int reversals=0, chatter=0, dips=0; };

bool verbose=false;  // per-frame dump for one case (trace=<case index>)
float init_q=0, init_p=0;  // initial pitch/roll rates (deg/s), for traces from a logged state
float rate_noise=2.0f;  // deg/s, white noise on the measured rates; reproduces logged chatter

// A target that keeps moving, as an enemy in a turning fight does: bearing changes at
// yaw_rate from yaw0, elevation weaves around elev by amp with the given period.
struct Chase { const char* name; float yaw0, yaw_rate, elev, amp, period, yaw_amp=0; };
V chase_direction(const Chase& c,float t) {
    const float phase=2*3.14159265f*t/c.period;
    return basis(c.elev+c.amp*std::sin(phase),c.yaw0+c.yaw_rate*t+c.yaw_amp*std::sin(phase+1.0f),0).f;
}
// With switch_at>=0 the target moves to next at that time (chained maneuvers) and the
// metrics restart there, times relative to the switch.
Result fly(const Truth& truth,float start_roll,V target,const Tuning& tuning,float seconds=10,
          V next=V{},float switch_at=-1,const Chase* chase=nullptr) {
    const float dt=1.0f/75;
    Basis b=basis(0,0,start_roll);
    float q=init_q,p=init_p,r=0, fq=init_q,fp=init_p,fr=0, sq=0,sr=0, last_q=0,last_r=0;
    std::deque<float> pq,pr;  // command delay lines
    std::mt19937 rng(1234);
    std::normal_distribution<float> jitter(0.0f,std::max(rate_noise,1e-6f));
    LogicState state;
    LogicOutput out{};
    Result res;
    float min_angle=180, held=0, held_level=0;
    int last_sign=0;
    float peak=0, trough=0, start=0, start_angle=-1, track_sum=0, track_time=0;
    bool dropping=false;
    const int nq=int(truth.pitch_delay/dt+0.5f), nr=int(truth.roll_delay/dt+0.5f);
    for(int k=0;k<int(seconds/dt);++k) {
        if(switch_at>=0 && k==int(switch_at/dt)) {
            target=unit(next); start=k*dt;
            const int dips=res.dips; res=Result{}; res.dips=dips; start_angle=-1;
            min_angle=180; held=held_level=0; last_sign=0;
        }
        if(chase) target=unit(chase_direction(*chase,k*dt));
        const V f=b.f;
        const float pitch=std::asin(std::clamp(f.z,-1.0f,1.0f))/rad, yaw=std::atan2(f.y,f.x)/rad;
        const V right0{-std::sin(yaw*rad),std::cos(yaw*rad),0}, up0=cross(f,right0);
        const float roll=std::atan2(dot(b.u,right0),dot(b.u,up0))/rad;
        const LogicInput in{pitch,yaw,roll,target.x,target.y,target.z,fq+jitter(rng),fr,fp+jitter(rng),dt,0};
        maneuver_debug=verbose && k%4==0;
        logic_step(tuning,state,in,out);
        if(verbose && k%4==0) std::printf("t=%.2f %s\n",k*dt,out.trace);
        pq.push_back(out.pitch); pr.push_back(out.roll);
        const float uq=pq.size()>size_t(nq)?pq.front():0, ur=pr.size()>size_t(nr)?pr.front():0;
        if(pq.size()>size_t(nq)) pq.pop_front();
        if(pr.size()>size_t(nr)) pr.pop_front();
        // Chatter: stick reversals between beyond +0.3 and beyond -0.3, after the first second.
        if(k*dt>1.0f) {
            if(std::abs(out.pitch)>0.3f) { const float sgn=out.pitch>0?1.0f:-1.0f; if(last_q*sgn<0) ++res.chatter; last_q=sgn; }
            if(std::abs(out.roll)>0.3f) { const float sgn=out.roll>0?1.0f:-1.0f; if(last_r*sgn<0) ++res.chatter; last_r=sgn; }
        }
        sq=input_smooth(sq,uq,dt,truth.pitch_input,truth.pitch_release);
        sr+=(ur-sr)*(1-std::exp(-dt/truth.roll_input));
        const float aq=std::pow(std::min(std::abs(sq),1.0f),truth.pitch_curve);
        const float push=std::max(2.0f,truth.pitch_push+truth.push_gravity*b.u.z);
        const float steady=(sq>=0?truth.pitch_pull*aq:-push*aq)-truth.pitch_gravity*b.u.z;
        q+=dt*(steady-q)/truth.pitch_tau;
        p=std::clamp(p+dt*(std::copysign(truth.roll_accel*std::pow(std::min(std::abs(sr),1.0f),truth.roll_curve),sr)
                           -p/truth.roll_tau),-truth.roll_max,truth.roll_max);
        r+=dt*(8*out.yaw-r)/0.5f;
        // Body rotations: pitch moves f toward u, roll moves u toward r, yaw moves f toward r.
        V nf=turn(b.f,b.u,q*dt*rad), nu=turn(b.u,b.f*-1,q*dt*rad);
        b.f=nf; b.u=nu;
        V nu2=turn(b.u,b.r,p*dt*rad), nr2=turn(b.r,b.u*-1,p*dt*rad);
        b.u=nu2; b.r=nr2;
        V nf2=turn(b.f,b.r,r*dt*rad), nr3=turn(b.r,b.f*-1,r*dt*rad);
        b.f=unit(nf2); b.r=unit(nr3-b.f*dot(nr3,b.f)); b.u=cross(b.f,b.r);
        const float a=1-std::exp(-12*dt);
        fq+=(q-fq)*a; fp+=(p-fp)*a; fr+=(r-fr)*a;
        const float angle=std::acos(std::clamp(dot(b.f,target),-1.0f,1.0f))/rad;
        const float t=k*dt-start;
        if(angle>6) {
            const float rate=std::abs(q);
            if(!dropping) { peak=std::max(peak,rate); if(peak>15 && peak-rate>8) { dropping=true; trough=rate; } }
            else { trough=std::min(trough,rate); if(rate-trough>6) { ++res.dips; dropping=false; peak=rate; } }
        } else { peak=0; dropping=false; }
        if(chase && t>3) { track_sum+=angle*dt; track_time+=dt; res.track=track_sum/track_time; }
        if(start_angle<0) start_angle=angle;
        if(res.half<0 && angle<0.5f*start_angle) res.half=t;
        if(res.to2<0 && angle<2) res.to2=t;
        if(res.to2>=0) {
            min_angle=std::min(min_angle,angle);
            res.overshoot=std::max(res.overshoot,angle-min_angle);
        }
        held=angle<0.7f?held+dt:0;
        if(res.settle<0 && held>=0.5f) res.settle=t-0.5f;
        // Done: on target and wings level together (bank measured against the horizon).
        const float bank=std::abs(std::atan2(b.r.z,b.u.z))/rad;
        if(bank>100) res.inverted+=dt;
        held_level=(angle<1.0f && bank<5.0f)?held_level+dt:0;
        if(res.done<0 && held_level>=0.5f) res.done=t-0.5f;
        if(std::abs(p)>15) {
            const int sign=p>0?1:-1;
            if(last_sign && sign!=last_sign) ++res.reversals;
            last_sign=sign;
        }
    }
    return res;
}

V direction(float pitch,float yaw) { return basis(pitch,yaw,0).f; }

struct Case { const char* name; float roll; V target; };
const Case cases[]={
        {"right 3",0,direction(0,3)}, {"right 10",0,direction(0,10)},
        {"right 30",0,direction(0,30)}, {"right 90",0,direction(0,90)},
        {"up 20",0,direction(20,0)}, {"down 15",0,direction(-15,0)},
        {"down 90",0,V{0.0001f,0,-1}}, {"down-left 30/40",0,direction(-30,-40)},
        {"right 150",0,direction(0,150)}, {"inverted, up 10",180,direction(10,0)},
        {"inverted, up 60",180,direction(60,0)},
        // Target near the wing line while inverted: push and pull need similar rolls
        // (logged flip-flopping between them for 4 s, then a ground impact).
        {"inverted, right 15",180,direction(0,15)}, {"inverted, down-right 12/12",180,direction(-12,12)},
        {"bank 100, up-left 10/15",100,direction(10,-15)},
        // Turns toward targets a little or well below (upright preference vs split-S).
        {"right 60, 12 below",0,direction(-12,60)}, {"left 100, 25 below",0,direction(-25,-100)},
        {"right 40, 50 below",0,direction(-50,40)},
};
// The next maneuver starts mid-turn or right after arrival (reported: slow to recover
// and slow to take up the next maneuver from the current state).
struct Chain { const char* name; V first; float at; V next; };
const Chain chains[]={
        {"up 60 >0.8s> right 40",direction(60,0),0.8f,direction(30,40)},
        {"right 90 >2.2s> down-left",direction(0,90),2.2f,direction(-30,50)},
        {"down 50 >1.2s> up 20",direction(-50,0),1.2f,direction(20,10)},
        {"right 30 >1.0s> left 30",direction(0,30),1.0f,direction(0,-20)},
};
// Turning fights: the target keeps moving (logged: after a reversal the logic flipped
// between leveling and lift alignment at 10-25 deg off and rolled through inverted).
const Chase chases[]={
        {"chase level turn 20/s",30,20,0,0,6}, {"chase climbing weave",20,15,10,15,6},
        {"chase descending left",-30,-25,-15,8,5}, {"chase hard turn 35/s",40,35,0,0,6},
        // A tail chase: the target jinks a few degrees either side of the nose (logged:
        // rolled left and right at full stick).
        {"chase tail jinking",3,8,0,3,3,6},
};
// Lower is better. Responsiveness first (time to reach 2 deg, then to settle), with
// penalties only for clear overshoot (>1.5 deg) and repeated roll reversals. The
// other plants count half: they guard robustness, not the tuning.
float cost(const Result& r) {
    return (r.to2<0?12.0f:r.to2)+0.5f*(r.settle<0?12.0f:r.settle)+
           3*std::max(0.0f,r.overshoot-1.5f)+0.5f*std::max(0,r.reversals-1)+0.3f*r.chatter+0.5f*r.dips;
}
float score(const Tuning& tuning) {
    float total=0;
    for(const Truth* truth:{&nominal,&sluggish,&weak,&combat}) {
        const float weight=truth==&nominal?1.0f:0.5f;
        for(const Case& c:cases) total+=weight*cost(fly(*truth,c.roll,unit(c.target),tuning));
        for(const Chain& c:chains) total+=weight*cost(fly(*truth,0,unit(c.first),tuning,10,c.next,c.at));
        for(const Chase& c:chases) {
            const Result r=fly(*truth,0,chase_direction(c,0),tuning,10,V{},-1,&c);
            total+=weight*(0.5f*r.track+0.5f*std::max(0,r.reversals-2)+0.3f*r.chatter+0.5f*r.inverted);
        }
    }
    return total;
}
void sweep() {
    float best=1e9;
    Tuning chosen;
    for(float kd:{0.25f}) for(float kv:{0.8f,1.5f,3.0f,5.0f})
    for(float pb:{0.45f}) for(float rb:{0.3f,0.45f,0.6f})
    for(float pg:{2.0f}) for(float rg:{2.0f,3.0f,4.0f})
    for(float fs:{20.0f}) for(float nb:{3.0f}) {
        Tuning t;
        t.pitch_kd=kd; t.roll_kv=kv;
        t.pitch_gain=pg; t.roll_gain=rg; t.pitch_brake=pb; t.roll_brake=rb;
        t.maneuver.far_start=fs; (void)nb;
        const float s=score(t);
        if(s<best) {
            best=s; chosen=t;
            std::printf("score %.1f: pitch_kd=%.3f roll_kv=%.1f pitch_brake=%.2f roll_brake=%.2f "
                        "pitch_gain=%.1f roll_gain=%.1f far_start=%.0f near_bank_gain=%.0f\n",
                        s,kd,kv,pb,rb,pg,rg,fs,nb);
        }
    }
    std::printf("default score %.1f\n",score(Tuning{}));
}
}

int main(int argc,char** argv) {
    if(argc>1 && std::string(argv[1])=="sweep") { sweep(); return 0; }
    Tuning tuning;
    int trace_case=-1, truth_pick=0;
    // Optional overrides for trying settings: key=value ... (controller and maneuver keys).
    for(int i=1;i<argc;++i) {
        const std::string arg=argv[i];
        const auto eq=arg.find('=');
        if(eq==std::string::npos) continue;
        const std::string key=arg.substr(0,eq);
        const float value=std::stof(arg.substr(eq+1));
        if(key=="trace") trace_case=int(value);
        else if(key=="truth") truth_pick=int(value);
        else if(key=="ident_memory") tuning.ident_memory=value;
        else if(key=="model_gravity") tuning.plant.pitch_gravity=value;
        else if(key=="noise") rate_noise=value;
        else if(key=="pitch_trim") tuning.pitch_trim=value;
        else if(key=="pitch_trim_limit") tuning.pitch_trim_limit=value;
        else if(key=="near_end") tuning.maneuver.near_end=value;
        else if(key=="far_start") tuning.maneuver.far_start=value;
        else if(key=="level_per_deg") tuning.maneuver.level_per_deg=value;
        else if(key=="rollout_rate") tuning.maneuver.rollout_rate=value;
        else if(key=="rollout_lag") tuning.maneuver.rollout_lag=value;
        else if(key=="pursuit_ahead") tuning.maneuver.pursuit_ahead=value;
        else if(key=="push_enter") tuning.maneuver.push_enter=value;
        else if(key=="push_exit") tuning.maneuver.push_exit=value;
        else if(key=="push_roll_enter") tuning.maneuver.push_roll_enter=value;
        else if(key=="push_roll_exit") tuning.maneuver.push_roll_exit=value;
        else if(key=="push_below") tuning.maneuver.push_below=value;
        else if(key=="pursuit_speed") tuning.maneuver.pursuit_speed=value;
        else if(key=="level_inside") tuning.maneuver.level_inside=value;
        else if(key=="slice_ease") tuning.maneuver.slice_ease=value;
        else if(key=="slice_hold") tuning.maneuver.slice_hold=value;
        else if(key=="chase_slice") tuning.maneuver.chase_slice=value;
        else if(key=="push_below_exit") tuning.maneuver.push_below_exit=value;
        else if(key=="choice_margin") tuning.maneuver.choice_margin=value;
        else if(key=="push_bias") tuning.maneuver.push_bias=value;
        else if(key=="tail_enter") tuning.maneuver.tail_enter=value;
        else if(key=="tail_exit") tuning.maneuver.tail_exit=value;
        else if(key=="tail_bank") tuning.maneuver.tail_bank=value;
        else if(key=="latch_limit") tuning.maneuver.latch_limit=value;
        else if(key=="overlap_max") tuning.maneuver.overlap_max=value;
        else if(key=="upright") tuning.maneuver.upright=value;
        else if(key=="slice_bank") tuning.maneuver.slice_bank=value;
        else if(key=="slice_deep") tuning.maneuver.slice_deep=value;
        else if(key=="slice_from") tuning.maneuver.slice_from=value;
        else if(key=="invert_min_angle") tuning.maneuver.invert_min_angle=value;
        else if(key=="rate_observer") tuning.rate_observer=value;
        else if(key=="authority_memory") tuning.authority_memory=value;
        else if(key=="authority_filter") tuning.authority_filter=value;
        else if(key=="authority_trust") tuning.authority_trust=value;
        else if(key=="pitch_unload") tuning.pitch_unload=value;
        else if(key=="pitch_kd") tuning.pitch_kd=value;
        else if(key=="roll_kv") tuning.roll_kv=value;
        else if(key=="pitch_brake") tuning.pitch_brake=value;
        else if(key=="roll_brake") tuning.roll_brake=value;
        else if(key=="pitch_gain") tuning.pitch_gain=value;
        else if(key=="roll_gain") tuning.roll_gain=value;
        else if(key=="lead_gain") tuning.lead_gain=value;
        else if(key=="lead_filter") tuning.lead_filter=value;
        else if(key=="init_q") init_q=value;
        else if(key=="init_p") init_p=value;
        else if(key=="pitch_gain_near") tuning.pitch_gain_near=value;
        else if(key=="pitch_trim_keep") tuning.pitch_trim_keep=value;
        else if(key=="pitch_slew") tuning.pitch_slew=value;
        else if(key=="roll_slew") tuning.roll_slew=value;
    }
    if(trace_case>=0) {
        verbose=true;
        const Truth& truth=truth_pick==3?combat:truth_pick==2?weak:truth_pick==1?sluggish:nominal;
        const int n=int(sizeof(cases)/sizeof(cases[0]));
        if(trace_case<n) fly(truth,cases[trace_case].roll,unit(cases[trace_case].target),tuning,8);
        else if(trace_case<n+int(sizeof(chains)/sizeof(chains[0]))) { const Chain& c=chains[trace_case-n]; fly(truth,0,unit(c.first),tuning,8,c.next,c.at); }
        else { const Chase& c=chases[trace_case-n-int(sizeof(chains)/sizeof(chains[0]))]; fly(truth,0,chase_direction(c,0),tuning,10,V{},-1,&c); }
        return 0;
    }
    std::printf("score %.1f\n",score(tuning));
    bool ok=true;
    for(const Truth* truth:{&nominal,&sluggish,&weak,&combat}) {
        std::printf("%s plant\n  %-26s %8s %8s %8s %9s %9s %8s %5s %5s %5s\n",truth==&nominal?"nominal":truth==&sluggish?"sluggish":truth==&weak?"weak":"combat",
                    "maneuver","to 2deg","settled","level","overshoot","roll rev","chatter","dips","inv","half");
        auto row=[&](const char* name,const Result& r) {
            std::printf("  %-26s %7.2fs %7.2fs %7.2fs %8.1fdeg %9d %8d %5d %4.1fs %4.2fs\n",name,r.to2,r.settle,r.done,r.overshoot,r.reversals,r.chatter,r.dips,r.inverted,r.half);
            if(truth!=&sluggish && (r.settle<0 || r.overshoot>3.0f)) ok=false;
        };
        for(const Case& c:cases) row(c.name,fly(*truth,c.roll,unit(c.target),tuning));
        for(const Chain& c:chains) row(c.name,fly(*truth,0,unit(c.first),tuning,10,c.next,c.at));
        for(const Chase& c:chases) {
            const Result r=fly(*truth,0,chase_direction(c,0),tuning,10,V{},-1,&c);
            std::printf("  %-26s mean %5.1fdeg after 3s, roll reversals %d, chatter %d, inverted %.1fs\n",c.name,r.track,r.reversals,r.chatter,r.inverted);
        }
    }
    std::printf(ok?"Simulation checks passed.\n":"Simulation checks FAILED (nominal and weak plants must settle with <3deg overshoot).\n");
    return ok?0:1;
}
