// Closed-loop simulation of the real flight logic against the measured plant model.
// Reports time to target, overshoot and roll reversals for standard maneuvers, on the
// nominal model and on a deliberately slower/weaker one (model error robustness).
#include "../src/flight_logic.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>

using namespace flight;

namespace {
// The game smooths stick input before the aircraft responds; in game, short full-stick
// pulses were averaged away (logged ~2.5 Hz full-stick chatter with ~1 deg/s response).
// Part of the measured onset delay is therefore modelled as an input filter.
struct Truth {  // plant actually simulated
    float pitch_delay, pitch_input, pitch_tau, pitch_pull, pitch_push, pitch_gravity, pitch_curve;
    float roll_delay, roll_input, roll_accel, roll_curve, roll_max, roll_tau;
};
// Nominal = calibration fit (delay, input smoothing, response); sluggish is ~25% slower
// everywhere with weaker authority, to check robustness to model error.
constexpr Truth nominal{0.24f,0.36f,0.40f,61,26,9,1.1f, 0.16f,0.06f,220,1.3f,150,0.8f};
constexpr Truth sluggish{0.30f,0.45f,0.50f,56,22,10,1.1f, 0.20f,0.08f,190,1.4f,140,0.7f};

V turn(V v,V toward,float a) { return v*std::cos(a)+toward*std::sin(a); }

struct Result { float to2=-1, settle=-1, done=-1, overshoot=0; int reversals=0, chatter=0; };

bool verbose=false;  // per-frame dump for one case (trace=<case index>)

Result fly(const Truth& truth,float start_roll,V target,const Tuning& tuning,float seconds=10) {
    const float dt=1.0f/75;
    Basis b=basis(0,0,start_roll);
    float q=0,p=0,r=0, fq=0,fp=0,fr=0, sq=0,sr=0, last_q=0,last_r=0;
    std::deque<float> pq,pr;  // command delay lines
    LogicState state;
    LogicOutput out{};
    Result res;
    float min_angle=180, held=0, held_level=0;
    int last_sign=0;
    const int nq=int(truth.pitch_delay/dt+0.5f), nr=int(truth.roll_delay/dt+0.5f);
    for(int k=0;k<int(seconds/dt);++k) {
        const V f=b.f;
        const float pitch=std::asin(std::clamp(f.z,-1.0f,1.0f))/rad, yaw=std::atan2(f.y,f.x)/rad;
        const V right0{-std::sin(yaw*rad),std::cos(yaw*rad),0}, up0=cross(f,right0);
        const float roll=std::atan2(dot(b.u,right0),dot(b.u,up0))/rad;
        const LogicInput in{pitch,yaw,roll,target.x,target.y,target.z,fq,fr,fp,dt,0};
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
        sq+=(uq-sq)*(1-std::exp(-dt/truth.pitch_input));
        sr+=(ur-sr)*(1-std::exp(-dt/truth.roll_input));
        const float aq=std::pow(std::min(std::abs(sq),1.0f),truth.pitch_curve);
        const float steady=(sq>=0?truth.pitch_pull*aq:-truth.pitch_push*aq)-truth.pitch_gravity*b.u.z;
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
        const float t=k*dt;
        if(res.to2<0 && angle<2) res.to2=t;
        if(res.to2>=0) {
            min_angle=std::min(min_angle,angle);
            res.overshoot=std::max(res.overshoot,angle-min_angle);
        }
        held=angle<0.7f?held+dt:0;
        if(res.settle<0 && held>=0.5f) res.settle=t-0.5f;
        // Done: on target and wings level together (bank measured against the horizon).
        const float bank=std::abs(std::atan2(b.r.z,b.u.z))/rad;
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
};
// Lower is better. Responsiveness first (time to reach 2 deg, then to settle), with
// penalties only for clear overshoot (>1.5 deg) and repeated roll reversals. The
// deliberately sluggish plant counts half: it guards robustness, not the tuning.
float score(const Tuning& tuning) {
    float total=0;
    for(const Truth* truth:{&nominal,&sluggish})
        for(const Case& c:cases) {
            const Result r=fly(*truth,c.roll,unit(c.target),tuning);
            const float s=(r.to2<0?12.0f:r.to2)+0.5f*(r.settle<0?12.0f:r.settle)+
                          3*std::max(0.0f,r.overshoot-1.5f)+0.5f*std::max(0,r.reversals-1)+0.3f*r.chatter;
            total+=truth==&nominal?s:0.5f*s;
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
        t.maneuver.far_start=fs; t.maneuver.near_bank_gain=nb;
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
    int trace_case=-1;
    // Optional overrides for trying settings: key=value ... (controller and maneuver keys).
    for(int i=1;i<argc;++i) {
        const std::string arg=argv[i];
        const auto eq=arg.find('=');
        if(eq==std::string::npos) continue;
        const std::string key=arg.substr(0,eq);
        const float value=std::stof(arg.substr(eq+1));
        if(key=="trace") trace_case=int(value);
        else if(key=="pitch_kd") tuning.pitch_kd=value;
        else if(key=="roll_kv") tuning.roll_kv=value;
        else if(key=="pitch_brake") tuning.pitch_brake=value;
        else if(key=="roll_brake") tuning.roll_brake=value;
        else if(key=="pitch_gain") tuning.pitch_gain=value;
        else if(key=="roll_gain") tuning.roll_gain=value;
        else if(key=="near_bank_gain") tuning.maneuver.near_bank_gain=value;
        else if(key=="far_start") tuning.maneuver.far_start=value;
        else if(key=="rollout_lead") tuning.maneuver.rollout_lead=value;
        else if(key=="pitch_slew") tuning.pitch_slew=value;
        else if(key=="roll_slew") tuning.roll_slew=value;
    }
    if(trace_case>=0) {
        verbose=true;
        const Case& c=cases[trace_case];
        fly(nominal,c.roll,unit(c.target),tuning,6);
        return 0;
    }
    std::printf("score %.1f\n",score(tuning));
    bool ok=true;
    for(const Truth* truth:{&nominal,&sluggish}) {
        std::printf("%s plant\n  %-18s %8s %8s %8s %9s %9s %8s\n",truth==&nominal?"nominal":"sluggish",
                    "maneuver","to 2deg","settled","level","overshoot","roll rev","chatter");
        for(const Case& c:cases) {
            const Result r=fly(*truth,c.roll,unit(c.target),tuning);
            std::printf("  %-18s %7.2fs %7.2fs %7.2fs %8.1fdeg %9d %8d\n",c.name,r.to2,r.settle,r.done,r.overshoot,r.reversals,r.chatter);
            if(truth==&nominal && (r.settle<0 || r.overshoot>3.0f)) ok=false;
        }
    }
    std::printf(ok?"Simulation checks passed.\n":"Simulation checks FAILED (nominal plant must settle with <3deg overshoot).\n");
    return ok?0:1;
}
