#include "../src/flight_math.h"
#include "../src/free_look.h"
#include "../src/flight_logic.h"
#include <cassert>
#include <cmath>
#include <cstring>

namespace {
using namespace flight;
bool close_to(float a,float b,float tolerance=0.1f) { return std::abs(a-b)<tolerance; }
Guidance guide(const Basis& body,V aim) { Maneuver m; return m.step(body,aim); }
}

int main() {
    CommandSlew command;
    for(int i=0;i<12;++i) {
        const float previous=command.value;
        const float next=command.step(1.0f,0.016f,4.0f);
        assert(next>=previous && next-previous<=0.0641f);
    }
    assert(command.value<1.0f);

    V current{1,0,0}, target{0,1,0};
    float previous_yaw=0;
    for(int i=0;i<30;++i) {
        current=smooth_direction(current,target,0.016f,0.045f);
        assert(std::abs(dot(current,current)-1.0f)<0.0001f);
        assert(yaw(current)>previous_yaw && yaw(current)<91.0f);
        previous_yaw=yaw(current);
    }
    assert(previous_yaw>89.0f);

    FreeLook look;
    V aim{1,0,0};
    const Basis view=basis(0,0,0);
    look.step(true,aim,view,8,0);
    look.step(true,aim,view,8,0);
    assert(std::abs(yaw(aim))<0.001f);
    look.step(false,aim,view,8,0);
    assert(std::abs(yaw(aim))<0.001f);

    const Basis level=basis(0,0,0);
    // Lift vector: a target 90 degrees right is reached by rolling, not by pulling yet.
    Guidance g=guide(level,{0,1,0});
    assert(close_to(g.angle,90) && g.turn_weight==1 && close_to(g.roll_error,90));
    assert(close_to(g.pitch_error,0) && g.yaw_error==0);
    // Once the target is over the canopy, pull the whole angle.
    g=guide(basis(0,0,90),{0,1,0});
    assert(close_to(g.roll_error,0) && close_to(g.pitch_error,90));

    // Behind and clearly below: roll inverted without pushing (Split-S emerges).
    g=guide(level,unit({-1,0,-0.8f}));
    assert(!g.tail && close_to(std::abs(g.roll_error),180) && g.pitch_error==0);

    // Directly behind: committed upward oblique reversal, right by default.
    g=guide(level,{-1,0,0});
    assert(g.tail && close_to(g.roll_error,ManeuverTuning{}.tail_bank) && g.pitch_error>0);
    // Nose vertical has no horizon: hold roll and pull straight through.
    g=guide(basis(90,0,0),{0,0,-1});
    assert(g.tail && g.roll_error==0 && close_to(g.pitch_error,180));

    // The side chosen on entry is kept while the target jitters behind the tail.
    {
        Maneuver m;
        g=m.step(level,unit({-1,-0.05f,0}));
        assert(g.tail && close_to(g.roll_error,-ManeuverTuning{}.tail_bank));
        g=m.step(level,unit({-1,0.05f,0}));
        assert(g.tail && close_to(g.roll_error,-ManeuverTuning{}.tail_bank));
        const float exit=150*rad;
        assert(m.step(level,{std::cos(exit),std::sin(exit),0}).tail);
        const float done=140*rad;
        assert(!m.step(level,{std::cos(done),std::sin(done),0}).tail);
    }

    // Fine tracking banks gently toward the horizontal offset; pitch and rudder trim.
    g=guide(level,basis(0.4f,0.8f,0).f);
    assert(g.turn_weight==0 && close_to(g.roll_error,0.8f*ManeuverTuning{}.near_bank_gain,0.2f));
    assert(close_to(g.yaw_error,0.8f,0.05f) && close_to(g.pitch_error,0.4f,0.05f));
    // Inverted near the target: roll upright.
    g=guide(basis(0,0,180),basis(0,1,0).f);
    assert(std::abs(g.roll_error)>165);

    // Logged case: 65.8 deg bank, target 3.3 deg below and 1.5 deg left of the
    // nose: in world terms it is 3.6 deg left on the horizon. Bank left and trim
    // down instead of rolling the canopy toward it.
    {
        const Basis banked=basis(0,0,65.8f);
        g=guide(banked,unit(banked.f+banked.u*std::tan(-3.3f*rad)+banked.r*std::tan(-1.5f*rad)));
        assert(g.turn_weight<0.05f && g.roll_error<-60 && g.roll_error>-120 && g.pitch_error<0);
    }
    // Moderate error below: push straight down without rolling.
    g=guide(level,basis(-10,0,0).f);
    assert(g.pushing && close_to(g.roll_error,0) && close_to(g.pitch_error,-10,0.2f));
    // Straight down from level flight: push, no half roll (push ~ pull, calibrated).
    g=guide(level,basis(-90,0,0).f);
    assert(g.pushing && close_to(g.roll_error,0) && close_to(g.pitch_error,-90,0.5f));
    // Inverted, target 10 deg above in world terms: pushing (weak inverted, ~17 deg/s)
    // still beats a 180 deg roll then pull on the measured model.
    g=guide(basis(0,0,180),basis(10,0,0).f);
    assert(g.pushing && std::abs(g.roll_error)<5);
    // Inverted, target 60 deg above: now the roll upright and pull is quicker.
    g=guide(basis(0,0,180),basis(60,0,0).f);
    assert(!g.pushing && std::abs(g.roll_error)>170);
    // Top of a loop: inverted, target over the canopy. Keep pulling, no flip.
    {
        Maneuver m;
        const Basis top=basis(0,0,180);
        g=m.step(top,unit(top.f+top.u*std::tan(30*rad)));
        assert(!g.pushing && std::abs(g.roll_error)<1 && g.pitch_error>20);
        g=m.step(top,unit(top.f+top.u*std::tan(30*rad)+top.r*std::tan(8*rad)));
        assert(!g.pushing && std::abs(g.roll_error)<30);
    }
    // A lateral target crossing the wing plane keeps the pull choice (no flip),
    // banking toward it on both sides of the plane.
    {
        Maneuver m;
        g=m.step(level,basis(0.3f,30,0).f);
        assert(!g.pushing && g.roll_error>80);
        g=m.step(level,basis(-0.3f,30,0).f);
        assert(!g.pushing && g.roll_error>80);
        Maneuver close_in;
        const float up=close_in.step(level,basis(0.3f,8,0).f).roll_error;
        const float down=close_in.step(level,basis(-0.3f,8,0).f).roll_error;
        assert(up>15 && down>15 && std::abs(up-down)<10);
    }

    // Axis linearization pre-inverts the game's power curve.
    assert(close_to(std::pow(linearize_axis(0.5f,3),3.0f),0.5f,0.001f));
    assert(close_to(linearize_axis(-0.125f,3),-0.5f,0.001f));
    assert(linearize_axis(0.4f,1)==0.4f && linearize_axis(2.0f,3)==1.0f && linearize_axis(0,3)==0);

    // A target beside the aircraft is never reached by pushing, even right after
    // a push toward a target below (logged: rolled away 90 deg, then pushed).
    {
        Maneuver m;
        assert(m.step(level,basis(-10,0,0).f).pushing);
        g=m.step(level,basis(0,-10,0).f);
        assert(!g.pushing && g.roll_error<-20);
        g=m.step(level,basis(0,-40,0).f);
        assert(!g.pushing && g.roll_error<-60);
    }

    // Roll direction latch across +-180 degrees.
    int side=1;
    assert(close_to(latch_roll(-170,side,150),190));
    assert(close_to(latch_roll(160,side,150),160) && side==1);
    assert(close_to(latch_roll(-100,side,150),-100) && side==-1);
    assert(close_to(latch_roll(170,side,150),-190));

    // Full logic step: bounded outputs, and the hot-swap ABI reports its sizes.
    {
        LogicState state;
        Tuning tuning;
        LogicOutput out{};
        for(int i=0;i<120;++i) {
            const LogicInput in{0,0,0, 0,1,0, 0,0,0, 1.0f/60, 0};
            logic_step(tuning,state,in,out);
            assert(std::abs(out.pitch)<=1 && std::abs(out.roll)<=1 && std::abs(out.yaw)<=1);
        }
        assert(out.roll>0.5f && std::strstr(out.trace,"abi=1"));
        unsigned a=0,b=0,c=0;
        assert(logic_api::abi(&a,&b,&c)==logic_abi && a==sizeof(LogicInput) && c==sizeof(LogicState));
    }

    // Calibration waits for a settled aircraft, holds each raw test value, then
    // hands back to normal control; keyboard input aborts it.
    {
        LogicState state;
        Tuning tuning;
        LogicOutput out{};
        state.cal.step=0;
        const LogicInput calm{0,0,0, 1,0,0, 0,0,0, 1.0f/60, 0};
        int frames=0;
        while(!state.cal.holding && frames<600) { logic_step(tuning,state,calm,out); ++frames; }
        assert(frames>=48 && frames<60);
        logic_step(tuning,state,calm,out);
        assert(std::abs(out.roll-0.1f)<1e-6f && out.pitch==0 && std::strstr(out.trace,"cal=0"));
        frames=0;
        while(state.cal.step>=0 && frames<200000) { logic_step(tuning,state,calm,out); ++frames; }
        assert(state.cal.step<0 && std::strstr(out.event,"CAL done"));
        state.cal.step=3;
        logic_step(tuning,state,{0,0,0, 1,0,0, 0,0,0, 1.0f/60, 1},out);
        assert(state.cal.step<0 && std::strstr(out.event,"aborted"));
        // Plant inverses round-trip the measured curves.
        const Plant plant;
        for(float u:{-1.0f,-0.6f,-0.2f,0.3f,0.7f,1.0f}) {
            assert(close_to(pitch_input_for(plant,pitch_steady(plant,u,0)),u,1e-3f));
            assert(close_to(roll_input_for(plant,roll_accel_for(plant,u)),u,1e-3f));
        }
        // Braking distance inverse: a rate that stops in exactly the distance given.
        const float r=stoppable_first_order(15,30,0.8f,100);
        assert(close_to(0.8f*(r-30*std::log1p(r/30)),15,0.05f));
    }

    // Beyond the push range the controller pulls or waits; it never pushes, and stays finite.
    for(int p=-80;p<=80;p+=20) for(int y=-180;y<180;y+=15) for(int r=-180;r<180;r+=30) {
        const Basis body=basis(float(p),0,float(r));
        const V target=basis(0,float(y),0).f;
        g=guide(body,target);
        assert(std::isfinite(g.roll_error) && std::isfinite(g.pitch_error) && std::isfinite(g.yaw_error));
        assert(std::abs(g.roll_error)<=210);
        if(g.angle>ManeuverTuning{}.push_exit) assert(g.pitch_error>=-0.01f);
    }
}
