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
    assert(!g.tail && close_to(std::abs(g.roll_error),180,12) && g.pitch_error==0);

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

    // Inside the aim circle the wings stay level; pitch and rudder trim.
    g=guide(level,basis(0.4f,0.8f,0).f);
    assert(g.turn_weight==0 && std::abs(g.roll_error)<0.5f);
    assert(close_to(g.yaw_error,0.8f,0.05f) && close_to(g.pitch_error,0.4f,0.05f));
    // A few degrees sideways: bank toward it (limited by the distance still to go) and
    // pull, rather than leave it to the weak rudder (logged: 5 deg took 2-3 s).
    g=guide(level,basis(0,5,0).f);
    assert(g.roll_error>60 && g.roll_error<=ManeuverTuning{}.level_per_deg*4+0.5f);
    g=guide(level,basis(0,2,0).f);
    assert(g.roll_error>5 && g.roll_error<=ManeuverTuning{}.level_per_deg+0.5f);
    // Inverted near the target: roll upright.
    g=guide(basis(0,0,180),basis(0,1,0).f);
    assert(std::abs(g.roll_error)>165);

    // Logged case: 65.8 deg bank, target 3.3 deg below and 1.5 deg left of the
    // nose: in world terms it is 3.6 deg left on the horizon. The floor already faces
    // it: push with a small roll, not a half roll to put the canopy on it.
    {
        const Basis banked=basis(0,0,65.8f);
        g=guide(banked,unit(banked.f+banked.u*std::tan(-3.3f*rad)+banked.r*std::tan(-1.5f*rad)));
        assert(g.turn_weight<0.05f && g.pushing && std::abs(g.roll_error)<30 && g.pitch_error<0);
    }
    // Moderate error below: push straight down without rolling.
    g=guide(level,basis(-10,0,0).f);
    assert(g.pushing && close_to(g.roll_error,0) && close_to(g.pitch_error,-10,0.2f));
    // Straight down from level flight: push, no half roll (push ~ pull, calibrated).
    g=guide(level,basis(-90,0,0).f);
    assert(g.pushing && close_to(g.roll_error,0) && close_to(g.pitch_error,-90,0.5f));
    // Inverted, target above in world terms: pushing would leave the aircraft inverted
    // and still owe the half roll back upright, so roll upright and pull (logged: an
    // inverted push toward a target 3-17 deg above held for 18 s at 2-10 deg/s).
    g=guide(basis(0,0,180),basis(10,0,0).f);
    assert(!g.pushing && close_to(std::abs(g.roll_error),180,5));
    g=guide(basis(0,0,180),basis(60,0,0).f);
    assert(!g.pushing && close_to(std::abs(g.roll_error),180,5));
    // Inverted, target 110 deg above: the half roll and pull is clearly quicker.
    g=guide(basis(0,0,180),basis(110,0,0).f);
    assert(!g.pushing && std::abs(g.roll_error)>170);
    // Top of a loop: inverted, target over the canopy. Keep pulling, no flip.
    {
        // The bank already held is eased off while pulling, not cut to the limit.
        Maneuver m;
        const Basis top=basis(0,0,180);
        g=m.step(top,unit(top.f+top.u*std::tan(30*rad)));
        assert(!g.pushing && std::abs(g.roll_error)<60 && g.pitch_error>5);
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

    // Once chosen, a small maneuver keeps pull (or push) unless the other is clearly
    // quicker: inverted with the target near the wing line, re-choosing every frame
    // flip-flopped between push and roll-and-pull for 4 s (logged, then ground impact).
    {
        const Basis inverted=basis(0,0,180);
        Maneuver m;
        int flips=0;
        // The target swings toward and away from the wing line on the floor side (world
        // up when inverted), across the 70/85 deg push limits.
        bool last=m.step(inverted,basis(8.5f,12,0).f).pushing;
        for(int i=0;i<=60;++i) {
            const bool now=m.step(inverted,basis(5+3.5f*std::sin(i*0.5f),12,0).f).pushing;
            flips+=now!=last; last=now;
        }
        assert(flips<=1);
    }

    // After a big turn, an offset left beside the pitch plane is not held: the wings
    // level (logged: held inverted 1.5 deg off target for 6 s, nothing closing it).
    {
        const Basis banked=basis(0,0,134);
        Maneuver m;
        m.step(banked,unit(banked.f+banked.u*std::tan(40*rad)));
        g=m.step(banked,unit(banked.f+banked.r*std::tan(1.5f*rad)));
        assert(std::abs(g.roll_error)>60);
    }

    // Rolling fast toward a lift 90 deg off a target 12 deg away: only a little pull yet.
    // Crediting all of the roll still to come pulled with the lift beside the target and
    // swung it around the nose as fast as the roll chased it (logged corkscrew).
    {
        const Basis level=basis(0,0,0);
        Maneuver m;
        g=m.step(level,unit(level.f+level.r*std::tan(12*rad)),Plant{},0,130);
        assert(std::abs(g.roll_error)>40 && g.pitch_error>=0 && g.pitch_error<6);
    }
    // Banked with the target straight below and 6 deg away: no stall at the bank limit
    // with only the rudder working (sim: stuck 6 deg out); roll toward level and push.
    {
        const Basis banked=basis(0,0,90);
        g=guide(banked,unit(banked.f+banked.r*std::tan(6*rad)));
        assert(g.pushing && g.roll_error<-45);
    }
    // A slice under way is eased off as the nose closes in, not cut back level for a
    // push halfway through (sim: stopped ~14 deg out for a second).
    {
        const Basis slicing=basis(-20,0,-130);
        const float off=25*rad, around=-20*rad;
        g=guide(slicing,unit(slicing.f*std::cos(off)+(slicing.u*std::cos(around)+slicing.r*std::sin(around))*std::sin(off)));
        assert(!g.pushing && std::abs(g.roll_error)<45);
    }

    // Down and to the left (40 deg off, 45 deg around from straight below): roll left,
    // pull and rudder, not roll right and push, nor roll belly-up (both reported).
    {
        const Basis level=basis(0,0,0);
        Maneuver m;
        const float off=40*rad, around=45*rad;
        g=m.step(level,unit(level.f*std::cos(off)+(level.u*(-std::cos(around))+level.r*(-std::sin(around)))*std::sin(off)));
        // A nose-low slice at most (bank up to slice_deep), never rolled belly-up.
        assert(!g.pushing && g.roll_error<-80 && g.roll_error>-140);
        // Once banked left, it keeps rolling left into a moderate slice (no reversal, no
        // push) with the rudder working alongside.
        const Basis banked=basis(0,0,-70);
        Maneuver n;
        g=n.step(banked,basis(-30,-30,0).f);
        assert(!g.pushing && g.roll_error>-75 && g.roll_error<10 && g.yaw_weight>0.5f);
    }

    // After a big turn that ended near inverted, the target moving a few degrees on is a
    // small correction: roll toward upright with pitch working at once, not a half roll
    // to align the lift with the pitch held (logged: stayed inverted 2 s, hit the ground).
    {
        const Basis inverted=basis(0,0,-150);
        Maneuver m;
        m.step(inverted,unit(inverted.f+inverted.u*std::tan(40*rad)));
        for(int i=0;i<3;++i) m.step(inverted,unit(inverted.f+inverted.u*std::tan(-1.5f*rad)));
        g=m.step(inverted,unit(inverted.f+inverted.u*std::tan(-6*rad))); // target moves up (world)
        assert(g.turn_weight<0.3f && std::abs(g.pitch_error)>2 && std::abs(std::abs(g.roll_error)-180)>20);
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
        assert(out.roll>0.5f && std::strstr(out.trace,"abi=5"));
        // Flight path: level flight along the nose is 0 deg AoA at 1 g; a nose 5 deg above a
        // level flight path is +5 deg AoA; pulling 3 g extra along the canopy reads 4 g.
        {
            LogicInput fp{};
            fp.vel_x=250;
            FlightPath path=flight_path(basis(0,0,0),fp);
            assert(close_to(path.speed,250) && close_to(path.aoa,0) && close_to(path.nz,1,1e-3f));
            path=flight_path(basis(5,0,0),fp);
            assert(close_to(path.aoa,5,0.01f));
            fp.acc_z=3*9.81f;
            assert(close_to(flight_path(basis(0,0,0),fp).nz,4,1e-3f));
        }
        // Throttle and brake held together (or the single high-G button) is a high-G turn.
        LogicInput high{0,0,0, 0,1,0, 0,0,0, 1.0f/60, 0, 0, 1, 1};
        logic_step(tuning,state,high,out);
        assert(std::strstr(out.event,"high-G turn on"));
        high.brake=0;
        logic_step(tuning,state,high,out);
        assert(std::strstr(out.event,"high-G turn off"));
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
