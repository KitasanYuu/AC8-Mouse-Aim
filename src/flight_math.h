#pragma once
#include <cmath>
#include <algorithm>
// MouseFlight-inspired local-space guidance; Unreal axes: X forward, Y right, Z up.
namespace flight {
constexpr float rad = 0.017453292519943295f;
struct V {
    float x{}, y{}, z{};
    V operator+(V b) const { return {x+b.x,y+b.y,z+b.z}; }
    V operator-(V b) const { return {x-b.x,y-b.y,z-b.z}; }
    V operator*(float k) const { return {x*k,y*k,z*k}; }
};
inline float dot(V a,V b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline V cross(V a,V b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline V unit(V a) { float n=std::sqrt(dot(a,a)); return n>1e-6f ? a*(1/n):V{1,0,0}; }
struct Basis { V f,r,u; };
inline Basis basis(float pitch,float yaw,float roll) {
    float p=pitch*rad,y=yaw*rad,r=roll*rad;
    V f{std::cos(p)*std::cos(y),std::cos(p)*std::sin(y),std::sin(p)};
    V right{-std::sin(y),std::cos(y),0};
    V up=cross(f,right);
    return {f,right*std::cos(r)-up*std::sin(r),up*std::cos(r)+right*std::sin(r)};
}
inline V rotate(V v,V axis,float angle) {
    float c=std::cos(angle),s=std::sin(angle);
    return unit(v*c+cross(axis,v)*s+axis*(dot(axis,v)*(1-c)));
}
inline float pitch(V v) { return std::asin(std::clamp(v.z,-1.0f,1.0f))/rad; }
inline float yaw(V v) { return std::atan2(v.y,v.x)/rad; }
inline float dead(float x,float d) { return std::copysign(std::max(0.0f,std::abs(x)-d),x); }
inline V smooth_direction(V current,V target,float dt,float seconds) {
    const float alpha=1.0f-std::exp(-std::clamp(dt,0.0f,0.1f)/seconds);
    // A recenter can reverse the direction; avoid normalizing a near-zero blend.
    if(dot(current,target)<-0.99f) return target;
    return unit(current*(1.0f-alpha)+target*alpha);
}
struct CommandSlew {
    float value=0;
    void reset() { value=0; }
    float step(float wanted,float dt,float units_per_second) {
        const float travel=units_per_second*std::clamp(dt,0.0f,0.1f);
        value+=std::clamp(wanted-value,-travel,travel);
        return value;
    }
};
// Conservative stopping envelope: distance = speed*delay + speed^2/(2*deceleration).
// Parameters describe an estimated input response, not changes to the flight model.
// delay is the measured command-to-rate latency, including pose and filter lag.
inline float arrival_rate(float error,float max_rate,float deceleration,float gain,float zone,
                          float delay=0.15f) {
    float distance=std::max(0.0f,std::abs(error)-zone);
    float delay_speed=deceleration*delay;
    float stoppable=std::sqrt(delay_speed*delay_speed+2*deceleration*distance)-delay_speed;
    return std::copysign(std::min({max_rate,gain*distance,stoppable}),error);
}
inline float arrival_command(float error,float actual,float max_rate,float deceleration,
                             float gain,float zone,float full_rate,float limit,float delay=0.15f) {
    float wanted=arrival_rate(error,max_rate,deceleration,gain,zone,delay);
    float command=wanted/full_rate+(wanted-actual)/full_rate;
    // When closing faster than the stopping envelope permits, actively counter-steer.
    if(std::abs(error)<=zone || actual*std::copysign(1.0f,error)>std::abs(wanted)+3.0f)
        command=(wanted-actual)/(full_rate*0.65f);
    return std::clamp(command,-limit,limit);
}
// AC8 applies a power curve (measured ~cubic) to injected axis values. Pre-invert
// it so a demand of 0.5 produces half the full-deflection response.
inline float linearize_axis(float demand,float exponent) {
    const float d=std::clamp(demand,-1.0f,1.0f);
    return exponent<=1.0f ? d : std::copysign(std::pow(std::abs(d),1.0f/exponent),d);
}
}
