// Build-specific offsets independently resolved from the read-only probe and
// native getter disassembly. Only the first 48 bytes of final camera POV change.
using CameraUpdate = void(__fastcall*)(void*,float);
CameraUpdate original_camera_update=nullptr;
struct CameraCommand { uintptr_t manager=0,pawn=0; double p=0,y=0,r=0; ULONGLONG tick=0; };
std::mutex camera_command_mutex;
CameraCommand camera_command;
bool native_camera_fault=false;
// The camera placed for the frame on screen, from the aircraft (cm): the HUD projects with it.
std::atomic<float> applied_offset_x{0},applied_offset_y{0},applied_offset_z{0};
// What the game's own camera did this frame, for the log: its view (cockpit or nose: at the
// aircraft; chase: behind it) and its place along and above its own view, cm.
struct GameView { bool near_view=false, game_turned=false; double along=0,rise=0; double rotation[3]{}; };   // rotation: the view on screen
// The final view's FOV (float, deg), after its place and rotation in the same cache: the engine's
// CameraCacheEntry has POV at +0x10 and MinimalViewInfo has FOV at +0x30 (object dump, this build).
constexpr size_t pov_fov_offset=0x14A0+0x30;
inline float* pov_fov(void* manager) { return reinterpret_cast<float*>(static_cast<unsigned char*>(manager)+pov_fov_offset); }
inline bool plausible_fov(float fov) { return std::isfinite(fov) && fov>5 && fov<170; }
// The game's own FOV this frame: what its update left in the cache, unless that is still the value
// written last frame (a camera that eased from the cache would carry the addition on and grow it).
struct FovWrite { float base=0, written=0; } fov_write;
float game_fov_of(float cached) {
    if(fov_write.written!=0 && std::abs(cached-fov_write.written)<1e-3f) return fov_write.base;
    return cached;
}
void write_fov(void* manager,float base,float add) {
    if(add==0) { fov_write={}; return; }
    const float fov=std::clamp(base+add,20.0f,150.0f);
    *pov_fov(manager)=fov;
    fov_write={base,fov};
}
// Single game-thread publisher; try-lock also makes an unexpected concurrent
// callback nonblocking. Old commands still expire at the original 250ms limit.
bool receive_camera(uintptr_t manager,uintptr_t pawn,double p,double y,double r) {
    std::unique_lock<std::mutex> lock(camera_command_mutex,std::try_to_lock);
    if(!lock.owns_lock()) return false;
    camera_command={manager,pawn,p,y,r,GetTickCount64()};
    return true;
}
// The head in a cockpit or nose view: where it looks, as yaw and pitch off the game's own view
// (deg), eased toward what the mode wants. At (0,0) the view is the game's exactly.
struct NearHead { float yaw=0, pitch=0; } near_head;
// UE rotation (pitch, yaw, roll; deg) of a view looking along f with up u (as camera.lua does)
void view_rotation(const flight::V& f,const flight::V& u,double out[3]) {
    using namespace flight;
    const float p=pitch(f), y=yaw(f);
    const V right=cross(u,f), neutral_right{-std::sin(y*rad),std::cos(y*rad),0}, neutral_up=cross(f,neutral_right);
    out[0]=p; out[1]=y; out[2]=std::atan2(-dot(right,neutral_up),dot(right,neutral_right))/rad;
}
inline void near_view_head_limit(float& cap_x,float& cap_y) {
    const float half_w=std::clamp(view_fov.load(),20.0f,150.0f)*0.5f;
    const float half_h=std::atan(std::tan(half_w*flight::rad)/std::max(view_aspect.load(),0.5f))/flight::rad;
    cap_x=half_w+config.near_view_hud; cap_y=half_h+config.near_view_hud;
}
// The near view's head for this frame: F free look, or the mouse aim by near_view_camera
struct NearSteady { bool primed=false; flight::V f{1,0,0}, u{0,0,1}; } near_steady;   // the view's give (near_view_inertia)
void near_view_head(const flight::Basis& game_view,float dt,double out[3]) {
    using namespace flight;
    // The view lags the aircraft by near_view_inertia s (a head's give), and its roll is held
    // near_view_level toward level: what the head turns from.
    NearSteady& n=near_steady;
    if(!n.primed || config.near_view_inertia<=0) { n.f=game_view.f; n.u=game_view.u; n.primed=true; }
    else {
        const float k=1-std::exp(-std::clamp(dt,0.0f,0.1f)/config.near_view_inertia);
        n.f=unit(n.f+(game_view.f-n.f)*k);
        const V u=n.u+(game_view.u-n.u)*k; n.u=unit(u-n.f*dot(u,n.f));
    }
    Basis game{n.f,cross(n.u,n.f),n.u};
    if(config.near_view_level>0) {
        const float y=yaw(game.f)*rad;
        const V level_up=cross(game.f,V{-std::sin(y),std::cos(y),0});
        V u=game.u+(level_up-game.u)*config.near_view_level; u=u-game.f*dot(u,game.f);
        if(dot(u,u)>1e-6f) { game.u=unit(u); game.r=cross(game.u,game.f); }
    }
    auto offsets=[&](const V& d,float& y,float& p) {
        y=std::atan2(dot(d,game.r),dot(d,game.f))/rad;
        p=std::asin(std::clamp(dot(d,game.u),-1.0f,1.0f))/rad;
    };
    float want_yaw=0, want_pitch=0;
    if(free_look_held.load()) {
        if(config.near_view_camera==0 || config.near_view_camera==3) { want_yaw=near_look_yaw.load(); want_pitch=near_look_pitch.load(); }   // turned in the aircraft's frame
        else offsets(basis(look_pitch.load(),look_yaw.load(),0).f,want_yaw,want_pitch);
    } else if(config.near_view_camera==3) {
        // toward the cockpit stick's ring, no further than leaves the HUD's near edge in view (looking
        // up, its top edge still at the bottom of the screen): half the view plus half the HUD
        float cap_x, cap_y; near_view_head_limit(cap_x,cap_y);
        want_yaw=near_stick_x.load()*config.near_view_follow; want_pitch=near_stick_y.load()*config.near_view_follow;
        const float reach=std::hypot(want_yaw/std::max(cap_x,1e-3f),want_pitch/std::max(cap_y,1e-3f));   // an ellipse, not a box
        if(reach>1) { want_yaw/=reach; want_pitch/=reach; }
    } else if(config.near_view_camera>0) {
        offsets(basis(target_pitch.load(),target_yaw.load(),0).f,want_yaw,want_pitch);
        if(config.near_view_camera==2) {   // only what lies beyond the edge
            const float off=std::hypot(want_yaw,want_pitch), keep=std::max(0.0f,off-config.near_view_edge);
            if(off>1e-3f) { want_yaw*=keep/off; want_pitch*=keep/off; } else want_yaw=want_pitch=0;
        }
    }
    const float k=1-std::exp(-std::max(1.0f,config.camera_follow)*std::clamp(dt,0.0f,0.1f));
    near_head.yaw+=std::remainder(want_yaw-near_head.yaw,360.0f)*k;
    near_head.pitch+=(want_pitch-near_head.pitch)*k;
    if(want_yaw==0 && want_pitch==0 && std::hypot(near_head.yaw,near_head.pitch)<0.05f) near_head={};
    const float cy=std::cos(near_head.yaw*rad), sy=std::sin(near_head.yaw*rad), cp=std::cos(near_head.pitch*rad), sp=std::sin(near_head.pitch*rad);
    const V f=unit(game.f*(cp*cy)+game.r*(cp*sy)+game.u*sp);
    const V u=unit(game.u-f*dot(game.u,f));
    view_rotation(f,u,out);
}
bool apply_native_camera(void* manager,const CameraCommand& cmd,GameView& game_view,float dt) {
    __try {
        auto base=reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
        if(*reinterpret_cast<uintptr_t*>(manager)!=base+0xC9D3160) return false;
        auto root=*reinterpret_cast<uintptr_t*>(cmd.pawn+0x1A0);
        if(!root) return false;
        const double* position=reinterpret_cast<const double*>(root+0x220);
        double loc[3]={position[0],position[1],position[2]};
        for(double n:loc) if(!std::isfinite(n)||std::abs(n)>1e12) return false;
        // The game's own camera for this frame: its update has just filled the cache.
        const double* previous=reinterpret_cast<const double*>(static_cast<unsigned char*>(manager)+0x14A0);
        for(unsigned i=0;i<6;++i) if(!std::isfinite(previous[i])) return false;
        auto axes=flight::basis(static_cast<float>(cmd.p),static_cast<float>(cmd.y),static_cast<float>(cmd.r));
        const auto game=flight::basis(static_cast<float>(previous[3]),static_cast<float>(previous[4]),static_cast<float>(previous[5]));
        const double off[3]={previous[0]-loc[0],previous[1]-loc[1],previous[2]-loc[2]};
        game_view.along=off[0]*game.f.x+off[1]*game.f.y+off[2]*game.f.z;
        game_view.rise=off[0]*game.u.x+off[1]*game.u.y+off[2]*game.u.z;
        // Cockpit and nose views put the game's camera at the aircraft (the fuselage hidden, or
        // the cockpit shown), its chase view well behind. A near view is left as the game has it,
        // looking along the aircraft; the mouse ring moves across it. (Moved 30 m back, it had been
        // a chase view without the fuselage or with the cockpit's HUD; at the game's position but
        // turned with the mouse, a free look all round, reported 2026-10-07.)
        game_view.near_view=game_view.along>-800;
        double result[6]={previous[0],previous[1],previous[2],cmd.p,cmd.y,cmd.r};
        if(game_view.near_view) {
            applied_offset_x.store(static_cast<float>(off[0]));
            applied_offset_y.store(static_cast<float>(off[1]));
            applied_offset_z.store(static_cast<float>(off[2]));
            // The game turning the view itself (the lock key held on a target or a story point):
            // its camera more than 12 deg off the nose is left alone, head and give reset (turned
            // further by ours, the view went wrong; reported 2026-10-07)
            const flight::V nose=flight::basis(pose_pitch.load(),pose_yaw.load(),pose_roll.load()).f;
            if(flight::dot(game.f,nose)<std::cos(12*flight::rad)) {
                near_head={}; near_steady.primed=false; game_view.game_turned=true;
                for(int i=0;i<3;++i) game_view.rotation[i]=previous[3+i];
                return true;
            }
            near_view_head(game,dt,game_view.rotation);
            if(near_head.yaw==0 && near_head.pitch==0 && config.near_view_inertia<=0 && config.near_view_level<=0) {
                for(int i=0;i<3;++i) game_view.rotation[i]=previous[3+i]; return true;
            }
            for(int i=0;i<3;++i) result[3+i]=game_view.rotation[i];
            memcpy(static_cast<unsigned char*>(manager)+0x14A0,result,sizeof(result));
            return true;
        }
        near_head={}; near_steady.primed=false;   // a chase view: the head starts ahead, the view steady, next time
        for(int i=0;i<3;++i) game_view.rotation[i]=result[3+i];
        {
            // camera_distance 0: the game's own distance behind and height, along this view
            const double back=config.camera_distance>0 ? config.camera_distance*100.0 : -game_view.along;
            const double up=config.camera_distance>0 ? config.camera_height*100.0 : game_view.rise;
            result[0]=loc[0]-back*axes.f.x+up*axes.u.x;
            result[1]=loc[1]-back*axes.f.y+up*axes.u.y;
            result[2]=loc[2]-back*axes.f.z+up*axes.u.z;
        }
        applied_offset_x.store(static_cast<float>(result[0]-loc[0]));
        applied_offset_y.store(static_cast<float>(result[1]-loc[1]));
        applied_offset_z.store(static_cast<float>(result[2]-loc[2]));
        // GetCameraLocation/Rotation both read this same current POV cache.
        memcpy(static_cast<unsigned char*>(manager)+0x14A0,result,sizeof(result));
        return true;
    } __except(EXCEPTION_EXECUTE_HANDLER) { return false; }
}
// Handing the view over (autopilot, gaze, F8, a scene) and taking it back are eased over
// camera_blend_s instead of cut (a jump to the game's camera and back, reported 2026-10-07): the
// last view of ours, held as an offset from the aircraft so that it moves with it, mixed into the
// game's own; and back, the game's mixed into ours.
constexpr float camera_blend_s=0.5f;
// A cockpit or nose view is held in the aircraft's frame (body: offset along nose, right wing, up;
// f and u the view's axes), so it turns with the aircraft: held still in the world while the aircraft
// pitched, it sank down the canopy and came back as the lock key's view took over (reported 2026-10-07).
struct CameraBlend { bool held=false, body=false; double offset[3]{}, rotation[3]{}; flight::V f{1,0,0}, u{0,0,1};
    uintptr_t manager=0; float release=1, resume=0; float fov_add=0; } camera_blend;   // fov_add: last added (camera_fov_add)
// a and b: UE POVs (location; pitch, yaw, roll); w 0 is a, 1 b
void mix_pov(const double a[6],const double b[6],float w,double out[6]) {
    using namespace flight;
    for(int i=0;i<3;++i) out[i]=a[i]+(b[i]-a[i])*w;
    const Basis A=basis(float(a[3]),float(a[4]),float(a[5])), B=basis(float(b[3]),float(b[4]),float(b[5]));
    V f=A.f*(1-w)+B.f*w; f=dot(f,f)>1e-6f ? unit(f) : B.f;
    V u=A.u*(1-w)+B.u*w; u=u-f*dot(u,f); u=dot(u,u)>1e-6f ? unit(u) : B.u;
    view_rotation(f,u,out+3);
}
bool aircraft_location(uintptr_t pawn,double loc[3]) {
    __try {
        if(!pawn) return false;
        auto root=*reinterpret_cast<uintptr_t*>(pawn+0x1A0);
        if(!root) return false;
        const double* position=reinterpret_cast<const double*>(root+0x220);
        for(int i=0;i<3;++i) { loc[i]=position[i]; if(!std::isfinite(loc[i])||std::abs(loc[i])>1e12) return false; }
        return true;
    } __except(EXCEPTION_EXECUTE_HANDLER) { return false; }
}
// The game's camera of this frame (in the cache) moved from our last view, by the release's progress
// Two views near enough to ease between: within 60 deg and 50 m (a scene's or a resupply's camera
// is cut to, not swung through: a view eased toward one facing away turned 180 deg, reported)
bool views_close(const double a[6],const double b[6]) {
    const flight::Basis A=flight::basis(float(a[3]),float(a[4]),float(a[5])), B=flight::basis(float(b[3]),float(b[4]),float(b[5]));
    const double d=std::sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2]));
    return flight::dot(A.f,B.f)>0.5f && d<5000;
}
void ease_release(void* manager,float dt) {
    CameraBlend& b=camera_blend;
    if(!b.held || b.release>=1 || reinterpret_cast<uintptr_t>(manager)!=b.manager) { b.held=false; return; }
    __try {
        auto base=reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
        if(*reinterpret_cast<uintptr_t*>(manager)!=base+0xC9D3160) { b.held=false; return; }
        double loc[3];
        if(!aircraft_location(aircraft.load(),loc)) { b.held=false; return; }
        double* cache=reinterpret_cast<double*>(static_cast<unsigned char*>(manager)+0x14A0);
        for(int i=0;i<6;++i) if(!std::isfinite(cache[i])) { b.held=false; return; }
        b.release=std::min(1.0f,b.release+std::clamp(dt,0.0f,0.1f)/camera_blend_s);
        const float w=b.release*b.release*(3-2*b.release);
        double ours[6]={loc[0]+b.offset[0],loc[1]+b.offset[1],loc[2]+b.offset[2],b.rotation[0],b.rotation[1],b.rotation[2]};
        if(b.body) {
            const flight::Basis B=flight::basis(pose_pitch.load(),pose_yaw.load(),pose_roll.load());
            const flight::V o=B.f*float(b.offset[0])+B.r*float(b.offset[1])+B.u*float(b.offset[2]);
            ours[0]=loc[0]+o.x; ours[1]=loc[1]+o.y; ours[2]=loc[2]+o.z;
            view_rotation(B.f*b.f.x+B.r*b.f.y+B.u*b.f.z,B.u*b.u.z+B.f*b.u.x+B.r*b.u.y,ours+3);
        }
        if(!views_close(ours,cache)) { b.held=false; return; }
        double out[6]; mix_pov(ours,cache,w,out);
        memcpy(cache,out,sizeof(out));
        const float game_fov=game_fov_of(*pov_fov(manager));   // the FOV added fades with the view
        if(plausible_fov(game_fov)) write_fov(manager,game_fov,b.fov_add*(1-w));
        if(b.release>=1) b.held=false;
    } __except(EXCEPTION_EXECUTE_HANDLER) { b.held=false; }
}
void __fastcall update_native_camera(void* manager,float dt) {
    original_camera_update(manager,dt);
    PerfSpan timing(perf_camera);
    if(native_camera_fault || !running.load()) return;
    // not ours this frame: ease it over, and start the head and its give afresh when it comes back
    // (kept, they swung the view from before a resupply to the aircraft after it)
    if(!active.load() || !enabled.load() || yielding()) { camera_released_since.store(0); ease_release(manager,dt); camera_blend.resume=0; near_head={}; near_steady.primed=false; return; }
    CameraCommand cmd;
    {
        std::unique_lock<std::mutex> lock(camera_command_mutex,std::try_to_lock);
        if(!lock.owns_lock()) return;
        cmd=camera_command;
    }
    // Report actual ownership interruptions, without relaxing release safeguards.
    // Ignore other camera managers rather than counting them as interruptions.
    if(cmd.manager && reinterpret_cast<uintptr_t>(manager)!=cmd.manager) return;
    const char* reason=nullptr;
    if(!cmd.manager) reason="Lua release";
    else if(cmd.pawn!=aircraft.load()) reason="aircraft changed";
    else if(GetTickCount64()-cmd.tick>250) reason="camera command timeout";
    else if(GetTickCount64()-pose_tick.load()>250) reason="pose timeout";
    else if(!foreground_is_game()) reason="foreground lost";
    if(!cmd.manager) { if(!camera_released_since.load()) camera_released_since.store(GetTickCount64()); }
    else camera_released_since.store(0);
    static const char* previous_reason=nullptr;
    if(reason!=previous_reason) {
        log_line("native camera ownership: %s",reason?reason:"resumed");
        previous_reason=reason;
    }
    if(reason) { ease_release(manager,dt); camera_blend.resume=0; near_head={}; near_steady.primed=false; return; }
    double game_pov[6]{};   // the game's own camera this frame, to ease in from
    memcpy(game_pov,static_cast<unsigned char*>(manager)+0x14A0,sizeof(game_pov));
    const float game_fov=game_fov_of(*pov_fov(manager));
    GameView game_view;
    if(!apply_native_camera(manager,cmd,game_view,dt)) {
        native_camera_fault=true;
        log_line("native camera disabled: invalid live camera/aircraft data");
        return;
    }
    {
        CameraBlend& b=camera_blend;
        double* cache=reinterpret_cast<double*>(static_cast<unsigned char*>(manager)+0x14A0);
        if(b.resume<1) {   // taking the view back: from the game's toward ours
            b.resume=std::min(1.0f,b.resume+std::clamp(dt,0.0f,0.1f)/camera_blend_s);
            bool finite=true; for(double v:game_pov) finite=finite && std::isfinite(v);
            if(finite && !views_close(game_pov,cache)) b.resume=1;   // too far apart: cut
            else if(finite) {
                double out[6]; mix_pov(game_pov,cache,b.resume*b.resume*(3-2*b.resume),out);
                memcpy(cache,out,sizeof(out));
                for(int i=0;i<3;++i) game_view.rotation[i]=out[3+i];
            }
        }
        double loc[3];   // ours, held as an offset from the aircraft for a release
        if(aircraft_location(cmd.pawn,loc)) {
            for(int i=0;i<3;++i) { b.offset[i]=cache[i]-loc[i]; b.rotation[i]=cache[3+i]; }
            b.body=game_view.near_view;
            if(b.body) {
                using namespace flight;
                const Basis B=basis(pose_pitch.load(),pose_yaw.load(),pose_roll.load());
                const Basis A=basis(float(cache[3]),float(cache[4]),float(cache[5]));
                const V o{float(b.offset[0]),float(b.offset[1]),float(b.offset[2])};
                b.offset[0]=dot(o,B.f); b.offset[1]=dot(o,B.r); b.offset[2]=dot(o,B.u);
                b.f={dot(A.f,B.f),dot(A.f,B.r),dot(A.f,B.u)}; b.u={dot(A.u,B.f),dot(A.u,B.r),dot(A.u,B.u)};
            }
            b.held=true; b.release=0; b.manager=reinterpret_cast<uintptr_t>(manager);
        }
    }
    // The FOV: the game's own (it widens with speed and afterburner) plus camera_fov_add in a chase
    // view or near_view_fov_add in a cockpit or nose view, eased in with the view; none while the
    // game turns a near view itself (the lock key held).
    if(plausible_fov(game_fov)) {
        const float add=game_view.game_turned ? 0.0f : game_view.near_view ? config.near_view_fov_add : config.camera_fov_add;
        const float r=camera_blend.resume, w=r>=1 ? 1.0f : r*r*(3-2*r);
        camera_blend.fov_add=add;
        write_fov(manager,game_fov,add*w);
    }
    // the game's view, logged when it changes and every 30 s (its chase distance, for the
    // player model's detail: issue #7)
    static int logged_view=-1; static ULONGLONG view_report=0;
    if(int(game_view.near_view)!=logged_view || GetTickCount64()-view_report>30000) {
        logged_view=int(game_view.near_view); view_report=GetTickCount64();
        log_line("native camera view: %s; the game's camera %.0f cm along its view, %.0f cm above, FOV %.1f",
            game_view.near_view?"cockpit/nose (the game's camera kept)":"chase",game_view.along,game_view.rise,view_fov.load());
    }
    near_view_active.store(game_view.near_view);
    applied_camera_pitch.store(static_cast<float>(game_view.rotation[0]));
    applied_camera_yaw.store(static_cast<float>(game_view.rotation[1]));
    applied_camera_roll.store(static_cast<float>(game_view.rotation[2]));
    applied_camera_tick.store(GetTickCount64());
    if(overlay_frame_event) SetEvent(overlay_frame_event);
    static bool reported=false;
    if(!reported) { reported=true; log_line("native camera POST-UPDATE ACTIVE: live aircraft position, final POV cache"); }
}
bool install_native_camera() {
    auto* base=reinterpret_cast<unsigned char*>(GetModuleHandleW(nullptr));
    auto* dos=reinterpret_cast<IMAGE_DOS_HEADER*>(base);
    auto* nt=reinterpret_cast<IMAGE_NT_HEADERS64*>(base+dos->e_lfanew);
    const unsigned char update_bytes[]={0x48,0x8B,0xC4,0x48,0x89,0x58,0x18,0x57,0x48,0x81,0xEC,0x40,0x01,0,0};
    const unsigned char cache_bytes[]={0x48,0x8D,0x81,0xA0,0x14,0,0,0xC3};
    const unsigned char actor_bytes[]={0x48,0x8B,0x81,0xA0,0x01,0,0};
    if(nt->FileHeader.TimeDateStamp!=0x6AA0E27D || nt->OptionalHeader.SizeOfImage!=0x2195A000 ||
       memcmp(base+0x70C8ED0,update_bytes,sizeof(update_bytes)) ||
       memcmp(base+0x48FC6B0,cache_bytes,sizeof(cache_bytes)) ||
       memcmp(base+0x3FD7430+0x19,actor_bytes,sizeof(actor_bytes))) {
        log_line("native camera refused: executable identity/signature mismatch"); return false;
    }
    auto slot=reinterpret_cast<void**>(base+0xC9D3160+241*sizeof(void*));
    if(*slot!=base+0x70C8ED0) { log_line("native camera refused: vtable slot already changed"); return false; }
    DWORD protection=0;
    if(!VirtualProtect(slot,sizeof(void*),PAGE_READWRITE,&protection)) return false;
    original_camera_update=reinterpret_cast<CameraUpdate>(*slot);
    InterlockedExchangePointer(slot,reinterpret_cast<void*>(&update_native_camera));
    DWORD ignored=0; VirtualProtect(slot,sizeof(void*),protection,&ignored);
    log_line("native camera post-update registered; activation requires matching live manager and fresh pose");
    return true;
}
