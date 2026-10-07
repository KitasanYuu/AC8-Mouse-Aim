// AC8 Mouse Aim: offline-only closed-loop mouse flight control for ACE COMBAT 8.
// Reads aircraft attitude supplied by UE4SS Lua, captures non-exclusive mouse
// deltas, and replaces only the three player input axes. Flight-model state is
// never edited.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define DIRECTINPUT_VERSION 0x0800
#include <windows.h>
#include <dinput.h>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <mutex>
#include <array>
#include "vendor/minhook/include/MinHook.h"
bool telemetry_send(int port, const char* data, int length);  // telemetry.cpp
bool gamepad_read(float values[6], unsigned& buttons);        // telemetry.cpp
#include "yaw_signature.h"
#include "flight_math.h"
#include "free_look.h"
#include "flight_logic.h"
#include "lua_bridge.h"

#pragma comment(lib, "dinput8.lib")
#pragma comment(lib, "dxguid.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")

namespace {
using Processor = uintptr_t(__fastcall*)(unsigned char*, unsigned char*);
using GetRawInputDataFn = UINT(WINAPI*)(HRAWINPUT, UINT, LPVOID, PUINT, UINT);

struct Config {
    float sensitivity = 0.10f;
    float roll_gain = 0.022f;
    float pitch_gain = 0.026f;
    float yaw_gain = 0.012f;
    float turn_pull = 0.42f;
    float max_bank = 65.0f;
    float smoothing = 0.12f;
    float roll_damping = 0.008f;
    float pitch_damping = 0.010f;
    float yaw_damping = 0.30f;
    float dead_zone = 1.25f;
    float pitch_sign = 1.0f;
    float roll_sign = -1.0f;
    float yaw_sign = -1.0f;
    float horizontal_fov = 100.0f;
    float vertical_fov = 62.0f;
    int pitch_slot = 0;
    int roll_slot = 2;
    int input_probe = 0;  // 1: log stepping values in the pawn's input block (discovery)
    // How fast the chase camera turns to the mouse direction (1/s). MouseFlight used 5:
    // the aim ring then darted across the screen with each mouse move and drifted back
    // over ~0.6 s; War Thunder's camera follows the mouse closely and the ring stays put.
    float camera_follow = 12.0f;
    float camera_distance = 30.0f, camera_height = 6.0f;   // chase camera, m; distance 0 = the game's own
    // Cockpit and nose views: 0 the view fixed ahead (the mouse ring moves across it), 1 turned to
    // the mouse aim (War Thunder's mouse aim in its cockpit), 2 fixed until the aim is near_view_edge
    // deg off the view's centre, then turned just enough to keep it there. F free look in all three.
    // 3 (default): as 0, the view turned toward the ring while flying (War Thunder's realistic
    // battles cockpit) by near_view_follow of its offset, no further than leaves the HUD's near edge
    // in view (half the view plus near_view_hud, the HUD's half size in deg); the ring reaches as far
    // as turns the view that far, or keeps itself 3 deg inside the screen (not near_view_box_*: a box
    // of 0.55 of the screen let the view turn 14 by 6 deg, "held small, ahead").
    int near_view_camera = 3; float near_view_edge = 15.0f, near_view_hud = 8.0f, near_view_follow = 0.6f;
    // near_view_camera 0: 1 the ring stays where the mouse puts it on screen, within a box of
    // near_view_box_x/_y of the half width/height (a mouse joystick, as War Thunder's simulator
    // battles: held off centre it keeps the aircraft turning); 0 a direction in the world, as in
    // the chase view (the ring comes back to the nose as the aircraft arrives), kept in the box
    int near_view_aim = 1; float near_view_box_x = 0.55f, near_view_box_y = 0.55f;
    // Comfort in a cockpit view (it was "dizzying, and very sensitive"): the ring's reach as
    // reach^near_view_expo of the mouse's (fine near the nose, all of it at the edge), the mouse
    // scaled by near_view_mouse; the view lagging the aircraft by near_view_inertia s (a head's
    // give: the jolt of a roll's start and stop softened), its roll near_view_level held level.
    float near_view_expo = 1.5f, near_view_mouse = 1.0f, near_view_inertia = 0.06f, near_view_level = 0.0f;
    // Live telemetry to dev/telemetry (UDP port on 127.0.0.1); 0 = off.
    int telemetry_port = 0;
    // Post-stall request key (virtual-key code; 0 = none): held, a high-G pressed below
    // 500 km/h enters the game's post-stall maneuver (see post_stall_override).
    int post_stall_key = VK_XBUTTON1;
};

struct Pose {
    uintptr_t pawn = 0;
    float pitch = 0, yaw = 0, roll = 0;
    unsigned long long tick = 0;
};

HMODULE self_module{};
Processor original{};
void* hook_address{};
bool hook_created = false;
std::atomic<bool> running{false};
std::atomic<bool> enabled{true};
std::atomic<bool> hud_enabled{true};
std::atomic<bool> trace_enabled{false};
// Axes the player is flying by keyboard (1 pitch, 2 roll, 4 yaw), for trace analysis.
std::atomic<int> keyboard_axes{0};
std::atomic<float> input_throttle{0}, input_brake{0};  // the game's InputThrottle / InputBrake
// Post-stall requested: the key held (a toggle that a later high-G used up could be spent by
// an unintended one, or one over 500 km/h, reported).
std::atomic<bool> post_stall_armed{false};
// The player's own axis input as the game wrote it, before the override (game convention).
std::atomic<float> raw_axis_pitch{0}, raw_axis_yaw{0}, raw_axis_roll{0};
// Actor position (cm) and the flight path derived from it (game thread only).
double actor_x=0, actor_y=0, actor_z=0, previous_actor[3]{};
bool actor_valid=false;
flight::V path_velocity{}, path_raw_velocity{}, path_accel{};
int aircraft_serial=0;  // game thread only
std::atomic<bool> game_paused{false};
std::atomic<bool> gaze_active{false};
// AC8 AutoPilot is engaged by pressing both yaw keys together. While it flies, the
// mod leaves controls and camera to the game; Q+E again or a deliberate mouse move
// takes back control with the target re-anchored to the nose.
std::atomic<bool> autopilot_active{false};
std::atomic<long> autopilot_travel{0};
constexpr float autopilot_takeover_degrees=6.0f;
bool yielding() { return gaze_active.load() || autopilot_active.load(); }
std::atomic<bool> resume_center_requested{false};
std::atomic<bool> active{false};
std::atomic<uintptr_t> aircraft{0};
std::atomic<float> pose_pitch{0}, pose_yaw{0}, pose_roll{0};
std::atomic<float> camera_pitch{0}, camera_yaw{0}, camera_roll{0};
// Camera the native hook applied for the frame being rendered: the overlay projects
// with it so the rings sit on the same frame as the scene, and redraws once per frame.
std::atomic<float> applied_camera_pitch{0}, applied_camera_yaw{0}, applied_camera_roll{0};
std::atomic<unsigned long long> applied_camera_tick{0};
HANDLE overlay_frame_event{};
std::atomic<float> view_fov{100};
// Cockpit and nose views (native_camera.h): the view is the game's, narrower than the chase view
// and fixed ahead, so the mouse is scaled by the views' FOVs and its ring kept on screen.
std::atomic<bool> near_view_active{false};
std::atomic<float> chase_fov{0};            // the FOV last seen in a chase view
std::atomic<float> view_aspect{16.0f/9};    // the game window's width over height
std::atomic<float> near_base_pitch{0}, near_base_yaw{0}, near_base_roll{0};   // the game's own near view (along the aircraft)
std::atomic<float> near_look_yaw{0}, near_look_pitch{0};   // F held in a near view: the head, deg off that view
std::atomic<float> near_stick_x{0}, near_stick_y{0};       // the cockpit stick's ring, deg off that view
// near_view_camera 3: how far the view may turn (deg): half the view plus the HUD's half size
inline void near_view_head_limit(float& cap_x,float& cap_y);
std::atomic<float> view_offset_x{0}, view_offset_y{0}, view_offset_z{0};
std::atomic<float> roll_reference{0};
std::atomic<unsigned long long> pose_tick{0};
std::atomic<long> mouse_dx{0}, mouse_dy{0};
std::atomic<float> command_pitch{0}, command_roll{0}, command_yaw{0};
std::atomic<float> target_pitch{0}, target_yaw{0};
// Camera destination is independent of the flight target while F is held.
std::atomic<float> look_pitch{0}, look_yaw{0};
std::atomic<bool> free_look_held{false};   // F held: the camera on look_pitch/look_yaw, not the aim
flight::FreeLook free_look;
flight::V desired_aim{1,0,0};
bool desired_aim_valid=false;
// Flight logic runs through this table: the built-in copy, or a hot-loaded
// ac8_flight_logic.dll during development. Called only on the game thread.
struct LogicApi {
    int (*abi)(unsigned*,unsigned*,unsigned*);
    void (*configure)(const wchar_t*);
    void (*reset)(void*);
    void (*step)(void*,const flight::LogicInput*,flight::LogicOutput*);
};
LogicApi logic{flight::logic_api::abi,flight::logic_api::configure,flight::logic_api::reset,flight::logic_api::step};
alignas(64) unsigned char logic_state[16384];  // model bank needs ~3 KB; headroom for hot-swapped logic
static_assert(sizeof(flight::LogicState)<=sizeof(logic_state),"logic state buffer too small");
HMODULE logic_module{};
wchar_t logic_live_path[MAX_PATH]{};
int logic_generation=0;
FILETIME logic_file_time{}, config_file_time{};
ULONGLONG dev_poll_at=0;
void logic_reset() { logic.reset(logic_state); }
std::atomic<bool> recenter_requested{true};
float previous_pitch{}, previous_yaw{}, previous_roll{};
float filtered_pitch_rate{}, filtered_yaw_rate{}, filtered_roll_rate{};
unsigned long long previous_pose_clock{}, telemetry_tick{};
// The game's world clock (s) for this pose and the previous one; -1 when unknown.
double game_time=-1, previous_game_time=-1;
Config config;
wchar_t module_folder[MAX_PATH]{};
wchar_t status_path[MAX_PATH]{};
wchar_t config_path[MAX_PATH]{};
wchar_t request_path[MAX_PATH]{};
HANDLE journal = INVALID_HANDLE_VALUE;
HWND game_window{};
HWND overlay_window{};
IDirectInput8W* direct_input{};
IDirectInputDevice8W* mouse_device{};
GetRawInputDataFn original_get_raw_input_data{};
std::atomic<bool> raw_input_hooked{false};

float clamp_axis(float value) { return std::clamp(value, -1.0f, 1.0f); }
float wrap_degrees(float value) {
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}

void init_paths() {
    if (module_folder[0]) return;
    GetModuleFileNameW(self_module, module_folder, MAX_PATH);
    if (auto* slash = wcsrchr(module_folder, L'\\')) *slash = 0;
    swprintf_s(status_path, L"%s\\mouse-aim-status.txt", module_folder);
    swprintf_s(config_path, L"%s\\..\\config.ini", module_folder);
    swprintf_s(request_path, L"%s\\mouse-aim-request.txt", module_folder);
}

#include "async_diagnostics.h"

float read_config_float(const wchar_t* key, float fallback) {
    wchar_t value[64]{};
    wchar_t fallback_text[64]{};
    swprintf_s(fallback_text, L"%.6f", fallback);
    GetPrivateProfileStringW(L"control", key, fallback_text, value, 64, config_path);
    wchar_t* end{};
    float parsed = wcstof(value, &end);
    return end != value && std::isfinite(parsed) ? parsed : fallback;
}

int read_config_int(const wchar_t* key, int fallback) {
    return GetPrivateProfileIntW(L"control", key, fallback, config_path);
}

// A key by name, for every bindable key: XButton1/XButton2 (mouse side buttons, back and
// forward), MButton, F1-F24, a letter or digit, or a virtual-key code (0x05 or 5). Empty or
// "none" is no key; anything else unreadable keeps the default.
int read_config_key(const wchar_t* key, int fallback) {
    wchar_t value[64]{};
    GetPrivateProfileStringW(L"control", key, L"", value, 64, config_path);
    std::wstring text(value);
    if(text==L"") return fallback;
    while(!text.empty() && iswspace(text.back())) text.pop_back();
    while(!text.empty() && iswspace(text.front())) text.erase(text.begin());
    for(auto& c:text) c=towupper(c);
    if(text.empty() || text==L"NONE") return 0;
    static const struct { const wchar_t* name; int vk; } names[]={
        {L"XBUTTON1",VK_XBUTTON1},{L"XBUTTON2",VK_XBUTTON2},{L"MBUTTON",VK_MBUTTON},
        {L"SHIFT",VK_SHIFT},{L"CTRL",VK_CONTROL},{L"ALT",VK_MENU},{L"SPACE",VK_SPACE},
        {L"TAB",VK_TAB},{L"CAPSLOCK",VK_CAPITAL}};
    for(const auto& n:names) if(text==n.name) return n.vk;
    if(text.size()>=2 && text[0]==L'F' && iswdigit(text[1])) {
        const int f=_wtoi(text.c_str()+1);
        if(f>=1 && f<=24) return VK_F1+f-1;
    }
    if(text.size()==1 && (iswalpha(text[0]) || iswdigit(text[0]))) return int(text[0]);
    wchar_t* end{};
    const long code=wcstol(text.c_str(),&end,0);
    if(end!=text.c_str() && *end==0 && code>0 && code<256) return int(code);
    return fallback;
}

void load_config() {
    init_paths();
    config.sensitivity = std::clamp(read_config_float(L"sensitivity", config.sensitivity), 0.01f, 1.0f);
    config.roll_gain = std::clamp(read_config_float(L"roll_gain", config.roll_gain), 0.001f, 0.2f);
    config.pitch_gain = std::clamp(read_config_float(L"pitch_gain", config.pitch_gain), 0.001f, 0.2f);
    config.yaw_gain = std::clamp(read_config_float(L"yaw_gain", config.yaw_gain), 0.0f, 0.2f);
    config.turn_pull = std::clamp(read_config_float(L"turn_pull", config.turn_pull), 0.0f, 1.0f);
    config.max_bank = std::clamp(read_config_float(L"max_bank", config.max_bank), 20.0f, 89.0f);
    config.smoothing = std::clamp(read_config_float(L"smoothing", config.smoothing), 0.02f, 1.0f);
    config.roll_damping = std::clamp(read_config_float(L"roll_damping", config.roll_damping), 0.0f, 0.05f);
    config.pitch_damping = std::clamp(read_config_float(L"pitch_damping", config.pitch_damping), 0.0f, 0.05f);
    config.yaw_damping = std::clamp(read_config_float(L"yaw_damping", config.yaw_damping), 0.0f, 2.0f);
    config.dead_zone = std::clamp(read_config_float(L"dead_zone", config.dead_zone), 0.1f, 5.0f);
    config.pitch_sign = read_config_float(L"pitch_sign", config.pitch_sign) < 0 ? -1.0f : 1.0f;
    config.roll_sign = read_config_float(L"roll_sign", config.roll_sign) < 0 ? -1.0f : 1.0f;
    config.yaw_sign = read_config_float(L"yaw_sign", config.yaw_sign) < 0 ? -1.0f : 1.0f;
    config.horizontal_fov = std::clamp(read_config_float(L"horizontal_fov", config.horizontal_fov), 40.0f, 170.0f);
    config.vertical_fov = std::clamp(read_config_float(L"vertical_fov", config.vertical_fov), 30.0f, 120.0f);
    config.pitch_slot = std::clamp(read_config_int(L"pitch_slot", config.pitch_slot), 0, 2);
    config.roll_slot = std::clamp(read_config_int(L"roll_slot", config.roll_slot), 0, 2);
    config.input_probe = read_config_int(L"input_probe", 0);
    config.camera_follow = std::clamp(read_config_float(L"camera_follow", config.camera_follow), 1.0f, 60.0f);
    config.camera_distance = std::clamp(read_config_float(L"camera_distance", config.camera_distance), 0.0f, 200.0f);
    config.camera_height = std::clamp(read_config_float(L"camera_height", config.camera_height), -50.0f, 50.0f);
    config.near_view_camera = std::clamp(read_config_int(L"near_view_camera", config.near_view_camera), 0, 3);
    config.near_view_hud = std::clamp(read_config_float(L"near_view_hud", config.near_view_hud), 0.0f, 40.0f);
    config.near_view_follow = std::clamp(read_config_float(L"near_view_follow", config.near_view_follow), 0.0f, 1.0f);
    config.near_view_edge = std::clamp(read_config_float(L"near_view_edge", config.near_view_edge), 2.0f, 60.0f);
    config.near_view_aim = read_config_int(L"near_view_aim", config.near_view_aim) != 0;
    config.near_view_expo = std::clamp(read_config_float(L"near_view_expo", config.near_view_expo), 1.0f, 4.0f);
    config.near_view_mouse = std::clamp(read_config_float(L"near_view_mouse", config.near_view_mouse), 0.1f, 3.0f);
    config.near_view_inertia = std::clamp(read_config_float(L"near_view_inertia", config.near_view_inertia), 0.0f, 0.5f);
    config.near_view_level = std::clamp(read_config_float(L"near_view_level", config.near_view_level), 0.0f, 1.0f);
    config.near_view_box_x = std::clamp(read_config_float(L"near_view_box_x", config.near_view_box_x), 0.1f, 0.95f);
    config.near_view_box_y = std::clamp(read_config_float(L"near_view_box_y", config.near_view_box_y), 0.1f, 0.95f);
    config.telemetry_port = std::clamp(read_config_int(L"telemetry_port", 0), 0, 65535);
    config.post_stall_key = read_config_key(L"post_stall_key", VK_XBUTTON1);
    if (config.roll_slot == config.pitch_slot || config.pitch_slot == 1 || config.roll_slot == 1) {
        config.pitch_slot = 0;
        config.roll_slot = 2;
    }
}

struct WindowCandidate {
    HWND window{};
    long long area{};
};

BOOL CALLBACK find_game_window(HWND window, LPARAM result) {
    DWORD process{};
    GetWindowThreadProcessId(window, &process);
    if (process == GetCurrentProcessId() && window != overlay_window &&
        IsWindowVisible(window) && GetWindow(window, GW_OWNER) == nullptr) {
        wchar_t class_name[64]{};
        GetClassNameW(window, class_name, 64);
        if (wcscmp(class_name, L"AC8MouseAimOverlay") == 0) return TRUE;
        RECT client{};
        if (GetClientRect(window, &client)) {
            const long long width = client.right - client.left;
            const long long height = client.bottom - client.top;
            const long long area = width * height;
            auto* candidate = reinterpret_cast<WindowCandidate*>(result);
            if (width >= 640 && height >= 360 && area > candidate->area) {
                candidate->window = window;
                candidate->area = area;
            }
        }
    }
    return TRUE;
}

HWND locate_game_window() {
    WindowCandidate candidate{};
    EnumWindows(find_game_window, reinterpret_cast<LPARAM>(&candidate));
    return candidate.window;
}

bool foreground_is_game() {
    HWND foreground = GetForegroundWindow();
    DWORD process{};
    GetWindowThreadProcessId(foreground, &process);
    const bool is_game = process == GetCurrentProcessId() && foreground != overlay_window;
    if (is_game) game_window = foreground;
    return is_game;
}

UINT WINAPI capture_get_raw_input_data(HRAWINPUT input, UINT command, LPVOID data,
                                       PUINT size, UINT header_size) {
    const UINT result = original_get_raw_input_data(input, command, data, size, header_size);
    if (result != static_cast<UINT>(-1) && command == RID_INPUT && data &&
        result >= sizeof(RAWINPUTHEADER) + sizeof(RAWMOUSE)) {
        const auto* raw = static_cast<const RAWINPUT*>(data);
        if (raw->header.dwType == RIM_TYPEMOUSE &&
            !(raw->data.mouse.usFlags & MOUSE_MOVE_ABSOLUTE) &&
            foreground_is_game() && active.load() && enabled.load()) {
            if(autopilot_active.load()) {
                autopilot_travel.fetch_add(std::abs(raw->data.mouse.lLastX)+std::abs(raw->data.mouse.lLastY));
            } else if(!game_paused.load() && !gaze_active.load()) {
                mouse_dx.fetch_add(raw->data.mouse.lLastX);
                mouse_dy.fetch_add(raw->data.mouse.lLastY);
            }
        }
    }
    return result;
}

bool prepare_raw_input_capture() {
    auto* base = reinterpret_cast<unsigned char*>(GetModuleHandleW(nullptr));
    auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
    if (!base || dos->e_magic != IMAGE_DOS_SIGNATURE) return false;
    auto* nt = reinterpret_cast<IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE) return false;
    const auto& directory = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    if (!directory.VirtualAddress) return false;
    auto* descriptor = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(base + directory.VirtualAddress);
    for (; descriptor->Name; ++descriptor) {
        if (!descriptor->OriginalFirstThunk) continue;
        auto* names = reinterpret_cast<IMAGE_THUNK_DATA64*>(base + descriptor->OriginalFirstThunk);
        auto* imports = reinterpret_cast<IMAGE_THUNK_DATA64*>(base + descriptor->FirstThunk);
        for (; names->u1.AddressOfData; ++names, ++imports) {
            if (IMAGE_SNAP_BY_ORDINAL64(names->u1.Ordinal)) continue;
            const auto* by_name = reinterpret_cast<IMAGE_IMPORT_BY_NAME*>(base + names->u1.AddressOfData);
            if (strcmp(reinterpret_cast<const char*>(by_name->Name), "GetRawInputData") != 0) continue;
            original_get_raw_input_data = reinterpret_cast<GetRawInputDataFn>(imports->u1.Function);
            DWORD previous{};
            if (!VirtualProtect(&imports->u1.Function, sizeof(imports->u1.Function),
                                PAGE_READWRITE, &previous)) return false;
            InterlockedExchangePointer(reinterpret_cast<void**>(&imports->u1.Function),
                                       reinterpret_cast<void*>(&capture_get_raw_input_data));
            DWORD ignored{};
            VirtualProtect(&imports->u1.Function, sizeof(imports->u1.Function), previous, &ignored);
            raw_input_hooked.store(true);
            return true;
        }
    }
    return false;
}

bool prepare_mouse() {
    if (!game_window) game_window = locate_game_window();
    if (!game_window) return false;
    if (!direct_input && FAILED(DirectInput8Create(GetModuleHandleW(nullptr), DIRECTINPUT_VERSION,
            IID_IDirectInput8W, reinterpret_cast<void**>(&direct_input), nullptr))) return false;
    if (!mouse_device && FAILED(direct_input->CreateDevice(GUID_SysMouse, &mouse_device, nullptr))) return false;
    if (FAILED(mouse_device->SetDataFormat(&c_dfDIMouse2))) return false;
    if (FAILED(mouse_device->SetCooperativeLevel(game_window, DISCL_NONEXCLUSIVE | DISCL_BACKGROUND))) return false;
    return SUCCEEDED(mouse_device->Acquire()) || GetLastError() == ERROR_SUCCESS;
}

void mouse_loop() {
    bool f8_down = false, f9_down = false;
    while (running.load()) {
        if (!raw_input_hooked.load()) {
            if (!mouse_device && !prepare_mouse()) {
                Sleep(50);
                continue;
            }
            DIMOUSESTATE2 state{};
            HRESULT result = mouse_device->GetDeviceState(sizeof(state), &state);
            if (FAILED(result)) {
                mouse_device->Acquire();
            } else if (foreground_is_game() && active.load()) {
                if(autopilot_active.load()) {
                    autopilot_travel.fetch_add(std::abs(state.lX)+std::abs(state.lY));
                } else if(!game_paused.load() && !gaze_active.load()) {
                    mouse_dx.fetch_add(state.lX);
                    mouse_dy.fetch_add(state.lY);
                }
            }
        }
        static bool f7_down=false;
        bool f7=(GetAsyncKeyState(VK_F7)&0x8000)!=0;
        if(f7 && !f7_down && foreground_is_game()) {
            hud_enabled.store(!hud_enabled.load());
            log_line("HUD only: %s",hud_enabled.load()?"ON":"OFF");
        }
        f7_down=f7;
        static bool chord_down=false;
        const bool chord=foreground_is_game() && active.load() &&
            (GetAsyncKeyState('Q')&0x8000) && (GetAsyncKeyState('E')&0x8000);
        if(chord && !chord_down) {
            const bool engaged=!autopilot_active.load();
            autopilot_travel.store(0);
            autopilot_active.store(engaged);
            if(!engaged) recenter_requested.store(true);
            log_line("autopilot: %s",engaged?"Q+E engaged; controls and camera left to the game (Q+E or mouse takes back)"
                                            :"Q+E released to mouse aim; target re-anchored to nose");
        }
        chord_down=chord;
        if(autopilot_active.load() && autopilot_travel.load()*config.sensitivity>autopilot_takeover_degrees) {
            autopilot_active.store(false);
            autopilot_travel.store(0);
            recenter_requested.store(true);
            log_line("autopilot: mouse takeover; target re-anchored to nose");
        }
        static bool post_stall_down=false;
        const int psk=config.post_stall_key;
        const bool post_stall=psk>0 && (GetAsyncKeyState(psk)&0x8000)!=0;
        const bool requested=post_stall && foreground_is_game() && active.load() && enabled.load();
        if(requested!=post_stall_down) log_line("post-stall: %s",requested?"key held":"key released");
        post_stall_armed.store(requested);
        post_stall_down=requested;
        static bool f4_down=false;
        const bool f4=(GetAsyncKeyState(VK_F4)&0x8000)!=0;
        if(f4 && !f4_down && foreground_is_game()) {
            trace_enabled.store(!trace_enabled.load());
            log_line("flight trace: %s",trace_enabled.load()?"ON (per-frame TRACE lines)":"OFF");
        }
        f4_down=f4;
        bool f8 = (GetAsyncKeyState(VK_F8) & 0x8000) != 0;
        bool f9 = (GetAsyncKeyState(VK_F9) & 0x8000) != 0;
        if (f8 && !f8_down) enabled.store(!enabled.load());
        if (f9 && !f9_down) recenter_requested.store(true);
        f8_down = f8;
        f9_down = f9;
        Sleep(4);
    }
}

// One JSON object per frame to dev/telemetry: attitude, aim, body rates, flight path,
// position, the mod's stick commands and the player's own input (controller convention:
// pull, right, right positive), the gamepad, game inputs and the logic's state.
// ctl: 1 the mod flies, 0 the player does (F8 off, gaze, autopilot).
void send_telemetry(long long clock,float dt,const flight::LogicOutput& output,bool manual,bool controlling) {
    if(config.telemetry_port<=0) return;
    char packet[1600];
    int n=std::snprintf(packet,sizeof(packet),
        "{\"t\":%.4f,\"dt\":%.4f,\"att\":[%.3f,%.3f,%.3f],\"aim\":[%.3f,%.3f],\"rate\":[%.2f,%.2f,%.2f],"
        "\"vel\":[%.2f,%.2f,%.2f],\"acc\":[%.2f,%.2f,%.2f],\"pos\":[%.2f,%.2f,%.2f],\"stick\":[%.3f,%.3f,%.3f],"
        "\"raw\":[%.3f,%.3f,%.3f],\"in\":[%.2f,%.2f],\"gt\":%.4f,\"ctl\":%d,\"kb\":%d,\"manual\":%d,\"aircraft\":%d,",
        double(clock)/perf_frequency.QuadPart,dt,pose_pitch.load(),pose_yaw.load(),pose_roll.load(),
        target_pitch.load(),target_yaw.load(),filtered_pitch_rate,filtered_yaw_rate,filtered_roll_rate,
        path_velocity.x,path_velocity.y,path_velocity.z,path_accel.x,path_accel.y,path_accel.z,
        actor_x/100,actor_y/100,actor_z/100,output.pitch,output.roll,output.yaw,
        raw_axis_pitch.load()*config.pitch_sign,raw_axis_roll.load()*config.roll_sign,raw_axis_yaw.load()*config.yaw_sign,
        input_throttle.load(),input_brake.load(),game_time,controlling?1:0,keyboard_axes.load(),manual?1:0,aircraft_serial);
    // Mouse buttons and Space (1 left, 2 right, 4 middle, 8 X1, 16 X2, 32 Space): when
    // the player fires, for scoring recorded attacks.
    unsigned mouse_buttons=0;
    const int button_keys[]={VK_LBUTTON,VK_RBUTTON,VK_MBUTTON,VK_XBUTTON1,VK_XBUTTON2,VK_SPACE};
    for(int i=0;i<6;++i) if(GetAsyncKeyState(button_keys[i])&0x8000) mouse_buttons|=1u<<i;
    if(n>0) n+=std::snprintf(packet+n,sizeof(packet)-n,"\"mb\":%u,\"psm\":%d,",mouse_buttons,post_stall_armed.load()?1:0);
    float pad[6]{}; unsigned buttons=0;
    if(n>0 && gamepad_read(pad,buttons))
        n+=std::snprintf(packet+n,sizeof(packet)-n,"\"pad\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u],",
                         pad[0],pad[1],pad[2],pad[3],pad[4],pad[5],buttons);
    if(n>0 && n<int(sizeof(packet))-8) { packet[n++]='"'; packet[n++]='d'; packet[n++]='"'; packet[n++]=':'; packet[n++]='['; }
    for(int i=0;i<flight::DiagCount && n>0 && n<int(sizeof(packet))-32;++i)
        n+=std::snprintf(packet+n,sizeof(packet)-n,i?",%.3f":"%.3f",output.diag[i]);
    if(n>0 && n<int(sizeof(packet))-4) { packet[n++]=']'; packet[n++]='}'; telemetry_send(config.telemetry_port,packet,n); }
}

void update_commands() {
    using namespace flight;
    // Not flying for the player: F8 off, gaze or autopilot. The flight state is still
    // measured and recorded, so a mission flown by hand can be compared with the mod.
    const bool controlling=enabled.load() && !yielding();
    if (!active.load() || game_paused.load() || !controlling) {
        free_look.reset();
        logic_reset();
        desired_aim_valid=false;
        command_pitch.store(0); command_roll.store(0); command_yaw.store(0);
        if(game_paused.load() || yielding()) { mouse_dx.store(0); mouse_dy.store(0); }
        if(!active.load() || game_paused.load() || config.telemetry_port<=0) { previous_pose_clock = 0; return; }
    }
    const auto now = GetTickCount64();
    const auto frame_clock=perf_clock();
    // Time since the previous pose by the game's own clock. Attitude and position advance
    // by game frames, and the wall clock between calls jittered 9-18 ms against them
    // (logged: per-frame displacement uncorrelated with the wall interval), which made
    // the measured rates and flight path noisy. The wall clock is only a fallback.
    float elapsed=previous_pose_clock ?
        static_cast<float>(double(frame_clock-previous_pose_clock)/perf_frequency.QuadPart) : 0.0f;
    if(previous_pose_clock && game_time>=0 && previous_game_time>=0) {
        const double game_elapsed=game_time-previous_game_time;
        if(game_elapsed<=0) return;   // same game frame again: nothing has moved
        elapsed=static_cast<float>(game_elapsed);
    }
    const float dt=previous_pose_clock ? std::clamp(elapsed,0.002f,0.1f) : 1.0f/60;
    const Basis b = basis(pose_pitch.load(),pose_yaw.load(),pose_roll.load());
    const Basis old = basis(previous_pitch,previous_yaw,previous_roll);
    // Estimate body angular velocity from the moving basis, avoiding Euler wrap/pole artifacts.
    V omega = (cross(old.f,b.f)+cross(old.r,b.r)+cross(old.u,b.u))*(0.5f/dt/rad);
    float a = 1-std::exp(-12*dt);
    // A pose jump no aircraft can fly (mission start placement, respawn) re-anchors
    // the target to the new nose instead of turning back toward the old heading.
    const float jump=std::acos(std::clamp(std::min(dot(old.f,b.f),dot(old.u,b.u)),-1.0f,1.0f))/rad;
    if (previous_pose_clock && jump>15.0f && jump/std::max(elapsed,0.001f)>600.0f) {
        recenter_requested.store(true);
        log_line("pose jump %.1f deg in %.3fs: target re-anchored to nose",jump,elapsed);
    }
    // Flight path: velocity from the actor position, acceleration from the velocity,
    // both low-passed (position steps are frame-quantized).
    {
        const double px[3]={actor_x,actor_y,actor_z};
        if(actor_valid && previous_pose_clock && elapsed>0.0005f && elapsed<=0.2f) {
            const V raw{float((px[0]-previous_actor[0])/elapsed/100),float((px[1]-previous_actor[1])/elapsed/100),
                        float((px[2]-previous_actor[2])/elapsed/100)};
            const V raw_accel=(raw-path_raw_velocity)*(1.0f/elapsed);
            path_raw_velocity=raw;
            path_velocity=path_velocity+(raw-path_velocity)*(1-std::exp(-elapsed/0.05f));
            path_accel=path_accel+(raw_accel-path_accel)*(1-std::exp(-elapsed/0.15f));
        } else {
            path_velocity=path_raw_velocity=path_accel=V{};
        }
        for(int i=0;i<3;++i) previous_actor[i]=px[i];
        actor_valid=true;
    }
    if (!previous_pose_clock || elapsed>0.2f) {
        omega={};
        filtered_pitch_rate=filtered_yaw_rate=filtered_roll_rate=0;
        logic_reset();
    }
    filtered_pitch_rate += (-dot(omega,b.r)-filtered_pitch_rate)*a;
    filtered_yaw_rate += (dot(omega,b.u)-filtered_yaw_rate)*a;
    filtered_roll_rate += (-dot(omega,b.f)-filtered_roll_rate)*a;
    previous_pitch=pose_pitch.load(); previous_yaw=pose_yaw.load(); previous_roll=pose_roll.load();
    previous_pose_clock=frame_clock;
    previous_game_time=game_time;
    if(!controlling) {
        flight::LogicInput input{pose_pitch.load(),pose_yaw.load(),pose_roll.load(),b.f.x,b.f.y,b.f.z,
            filtered_pitch_rate,filtered_yaw_rate,filtered_roll_rate,dt,keyboard_axes.load(),aircraft_serial,
            input_throttle.load(),input_brake.load(),
            path_velocity.x,path_velocity.y,path_velocity.z,path_accel.x,path_accel.y,path_accel.z,float(actor_z/100)};
        flight::LogicOutput output{};
        const flight::FlightPath path=flight::flight_path(b,input);
        output.diag[flight::DiagSpeed]=path.speed; output.diag[flight::DiagAoa]=path.aoa;
        output.diag[flight::DiagBeta]=path.beta; output.diag[flight::DiagNz]=path.nz;
        send_telemetry(frame_clock,dt,output,!foreground_is_game(),false);
        return;
    }
    V aim=basis(target_pitch.load(),target_yaw.load(),0).f;
    if(!desired_aim_valid) { desired_aim=aim; desired_aim_valid=true; }
    const bool manual = !foreground_is_game();
    if (recenter_requested.exchange(false) || manual) {
        desired_aim=aim=b.f;
        logic_reset();
        mouse_dx.store(0); mouse_dy.store(0);
    }
    const Basis view=basis(camera_pitch.load(),camera_yaw.load(),camera_roll.load());
    if(resume_center_requested.exchange(false)) {
        // Intersect the camera-centre ray with the HUD's 500m aim sphere.
        V offset{view_offset_x.load(),view_offset_y.load(),view_offset_z.load()};
        const float along=dot(offset,view.f);
        const float t=-along+std::sqrt(std::max(0.0f,along*along+50000.0f*50000.0f-dot(offset,offset)));
        desired_aim=aim=unit(offset+view.f*t);
        logic_reset();
        mouse_dx.store(0); mouse_dy.store(0);
        filtered_pitch_rate=filtered_yaw_rate=filtered_roll_rate=0;
    }
    const bool looking=!manual && (GetAsyncKeyState('F')&0x8000)!=0;
    if(looking && !free_look.held) desired_aim=aim;
    // In a cockpit or nose view the mouse turns the aim by as much less as the view is narrower
    // than the chase view's, so that the ring crosses the screen at the same speed.
    const bool in_near_view=near_view_active.load();
    if(!in_near_view && view_fov.load()>1) chase_fov.store(view_fov.load());
    float sensitivity=config.sensitivity;
    if(in_near_view && chase_fov.load()>1) sensitivity*=std::clamp(view_fov.load()/chase_fov.load(),0.2f,1.0f);
    const float mdx=mouse_dx.exchange(0)*sensitivity, mdy=mouse_dy.exchange(0)*sensitivity;
    // A cockpit or nose view fixed ahead (near_view_camera 0): the mouse works in the aircraft's
    // own frame. F turns the head (left and right 135, up 75, down 25 deg, as a head turns; a world
    // direction had swung the free look with the bank, all round); otherwise the ring, by
    // near_view_aim, stays where it is put on screen (a mouse joystick) or is a world direction,
    // either way inside the box (flung past the narrow view it had been lost off screen).
    const bool cockpit=in_near_view && (config.near_view_camera==0 || config.near_view_camera==3);
    static bool was_cockpit=false, was_attached=false; static float stick_x=0, stick_y=0;
    // The cockpit's mouse joystick, but for a post-stall maneuver (its key held) a direction in the
    // world: flung behind, the nose swings to it and stops (attached, the ring stayed 40 deg off the
    // nose as it turned, and the aircraft looped on, reported 2026-10-07).
    const bool attached=config.near_view_aim && !post_stall_armed.load();
    V camera_target=desired_aim;
    auto head_look=[&]() {   // F in a near view: the head in the aircraft's frame, as far as a head turns
        if(looking) {
            near_look_yaw.store(std::clamp(near_look_yaw.load()+mdx,-135.0f,135.0f));
            near_look_pitch.store(std::clamp(near_look_pitch.load()-mdy,-25.0f,75.0f));
        } else { near_look_yaw.store(0); near_look_pitch.store(0); }
    };
    if(cockpit) {
        free_look.reset();
        const Basis base=basis(near_base_pitch.load(),near_base_yaw.load(),near_base_roll.load());
        head_look();
        const float half_w=std::clamp(view_fov.load(),20.0f,150.0f)*0.5f*rad;
        const float half_h=std::atan(std::tan(half_w)/std::max(view_aspect.load(),0.5f));
        float lim_x=std::atan(config.near_view_box_x*std::tan(half_w))/rad, lim_y=std::atan(config.near_view_box_y*std::tan(half_h))/rad;
        if(config.near_view_camera==3) {   // as far as turns the view to its limit, or keeps the ring on screen
            float cap_x, cap_y; near_view_head_limit(cap_x,cap_y);
            const float f=config.near_view_follow, screen_x=half_w/rad-3, screen_y=half_h/rad-3;
            lim_x=std::min(f>0.01f ? cap_x/f : 1e3f, f<0.99f ? screen_x/(1-f) : 1e3f);
            lim_y=std::min(f>0.01f ? cap_y/f : 1e3f, f<0.99f ? screen_y/(1-f) : 1e3f);
            lim_x=std::clamp(lim_x,2.0f,89.0f); lim_y=std::clamp(lim_y,2.0f,89.0f);
        }
        auto offsets=[&](const V& d,float& x,float& y) {
            x=std::atan2(dot(d,base.r),dot(d,base.f))/rad; y=std::atan2(dot(d,base.u),std::max(std::hypot(dot(d,base.f),dot(d,base.r)),1e-6f))/rad;
        };
        // stick_x/y: where the mouse has the stick; ring_x/y: the ring, reach^expo of it (an ellipse
        // through the limits: a box let a corner reach further than either axis)
        const float expo=attached ? config.near_view_expo : 1.0f;
        auto reach_of=[&](float x,float y) { return std::hypot(x/std::max(lim_x,1e-3f),y/std::max(lim_y,1e-3f)); };
        if(!was_cockpit || (attached && !was_attached)) {   // from the aim as it was: the stick that gives that ring
            offsets(desired_aim,stick_x,stick_y);
            const float r=std::min(reach_of(stick_x,stick_y),1.0f);
            if(r>1e-4f) { const float k=std::pow(r,1/expo-1); stick_x*=k; stick_y*=k; }
        }
        if(attached) {
            if(!looking) { stick_x+=mdx*config.near_view_mouse; stick_y-=mdy*config.near_view_mouse; }
        } else if(!looking) {
            desired_aim=rotate(desired_aim,base.r,mdy*rad);
            desired_aim=rotate(desired_aim,base.u,mdx*rad);
            offsets(desired_aim,stick_x,stick_y);
        }
        const float reach=reach_of(stick_x,stick_y);
        if(reach>1) { stick_x/=reach; stick_y/=reach; }
        const float curve=std::pow(std::min(reach,1.0f),expo-1);
        const float ring_x=stick_x*curve, ring_y=stick_y*curve;
        near_stick_x.store(ring_x); near_stick_y.store(ring_y);
        if(attached || !looking)
            desired_aim=unit(base.f+base.r*std::tan(ring_x*rad)+base.u*(std::tan(ring_y*rad)/std::cos(ring_x*rad)));
        camera_target=desired_aim;
    } else {
        near_look_yaw.store(0); near_look_pitch.store(0);
        camera_target=free_look.step(looking,desired_aim,view,mdx,mdy);
    }
    was_cockpit=cockpit; was_attached=cockpit && attached;
    // (a cockpit's stick keeps steering while the head turns)
    if(!looking || (cockpit && attached)) aim=smooth_direction(aim,desired_aim,dt,0.045f);
    look_pitch.store(flight::pitch(camera_target)); look_yaw.store(flight::yaw(camera_target));
    free_look_held.store(looking);   // (in a cockpit view the head is near_look_yaw/pitch)
    target_pitch.store(flight::pitch(aim)); target_yaw.store(flight::yaw(aim));
    // Lift-vector maneuver paradigm (docs/maneuver-spec.md), see flight_logic.h.
    flight::LogicInput input{pose_pitch.load(),pose_yaw.load(),pose_roll.load(),aim.x,aim.y,aim.z,
        filtered_pitch_rate,filtered_yaw_rate,filtered_roll_rate,dt,keyboard_axes.load(),aircraft_serial,
        input_throttle.load(),input_brake.load(),
        path_velocity.x,path_velocity.y,path_velocity.z,path_accel.x,path_accel.y,path_accel.z,float(actor_z/100),
        post_stall_armed.load()?1:0, cockpit && attached ? 1 : 0};
    flight::LogicOutput output{};
    logic.step(logic_state,&input,&output);
    if(output.event[0]) log_line("%s",output.event);
    command_pitch.store(manual?0:output.pitch*config.pitch_sign);
    command_yaw.store(manual?0:output.yaw*config.yaw_sign);
    command_roll.store(manual?0:output.roll*config.roll_sign);
    if(trace_enabled.load()) log_line("TRACE %s",output.trace);
    send_telemetry(frame_clock,dt,output,manual,true);
    if(now-telemetry_tick>=10000) {
        telemetry_tick=now;
        log_line("guidance %s",output.trace);
    }
}
#include "native_camera.h"

void release_controls() {
    active.store(false); aircraft.store(0); post_stall_armed.store(false);
    command_pitch.store(0); command_yaw.store(0); command_roll.store(0);
    mouse_dx.store(0); mouse_dy.store(0);
    previous_pose_clock=0; free_look.reset(); desired_aim_valid=false; logic_reset();
    actor_valid=false;
    receive_camera(0,0,0,0,0);
}

// Called synchronously on the game thread. Fixed numeric arguments replace
// the pipe queue, sscanf and the camera-target disk snapshot entirely.
void receive_pose(const double (&v)[19]) {
    const uintptr_t address=static_cast<uintptr_t>(v[0]);
    const float pitch=float(v[1]),yaw=float(v[2]),roll=float(v[3]);
    const float view_pitch=float(v[4]),view_yaw=float(v[5]),view_roll=float(v[6]);
    const float fov=float(v[7]),ox=float(v[8]),oy=float(v[9]),oz=float(v[10]);
    const bool paused=v[11]!=0,gazing=v[12]!=0;
    input_throttle.store(float(std::clamp(v[13],-2.0,2.0)));
    input_brake.store(float(std::clamp(v[14],-2.0,2.0)));
    actor_x=v[15]; actor_y=v[16]; actor_z=v[17];
    game_time=v[18];
    if(gaze_active.exchange(gazing)!=gazing) {
        mouse_dx.store(0); mouse_dy.store(0);
        command_pitch.store(0); command_yaw.store(0); command_roll.store(0);
        log_line("gaze: %s",gazing?"native camera and controls; mouse target frozen":"mouse mode resumed");
    }
    const bool was_paused=game_paused.exchange(paused);
    if(was_paused!=paused) {
        mouse_dx.store(0); mouse_dy.store(0);
        if(was_paused) resume_center_requested.store(true);
        else { command_pitch.store(0); command_roll.store(0); command_yaw.store(0); }
        log_line("game pause: %s",paused?"paused; aim frozen":"resumed; centre aim on next pose");
    }
    view_offset_x.store(ox); view_offset_y.store(oy); view_offset_z.store(oz);
    view_fov.store(std::clamp(fov,30.0f,150.0f));
    if (aircraft.load() != static_cast<uintptr_t>(address)) {
        aircraft.store(static_cast<uintptr_t>(address));
        ++aircraft_serial;  // tells the flight logic to identify the new aircraft afresh
        actor_valid=false;
        autopilot_active.store(false);
        roll_reference.store(roll);
        previous_pose_clock = 0;
        desired_aim_valid=false;
        logic_reset();
        telemetry_tick = 0;
        recenter_requested.store(true);
        free_look.reset();
        post_stall_armed.store(false);
        log_line("aircraft acquired 0x%llX pose=(%.3f,%.3f,%.3f) camera=(%.3f,%.3f,%.3f)",
                 static_cast<unsigned long long>(address), pitch, yaw, roll, view_pitch, view_yaw, view_roll);
    }
    pose_pitch.store(pitch);
    pose_yaw.store(yaw);
    pose_roll.store(roll);
    camera_pitch.store(view_pitch);
    camera_yaw.store(view_yaw);
    camera_roll.store(view_roll);
    pose_tick.store(GetTickCount64());
    active.store(true);
    update_commands();
}

// Antialiased ring into the premultiplied BGRA surface, composited over what is there:
// a white line on a soft dark halo, both partly transparent, at sub-pixel position.
// GDI rings had no antialiasing (jagged edges, reported) and were fully opaque.
RECT draw_ring(uint32_t* pixels,int width,int height,float cx,float cy,float radius,
               float line,float halo,float line_alpha,float halo_alpha,uint32_t rgb=0xFFFFFF) {
    const float lr=float((rgb>>16)&0xFF), lg=float((rgb>>8)&0xFF), lb=float(rgb&0xFF);
    const float half=line*0.5f, extent=radius+half+halo+1.5f;
    RECT box{std::max(0,static_cast<int>(std::floor(cx-extent))),std::max(0,static_cast<int>(std::floor(cy-extent))),
             std::min(width,static_cast<int>(std::ceil(cx+extent))+1),std::min(height,static_cast<int>(std::ceil(cy+extent))+1)};
    for(int y=box.top;y<box.bottom;++y)
        for(int x=box.left;x<box.right;++x) {
            const float dx=x+0.5f-cx, dy=y+0.5f-cy;
            const float d=std::abs(std::sqrt(dx*dx+dy*dy)-radius);
            const float line_cover=std::clamp(half+0.5f-d,0.0f,1.0f)*line_alpha;
            float h=std::clamp(1.0f-(d-half)/std::max(halo,0.5f),0.0f,1.0f);
            h=h*h*(3-2*h)*halo_alpha;
            const float a=line_cover+h*(1-line_cover);
            if(a<=0.002f) continue;
            const float dark=12.0f*h*(1-line_cover);  // premultiplied, per channel
            const float cr=lr*line_cover+dark, cg=lg*line_cover+dark, cb=lb*line_cover+dark;
            uint32_t& pixel=pixels[static_cast<size_t>(y)*width+x];
            const float keep=1-a;
            const float da=(pixel>>24)/255.0f;
            const float oa=a+da*keep;
            auto mix=[&](float c,int shift) {
                return static_cast<uint32_t>(std::lround(std::min(c+float((pixel>>shift)&0xFF)*keep,255.0f)));
            };
            const uint32_t A=static_cast<uint32_t>(std::lround(std::min(oa,1.0f)*255));
            pixel=(A<<24)|(mix(cr,16)<<16)|(mix(cg,8)<<8)|mix(cb,0);
        }
    return box;
}

// Draws the HUD and returns the regions drawn (for clearing and dirty-rect presents).
unsigned draw_overlay(HWND window, HDC dc, const RECT& rect, uint32_t* pixels, RECT (&drawn)[5]) {
        drawn[0]={20,35,1100,85};
        unsigned count=1;
        unsigned text_count=1;  // the first regions hold GDI text, which needs its alpha set
        // The DIB was cleared before drawing; a full-screen GDI fill is redundant.
        SetBkMode(dc,TRANSPARENT);
        SetTextColor(dc,RGB(245,245,245));
        char label[160]{};
        snprintf(label,sizeof(label),"MouseFlight 0.2.30 POST-CAMERA %s | aim %.1f / %.1f | camera %.1f / %.1f",
            active.load() && enabled.load()?"ON":"STANDBY",
            target_pitch.load(),target_yaw.load(),camera_pitch.load(),camera_yaw.load());
        TextOutA(dc,28,40,label,static_cast<int>(strlen(label)));
        const int width=rect.right-rect.left, height=rect.bottom-rect.top;
        if(width>0 && height>0) view_aspect.store(float(width)/float(height));
        float x{},y{},bx{},by{};
        bool aim_visible=false, nose_visible=false;
        if (active.load() && enabled.load()) {
            // The camera applied for the frame on screen (native camera: the chase distance
            // behind and above along its own axes, or the game's cockpit or nose position);
            // else the last camera read from the game.
            const bool applied=GetTickCount64()-applied_camera_tick.load()<100;
            const auto view=applied ? flight::basis(applied_camera_pitch.load(),applied_camera_yaw.load(),applied_camera_roll.load())
                                    : flight::basis(camera_pitch.load(),camera_yaw.load(),camera_roll.load());
            const flight::V offset=applied ? flight::V{applied_offset_x.load(),applied_offset_y.load(),applied_offset_z.load()}
                                           : flight::V{view_offset_x.load(),view_offset_y.load(),view_offset_z.load()};
            const auto aim=flight::basis(target_pitch.load(),target_yaw.load(),0).f;
            auto project=[&](flight::V v,float& sx,float& sy) {
                // MouseFlight HUD projects points 500m ahead of the aircraft.
                v=v*50000.0f-offset;
                float depth=flight::dot(v,view.f);
                if(depth<=0.01f) return false;
                float focal=width*0.5f/std::tan(view_fov.load()*0.5f*flight::rad);
                sx=width*0.5f+focal*flight::dot(v,view.r)/depth;
                sy=height*0.5f-focal*flight::dot(v,view.u)/depth;
                return sx>=0 && sy>=0 && sx<width && sy<height;
            };
            aim_visible=project(aim,x,y);
            // Nose direction: a small ring (War Thunder style: the big ring is where to fly,
            // the small one where the nose points). A cross read as a gun sight and was
            // confused with the game's own gun reticle, which it is not.
            nose_visible=project(flight::basis(pose_pitch.load(),pose_yaw.load(),pose_roll.load()).f,bx,by);
            static ULONGLONG draw_report=0;
            if(GetTickCount64()-draw_report>5000) {
                draw_report=GetTickCount64();
                log_line("overlay paint %dx%d target=%s at=(%.0f,%.0f) visible=%d",
                    width,height,aim_visible?"inside":"outside",x,y,IsWindowVisible(window));
            }
            if(!aim_visible) {
                const char* offscreen="TARGET OUTSIDE VIEW";
                TextOutA(dc,28,62,offscreen,static_cast<int>(strlen(offscreen)));
                drawn[count++]={20,58,400,85};
                text_count=count;
            }
        }
        GdiFlush();
        // GDI produces RGB with zero alpha. Set alpha only in the text regions.
        for(unsigned n=0;n<text_count;++n) {
            const RECT& a=drawn[n];
            for(int py=std::max(0L,a.top);py<std::min(rect.bottom,a.bottom);++py)
                for(int px=std::max(0L,a.left);px<std::min(rect.right,a.right);++px) {
                    auto& pixel=pixels[static_cast<size_t>(py)*rect.right+px];
                    if(pixel&0x00FFFFFF) pixel|=0xFF000000;
                }
        }
        // Sizes at 1080p (scaled with height): ring 50 px across, nose ring 14 px.
        const float scale=std::max(0.75f,height/1080.0f);
        if(aim_visible) drawn[count++]=draw_ring(pixels,width,height,x,y,25*scale,2.0f*scale,2.5f*scale,0.8f,0.35f);
        // Post-stall key held: a second ring outside the aim ring, amber (white blended into
        // the sky and the HUD; reported).
        if(aim_visible && post_stall_armed.load())
            drawn[count++]=draw_ring(pixels,width,height,x,y,34*scale,3.0f*scale,2.5f*scale,1.0f,0.45f,0xFFA000);
        if(nose_visible) drawn[count++]=draw_ring(pixels,width,height,bx,by,7*scale,2.0f*scale,2.5f*scale,0.8f,0.35f);
        return count;
}

LRESULT CALLBACK overlay_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
    if (message == WM_PAINT) {
        PAINTSTRUCT paint{};
        BeginPaint(window, &paint);
        EndPaint(window, &paint);
        return 0;
    }
    if (message == WM_ERASEBKGND) return 1;
    return DefWindowProcW(window, message, wparam, lparam);
}

void overlay_loop() {
    WNDCLASSW window_class{};
    window_class.lpfnWndProc = overlay_proc;
    window_class.hInstance = self_module;
    window_class.lpszClassName = L"AC8MouseAimOverlay";
    window_class.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512));
    RegisterClassW(&window_class);
    overlay_window = CreateWindowExW(WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_TOOLWINDOW | WS_EX_NOACTIVATE,
        window_class.lpszClassName, L"AC8 Mouse Aim", WS_POPUP, 0, 0, 100, 100,
        nullptr, nullptr, self_module, nullptr);
    if (!overlay_window) { log_line("overlay creation failed: %lu", GetLastError()); return; }
    // Submit a complete backing surface explicitly instead of relying on WM_PAINT
    // redirection. Do not mix SetLayeredWindowAttributes with UpdateLayeredWindow.
    HDC screen=GetDC(nullptr);
    HDC surface=CreateCompatibleDC(screen);
    HBITMAP bitmap=nullptr;
    uint32_t* pixels=nullptr;
    HGDIOBJ original_bitmap=nullptr;
    SIZE surface_size{};
    HWND owner_window=nullptr;
    ULONGLONG present_report=0;
    bool window_reported = false;
    ULONGLONG window_check=0;
    MSG message{};
    RECT drawn[5]{}; unsigned drawn_count=0;   // regions drawn last time (to clear)
    POINT last_origin{-1,-1}; SIZE last_size{};
    bool full_clear=true;
    while (running.load()) {
        if (!game_window || !IsWindow(game_window) || GetTickCount64()-window_check>1000) {
            HWND found=locate_game_window();
            if(found) game_window=found;
            window_check=GetTickCount64();
        }
        if (hud_enabled.load() && !game_paused.load() && !yielding() && game_window && !IsIconic(game_window)) {
            RECT client{};
            GetClientRect(game_window, &client);
            if (!window_reported) {
                wchar_t title[128]{};
                GetWindowTextW(game_window, title, 128);
                log_line("overlay attached to game window %dx%d", client.right - client.left,
                         client.bottom - client.top);
                window_reported = true;
            }
            POINT origin{client.left, client.top};
            ClientToScreen(game_window, &origin);
            if (owner_window!=game_window) {
                SetWindowLongPtrW(overlay_window,GWLP_HWNDPARENT,reinterpret_cast<LONG_PTR>(game_window));
                owner_window=game_window;
            }
            SIZE size{client.right-client.left,client.bottom-client.top};
            if (size.cx>0 && size.cy>0 && surface &&
                (!bitmap || size.cx!=surface_size.cx || size.cy!=surface_size.cy)) {
                if (bitmap) { SelectObject(surface,original_bitmap); DeleteObject(bitmap); }
                BITMAPINFO info{};
                info.bmiHeader.biSize=sizeof(BITMAPINFOHEADER);
                info.bmiHeader.biWidth=size.cx; info.bmiHeader.biHeight=-size.cy;
                info.bmiHeader.biPlanes=1; info.bmiHeader.biBitCount=32;
                info.bmiHeader.biCompression=BI_RGB;
                bitmap=CreateDIBSection(screen,&info,DIB_RGB_COLORS,reinterpret_cast<void**>(&pixels),nullptr,0);
                if (bitmap) { original_bitmap=SelectObject(surface,bitmap); surface_size=size; }
                full_clear=true;
            }
            if (bitmap && size.cx>0 && size.cy>0) {
                GdiFlush();
                // Clear and present only what changed: a full 3440x1440 clear and submit
                // every time was most of the overlay's cost.
                const bool moved=origin.x!=last_origin.x || origin.y!=last_origin.y ||
                                 size.cx!=last_size.cx || size.cy!=last_size.cy;
                RECT dirty{0,0,0,0};
                auto clip=[&](RECT r) {
                    r.left=std::max(0L,r.left); r.top=std::max(0L,r.top);
                    r.right=std::min(size.cx,r.right); r.bottom=std::min(size.cy,r.bottom);
                    return r;
                };
                auto add=[&](const RECT& r) {
                    if(r.right<=r.left || r.bottom<=r.top) return;
                    if(dirty.right<=dirty.left) dirty=r; else UnionRect(&dirty,&dirty,&r);
                };
                if(full_clear || moved) {
                    memset(pixels,0,static_cast<size_t>(size.cx)*size.cy*4);
                    dirty={0,0,size.cx,size.cy};
                    full_clear=false;
                } else {
                    for(unsigned n=0;n<drawn_count;++n) {
                        const RECT r=clip(drawn[n]);
                        for(LONG y=r.top;y<r.bottom;++y)
                            memset(pixels+static_cast<size_t>(y)*size.cx+r.left,0,static_cast<size_t>(r.right-r.left)*4);
                        add(r);
                    }
                }
                drawn_count=draw_overlay(overlay_window,surface,client,pixels,drawn);
                for(unsigned n=0;n<drawn_count;++n) add(clip(drawn[n]));
                last_origin=origin; last_size=size;
                POINT source{};
                BLENDFUNCTION blend{AC_SRC_OVER,0,255,AC_SRC_ALPHA};
                UPDATELAYEREDWINDOWINFO update{sizeof(update),screen,&origin,&size,surface,&source,0,&blend,ULW_ALPHA,
                    dirty.right>dirty.left ? &dirty : nullptr};
                BOOL presented=UpdateLayeredWindowIndirect(overlay_window,&update);
                DWORD error=presented?0:GetLastError();
                static unsigned presents=0;
                ++presents;
                if (GetTickCount64()-present_report>5000) {
                    const double seconds=present_report?(GetTickCount64()-present_report)/1000.0:0;
                    present_report=GetTickCount64();
                    log_line("overlay surface submit=%d error=%lu size=%ldx%ld rate=%.0f/s",presented,error,size.cx,size.cy,
                             seconds>0?presents/seconds:0.0);
                    presents=0;
                }
            }
            SetWindowPos(overlay_window, HWND_TOPMOST, origin.x, origin.y,
                client.right - client.left, client.bottom - client.top,
                SWP_NOACTIVATE | SWP_SHOWWINDOW);
        } else {
            ShowWindow(overlay_window,SW_HIDE);
            full_clear=true;
        }
        while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&message);
            DispatchMessageW(&message);
        }
        // Redraw once per game frame, right after the camera for it is applied; with the
        // native camera off, about 60 times a second. A 16 ms Sleep paced by the 15.6 ms
        // tick clock redrew only ~32 times a second, out of step with the game (the ring
        // seemed to jump).
        WaitForSingleObject(overlay_frame_event,16);
    }
    if (bitmap) { SelectObject(surface,original_bitmap); DeleteObject(bitmap); }
    if (surface) DeleteDC(surface);
    if (screen) ReleaseDC(nullptr,screen);
    if (overlay_window) DestroyWindow(overlay_window);
    UnregisterClassW(window_class.lpszClassName, self_module);
}

unsigned char* find_hook() {
    auto* base = reinterpret_cast<unsigned char*>(GetModuleHandleW(nullptr));
    auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
    if (!base || dos->e_magic != IMAGE_DOS_SIGNATURE) return nullptr;
    auto* nt = reinterpret_cast<IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE) return nullptr;
    auto* sections = IMAGE_FIRST_SECTION(nt);
    unsigned char* found{};
    for (unsigned index = 0; index < nt->FileHeader.NumberOfSections; ++index) {
        auto& section = sections[index];
        if (!(section.Characteristics & IMAGE_SCN_MEM_EXECUTE)) continue;
        auto* begin = base + section.VirtualAddress;
        const size_t size = section.Misc.VirtualSize;
        for (size_t offset = 0; offset + sizeof(yawCode) <= size; ++offset) {
            if (begin[offset] == yawCode[0] && matchesYawCode(begin + offset, size - offset)) {
                if (found) return nullptr;
                found = begin + offset;
            }
        }
    }
    return found;
}

bool create_absolute_hook(void* target, void* detour, void** trampoline_out) {
    // The matched AC8 function starts with 16 bytes of complete instructions and
    // none of them use RIP-relative addressing. Copying those instructions lets
    // us place the trampoline anywhere in the 64-bit address space instead of
    // relying on MinHook finding a free executable page within +/-2 GB.
    constexpr size_t copied_size = 16;
    constexpr size_t jump_size = 14;
    auto* target_bytes = static_cast<unsigned char*>(target);
    if (memcmp(target_bytes, yawCode, copied_size) != 0) return false;

    auto* trampoline = static_cast<unsigned char*>(VirtualAlloc(
        nullptr, copied_size + jump_size, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    if (!trampoline) return false;

    auto write_absolute_jump = [](unsigned char* destination, const void* address) {
        destination[0] = 0xFF;
        destination[1] = 0x25;
        *reinterpret_cast<uint32_t*>(destination + 2) = 0;
        *reinterpret_cast<uintptr_t*>(destination + 6) = reinterpret_cast<uintptr_t>(address);
    };

    memcpy(trampoline, target_bytes, copied_size);
    write_absolute_jump(trampoline + copied_size, target_bytes + copied_size);
    DWORD previous_trampoline_protection{};
    if (!VirtualProtect(trampoline, copied_size + jump_size, PAGE_EXECUTE_READ,
                        &previous_trampoline_protection)) {
        VirtualFree(trampoline, 0, MEM_RELEASE);
        return false;
    }
    FlushInstructionCache(GetCurrentProcess(), trampoline, copied_size + jump_size);

    DWORD previous_target_protection{};
    if (!VirtualProtect(target_bytes, copied_size, PAGE_EXECUTE_READWRITE,
                        &previous_target_protection)) {
        VirtualFree(trampoline, 0, MEM_RELEASE);
        return false;
    }
    write_absolute_jump(target_bytes, detour);
    memset(target_bytes + jump_size, 0x90, copied_size - jump_size);
    FlushInstructionCache(GetCurrentProcess(), target_bytes, copied_size);
    DWORD ignored{};
    VirtualProtect(target_bytes, copied_size, previous_target_protection, &ignored);

    *trampoline_out = trampoline;
    return true;
}

// Discovery aid (input_probe=1): logs values around the pawn's input block that step
// between input calls the way buttons and triggers do (throttle, brake, high-G), so the
// game's own state is found whatever the binding. Our pitch/yaw/roll slots and the yaw
// double written after the call are skipped. At most 40 lines per second.
void probe_input_block(uintptr_t pawn) {
    constexpr size_t first = 0x2100, last = 0x2500, count = (last - first) / 4;
    static float previous[count];
    static unsigned char previous_bytes[last - first];
    static uintptr_t previous_pawn = 0;
    static ULONGLONG window_start = 0;
    static int lines = 0;
    float now[count];
    unsigned char bytes[last - first];
    __try {
        memcpy(now, reinterpret_cast<const void*>(pawn + first), sizeof(now));
        memcpy(bytes, reinterpret_cast<const void*>(pawn + first), sizeof(bytes));
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return;
    }
    if (pawn != previous_pawn) {
        memcpy(previous, now, sizeof(now));
        memcpy(previous_bytes, bytes, sizeof(bytes));
        previous_pawn = pawn;
        log_line("input probe armed pawn=%p range=+0x%zx..+0x%zx", reinterpret_cast<void*>(pawn), first, last);
        return;
    }
    const ULONGLONG tick = GetTickCount64();
    if (tick - window_start >= 1000) { window_start = tick; lines = 0; }
    auto skipped = [](size_t offset) {
        return (offset >= 0x2268 && offset < 0x2274) || (offset >= 0x22a8 && offset < 0x22b0);
    };
    for (size_t i = 0; i < count; ++i) {
        const size_t offset = first + i * 4;
        const float a = previous[i], b = now[i];
        previous[i] = b;
        if (skipped(offset) || !std::isfinite(a) || !std::isfinite(b)) continue;
        if (std::abs(a) > 1.5f || std::abs(b) > 1.5f || std::abs(b - a) < 0.3f) continue;
        if (lines++ < 40) log_line("input probe float +0x%zx %.3f -> %.3f", offset, a, b);
    }
    for (size_t i = 0; i < sizeof(bytes); ++i) {
        const unsigned char a = previous_bytes[i], b = bytes[i];
        previous_bytes[i] = b;
        if (a == b || a > 1 || b > 1 || skipped(first + i)) continue;
        if (lines++ < 40) log_line("input probe byte +0x%zx %u -> %u", first + i, a, b);
    }
}

uintptr_t __fastcall process_input(unsigned char* state, unsigned char* context) {
    // Enemy/non-player invocations do not query the keyboard or foreground.
    const uintptr_t pawn = aircraft.load();
    if (!pawn || reinterpret_cast<uintptr_t>(state)!=pawn+0x22a0) {
        if(perf_enabled.load()) ++perf_other_inputs;
        return original(state,context);
    }
    if(perf_enabled.load()) ++perf_player_inputs;
    if(config.input_probe) probe_input_block(pawn);
    if(config.telemetry_port>0) {
        __try {
            const float* axes = reinterpret_cast<const float*>(pawn + 0x2268);
            raw_axis_pitch.store(axes[config.pitch_slot]); raw_axis_yaw.store(axes[1]); raw_axis_roll.store(axes[config.roll_slot]);
        } __except (EXCEPTION_EXECUTE_HANDLER) {}
    }
    if(!active.load() || GetTickCount64()-pose_tick.load()>1000 || !enabled.load() ||
       game_paused.load() || yielding() || !foreground_is_game()) return original(state,context);
    // Manual pitch also suspends automatic roll, matching MouseFlight maneuvers.
    // Preserve AC's native keyboard values, including opposing-key handling.
    auto held=[](int key) { return (GetAsyncKeyState(key)&0x8000)!=0; };
    const bool keyboard_pitch=held('W') || held('S');
    const bool keyboard_roll=keyboard_pitch || held('A') || held('D');
    const bool keyboard_yaw=held('Q') || held('E');
    keyboard_axes.store((keyboard_pitch?1:0)|(keyboard_roll?2:0)|(keyboard_yaw?4:0));
    bool override_input = true; // Lifecycle/foreground already checked above.
    if (override_input && reinterpret_cast<uintptr_t>(state) == pawn + 0x22a0) {
        __try {
            if (*reinterpret_cast<uintptr_t*>(context) != pawn + 0x2268 ||
                *reinterpret_cast<uintptr_t*>(context + 8) != pawn + 0x2c28) {
                override_input = false;
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            override_input = false;
        }
    }
    if (override_input && reinterpret_cast<uintptr_t>(state) == pawn + 0x22a0) {
        __try {
            float* axes = reinterpret_cast<float*>(pawn + 0x2268);
            if(!keyboard_pitch) axes[config.pitch_slot] = command_pitch.load();
            if(!keyboard_yaw) axes[1] = command_yaw.load();
            if(!keyboard_roll) axes[config.roll_slot] = command_roll.load();
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            active.store(false);
            override_input = false;
        }
    }
    uintptr_t result = original(state, context);
    if (override_input && reinterpret_cast<uintptr_t>(state) == pawn + 0x22a0) {
        __try {
            // AC8's stock yaw path turns every non-zero axis value into full yaw.
            // Preserve only the proportional input target; downstream response remains stock.
            if(!keyboard_yaw)
                *reinterpret_cast<double*>(state + 8) = static_cast<double>(command_yaw.load());
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            active.store(false);
        }
    }
    return result;
}

bool prepare_hook() {
    if (hook_created) return true;
    hook_address = find_hook();
    if (!hook_address) {
        log_line("error input processor signature missing or ambiguous");
        return false;
    }
    MH_STATUS result = MH_Initialize();
    if (result != MH_OK && result != MH_ERROR_ALREADY_INITIALIZED) return false;
    result = MH_CreateHook(hook_address, reinterpret_cast<void*>(&process_input), reinterpret_cast<void**>(&original));
    if (result == MH_ERROR_MEMORY_ALLOC) {
        if (create_absolute_hook(hook_address, reinterpret_cast<void*>(&process_input),
                                 reinterpret_cast<void**>(&original))) {
            hook_created = true;
            log_line("input hook installed with absolute trampoline fallback");
            return true;
        }
        log_line("error absolute hook fallback: win32=%lu", GetLastError());
    }
    if (result != MH_OK) {
        log_line("error create hook: %s", MH_StatusToString(result));
        return false;
    }
    result = MH_EnableHook(hook_address);
    if (result != MH_OK) {
        log_line("error enable hook: %s", MH_StatusToString(result));
        return false;
    }
    hook_created = true;
    return true;
}

bool file_write_time(const wchar_t* path,FILETIME& time) {
    WIN32_FILE_ATTRIBUTE_DATA data{};
    if(!GetFileAttributesExW(path,GetFileExInfoStandard,&data)) return false;
    time=data.ftLastWriteTime;
    return true;
}
void remove_stale_logic_copies() {
    wchar_t pattern[MAX_PATH]{},path[MAX_PATH]{};
    swprintf_s(pattern,L"%s\\ac8_flight_logic.live*.dll",module_folder);
    WIN32_FIND_DATAW found{};
    HANDLE search=FindFirstFileW(pattern,&found);
    if(search==INVALID_HANDLE_VALUE) return;
    do {
        swprintf_s(path,L"%s\\%s",module_folder,found.cFileName);
        DeleteFileW(path);
    } while(FindNextFileW(search,&found));
    FindClose(search);
}
// Load a copy so the original can be rebuilt while the game holds the module.
void load_logic_override(const wchar_t* path) {
    wchar_t live[MAX_PATH]{};
    swprintf_s(live,L"%s\\ac8_flight_logic.live%d.dll",module_folder,++logic_generation);
    if(!CopyFileW(path,live,FALSE)) { log_line("flight logic: copy failed (%lu)",GetLastError()); return; }
    HMODULE module=LoadLibraryW(live);
    LogicApi api{};
    if(module) {
        api.abi=reinterpret_cast<decltype(api.abi)>(GetProcAddress(module,"ac8_logic_abi"));
        api.configure=reinterpret_cast<decltype(api.configure)>(GetProcAddress(module,"ac8_logic_configure"));
        api.reset=reinterpret_cast<decltype(api.reset)>(GetProcAddress(module,"ac8_logic_reset"));
        api.step=reinterpret_cast<decltype(api.step)>(GetProcAddress(module,"ac8_logic_step"));
    }
    unsigned input_size=0,output_size=0,state_size=0;
    if(!module || !api.abi || !api.configure || !api.reset || !api.step ||
       api.abi(&input_size,&output_size,&state_size)!=flight::logic_abi ||
       input_size!=sizeof(flight::LogicInput) || output_size!=sizeof(flight::LogicOutput) ||
       state_size>sizeof(logic_state)) {
        log_line("flight logic: generation %d rejected (missing exports or ABI mismatch); keeping current logic",logic_generation);
        if(module) FreeLibrary(module);
        DeleteFileW(live);
        return;
    }
    api.configure(config_path);
    api.reset(logic_state);
    if(logic_module) { FreeLibrary(logic_module); DeleteFileW(logic_live_path); }
    logic_module=module;
    wcscpy_s(logic_live_path,live);
    logic=api;
    log_line("flight logic: hot-loaded generation %d",logic_generation);
}
// Development loop: config.ini and ac8_flight_logic.dll changes apply without a restart.
void poll_dev_files() {
    const ULONGLONG now=GetTickCount64();
    if(now<dev_poll_at) return;
    dev_poll_at=now+500;
    FILETIME time{};
    if(file_write_time(config_path,time) && CompareFileTime(&time,&config_file_time)!=0) {
        config_file_time=time;
        load_config();
        logic.configure(config_path);
        log_line("configuration hot-reloaded");
    }
    wchar_t path[MAX_PATH]{};
    swprintf_s(path,L"%s\\ac8_flight_logic.dll",module_folder);
    if(file_write_time(path,time) && CompareFileTime(&time,&logic_file_time)!=0) {
        // Let the writer finish: require the file to be at least 0.3 s old.
        FILETIME system{};
        GetSystemTimeAsFileTime(&system);
        ULARGE_INTEGER written{},current{};
        written.LowPart=time.dwLowDateTime; written.HighPart=time.dwHighDateTime;
        current.LowPart=system.dwLowDateTime; current.HighPart=system.dwHighDateTime;
        if(current.QuadPart>written.QuadPart+3000000ull) {
            logic_file_time=time;
            load_logic_override(path);
        }
    }
}

bool offline_authorized() {
    wchar_t value[16]{};
    return GetEnvironmentVariableW(L"EOS_USE_ANTICHEATCLIENTNULL", value, 16) > 0 && wcscmp(value, L"1") == 0;
}
bool bridge_verified=false;
DWORD bridge_thread=0;
std::atomic<bool> reload_requested{false};
bool on_bridge_thread() { return bridge_thread && bridge_thread==GetCurrentThreadId(); }
} // namespace

extern "C" __declspec(dllexport) int ac8_mouseaim_start(lua_State* state) {
    init_paths();
    if(!logger_started.exchange(true)) {
        QueryPerformanceFrequency(&perf_frequency);
        std::thread(logger_loop).detach();
    }
    if(!bridge_verified) bridge_verified=compatible_lua_runtime();
    if(!bridge_verified) { log_line("bridge refused: UE4SS runtime hash mismatch"); return 0; }
    LuaView lua(state);
    double handshake[2]{};
    if(!read_numbers(lua,handshake) || handshake[0]!=1729 || handshake[1]!=0.125) return 0;
    if(running.load()) { lua.set_number(30); return 1; }
    if (!offline_authorized()) {
        log_line("inactive: offline launch marker missing; multiplayer-safe refusal");
        return 0;
    }
    load_config();
    file_write_time(config_path,config_file_time);
    remove_stale_logic_copies();
    logic.configure(config_path);
    logic_reset();
    if (!prepare_hook()) return 0;
    install_native_camera();
    if (prepare_raw_input_capture()) {
        log_line("mouse capture attached to AC8 raw input");
    } else {
        log_line("warning raw input hook unavailable; using DirectInput fallback");
    }
    running.store(true);
    std::thread(mouse_loop).detach();
    if(!overlay_frame_event) overlay_frame_event=CreateEventW(nullptr,FALSE,FALSE,nullptr);
    std::thread(overlay_loop).detach();
    log_line("ready: F8 toggle, F9 recenter; RMB reserved for game actions");
    log_line("0.2.30 direct numeric bridge; realtime file/pipe transport removed; F5 performance counters");
    lua.set_number(30);
    return 1;
}

extern "C" __declspec(dllexport) int ac8_mouseaim_reload(void*) {
    reload_requested.store(true);
    return 0;
}

extern "C" __declspec(dllexport) int ac8_mouseaim_begin(void*) {
    if(!running.load()) return 0;
    if(!bridge_thread) bridge_thread=GetCurrentThreadId();
    if(!on_bridge_thread()) return 0;
    if(reload_requested.exchange(false)) {
        load_config(); logic.configure(config_path);
        recenter_requested.store(true); log_line("configuration reloaded");
    }
    poll_dev_files();
    if(perf_enabled.load()) {
        script_start=perf_clock();
        if(previous_frame_start) {
            const auto gap=perf_us(script_start-previous_frame_start);
            perf_frame_gap.add(gap);
            if(gap>50000) ++perf_hitches;
        }
        previous_frame_start=script_start;
    } else { script_start=0; previous_frame_start=0; }
    return 0;
}

extern "C" __declspec(dllexport) int ac8_mouseaim_frame(lua_State* state) {
    if(!running.load() || !on_bridge_thread()) return 0;
    PerfSpan timing(perf_bridge);
    LuaView lua(state); double v[19]{};
    if(!read_numbers(lua,v) || !live_pointer_number(v[0])) { release_controls(); return 0; }
    for(size_t i=1;i<11;++i) if(std::abs(v[i])>1e12) { release_controls(); return 0; }
    if((v[11]!=0 && v[11]!=1) || (v[12]!=0 && v[12]!=1)) { release_controls(); return 0; }
    receive_pose(v);
    const bool on=enabled.load() && !game_paused.load() && !yielding() && foreground_is_game();
    lua.set_number(on?1:0); lua.set_number(look_pitch.load()); lua.set_number(look_yaw.load());
    lua.set_number(config.camera_follow);
    return 4;
}

// Forwards one JSON text from Lua (contacts: other aircraft) to the telemetry stream.
// Returns 1 when sent, 0 when telemetry is off (Lua then backs off).
extern "C" __declspec(dllexport) int ac8_mouseaim_send(lua_State* state) {
    if(!running.load() || !on_bridge_thread()) return 0;
    LuaView lua(state);
    if(lua.get_stack_size()!=1 || !lua.is_string(1)) return 0;
    const std::string_view text=lua.get_string(1);
    const bool sent=config.telemetry_port>0 && text.size()<60000 &&
        telemetry_send(config.telemetry_port,text.data(),static_cast<int>(text.size()));
    lua.set_number(sent?1:0);
    return 1;
}

extern "C" __declspec(dllexport) int ac8_mouseaim_camera(lua_State* state) {
    if(!running.load() || !on_bridge_thread()) return 0;
    LuaView lua(state); double v[5]{};
    if(!read_numbers(lua,v) ||
       std::abs(v[2])>1e9 || std::abs(v[3])>1e9 || std::abs(v[4])>1e9 ||
       (v[0]!=0 && (!live_pointer_number(v[0]) || !live_pointer_number(v[1]) ||
                    static_cast<uintptr_t>(v[1])!=aircraft.load()))) {
        release_controls(); return 0;
    }
    const bool accepted=receive_camera(static_cast<uintptr_t>(v[0]),v[0]?static_cast<uintptr_t>(v[1]):0,v[2],v[3],v[4]);
    if(script_start) { perf_script.add(perf_us(perf_clock()-script_start)); script_start=0; }
    lua.set_number(accepted?1:0);
    return 1;
}

extern "C" __declspec(dllexport) int ac8_mouseaim_release(void*) {
    if(running.load() && on_bridge_thread()) { release_controls(); script_start=0; }
    return 0;
}

extern "C" __declspec(dllexport) int ac8_mouseaim_perf(void*) {
    if(running.load()) {
        const bool enabled_now=!perf_enabled.load(); perf_enabled.store(enabled_now);
        if(!enabled_now) perf_flush.store(true);
        log_line("PERF capture %s (10-second summaries, no per-frame disk writes)",enabled_now?"ON":"OFF");
    }
    return 0;
}

BOOL WINAPI DllMain(HINSTANCE module, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        self_module = module;
        DisableThreadLibraryCalls(module);
    }
    return TRUE;
}
