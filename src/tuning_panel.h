// In-game tuning panel: [keys] tuning (F3) opens a list of config.ini values over the game, on two
// pages: Settings (values, changed with the arrow keys while flying) and Keys (the mod's keys:
// Enter, then the new key). Each change is written to config.ini, which the mod reloads within
// 0.5 s (poll_dev_files); config.ini stays the only place values live, and players need not edit
// it. Keys the panel uses are kept from the game while it is open (its window procedure), so the
// arrows do not also act in the game; while a key is being recorded, every key is kept from it.
// Its texts are the language's (localization.h): "item.<key>" a name, "item.<key>.desc" what it
// does, "group.<id>" a heading.
struct PanelItem {
    const char* group;              // the id of a heading before this item, or nullptr
    const wchar_t* group_en;
    const wchar_t* section;         // config.ini section
    const char* key;
    const wchar_t* name;            // English
    const wchar_t* label;           // English: what it does
    float min, max, step;           // Shift: a fifth of the step
    bool integer;
    float fallback;                 // shown when config.ini has no value
    int Config::Keys::* bind;       // a key of [keys] (Keys page), not a value
    bool language;                  // the language: auto or a language file (the value is its place in that list)
};
PanelItem value_item(const char* group,const wchar_t* group_en,const wchar_t* section,const char* key,const wchar_t* name,
                     const wchar_t* label,float min,float max,float step,bool integer,float fallback) {
    return {group,group_en,section,key,name,label,min,max,step,integer,fallback,nullptr,false};
}
PanelItem key_item(const char* group,const wchar_t* group_en,const char* key,const wchar_t* name,const wchar_t* label,
                   int Config::Keys::* bind) {
    return {group,group_en,L"keys",key,name,label,0,255,1,true,float(Config::Keys{}.*bind),bind,false};
}
const PanelItem panel_items[]={
    value_item("mouse_camera",L"Mouse and Chase Camera",L"control","sensitivity",L"Mouse Sensitivity",L"Sets how far the aim turns per mouse count (deg). Lower in the cockpit and nose views, in proportion to their field of view.",0.01f,1.0f,0.005f,false,0.10f),
    value_item(nullptr,nullptr,L"control","camera_follow",L"Camera Tracking Speed",L"Sets how fast the chase camera turns toward the aim (1/s). Higher keeps the camera closer to the aim.",1,60,1,false,12),
    value_item(nullptr,nullptr,L"control","camera_distance",L"Camera Distance",L"Sets the chase camera's distance behind the aircraft (m). 0 uses the game's own distance and height.",0,100,1,false,30),
    value_item(nullptr,nullptr,L"control","camera_height",L"Camera Height",L"Sets the chase camera's height above the aircraft (m). Not used when Camera Distance is 0.",-20,30,0.5f,false,6),
    value_item(nullptr,nullptr,L"control","camera_fov_add",L"Field of View Offset",L"Degrees added to the game's field of view in the chase view. The game's changes with speed are kept.",-40,40,1,false,0),
    value_item("near_views",L"Cockpit and Nose Views",L"control","near_view_camera",L"View Mode",L"Sets the camera in the cockpit and nose views. 0: fixed ahead  1: turns to the aim  2: turns once the aim passes the edge  3: fixed ahead, the head turns toward the ring",0,3,1,true,3),
    value_item(nullptr,nullptr,L"control","near_view_follow",L"Head Turn Ratio",L"Mode 3 only. Sets how far the view turns toward the ring, as a share of the ring's offset (0-1).",0,1,0.05f,false,0.6f),
    value_item(nullptr,nullptr,L"control","near_view_hud",L"Head Turn Limit",L"Mode 3 only. The view turns at most half the screen plus this many degrees, so the HUD's edge stays on screen.",0,40,1,false,8),
    value_item(nullptr,nullptr,L"control","near_view_mouse",L"Mouse Multiplier",L"Modes 0 and 3 only. Sets the mouse movement multiplier for the cockpit mouse stick.",0.1f,3,0.05f,false,1),
    value_item(nullptr,nullptr,L"control","near_view_expo",L"Ring Curve",L"Modes 0 and 3 only. The ring moves as the stick deflection to this power: 1 is linear, higher is finer near the nose.",1,4,0.1f,false,1.5f),
    value_item(nullptr,nullptr,L"control","near_view_inertia",L"View Delay",L"Sets how far the view lags the aircraft's attitude (s). 0 is no delay.",0,0.5f,0.01f,false,0.06f),
    value_item(nullptr,nullptr,L"control","near_view_level",L"View Leveling",L"Sets how much of the view's roll is corrected toward level. 0: rolls with the aircraft  1: stays level",0,1,0.05f,false,0),
    value_item(nullptr,nullptr,L"control","near_view_fov_add",L"Field of View Offset",L"Degrees added to the game's field of view in the cockpit and nose views.",-40,40,1,false,0),
    // What sets the turn (closed-loop simulation, measured plant, 2026-10-07): pitch_gain is what limits
    // the pitch (1.5 to 4: up 20 deg reached in 2.4 to 1.4 s; past 4 the stick chatters near the aim;
    // pitch_brake changed nothing at all); roll_brake slows a big roll's stop when lowered (right 30
    // deg 2.8 s at 0.45, 4.1 s at 0.2) and changes little when raised (the roll rate is the limit).
    value_item("flight",L"Flight Control (near the aim only)",L"tuning","pitch_gain",L"Pitch Gain",L"Sets the pitch speed near the aim: wanted pitch rate = this x the angle left (1/s). Higher arrives faster; too high corrects back and forth near the aim.",1,4,0.1f,false,2.5f),
    value_item(nullptr,nullptr,L"tuning","roll_brake",L"Roll Deceleration",L"Sets the deceleration planned for stopping a roll, as a share of the aircraft's roll acceleration. Lower starts slowing earlier.",0.2f,1,0.05f,false,0.45f),
    value_item(nullptr,nullptr,L"tuning","level_per_deg",L"Level Recovery",L"Near the aim, bank is limited to this x the angle left. Lower returns the wings to level sooner.",2,30,1,false,10),
    value_item(nullptr,nullptr,L"tuning","lead_gain",L"Moving Aim Correction",L"Sets the share of a moving aim's angular rate added to the turn. 0 is off.",0,2,0.05f,false,1.0f),
    value_item(nullptr,nullptr,L"tuning","lead_filter",L"Correction Smoothing",L"Sets the smoothing time used to estimate the aim's angular rate (s).",0.05f,1.5f,0.05f,false,0.4f),
    value_item(nullptr,nullptr,L"tuning","highg_full_from",L"High-G Full Pull Angle",L"During a High-G Turn with the aim more than this far off, the stick is pulled fully without slowing early (deg). 0 is off.",0,45,1,false,8),
    PanelItem{"display",L"Display",L"control","language",L"Language",L"Sets the language of this panel and the mod's on-screen text. Auto: the game's UI language.",0,1,1,true,0,nullptr,true},
    value_item(nullptr,nullptr,L"control","status_line",L"Status Display",L"Sets whether the version, state and angles are shown at the top left of the screen.",0,1,1,true,0),
    key_item("keys_flight",L"Flight","toggle",L"Mouse Aim ON/OFF",L"Sets the key that turns mouse aim on and off. When off, the game's own controls fly the aircraft.",&Config::Keys::toggle),
    key_item(nullptr,nullptr,"recenter",L"Reset Aim",L"Sets the key that returns the aim to where the nose points.",&Config::Keys::recenter),
    key_item(nullptr,nullptr,"free_look",L"Free Look",L"Sets the key that, while held, lets the mouse move only the camera. The aim is kept.",&Config::Keys::free_look),
    key_item(nullptr,nullptr,"hud",L"Mod HUD ON/OFF",L"Sets the key that shows or hides the mod's rings. Flight control is unaffected.",&Config::Keys::hud),
    key_item(nullptr,nullptr,"post_stall",L"Post-Stall Assist",L"Sets the key for post-stall assist. While it is held, a High-G Turn below 500 km/h holds the pitch neutral for about 0.04 s, then pulls fully, meeting the game's post-stall condition.",&Config::Keys::post_stall),
    key_item("keys_panel",L"Settings Panel","tuning",L"Settings Panel",L"Sets the key that opens and closes this panel.",&Config::Keys::tuning),
    key_item(nullptr,nullptr,"tuning_up",L"Previous Item",L"Sets the key that selects the previous item.",&Config::Keys::tuning_up),
    key_item(nullptr,nullptr,"tuning_down",L"Next Item",L"Sets the key that selects the next item.",&Config::Keys::tuning_down),
    key_item(nullptr,nullptr,"tuning_less",L"Decrease",L"Sets the key that decreases the value.",&Config::Keys::tuning_less),
    key_item(nullptr,nullptr,"tuning_more",L"Increase",L"Sets the key that increases the value.",&Config::Keys::tuning_more),
    key_item(nullptr,nullptr,"tuning_fine",L"Fine Adjustment",L"Sets the key that, while held, adjusts in steps of 1/5.",&Config::Keys::tuning_fine),
    key_item(nullptr,nullptr,"tuning_undo",L"Restore",L"Sets the key that restores the value from when the panel was opened.",&Config::Keys::tuning_undo),
    key_item(nullptr,nullptr,"tuning_bind",L"Change Key",L"Sets the key that records a new key for the selected item on the Key Config page.",&Config::Keys::tuning_bind),
    key_item("keys_diagnostics",L"Diagnostics","reload",L"Reload Settings",L"Sets the key that reads config.ini again.",&Config::Keys::reload),
    key_item(nullptr,nullptr,"trace",L"Flight Log",L"Sets the key that turns the per-frame flight log on and off.",&Config::Keys::trace),
    key_item(nullptr,nullptr,"perf",L"Performance Statistics",L"Sets the key that turns performance statistics on and off (written to the log every 10 s).",&Config::Keys::perf),
    key_item(nullptr,nullptr,"camera_probe",L"Camera State Capture",L"Sets the key that starts or stops capturing the camera state (up to 180 s).",&Config::Keys::camera_probe),
    key_item(nullptr,nullptr,"hangar_specs",L"Hangar: Aircraft Data",L"Sets the key that, in the hangar, writes the aircraft's parameter tables to a file.",&Config::Keys::hangar_specs),
    key_item(nullptr,nullptr,"hangar_geometry",L"Hangar: Aircraft Geometry",L"Sets the key that, in the hangar, writes the aircraft's geometry to a file.",&Config::Keys::hangar_geometry),
};
constexpr int panel_count=int(sizeof(panel_items)/sizeof(panel_items[0]));
int item_page(int i) { return panel_items[i].bind ? 1 : 0; }
const char* const page_names[2]={"Settings","Keys"};   // for the log
// The language item's choices: auto, then each language file.
std::vector<std::string> language_options() {
    std::vector<std::string> options{"auto"};
    for(const auto& code:languages_available()) options.push_back(code);
    return options;
}
float item_max(const PanelItem& item) { return item.language ? float(language_options().size()-1) : item.max; }
struct PanelState {
    int page=0;
    int selected=0;          // an item of the page, or -1: the page row (Left/Right turn the page)
    float value[panel_count]{}, was[panel_count]{};   // was: when the panel was opened (Backspace)
    bool pending[panel_count]{};                       // changed, not yet written
    ULONGLONG write_at=0;
    bool recording=false;    // waiting for the selected key item's new key
    bool held_then[256]{};   // keys already down when the recording began (taken once released)
};
std::mutex panel_mutex;
PanelState panel;
std::atomic<bool> panel_open{false};
std::atomic<bool> panel_recording{false};   // every key kept from the game
// Open and on screen (the overlay hides while paused, in gaze or autopilot, or with F7): only
// then are its keys taken from the game (the pause menu keeps its arrows).
bool panel_active() { return panel_open.load() && hud_enabled.load() && !game_paused.load() && !yielding(); }

int step_decimals(float step) {
    int d=0;
    for(float s=step;d<4 && std::abs(s-std::round(s))>1e-4f;s*=10) ++d;
    return d;
}
// The value as config.ini holds it: no trailing zeros; a key by its name; the language by its code.
std::string panel_text(const PanelItem& item,float value) {
    if(item.bind) return key_label(int(std::lround(value)));
    if(item.language) {
        const auto options=language_options();
        const size_t at=size_t(std::clamp(int(std::lround(value)),0,int(options.size())-1));
        return options[at];
    }
    char text[32]{};
    if(item.integer) snprintf(text,sizeof(text),"%d",int(std::lround(value)));
    else {
        snprintf(text,sizeof(text),"%.*f",step_decimals(item.step/5),value);
        if(strchr(text,'.')) {
            size_t n=strlen(text);
            while(n>0 && text[n-1]=='0') text[--n]=0;
            if(n>0 && text[n-1]=='.') text[--n]=0;
        }
    }
    return text;
}
// As the panel shows it: an on/off value as such, the language by its name.
std::wstring panel_shown(const PanelItem& item,float value) {
    if(item.language) {
        const std::string code=panel_text(item,value);
        if(code=="auto") return tr("panel.auto",L"Auto (game)");
        if(code=="en") return L"English";
        if(code=="zh-Hans") return L"\x7b80\x4f53\x4e2d\x6587";
        return widen(code);
    }
    if(!item.bind && item.integer && item.min==0 && item.max==1)
        return value>0.5f ? tr("panel.on",L"ON") : tr("panel.off",L"OFF");
    if(item.bind && value<0.5f) return tr("panel.none",L"none");
    return widen(panel_text(item,value));
}
float panel_read(const PanelItem& item) {
    wchar_t key[64]{}, text[64]{};
    MultiByteToWideChar(CP_ACP,0,item.key,-1,key,64);
    if(item.bind) return float(read_key(item.section,key,int(item.fallback)));
    GetPrivateProfileStringW(item.section,key,L"",text,64,config_path);
    if(item.language) {
        const auto options=language_options();
        const std::string code=wide_to_utf8(text);
        for(size_t i=0;i<options.size();++i) if(_stricmp(options[i].c_str(),code.c_str())==0) return float(i);
        return 0;
    }
    wchar_t* end{};
    const float parsed=wcstof(text,&end);
    return end!=text && std::isfinite(parsed) ? parsed : item.fallback;
}
std::wstring item_name(const PanelItem& item) { return tr((std::string("item.")+item.key).c_str(),item.name); }
std::wstring item_label(const PanelItem& item) { return tr((std::string("item.")+item.key+".desc").c_str(),item.label); }
std::wstring group_name(const PanelItem& item) { return tr((std::string("group.")+item.group).c_str(),item.group_en); }

// Replaces key's value in [section] of config.ini, keeping every other byte (comments, order,
// line ends); a key not there is added at the end of its section. Written to a copy that then
// replaces the file, so a reload never reads half a file.
bool write_config_value(const wchar_t* section_name,const char* key,const std::string& value) {
    HANDLE file=CreateFileW(config_path,GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,
                            nullptr,OPEN_EXISTING,0,nullptr);
    if(file==INVALID_HANDLE_VALUE) return false;
    std::string text;
    char buffer[8192]; DWORD got=0;
    while(ReadFile(file,buffer,sizeof(buffer),&got,nullptr) && got>0) text.append(buffer,got);
    CloseHandle(file);
    char section[64]{};
    WideCharToMultiByte(CP_ACP,0,section_name,-1,section,64,nullptr,nullptr);
    const std::string eol=text.find("\r\n")!=std::string::npos ? "\r\n" : "\n";
    auto trimmed=[](std::string s) {
        while(!s.empty() && (s.back()=='\r' || isspace(static_cast<unsigned char>(s.back())))) s.pop_back();
        size_t i=0; while(i<s.size() && isspace(static_cast<unsigned char>(s[i]))) ++i;
        return s.substr(i);
    };
    bool in_section=false, done=false;
    size_t section_end=std::string::npos;   // where a missing key goes: after the section's last value line
    for(size_t start=0;start<text.size() && !done;) {
        size_t end=text.find('\n',start);
        const size_t next=end==std::string::npos ? text.size() : end+1;
        if(end==std::string::npos) end=text.size();
        const std::string line=trimmed(text.substr(start,end-start));
        if(!line.empty() && line[0]=='[') {
            if(in_section) break;
            in_section=_stricmp(line.c_str(),(std::string("[")+section+"]").c_str())==0;
            if(in_section) section_end=next;
        } else if(in_section && !line.empty() && line[0]!=';') {
            section_end=next;
            const size_t equals=line.find('=');
            if(equals!=std::string::npos && _stricmp(trimmed(line.substr(0,equals)).c_str(),key)==0) {
                const size_t at=text.find('=',start)+1;
                size_t stop=end; if(stop>at && text[stop-1]=='\r') --stop;
                text.replace(at,stop-at,value);
                done=true;
            }
        }
        start=next;
    }
    if(!done) {
        const std::string line=std::string(key)+"="+value+eol;
        if(section_end==std::string::npos) text+=eol+"["+section+"]"+eol+line;
        else text.insert(section_end,line);
    }
    wchar_t temporary[MAX_PATH]{};
    swprintf_s(temporary,L"%s.panel",config_path);
    file=CreateFileW(temporary,GENERIC_WRITE,0,nullptr,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr);
    if(file==INVALID_HANDLE_VALUE) return false;
    DWORD written=0;
    const bool ok=WriteFile(file,text.data(),DWORD(text.size()),&written,nullptr) && written==text.size();
    CloseHandle(file);
    if(ok) for(int attempt=0;attempt<10;++attempt) {   // a reload may have the file open for a moment
        if(MoveFileExW(temporary,config_path,MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH)) return true;
        Sleep(10);
    }
    DeleteFileW(temporary);
    return false;
}

void panel_flush(bool now) {   // with panel_mutex held
    if(!now && GetTickCount64()<panel.write_at) return;
    for(int i=0;i<panel_count;++i) {
        if(!panel.pending[i]) continue;
        const std::string text=panel_text(panel_items[i],panel.value[i]);
        if(write_config_value(panel_items[i].section,panel_items[i].key,text)) {
            panel.pending[i]=false;
            dev_poll_at=0;   // read it again on the next frame, not up to half a second later
            log_line("tuning panel: %s=%s",panel_items[i].key,text.c_str());
        } else {
            log_line("tuning panel: writing %s to config.ini failed (%lu)",panel_items[i].key,GetLastError());
            panel.pending[i]=false;
        }
    }
}

// A key press, then repeats while held (after 0.35 s, every 70 ms).
struct KeyRepeat {
    bool down=false; ULONGLONG next=0;
    bool fire(int vk) {
        const bool held=(GetAsyncKeyState(vk)&0x8000)!=0;
        const ULONGLONG now=GetTickCount64();
        bool fired=false;
        if(held && !down) { fired=true; next=now+350; }
        else if(held && now>=next) { fired=true; next=now+70; }
        down=held;
        return fired;
    }
};

// The items of a page, in order.
int page_items(int page,int (&out)[panel_count]) {
    int n=0;
    for(int i=0;i<panel_count;++i) if(item_page(i)==page) out[n++]=i;
    return n;
}
// A recording's key: one going down that was not already down when it began; Esc cancels (-1),
// Delete is no key (0). The left and right mouse buttons and the left/right variants of Shift,
// Ctrl, Alt and Windows are left out (the plain Shift, Ctrl and Alt are taken).
int recorded_key(PanelState& p) {
    int found=-2;   // nothing yet
    for(int vk=1;vk<255;++vk) {
        if(vk==VK_LBUTTON || vk==VK_RBUTTON || (vk>=0xA0 && vk<=0xA5) || vk==VK_LWIN || vk==VK_RWIN) continue;
        const bool down=(GetAsyncKeyState(vk)&0x8000)!=0;
        if(!down) { p.held_then[vk]=false; continue; }
        if(p.held_then[vk] || found!=-2) continue;
        found=vk==VK_ESCAPE ? -1 : vk==VK_DELETE ? 0 : vk;
    }
    return found;
}

// mouse_loop, every 4 ms: the toggle key, then the panel's keys while it is open and on screen.
void panel_poll() {
    static bool toggle_down=false;
    static KeyRepeat up,down,left,right,back,bind;
    const Config::Keys& keys=config.keys;
    const int toggle=keys.tuning;
    const bool toggle_held=toggle>0 && (GetAsyncKeyState(toggle)&0x8000)!=0;
    std::lock_guard<std::mutex> lock(panel_mutex);
    PanelState& p=panel;
    if(p.recording) {   // the toggle key may be the one being recorded
        toggle_down=toggle_held;
        if(!panel_active() || !foreground_is_game()) { p.recording=false; panel_recording.store(false); return; }
        const int vk=recorded_key(p);
        if(vk==-2) return;
        if(vk>=0) {
            p.value[p.selected]=float(vk);
            p.pending[p.selected]=true; p.write_at=0;
        }
        p.recording=false; panel_recording.store(false);
        for(KeyRepeat* k:{&up,&down,&left,&right,&back,&bind}) k->down=true;   // the key just pressed acts no further
        panel_flush(true);
        return;
    }
    if(toggle_held && !toggle_down && foreground_is_game()) {
        if(panel_open.load()) {
            panel_flush(true);
            panel_open.store(false);
            log_line("tuning panel: closed");
        } else {
            for(int i=0;i<panel_count;++i) { p.value[i]=p.was[i]=panel_read(panel_items[i]); p.pending[i]=false; }
            panel_open.store(true);
            log_line("tuning panel: open");
        }
    }
    toggle_down=toggle_held;
    if(!panel_open.load()) return;
    panel_flush(false);
    if(!panel_active() || !foreground_is_game()) return;
    int items[panel_count]; const int n=page_items(p.page,items);
    int at=-1;   // where the selection is in the page (-1: the page row)
    for(int k=0;k<n;++k) if(items[k]==p.selected) at=k;
    if(up.fire(keys.tuning_up)) at=at<0 ? n-1 : at-1;
    if(down.fire(keys.tuning_down)) at=at>=n-1 ? -1 : at+1;
    const bool less=left.fire(keys.tuning_less), more=right.fire(keys.tuning_more);
    if(at<0) {   // the page row: Left/Right turn the page
        p.selected=-1;
        if(less || more) { p.page=1-p.page; log_line("tuning panel: page %s",page_names[p.page]); }
        bind.fire(keys.tuning_bind); back.fire(keys.tuning_undo);
        return;
    }
    p.selected=items[at];
    const PanelItem& item=panel_items[p.selected];
    float& value=p.value[p.selected];
    const float previous=value;
    if(back.fire(keys.tuning_undo)) value=p.was[p.selected];
    if(item.bind) {
        if(bind.fire(keys.tuning_bind)) {
            p.recording=true; panel_recording.store(true);
            for(int vk=0;vk<256;++vk) p.held_then[vk]=(GetAsyncKeyState(vk)&0x8000)!=0;
            return;
        }
        if(value!=previous) { p.pending[p.selected]=true; p.write_at=0; }
        return;
    }
    bind.fire(keys.tuning_bind);
    const bool fine=!item.integer && keys.tuning_fine>0 && (GetAsyncKeyState(keys.tuning_fine)&0x8000)!=0;
    const float step=fine ? item.step/5 : item.step;
    if(less) value-=step;
    if(more) value+=step;
    if(value!=previous) {
        value=std::clamp(std::round(value/(item.step/5))*(item.step/5),item.min,item_max(item));
        if(item.integer) value=std::round(value);
        if(value!=previous) { p.pending[p.selected]=true; p.write_at=GetTickCount64()+150; }
    }
}

// Keys kept from the game: the toggle key always; the panel's keys while it is open; every key
// while one is being recorded. A key's release goes where its press went (a press before the
// panel opened is the game's to release).
std::atomic<bool> panel_swallowed[256]{};
bool panel_key(int vk) {
    const Config::Keys& k=config.keys;
    return vk>0 && (vk==k.tuning_up || vk==k.tuning_down || vk==k.tuning_less || vk==k.tuning_more ||
                    vk==k.tuning_undo || vk==k.tuning_bind);
}
WNDPROC game_window_proc{};
HWND panel_window{};
LRESULT CALLBACK panel_window_proc(HWND window,UINT message,WPARAM wparam,LPARAM lparam) {
    if((message==WM_KEYDOWN || message==WM_KEYUP || message==WM_SYSKEYDOWN || message==WM_SYSKEYUP) && wparam<256) {
        const int vk=int(wparam);
        const bool press=message==WM_KEYDOWN || message==WM_SYSKEYDOWN;
        if(press && (panel_recording.load() || (vk==config.keys.tuning && vk>0) || (panel_active() && panel_key(vk)))) {
            static bool reported=false;
            if(!reported) { reported=true; log_line("tuning panel: key 0x%02X kept from the game",vk); }
            panel_swallowed[vk].store(true);
        }
        if(panel_swallowed[vk].load()) {
            if(!press) panel_swallowed[vk].store(false);
            return 0;
        }
    }
    // the characters of keys pressed while recording (a letter would type into the game's text input)
    if((message==WM_CHAR || message==WM_SYSCHAR || message==WM_DEADCHAR) && panel_recording.load()) return 0;
    return CallWindowProcW(game_window_proc,window,message,wparam,lparam);
}
// overlay_loop: the game's main window, once.
void panel_attach(HWND window) {
    if(panel_window || !window) return;
    wchar_t name[64]{};
    GetClassNameW(window,name,64);
    if(wcscmp(name,L"UnrealWindow")!=0) return;
    game_window_proc=reinterpret_cast<WNDPROC>(GetWindowLongPtrW(window,GWLP_WNDPROC));
    if(!game_window_proc) return;
    panel_window=window;
    SetWindowLongPtrW(window,GWLP_WNDPROC,reinterpret_cast<LONG_PTR>(&panel_window_proc));
    log_line("tuning panel: attached to the game window (toggle key %s)",key_label(config.keys.tuning).c_str());
}

// draw_overlay: the panel, top left below the HUD's lines; returns where it was drawn. Under the title
// the two pages are tabs (the open one lit, the page keys beside them: a page row of plain text was
// missed, reported 2026-10-07); a row is a name and its value (and the opening value once changed),
// in columns, values left-aligned; the selected item's description and config.ini name sit at the
// bottom, wrapped to three lines. Columns, rows and size are fixed in units of the font size, the
// same whatever is selected, on either page and in every language; text is centred in its row by
// the font's own height (Segoe UI sat low in a row sized for the text height).
RECT draw_tuning_panel(HDC dc,uint32_t* pixels,int width,int height,float scale) {
    if(!panel_open.load()) return RECT{0,0,0,0};
    PanelState state;
    { std::lock_guard<std::mutex> lock(panel_mutex); state=panel; }
    static HFONT font{}; static int font_height=0, font_generation=-1;
    const int text_height=std::max(13,int(std::lround(15*scale)));
    if(font_height!=text_height || font_generation!=text_generation.load()) {
        if(font) DeleteObject(font);
        font=CreateFontW(-text_height,0,0,0,FW_NORMAL,0,0,0,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,
                         ANTIALIASED_QUALITY,DEFAULT_PITCH,text_font().c_str());
        font_height=text_height; font_generation=text_generation.load();
    }
    HGDIOBJ old_font=SelectObject(dc,font);
    TEXTMETRICW metrics{};
    GetTextMetricsW(dc,&metrics);
    auto measure=[&](const std::wstring& s) { SIZE size{}; GetTextExtentPoint32W(dc,s.c_str(),int(s.size()),&size); return LONG(size.cx); };
    const LONG h=text_height, inner=38*h, value_x=15*h, was_x=25*h, column_gap=h;
    const int line=int(h*1.55f), pad=int(10*scale), text_offset=std::max(0,(line-int(metrics.tmHeight))/2);
    const Config::Keys& keys=config.keys;   // as [keys] has them
    auto key=[](int vk) { return widen(key_label(vk)); };
    auto fit=[&](std::wstring s,LONG room) {
        if(measure(s)<=room) return s;
        while(s.size()>1 && measure(s+L"\x2026")>room) s.pop_back();
        return s+L"\x2026";
    };
    // rows: title, tabs, help, then each item of the page (and its group's heading)
    enum Kind { Title, Tabs, Help, Group, Item };
    struct Row { Kind kind; std::wstring text, value, was; bool selected=false; };
    auto rows_for=[&](int page) {
        std::vector<Row> rows;
        rows.push_back({Title,tr("panel.title",L"MouseFlight Settings ({1}: close)",{key(keys.tuning)})});
        rows.push_back({Tabs,L"",L"",L"",page==state.page && state.selected<0});
        rows.push_back({Help,page==0
            ? tr("panel.help.settings",L"{1}/{2} Select   {3}/{4} Adjust (hold {5}: fine)   {6} Restore",
                 {key(keys.tuning_up),key(keys.tuning_down),key(keys.tuning_less),key(keys.tuning_more),key(keys.tuning_fine),key(keys.tuning_undo)})
            : tr("panel.help.keys",L"{1}/{2} Select   {3} Change Key   {4} Restore",
                 {key(keys.tuning_up),key(keys.tuning_down),key(keys.tuning_bind),key(keys.tuning_undo)})});
        for(int i=0;i<panel_count;++i) {
            if(item_page(i)!=page) continue;
            const PanelItem& item=panel_items[i];
            if(item.group) rows.push_back({Group,group_name(item)});
            const bool selected=page==state.page && i==state.selected;
            Row row{Item,item_name(item),L"",L"",selected};
            row.value=selected && state.recording ? L"..." : panel_shown(item,state.value[i]);
            if(!(selected && state.recording) && panel_text(item,state.value[i])!=panel_text(item,state.was[i]))
                row.was=tr("panel.was",L"was: {1}",{panel_shown(item,state.was[i])});
            rows.push_back(row);
        }
        return rows;
    };
    const std::vector<Row> rows=rows_for(state.page);
    const size_t most_rows=std::max(rows_for(0).size(),rows_for(1).size());
    // the description: the selected item's (and its config.ini name), or the recording's prompt
    std::wstring footer;
    if(state.recording) footer=tr("panel.recording",L"Press the new key (Esc: cancel  Delete: clear)");
    else if(state.selected>=0) {
        const PanelItem& item=panel_items[state.selected];
        footer=item_label(item)+L"   ["+widen(item.key)+L"]";
        if(item.bind && state.value[state.selected]>0) {   // the same key elsewhere
            std::wstring also;
            for(int i=0;i<panel_count;++i)
                if(i!=state.selected && panel_items[i].bind && state.value[i]==state.value[state.selected])
                    also+=(also.empty()?L"":L", ")+item_name(panel_items[i]);
            if(!also.empty()) footer+=L"   "+tr("panel.also",L"Same key as: {1}",{also});
        }
    }
    else footer=tr("panel.tabs.help",L"Use {1}/{2} to change the page.",{key(keys.tuning_less),key(keys.tuning_more)});
    // wrapped to the width: at a space or after punctuation where there is one, else anywhere (CJK)
    constexpr size_t footer_rows=3;
    std::vector<std::wstring> footer_lines;
    for(size_t start=0;start<footer.size() && footer_lines.size()<footer_rows;) {
        size_t end=start, last_break=std::wstring::npos;
        while(end<footer.size() && measure(footer.substr(start,end-start+1))<=inner) {
            const wchar_t c=footer[end];
            if(c==L' ' || c==L',' || c==L';' || c==L'.' || c==L'\xFF0C' || c==L'\xFF1B' || c==L'\x3002' || c==L'\x3001' || c==L'\xFF09') last_break=end+1;
            ++end;
        }
        if(end<footer.size() && last_break!=std::wstring::npos && last_break>start) end=last_break;
        std::wstring part=footer.substr(start,std::max<size_t>(end-start,1));
        start+=part.size();
        while(start<footer.size() && footer[start]==L' ') ++start;
        if(footer_lines.size()+1==footer_rows && start<footer.size()) part=fit(part+L"\x2026",inner);
        footer_lines.push_back(part);
    }
    const int gap=line/2;   // before the description
    RECT box{LONG(28*scale),LONG(100*scale),0,0};
    box.right=std::min(LONG(width),box.left+inner+2*pad);
    box.bottom=std::min(LONG(height),box.top+LONG(most_rows+footer_rows)*line+gap+2*pad);
    if(box.right<=box.left || box.bottom<=box.top) { SelectObject(dc,old_font); return RECT{0,0,0,0}; }
    auto fill=[&](RECT r,uint32_t color) {   // premultiplied
        for(LONG y=std::max(0L,r.top);y<std::min(LONG(height),r.bottom);++y)
            for(LONG x=std::max(0L,r.left);x<std::min(LONG(width),r.right);++x) pixels[size_t(y)*width+x]=color;
    };
    auto frame=[&](RECT r,uint32_t color,LONG t) {
        fill(RECT{r.left,r.top,r.right,r.top+t},color); fill(RECT{r.left,r.bottom-t,r.right,r.bottom},color);
        fill(RECT{r.left,r.top,r.left+t,r.bottom},color); fill(RECT{r.right-t,r.top,r.right,r.bottom},color);
    };
    fill(box,0xC8000000);
    SetBkMode(dc,TRANSPARENT);
    auto out=[&](LONG x,int top,const std::wstring& s,COLORREF color) {   // in a row starting at top, centred
        SetTextColor(dc,color); TextOutW(dc,x,top+text_offset,s.c_str(),int(s.size()));
    };
    const COLORREF white=RGB(240,240,240), dim=RGB(150,150,150), lit=RGB(255,220,120), heading=RGB(120,200,255);
    const LONG left=box.left+pad;
    int top=box.top+pad;
    for(const Row& row:rows) {
        switch(row.kind) {
        case Title: out(left,top,fit(row.text,inner),RGB(255,255,255)); break;
        case Tabs: {
            // the two pages as tabs: the open one lit; a frame on them while the cursor is on this row
            LONG x=left;
            const LONG tab_pad=h/2, border=std::max(1L,h/10);
            for(int page=0;page<2;++page) {
                const std::wstring name=page==0 ? tr("panel.page.settings",L"Settings") : tr("panel.page.keys",L"Key Config");
                RECT tab{x,LONG(top+line/12),x+measure(name)+2*tab_pad,LONG(top+line-line/12)};
                GdiFlush();
                fill(tab,page==state.page ? 0xE0305A88u : 0xC8262626u);
                if(page==state.page && row.selected) frame(tab,0xFFFFDC78u,border);
                out(x+tab_pad,top,name,page==state.page ? (row.selected ? lit : RGB(255,255,255)) : dim);
                x=tab.right+h/3;
            }
            out(x+h/2,top,fit(tr("panel.page.hint",L"({1}/{2}: change page)",{key(keys.tuning_less),key(keys.tuning_more)}),inner-(x+h/2-left)),
                row.selected ? lit : dim);
            break;
        }
        case Help: out(left,top,fit(row.text,inner),dim); break;
        case Group: out(left,top,fit(row.text,inner),heading); break;
        case Item: {
            if(row.selected) { GdiFlush(); fill(RECT{box.left+pad/2,LONG(top),box.right-pad/2,LONG(top+line)},0xC8303A46); }
            const COLORREF color=row.selected ? lit : white;
            out(left,top,fit((row.selected?L"> ":L"  ")+row.text,value_x-column_gap),color);
            out(left+value_x,top,fit(row.value,was_x-value_x-column_gap),color);   // values left-aligned in their column
            if(!row.was.empty()) out(left+was_x,top,fit(row.was,inner-was_x),dim);
            break;
        }
        }
        top+=line;
    }
    top=box.bottom-pad-int(footer_rows)*line;   // the description at the bottom, where it always is
    for(const auto& part:footer_lines) { out(left,top,part,state.recording ? lit : RGB(190,190,190)); top+=line; }
    SelectObject(dc,old_font);
    GdiFlush();
    // GDI writes its pixels with zero alpha: make them opaque; the background keeps its own.
    for(LONG py=std::max(0L,box.top);py<std::min(LONG(height),box.bottom);++py)
        for(LONG px=std::max(0L,box.left);px<std::min(LONG(width),box.right);++px) {
            auto& pixel=pixels[size_t(py)*width+px];
            if(!(pixel>>24) && (pixel&0x00FFFFFF)) pixel|=0xFF000000;
        }
    return box;
}
