// In-game tuning panel: [keys] tuning (F3) opens a list of config.ini values over the game, on two
// pages: Settings (values, changed with the arrow keys while flying) and Keys (the mod's keys:
// Enter, then the new key). Each change is written to config.ini, which the mod reloads within
// 0.5 s (poll_dev_files); config.ini stays the only place values live, and players need not edit
// it. Keys the panel uses are kept from the game while it is open (its window procedure), so the
// arrows do not also act in the game; while a key is being recorded, every key is kept from it.
struct PanelItem {
    const char* group;              // a heading before this item, or nullptr
    const wchar_t* section;         // config.ini section
    const char* key;
    const char* label;
    float min, max, step;           // Shift: a fifth of the step
    bool integer;
    float fallback;                 // shown when config.ini has no value
    int Config::Keys::* bind;       // a key of [keys] (Keys page), not a value
};
PanelItem value_item(const char* group,const wchar_t* section,const char* key,const char* label,
                     float min,float max,float step,bool integer,float fallback) {
    return {group,section,key,label,min,max,step,integer,fallback,nullptr};
}
PanelItem key_item(const char* group,const char* key,const char* label,int Config::Keys::* bind) {
    return {group,L"keys",key,label,0,255,1,true,float(Config::Keys{}.*bind),bind};
}
const PanelItem panel_items[]={
    value_item("Mouse and chase camera",L"control","sensitivity","mouse, deg per count",0.01f,1.0f,0.005f,false,0.10f),
    value_item(nullptr,L"control","camera_follow","camera turn to the aim, 1/s",1,60,1,false,12),
    value_item(nullptr,L"control","camera_distance","behind, m (0: the game's own)",0,100,1,false,30),
    value_item(nullptr,L"control","camera_height","above, m",-20,30,0.5f,false,6),
    value_item(nullptr,L"control","camera_fov_add","FOV added to the game's, deg",-40,40,1,false,0),
    value_item("Cockpit and nose views",L"control","near_view_camera","0 fixed, 1 follow, 2 edge, 3 head",0,3,1,true,3),
    value_item(nullptr,L"control","near_view_follow","head turn toward the ring",0,1,0.05f,false,0.6f),
    value_item(nullptr,L"control","near_view_hud","head limit past half the view, deg",0,40,1,false,8),
    value_item(nullptr,L"control","near_view_mouse","mouse scale",0.1f,3,0.05f,false,1),
    value_item(nullptr,L"control","near_view_expo","ring curve (1 linear)",1,4,0.1f,false,1.5f),
    value_item(nullptr,L"control","near_view_inertia","view lag behind the aircraft, s",0,0.5f,0.01f,false,0.06f),
    value_item(nullptr,L"control","near_view_level","roll held toward level",0,1,0.05f,false,0),
    value_item(nullptr,L"control","near_view_fov_add","FOV added to the game's, deg",-40,40,1,false,0),
    // What sets the turn (closed-loop simulation, measured plant, 2026-10-07): pitch_gain is what limits
    // the pitch (1.5 to 4: up 20 deg reached in 2.4 to 1.4 s; past 4 the stick chatters near the aim;
    // pitch_brake changed nothing at all); roll_brake slows a big roll's stop when lowered (right 30
    // deg 2.8 s at 0.45, 4.1 s at 0.2) and changes little when raised (the roll rate is the limit).
    value_item("Flight",L"tuning","pitch_gain","pitch response (higher: faster, past 4 jittery)",1,4,0.1f,false,2.5f),
    value_item(nullptr,L"tuning","roll_brake","roll stop planned (lower: softer, slower)",0.2f,1,0.05f,false,0.45f),
    value_item(nullptr,L"tuning","level_per_deg","bank kept per deg to go (lower: levels sooner)",2,30,1,false,10),
    value_item(nullptr,L"tuning","lead_gain","follow a moving aim",0,2,0.05f,false,1.0f),
    value_item(nullptr,L"tuning","lead_filter","smoothing of that, s",0.05f,1.5f,0.05f,false,0.4f),
    value_item(nullptr,L"tuning","highg_full_from","high-G full pull beyond, deg",0,45,1,false,8),
    key_item("Flight","toggle","mouse aim on and off",&Config::Keys::toggle),
    key_item(nullptr,"recenter","the aim back on the nose",&Config::Keys::recenter),
    key_item(nullptr,"free_look","held: look around",&Config::Keys::free_look),
    key_item(nullptr,"hud","the mod's rings on and off",&Config::Keys::hud),
    key_item(nullptr,"post_stall","held: post-stall with the high-G",&Config::Keys::post_stall),
    key_item("This panel","tuning","open and close it",&Config::Keys::tuning),
    key_item(nullptr,"tuning_up","previous item",&Config::Keys::tuning_up),
    key_item(nullptr,"tuning_down","next item",&Config::Keys::tuning_down),
    key_item(nullptr,"tuning_less","lower the value",&Config::Keys::tuning_less),
    key_item(nullptr,"tuning_more","raise the value",&Config::Keys::tuning_more),
    key_item(nullptr,"tuning_fine","held: finer steps",&Config::Keys::tuning_fine),
    key_item(nullptr,"tuning_undo","back to the opening value",&Config::Keys::tuning_undo),
    key_item(nullptr,"tuning_bind","record a new key (this page)",&Config::Keys::tuning_bind),
    key_item("Diagnostics","reload","read config.ini again",&Config::Keys::reload),
    key_item(nullptr,"trace","per-frame flight trace in the log",&Config::Keys::trace),
    key_item(nullptr,"perf","performance counters",&Config::Keys::perf),
    key_item(nullptr,"camera_probe","camera state capture",&Config::Keys::camera_probe),
    key_item(nullptr,"hangar_specs","hangar: the aircraft's ratings",&Config::Keys::hangar_specs),
    key_item(nullptr,"hangar_geometry","hangar: the aircraft's geometry",&Config::Keys::hangar_geometry),
};
constexpr int panel_count=int(sizeof(panel_items)/sizeof(panel_items[0]));
int item_page(int i) { return panel_items[i].bind ? 1 : 0; }
const char* const page_names[2]={"Settings","Keys"};
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
// The value as config.ini holds it: no trailing zeros; a key by its name.
std::string panel_text(const PanelItem& item,float value) {
    if(item.bind) return key_label(int(std::lround(value)));
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
float panel_read(const PanelItem& item) {
    wchar_t key[64]{}, text[64]{};
    MultiByteToWideChar(CP_ACP,0,item.key,-1,key,64);
    if(item.bind) return float(read_key(item.section,key,int(item.fallback)));
    GetPrivateProfileStringW(item.section,key,L"",text,64,config_path);
    wchar_t* end{};
    const float parsed=wcstof(text,&end);
    return end!=text && std::isfinite(parsed) ? parsed : item.fallback;
}

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
        value=std::clamp(std::round(value/(item.step/5))*(item.step/5),item.min,item.max);
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

// draw_overlay: the panel, top left below the status line; returns where it was drawn. Compact:
// a row is the key and its value (and the opening value once changed); the selected item's
// description sits at the bottom (each row's own had made the panel half the screen wide).
RECT draw_tuning_panel(HDC dc,uint32_t* pixels,int width,int height,float scale) {
    if(!panel_open.load()) return RECT{0,0,0,0};
    PanelState state;
    { std::lock_guard<std::mutex> lock(panel_mutex); state=panel; }
    static HFONT font{}; static int font_height=0;
    const int text_height=std::max(13,int(std::lround(15*scale)));
    if(font_height!=text_height) {
        if(font) DeleteObject(font);
        font=CreateFontW(-text_height,0,0,0,FW_NORMAL,0,0,0,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,
                         ANTIALIASED_QUALITY,FIXED_PITCH|FF_MODERN,L"Consolas");
        font_height=text_height;
    }
    HGDIOBJ old_font=SelectObject(dc,font);
    struct Line { std::string text; COLORREF color; bool selected=false; };
    std::vector<Line> lines;
    char row[200]{};
    const Config::Keys& keys=config.keys;   // as [keys] has them
    auto name=[](int vk) { return key_label(vk); };
    snprintf(row,sizeof(row),"MouseFlight tuning  (%s close)",name(keys.tuning).c_str());
    lines.push_back({row,RGB(255,255,255)});
    snprintf(row,sizeof(row),"%s %s  %s",state.selected<0?">":" ",
        state.page==0?"[Settings]  Keys":" Settings  [Keys]",state.selected<0?"(Left/Right: page)":"");
    lines.push_back({row,state.selected<0?RGB(255,220,120):RGB(200,200,200),state.selected<0});
    if(state.page==0)
        snprintf(row,sizeof(row),"%s/%s  %s/%s (%s fine)  %s undo",name(keys.tuning_up).c_str(),name(keys.tuning_down).c_str(),
            name(keys.tuning_less).c_str(),name(keys.tuning_more).c_str(),name(keys.tuning_fine).c_str(),name(keys.tuning_undo).c_str());
    else
        snprintf(row,sizeof(row),"%s/%s  %s new key  %s undo",name(keys.tuning_up).c_str(),name(keys.tuning_down).c_str(),
            name(keys.tuning_bind).c_str(),name(keys.tuning_undo).c_str());
    lines.push_back({row,RGB(150,150,150)});
    for(int i=0;i<panel_count;++i) {
        if(item_page(i)!=state.page) continue;
        const PanelItem& item=panel_items[i];
        if(item.group) lines.push_back({item.group,RGB(120,200,255)});
        const bool selected=i==state.selected;
        const std::string now=selected && state.recording ? "..." : panel_text(item,state.value[i]);
        const std::string was=panel_text(item,state.was[i]);
        if(now!=was && !(selected && state.recording))
            snprintf(row,sizeof(row),"%s %-17s %9s  was %s",selected?">":" ",item.key,now.c_str(),was.c_str());
        else snprintf(row,sizeof(row),"%s %-17s %9s",selected?">":" ",item.key,now.c_str());
        lines.push_back({row,selected?RGB(255,220,120):RGB(235,235,235),selected});
    }
    // the bottom line: the selected item's description, or what a recording waits for
    std::string footer;
    if(state.selected>=0) {
        const PanelItem& item=panel_items[state.selected];
        footer=item.label;
        if(state.recording) footer="press the new key (Esc: cancel, Delete: none)";
        else if(item.bind && state.value[state.selected]>0) {   // the same key elsewhere
            std::string also;
            for(int i=0;i<panel_count;++i)
                if(i!=state.selected && panel_items[i].bind && state.value[i]==state.value[state.selected])
                    also+=(also.empty()?"":", ")+std::string(panel_items[i].key);
            if(!also.empty()) footer+="  (also: "+also+")";
        }
    }
    lines.push_back({footer,state.recording?RGB(255,220,120):RGB(190,190,190)});
    const int line=int(text_height*1.25f), pad=int(9*scale), gap=line/2;   // gap: before the description
    LONG text_width=0;
    for(const auto& l:lines) {
        SIZE size{};
        GetTextExtentPoint32A(dc,l.text.c_str(),int(l.text.size()),&size);
        text_width=std::max(text_width,size.cx);
    }
    RECT box{LONG(28*scale),LONG(100*scale),0,0};
    box.right=std::min(LONG(width),box.left+text_width+2*pad);
    box.bottom=std::min(LONG(height),box.top+LONG(lines.size())*line+gap+2*pad);
    if(box.right<=box.left || box.bottom<=box.top) { SelectObject(dc,old_font); return RECT{0,0,0,0}; }
    auto fill=[&](RECT r,uint32_t color) {   // premultiplied
        for(LONG y=std::max(0L,r.top);y<std::min(LONG(height),r.bottom);++y)
            for(LONG x=std::max(0L,r.left);x<std::min(LONG(width),r.right);++x) pixels[size_t(y)*width+x]=color;
    };
    fill(box,0xC8000000);
    SetBkMode(dc,TRANSPARENT);
    int y=box.top+pad;
    for(size_t n=0;n<lines.size();++n) {
        const Line& l=lines[n];
        if(n+1==lines.size()) y+=gap;
        if(l.selected) {
            GdiFlush();
            fill(RECT{box.left+pad/2,LONG(y-line/10),box.right-pad/2,LONG(y+line-line/10)},0xC8303A46);
        }
        SetTextColor(dc,l.color);
        TextOutA(dc,box.left+pad,y,l.text.c_str(),int(l.text.size()));
        y+=line;
    }
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
