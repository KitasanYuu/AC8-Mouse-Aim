// In-game tuning panel: [keys] tuning (F3) opens a list of config.ini values over the game; the
// arrow keys pick and change them while flying, and each change is written to config.ini, which
// the mod reloads within 0.5 s (poll_dev_files). config.ini stays the only place values live.
// Keys the panel uses are kept from the game while it is open (its window procedure), so the
// arrows do not also act in the game.
struct PanelItem {
    const char* group;        // a heading before this item, or nullptr
    const wchar_t* section;   // config.ini section
    const char* key;
    const char* label;
    float min, max, step;     // Shift: a fifth of the step
    bool integer;
    float fallback;           // shown when config.ini has no value
};
const PanelItem panel_items[]={
    {"Mouse and chase camera",L"control","sensitivity","mouse, deg per count",0.01f,1.0f,0.005f,false,0.10f},
    {nullptr,L"control","camera_follow","camera turn to the aim, 1/s",1,60,1,false,12},
    {nullptr,L"control","camera_distance","behind, m (0: the game's own)",0,100,1,false,30},
    {nullptr,L"control","camera_height","above, m",-20,30,0.5f,false,6},
    {nullptr,L"control","camera_fov_add","FOV added to the game's, deg",-40,40,1,false,0},
    {"Cockpit and nose views",L"control","near_view_camera","0 fixed, 1 follow, 2 edge, 3 head",0,3,1,true,3},
    {nullptr,L"control","near_view_follow","head turn toward the ring",0,1,0.05f,false,0.6f},
    {nullptr,L"control","near_view_hud","head limit past half the view, deg",0,40,1,false,8},
    {nullptr,L"control","near_view_mouse","mouse scale",0.1f,3,0.05f,false,1},
    {nullptr,L"control","near_view_expo","ring curve (1 linear)",1,4,0.1f,false,1.5f},
    {nullptr,L"control","near_view_inertia","view lag behind the aircraft, s",0,0.5f,0.01f,false,0.06f},
    {nullptr,L"control","near_view_level","roll held toward level",0,1,0.05f,false,0},
    {nullptr,L"control","near_view_fov_add","FOV added to the game's, deg",-40,40,1,false,0},
    // What sets the turn (closed-loop simulation, measured plant, 2026-10-07): pitch_gain is what limits
    // the pitch (1.5 to 4: up 20 deg reached in 2.4 to 1.4 s; past 4 the stick chatters near the aim;
    // pitch_brake changed nothing at all); roll_brake slows a big roll's stop when lowered (right 30
    // deg 2.8 s at 0.45, 4.1 s at 0.2) and changes little when raised (the roll rate is the limit).
    {"Flight",L"tuning","pitch_gain","pitch response (higher: faster, past 4 jittery)",1,4,0.1f,false,2.5f},
    {nullptr,L"tuning","roll_brake","roll stop planned (lower: softer, slower)",0.2f,1,0.05f,false,0.45f},
    {nullptr,L"tuning","level_per_deg","bank kept per deg to go (lower: levels sooner)",2,30,1,false,10},
    {nullptr,L"tuning","lead_gain","follow a moving aim",0,2,0.05f,false,1.0f},
    {nullptr,L"tuning","lead_filter","smoothing of that, s",0.05f,1.5f,0.05f,false,0.4f},
    {nullptr,L"tuning","highg_full_from","high-G full pull beyond, deg",0,45,1,false,8},
};
constexpr int panel_count=int(sizeof(panel_items)/sizeof(panel_items[0]));
struct PanelState {
    int selected=0;
    float value[panel_count]{}, was[panel_count]{};   // was: when the panel was opened (Backspace)
    bool pending[panel_count]{};                       // changed, not yet written
    ULONGLONG write_at=0;
};
std::mutex panel_mutex;
PanelState panel;
std::atomic<bool> panel_open{false};
// Open and on screen (the overlay hides while paused, in gaze or autopilot, or with F7): only
// then are its keys taken from the game (the pause menu keeps its arrows).
bool panel_active() { return panel_open.load() && hud_enabled.load() && !game_paused.load() && !yielding(); }

int step_decimals(float step) {
    int d=0;
    for(float s=step;d<4 && std::abs(s-std::round(s))>1e-4f;s*=10) ++d;
    return d;
}
// The value as config.ini holds it: no trailing zeros.
std::string panel_text(const PanelItem& item,float value) {
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

// mouse_loop, every 4 ms: the toggle key, then the panel's keys while it is open and on screen.
void panel_poll() {
    static bool toggle_down=false;
    static KeyRepeat up,down,left,right,back;
    const Config::Keys& keys=config.keys;
    const int toggle=keys.tuning;
    const bool toggle_held=toggle>0 && (GetAsyncKeyState(toggle)&0x8000)!=0;
    std::lock_guard<std::mutex> lock(panel_mutex);
    if(toggle_held && !toggle_down && foreground_is_game()) {
        if(panel_open.load()) {
            panel_flush(true);
            panel_open.store(false);
            log_line("tuning panel: closed");
        } else {
            for(int i=0;i<panel_count;++i) { panel.value[i]=panel.was[i]=panel_read(panel_items[i]); panel.pending[i]=false; }
            panel_open.store(true);
            log_line("tuning panel: open");
        }
    }
    toggle_down=toggle_held;
    if(!panel_open.load()) return;
    panel_flush(false);
    if(!panel_active() || !foreground_is_game()) return;
    if(up.fire(keys.tuning_up)) panel.selected=(panel.selected+panel_count-1)%panel_count;
    if(down.fire(keys.tuning_down)) panel.selected=(panel.selected+1)%panel_count;
    const PanelItem& item=panel_items[panel.selected];
    float& value=panel.value[panel.selected];
    const float previous=value;
    const bool fine=!item.integer && keys.tuning_fine>0 && (GetAsyncKeyState(keys.tuning_fine)&0x8000)!=0;
    const float step=fine ? item.step/5 : item.step;
    if(left.fire(keys.tuning_less)) value-=step;
    if(right.fire(keys.tuning_more)) value+=step;
    if(back.fire(keys.tuning_undo)) value=panel.was[panel.selected];
    if(value!=previous) {
        value=std::clamp(std::round(value/(item.step/5))*(item.step/5),item.min,item.max);
        if(item.integer) value=std::round(value);
        if(value!=previous) { panel.pending[panel.selected]=true; panel.write_at=GetTickCount64()+150; }
    }
}

// Keys kept from the game: the toggle key always; the panel's keys while it is open. A key's
// release goes where its press went (a press before the panel opened is the game's to release).
std::atomic<bool> panel_swallowed[256]{};
bool panel_key(int vk) {
    const Config::Keys& k=config.keys;
    return vk>0 && (vk==k.tuning_up || vk==k.tuning_down || vk==k.tuning_less || vk==k.tuning_more || vk==k.tuning_undo);
}
WNDPROC game_window_proc{};
HWND panel_window{};
LRESULT CALLBACK panel_window_proc(HWND window,UINT message,WPARAM wparam,LPARAM lparam) {
    if((message==WM_KEYDOWN || message==WM_KEYUP || message==WM_SYSKEYDOWN || message==WM_SYSKEYUP) && wparam<256) {
        const int vk=int(wparam);
        const bool press=message==WM_KEYDOWN || message==WM_SYSKEYDOWN;
        if(press && ((vk==config.keys.tuning && vk>0) || (panel_active() && panel_key(vk)))) {
            static bool reported=false;
            if(!reported) { reported=true; log_line("tuning panel: key 0x%02X kept from the game",vk); }
            panel_swallowed[vk].store(true);
        }
        if(panel_swallowed[vk].load()) {
            if(!press) panel_swallowed[vk].store(false);
            return 0;
        }
    }
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
    struct Line { std::string text; COLORREF color; int item=-1; };
    std::vector<Line> lines;
    char row[160]{};
    const Config::Keys& keys=config.keys;   // as [keys] has them
    snprintf(row,sizeof(row),"MouseFlight tuning  (%s close)",key_label(keys.tuning).c_str());
    lines.push_back({row,RGB(255,255,255)});
    snprintf(row,sizeof(row),"%s/%s  %s/%s (%s fine)  %s undo",key_label(keys.tuning_up).c_str(),key_label(keys.tuning_down).c_str(),
        key_label(keys.tuning_less).c_str(),key_label(keys.tuning_more).c_str(),key_label(keys.tuning_fine).c_str(),key_label(keys.tuning_undo).c_str());
    lines.push_back({row,RGB(150,150,150)});
    for(int i=0;i<panel_count;++i) {
        const PanelItem& item=panel_items[i];
        if(item.group) lines.push_back({item.group,RGB(120,200,255)});
        const std::string now=panel_text(item,state.value[i]), was=panel_text(item,state.was[i]);
        if(now!=was) snprintf(row,sizeof(row),"%s %-17s %6s  was %s",i==state.selected?">":" ",item.key,now.c_str(),was.c_str());
        else snprintf(row,sizeof(row),"%s %-17s %6s",i==state.selected?">":" ",item.key,now.c_str());
        lines.push_back({row,i==state.selected?RGB(255,220,120):RGB(235,235,235),i});
    }
    lines.push_back({panel_items[state.selected].label,RGB(190,190,190)});
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
        if(l.item>=0 && l.item==state.selected) {
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
