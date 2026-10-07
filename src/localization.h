// Text the player sees (the HUD's lines, the tuning panel) in the game's language. Each language
// is a file lang/<code>.txt beside the scripts: UTF-8 lines "id = text" ("#" begins a comment),
// {1}, {2}... standing for what is put in. The language is [control] language: "auto" follows the
// game (the Lua script reports its language each mission), or a file's code. A language without a
// file falls back to one of the same family (zh-Hant to zh-Hans), then English; a text missing from
// the file to the English written where it is used. Adding a language is adding a file.
std::mutex text_mutex;
std::unordered_map<std::string,std::wstring> texts;   // the loaded language's
std::string text_language;                           // its code ("" before the first load)
std::string game_language;                           // the game's, from the Lua script
std::vector<std::string> text_languages;             // the files there are, for the panel
std::atomic<int> text_generation{0};                 // counts loads (the panel rebuilds its font)

std::wstring utf8_to_wide(const std::string& text) {
    if(text.empty()) return {};
    const int n=MultiByteToWideChar(CP_UTF8,0,text.data(),int(text.size()),nullptr,0);
    std::wstring out(size_t(n),L'\0');
    MultiByteToWideChar(CP_UTF8,0,text.data(),int(text.size()),out.data(),n);
    return out;
}
std::string wide_to_utf8(const std::wstring& text) {
    if(text.empty()) return {};
    const int n=WideCharToMultiByte(CP_UTF8,0,text.data(),int(text.size()),nullptr,0,nullptr,nullptr);
    std::string out(size_t(n),'\0');
    WideCharToMultiByte(CP_UTF8,0,text.data(),int(text.size()),out.data(),n,nullptr,nullptr);
    return out;
}
std::wstring lang_folder() { return std::wstring(module_folder)+L"\\lang"; }
// The language files there are (their codes), sorted.
std::vector<std::string> find_languages() {
    std::vector<std::string> found;
    WIN32_FIND_DATAW entry{};
    HANDLE search=FindFirstFileW((lang_folder()+L"\\*.txt").c_str(),&entry);
    if(search==INVALID_HANDLE_VALUE) return found;
    do {
        std::wstring name=entry.cFileName;
        found.push_back(wide_to_utf8(name.substr(0,name.size()-4)));
    } while(FindNextFileW(search,&entry));
    FindClose(search);
    std::sort(found.begin(),found.end());
    return found;
}
// The file for a language code: the same code, else one of its family (the part before "-"), else English.
std::string language_file_for(const std::string& wanted,const std::vector<std::string>& files) {
    auto same=[](const std::string& a,const std::string& b) { return _stricmp(a.c_str(),b.c_str())==0; };
    for(const auto& f:files) if(same(f,wanted)) return f;
    const std::string family=wanted.substr(0,wanted.find('-'));
    for(const auto& f:files) if(same(f.substr(0,f.find('-')),family)) return f;
    return "en";
}
void load_texts(const std::string& code) {
    std::unordered_map<std::string,std::wstring> loaded;
    std::ifstream file(lang_folder()+L"\\"+utf8_to_wide(code)+L".txt",std::ios::binary);
    std::string line;
    while(std::getline(file,line)) {
        if(line.size()>=3 && (unsigned char)line[0]==0xEF && (unsigned char)line[1]==0xBB && (unsigned char)line[2]==0xBF) line.erase(0,3);
        if(!line.empty() && line.back()=='\r') line.pop_back();
        const size_t start=line.find_first_not_of(" \t");
        if(start==std::string::npos || line[start]=='#') continue;
        const size_t equals=line.find('=');
        if(equals==std::string::npos) continue;
        std::string id=line.substr(start,equals-start);
        while(!id.empty() && (id.back()==' ' || id.back()=='\t')) id.pop_back();
        std::string value=line.substr(equals+1);
        const size_t first=value.find_first_not_of(" \t");
        value=first==std::string::npos ? "" : value.substr(first);
        loaded[id]=utf8_to_wide(value);
    }
    const size_t count=loaded.size();
    {
        std::lock_guard<std::mutex> lock(text_mutex);
        texts.swap(loaded);
        text_language=code;
    }
    ++text_generation;
    log_line("text: language %s (%zu texts; game %s, setting %s)",code.c_str(),count,
        game_language.empty()?"not reported yet":game_language.c_str(),config.language.c_str());
}
// After the configuration is read, and when the game reports its language.
void choose_language() {
    const std::vector<std::string> files=find_languages();
    { std::lock_guard<std::mutex> lock(text_mutex); text_languages=files; }
    const std::string wanted=_stricmp(config.language.c_str(),"auto")==0 ? (game_language.empty() ? "en" : game_language) : config.language;
    const std::string code=language_file_for(wanted,files);
    std::string loaded;
    { std::lock_guard<std::mutex> lock(text_mutex); loaded=text_language; }
    if(code!=loaded) load_texts(code);
}
// A text: the language's, else the English given. {1}.. are replaced by the arguments.
std::wstring tr(const char* id,const wchar_t* english,std::initializer_list<std::wstring> args={}) {
    std::wstring text;
    {
        std::lock_guard<std::mutex> lock(text_mutex);
        const auto found=texts.find(id);
        text=found!=texts.end() && !found->second.empty() ? found->second : english;
    }
    int n=1;
    for(const auto& arg:args) {
        const std::wstring mark=L"{"+std::to_wstring(n++)+L"}";
        for(size_t at=text.find(mark);at!=std::wstring::npos;at=text.find(mark,at+arg.size())) text.replace(at,mark.size(),arg);
    }
    return text;
}
std::wstring widen(const std::string& ascii) { return std::wstring(ascii.begin(),ascii.end()); }
std::vector<std::string> languages_available() { std::lock_guard<std::mutex> lock(text_mutex); return text_languages; }
// The font for the language's texts (its file's "font"; CJK text needs one that has it).
std::wstring text_font() { return tr("font",L"Segoe UI"); }
