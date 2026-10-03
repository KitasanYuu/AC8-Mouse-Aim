// Development-only flight logic module. The main DLL copies and hot-loads it when it
// changes, so control-law edits take effect without restarting the game.
#include "flight_logic.h"
using namespace flight;
extern "C" __declspec(dllexport) int ac8_logic_abi(unsigned* input_size,unsigned* output_size,unsigned* state_size) {
    return logic_api::abi(input_size,output_size,state_size);
}
extern "C" __declspec(dllexport) void ac8_logic_configure(const wchar_t* config_path) { logic_api::configure(config_path); }
extern "C" __declspec(dllexport) void ac8_logic_reset(void* state) { logic_api::reset(state); }
extern "C" __declspec(dllexport) void ac8_logic_step(void* state,const LogicInput* in,LogicOutput* out) {
    logic_api::step(state,in,out);
}
