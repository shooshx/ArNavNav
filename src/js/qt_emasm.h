#pragma once

#include <sstream>

void makeArgs(int lvl, std::ostringstream& defs, std::ostringstream& call) {
}
template<typename... Args>
void makeArgs(int lvl, std::ostringstream& defs, std::ostringstream& call, const char* first, Args... rest) {
    defs << "$" << lvl << ",";
    call << "'" << first << "',";
    makeArgs(lvl + 1, defs, call, rest...);
}
template<typename T, typename... Args>
void makeArgs(int lvl, std::ostringstream& defs, std::ostringstream& call, T* first, Args... rest) {
    defs << "$" << lvl << ",";
    call << "0x" << first << ",";
    makeArgs(lvl + 1, defs, call, rest...);
}
template<typename T, typename... Args>
void makeArgs(int lvl, std::ostringstream& defs, std::ostringstream& call, T first, Args... rest) {
    defs << "$" << lvl << ",";
    call << first << ",";
    makeArgs(lvl + 1, defs, call, rest...);
}

void js_run(const std::string& js);
template<typename... Args>
void asm_const_int(const char* code, Args... args) {
    std::ostringstream defs, call;
    makeArgs(0, defs, call, args...);
    auto defss = defs.str();
    defss.pop_back(); // last comma
    auto calls = call.str();
    calls.pop_back();
    js_run("(function(" + defss + "){" + code + "})(" + calls + ")");
}

#define EM_ASM_(code, ...) asm_const_int(#code, __VA_ARGS__)
