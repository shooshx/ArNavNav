#pragma once
#include <exception>
#include <string>
#include <sstream>

#ifndef _MSC_VER
#define NOEXCEPT noexcept
#else
#define NOEXCEPT
#endif

class Exception : public std::exception {
public:
    Exception(const std::string& s) :m_desc(s) {}
    virtual ~Exception() {}
    virtual const char* what() const NOEXCEPT { return m_desc.c_str(); }
    std::string m_desc;
};

#define CHECK(pred, msg) do { if(!(pred)) throw Exception(msg); } while(false)

extern void cpp_out(const char* s);
#define OUT(strm) { ostringstream ss; ss << strm; cpp_out(ss.str().c_str()); }
