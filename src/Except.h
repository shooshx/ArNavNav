#pragma once
#include <exception>
#include <string>
#include <iostream>
#include <sstream>

#ifndef _MSC_VER
#define NOEXCEPT noexcept
#else
#define NOEXCEPT
#endif

using namespace std;

extern void cpp_out(const char* s);
#define OUT(strm) { cout << strm << endl; }


class Exception : public std::exception {
public:
    Exception(const std::string& s) :m_desc(s) {
        OUT("EXCEPTION " << s);
    }
    virtual ~Exception() {}
    virtual const char* what() const NOEXCEPT { return m_desc.c_str(); }
    std::string m_desc;
};

#define CHECK(pred, msg) do { if(!(pred)) throw Exception(msg); } while(false)

