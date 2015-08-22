#pragma once
#include <exception>

class Exception : public std::exception {
public:
    Exception(const std::string& s) :m_desc(s) {}
    virtual ~Exception() {}
    virtual const char* what() const { return m_desc.c_str(); }
    std::string m_desc;
};

#define CHECK(pred, msg) do { if(!(pred)) throw Exception(msg); } while(false)