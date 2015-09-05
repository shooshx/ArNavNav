#pragma once
#include <cmath>
#include <cfloat>

template<typename T>
inline T imin(T a, T b) {
    return (a < b)?a:b;
}

template<typename T>
inline T imax(T a, T b) {
    return (a > b)?a:b;
}

#define INVALID_VEC2 Vec2(FLT_MAX, FLT_MAX)

class Vec2
{
public:
    Vec2() : x(0.0f), y(0.0f) 
    {}

    Vec2(float _x, float _y) : x(_x), y(_y)
    {}

    union {
        struct {
            float x, y;
        };
        float v[2];
    };


    void operator+=(const Vec2 &o)
    {
        x += o.x;
        y += o.y;
    }

    void mmax(const Vec2& v) {
        x = imax(x, v.x);
        y = imax(y, v.y);
    }
    void mmin(const Vec2& v) {
        x = imin(x, v.x);
        y = imin(y, v.y);
    }

    Vec2 abs() const {
        return Vec2(x>0?x:-x, y>0?y:-y);
    }
    Vec2& normalize();

    bool isValid() const {
        return x != FLT_MAX;
    }
};

inline Vec2 operator+(const Vec2& a, const Vec2& b) {
    return Vec2(a.x + b.x, a.y + b.y);
}
inline Vec2 operator-(const Vec2& a, const Vec2& b) {
    return Vec2(a.x - b.x, a.y - b.y);
}

// dot product
inline float operator*(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

inline float dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}


inline Vec2 operator*(const Vec2& a, float v) {
    return Vec2(a.x * v, a.y * v);
}
inline Vec2 operator*(float v, const Vec2& a) {
    return Vec2(a.x * v, a.y * v);
}

inline bool operator==(const Vec2& a, const Vec2& b) {
    return a.x == b.x && a.y == b.y;
}

inline float absSq(const Vec2& v) {
    return v * v;
}

inline float distSq(const Vec2& a, const Vec2& b) {
    return absSq(a - b);
}

inline float abs(const Vec2& v) {
    return std::sqrt(v * v);
}

inline float det(const Vec2& a, const Vec2& b) {
    return a.x * b.y - a.y * b.x;
}

inline Vec2 operator/(const Vec2& a, float v) {
    const float inv = 1.0f / v;
    return Vec2(a.x * inv, a.y * inv);
}

inline Vec2 normalize(const Vec2 &v) {
    return v / ::abs(v);
}

inline Vec2 normal(const Vec2& a, const Vec2& b) {
    return normalize(Vec2(b.y - a.y, a.x - b.x));
}

inline Vec2& Vec2::normalize() {
    float d = 1.0f / ::abs(*this);
    x *= d;
    y *= d;
    return *this;
}

inline Vec2 operator-(const Vec2& v) {
    return Vec2(-v.x, -v.y);
}

inline float atan(const Vec2& v) {
    return std::atan2(v.y, v.x);
}

inline float sqr(float scalar) {
    return scalar * scalar;
}

inline Vec2 project(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = (b-a);
    Vec2 ap = (p-a);
    float d = dot(ap, ab) / dot(ab, ab);
    d = imin(1.0f, imax(0.0f, d));
    return a + d * ab;
}