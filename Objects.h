#pragma once

#include "Vec2.h"
#include "Except.h"

class Object
{
public:
    Object() {}
    Object(const Vec2& _center, const Vec2& _size, int _index)
        : m_position(_center), size(_size), index(_index)
    {}
    virtual ~Object() {}
    virtual float distSqToSurface(const Vec2& fromp) const = 0;

    virtual void setPos(const Vec2& p) {
        m_position = p;
    }
    virtual void setSize(const Vec2& v) {
        size = v;
    }

    int index = 0;
    Vec2 m_position;
    Vec2 size;

    bool highlight = false;
};

class Circle : public Object
{
public:
    Circle(const Vec2& center, float r, int _index) 
        : Object(center, Vec2(2 * r, 2 * r), _index), m_radius(r) 
    {}
    virtual ~Circle() {}

    virtual float distSqToSurface(const Vec2& fromp) const {
        float distSq = absSq(fromp - m_position);
        float checkSq = sqrt(distSq) - m_radius; 
        checkSq *= checkSq;
        return checkSq;
    }

    float m_radius;
};

class AABB : public Object
{
public:
    AABB(const Vec2& center, const Vec2& size, int _index) 
        : Object(center, size, _index)
        , maxp(center.x + size.x * 0.5, center.y + size.y * 0.5)
        , minp(center.x - size.x * 0.5, center.y - size.y * 0.5)
    {}

    // return a bitmask of one of the 9 sectors fromp is in compared to this aabb
    static int inSector(const Vec2& fromp, const Vec2& maxp, const Vec2& minp) 
    {
        int xabove = fromp.x > maxp.x; //1
        int xbelow = fromp.x < minp.x; //2
        int xin = !xabove && !xbelow;  //4

        int yabove = fromp.y > maxp.y; //8
        int ybelow = fromp.y < minp.y; //16
        int yin = !yabove && !ybelow;  //32

        return xabove + (xbelow << 1) + (xin << 2) + (yabove << 3) + (ybelow << 4) + (yin << 5);
    }

    virtual float distSqToSurface(const Vec2& fromp) const
    {
        switch(inSector(fromp, maxp, minp)) {
        case 4+8:  return sqr(fromp.y - maxp.y);
        case 4+16: return sqr(fromp.y - minp.y);
        case 32+1: return sqr(fromp.x - maxp.x);
        case 32+2: return sqr(fromp.x - minp.x);
        case 1+8:  return absSq(maxp - fromp);
        case 16+2: return absSq(minp - fromp);
        case 1+16: return absSq(Vec2(maxp.x, minp.y) - fromp);
        case 8+2:  return absSq(Vec2(minp.x, maxp.y) - fromp);
        default: return 0; // inside
        }
    }

    bool spanningPoints(const Vec2& fromp, float keepDist, Vec2* p1, Vec2* p2) const
    {
        Vec2 a = maxp + Vec2(keepDist, keepDist);
        Vec2 b = minp - Vec2(keepDist, keepDist);;
        Vec2 c = Vec2(a.x, b.y);
        Vec2 d = Vec2(b.x, a.y);

        switch(inSector(fromp, a, b)) {
        case 4+8:  *p1 = d; *p2 = a; break;
        case 4+16: *p1 = c; *p2 = b; break;
        case 32+1: *p1 = a; *p2 = c; break; 
        case 32+2: *p1 = b; *p2 = d; break;
        case 1+8:  *p1 = d; *p2 = c; break;
        case 16+2: *p1 = c; *p2 = d; break;
        case 1+16: *p1 = a; *p2 = b; break;
        case 8+2:  *p1 = b; *p2 = a; break;
        default: return false; // inside
        }
        return true;
    }

    Vec2 maxp, minp;
};