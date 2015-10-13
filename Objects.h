#pragma once

#include "Vec2.h"
#include "Except.h"

class Object
{
public:
    enum EType {
        TypeCircle = 1,
        TypeAgent = 2,
        TypeOther = 3
    };

    Object() {}
    Object(const Vec2& _center, const Vec2& _size, int _index, EType type = TypeOther)
        : m_position(_center), size(_size), index(_index), m_type(type)
    {}
    virtual ~Object() {}
    virtual float distSqToSurface(const Vec2& fromp) const = 0;
    virtual bool spanningPoints(const Vec2& fromp, float keepDist, Vec2* p1, Vec2* p2) const {
        return false;
    }

    virtual void setPos(const Vec2& p) {
        m_position = p;
    }
    virtual void setSize(const Vec2& v) {
        size = v;
    }


    EType m_type;  // used for avoiding dynamic_cast
    int index = 0;
    Vec2 m_position;
    Vec2 size;

    //bool highlight = false;
};

class Circle : public Object
{
public:
    Circle(const Vec2& center, float r, int _index, EType type = TypeCircle) 
        : Object(center, Vec2(2 * r, 2 * r), _index, type), m_radius(r) 
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
        case 4+8:  return sqr(fromp.y - maxp.y); // distance to axis-aligned segment
        case 4+16: return sqr(fromp.y - minp.y);
        case 32+1: return sqr(fromp.x - maxp.x);
        case 32+2: return sqr(fromp.x - minp.x);
        case 1+8:  return absSq(maxp - fromp);  // distance to point
        case 16+2: return absSq(minp - fromp);
        case 1+16: return absSq(Vec2(maxp.x, minp.y) - fromp);
        case 8+2:  return absSq(Vec2(minp.x, maxp.y) - fromp);
        default: return 0; // inside
        }
    }

    virtual bool spanningPoints(const Vec2& fromp, float keepDist, Vec2* p1, Vec2* p2) const
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

class Segment;

// all segments of a polyline have a pointer to one such object 
// not used for anything
class MultiSegment
{
public:
    /*void clear() {
        m_minInPass_sqDist = FLT_MAX;
        m_minInPass = nullptr;
    }*/
    // in every pass, for every agent that's relevant, keep track of which segment was closest to it 
    // used for finding if the agent is inside the polyline
    //float m_minInPass_sqDist;
    //const Segment* m_minInPass;
};


class Segment : public Object
{
public:
    Segment()
    {}
    Segment(const Vec2& _a, const Vec2& _b, const Vec2& _dpa, const Vec2& _dpb, int _index, MultiSegment* ms)
        : Object((_a + _b)*0.5f, (_a - _b).abs(), _index)
        , a(_a), b(_b), dpa(_dpa), dpb(_dpb), ms(ms)
    {}

  /*  Segment(const Vec2& _a, const Vec2& _b, int _index) 
        : Object((_a + _b)*0.5f, (_a - _b).abs(), _index), a(_a), b(_b)
    {
        Vec2 nab = normalize(a - b);
        Vec2 dp = Vec2(nab.y, -nab.x);
        dpa = dp + nab;
        dpb = dp - nab;
    }*/

    virtual float distSqToSurface(const Vec2& fromp) const
    {
        Vec2 prj = project(fromp, a, b);
        float d = distSq(fromp, prj);
        /*if (d < ms->m_minInPass_sqDist) { disabled
            ms->m_minInPass = this;
            ms->m_minInPass_sqDist = d;
        }*/
        return d;
    }

    void justExtPoints(float keepDist, Vec2* p1, Vec2* p2) const
    {
        *p1 = a + dpa * keepDist;
        *p2 = b + dpb * keepDist;
    }

    virtual bool spanningPoints(const Vec2& fromp, float keepDist, Vec2* p1, Vec2* p2) const
    {
        justExtPoints(keepDist, p1, p2);
       /* if (det(*p1 - fromp, *p2 - fromp) < 0) {
            if (ms->m_minInPass == this) {
                return false;
            }
        }*/
        return true; 
        // if its on the other size of the segment, it means nothing since its not necessarily inside the object, will be ignored by Agent
    }

    Vec2 a, b;
    Vec2 dpa, dpb; // these point in diagonal away from the points. they are an approximation of a circle around the point
    MultiSegment* ms = nullptr;
};


// used for the gap between VOs on a sharp angle of a polyline
class PointSegment : public Object
{
public:
    PointSegment(const Vec2& _b, const Vec2& _dpa, const Vec2& _dpb, int _index)
        : Object(_b, Vec2(0,0), _index), b(_b), dpa(_dpa), dpb(_dpb) 
    {}

    virtual float distSqToSurface(const Vec2& fromp) const
    {
        return distSq(b, fromp);
    }

    void justExtPoints(float keepDist, Vec2* p1, Vec2* p2) const
    {
        *p1 = b + dpa * keepDist;
        *p2 = b + dpb * keepDist;
    }

    virtual bool spanningPoints(const Vec2& fromp, float keepDist, Vec2* p1, Vec2* p2) const
    {
        justExtPoints(keepDist, p1, p2);
        return true; 
    }


    Vec2 b;
    Vec2 dpa, dpb; 
};





/*
class SegmentRef : public Object
{
public:
    SegmentRef(MultiSegment* ms, int p0index, int objIndex)
        :m_ms(ms), m_p0index(p0index), Object()
    {
        const Vec2& p0 = ms->get(p0index);
        const Vec2& p1 = ms->get(p0index + 1);
        m_position = (p0 + p1)*0.5f;
        size = (_a - _b).abs();
        index = objIndex;
    }

    virtual float distSqToSurface(const Vec2& fromp) const
    {
        Vec2 prj = project(fromp, ms->get(p0index), ms->get(p0index + 1));
        return distSq(fromp, prj);
    }

    bool spanningPoints(const Vec2& fromp, float keepDist, Vec2* p1, Vec2* p2) const
    {
        *p1 = a + dpa * keepDist;
        *p2 = b + dpb * keepDist;
        return true; 
        // if its on the other size of the segment, it means nothing since its not necessarily inside the object, will be ignored by Agent
    }


    int m_p0index;
};
*/