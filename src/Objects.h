#pragma once

#include "Vec2.h"
#include "Except.h"

class Object
{
public:
    enum EType {
        TypeCircle = 1,
        TypeAgent = 2,
        TypeOther = 3 // Segment, PointSegment
    };

    Object() {}
    Object(const Vec2& _center, const Vec2& _size, int _index, EType type = TypeOther)
        : m_position(_center), size(_size), index(_index), m_type(type)
    {}
    virtual ~Object() {}



    virtual void setPos(const Vec2& p) {
        m_position = p;
    }
    virtual void setSize(const Vec2& v) {
        size = v;
    }


    EType m_type;  // used for avoiding dynamic_cast
    int index = 0;
    Vec2 m_position;
    Vec2 size; // used in BihTree

    //bool highlight = false;
};

class Circle : public Object
{
public:
    Circle(const Vec2& center, float r, int _index, EType type = TypeCircle) 
        : Object(center, Vec2(2 * r, 2 * r), _index, type), m_radius(r) 
    {}
    virtual ~Circle() {}



    float m_radius;
};

class AABB : public Object
{
public:
    AABB(const Vec2& center, const Vec2& size, int _index) 
        : Object(center, size, _index)
        , maxp(center.x + size.x * 0.5f, center.y + size.y * 0.5f)
        , minp(center.x - size.x * 0.5f, center.y - size.y * 0.5f)
    {}


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



    void justExtPoints(float keepDist, Vec2* p1, Vec2* p2) const
    {
        *p1 = a + dpa * keepDist;
        *p2 = b + dpb * keepDist;
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



    void justExtPoints(float keepDist, Vec2* p1, Vec2* p2) const
    {
        *p1 = b + dpa * keepDist;
        *p2 = b + dpb * keepDist;
    }




    Vec2 b;
    Vec2 dpa, dpb; 
};





