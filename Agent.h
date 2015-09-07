#pragma once

#include <cstddef>
#include <map>
#include <set>
#include <utility>
#include <vector>


#include "Vec2.h"
#include "Objects.h"

class BihTree;


struct Candidate
{
    Vec2 m_position;
    int m_velocityObstacle1 = 0;
    int m_velocityObstacle2 = 0;
};

struct VelocityObstacle
{
    Vec2 m_apex;
    Vec2 m_side1;
    Vec2 m_side2;
};

struct VODump
{
    std::vector<VelocityObstacle> vos;
    std::multimap<float, Candidate> candidates;
    Vec2 selected;
};


class ISubGoal
{
public:
    virtual ~ISubGoal() {}
    virtual Vec2 getDest(const Vec2& comingFrom) const = 0;
    virtual bool isPassed(const Vec2& amAt) const = 0;
    virtual bool shouldTaper() const = 0;
    virtual Vec2 representPoint() const = 0; // point required for knowing the direction we're coming from
};

#define GOAL_SEGMENT_LEN 30.0f

// just arrive anywhere on the segment, that would be great
class SegmentSubGoal : public ISubGoal
{
public:
    SegmentSubGoal() {}
    SegmentSubGoal(const Vec2& _a, const Vec2& _b) :a(_a), b(_b), ab(_b - _a)
    {}
    virtual Vec2 getDest(const Vec2& comingFrom) const {
        return project(comingFrom, a, b);
    }
    virtual bool isPassed(const Vec2& amAt) const {
        return det(ab, amAt - a) < 0; // did we pass the line
    }
    virtual bool shouldTaper() const {
        return false;
    }
    virtual Vec2 representPoint() const {
        return a;
    }
    Vec2 a, b, ab;

};

// for the end point
class PointSubGoal : public ISubGoal
{
public:
    PointSubGoal() : radiusSq(0) 
    {}
    PointSubGoal(const Vec2& _p, float _radius) :p(_p), radiusSq(_radius * _radius)
    {}
    virtual Vec2 getDest(const Vec2& comingFrom) const {
        return p;
    }
    virtual bool isPassed(const Vec2& amAt) const {
        return (distSq(amAt, p) < radiusSq);
    }
    virtual bool shouldTaper() const {
        return true;
    }
    virtual Vec2 representPoint() const { // not really needed since this is the end but for consistancy
        return p;
    }

    Vec2 p;
    float radiusSq;
};

template<typename T>
void iswap(T& a, T& b) {
    T c = a;
    a = b;
    b = c;
}

class Plan
{
public:
    Plan() {}
    void addSeg(Vec2 a, const Vec2& dir, bool rev) {
        Vec2 b = a + dir * GOAL_SEGMENT_LEN;
        CHECK(m_segs.size() < m_segs.capacity(), "You did not reserve enough space");
        if (rev)
            iswap(a, b);
        m_segs.push_back(SegmentSubGoal(a, b));
        m_d.push_back(&m_segs.back());
    }
    void setEnd(const Vec2& p, float radius) {
        m_endp = PointSubGoal(p, radius);
        m_d.push_back(&m_endp);
    }
    void clearAndReserve(int size) {
        m_segs.clear();
        m_d.clear();
        m_d.reserve(size);
        m_segs.reserve(size - 1);
        m_endp = PointSubGoal();
    }

    std::vector<ISubGoal*> m_d; // pointers to m_planData and m_planEnd;
    std::vector<SegmentSubGoal> m_segs;
    PointSubGoal m_endp;

    // should not copy since it contains pointers to content
    Plan(const Plan&) = delete;
    void operator=(const Plan&) = delete;
};


class Agent : public Circle
{
public:
    Agent(int index, const Vec2& position, const Vec2& goalPos, float neighborDist, int maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed)
        : Circle(position, radius, index)
        , m_endGoalPos(goalPos)
        , m_maxNeighbors(maxNeighbors), m_goalRadius(goalRadius)
        , m_maxSpeed(maxSpeed), m_neighborDist(neighborDist)
        , m_prefSpeed(prefSpeed), m_radius(radius)
    {
    }

    void computeNeighbors(BihTree& bihTree);
    void computeNewVelocity(VODump* dump = nullptr);
    void computePreferredVelocity(float deltaTime);

    // returns true if goal reached
    bool update(float deltaTime);

    void setPos(const Vec2& p) {
        m_position = p;
    }
    void setRadius(float r) {
        m_radius = r;
        size = Vec2(r * 2, r * 2);
    }


private:
    // rangeSq changing according to the furthest added neibor to trim neigbors early
    void insertNeighbor(Object* other, float& rangeSq);

public:
    // configs
    float m_radius = 0.0f;
    bool m_isMobile = true;

    Vec2 m_endGoalPos; // end of the plan
    float m_goalRadius = 0.0f;

    int m_maxNeighbors = 0;
    float m_neighborDist = 0.0f;

	Vec2 m_prefVelocity;
	float m_maxAccel = std::numeric_limits<float>::infinity();
	float m_maxSpeed = 0.0f;
    float m_prefSpeed = 0.0f;
    

    // running vars
    int m_indexInPlan = -1;
    ISubGoal* m_curGoalPos = nullptr; // in the plan

    Vec2 m_velocity;
    Vec2 m_newVelocity;
    //float m_orientation = 0.0;

	std::set<std::pair<float, Object*> > m_neighbors; // range,id - sorted by range
	
    Plan m_plan;
};


