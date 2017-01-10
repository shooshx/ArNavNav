#pragma once


#define NEI_DIST_RADIUS_FACTOR (2.0f)
// changing this factor also changes how narrow a tri-to-segment corridor the agent can pass
// since the neighbors check determines when the agent detects the segment


class ISubGoal
{
public:
    virtual ~ISubGoal() {}
    virtual Vec2 getDest(const Vec2& comingFrom) const = 0;
    virtual bool isPassed(const Vec2& amAt) const = 0;
    virtual bool isPoint() const = 0;
    virtual Vec2 representPoint() const = 0; // point required for knowing the direction we're coming from
};

#define GOAL_SEGMENT_LEN 30.0f
#define GOAL_PERPENDICULAR_OFFSET 10.1f


// just arrive anywhere on the segment, that would be great
class SegmentSubGoal : public ISubGoal
{
public:
    SegmentSubGoal() {}
    SegmentSubGoal(const Vec2& _a, const Vec2& _b, int _passCheckSign) :a(_a), b(_b), ab(_b - _a), passCheckSign(_passCheckSign)
    {
        pab = Vec2(ab.y, -ab.x);
        pab.normalize();
        pab *= passCheckSign * GOAL_PERPENDICULAR_OFFSET;
        // want to go slightly further than the goal so we'll be able to pass it
    }
    virtual Vec2 getDest(const Vec2& comingFrom) const {
        //return project(comingFrom, a, b);

        return a + pab;
    }
    virtual bool isPassed(const Vec2& amAt) const {
        return (passCheckSign * det(ab, amAt - a)) < 0; // did we pass the line
    }
    virtual bool isPoint() const {
        return false;
    }
    virtual Vec2 representPoint() const {
        return a + pab;
    }
    Vec2 a, b, ab, pab;
    int passCheckSign = 1;
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
    virtual bool isPoint() const {
        return true;
    }
    virtual Vec2 representPoint() const { // not really needed since this is the end but for consistancy
        return p;
    }

    Vec2 p = INVALID_VEC2;
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

        m_segs.push_back(SegmentSubGoal(a, b, rev ? -1 : 1));
        m_d.push_back(&m_segs.back());
    }
    void setEnd(const Vec2& p, float radius) {
        m_endp = PointSubGoal(p, radius);
        m_d.push_back(&m_endp);
    }
    void clear() {
        m_segs.clear();
        m_d.clear();
        m_endp = PointSubGoal();
    }
    void reserve(int size) {
        m_d.reserve(size);
        m_segs.reserve(size - 1);
    }

    std::vector<ISubGoal*> m_d; // pointers to m_planData and m_planEnd;
    std::vector<SegmentSubGoal> m_segs;
    PointSubGoal m_endp;

    // should not copy since it contains pointers to content
    Plan(const Plan&) = delete;
    void operator=(const Plan&) = delete;
};

template<typename T>
class MyPrioQueue // one that exposes the underlying vector
{
public:
    const T& top() const {
        return c.front();
    }
    void qpush(const T& _Val) {
        c.push_back(_Val);
        if (m_isheap)
            std::push_heap(c.begin(), c.end());
    }
    // if we never pop, it doesn't need to be a heap
    void qpop() {
        if (!m_isheap) {
            std::make_heap(c.begin(), c.end());
            m_isheap = true;
        }
        std::pop_heap(c.begin(), c.end());
        c.pop_back();
    }
    void sort() { // make it not a heap
        if (m_isheap)
            std::sort_heap(c.begin(), c.end());
        else
            std::sort(c.begin(), c.end());
        m_isheap = false;
    }
    void clear() {
        c.clear();
        m_isheap = false;
    }

    bool m_isheap = false;
    std::vector<T> c;
};


enum EGoalType {
    GOAL_POINT = 0,
    GOAL_ATTACK = 1
};

class Agent;

class GoalDef 
{
public:
    GoalDef() {}
    GoalDef(const Vec2& _p, float r, EGoalType t) :p(_p), radius(r), type(t)
    {}
    Vec2 p = INVALID_VEC2;
    float radius = 0.0f;
    EGoalType type = GOAL_POINT;
};

namespace RVO { class Agent; }

class Goal
{
public:
    Goal() {}
    Goal(const Vec2& _p, float r, EGoalType t) : def(_p, r, t)
    {}
    GoalDef def;
    std::vector<RVO::Agent*> agents; // agents that have this goal
};

