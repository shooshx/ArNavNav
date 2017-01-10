#pragma once

#include <cstddef>
#include <map>
#include <set>
#include <utility>
#include <vector>
#include <queue>


#include "Vec2.h"
#include "Objects.h"
#include "Goal.h"

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
    Vec2 p1 = INVALID_VEC2;
    Vec2 p2 = INVALID_VEC2; // the points that originated the sides, for unification of VOs
    Vec2 m_sideMid = INVALID_VEC2;
    bool isBig = false; // is the angle more than 180, use the sideMid point as well
    bool fromObstacle = false; // is this VO made by an obstacle (should never be entered into)
};

struct VODump
{
    std::vector<VelocityObstacle> vos;
    std::multimap<float, Candidate> candidates;
    Vec2 selected;
};



class Agent : public Circle
{
public:
    Agent(int index, const Vec2& position, const GoalDef& goalPos, float neighborDist, int maxNeighbors, float radius, float prefSpeed, float maxSpeed)
        : Circle(position, radius, index, TypeAgent)
        , m_endGoalPos(goalPos), m_endGoalId(nullptr)
        , m_maxNeighbors(maxNeighbors)
        , m_maxSpeed(maxSpeed), m_neighborDist(neighborDist)
        , m_prefSpeed(prefSpeed)
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
        m_neighborDist = r * NEI_DIST_RADIUS_FACTOR;
    }
    void setEndGoal(const GoalDef& g, void* gid) {
        m_endGoalPos = g;
        m_endGoalId = gid; // actually a pointer to Goal
        m_reached = false;
        m_goalIsReachable = false;
    }
    void setSpeed(float speed) {
        m_prefSpeed = speed;
        m_maxSpeed = speed * 2;
    }

private:
    // rangeSq changing according to the furthest added neibor to trim neigbors early
    void insertNeighbor(Object* other, float& rangeSq);

public:
    // configs
    float m_radius = 0.0f;

    GoalDef m_endGoalPos; // end of the plan, if invalid, there's no current goal, for knowing if we reached the end
    // radius is in Circle object
    void* m_endGoalId = 0; // used for knowing if my neighbors are heading the same way for replanning. type erased since Agent does not know Goal (it's actually Goal*)

    int m_maxNeighbors = 0;
    float m_neighborDist = 0.0f;

	float m_maxAccel = std::numeric_limits<float>::infinity();
	float m_maxSpeed = 0.0f;
    float m_prefSpeed = 0.0f;
    

    // running vars
    Vec2 m_prefVelocity;  // computed each step
    int m_indexInPlan = -1;
    ISubGoal* m_curGoalPos = nullptr; // in the plan
    bool m_reached = false;
    bool m_goalIsReachable = false; //determined in updatePlan

    Vec2 m_velocity;
    Vec2 m_newVelocity;
    std::vector<VelocityObstacle> m_voStore; // used in computeNewVelocity, should not be reallocated every time
    float m_orientation = 0.0;


    MyPrioQueue<std::pair<float, Object*> > m_neighbors; // range,id - sorted by range
    //std::set<std::pair<float, Object*> > m_neighbors; // range,id - sorted by range
	
    Plan m_plan;
};


