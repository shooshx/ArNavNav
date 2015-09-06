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


class SubGoal
{
public:
    SubGoal() {}
    explicit SubGoal(const Vec2& _p) :p(_p) {}
    Vec2 p;
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
    SubGoal m_curGoalPos; // in the plan

    Vec2 m_velocity;
    Vec2 m_newVelocity;
    //float m_orientation = 0.0;

	std::set<std::pair<float, Object*> > m_neighbors; // range,id - sorted by range
	
    std::vector<SubGoal> m_plan;

};


