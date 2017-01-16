#ifndef RVO_AGENT_H_
#define RVO_AGENT_H_


#include "Definitions.h"
#include "RVOSimulator.h"

#include "../Objects.h"
#include "../Goal.h"

namespace RVO {


template<typename T, int N>
class CyclicBuffer
{
    T m_buf[N];
    int m_ind;

public:
    void init(T v) {
        for(int i = 0; i < N; ++i)
            m_buf[i] = v;
    }
    void add(T v) {
        m_ind = (m_ind + 1) % N;
        m_buf[m_ind] = v;
    }
    T getPrev(int howBack) {
        return m_buf[ (m_ind + N - howBack) % N ];
    }
};

class Agent 
{
public:
	//explicit Agent(RVOSimulator *sim);

	Agent(int id, const Vec2 &position, const GoalDef& goalPos,
            float neighborDist, int maxNeighbors, float timeHorizon, 
            float timeHorizonObst, float radius, float maxSpeed)
	{
        id_ = id;
        setEndGoal(goalPos, nullptr);
       // sim_ = sim;
		m_position = position;
		maxNeighbors_ = maxNeighbors;
		maxSpeed_ = maxSpeed;
		neighborDist_ = neighborDist;
		m_radius = radius;
		timeHorizon_ = timeHorizon;
		timeHorizonObst_ = timeHorizonObst;
		m_velocity = Vec2();

        m_lastGoalDists.init(FLT_MAX);
    }

	void computeNeighbors(KdTree& kdTree);

	void computeNewVelocity(float timeStep);

	void insertAgentNeighbor(const Agent *agent, float &rangeSq);

	void insertObstacleNeighbor(const Obstacle *obstacle, float rangeSq);

    bool update(float timeStep);

    void setPos(const Vec2& p) {
        m_position = p;
    }       
    void setRadius(float r) {
        m_radius = r;
        //size = Vec2(r * 2, r * 2);
        neighborDist_ = r * NEI_DIST_RADIUS_FACTOR;
    }
    void setEndGoal(const GoalDef& g, Goal* gid) {
        m_endGoalPos = g;
        m_endGoalPos.p += Vec2(0.001 * (rand()%100), 0.001 * (rand()%100)); // random small pertrub to break symmetry
        m_endGoalId = gid; // actually a pointer to Goal
        m_reached = false;
        m_goalIsReachable = false;
    }

    void setSpeed(float speed) {
        //m_prefSpeed = speed;
        maxSpeed_ = speed;// * 2;
    }

    void setTrivialPlan(bool goalIsReachable);

    void computePreferredVelocity(float deltaTime);

    void updateOrientation(float deltaTime);

public:
	//RVOSimulator *sim_;
    int id_;

    // configs
	int maxNeighbors_;
	float maxSpeed_;
	float neighborDist_;
	Vec2 prefVelocity_;
	float m_radius;
	float timeHorizon_;
	float timeHorizonObst_;

    // goal config
    // Vec2 goal_;
    GoalDef m_endGoalPos; // end of the plan, if invalid, there's no current goal, for knowing if we reached the end
    Goal* m_endGoalId = 0; // used for knowing if my neighbors are heading the same way for replanning. type erased since Agent does not know Goal (it's actually Goal*)

    // running vars
	Vec2 m_position;
	Vec2 m_velocity;
	Vec2 newVelocity_;
	std::vector<Line> orcaLines_;
	std::vector<std::pair<float, const Agent *> > agentNeighbors_;
	std::vector<std::pair<float, const Obstacle *> > obstacleNeighbors_;

    float m_orientation = 0.0;

	friend class KdTree;
	friend class RVOSimulator;

    // TBD-move all of these to an object
    ISubGoal* m_curGoalPos = nullptr; // in the plan
    bool m_reached = false;
    bool m_goalIsReachable = false; //determined in updatePlan
    int m_indexInPlan = -1;
    Plan m_plan;

    CyclicBuffer<float, 4> m_lastGoalDists;
};


}

#endif /* RVO_AGENT_H_ */
