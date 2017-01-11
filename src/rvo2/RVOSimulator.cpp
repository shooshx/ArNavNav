
#include "RVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include "Obstacle.h"
#include <iostream>

using namespace std;

namespace RVO {
	RVOSimulator::RVOSimulator() :  globalTime_(0.0f), kdTree_(this)
	{
	}


	RVOSimulator::~RVOSimulator()
	{
        clear();
	}

    void RVOSimulator::clearObstacles()
    {
        for (size_t i = 0; i < obstacles_.size(); ++i) {
			delete obstacles_[i];
		}
        obstacles_.clear();
    }

    void RVOSimulator::clear()
    {
		for (size_t i = 0; i < agents_.size(); ++i) {
			delete agents_[i];
		}
        agents_.clear();
        clearObstacles();

        kdTree_.clear();
    }



    void RVOSimulator::addAgent(Agent* agent)
    {
		agent->id_ = (int)agents_.size();
		agents_.push_back(agent);
    }


	size_t RVOSimulator::addObstacle(const std::vector<Vec2> &vertices)
	{
		if (vertices.size() < 2) {
			return RVO_ERROR;
		}

		const size_t obstacleNo = obstacles_.size();

		for (size_t i = 0; i < vertices.size(); ++i) 
        {
			Obstacle *obstacle = new Obstacle();
			obstacle->point_ = vertices[i];

			if (i != 0) {
				obstacle->prevObstacle_ = obstacles_.back();
				obstacle->prevObstacle_->nextObstacle_ = obstacle;
			}

			if (i == vertices.size() - 1) {
				obstacle->nextObstacle_ = obstacles_[obstacleNo];
				obstacle->nextObstacle_->prevObstacle_ = obstacle;
			}

			obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);

			if (vertices.size() == 2) {
				obstacle->isConvex_ = true;
			}
			else {
				obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
			}

			obstacle->id_ = (int)obstacles_.size();

			obstacles_.push_back(obstacle);
		}

		return obstacleNo;
	}

	void RVOSimulator::doStep(float timeStep)
	{
        //cout << "step " << timeStep << endl;
		kdTree_.buildAgentTree();

		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->computeNeighbors(kdTree_);
			agents_[i]->computeNewVelocity(timeStep);
		}

		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->update(timeStep);
		}
        
		globalTime_ += timeStep;
	}


	void RVOSimulator::processObstacles()
	{
		kdTree_.buildObstacleTree();
	}


void RVOSimulator::setupBlocks()
{
	std::srand(0);


	/* Specify the global time step of the simulation. */
	//timeStep_ = 0.25f;

	/* Specify the default parameters for agents that are subsequently added. */
	//setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);

	/*
	 * Add agents, specifying their start position, and store their goals on the
	 * opposite side of the environment.
	 */
    RVO::Agent* a = nullptr;
	for (size_t i = 0; i < 5; ++i) 
    {
		for (size_t j = 0; j < 5; ++j) 
        {
            //addAgent(RVO::Vec2(55.0f + i * 10.0f,  55.0f + j * 10.0f));
			a = new Agent(-1, Vec2(55.0f + i * 10.0f,  55.0f + j * 10.0f), 
                GoalDef(Vec2(-75.0f, -75.0f), 2.0, GOAL_POINT),
                15.0f, // neighborDist
                10,  // maxNeighbors
                5.0f, // timeHorizon
                5.0f, // timeHorizonObst
                2.0f, // radius
                2.0f); // maxSpeed
            a->setTrivialPlan(true);
            addAgent(a);                                

			//a = addAgent(RVO::Vec2(-55.0f - i * 10.0f,  55.0f + j * 10.0f));
			a = new Agent(-1, Vec2(-55.0f - i * 10.0f,  55.0f + j * 10.0f), 
                GoalDef(Vec2(75.0f, -75.0f), 2.0, GOAL_POINT),
                15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);
            a->setTrivialPlan(true);
            addAgent(a);                                

			//a = addAgent(RVO::Vec2(55.0f + i * 10.0f, -55.0f - j * 10.0f));
			a = new Agent(-1, Vec2(55.0f + i * 10.0f, -55.0f - j * 10.0f), 
                GoalDef(Vec2(-75.0f, 75.0f), 2.0, GOAL_POINT),
                15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);
            a->setTrivialPlan(true);
            addAgent(a);                                

			//a = addAgent(RVO::Vec2(-55.0f - i * 10.0f, -55.0f - j * 10.0f));
			a = new Agent(-1, Vec2(-55.0f - i * 10.0f, -55.0f - j * 10.0f), 
                GoalDef(Vec2(75.0f, 75.0f), 2.0, GOAL_POINT),
                15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);
            a->setTrivialPlan(true);
            addAgent(a);                                
		}
	}

	/*
	 * Add (polygonal) obstacles, specifying their vertices in counterclockwise
	 * order.
	 */
	std::vector<Vec2> obstacle1, obstacle2, obstacle3, obstacle4;

	obstacle1.push_back(Vec2(-10.0f, 40.0f));
	obstacle1.push_back(Vec2(-40.0f, 40.0f));
	obstacle1.push_back(Vec2(-40.0f, 10.0f));
	obstacle1.push_back(Vec2(-10.0f, 10.0f));

	obstacle2.push_back(Vec2(10.0f, 40.0f));
	obstacle2.push_back(Vec2(10.0f, 10.0f));
	obstacle2.push_back(Vec2(40.0f, 10.0f));
	obstacle2.push_back(Vec2(40.0f, 40.0f));

	obstacle3.push_back(Vec2(10.0f, -40.0f));
	obstacle3.push_back(Vec2(40.0f, -40.0f));
	obstacle3.push_back(Vec2(40.0f, -10.0f));
	obstacle3.push_back(Vec2(10.0f, -10.0f));

	obstacle4.push_back(Vec2(-10.0f, -40.0f));
	obstacle4.push_back(Vec2(-10.0f, -10.0f));
	obstacle4.push_back(Vec2(-40.0f, -10.0f));
	obstacle4.push_back(Vec2(-40.0f, -40.0f));

	addObstacle(obstacle1);
	addObstacle(obstacle2);
	addObstacle(obstacle3);
	addObstacle(obstacle4);

	/* Process the obstacles so that they are accounted for in the simulation. */
	processObstacles();
    setPreferredVelocities(false);
}

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif


void RVOSimulator::setPreferredVelocities(bool rnd)
{

    /*
	 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
	 * direction of the goal.
	 */
	for (int i = 0; i < (int)agents_.size(); ++i) 
    {

		Vec2 goalVector = agents_[i]->m_endGoalPos.p - agents_[i]->m_position;

		if (absSq(goalVector) > 1.0f) {
			goalVector = normalize(goalVector);
		}

		agents_[i]->prefVelocity_ = goalVector;

		/*
		 * Perturb a little to avoid deadlocks due to perfect symmetry.
		 */
        if (rnd) {
		    float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
		    float dist = std::rand() * 0.0001f / RAND_MAX;

            agents_[i]->prefVelocity_ = goalVector +  dist * Vec2(std::cos(angle), std::sin(angle));
        }
    }

}




}
