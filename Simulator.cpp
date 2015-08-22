#include "Simulator.h"

#include "Agent.h"



Simulator::~Simulator()
{
/*	for (auto iter = m_objects.begin(); iter != m_objects.end(); ++iter) {
		delete *iter;
	}*/
}


void Simulator::doStep(float deltaTime, bool doUpdate)
{
    if (deltaTime <= 0.0f)
        return;

    m_bihTree.build(m_objects);

	for (auto iter = m_objects.begin(); iter != m_objects.end(); ++iter)
    {
        Object* obj = *iter;
        if (obj->index == 39) {
            int i = 0;
        }
        Agent* agent = dynamic_cast<Agent*>(obj);
        if (agent == nullptr || !agent->m_isMobile)
            continue;
        agent->computePreferredVelocity(deltaTime);

        agent->computeNeighbors(m_bihTree);
        agent->computeNewVelocity();

	}

    if (!doUpdate)
        return;

    m_reachedGoals = true;
	for (auto iter = m_objects.begin(); iter != m_objects.end(); ++iter) {
        Object* obj = *iter;
        Agent* agent = dynamic_cast<Agent*>(obj);
        if (agent == nullptr || !agent->m_isMobile)
            continue;
        m_reachedGoals &= agent->update(deltaTime);
	}

	m_globalTime += deltaTime;
}



