#pragma once

#include <limits>
#include <vector>

#include "Agent.h"
#include "BihTree.h"




class Simulator 
{
public:
    Simulator()
        : m_globalTime(0.0f), m_reachedGoals(false)
    {}

	~Simulator();

	void doStep(float deltaTime, bool doUpdate = true);

	bool haveReachedGoals() const {
        return m_reachedGoals;
    }

    int addObject(Object* agent)
    {
	    m_objects.push_back(agent);
	    return m_objects.size() - 1;
    }

    void removeAgent(std::size_t id) {
        m_objects.erase(m_objects.begin() + id);
    }

 	int getNumAgents() const { 
        return m_objects.size(); 
    }

    Object* getObject(int i) {
        return m_objects[i];
    }


private:
	Simulator(const Simulator &other);
	Simulator &operator=(const Simulator &other);

public:
    BihTree m_bihTree;

	float m_globalTime;
	bool m_reachedGoals;
    Document* m_doc;
	std::vector<Object*> m_objects;

	friend class Agent;

};

