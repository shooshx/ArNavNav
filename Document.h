#ifndef DOCUMENT_H 
#define DOCUMENT_H


#include <vector>

#include "Agent.h"
#include "Objects.h"
#include "Mesh.h"
#include "BihTree.h"

using namespace std;
class Agent;
struct VODump;

class Goal
{
public:
    explicit Goal(const Vec2& _p, int _index) :p(_p), index(_index) {}
    Vec2 p;
    int index;
    vector<Agent*> agents; // agents that have this goal
};



class ISubGoalMaker
{
public:
    virtual ~ISubGoalMaker() {}
    // add points to the path that. PointSegment adds 2 goal segments for a sharp corner 
    // normal segment adds 1 goal segment
    virtual void makeSubGoal(float keepDist, const Vec2& comingFrom, Plan& addto) = 0;
    // return a single "representative" point for this vertex so that extracting the path from the corridor would be accurate
    virtual Vec2 makePathRef(float keepDist) = 0;
};




class Document 
{
public:
    Document();
    ~Document() {}

    void runTriangulate();

    void init_preset();
    void init_preset_grid();
    void init_test();
    void init_tri();

    void init_circle();
    void init_grid();

    void clearObst();
    void clearAllObj();

    Agent* addAgent(const Vec2& pos, Goal* g);
    Goal* addGoal(const Vec2& p);

    void clearSegMinDist();
    bool doStep(float deltaTime, bool doUpdate);

    void updatePlan(Agent* agent);

public:
    // input
    vector<Agent*> m_agents; // not owning
    MapDef m_mapdef;

    // state
    vector<Object*> m_objs; // owning
    BihTree m_bihTree;

    vector<MultiSegment> m_multisegs; // owning
    vector<ISubGoalMaker*> m_seggoals; // save size as m_mesh.m_vtx. for every vertex, get goals that are away from it

    Mesh m_mesh;
    vector<Goal*> m_goals;

    // display
    vector<Vertex*> m_markers;
    Object *m_prob = nullptr;

    VODump* m_debugVoDump = nullptr; 
};





#endif // DOCUMENT_H
