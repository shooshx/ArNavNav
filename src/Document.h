#ifndef DOCUMENT_H 
#define DOCUMENT_H


#include <vector>
#include <memory>

#include "Agent.h"
#include "Objects.h"
#include "Mesh.h"
#include "BihTree.h"

using namespace std;
class Agent;
struct VODump;


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

    void init_test();
    void init_circle();

    void clearObst();
    void clearAllObj();

    Agent* addAgent(const Vec2& pos, Goal* g, float radius = 15.0, float prefSpeed = -1.0f, float maxSpeed = -1.0f);
    void addAgentRadius(float radius);
    
    Goal* addGoal(const Vec2& p, float radius, EGoalType type);
    void removeGoal(Goal* g);

    void clearSegMinDist();
    bool doStep(float deltaTime, bool doUpdate, int dbg_frameNum);

    void updatePlan(Agent* agent);
    bool shouldReplan(Agent* agent);

    void serialize(ostream& os);
    void deserialize(istream& is, map<string, string>& imported);

    void readStream(istream& is, map<string, string>& imported, const string& module);
public:
    // input
    vector<Agent*> m_agents; // not owning
    MapDef m_mapdef;

    // state
    vector<Object*> m_objs; // owning
    BihTree m_bihTree;

    vector<MultiSegment> m_multisegs; 
    vector<ISubGoalMaker*> m_seggoals; // save size as m_mesh.m_vtx. for every vertex, get goals that are away from it

    Mesh m_mesh;
    vector<unique_ptr<Goal>> m_goals;

    // display
    vector<Vertex*> m_markers;
    Object *m_prob = nullptr;

    VODump* m_debugVoDump = nullptr; 
};





#endif // DOCUMENT_H
