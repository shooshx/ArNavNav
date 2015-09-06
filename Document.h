#ifndef DOCUMENT_H 
#define DOCUMENT_H

#include <QObject>
#include <QPointF>
#include <QColor>

#include <vector>

#include "Agent.h"
#include "Objects.h"
#include "Mesh.h"


using namespace std;
class Agent;
struct VODump;

class Goal
{
public:
    explicit Goal(const Vec2& _p) :p(_p) {}
    Vec2 p;
    vector<Agent*> agents; // agents that have this goal
};



class ISubGoalMaker
{
public:
    virtual ~ISubGoalMaker() {}
    virtual void make(float distAway, const Vec2& comingFrom, vector<SubGoal>& addto) = 0;
};




class Document : public QObject
{
    Q_OBJECT

public:
    Document(QObject *parent);
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

    void addAgent(const Vec2& pos, Goal* g);

    void clearSegMinDist();
    bool doStep(float deltaTime, bool doUpdate);

public:
    // input
    vector<Agent*> m_agents;
    MapDef m_mapdef;

    // state
    vector<Object*> m_objs; // owning
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
