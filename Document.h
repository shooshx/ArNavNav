#ifndef DOCUMENT_H 
#define DOCUMENT_H

#include <QObject>
#include <QPointF>
#include <QColor>

#include <vector>



#include "Objects.h"
#include "Mesh.h"


using namespace std;
class Agent;
struct VODump;

class Goal
{
public:
    Goal(const Vec2& _p) :p(_p){}
    Vec2 p;
    vector<Agent*> agents;
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
    vector<Object*> m_objs; // owning
    vector<MultiSegment> m_multisegs; // owning

    vector<Agent*> m_agents;
    Mesh m_mesh;
    MapDef m_mapdef;

    Vertex *m_start = nullptr;
    Goal *m_end = nullptr;
    vector<Goal*> m_goals;
    vector<Vec2> m_path;
    vector<Vertex*> m_markers;
    Object *m_prob = nullptr;

    VODump* m_debugVoDump = nullptr; 
};





#endif // DOCUMENT_H
