#ifdef EMSCRIPTEN
#include <emscripten.h>
#else // Qt WebView
#include "qt_emasm.h"
#endif

#include "js_main.h"
#include <string>
#include <vector>
#include <memory>
#include "../Vec2.h"
#include "../Document.h"
#include "../Except.h"

using namespace std;

// bias the index so that they would be displayed in the proper z order

#define Z_POLYPOINT 10
#define Z_TRI 0
#define Z_AGENT 20
#define Z_GOAL 25

class NavCtrl;


class Item 
{
public:
    Item(NavCtrl* ctrl) : m_ctrl(ctrl)
    {}
    virtual ~Item() {
        EM_ASM_( remove_object($0), this );
    }
    virtual void setPos(const Vec2& p) 
    {}

    NavCtrl* m_ctrl;
};

class PolyPointItem : public Item
{
public:
    PolyPointItem(NavCtrl* ctrl, Vertex* v) :Item(ctrl), m_v(v)
    {
        EM_ASM_( add_circle($0, $1, $2, $3, $4, 'rgb(50,50,50)', false), this, Z_POLYPOINT, m_v->p.x, m_v->p.y, 5);
    }
    virtual void setPos(const Vec2& p) {
        m_v->p = p;
        EM_ASM_( move_object($0, $1, $2), this, m_v->p.x, m_v->p.y);
    }
    
    Vertex* m_v;
};

class TriItem  : public Item
{
public:
    TriItem(NavCtrl* ctrl, Triangle* t) :Item(ctrl), m_t(t)
    {
        const Vec2& a = m_t->v[0]->p;
        const Vec2& b = m_t->v[1]->p;
        const Vec2& c = m_t->v[2]->p;
        //EM_ASM_( out("tri (" + $0 + " " + $1 + ") (" + $2 + " " + $3 + ") (" + $4 + " " + $5 + ")"), a.x, a.y, b.x, b.y, c.x, c.y);
        EM_ASM_( add_tri($0, $1, $2, $3, $4, $5, $6, $7), this, Z_TRI, a.x, a.y, b.x, b.y, c.x, c.y);
    }

    Triangle* m_t;
};

class AgentItem : public Item
{
public:
    AgentItem(NavCtrl* ctrl, Agent* a) :Item(ctrl), m_a(a)
    {
        EM_ASM_( add_circle($0, $1, $2, $3, $4, 'rgb(255,50,50)', true), this, Z_AGENT, m_a->m_position.x, m_a->m_position.y, m_a->m_radius);
    }
    virtual ~AgentItem() {}
    virtual void setPos(const Vec2& p) {
        m_a->setPos(p);
        updatePos();
    }
    void updatePos() {
        EM_ASM_( move_object($0, $1, $2), this, m_a->m_position.x, m_a->m_position.y);
    }
    Agent* m_a;
};

class GoalItem : public Item
{
public:
    GoalItem(NavCtrl* ctrl, Goal* g) :Item(ctrl), m_g(g)
    {
        EM_ASM_( add_circle($0, $1, $2, $3, $4, 'rgb(165,85,255)', false), this, Z_GOAL, m_g->p.x, m_g->p.y, 7);
    }
    virtual void setPos(const Vec2& p);
    Goal* m_g;
};

struct AgentData {
    Vec2 pos;
    Vec2 vel;
};
struct Frame
{
    vector<AgentData> m_agents;
};

class NavCtrl
{
public:
    NavCtrl() {
    }
    void addPoly() {
        if (!m_doc.m_mapdef.isLastEmpty())
            m_doc.m_mapdef.add();
    }
    void addPolyPoint(const Vec2& p) 
    {
        auto pv = m_doc.m_mapdef.addToLast(p);
        auto i = new PolyPointItem(this, pv);
        m_polypointitems.push_back(shared_ptr<PolyPointItem>(i));
        updateMesh();
    }
    void addAgent(const Vec2& p)
    {
        auto pa = m_doc.addAgent(p, nullptr);
        auto i = new AgentItem(this, pa);
        m_agentitems.push_back(shared_ptr<AgentItem>(i));
    }
    GoalItem* addGoal(const Vec2& p) 
    {
        auto pg = m_doc.addGoal(p);
        auto i = new GoalItem(this, pg);
        m_goalitems.push_back(shared_ptr<GoalItem>(i)); 
        return i;
    }

    void removeGoal(GoalItem* g) {
        auto it = m_goalitems.begin();
        while(it != m_goalitems.end()) {
            if (it->get() == g) {
                m_doc.removeGoal( (*it)->m_g);
                it = m_goalitems.erase(it);
            }
            else
                ++it;
        }
    }
    void setAgentGoalPos(Agent* a, Goal* g) {
        a->m_endGoalPos = g->p;
        m_doc.updatePlan(a);
    }
    void setGoal(AgentItem* a, GoalItem* g) {
        setAgentGoalPos(a->m_a, g->m_g);
        g->m_g->agents.push_back(a->m_a);
    }

    
    
    void progress(float deltaSec) 
    {
        if (m_doc.m_goals.empty())
            return;
        m_doc.doStep(deltaSec, true);  //0.25

        if (m_atFrame < m_frames.size())
            m_frames.resize(m_atFrame); // truncate any future frames
        m_frames.push_back(Frame());
        auto& frame = m_frames.back();
        ++m_atFrame;

        for(auto& a: m_agentitems) {
            a->updatePos();
            frame.m_agents.push_back(AgentData{a->m_a->m_position, a->m_a->m_velocity});
        }
        EM_ASM_( set_max_frame($0), m_frames.size()-1);
    }

    void goToFrame(int f) 
    {
        if (f >= m_frames.size()) {
            OUT("bad frame index " << f << " sz=" << m_frames.size());
            return;
        }

        auto& frame = m_frames[f];
        for(int i = 0; i < frame.m_agents.size(); ++i) {
            auto ai = m_agentitems[i];
            const auto& fa = frame.m_agents[i];
            ai->m_a->m_position = fa.pos;
            ai->m_a->m_velocity = fa.vel;
            ai->updatePos();
        }
        m_atFrame = f;
    }
    
    void updateMesh();
    void readDoc();
    
    vector<shared_ptr<PolyPointItem>> m_polypointitems;
    vector<shared_ptr<TriItem>> m_meshitems;
    vector<shared_ptr<AgentItem>> m_agentitems;
    vector<shared_ptr<GoalItem>> m_goalitems;
    Document m_doc;
    vector<Frame> m_frames;
    int m_atFrame = 0;
};

// depends on NavCtrl
void GoalItem::setPos(const Vec2& p) {
    m_g->p = p;
    for(auto* a: m_g->agents)
        m_ctrl->setAgentGoalPos(a, m_g);
    EM_ASM_( move_object($0, $1, $2), this, m_g->p.x, m_g->p.y);
}


void NavCtrl::updateMesh()
{
    m_doc.runTriangulate();
    m_meshitems.clear();
    for(auto& tri : m_doc.m_mesh.m_tri) {
        auto i = new TriItem(this, &tri);
        m_meshitems.push_back(shared_ptr<TriItem>(i));
    }
}

void NavCtrl::readDoc()
{
    m_agentitems.clear();
    m_goalitems.clear();
    m_polypointitems.clear();
    //OUT("AItems " << m_doc.m_agents.size());
    for(auto* agent: m_doc.m_agents) {
        m_agentitems.push_back(shared_ptr<AgentItem>(new AgentItem(this, agent)));
    }
    //OUT("GItems " << m_doc.m_goals.size());
    for(auto& goal: m_doc.m_goals) {
        m_goalitems.push_back(shared_ptr<GoalItem>(new GoalItem(this, goal.get()))); 
    }
    //OUT("PPItems " << m_doc.m_mapdef.m_vtx.size());
    for(auto* pv: m_doc.m_mapdef.m_vtx) {
        m_polypointitems.push_back(shared_ptr<PolyPointItem>(new PolyPointItem(this, pv)));
    }
}


NavCtrl* g_ctrl = nullptr;

#ifdef EMSCRIPTEN
void cpp_out(const char* s) {
    EM_ASM_( out(Pointer_stringify($0)), s);
}
#endif

extern "C" {

void cpp_start()
{
    g_ctrl = new NavCtrl;
}

void started_new_poly() {
    g_ctrl->addPoly();
}
void added_poly_point(int x, int y) {
    g_ctrl->addPolyPoint(Vec2(x, y));
}
void added_agent(int x, int y) {
    g_ctrl->addAgent(Vec2(x, y));
}

void moved_object(ptr_t ptr, int x, int y)
{
    Item* p = (Item*)ptr;
    p->setPos(Vec2(x, y));
    g_ctrl->updateMesh();
}

ptr_t add_goal(int x, int y) {
    auto g = g_ctrl->addGoal(Vec2(x, y));
    //OUT("AddGoal " << g);
    return (ptr_t)g;
}
void set_goal(ptr_t agentPtr, ptr_t goalPtr) {
    AgentItem* a = (AgentItem*)agentPtr;
    GoalItem* g = (GoalItem*)goalPtr;
    //OUT("SetGoal " << a << " " << g);
    g_ctrl->setGoal(a, g);
}
void remove_goal(ptr_t ptr) {
    GoalItem* g = (GoalItem*)ptr;
    g_ctrl->removeGoal(g);
}

void cpp_progress(float deltaSec) {
    g_ctrl->progress(deltaSec);
    
}

const char* serialize() {
    ostringstream ss;
    g_ctrl->m_doc.serialize(ss);
    static string s;
    s = ss.str();
    return s.c_str();
}
void deserialize(const char* sp) {
    string a(sp);
    istringstream ss(a);
    //OUT("DESER " << a);
    g_ctrl->m_doc.deserialize(ss);
    g_ctrl->readDoc();
    g_ctrl->updateMesh();
}

void go_to_frame(int f) {
    g_ctrl->goToFrame(f);
}


}