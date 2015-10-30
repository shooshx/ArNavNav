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
        if (m_isCircle)
            EM_ASM_( remove_circle($0), this );
        else 
            EM_ASM_( remove_triangle($0), this );
    }
    virtual void setPos(const Vec2& p) 
    {}

    NavCtrl* m_ctrl;
    bool m_isCircle = true;
};

class PolyPointItem : public Item
{
public:
    PolyPointItem(NavCtrl* ctrl, Vertex* v) :Item(ctrl), m_v(v)
    {
        EM_ASM_( add_circle($0, $1, $2, $3, RADIUS_POLYPOINT, 'rgb(50,50,50)', ObjSelType.NONE, ObjType.POLYPOINT, null), this, Z_POLYPOINT, m_v->p.x, m_v->p.y);
    }
    virtual void setPos(const Vec2& p);
    
    Vertex* m_v;
};

class TriItem  : public Item
{
public:
    TriItem(NavCtrl* ctrl, Triangle* t) :Item(ctrl), m_t(t)
    {
        m_isCircle = false;
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
        EM_ASM_( add_circle($0, $1, $2, $3, $4, COLOR_AGENT, ObjSelType.MULTI, ObjType.AGENT, null), this, Z_AGENT, m_a->m_position.x, m_a->m_position.y, m_a->m_radius);
    }
    virtual ~AgentItem() {}
    virtual void setPos(const Vec2& p) {
        m_a->setPos(p);
        updatePos();
    }
    void updatePos() {
        EM_ASM_( move_circle($0, $1, $2), this, m_a->m_position.x, m_a->m_position.y);
    }
    void updateSize() {
        EM_ASM_( changed_size($0, $1), this, m_a->m_radius);
    }
    Agent* m_a;
};

class GoalItem : public Item
{
public:
    GoalItem(NavCtrl* ctrl, Goal* g) :Item(ctrl), m_g(g)
    {
        EM_ASM_( add_circle($0, $1, $2, $3, RADIUS_GOAL, ($4 == 0) ? COLOR_POINT_GOAL : COLOR_ATTACK_GOAL, ObjSelType.SINGLE, ObjType.GOAL, $5), 
                 this, Z_GOAL, m_g->def.p.x, m_g->def.p.y, m_g->def.type, m_g->def.radius);
    }
    virtual void setPos(const Vec2& p);
    void update() {
        EM_ASM_( updated_goal($0, $1, ($2 == 0) ? COLOR_POINT_GOAL : COLOR_ATTACK_GOAL ), this, m_g->def.radius, m_g->def.type);
    }
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
        m_quiteCount = 0;
    }
    void addAgent(const Vec2& p, float radius, float speed)
    {
        auto pa = m_doc.addAgent(p, nullptr, radius, speed);
        auto i = new AgentItem(this, pa);
        m_agentitems.push_back(shared_ptr<AgentItem>(i));
        m_quiteCount = 0;
    }
    GoalItem* addGoal(const Vec2& p, float radius, int type) 
    {
        auto pg = m_doc.addGoal(p, radius, (EGoalType)type);
        auto i = new GoalItem(this, pg);
        m_goalitems.push_back(shared_ptr<GoalItem>(i)); 
        return i;
    }

    void removeGoal(GoalItem* g) 
    {
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

    void updateGoal(GoalItem* g, float radius, int type)
    {
        if (radius >= 0)
            g->m_g->def.radius = radius;
        if (type >= 0)
            g->m_g->def.type = (EGoalType)type;
        for(auto* a: g->m_g->agents) {
            a->m_endGoalPos = g->m_g->def;
            // goal-id stays the same
            m_doc.updatePlan(a);
        }
        m_quiteCount = 0;
        g->update();
    }

    // also called from GoalItem::setPos
    void setAgentGoalPos(Agent* a, Goal* g) {
        a->setEndGoal(g->def, (void*)g);
        m_doc.updatePlan(a);
        m_quiteCount = 0;
    }

    void setGoal(AgentItem* a, GoalItem* g) {
        setAgentGoalPos(a->m_a, g->m_g);
        g->m_g->agents.push_back(a->m_a);

        // remove the agent from its previous goal, if any
        auto oldg = m_agentToGoal[a];
        m_agentToGoal[a] = g;
        if (oldg != nullptr) {
            auto it = find(oldg->m_g->agents.begin(), oldg->m_g->agents.end(), a->m_a);
            CHECK(it != oldg->m_g->agents.end(), "Did not find anget in prev goal");
            oldg->m_g->agents.erase(it);
        }
    }



    void setAgentSize(AgentItem* a, float sz) {
        a->m_a->setRadius(sz);
        a->updateSize();
        m_doc.addAgentRadius(sz);
        m_quiteCount = 0;
    }

    void recordFrame() 
    {
        m_frames.resize(m_atFrame); // truncate any future frames
        m_frames.push_back(Frame());
        auto& frame = m_frames.back();

        for(auto& a: m_agentitems) {
            a->updatePos();
            frame.m_agents.push_back(AgentData{a->m_a->m_position, a->m_a->m_velocity});
        }
        EM_ASM_( set_max_frame($0), m_frames.size()-1);
        ++m_atFrame;
    }
    
    bool progress(float deltaSec) 
    {
        if (m_quiteCount >= 100 || m_goalitems.empty())
            return true;
        recordFrame();
        if (m_doc.doStep(deltaSec, true))  //0.25
            ++m_quiteCount;

        return false;
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
        m_quiteCount = 0;
    }
    
    void updateMesh();
    void readDoc();

    void resetFrames() {
        m_frames.clear();
        m_atFrame = 0;
        EM_ASM_( set_max_frame($0), 0);
    }
    
    vector<shared_ptr<PolyPointItem>> m_polypointitems;
    vector<shared_ptr<TriItem>> m_meshitems;
    vector<shared_ptr<AgentItem>> m_agentitems;
    vector<shared_ptr<GoalItem>> m_goalitems;
    Document m_doc;
    vector<Frame> m_frames;
    int m_atFrame = 0; // the index of the last frame that was recorded
    int m_quiteCount = 0; // frames that are quiet
    map<AgentItem*, GoalItem*> m_agentToGoal; // this info should not be in Agent because Anget is not aware of Goal
    map<string, string> m_importedTexts;
};

// depends on NavCtrl
void GoalItem::setPos(const Vec2& p) {
    m_g->def.p = p;
    for(auto* a: m_g->agents)
        m_ctrl->setAgentGoalPos(a, m_g);
    EM_ASM_( move_circle($0, $1, $2), this, m_g->def.p.x, m_g->def.p.y);
}

void PolyPointItem::setPos(const Vec2& p) {
    OUT("setPos " << m_v->index << " " << m_v->p.x << " " << m_v->p.y);
    m_v->p = p;
    EM_ASM_( move_circle($0, $1, $2), this, m_v->p.x, m_v->p.y);
    m_ctrl->updateMesh();
}

void NavCtrl::updateMesh()
{
    try {
        m_doc.runTriangulate(); 
    }
    catch(const exception& e) {
        OUT("failed triangulation");
        return;
    }
    m_meshitems.clear();
    OUT("Triangles " << m_doc.m_mesh.m_tri.size());
    for(auto& tri : m_doc.m_mesh.m_tri) {
        auto i = new TriItem(this, &tri);
        m_meshitems.push_back(shared_ptr<TriItem>(i));
    }
    m_quiteCount = 0;
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
    for(const auto& pv: m_doc.m_mapdef.m_vtx) {
        m_polypointitems.push_back(shared_ptr<PolyPointItem>(new PolyPointItem(this, pv.get())));
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
void added_agent(int x, int y, float radius, float speed) {
    g_ctrl->addAgent(Vec2(x, y), radius, speed);
}

void moved_object(ptr_t ptr, int x, int y)
{
    Item* p = (Item*)ptr;
    p->setPos(Vec2(x, y));
}

// type - see EGoalType
ptr_t add_goal(int x, int y, float radius, int type) {
    auto g = g_ctrl->addGoal(Vec2(x, y), radius, type);
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

bool cpp_progress(float deltaSec) {
    try {
        return g_ctrl->progress(deltaSec);
    }
    catch(const std::exception& e) {
        OUT("EXCEPTION: " << e.what());
        return false;
    }
}
// write doc
const char* serialize() {
    g_ctrl->resetFrames(); // new script so we start the play from the start
    ostringstream ss;
    g_ctrl->m_doc.serialize(ss);
    static string s;
    s = ss.str();
    return s.c_str();
}
// read to doc
void deserialize(const char* sp) {
    string a(sp);
    istringstream ss(a);
    //OUT("DESER " << a);
    g_ctrl->m_doc.deserialize(ss, g_ctrl->m_importedTexts);
    g_ctrl->readDoc();
    g_ctrl->updateMesh();
    if (g_ctrl->m_doc.m_mapdef.m_objModules.size() > 0) // there were imported modules, need to emit scene externts
    {
        Vec2 mn(FLT_MAX, FLT_MAX), mx(-FLT_MAX, -FLT_MAX);
        for(const auto& vtx: g_ctrl->m_doc.m_mapdef.m_vtx) {
            mn.mmin(vtx->p);
            mx.mmax(vtx->p);
        }
        EM_ASM_(set_scene_extents($0, $1, $2, $3), mn.x, mn.y, mx.x, mx.y);
    }
}

void go_to_frame(int f) {
    g_ctrl->goToFrame(f);
}

void change_size(ptr_t ptr, float sz) {
    AgentItem* a = dynamic_cast<AgentItem*>((Item*)ptr);
    if (a == nullptr)
        return;
    g_ctrl->setAgentSize(a, sz);
}

void update_goal(ptr_t ptr, float radius, int type) {
    GoalItem* g = dynamic_cast<GoalItem*>((Item*)ptr);
    if (g == nullptr)
        return; // shouldn't happen
    g_ctrl->updateGoal(g, radius, type);
}

void add_imported(const char* name, const char* text) {
    g_ctrl->m_importedTexts.clear();
    g_ctrl->m_importedTexts[name] = text;
}


}