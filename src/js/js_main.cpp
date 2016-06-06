#ifdef EMSCRIPTEN
#include <emscripten.h>
#else // Qt WebView
#include "qt_emasm.h"
#endif

#include "js_main.h"
#include <string>
#include <vector>
#include <memory>
#include <limits>
#include "../Vec2.h"
#include "../Document.h"
#include "../Except.h"
#include "../mtrig.h"

using namespace std;

// bias the index so that they would be displayed in the proper z order

#define Z_POLYPOINT 10
#define Z_TRI 0
#define Z_AGENT 20
#define Z_GOAL 25
#define Z_ERRBOX 5

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
    PolyPointItem(NavCtrl* ctrl, Vertex* v, int pli, int vi) :Item(ctrl), m_v(v), m_plindex(pli), m_vindex(vi)
    {
        EM_ASM_( add_circle($0, $1, $2, $3, RADIUS_POLYPOINT, 'rgb(50,50,50)', ObjSelType.NONE, ObjType.POLYPOINT, null), this, Z_POLYPOINT, m_v->p.x, m_v->p.y);
    }
    virtual void setPos(const Vec2& p);
    
    Vertex* m_v;
    bool m_moved = false;
    int m_plindex, m_vindex; //polyline index, vertex index in the polyline
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
    virtual void setPos(const Vec2& p);
    void updatePos() {
        EM_ASM_( move_orient_circle($0, $1, $2, $3), this, m_a->m_position.x, m_a->m_position.y, m_a->m_orientation);
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

class BuildingMoveItem;
class BuildingPointItem : public Item
{
public:
    BuildingPointItem(NavCtrl* ctrl, Vertex* v, Vertex* onlyx, Vertex* onlyy, BuildingMoveItem* centerItem)
        :Item(ctrl), m_v(v), m_onlyx(onlyx), m_onlyy(onlyy), m_centerItem(centerItem)
    {
        EM_ASM_( add_circle($0, $1, $2, $3, RADIUS_POLYPOINT, 'rgb(50,50,50)', ObjSelType.SINGLE, ObjType.BUILDING_PNT, null), this, Z_POLYPOINT, m_v->p.x, m_v->p.y);
    }
    virtual void setPos(const Vec2& p);
    void update() {
        EM_ASM_( move_circle($0, $1, $2), this, m_v->p.x, m_v->p.y);
    }

    Vertex *m_v, *m_onlyx, *m_onlyy;
    BuildingMoveItem* m_centerItem;
};

class ErrorBoxItem : public Item
{
public:
    ErrorBoxItem(NavCtrl* ctrl, AABox* b) 
        :Item(ctrl), m_box(b)
    {
        m_isCircle = false;
        EM_ASM_( add_rect($0, $1, $2, $3, $4, $5), this, Z_ERRBOX, b->v[0]->p.x, b->v[0]->p.y, b->v[2]->p.x, b->v[2]->p.y);
    }
    virtual void update() {
        EM_ASM_( change_rect($0, $1, $2, $3, $4), this, m_box->v[0]->p.x, m_box->v[0]->p.y, m_box->v[2]->p.x, m_box->v[2]->p.y);
    }
    AABox* m_box;
};

// center circle that moves the building
class BuildingMoveItem : public Item 
{
public:
    BuildingMoveItem(NavCtrl* ctrl, AABox* b)
        :Item(ctrl), m_boxCp(b)
    {
        calcCenter();
        EM_ASM_( add_circle($0, $1, $2, $3, RADIUS_POLYPOINT, 'rgb(180,100,100)', ObjSelType.NONE, ObjType.POLYPOINT, null), this, Z_POLYPOINT, m_center.x, m_center.y);
    }
    virtual void setPos(const Vec2& p);
    void calcCenter() {
        m_center = Vec2();
        for(int i = 0; i < 4; ++i)
            m_center += m_boxCp->v[i]->p;
        m_center /= 4;
    }
    void update() {
        calcCenter();
        EM_ASM_( move_circle($0, $1, $2), this, m_center.x, m_center.y);
        if (m_errBox)
            m_errBox->update();
    }
    void checkError() {
        if (m_boxCp->intersectError && m_errBox.get() == nullptr) {
            m_errBox.reset(new ErrorBoxItem(m_ctrl, m_boxCp));
        }
        else if (!m_boxCp->intersectError && m_errBox.get() != nullptr) {
            m_errBox.reset();
        }
    }
    AABox* m_boxCp;
    Vec2 m_center;
    BuildingPointItem *m_pItem1 = nullptr, *m_pItem2 = nullptr; 
    unique_ptr<ErrorBoxItem> m_errBox;
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
        auto i = new PolyPointItem(this, pv, -1, -1);
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
    void addBuilding(const Vec2& p)
    {
        auto box = m_doc.m_mapdef.addBox(p, p + Vec2(1,1));

/*        m_doc.m_mapdef.add();
        auto pv1 = m_doc.m_mapdef.addToLast(p); // press down point
        auto pv2 = m_doc.m_mapdef.addToLast(p + Vec2(0,1)); // x of down, y of up
        auto pv3 = m_doc.m_mapdef.addToLast(p + Vec2(1,1)); // up point
        auto pv4 = m_doc.m_mapdef.addToLast(p + Vec2(1,0)); // x of up, y of down
        */
        auto im = new BuildingMoveItem(this, box);
        m_buildingCenterItems.push_back(shared_ptr<BuildingMoveItem>(im));
        im->m_pItem1 = new BuildingPointItem(this, box->v[0], box->v[1], box->v[3], im);
        m_buildingitems.push_back(shared_ptr<BuildingPointItem>(im->m_pItem1));
        im->m_pItem2 = new BuildingPointItem(this, box->v[2], box->v[3], box->v[1], im);
        m_buildingitems.push_back(shared_ptr<BuildingPointItem>(im->m_pItem2));

        EM_ASM_( setPressedObj($0), im->m_pItem2);

        updateBoxesAndMesh();
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

    void changedAgentPos(Agent* a) {
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

    void updateAgent(AgentItem* a, float sz, float speed) {
        if (sz > 0) {
            a->m_a->setRadius(sz);
            a->updateSize();
            m_doc.addAgentRadius(sz);
        }
        if (speed > 0) {
            a->m_a->setSpeed(speed);
        }
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
        if (m_doc.doStep(deltaSec, true, m_frames.size() - 1))  //0.25
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
    
    void updateMesh(); // when plylines change
    void updateBoxesAndMesh(); // when there's a chance boxes changed
    void readDoc();
    void sendPerminiters();

    void resetFrames() {
        m_frames.clear();
        m_atFrame = 0;
        EM_ASM_( set_max_frame($0), 0);
    }

    const char* serialize() {
        resetFrames(); // new script so we start the play from the start
        ostringstream ss;
        m_doc.serialize(ss);
        if (!m_doc.m_mapdef.m_objModules.empty()) // only if there are modules
        {
            for(auto& pp: m_polypointitems) {
                if (pp->m_moved) {
                    ss << "# v:" << pp->m_plindex << ":" << pp->m_vindex << "," << pp->m_v->p.x << "," << pp->m_v->p.y << ",\n";
                }
            }
        }
        static string s;
        s = ss.str();
        return s.c_str();
    }
    
    vector<shared_ptr<PolyPointItem>> m_polypointitems;
    vector<shared_ptr<TriItem>> m_meshitems;
    vector<shared_ptr<AgentItem>> m_agentitems;
    vector<shared_ptr<GoalItem>> m_goalitems;
    vector<shared_ptr<BuildingPointItem>> m_buildingitems;
    vector<shared_ptr<BuildingMoveItem>> m_buildingCenterItems;
    Document m_doc;
    vector<Frame> m_frames;
    int m_atFrame = 0; // the index of the last frame that was recorded
    int m_quiteCount = 0; // frames that are quiet
    map<AgentItem*, GoalItem*> m_agentToGoal; // this info should not be in Agent because Anget is not aware of Goal
    map<string, string> m_importedTexts;
};

void AgentItem::setPos(const Vec2& p) {
    m_a->setPos(p);
    updatePos();
    m_ctrl->changedAgentPos(m_a);
}

// depends on NavCtrl
void GoalItem::setPos(const Vec2& p) {
    m_g->def.p = p;
    for(auto* a: m_g->agents)
        m_ctrl->setAgentGoalPos(a, m_g);
    EM_ASM_( move_circle($0, $1, $2), this, m_g->def.p.x, m_g->def.p.y);
}

void PolyPointItem::setPos(const Vec2& p) {
    //OUT("setPos " << m_v->index << " " << m_v->p.x << " " << m_v->p.y);
    m_v->p = p;
    EM_ASM_( move_circle($0, $1, $2), this, m_v->p.x, m_v->p.y);
    m_ctrl->updateMesh();
    m_moved = true;

}

void BuildingPointItem::setPos(const Vec2& p) {
    m_v->p = p;
    m_onlyx->p.x = p.x;
    m_onlyy->p.y = p.y;
    m_centerItem->update();
    EM_ASM_( move_circle($0, $1, $2), this, m_v->p.x, m_v->p.y);

    m_ctrl->updateBoxesAndMesh();
}

// see https://github.com/kripken/emscripten/issues/3876
float iround(float f) {
#ifdef EMSCRIPTEN
    return EM_ASM_DOUBLE( return Math.round($0), f);
#else
    return std::round(f);
#endif
}

#ifdef EMSCRIPTEN

namespace jsMath {
float atan2(float y, float x) {
    return EM_ASM_DOUBLE(return Math.atan2($0, $1), y, x);
}
float asin(float a) {
    return EM_ASM_DOUBLE(return Math.asin($0), a);
}
float sin(float a) {
    return EM_ASM_DOUBLE(return Math.sin($0), a);
}
float cos(float a) {
    return EM_ASM_DOUBLE(return Math.cos($0), a);
}
}

#endif


void BuildingMoveItem::setPos(const Vec2& p) {
    Vec2 d = p - m_center;
    d.x = iround(d.x); // round to integer so it would stay on the integer grid
    d.y = iround(d.y); 
    for(int i = 0; i < 4; ++i)
        m_boxCp->v[i]->p += d;
    m_center = p;
    EM_ASM_( move_circle($0, $1, $2), this, m_center.x, m_center.y);
    m_pItem1->update();
    m_pItem2->update();
    if (m_errBox)
        m_errBox->update();

    m_ctrl->updateBoxesAndMesh();
}

static float isign(float v) {
    return (v < 0)?-1.0:1.0;
}

void orderPerimiters(vector<Polyline>& p, vector<Polyline*>& o); 

void NavCtrl::sendPerminiters()
{
    static vector<float> v;
    v.clear();
    vector<Polyline*> polyorder;
    orderPerimiters(m_doc.m_mesh.m_perimiters, polyorder);
    for(const auto* pr: polyorder)
    {
        // CW or CCW? http://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
        float sum = 0;
        int sz = pr->m_d.size();
        for(int i = 0; i < sz; ++i) {
            auto* v0 = pr->m_d[i];
            auto* v1 = pr->m_d[(i+1) % sz ];
            float segmentArea = (v0->p.x - v1->p.x)*(v0->p.y + v1->p.y);
            sum += segmentArea;
        }
        for(const auto* vtx: pr->m_d) {
            v.push_back(vtx->p.x);
            v.push_back(vtx->p.y);
        }
        // end of polygon marker
        v.push_back(isign(sum) * numeric_limits<float>::infinity());
    }
    float* p = nullptr;
    if (v.size() > 0)
        p = &v[0];
    EM_ASM_( take_perimiters($0, $1), p, v.size());
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

    sendPerminiters();

    m_meshitems.clear();
    //OUT("Triangles " << m_doc.m_mesh.m_tri.size());
    for(auto& tri : m_doc.m_mesh.m_tri) {
        auto i = new TriItem(this, &tri);
        m_meshitems.push_back(shared_ptr<TriItem>(i));
    }
    m_quiteCount = 0;
}

void NavCtrl::updateBoxesAndMesh()
{
    try {
        m_doc.m_mapdef.makeBoxPoly();
        for(auto& bi: m_buildingCenterItems) {
            bi->checkError(); // display an error box in intersections
        }
    }
    catch(const Exception& e) {
        OUT("EXCEPTION makeBoxPoly");
    }
    updateMesh();
}

void NavCtrl::readDoc()
{
    m_agentitems.clear();
    m_goalitems.clear();
    m_polypointitems.clear();
    m_buildingitems.clear();
    //OUT("AItems " << m_doc.m_agents.size());
    for(auto* agent: m_doc.m_agents) {
        m_agentitems.push_back(shared_ptr<AgentItem>(new AgentItem(this, agent)));
    }
    //OUT("GItems " << m_doc.m_goals.size());
    for(auto& goal: m_doc.m_goals) {
        m_goalitems.push_back(shared_ptr<GoalItem>(new GoalItem(this, goal.get()))); 
    }
    //OUT("PPItems " << m_doc.m_mapdef.m_vtx.size());
    int pli = 0;
    for(auto& pl: m_doc.m_mapdef.m_pl) {
        if (pl->m_fromBox)
            continue;
        int vi = 0;
        for(auto* v: pl->m_d) {
            m_polypointitems.push_back(shared_ptr<PolyPointItem>(new PolyPointItem(this, v, pli, vi++)));
        }
        pli++;
    }
    for(const auto& bx: m_doc.m_mapdef.m_bx) {
        auto im = new BuildingMoveItem(this, bx.get());
        m_buildingCenterItems.push_back(shared_ptr<BuildingMoveItem>(im));
        im->m_pItem1 = new BuildingPointItem(this, bx->v[0], bx->v[1], bx->v[3], im);
        m_buildingitems.push_back(shared_ptr<BuildingPointItem>(im->m_pItem1));
        im->m_pItem2 = new BuildingPointItem(this, bx->v[2], bx->v[3], bx->v[1], im);
        m_buildingitems.push_back(shared_ptr<BuildingPointItem>(im->m_pItem2));
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
    return g_ctrl->serialize();
}
// read to doc
void deserialize(const char* sp) {
    string a(sp);
    //OUT("----------\n" << a);
    istringstream ss(a);
    g_ctrl->m_doc.deserialize(ss, g_ctrl->m_importedTexts);
    g_ctrl->readDoc();

    g_ctrl->updateBoxesAndMesh();
    //OUT("----" << g_ctrl->m_doc.m_mapdef.m_vtx.size() << "  " << g_ctrl->m_doc.m_mapdef.m_objModules.size());
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

void update_agent(ptr_t ptr, float sz, float speed) {
    AgentItem* a = dynamic_cast<AgentItem*>((Item*)ptr);
    if (a == nullptr)
        return;
    g_ctrl->updateAgent(a, sz, speed);
}

void update_goal(ptr_t ptr, float radius, int type) {
    GoalItem* g = dynamic_cast<GoalItem*>((Item*)ptr);
    if (g == nullptr)
        return; // shouldn't happen
    g_ctrl->updateGoal(g, radius, type);
}

void add_imported(const char* name, const char* text) {
    //OUT("IIIIIIII" << name << "\n" << text  );
    g_ctrl->m_importedTexts.clear();
    g_ctrl->m_importedTexts[name] = text;
}

void added_building(int x, int y) {
    g_ctrl->addBuilding(Vec2(x, y));
}


}