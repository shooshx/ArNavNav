#include "Document.h"
#include "BihTree.h"
#include "Agent.h"
#include <iostream>
#include <sstream>


Document::Document()
{
    //init_preset();
    init_test();

    //init_tri();

    //init_circle();
    //init_grid();

    for(int i = 0;i < 100; ++i)
        m_markers.push_back(new Vertex(0, Vec2(-200, -200)));

}

void Document::init_test()
{
    auto gend = addGoal(Vec2(-200, 0), 20, GOAL_POINT);
    for(int i = 0; i < 1 ; ++i)
    {
        addAgent(Vec2(0, -140 + 50*i), gend);
    }    

}

static ostream& operator<<(ostream& os, const Vec2& p) {
    os << p.x << "," << p.y;
    return os;
}

int runTriC(const string& cmd, vector<Vec2>& out);


//Vec2 g1,g2;

class SubGoalFromSegment : public ISubGoalMaker
{
public:
    SubGoalFromSegment(Segment* s) :m_seg(s) 
    {}
    virtual void makeSubGoal(float keepDist, const Vec2& goingTo, Plan& addto) {
        Vec2 a, b;
        m_seg->justExtPoints(keepDist, &a, &b); // TBD superflous b
        Vec2 outv = a - m_seg->a; // from segment point outside
        Vec2 toNext = goingTo - m_seg->a;
        //g1 = outv; g2 = toNext;
        bool rev = det(outv, toNext) > 0;
        addto.addSeg(a, m_seg->dpa, rev); // always take the first, that's how they are built
        // dpa is not normalized, but it's close to 1 and the length of the SubGoalSegment does not need to be accurate
    }

    virtual Vec2 makePathRef(float keepDist) {
        Vec2 a, b;
        m_seg->justExtPoints(keepDist, &a, &b); // TBD superflous b
        return a;
    }

    Segment* m_seg;
};


class SubGoalFromPointSeg : public ISubGoalMaker
{
public:
    SubGoalFromPointSeg(PointSegment* ps) :m_pnseg(ps)
    {}
    virtual void makeSubGoal(float keepDist, const Vec2& goingTo, Plan& addto) 
    {
        Vec2 a, b;
        m_pnseg->justExtPoints(keepDist, &a, &b);
        // don't know which order to put them, middle will be enough?
        Vec2 mid = (a + b)*0.5f - m_pnseg->b;
        Vec2 toNext = goingTo - m_pnseg->b;
        if (det(mid, toNext) < 0) { // check which side of the mid line we're coming from and decide what order should the points be
            addto.addSeg(a, m_pnseg->dpa, false);
            addto.addSeg(b, m_pnseg->dpb, false);
        }  // dpa,dpb are not normalized, but close enough, see above
        else {
            addto.addSeg(b, m_pnseg->dpb, true); // rev true since if we're coming from this size, determining the isPassed half-plane is reveresed
            addto.addSeg(a, m_pnseg->dpa, true);
        }
        //addto.push_back( SubGoal(mid) );
    }

    virtual Vec2 makePathRef(float keepDist)
    {
        Vec2 a, b;
        m_pnseg->justExtPoints(keepDist, &a, &b);
        Vec2 mid = (a + b)*0.5f;
        return mid;
    }

    PointSegment* m_pnseg;
};


#define SQRT_2 (1.4142135623730950488016887242097f)

#define ANTI_OVERLAP_FACTOR 0.0 //0.1


// return the intersection point of lines L1=a+tv L2=b+ku
Vec2 lineIntersect(const Vec2& a, const Vec2& v, const Vec2& b, const Vec2& u)
{
    float k = (v.x*(b.y-a.y) + v.y*(a.x-b.x))/(u.x*v.y - u.y*v.x);
    return b+k*u;
}


class MultiSegMaker 
{
public:
    MultiSegMaker(vector<Vertex*>& v, Document* doc, MultiSegment* ms)
        :m_v(v), m_doc(doc), m_ms(ms)
    {}

    Segment* addSegment(const Vec2& a, const Vec2& b, const Vec2& dpa, const Vec2& dpb, int vtxIndex)
    {
        auto s = new Segment(a, b, dpa, dpb, m_doc->m_objs.size(), m_ms);
        m_doc->m_objs.push_back(s);
        if (vtxIndex >= 0)
            m_doc->m_seggoals[vtxIndex] = new SubGoalFromSegment(s);
        return s;
    }
    void addPSegment(const Vec2& b, const Vec2& dpa, const Vec2& dpb, int vtxIndex)
    {
        auto ps = new PointSegment(b, dpa, dpb, m_doc->m_objs.size());
        m_doc->m_objs.push_back(ps);
        m_doc->m_seggoals[vtxIndex] = new SubGoalFromPointSeg(ps);
    }

    void makeSegments()
    {
        Segment beforeFirst;
        Segment* prevSeg = &beforeFirst;
        for(int i = 0; i < m_v.size(); ++i)
        {
            const Vec2& a = get(i - 1);
            const Vec2& b = get(i);
            const Vec2& c = get(i + 1);
            Vec2 ab = b - a;
            Vec2 bc = c - b;
            Vec2 nab = normalize(ab);
            Vec2 nbc = normalize(bc);
            Vec2 dp1 = Vec2(-nab.y, nab.x); // perp to ab
            Vec2 dp2 = Vec2(-nbc.y, nbc.x); // perp to bc
            if (dot(ab, bc) < 0) // sharp angle - use the 45 points and add a segment between
            {
                Vec2 dpa = dp1 - nab;
                Vec2 dpb1 = dp1 + nab;
                Vec2 dpb1_m = dp1 + nab * (1.0 + ANTI_OVERLAP_FACTOR); 
                // slightly more, to have overlap between the segment VO and the point VO to avoid floating point problems

                Vec2 dpb2 = dp2 - nbc;
                Vec2 dpb2_m = dp2 - nbc * (1.0 + ANTI_OVERLAP_FACTOR);
                Vec2 dpc = dp2 + nbc;

                prevSeg->dpb = dpb1_m;
             //   addSegment(a, b, dpa, dpb1_m);
                addPSegment(b, dpb1, dpb2, m_v[i]->index);
                prevSeg = addSegment(b, c, dpb2_m, dpc, -1); // PointSegment takes precedence for representing this vertex in m_seggoals
            }
            else
            {
                Vec2 near_b1 = b + dp1; // near b with distanct perpendicular to ab
                Vec2 near_b2 = b + dp2; // near b with distanct perpendicular to bc
                Vec2 mid = lineIntersect(near_b1, nab, near_b2, nbc);
                mid = mid - b;

                Vec2 amid = normalize(nab - nbc) * SQRT_2;
                prevSeg->dpb = mid + nab * ANTI_OVERLAP_FACTOR; // avoid overlap
                prevSeg = addSegment(b, c, mid, mid, m_v[i]->index);
            }
            // TBD exactly 0

        }
        prevSeg->dpb = beforeFirst.dpb;

    }

    const Vec2& get(int i) {
        int sz = m_v.size();
        return m_v[(i + sz)% sz]->p;
    }

    vector<Vertex*>& m_v; // vertices of polyline
    Document* m_doc;
    MultiSegment* m_ms;

};


void runTri(MapDef* mapdef, Mesh& out);

// add another set of vertices to the mesh with this radius, if its the first time we see it
void Document::addAgentRadius(float radius)
{
    if (m_mesh.m_vtx.empty())
        return;
    if (m_mesh.m_altVtxPosByRadius.find(radius) != m_mesh.m_altVtxPosByRadius.end())
        return;
    vector<Vec2>& altVtx = m_mesh.m_altVtxPosByRadius[radius];
    altVtx.resize(m_mesh.m_vtx.size());
    for(int i = 0; i < m_mesh.m_vtx.size(); ++i) {
        if (m_seggoals[i] != nullptr)
            altVtx[i] = m_seggoals[i]->makePathRef(radius);
    }
}

bool checkSelfIntersect(vector<Vec3>& vtx, vector<int>& pl);

void Document::runTriangulate()
{
   /* {
        vector<Vec3> vtx;
        for(auto& v: m_mapdef.m_vtx) 
            vtx.push_back(Vec3(v->p.x, 0, v->p.y));
        checkSelfIntersect(vtx, m_mapdef.m_pl[0].m_di);
        return;
    }*/
    

    vector<Vec2> gt;
    m_mesh.clear();
    runTri(&m_mapdef, m_mesh);

    m_mesh.connectTri();

    clearObst();

    m_multisegs.clear();
    m_multisegs.resize(m_mesh.m_perimiters.size()); // elements will not change address
    m_seggoals.clear();
    m_seggoals.resize(m_mesh.m_vtx.size());

    for(int i = 0; i < m_mesh.m_perimiters.size(); ++i)
    {
        auto& poly = m_mesh.m_perimiters[i];
        int sz = poly.m_d.size();
        MultiSegMaker ms(poly.m_d, this, &m_multisegs[i]); 
        ms.makeSegments();
    }

    // redo the radiuses
    m_mesh.m_altVtxPosByRadius.clear();
    vector<float> possibleRadiuses;
    for(auto agent: m_agents)
        possibleRadiuses.push_back(agent->m_radius);
    for(float radius: possibleRadiuses) {
        addAgentRadius(radius);
    }

    // set segment markers
    int cnt = 0;
    for(auto* obj: m_objs) {
        auto* seg = dynamic_cast<Segment*>(obj);
        if (seg == nullptr)
            continue;
        Vec2 p1, p2;
        seg->spanningPoints(m_prob->m_position, 15, &p1, &p2);

        /*if (cnt < 97) {
            m_markers[cnt++]->p = p1;
            m_markers[cnt++]->p = p2;
        }*/
    }



    //------------------------------------

    for(auto agent: m_agents)
    {
        updatePlan(agent);
    }
}


void Document::updatePlan(Agent* agent)
{
    if (m_mesh.m_vtx.empty())
        return;
    if (!agent->m_endGoalPos.p.isValid())
        return;
    const Vec2& startp = agent->m_position;
    const Vec2& endp = agent->m_endGoalPos.p;

    // find start and end triangles
    auto it = m_mesh.m_altVtxPosByRadius.find(agent->m_radius);
    CHECK(it != m_mesh.m_altVtxPosByRadius.end(), "unexpected radius");
    auto posReference = it->second;
    Triangle* startTri = m_mesh.findContaining(startp, posReference);
    Triangle* endTri = m_mesh.findContaining(endp, posReference);

    agent->m_plan.clear();
    if (!endTri || !startTri || startTri == endTri) 
    {
        agent->m_plan.setEnd(endp, agent->m_endGoalPos.radius);
        agent->m_indexInPlan = 0;
        agent->m_curGoalPos = agent->m_plan.m_d[0];
        return;
    }

    // find corridor
    vector<Triangle*> corridor;
    if (m_mesh.edgesAstarSearch(startp, endp, startTri, endTri, corridor))
    {
        //for(auto* t: corridor)
        //    if (t->highlight == 0)
        //        t->highlight = 3;

        agent->m_plan.reserve(corridor.size() * 2); // size of the corridor is the max it can get to, every triangle can add 2 point if the angle is sharp

        // male path from corridor
        int prevVtxIndex = -2;
        vector<Vertex*> planSketch;
        PathMaker::TOutputCallback outf([&](Vertex* v) {
            if (v->index == prevVtxIndex)
                return; // string pull may produce the same vertex multiple times, ignore it
            prevVtxIndex = v->index;
            planSketch.push_back(v);
        });
        PathMaker::TGetPosCallback posf([&](Vertex* v)->Vec2 {
            if (v->index < 0) { // means its the end dummy vertex
                return v->p;
            }
            auto* subGoalMaker = m_seggoals[v->index];
            if (subGoalMaker == nullptr)
                return v->p; // vertex that is not part of a parimiter
                             //CHECK(subGoalMaker != nullptr, "null subGoalMaker");
            return subGoalMaker->makePathRef(agent->m_radius);
        });
        PathMaker pm(outf, posf);
        pm.makePath(corridor, startp, endp);

        // make the actual plan when all vertices are known since we need to reference the next vertex
        //Vec2 prevInPath = startp;
        for(int i = 0; i < planSketch.size(); ++i) 
        {
            Vertex* v = planSketch[i];
            if (v->index < 0) { // means its the end dummy vertex
                agent->m_plan.setEnd(v->p, agent->m_endGoalPos.radius);
            }
            else {
                auto* subGoalMaker = m_seggoals[v->index];
                Vec2 nextPosInPath;
                if (subGoalMaker == nullptr)
                    continue; // vertex that is not part of a parimiter

                //CHECK(subGoalMaker != nullptr, "null subGoalMaker");

                int nextIndex = planSketch[i+1]->index;
                if (nextIndex < 0 || m_seggoals[nextIndex] == nullptr) // vertex that is not part of a parimiter
                    nextPosInPath = planSketch[i+1]->p;
                else 
                    nextPosInPath = m_seggoals[nextIndex]->makePathRef(agent->m_radius);

                subGoalMaker->makeSubGoal(agent->m_radius, nextPosInPath, agent->m_plan);

            }   

        }

        //for(auto& sg: agent->m_plan)
        //    cout << sg.p << "  ";

        // set agent to the start of the plan
        //agent->m_indexInPlan = agent->m_plan.m_d.size()-1; // end of the plan, disables plan
        agent->m_indexInPlan = 0;
        agent->m_curGoalPos = agent->m_plan.m_d[agent->m_indexInPlan];
    }
    else 
    { // go in the direction but never reach it
        agent->m_plan.setEnd(endp, agent->m_endGoalPos.radius);
        agent->m_indexInPlan = 0;
        agent->m_curGoalPos = agent->m_plan.m_d[agent->m_indexInPlan];
    }


}

void Document::clearAllObj()
{
    for(auto* obj: m_objs)
        delete obj;
    m_objs.clear();
    m_agents.clear();
    m_prob = nullptr;
}

void Document::clearObst()
{
    auto it = m_objs.begin();
    while (it != m_objs.end()) 
    {
        Object* o = *it;
        if (dynamic_cast<Segment*>(o) != nullptr || dynamic_cast<PointSegment*>(o) != nullptr) {
            delete o;
            it = m_objs.erase(it);
        }
        else
            ++it;
    }
}

Goal* Document::addGoal(const Vec2& p, float radius, EGoalType type) {
    int index = m_goals.size();
    // TBD - reclaim unused spaces
    auto ptr = new Goal(p, radius, type);
    m_goals.push_back(unique_ptr<Goal>(ptr));
    return ptr;
}
void Document::removeGoal(Goal* g) {
    auto it = m_goals.begin();
    while(it != m_goals.end()) {
        if (it->get() == g) 
            it = m_goals.erase(it);
        else
            ++it;
    }
}

Agent* Document::addAgent(const Vec2& pos, Goal* g, float radius, float prefSpeed, float maxSpeed)
{
    if (prefSpeed < 0)
        prefSpeed = 1.0f;
    if (maxSpeed < 0)
        maxSpeed = prefSpeed * 2;
    //OUT("addAgent " << pos << " " << g << " " << radius << " " << prefSpeed << " " << maxSpeed);
    Agent* a = new Agent(m_agents.size(), pos,
        (g != nullptr)?g->def : GoalDef(), // goal 
        radius * NEI_DIST_RADIUS_FACTOR, //30 for r=15, 15 for r=6, // 400 nei dist
        10, // max nei
        radius, // 15 radius
        prefSpeed, // pref speed
        maxSpeed); // max speed
               //a->m_velocity = Vec2(0, 1);
    m_objs.push_back(a);
    m_agents.push_back(a);
    if (m_agents.size() == 1)
        m_prob = a;
    if (g != nullptr) {
        g->agents.push_back(a);
    }

    addAgentRadius(radius);
    //OUT("addAgentDone");
    return a;
}




// ----------------------------------------------

/*
void Document::clearSegMinDist()
{
    for(auto& ms: m_multisegs)
        ms.clear();
}*/


bool Document::doStep(float deltaTime, bool doUpdate)
{
    if (deltaTime <= 0.0f)
        return false;
    if (m_objs.size() == 0)
        return true;

   // BihTree m_bihTree(m_objs);
    m_bihTree.build(m_objs);


    for(auto* agent: m_agents)
    {
        if (!agent->m_isMobile || agent->m_curGoalPos == nullptr)
            continue;
        agent->computePreferredVelocity(deltaTime);

        agent->computeNeighbors(m_bihTree);

        VODump* vod = nullptr;
        if (m_debugVoDump != nullptr && agent == m_prob)
            vod = m_debugVoDump;

        agent->computeNewVelocity(vod);
    }

    if (!doUpdate)
        return false;

    bool reachedGoals = true;
    for (auto* agent: m_agents) 
    {
        if (!agent->m_isMobile || agent->m_curGoalPos == nullptr)
            continue;

        agent->m_reached = agent->update(deltaTime);
        reachedGoals &= agent->m_reached;
    }

    //m_globalTime += deltaTime;
    return reachedGoals;
}


void Document::serialize(ostream& os)
{
    set<string> wroteImport;
    for(const auto& kv : m_mapdef.m_objModules) {
        auto& name = kv.second;
        if (wroteImport.count(name) > 0)
            continue;
        wroteImport.insert(name);
        os << "i," << kv.second << ",\n";
    }

    int count = 0;
    for(const auto& pl : m_mapdef.m_pl) {
        if (pl->m_d.size() == 0)
            continue;
        if (m_mapdef.m_objModules.find(pl.get()) != m_mapdef.m_objModules.end())
            continue; // this polyline was included in a file

        os << "p,\n";
        for(const auto& pv : pl->m_d) {
            os << "v," << pv->p.x << "," << pv->p.y << ",\n";
            ++count;
        }
    }

    map<Agent*, int> agentToGoal;
    for(int i = 0; i < m_goals.size(); ++i) {
        auto& g = m_goals[i];
        for(auto* ag: g->agents)
            agentToGoal[ag] = i;
        os << "g," << g->def.p.x << "," << g->def.p.y << "," << g->def.radius << "," << g->def.type << ",\n";
    }
    for(auto* agent: m_agents)
        os << "a," << agent->m_position.x << "," << agent->m_position.y << "," << ((agentToGoal.find(agent) != agentToGoal.end())?agentToGoal[agent]:-1)
           << "," << agent->m_velocity.x << "," << agent->m_velocity.y << "," << agent->m_radius << "," << agent->m_prefSpeed << "," << agent->m_maxSpeed << ",\n";

    //cout << "Saved " << count << " vertices, " << m_doc->m_mapdef.m_pl.size() << " polylines" << endl;
}

void Document::readStream(istream& is, map<string, string>& imported, const string& module)
{
    // see http://stackoverflow.com/questions/7302996/changing-the-delimiter-for-cin-c
    vector<ctype<char>::mask> bar(ctype<char>::classic_table(), ctype<char>::classic_table() + ctype<char>::table_size);
    bar[','] ^= ctype_base::space;
    is.imbue(locale(cin.getloc(), new ctype<char>(bar.data()))); // treat comma as a space, locale will delete it

    int count = 0;
    while (!is.eof()) 
    {
        string h;
        is >> h;
        //OUT("CMD `" << h << "`");
        if (h[0] == 'p') {
            m_mapdef.add(module);
        }
        else if (h[0] == 'v') {
            Vec2 v;
            is >> v.x >> v.y;
            if (is.fail())
                break;
            m_mapdef.addToLast(v, module);
            ++count;
        }
        else if (h[0] == 'g') {
            Vec2 v;
            float radius;
            int type;
            is >> v.x >> v.y >> radius >> type;
            if (is.fail())
                break;
            addGoal(v, radius, (EGoalType)type);
        }
        else if (h[0] == 'a') {
            Vec2 pos, vel;
            int goali = 0;
            float radius = 0.0f, ps = 0.0f, ms = 0.0f;
            is >> pos.x >> pos.y >> goali >> vel.x >> vel.y >> radius >> ps >> ms;
            if (is.fail())
                break;
            if (goali >= (int)m_goals.size() || radius <= 0.0f)
                break;
            auto* a = addAgent(pos, (goali >= 0)?(m_goals[goali].get()):nullptr, radius, ps, ms);
            a->m_velocity = vel;
        }
        else if (h[0] == 'e') {
            return;
        }
        else if (h[0] == 'i') {
            string name;
            is >> name;
            auto it = imported.find(name);
            CHECK(it != imported.end(), "Imported file not found " + name);
            istringstream iss(it->second);
            readStream(iss, imported, name);
        }
        else if (!h.empty()){
            OUT("Unknown CMD " << h);
        }

    }
}

void Document::deserialize(istream& is, map<string, string>& imported)
{
    m_mapdef.clear();
    clearAllObj();
    m_goals.clear();

    readStream(is, imported, "");

    //cout << "Read " << m_mapdef.m_pl[0].m_d.size() << endl;
}
