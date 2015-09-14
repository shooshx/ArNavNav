#include "Document.h"
#include "BihTree.h"
#include "Agent.h"
#include <iostream>


Document::Document(QObject *parent)
    : QObject(parent)
{
    //init_preset();
    init_test();

    //init_tri();

    //init_circle();
    //init_grid();

    for(int i = 0;i < 100; ++i)
        m_markers.push_back(new Vertex(0, Vec2(-200, -200)));

}

static ostream& operator<<(ostream& os, const Vec2& p) {
    os << p.x << "," << p.y;
    return os;
}

int runTriC(const string& cmd, vector<Vec2>& out);

void Document::init_tri()
{
    //vector<Vec2> gt;

  /*  float d = 1;
    runTri("C:\\projects\\nav\\poly2tri\\data\\funny.dat", gt);
    for(int i = 0; i < gt.size(); i += 3) {
        m_mesh.addTri(gt[i]*d, gt[i + 1]*d, gt[i + 2]*d);
        //std::cout << gt[i] << "  " << gt[i + 1] << "  " << gt[i + 2] << endl;
    }*/
    
    //m_start = new Vertex(0, Vec2(200, 0));
    //auto _end = ;
    m_goals.push_back(new Goal(Vec2(-200, 0)));

/*
    m_markers.push_back(new Vertex(0, Vec2(-100, -100)));
    m_markers.push_back(new Vertex(1, Vec2(-50,-50)));
    m_markers.push_back(new Vertex(2, Vec2(100,100)));
    m_markers.push_back(new Vertex(4, Vec2(50, 50)));
    m_markers.push_back(new Vertex(5, Vec2(0,0)));
    */

}

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


void Document::runTriangulate()
{
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

    m_mesh.m_altVtxPosByRadius.clear();

    vector<float> possibleRadiuses;
    for(auto agent: m_agents)
        possibleRadiuses.push_back(agent->m_radius);
    for(float radius: possibleRadiuses)
    {
        vector<Vec2>& altVtx = m_mesh.m_altVtxPosByRadius[radius];
        altVtx.resize(m_mesh.m_vtx.size());
        for(int i = 0; i < m_mesh.m_vtx.size(); ++i) {
            if (m_seggoals[i] != nullptr)
                altVtx[i] = m_seggoals[i]->makePathRef(radius);
        }
    }

    // set segment markers
    int cnt = 0;
    for(auto* obj: m_objs) {
        auto* seg = dynamic_cast<Segment*>(obj);
        if (seg == nullptr)
            continue;
        Vec2 p1, p2;
        seg->spanningPoints(m_prob->m_position, 15, &p1, &p2);
        m_markers[cnt++]->p = p1;
        m_markers[cnt++]->p = p2;
    }



    //------------------------------------

    for(auto agent: m_agents)
    {
        const Vec2& startp = agent->m_position;
        const Vec2& endp = agent->m_endGoalPos;
        
        auto it = m_mesh.m_altVtxPosByRadius.find(agent->m_radius);
        CHECK(it != m_mesh.m_altVtxPosByRadius.end(), "unexpected radius");
        auto posReference = it->second;
        Triangle* startTri = m_mesh.findContaining(startp, posReference);
        Triangle* endTri = m_mesh.findContaining(endp, posReference);

        if (!endTri || !startTri) {
            continue;
        }
        agent->m_plan.clear();
        if (startTri == endTri) 
        {
            agent->m_plan.setEnd(endp, agent->m_goalRadius);
            agent->m_indexInPlan = 0;
            agent->m_curGoalPos = agent->m_plan.m_d[0];
            continue;
        }

        vector<Triangle*> corridor;
        if (m_mesh.edgesAstarSearch(startp, endp, startTri, endTri, corridor))
        {

            //for(auto* t: corridor)
            //    if (t->highlight == 0)
            //        t->highlight = 3;

            agent->m_plan.reserve(corridor.size() * 2); // size of the corridor is the max it can get to, every triangle can add 2 point if the angle is sharp

           
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
                    agent->m_plan.setEnd(v->p, agent->m_goalRadius);
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

            //agent->m_indexInPlan = agent->m_plan.m_d.size()-1;
            agent->m_indexInPlan = 0;
            agent->m_curGoalPos = agent->m_plan.m_d[agent->m_indexInPlan];
        }


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

void Document::addAgent(const Vec2& pos, Goal* g)
{
    Agent* a = new Agent(m_agents.size(), pos,
        g->p, // goal 
        30.0, //30 for r=15, 15 for r=6, // 400 nei dist
        10, // max nei
        15.0, // 15 radius
        20, // goal radius
        5.0f, // pref speed
        7.0f); // max speed
               //a->m_velocity = Vec2(0, 1);
    m_objs.push_back(a);
    m_agents.push_back(a);
    if (m_agents.size() == 1)
        m_prob = a;
    g->agents.push_back(a);
}

void Document::init_test()
{
    auto gend = new Goal(Vec2(-200, 0));
    m_goals.push_back(gend);
    //m_start = new Vertex(0, Vec2(200, 0));

   // m_prob = new AABB(Vec2(0, 0), Vec2(100, 80), 0);
    for(int i = 0; i < 1 ; ++i)
    {
        addAgent(Vec2(0, -140 + 50*i), gend);
    }    

  /*  Agent* a2 = new Agent(100, Vec2(50, -140),
        Vec2(0,140), // goal 
        400.0, //15.0, // nei dist
        10, // max nei
        15.0, // radius
        1.5f, // goal radius
        1.0f, // pref speed
        2.0f); // max speed
    a2->m_velocity = Vec2(0, 1);
    m_objs.push_back(a2);*/

   // m_objs.push_back(new Circle(Vec2(0, 0), 50.0, 0));
   // m_objs.push_back(new Circle(Vec2(100, 0), 50.0, 0));

    //m_objs.push_back(new AABB(Vec2(0, 0), Vec2(50+30, 50+30), 0));

    //m_objs.push_back(new AABB(Vec2(0, 0), Vec2(150, 150), 1));

   // m_objs.push_back(new Segment(Vec2(0, -50), Vec2(50, 50), 1));
    //m_objs.push_back(new Segment(Vec2(50, 50), Vec2(50, 100), 1));

    //m_objs.push_back(new Circle(Vec2(-39, 0), 40.0, 0));
    //m_objs.push_back(new Circle(Vec2(39, 0), 40.0, 0));


}

#define TWO_PI (6.283185307179586f)

#define COUNT 10
#define ANG_OFFST 0

void Document::init_circle()
{
    float d = 1.0/COUNT;

    for (int i = 0; i < COUNT; ++i) 
    {
        Vec2 pos = 200.0f * Vec2(std::cos(d * i * TWO_PI + ANG_OFFST), std::sin(d * i * TWO_PI + ANG_OFFST));
        Goal *g = new Goal(-pos);
        m_goals.push_back(g);
        addAgent(pos, g); 
    }

    m_objs.push_back(new Circle(Vec2(0, 0), 50.0, 0));
}

void Document::init_grid()
{
    float d = 1.0/COUNT;

    for (int i = 0; i < COUNT; ++i) 
    {
        Vec2 pos(-200, -200+i*(400/COUNT));
        Goal *g = new Goal(pos+Vec2(400,0));
        m_goals.push_back(g);
        addAgent(pos, g); 
    }
    for (int i = 1; i < COUNT+1; ++i) 
    {
        Vec2 pos(-200+i*(400/COUNT), -200);
        Goal *g = new Goal(pos+Vec2(0,400));
        m_goals.push_back(g);
        addAgent(pos, g); 
    }


    //m_objs.push_back(new Circle(Vec2(0, 0), 50.0, 0));
}

/*
void Document::init_preset()
{
    m_prob = new Circle(Vec2(0, -40), 15, -1);
    m_objs.push_back(m_prob);

    float d[] = { 47.1101,-0.157709, 56.7234,7.76938, 41.4458,15.3028, 50.7229,25.4309, 38.6947,30.5933, 42.9773,41.8812, 30.2348,42.7637, 18.0205,42.1195, 21.6358,56.065, 8.88476,56.8912, -0.806088,47.2093, -6.18693,57.9651, -17.7255,53.7195, -18.9269,41.51, -30.5465,48.9946, -33.0645,37.3273, -45.3724,34.2746, -37.4965,22.3053, -46.3221,14.1752, -58.0199,11.368, -55.9429,-1.65047, -43.163,-6.57822, -56.571,-15.48, -50.854,-26.0287, -39.6585,-31.0833, -40.8543,-43.4756, -25.6486,-37.0539, -26.1224,-51.2389, -11.8896,-44.4822, -11.6569,-57.5834, 0.0261129,-47.9413, 7.87078,-58.0605, 12.4242,-44.817, 24.3505,-54.2656, 23.9572,-40.0356, 36.3315,-41.5397, 36.5995,-29.2206, 48.9593,-26.3898, 41.5958,-13.4563, 55.5355,-11.1551, }; //192

    int index = 0;
    for (int i = 0; i < _countof(d); i += 2 ) {
        Vec2 pos(d[i], d[i+1]);
        m_objs.push_back(new Circle(pos, 6, index++));
    }

}
*/

void Document::init_preset_grid()
{
  //  m_objs = { new Circle(0, 0, 30), new Circle(100, 0, 10), new Circle(100, 100, 10),
  //           new Circle(0, 100, 10), new Circle(0, -100, 10), new Circle(100, -100, 10) };

    int index = 0;

    m_prob = new Circle(Vec2(0, -40), 20, index++);
    //m_prob->color = QColor(0, 0, 255, 128);
    m_objs.push_back(m_prob);
    
    Vec2 dx(10, 15);
    Vec2 dy(-15, 10);
    Vec2 offs(0, -400);
    int matSize = 40;
    int radius = 5;
    
   /* Vec2 dx(20, 30);
    Vec2 dy(-30, 20);
    Vec2 offs(0, -400);
    int matSize = 20;
    int radius = 10;
    */
    for(int i = 0; i < matSize; ++i) {
        for(int j = 0; j < matSize; ++j) {
            Vec2 pos = dx * i + dy * j + offs;
            m_objs.push_back(new Circle(pos, radius, index++));
        }
    }

}

// ----------------------------------------------


void Document::clearSegMinDist()
{
    for(auto& ms: m_multisegs)
        ms.clear();
}


bool Document::doStep(float deltaTime, bool doUpdate)
{
    if (deltaTime <= 0.0f)
        return false;

    BihTree bihTree;
    bihTree.build(m_objs);


    for(auto* agent: m_agents)
    {
        if (!agent->m_isMobile || agent->m_curGoalPos == nullptr)
            continue;
        agent->computePreferredVelocity(deltaTime);

        clearSegMinDist();
        agent->computeNeighbors(bihTree);

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
        reachedGoals &= agent->update(deltaTime);
    }

    //m_globalTime += deltaTime;
    return reachedGoals;
}

