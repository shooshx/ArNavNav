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
}

static ostream& operator<<(ostream& os, const Vec2& p) {
    os << p.x << ", " << p.y;
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
    
    m_start = new Vertex(0, Vec2(200, 0));
    m_end = new Vertex(1, Vec2(-200, 0));

/*
    m_markers.push_back(new Vertex(0, Vec2(-100, -100)));
    m_markers.push_back(new Vertex(1, Vec2(-50,-50)));
    m_markers.push_back(new Vertex(2, Vec2(100,100)));
    m_markers.push_back(new Vertex(4, Vec2(50, 50)));
    m_markers.push_back(new Vertex(5, Vec2(0,0)));
    */
    for(int i = 0;i < 100; ++i)
        m_markers.push_back(new Vertex(0, Vec2(-200, -200)));

}

void runTri(MapDef* mapdef, Mesh& out);

#define SQRT_2 (1.4142135623730950488016887242097f)

#define ANTI_OVERLAP_FACTOR 0.1


// return the intersection point of lines L1=a+tv L2=b+ku
Vec2 lineIntersect(const Vec2& a, const Vec2& v, const Vec2& b, const Vec2& u)
{
    float k = (v.x*(b.y-a.y) + v.y*(a.x-b.x))/(u.x*v.y - u.y*v.x);
    return b+k*u;
}


class MultiSegment 
{
public:
    MultiSegment(vector<Vertex*>& v, Document* doc)
        :m_v(v), m_doc(doc)
    {}

    Segment* addSegment(const Vec2& a, const Vec2& b, const Vec2& dpa, const Vec2& dpb)
    {
        auto s = new Segment(a, b, dpa, dpb, m_doc->m_objs.size());
        m_doc->m_objs.push_back(s);
        return s;
    }
    void addPSegment(const Vec2& b, const Vec2& dpa, const Vec2& dpb)
    {
        m_doc->m_objs.push_back(new PointSegment(b, dpa, dpb, m_doc->m_objs.size()));
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
                addPSegment(b, dpb1, dpb2);
                prevSeg = addSegment(b, c, dpb2_m, dpc);
            }
            else
            {
                Vec2 near_b1 = b + dp1; // near b with distanct perpendicular to ab
                Vec2 near_b2 = b + dp2; // near b with distanct perpendicular to bc
                Vec2 mid = lineIntersect(near_b1, nab, near_b2, nbc);
                mid = mid - b;

                Vec2 amid = normalize(nab - nbc) * SQRT_2;
                prevSeg->dpb = mid + nab * ANTI_OVERLAP_FACTOR; // avoid overlap
                prevSeg = addSegment(b, c, mid, mid);
            }
            // TBD exactly 1

        }
        prevSeg->dpb = beforeFirst.dpb;

    }

    const Vec2& get(int i) {
        int sz = m_v.size();
        return m_v[(i + sz)% sz]->p;
    }

    vector<Vertex*>& m_v;
    Document* m_doc;

};



void Document::runTriangulate()
{
    vector<Vec2> gt;
    m_mesh.clear();
    runTri(&m_mapdef, m_mesh);

    m_mesh.connectTri();


    if (m_start && m_end)
    {
        Triangle* startTri = m_mesh.findContaining(m_start->p);
        //if (startTri)
        //     startTri->highlight = 1;
        Triangle* endTri = m_mesh.findContaining(m_end->p);
        //if (endTri)
        //    endTri->highlight = 2;

        if (endTri && startTri)
        {
            vector<Triangle*> corridor;
            m_mesh.edgesAstarSearch(m_start->p, m_end->p, startTri, endTri, corridor);

            for(auto* t: corridor)
                if (t->highlight == 0)
                    t->highlight = 3;

            m_path.clear();
            Mesh::makePath(corridor, m_start->p, m_end->p, m_path);
        }
    }
    // ---------------------

    clearObst();

    for(auto& poly: m_mesh.m_perimiters)
    {
        int sz = poly.m_d.size();
        MultiSegment ms(poly.m_d, this);
        ms.makeSegments();

/*        for(int i = 0; i < sz; ++i) 
        {
            Vertex* c = poly.m_d[(i - 1 + sz) % sz];
            Vertex* a = poly.m_d[i];
            Vertex* b = poly.m_d[(i + 1) % sz];

            Vec2 ca = a->p - c->p;
            Vec2 perp_ca = Vec2(ca.y, -ca.x).normalize() * 25;
            Vec2 near_c = c->p + perp_ca;
            
            
            Vec2 ab = b->p - a->p;
            Vec2 perp_ab = Vec2(ab.y, -ab.x).normalize() * 25;
            Vec2 near_a = a->p + perp_ab;

            Vec2 p = lineIntersect(near_c, ca, near_a, ab);

            //if (cnt < m_markers.size())
            //    m_markers[cnt++]->p = p;
            
            auto s = new Segment(a->p, b->p, i+1);
            m_objs.push_back(s);
            Vec2 p1, p2;
            s->spanningPoints(m_prob->m_position, 15, &p1, &p2);
            m_markers[cnt++]->p = p1;
            m_markers[cnt++]->p = p2;
        }*/
    }

    int cnt = 0;
    for(auto* obj: m_objs) {
        auto* seg = dynamic_cast<Segment*>(obj);
        if (seg == nullptr)
            continue;
        Vec2 p1, p2;
        seg->spanningPoints(m_prob->m_position, 30, &p1, &p2);
        m_markers[cnt++]->p = p1;
        m_markers[cnt++]->p = p2;
    }
}

void Document::clearObst()
{
    auto it = m_objs.begin();
    while (it != m_objs.end()) 
    {
        Object* o = *it;
        if (dynamic_cast<Agent*>(o) == nullptr) {
            delete o;
            it = m_objs.erase(it);
        }
        else
            ++it;
    }
}

void Document::init_test()
{
    m_end = new Vertex(1, Vec2(-200, 0));
    m_start = new Vertex(0, Vec2(200, 0));

   // m_prob = new AABB(Vec2(0, 0), Vec2(100, 80), 0);

    Agent* a = new Agent(100, Vec2(0, -140),
        Vec2(0,140), // goal 
        45.0, //30, 15.0, // 400 nei dist
        10, // max nei
        30.0, // 15 radius
        1.5f, // goal radius
        5.0f, // pref speed
        7.0f); // max speed
    //a->m_velocity = Vec2(0, 1);
    m_objs.push_back(a);
    m_prob = a;
    

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

    //m_objs.push_back(new Circle(Vec2(0, 0), 30.0, 0));

    //m_objs.push_back(new AABB(Vec2(0, 0), Vec2(50+30, 50+30), 0));

    //m_objs.push_back(new AABB(Vec2(0, 0), Vec2(50, 50), 1));

   // m_objs.push_back(new Segment(Vec2(0, -50), Vec2(50, 50), 1));
    //m_objs.push_back(new Segment(Vec2(50, 50), Vec2(50, 100), 1));

    //m_objs.push_back(new Circle(Vec2(-39, 0), 40.0, 0));
    //m_objs.push_back(new Circle(Vec2(39, 0), 40.0, 0));

    for(int i = 0;i < 100; ++i)
        m_markers.push_back(new Vertex(0, Vec2(-200, -200)));

}

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



