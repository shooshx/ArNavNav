#include "Document.h"
#include "BihTree.h"
#include "Agent.h"
#include <iostream>


Document::Document(QObject *parent)
    : QObject(parent)
{
    //init_preset();
    //init_test();

    init_tri();
}

static ostream& operator<<(ostream& os, const Vec2& p) {
    os << p.x << ", " << p.y;
    return os;
}

int runTriC(const string& cmd, vector<Vec2>& out);

void Document::init_tri()
{
    vector<Vec2> gt;

  /*  float d = 1;
    runTri("C:\\projects\\nav\\poly2tri\\data\\funny.dat", gt);
    for(int i = 0; i < gt.size(); i += 3) {
        m_mesh.addTri(gt[i]*d, gt[i + 1]*d, gt[i + 2]*d);
        //std::cout << gt[i] << "  " << gt[i + 1] << "  " << gt[i + 2] << endl;
    }*/
    
    m_start = new Vertex(0, Vec2(200, 0));
    m_end = new Vertex(1, Vec2(-200, 0));
}

void runTri(MapDef* mapdef, Mesh& out);

void Document::runTriangulate()
{
    vector<Vec2> gt;
    m_mesh.clear();
    runTri(&m_mapdef, m_mesh);

    m_mesh.connectTri();

    Triangle* startTri = m_mesh.findContaining(m_start->p);
    //if (startTri)
   //     startTri->highlight = 1;
    Triangle* endTri = m_mesh.findContaining(m_end->p);
    //if (endTri)
    //    endTri->highlight = 2;

    if (!endTri || !startTri)
        return;

    vector<Triangle*> corridor;
    m_mesh.edgesAstarSearch(m_start->p, m_end->p, startTri, endTri, corridor);

    for(auto* t: corridor)
        if (t->highlight == 0)
            t->highlight = 3;

    m_path.clear();
    Mesh::makePath(corridor, m_start->p, m_end->p, m_path);
}

void Document::init_test()
{
   // m_prob = new AABB(Vec2(0, 0), Vec2(100, 80), 0);

    Agent* a = new Agent(100, Vec2(0, -140),
        Vec2(0,140), // goal 
        400.0, //15.0, // nei dist
        10, // max nei
        15.0, // radius
        1.5f, // goal radius
        1.0f, // pref speed
        2.0f); // max speed
    a->m_velocity = Vec2(0, 1);
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

    m_objs.push_back(new AABB(Vec2(0, 0), Vec2(50,50), 0));

    //m_objs.push_back(new Circle(Vec2(-39, 0), 40.0, 0));
    //m_objs.push_back(new Circle(Vec2(39, 0), 40.0, 0));

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



