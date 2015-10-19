#include "Document.h"
#include <map>

class Exception : public exception {
public:
    Exception(const string& s) :m_desc(s) {}
    virtual ~Exception() {}
    virtual const char* what() const { return m_desc.c_str(); }
    string m_desc;
};

Document::Document(QObject *parent)
    : QObject(parent)
{
    init_preset();
}


void makePath(vector<Triangle*>& tripath, Vec2 start, Vec2 end, vector<Vec2>& output);

void Document::init_preset()
{
    m_vtx = { new Vertex(0, 0, 0), new Vertex(1, 100, 0), new Vertex(2, 100, 100), 
              new Vertex(3, 0, 100),  new Vertex(4, 0, -100), new Vertex(5, 100, -100) };

    m_tri = { 
        new Triangle(m_vtx[3], m_vtx[1], m_vtx[2]),
        new Triangle(m_vtx[0], m_vtx[1], m_vtx[3]),
        new Triangle(m_vtx[0], m_vtx[5], m_vtx[1]), 
        new Triangle(m_vtx[0], m_vtx[4], m_vtx[5]), 
    };

    buildHalfEdge();

    runPath();

}

void Document::runPath()
{
    Vec2 start(70, 70);
    Vec2 end(30, -70);

    m_path.clear();
    makePath(m_tri, start, end, m_path);
}


typedef pair<Vertex*, Vertex*> VPair;

// map (from,to) -> halfedge
map<VPair, HalfEdge*> unpaired;

HalfEdge* seekPair(Vertex* v1, Vertex* v2, HalfEdge* add) {
    VPair ko(v2, v1);
    auto it = unpaired.find(ko);
    if (it != unpaired.end()) {
        HalfEdge* p = it->second;
        unpaired.erase(it);
        p->opposite = add;
        add->opposite = p;
    }
    VPair ks(v1, v2);
    if (unpaired.find(ks) != unpaired.end())
        throw Exception("Not all triangles are clockwise");
    unpaired[ks] = add;
    return nullptr;
}


void Document::buildHalfEdge()
{
    unpaired.clear();

    for(auto t: m_tri) 
    {
        HalfEdge* h0 = new HalfEdge();
        h0->tri = t;
        //h0->from = t->v[0];
        h0->to = t->v[1];
        t->h[0] = h0;

        HalfEdge* h1 = new HalfEdge();
        h1->tri = t;
        //h1->from = t->v[1];
        h1->to = t->v[2];
        t->h[1] = h1;

        HalfEdge* h2 = new HalfEdge();
        h2->tri = t;
        //h2->from = t->v[2];
        h2->to = t->v[0];
        t->h[2] = h2;

        h0->next = h1;
        h1->next = h2;
        h2->next = h0;

        seekPair(t->v[0], t->v[1], h0);
        seekPair(t->v[1], t->v[2], h1);
        seekPair(t->v[2], t->v[0], h2);
    }

    // go over half edges, create triangles links
    for (auto t : m_tri)
    {
        for(int i = 0; i < 3; ++i) {
            if (t->h[i]->opposite != nullptr)
                t->nei[i] = t->h[i]->opposite->tri;
        }
    }

}



inline float triarea2(const Vec2* a, const Vec2* b, const Vec2* c)
{
    const float ax = b->x - a->x;
    const float ay = b->y - a->y;
    const float bx = c->x - a->x;
    const float by = c->y - a->y;
    return bx*ay - ax*by;
}

float vdistsqr(const Vec2* a, const Vec2* b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    return dx*dx + dy*dy;
}

inline bool vequal(const Vec2* a, const Vec2* b)
{
    static const float eq = 0.001f*0.001f;
    return vdistsqr(a, b) < eq;
}

// http://digestingduck.blogspot.co.il/2010/03/simple-stupid-funnel-algorithm.html
// http://digestingduck.blogspot.co.il/2010/07/my-paris-game-ai-conference.html
void stringPull(vector<Vec2*> portalsRight, vector<Vec2*> portalsLeft, vector<Vec2>& output)
{

    // Init scan state
    int apexIndex = 0, leftIndex = 0, rightIndex = 0;
    auto *portalApex = portalsLeft[0];
    auto *portalLeft = portalsLeft[0];
    auto *portalRight = portalsRight[0];

    // Add start point.
    output.push_back(*portalApex);


    for (int i = 1; i < portalsRight.size(); ++i)
    {
        auto left = portalsLeft[i];
        auto right = portalsRight[i];

        // Update right vertex.
        if (triarea2(portalApex, portalRight, right) >= 0.0f)
        {
            if (vequal(portalApex, portalRight) || triarea2(portalApex, portalLeft, right) < 0.0f)
            {
                // Tighten the funnel.
                portalRight = right;
                rightIndex = i;
            }
            else
            {
                // Right over left, insert left to path and restart scan from portal left point.
                output.push_back(*portalLeft);

                // Make current left the new apex.
                portalApex = portalLeft;
                apexIndex = leftIndex;
                // Reset portal
                portalLeft = portalApex;
                portalRight = portalApex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;
                // Restart scan
                i = apexIndex;
                continue;
            }
        }

        // Update left vertex.
        if (triarea2(portalApex, portalLeft, left) <= 0.0f)
        {
            if (vequal(portalApex, portalLeft) || triarea2(portalApex, portalRight, left) > 0.0f)
            {
                // Tighten the funnel.
                portalLeft = left;
                leftIndex = i;
            }
            else
            {
                // Left over right, insert right to path and restart scan from portal right point.
                output.push_back(*portalRight);

                // Make current right the new apex.
                portalApex = portalRight;
                apexIndex = rightIndex;
                // Reset portal
                portalLeft = portalApex;
                portalRight = portalApex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;
                // Restart scan
                i = apexIndex;
                continue;
            }
        }
    }
    // Append last point to path.
    output.push_back(*portalsRight.back());
}



void commonVtx(Triangle* a, Triangle* b, Vertex** right, Vertex** left)
{
    Vertex *a0 = a->v[0], *a1 = a->v[1], *a2 = a->v[2];
    Vertex *b0 = b->v[0], *b1 = b->v[1], *b2 = b->v[2];
    if ((a0 == b0 && a1 == b2) || (a0 == b2 && a1 == b1) || (a0 == b1 && a1 == b0)) {
        *left = a0;
        *right = a1;
        return;
    }
    if ((a2 == b0 && a0 == b2) || (a2 == b2 && a0 == b1) || (a2 == b1 && a0 == b0)) {
        *left = a2;
        *right = a0;
        return;
    }
    if ((a1 == b0 && a2 == b2) || (a1 == b2 && a2 == b1) || (a1 == b1 && a2 == b0)) {
        *left = a1;
        *right = a2;
        return;
    }
    throw Exception("Unexpeteced common vtx");

}


void makePath(vector<Triangle*>& tripath, Vec2 start, Vec2 end, vector<Vec2>& output)
{
    vector<Vec2*> leftPath, rightPath;
    leftPath.reserve(tripath.size() + 1);
    rightPath.reserve(tripath.size() + 1);

    leftPath.push_back(&start);
    rightPath.push_back(&start);
    for (int i = 0; i < tripath.size() - 1; ++i) {
        Vertex *right, *left;
        commonVtx(tripath[i], tripath[i + 1], &right, &left);
        leftPath.push_back(&left->p);
        rightPath.push_back(&right->p);
    }

    leftPath.push_back(&end);
    rightPath.push_back(&end);

    stringPull(rightPath, leftPath, output);

}



