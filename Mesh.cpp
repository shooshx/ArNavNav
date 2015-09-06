#include "Mesh.h"
#include "Except.h"
#include <map>
#include <queue>

typedef pair<Vertex*, Vertex*> VPair;

// map (from,to) -> halfedge
map<VPair, HalfEdge*> unpaired;

void seekPair(Vertex* v1, Vertex* v2, HalfEdge* add) {
    VPair ko(v2, v1);
    auto it = unpaired.find(ko);
    if (it != unpaired.end()) {
        HalfEdge* p = it->second;
        unpaired.erase(it);
        p->opposite = add;
        add->opposite = p;
        return;
    }
    VPair ks(v1, v2);
    if (unpaired.find(ks) != unpaired.end())
        throw Exception("Not all triangles are clockwise");
    unpaired[ks] = add;
    return;
}


void Mesh::connectTri()
{
    unpaired.clear();
    m_perimiters.clear();
    
    m_he.clear();
    m_he.reserve(m_tri.size() * 3);

    for(auto t: m_tri) 
    {
        HalfEdge* h0 = addHe();
        h0->tri = t;
        h0->from = t->v[0];
        h0->to = t->v[1];
        h0->midPnt = (t->v[1]->p + t->v[0]->p) * 0.5f;
        t->h[0] = h0;

        HalfEdge* h1 = addHe();
        h1->tri = t;
        h1->from = t->v[1];
        h1->to = t->v[2];
        h1->midPnt = (t->v[2]->p + t->v[1]->p) * 0.5f;
        t->h[1] = h1;

        HalfEdge* h2 = addHe();
        h2->tri = t;
        h2->from = t->v[2];
        h2->to = t->v[0];
        h2->midPnt = (t->v[0]->p + t->v[2]->p) * 0.5f;
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

    // from the unpaired, make ordered perminiters
    while (!unpaired.empty())
    {
        m_perimiters.push_back(Polyline()); // TBD reserve
        Polyline& poly = m_perimiters.back();

        HalfEdge* h = unpaired.begin()->second;
        unpaired.erase(unpaired.begin());
        // find the adjacent unpaired
        HalfEdge* start = h;

        while(true) 
        {
            poly.m_d.push_back(h->from);
            // rotate around the vertex until finding the next unpaired
            while(h->next->opposite != nullptr) {
                h = h->next->opposite;
            }
            h = h->next;
            if (h == start)
                break;

            auto it = unpaired.find(VPair(h->from, h->to));
            if (it == unpaired.end())
                throw Exception("Stange half edge connection");
            unpaired.erase(it);
        }

    }

}


static float sign(const Vec2& p1, const Vec2& p2, const Vec2& p3)
{
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

static bool isPointInTri(const Vec2& pt, Triangle* t)//const Vec2& v1, const Vec2& v2, const Vec2& v3)
{
    bool b1, b2, b3;

    b1 = sign(pt, t->v[0]->p, t->v[1]->p) < 0.0f;
    b2 = sign(pt, t->v[1]->p, t->v[2]->p) < 0.0f;
    b3 = sign(pt, t->v[2]->p, t->v[0]->p) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
}


Triangle* Mesh::findContaining(const Vec2& p)
{
    for(auto t: m_tri) {
        if (isPointInTri(p, t))
            return t;
    }
    return nullptr;
}

struct PrioNode
{
    PrioNode(HalfEdge* _h, float _p) :h(_h), prio(_p) {}
    HalfEdge* h;
    float prio = 0.0f;
};

bool lessPrioNode(const PrioNode& a, const PrioNode& b) {
    return a.prio > b.prio;
}

float distm(const Vec2& a, const Vec2& b) {
    // can't use distSq since its not linear so it makes the heuristic bad
    return std::sqrt(distSq(a, b));
}

bool Mesh::edgesAstarSearch(const Vec2& startPos, const Vec2& endPos, Triangle* start, Triangle* end, vector<Triangle*>& corridor)
{
    if (start == end)
        return false;
    priority_queue<PrioNode, vector<PrioNode>, decltype(lessPrioNode)*> tq(lessPrioNode);
    HalfEdge* dummy = (HalfEdge*)0xff; // dummy cameFrom to mark the start edge
    vector<HalfEdge*> destEdges; // max 3 possible dest edges
    vector<float> destCost; // used when selecting the best dest reached out of possible 3

    // clear HalfEdge data
    for(auto& he: m_he)
        he.clearData();
    
    // set up start edges and dest edges. 
    // Start from end and go to start so its easy to connect the cameFrom pointers
    for(int i = 0; i < 3; ++i) 
    {
        auto sh = start->h[i];
        if (sh) {
            // fix mid point of start triangle to be closer to the real target
            sh->midPnt = project(startPos, sh->from->p, sh->to->p); // project to the line of the edge
            if (sh->opposite)
                sh->opposite->midPnt = sh->midPnt;
            destEdges.push_back(sh);
            destCost.push_back(FLT_MAX);
        }
        auto h = end->h[i]->opposite;
        if (h) {
            h->midPnt = project(endPos, h->from->p, h->to->p); // fix mid point of end triangle to be closer to the real target
            if (h->opposite)
                h->opposite->midPnt = h->midPnt;
            h->costSoFar = distm(endPos, h->midPnt);
            h->cameFrom = dummy;
            float heur = distm(h->midPnt, startPos);
            tq.push(PrioNode(h, h->costSoFar + heur));
        }
    }

    int destReached = 0;
    while (!tq.empty() ) 
    {
        PrioNode curn = tq.top();
        HalfEdge* cur = curn.h;
        tq.pop();    

        // was any dest edge reached?
        auto dsit = std::find(destEdges.begin(), destEdges.end(), cur);
        if (dsit != destEdges.end()) 
        {
            destCost[dsit - destEdges.begin()] = curn.prio;
            ++destReached;
            if (destReached > destEdges.size())
                break;
            continue; // need to find more ways to get there
        }

        // two ways to go from this triangle
        HalfEdge* next[2] = { cur->next->opposite, cur->next->next->opposite }; 
        for(int i = 0; i < 2; ++i) 
        {
            HalfEdge* n = next[i];
            if (!n)
                continue;
            float costToThis = cur->costSoFar + distm(cur->midPnt, n->midPnt);
            if (costToThis >= n->costSoFar) // need to update an edge that was already reached? 
                continue;                   // Equals avoid endless loop in degenerate triangulation
            n->costSoFar = costToThis;
            n->cameFrom = cur;
            float heur = n->costSoFar + distm(n->midPnt, startPos);
            tq.push(PrioNode(n, heur));
        }
    }
    if (destReached == 0)
        return false;

    auto dit = min_element(destCost.begin(), destCost.end());
    HalfEdge *h = destEdges[dit - destCost.begin()];

    // make it in reverse order
    while (h != dummy) {
        corridor.push_back(h->tri);
        h = h->cameFrom;
    }
    // the end triangle doesn't have any halfedges that are part of the the corridor so just add it
    corridor.push_back(end);
    return true;
}



inline float triarea2(const Vertex* a, const Vertex* b, const Vertex* c)
{
    const float ax = b->p.x - a->p.x;
    const float ay = b->p.y - a->p.y;
    const float bx = c->p.x - a->p.x;
    const float by = c->p.y - a->p.y;
    return bx*ay - ax*by;
}

float vdistsqr(const Vertex* a, const Vertex* b)
{
    float dx = a->p.x - b->p.x;
    float dy = a->p.y - b->p.y;
    return dx*dx + dy*dy;
}

inline bool vequal(const Vertex* a, const Vertex* b)
{
    static const float eq = 0.001f*0.001f;
    return vdistsqr(a, b) < eq;
}

// http://digestingduck.blogspot.co.il/2010/03/simple-stupid-funnel-algorithm.html
// http://digestingduck.blogspot.co.il/2010/07/my-paris-game-ai-conference.html
void PathMaker::stringPull(vector<Vertex*> portalsRight, vector<Vertex*> portalsLeft)
{

    // Init scan state
    int apexIndex = 0, leftIndex = 0, rightIndex = 0;
    auto *portalApex = portalsLeft[0];
    auto *portalLeft = portalsLeft[0];
    auto *portalRight = portalsRight[0];

    // Add start point.
    //m_outputCall(portalApex);


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
                m_outputCall(portalLeft);

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
                m_outputCall(portalRight);

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
    m_outputCall(portalsRight.back());
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


void PathMaker::makePath(vector<Triangle*>& tripath, const Vec2& start, const Vec2& end)
{
    if (tripath.size() == 0)
        return;
    Vertex startDummy(-1, start), endDummy(-1, end);

    vector<Vertex*> leftPath, rightPath;
    leftPath.reserve(tripath.size() + 1);
    rightPath.reserve(tripath.size() + 1);

    leftPath.push_back(&startDummy);
    rightPath.push_back(&startDummy);
    for (int i = 0; i < tripath.size() - 1; ++i) {
        Vertex *right, *left;
        commonVtx(tripath[i], tripath[i + 1], &right, &left);
        leftPath.push_back(left);
        rightPath.push_back(right);
    }

    leftPath.push_back(&endDummy);
    rightPath.push_back(&endDummy);

    stringPull(rightPath, leftPath);

}
