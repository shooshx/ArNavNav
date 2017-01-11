#include "Mesh.h"
#include "Except.h"
#include <map>
#include <queue>
#include <iostream>

#include "Agent.h"

using namespace std;

typedef pair<Vertex*, Vertex*> VPair;

// map (from,to) -> halfedge
map<VPair, HalfEdge*> unpaired; // TBD - global

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

    for(auto& t: m_tri) 
    {
        HalfEdge* h0 = addHe();
        h0->tri = &t;
        h0->from = t.v[0];
        h0->to = t.v[1];
        h0->_midPnt = (t.v[1]->p + t.v[0]->p) * 0.5f;
        h0->lengthSq = Vec2::distSq(h0->from->p, h0->to->p);
        h0->passToNextSq = distSqToProjectOrMax(h0->to->p, t.v[0]->p, t.v[2]->p);
        t.h[0] = h0;

        HalfEdge* h1 = addHe();
        h1->tri = &t;
        h1->from = t.v[1];
        h1->to = t.v[2];
        h1->_midPnt = (t.v[2]->p + t.v[1]->p) * 0.5f;
        h1->lengthSq = Vec2::distSq(h1->from->p, h1->to->p);
        h1->passToNextSq = distSqToProjectOrMax(h1->to->p, t.v[1]->p, t.v[0]->p);
        t.h[1] = h1;

        HalfEdge* h2 = addHe();
        h2->tri = &t;
        h2->from = t.v[2];
        h2->to = t.v[0];
        h2->_midPnt = (t.v[0]->p + t.v[2]->p) * 0.5f;
        h2->lengthSq = Vec2::distSq(h2->from->p, h2->to->p);
        h2->passToNextSq = distSqToProjectOrMax(h2->to->p, t.v[2]->p, t.v[1]->p);
        t.h[2] = h2;

        h0->next = h1;
        h1->next = h2;
        h2->next = h0;

        seekPair(t.v[0], t.v[1], h0);
        seekPair(t.v[1], t.v[2], h1);
        seekPair(t.v[2], t.v[0], h2);
    }

    // go over half edges, create triangles links
    for (auto& t : m_tri)
    {
        for(int i = 0; i < 3; ++i) {
            if (t.h[i]->opposite != nullptr)
                t.nei[i] = t.h[i]->opposite->tri;
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

    for(auto& he: m_he) {
        he.curMidPntPtr = &he._midPnt;
    }

    // perminiters CW or CCW? http://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
    for(auto& pr: m_perimiters)
    {
        float sum = 0;
        int sz = pr.m_d.size();
        for(int i = 0; i < sz; ++i) {
            auto* v0 = pr.m_d[i];
            auto* v1 = pr.m_d[(i+1) % sz ];
            float segmentArea = (v0->p.x - v1->p.x)*(v0->p.y + v1->p.y);
            sum += segmentArea;
        }
        pr.m_isCW = sum > 0;
    }
}


static float sign(const Vec2& p1, const Vec2& p2, const Vec2& p3)
{
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

static bool isPointInTri(const Vec2& pt, const Triangle& t, vector<Vec2>& posRef)//const Vec2& v1, const Vec2& v2, const Vec2& v3)
{
    Vec2 a = posRef[t.v[0]->index];
    Vec2 b = posRef[t.v[1]->index];
    Vec2 c = posRef[t.v[2]->index];

    bool b1, b2, b3;

    b1 = sign(pt, a, b) < 0.0f;
    b2 = sign(pt, b, c) < 0.0f;
    b3 = sign(pt, c, a) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
}

// take vertex positions from posRef
Triangle* Mesh::findContaining(const Vec2& p, vector<Vec2>& posRef)
{
    for(auto& t: m_tri) {
        if (isPointInTri(p, t, posRef))
            return &t;
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

bool Mesh::edgesAstarSearch(const Vec2& startPos, const Vec2& endPos, Triangle* start, Triangle* end, vector<Triangle*>& corridor, float agetnRadius)
{
    if (start == end)
        return false;
    priority_queue<PrioNode, vector<PrioNode>, decltype(lessPrioNode)*> tq(lessPrioNode);
    HalfEdge* dummy = (HalfEdge*)0xff; // dummy cameFrom to mark the start edge
    vector<HalfEdge*> destEdges; // max 3 possible dest edges
    destEdges.reserve(3);
    vector<float> destCost; // used when selecting the best dest reached out of possible 3
    destCost.reserve(3);


    vector<Vec2> midPntOverride;
    midPntOverride.reserve(6); // don't reallocate, that will change addresses
    
    // set up start edges and dest edges. 
    // Start from end and go to start so its easy to connect the cameFrom pointers
    for(int i = 0; i < 3; ++i) 
    {
        auto sh = start->h[i];
        if (sh->opposite) // if it doesn't have an opposite, it can't be reached so its not a destination
        { 
            // fix mid point of start triangle to be closer to the real target
            midPntOverride.push_back(project(startPos, sh->from->p, sh->to->p)); // project to the line of the edge
            sh->curMidPntPtr = &midPntOverride.back();
            sh->opposite->curMidPntPtr = sh->curMidPntPtr;
            destEdges.push_back(sh);
            destCost.push_back(FLT_MAX);
            //cout << "END " << sh->index << endl;
        }
        auto h = end->h[i]->opposite;
        if (h) 
        {
            midPntOverride.push_back(project(endPos, h->from->p, h->to->p)); // fix mid point of end triangle to be closer to the real target
            h->curMidPntPtr = &midPntOverride.back();
            if (h->opposite)
                h->opposite->curMidPntPtr = h->curMidPntPtr;
            h->costSoFar = distm(endPos, *h->curMidPntPtr);
            h->cameFrom = dummy;
            float heur = distm(*h->curMidPntPtr, startPos);
            tq.push(PrioNode(h, h->costSoFar + heur));
            //cout << "START " << h->index << endl;
        }
    }

    // main loop
    int destReached = 0;

    // passing an edge case the diameter to check needs to be multiplied by SQRT_2 since that's the worst case for the edge points of a segment
    // this means the narrowest passage an agent can pass through is larger than its diameter
    // this limitation stems from the fact that the VOs we make for a polyline does not have round corners (which will be hard to simulate) see narrow_worst_cast.txt
    float edgeLenCheck = sqr(agetnRadius * SQRT_2 * 2);
    // in the point-to-segment case the distance to check is one half radius*SQRT_2 - the half near the point
    // and the other half is radius*nei_dist since that's the distance where an agent find it going to bump into a wall and stop
    // (the simulation of a "chopped" VO)
    float triMidCheck = sqr(agetnRadius * SQRT_2 + agetnRadius * NEI_DIST_RADIUS_FACTOR);
    while (!tq.empty() ) 
    {
        PrioNode curn = tq.top();
        HalfEdge* cur = curn.h;
        tq.pop();    
        //cout << "POPED " << cur->index << endl;

        // was any dest edge reached?
        auto dsit = std::find(destEdges.begin(), destEdges.end(), cur);
        if (dsit != destEdges.end()) 
        {
            destCost[dsit - destEdges.begin()] = curn.prio;
            ++destReached;
            //cout << "  Reached " << cur->index << " " << curn.prio << endl;
            if (destReached > destEdges.size())
                break;
            continue; // need to find more ways to get there
        }

        // two ways to go from this triangle
        HalfEdge* next[2] = { cur->next->opposite, cur->next->next->opposite }; 
        float widthSqToNext[2] = { cur->passToNextSq, cur->next->next->passToNextSq };

        for(int i = 0; i < 2; ++i) 
        {
            HalfEdge* n = next[i];
            if (!n)
                continue;
            if (n->lengthSq < edgeLenCheck) // edge is too narrow to pass through 
                continue;
            if (widthSqToNext[i] < triMidCheck) // or width of triangle to narrow (opposite since we want the dist inside the triangle we're in)
                continue;

            float costToThis = cur->costSoFar + distm(*cur->curMidPntPtr, *n->curMidPntPtr);
            if (costToThis >= n->costSoFar) // need to update an edge that was already reached? 
                continue;                   // Equals avoid endless loop in degenerate triangulation

           /* if (n->costSoFar == FLT_MAX)
                cout << "  NewCost " << n->index << " =" << costToThis << endl;
            else
                cout << "  UpdateC " << n->index << " =" << n->costSoFar << " -> " << costToThis << endl;
            */
            n->costSoFar = costToThis;
            n->cameFrom = cur;
            float heur = n->costSoFar + distm(*n->curMidPntPtr, startPos);
            tq.push(PrioNode(n, heur));
        }
    }

    bool reached = (destReached != 0);
    if (reached)
    {
        auto dit = min_element(destCost.begin(), destCost.end());
        HalfEdge *firsth = destEdges[dit - destCost.begin()];
        HalfEdge *h = firsth;
        // find the length of the corridor
        int len = 0;
        while (h != dummy) {
            ++len;
            h = h->cameFrom;
        }
        h = firsth;
        corridor.reserve(len + 1);
        // make it in reverse order
        while (h != dummy) {
            corridor.push_back(h->tri);
            h = h->cameFrom;
        }
        // the end triangle doesn't have any halfedges that are part of the the corridor so just add it
        corridor.push_back(end);
    }

    // clear HalfEdge data
    for(auto& he: m_he)
        he.clearData();

    return reached;
}




inline float triarea2(const VtxWrap& a, const VtxWrap& b, const VtxWrap& c)
{
    const double ax = b.p.x - a.p.x;
    const double ay = b.p.y - a.p.y;
    const double bx = c.p.x - a.p.x;
    const double by = c.p.y - a.p.y;
    return (float)(bx*ay - ax*by);
}


float vdistsqr(const VtxWrap& a, const VtxWrap& b)
{
    float dx = a.p.x - b.p.x;
    float dy = a.p.y - b.p.y;
    return dx*dx + dy*dy;
}

inline bool vequal(const VtxWrap& a, const VtxWrap& b)
{
    static const float eq = 0.001f*0.001f;
    return vdistsqr(a, b) < eq;
}

// http://digestingduck.blogspot.co.il/2010/03/simple-stupid-funnel-algorithm.html
// http://digestingduck.blogspot.co.il/2010/07/my-paris-game-ai-conference.html
void PathMaker::stringPull(const vector<VtxWrap>& portalsRight, const vector<VtxWrap>& portalsLeft)
{
    // Init scan state
    int apexIndex = 0, leftIndex = 0, rightIndex = 0;
    auto portalApex = portalsLeft[0];
    auto portalLeft = portalsLeft[0];
    auto portalRight = portalsRight[0];


    // Add start point.
    //m_outputCall(portalApex.v);


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
                m_outputCall(portalLeft.v);

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
                m_outputCall(portalRight.v);

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
    m_outputCall(portalsRight.back().v);
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


void PathMaker::makePath(const vector<Triangle*>& tripath, const Vec2& start, const Vec2& end)
{
    if (tripath.size() == 0)
        return;
    m_startDummy = Vertex(-1, start);
    m_endDummy = Vertex(-1, end);
    VtxWrap startWrap(&m_startDummy, start), endWrap(&m_endDummy, end);

    vector<VtxWrap> leftPath, rightPath;
    leftPath.reserve(tripath.size() + 1);
    rightPath.reserve(tripath.size() + 1);

    leftPath.push_back(startWrap);
    rightPath.push_back(startWrap);
    for (int i = 0; i < tripath.size() - 1; ++i) {
        Vertex *right, *left;
        commonVtx(tripath[i], tripath[i + 1], &right, &left);
        leftPath.push_back(VtxWrap(left, m_getPos(left)));
        rightPath.push_back(VtxWrap(right, m_getPos(right)));
    }

    leftPath.push_back(endWrap);
    rightPath.push_back(endWrap);

    stringPull(rightPath, leftPath);

}

void iminmax(float a, float b, float* mn, float* mx) {
    if (a < b) {
        *mn = a;
        *mx = b;
    }
    else {
        *mn = b;
        *mx = a;
    }
}

struct BHalfEdge
{
    Vertex *from, *to;
};

// if two consecutively added points are on the same axis aligned line, we can dispose of the previous one in favor of the next one
void MapDef::popIfLinear(Vertex* nextv) {
    Polyline* pl = m_pl.back().get();
    int sz = pl->m_d.size();
    if (sz < 2) 
        return;
    Vec2& beflast = pl->m_d[sz - 2]->p;
    Vec2& last = pl->m_d[sz - 1]->p;
    Vec2& next = nextv->p;
    if ((beflast.x == last.x && last.x == next.x) || (beflast.y == last.y && last.y == next.y)) {
        pl->m_d.pop_back();
        pl->m_di.pop_back();
    }
}

void MapDef::delFirstIfLinear() {
    Polyline* pl = m_pl.back().get();
    int sz = pl->m_d.size();
    if (sz < 2) 
        return;
    Vec2& beflast = pl->m_d[sz - 1]->p;
    Vec2& last = pl->m_d[0]->p;
    Vec2& next = pl->m_d[1]->p;
    if ((beflast.x == last.x && last.x == next.x) || (beflast.y == last.y && last.y == next.y)) {
        pl->m_d.erase(pl->m_d.begin());
        pl->m_di.erase(pl->m_di.begin());
    }
}

void MapDef::makeBoxPoly()
{
    // remove polylines added previously by boxes
    auto it = m_pl.begin();
    while (it != m_pl.end()) {
        if ((*it)->m_fromBox)
            it = m_pl.erase(it);
        else
            ++it;
    }
    m_boxAddedVtx.clear();

    vector<BHalfEdge> bh;
    bh.reserve(m_bx.size() * 4);
    vector<Vertex*> vtx;
    vtx.reserve(m_bx.size() * 4);

    map<pair<float, float>, Vertex*> uniqvtx;
    auto checkUniq = [&](Vertex* v)->Vertex* {
        auto pr = make_pair(v->p.x, v->p.y);
        auto it = uniqvtx.find(pr);
        if (it != uniqvtx.end()) 
            return it->second;    
        uniqvtx[pr] = v;
        return v;
    };

    // create half edges for all boxes
    for(int i = 0; i < m_bx.size(); ++i) 
    {
        auto& box = *m_bx[i];
        const Vec2& a1 = box.v[0]->p;
        const Vec2& a2 = box.v[2]->p;
        Vec2 d = a1 - a2;
        // check empty box
        if (d.x == 0 || d.y == 0)
            continue; // empty box
        // check its not intersecting with other boxes
        float amnx, amxx, amny, amxy;
        iminmax(a1.x, a2.x, &amnx, &amxx);
        iminmax(a1.y, a2.y, &amny, &amxy);
        box.intersectError = false;
        for(int j = 0; j < m_bx.size() && !box.intersectError; ++j) {
            if (i == j)
                continue;
            auto& checkBox = *m_bx[j];
            if (checkBox.intersectError)
                continue;
            const Vec2& b1 = checkBox.v[0]->p;
            const Vec2& b2 = checkBox.v[2]->p;
            float bmnx, bmxx, bmny, bmxy;
            iminmax(b1.x, b2.x, &bmnx, &bmxx);
            iminmax(b1.y, b2.y, &bmny, &bmxy);

            box.intersectError = (!(bmxx <= amnx || bmnx >= amxx || bmxy <= amny || bmny >= amxy));
        }
        if (box.intersectError)
            continue;

        int sign = (d.x * d.y < 0) ? -1 : 1;  // means its ordered in the reverse order, need to reverse it

        Vertex* av[4];
        for(int i = 0; i < 4; ++i) {
            av[i] = checkUniq(box.v[i]);
            vtx.push_back(av[i]);
        }
        for(int i = 0; i < 4; ++i) 
            bh.push_back(BHalfEdge{av[i], av[(4 + i + sign)%4]});
    }

    // go over half edges, find if there is a vertex that divides a subedge, if there is, divide it
    for(int curhi = 0; curhi < bh.size(); )
    {
        auto curh = bh[curhi]; // need to copy the BHalfEdge since the vector might be reallocating
        Vec2 fp = curh.from->p, tp = curh.to->p;
        float mnx, mxx, mny, mxy;
        iminmax(fp.x, tp.x, &mnx, &mxx);
        iminmax(fp.y, tp.y, &mny, &mxy);
        bool removeCur = false;
        for(int vi = 0; vi < vtx.size(); ++vi)
        {
            Vec2 p = vtx[vi]->p;
            bool divx = (p.y == fp.y && p.y == tp.y && p.x > mnx && p.x < mxx);
            bool divy = (p.x == fp.x && p.x == tp.x && p.y > mny && p.y < mxy);
            if (divx || divy) 
            {
                // add two half edges instead of the one we're removing
                bh.push_back(BHalfEdge{curh.from, vtx[vi]});
                bh.push_back(BHalfEdge{vtx[vi], curh.to});
                removeCur = true;
                break;
            }
        }

        if (removeCur)
            bh.erase(bh.begin() + curhi);
        else
            ++curhi;
    }

    // now find all the unpaired half edges
    set<VPair> unpaired; // pair from,to
    for(auto& h: bh) {
        VPair rv(h.to, h.from); // find h's opposite
        auto it = unpaired.find(rv);
        if (it != unpaired.end()) {
            unpaired.erase(it);
            continue;
        }
        VPair ks(h.from, h.to);
        if (unpaired.find(ks) != unpaired.end())
            throw Exception("unpexpected unpaired");
        unpaired.insert(ks);
    }

    // now order the unpaired edges to a polyline
    map<Vertex*, pair<Vertex*, Vertex*>> vindex; // fromVtx->toVtx, second is nullptr unless its a junction point
    for(auto& up: unpaired) {
        auto it = vindex.find(up.first);
        if (it != vindex.end()) {
            CHECK(it->second.second == nullptr, "unexpected junction with more than two items");
            it->second.second = up.second;  
            continue;
        }
        vindex[up.first] = make_pair(up.second, nullptr);
    }
    // extract polylines
    stringstream szds;
    int pcount = 0;
    while (!vindex.empty())
    {
        auto p = add();
        p->m_fromBox = true;
        auto itStart = vindex.begin();
        // need to start from somewhere that is not a junction since junctions need to be visited between vertices
        while(itStart->second.second != nullptr) {
            ++itStart;
            CHECK(itStart != vindex.end(), "did not find non junction"); // somewhere we must find one
        }
        Vertex* start = itStart->first;
        
        Vertex* cur = start;
        Vertex* prev = nullptr;
        do {
            
            auto it = vindex.find(cur);
            CHECK(it != vindex.end(), "Unexpected end of polyline");
            auto& to = it->second;

            popIfLinear(cur);
            addToLast(cur);

            if (to.second == nullptr) { // normal case
                prev = cur;
                cur = to.first;
                vindex.erase(it);
            }
            else { // junction case
                Vertex* selectedTo = to.second;
                Vertex* otherTo = to.first;

                CHECK(prev != nullptr, "first iteration junction?"); // should not happen since we jumped to start from a non junction
 
                // need to find the correct next one according to the direction we're going to
                auto a = cur->p - prev->p;
                auto b = selectedTo->p - cur->p;
                if (det(a, b) < 0) {
                    swap(selectedTo, otherTo);
                }

                // need new vertex, redirect the old vertex with the new
                Vec2 dirTo = normalize(otherTo->p - cur->p);
                Vertex* newv = addBoxVtx(cur->p + dirTo * 0.1);

                vindex.erase(it);
                vindex[newv] = make_pair(otherTo, nullptr);
                // rewrite the remaining reference to the old to point to the new
                for(auto& kv: vindex) {
                    if (kv.second.first == cur) {
                        kv.second.first = newv;
                    }
                    if (kv.second.second == cur) {
                        kv.second.second = newv; // probably can't happen
                    }
                }

                prev = cur;
                cur = selectedTo;
            }
            
        } while(cur != start);
        popIfLinear(start); // if the last two segments are linear (first point is the past point)
        delFirstIfLinear(); // if the first segment is linear with the last segment
        ++pcount;
        szds << m_pl.back()->m_d.size() << ", ";
    }

    string xx = szds.str();
    //OUT("unified " << pcount << " polylines sz=" << xx);

}
