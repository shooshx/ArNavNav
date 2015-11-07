#pragma once

#include <vector>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include "Vec2.h"

using namespace std;


class HalfEdge;
class Triangle;
class Vertex;

class Vertex
{
public:
    Vertex() :index(-1) {}
    Vertex(int idx, float x, float y) : index(idx), p(x, y) {}
    Vertex(int idx, const Vec2& v) : index(idx), p(v) {}
    int index;
    Vec2 p;
};


class HalfEdge
{
public:
    Vertex* to = nullptr;
    Vertex* from = nullptr;
    HalfEdge *opposite = nullptr;
    HalfEdge *next = nullptr;
    Triangle *tri = nullptr; 
    int index = 0;
    float lengthSq = 0;
    float passToNextSq = FLT_MAX; // distance squared between the 'to' point to the segment of the other two points in the tri, or FLT_MAX if projection is outside the segment
                                  // used for detecting if an agent can pass through this trignagle to the HalfEdge in 'next'

    void clearData() {
        //midPnt = Vec2();
        // TBD- save midPnt before A-star change
        cameFrom = nullptr;
        costSoFar = FLT_MAX;
        curMidPntPtr = &_midPnt;
    }

    // mutable data in Astar 
    Vec2* curMidPntPtr = nullptr; // points to midPnt or to an override in astar
    Vec2 _midPnt; // not referenced directly
    HalfEdge* cameFrom = nullptr; 
    float costSoFar = FLT_MAX;
};


class Triangle
{
public:
    Triangle(Vertex* v0, Vertex* v1, Vertex* v2) {
        v[0] = v0; v[1] = v1; v[2] = v2;
        nei[0] = nei[1] = nei[2] = nullptr;
    }

    Vertex* v[3];
    HalfEdge* h[3];
    Triangle* nei[3]; // nei[0] is across of h[0]
    //int highlight = false;
};


struct Polyline
{
    vector<Vertex*> m_d; // never owns
    vector<int> m_di;
    bool m_fromBox = false; // should be removed when redoing the boxes
};

struct AABox
{
    Vertex *v[4];
};

class MapDef
{
public:
    ~MapDef() {
        clear();
    }
    Polyline* add(const string& module = string()) {
        auto p = new Polyline();
        m_pl.push_back(unique_ptr<Polyline>(p));
        if (!module.empty())
            m_objModules[p] = module;
        return p;
    }
    Vertex* addToLast(const Vec2& p, const string& module = string()) {
        if (m_pl.empty())
            add(module);
        auto v = addVtx(p);
        return addToLast(v, module);
    }
    Vertex* addToLast(Vertex* v, const string& module = string()) {
        Polyline* pl = m_pl.back().get();
        pl->m_d.push_back(v);
        pl->m_di.push_back(m_vtx.size() - 1);
        if (!module.empty())
            m_objModules[v] = module;
        return v;
    }

    void popIfLinear(Vertex* nextv);
    void delFirstIfLinear();

    bool isLastEmpty() {
        return m_pl.empty() || m_pl.back()->m_d.empty();
    }
    void clear() {
        m_vtx.clear();
        m_pl.clear();
        m_objModules.clear();
        m_bx.clear();
        m_boxAddedVtx.clear();
    }

    Vertex* addVtx(const Vec2& p) {
        auto v = new Vertex(m_vtx.size(), p);
        m_vtx.push_back(unique_ptr<Vertex>(v));
        return v;
    }
    Vertex* addBoxVtx(const Vec2& p) {
        auto v = new Vertex(m_vtx.size() + m_boxAddedVtx.size(), p);
        m_boxAddedVtx.push_back(unique_ptr<Vertex>(v));
        return v;
    }

    AABox& addBox(const Vec2& pa, const Vec2& pb) {
        AABox b;
        b.v[0] = addVtx(pa);
        b.v[1] = addVtx(Vec2(pa.x, pb.y));
        b.v[2] = addVtx(pb);
        b.v[3] = addVtx(Vec2(pb.x, pa.y));
        m_bx.push_back(b);
        return m_bx.back();
    }

    void makeBoxPoly();

    vector<unique_ptr<Vertex>> m_vtx; // owns the objects. don't know how much are going to be so can't preallocate
                           // needs to be pointers since display items reference them
    vector<unique_ptr<Polyline>> m_pl;
    vector<AABox> m_bx;
    vector<unique_ptr<Vertex>> m_boxAddedVtx; // vertices added when parsing the boxes, should be discarded when boxes are reparsed
    
    map<void*, string> m_objModules; // defined objects can have optional string modules where they came from
};


class Mesh
{
public:
    ~Mesh() {
        clear();
    }

    // takes control of them
    void addTri(Vertex* va, Vertex* vb, Vertex* vc) {
        m_tri.push_back(Triangle(va, vb, vc));    
    }
    void clear() {
        m_vtx.clear();
        m_tri.clear();
        m_perimiters.clear();
        m_he.clear();
    }

    void connectTri();
    Triangle* findContaining(const Vec2& p, vector<Vec2>& posRef);
    bool edgesAstarSearch(const Vec2& startPos, const Vec2& endPos, Triangle* start, Triangle* end, vector<Triangle*>& corridor, float agetnRadiusSq);

    HalfEdge* addHe() {
        m_he.push_back(HalfEdge());
        m_he.back().index = m_he.size() - 1;
        return &m_he.back();
    }

    // owns these vertices
    vector<Vertex> m_vtx;
    vector<Triangle> m_tri;
    vector<Polyline> m_perimiters;
    vector<HalfEdge> m_he;

    // for every radius, have a set of alternative position per vertex for plan creation
    // used at the beginning of the planning to determine the correct triangle the agent and the goal is at
    map<float, vector<Vec2>> m_altVtxPosByRadius;
};

// for the stringPull algorithm we need both the vertex pointer to know 
// what point is related to what vertex and the position that is related to this vertex 
// that will be used for calculating the path
struct VtxWrap
{
    VtxWrap(Vertex* _v, const Vec2& _p) :v(_v), p(_p) {}
    Vertex* v = nullptr;
    Vec2 p;
};

class PathMaker
{
public:
    typedef std::function<void(Vertex*)> TOutputCallback;
    typedef std::function<Vec2(Vertex*)> TGetPosCallback;
    PathMaker(const TOutputCallback& outCallback, const TGetPosCallback& posCallback) :m_outputCall(outCallback), m_getPos(posCallback)
    {}

    void stringPull(const vector<VtxWrap>& portalsRight, const vector<VtxWrap>& portalsLeft);
    void makePath(const vector<Triangle*>& tripath, const Vec2& start, const Vec2& end);

    const TOutputCallback& m_outputCall;
    const TGetPosCallback& m_getPos;

    Vertex m_startDummy, m_endDummy;
};