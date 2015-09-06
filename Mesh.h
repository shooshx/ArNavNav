#pragma once

#include <vector>
#include <functional>
#include "Vec2.h"

using namespace std;


class HalfEdge;
class Triangle;
class Vertex;

class Vertex
{
public:
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

    void clearData() {
        midPnt = Vec2();
        cameFrom = nullptr;
        costSoFar = FLT_MAX;
    }

    Vec2 midPnt;
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
    int highlight = false;
};


struct Polyline
{
    void clear() {
        for(auto v: m_d)
            delete v;
    }
    Vertex* add(const Vec2& v) {
        auto* av = new Vertex(m_d.size(), v);
        m_d.push_back(av);
        return av;
    }
    // not always owns these vertices (yes in MapDef, no in Mesh)
    vector<Vertex*> m_d;
};

class MapDef
{
public:
    ~MapDef() {
        clear();
    }
    Polyline* add() {
        m_p.push_back(new Polyline());
        return m_p.back();
    }
    Polyline* top() {
        if (m_p.empty())
            m_p.push_back(new Polyline());
        return m_p.back();
    }
    void clear() {
        for(auto p: m_p)
            p->clear();
        m_p.clear();
    }

    vector<Polyline*> m_p;
};


class Mesh
{
public:
    ~Mesh() {
        clear();
    }
    void addTri(const Vec2& a, const Vec2& b, const Vec2& c) {
        auto va = new Vertex(m_vtx.size(), a);
        m_vtx.push_back(va);
        auto vb = new Vertex(m_vtx.size(), b);
        m_vtx.push_back(vb);
        auto vc = new Vertex(m_vtx.size(), c);
        m_vtx.push_back(vc);
        m_tri.push_back(new Triangle(va, vb, vc));    
    }
    // takes control of them
    void addTri(Vertex* va, Vertex* vb, Vertex* vc) {
        m_tri.push_back(new Triangle(va, vb, vc));    
    }
    void clear() {
        for(auto v: m_vtx)
            delete v;
        for(auto t: m_tri)
            delete t;
        m_vtx.clear();
        m_tri.clear();
        m_perimiters.clear();
        m_he.clear();
    }

    void connectTri();
    Triangle* findContaining(const Vec2& p);
    bool edgesAstarSearch(const Vec2& startPos, const Vec2& endPos, Triangle* start, Triangle* end, vector<Triangle*>& corridor);

    HalfEdge* addHe() {
        m_he.push_back(HalfEdge());
        return &m_he.back();
    }

    // owns these vertices
    vector<Vertex*> m_vtx;
    vector<Triangle*> m_tri;
    vector<Polyline> m_perimiters;
    vector<HalfEdge> m_he;
};


class PathMaker
{
public:
    typedef std::function<void(Vertex*)> TCallback;
    PathMaker(TCallback& callback) :m_outputCall(callback) 
    {}

    void stringPull(vector<Vertex*> portalsRight, vector<Vertex*> portalsLeft);
    void makePath(vector<Triangle*>& tripath, const Vec2& start, const Vec2& end);

    TCallback& m_outputCall;
};