#ifndef DOCUMENT_H 
#define DOCUMENT_H

#include <QObject>
#include <QPointF>
#include <QColor>

#include <vector>



#include "Objects.h"
#include "Mesh.h"

using namespace std;



struct Polyline
{
    ~Polyline() {
        for(auto v: m_d)
            delete v;
    }
    Vertex* add(const Vec2& v) {
        auto* av = new Vertex(m_d.size(), v);
        m_d.push_back(av);
        return av;
    }
    // owns these vertices
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
            delete p;
        m_p.clear();
    }

    vector<Polyline*> m_p;
};

class Document : public QObject
{
    Q_OBJECT

public:
    Document(QObject *parent);
    ~Document() {}

    void runTriangulate();

    void init_preset();
    void init_preset_grid();
    void init_test();
    void init_tri();

    vector<Object*> m_objs;
    Object *m_prob = nullptr;

    Mesh m_mesh;
    MapDef m_mapdef;
    Vertex *m_start = nullptr;
    Vertex *m_end = nullptr;
    vector<Vec2> m_path;

};





#endif // DOCUMENT_H
