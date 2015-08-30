#ifndef DOCUMENT_H 
#define DOCUMENT_H

#include <QObject>
#include <QPointF>
#include <QColor>

#include <vector>



#include "Objects.h"
#include "Mesh.h"


using namespace std;


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

    void clearObst();

    void clearSegMinDist();
    bool doStep(float deltaTime, bool doUpdate);

public:
    vector<Object*> m_objs; // owning
    vector<MultiSegment> m_multisegs; // owning

    Mesh m_mesh;
    MapDef m_mapdef;

    Vertex *m_start = nullptr;
    Vertex *m_end = nullptr;
    vector<Vec2> m_path;
    vector<Vertex*> m_markers;
    Object *m_prob = nullptr;

};





#endif // DOCUMENT_H
