

#include "../Vec2.h"
#include "../Document.h"
#include "../Except.h"
#include "poly2tri.h"




void runTri(MapDef* mapdef, Mesh& out)
{
    if (mapdef->m_p.size() == 0)
        return;

    int vcount = 0;
    for(auto* mp: mapdef->m_p) 
        vcount += mp->m_d.size();

    vector<p2t::Point> rep;
    rep.reserve(vcount);
    out.m_vtx.reserve(vcount + 2); // the algorithm can add two additional max,min points

    vector<p2t::Point*> emptyPolyline;
    p2t::CDT cdt(emptyPolyline);

    int holeCount = 0;
    for(auto* mp: mapdef->m_p) 
    {
        vector<p2t::Point*> polyline;
        for(auto* pv: mp->m_d) {
            rep.push_back( p2t::Point(pv->p.x, pv->p.y) ); // will not reallocate due to reserve
            out.m_vtx.push_back(new Vertex(out.m_vtx.size(), pv->p) );
            polyline.push_back(&rep.back());
        }
        if (polyline.size() < 3)
            continue;
        cdt.AddHole(polyline);
        ++holeCount;
    }
    if (holeCount == 0)
        return;

    cdt.Triangulate();

    vector<p2t::Triangle*> triangles = cdt.GetTriangles();

    map<p2t::Point*, int> added; // min and max points can be added in case of self intersection
    for(auto* t: triangles) {
        Vertex* nt[3];
        for(int i = 0; i < 3; ++i) 
        {
            auto* p = t->GetPoint(i);
            int vindex = p - &rep[0]; // its index in rep
            if (vindex < 0 || vindex >= out.m_vtx.size()) 
            {
                auto ait = added.find(p);
                if (ait != added.end())
                    vindex = ait->second;
                else {
                    CHECK(added.size() < 2, "unexpected added vertices");
                    vindex = out.m_vtx.size();
                    added[p] = vindex;
                    out.m_vtx.push_back(new Vertex(vindex, Vec2(p->x, p->y)) );
                }
            }
            nt[i] = out.m_vtx[vindex];
        }
        out.addTri(nt[0], nt[1], nt[2]);
    }

}