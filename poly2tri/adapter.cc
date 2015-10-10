

#include "../Vec2.h"
#include "../Document.h"
#include "../Except.h"
#include "poly2tri.h"




void runTri(MapDef* mapdef, Mesh& out)
{
    if (mapdef->m_pl.size() == 0)
        return;

    int vcount = 0;
    for(const auto& mp: mapdef->m_pl) 
        vcount += mp.m_d.size();

    vector<p2t::Point> rep;
    rep.reserve(vcount);
    out.m_vtx.reserve(vcount + 2); // the algorithm can add two additional max,min points

    p2t::CDT cdt;
    cdt.sweep_context_.points_.reserve(vcount + 2);

    vector<p2t::Point*> polyline;

    // add the polylines one by one
    int holeCount = 0;
    for(const auto& mp: mapdef->m_pl) 
    {
        polyline.clear();
        for(int i = 0; i < mp.m_d.size(); ++i) 
        {
            Vertex* pv = mp.m_d[i];
            if (i > 0 && pv->p == mp.m_d[i - 1]->p) // repeat vertex - ignore it
                continue;
            rep.push_back( p2t::Point(pv->p.x, pv->p.y, out.m_vtx.size()) ); // will not reallocate due to reserve
            out.m_vtx.push_back(Vertex(out.m_vtx.size(), pv->p) );
            polyline.push_back(&rep.back());
        }
        if (polyline.size() < 3)
            continue;
        cdt.sweep_context_.AddHole(polyline);
        ++holeCount;
    }

    if (holeCount == 0)
        return;

    int iter = 0;
    while(true)
    {
        cdt.Triangulate();

        vector<p2t::Triangle*> triangles = cdt.GetTriangles();
        out.m_tri.reserve(out.m_tri.size() + triangles.size());

        map<p2t::Point*, int> added; // min and max points can be added in case of self intersection
        for(auto* t: triangles) 
        {
            Vertex* nt[3];
            for(int i = 0; i < 3; ++i) 
            {
                auto* p = t->GetPoint(i);
                int vindex = p->vindex; // its index in my vertices
                p->visited = true; // mark it as used
                if (vindex < 0) // it's not a vertex from the input
                {
                    auto ait = added.find(p);
                    if (ait != added.end())
                        vindex = ait->second;
                    else {
                        CHECK(added.size() < 2, "unexpected added vertices");
                        vindex = out.m_vtx.size();
                        added[p] = vindex;
                        out.m_vtx.push_back(Vertex(vindex, Vec2(p->x, p->y)) );
                    }
                }
                nt[i] = &out.m_vtx[vindex];
            }
            out.addTri(nt[0], nt[1], nt[2]);
        }

        vector<p2t::Point*> leftOver;
        for(auto& p: rep) {
            if (!p.visited)
                leftOver.push_back(&p);
        }
        if (leftOver.size() < 3)
            break;

        cdt.sweep_context_.points_ = std::move(leftOver);
        cdt.sweep_context_.clearMap();
        ++iter;
    }

}