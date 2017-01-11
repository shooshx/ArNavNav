#include "../Mesh.h"
#include "../Except.h"
#include  <algorithm>

// http://geomalgorithms.com/a03-_inclusion.html
// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
//    See: Algorithm 1 "Area of Triangles and Polygons"
inline int isLeft(const Vec2& P0, const Vec2& P1, const Vec2& P2)
{
    return ((P1.x - P0.x) * (P2.y - P0.y)
        - (P2.x - P0.x) * (P1.y - P0.y));
}

// wn_PnPoly(): winding number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  wn = the winding number (=0 only when P is outside)
bool wn_PnPoly_inside(const Vec2& P, const vector<Vertex*>& V)
{
    int wn = 0;    // the  winding number counter
    int n = V.size();
                      // loop through all edges of the polygon
    for (int i = 0; i<n; i++) {   // edge from V[i] to  V[i+1]
        if (V[i]->p.y <= P.y) {          // start y <= P.y
            if (V[(i + 1)%n]->p.y  > P.y)      // an upward crossing
                if (isLeft(V[i]->p, V[(i + 1)%n]->p, P) > 0)  // P left of  edge
                    ++wn;            // have  a valid up intersect
        }
        else {                        // start y > P.y (no test needed)
            if (V[(i + 1)%n]->p.y <= P.y)     // a downward crossing
                if (isLeft(V[i]->p, V[(i + 1)%n]->p, P) < 0)  // P right of  edge
                    --wn;            // have  a valid down intersect
        }
    }
    return wn != 0;
}


void orderPerimiters(vector<Polyline>& p, vector<Polyline*>& o)
{
    vector<pair<int, Polyline*>> depth(p.size());
    for(int i = 0; i < p.size(); ++i)
        depth[i].second = &p[i];
    for(int i = 0; i < p.size(); ++i) 
    {
        for(int j = 0; j < p.size(); ++j) 
        {
            if (j == i)
                continue;
            if (wn_PnPoly_inside(p[i].m_d[0]->p, p[j].m_d))
                depth[i].first += 1;
        }
    }
    std::sort(depth.begin(), depth.end());
    o.reserve(p.size());
    for(auto& pl: depth)
        o.push_back(pl.second);
}

#if 0

vector<int> insideOnly(p.size());
for (int i = 0; i < p.size(); ++i)
{
    int removed = 0;
    for (int ii = 0; ii < isInside[i].size(); ++ii)
    {
        int ref = isInside[i][ii];
        if (!isInside[ref].empty()) {
            isInside[i][ii] = -1;
            ++removed;
        }
    }
    if (isInside[i].size() > 0)
        CHECK(isInside[i].size() - removed == 1, "polyline inside more than one other polyline?");
    insideOnly[i] = -1;
    for (int ii = 0; ii < isInside[i].size(); ++ii) {
        if (isInside[i][ii] != -1) {
            insideOnly[i] = isInside[i][ii];
            break;
        }
    }
}
vector<pair<int, Polyline*>> depth;
depth.reserve(p.size());
for (int i = 0; i < p.size(); ++i) {
    int at = insideOnly[i];
    int d = 0;
    while (at != -1) {
        at = insideOnly[at];
        ++d;
    }
    depth.push_back(make_pair(d, &p[i]));
}
#endif