#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <iterator> // inserter
#include <queue>
#include "Vec2.h"
#include "Except.h"

using namespace std;



void zerofy(float& v) {
    if (iabs(v) < 0.001)
        v = 0;
}

typedef tuple<int, int, int> Key3;
Key3 makeKey3(float x, float y, float z) {
    return make_tuple((int)std::round(x*100.0), (int)std::round(y*100.0), (int)std::round(z*100.0));
}

pair<int, int> sedge(int a, int b) {
    if (a < b)
        return make_pair(a, b);
    return make_pair(b, a);
}


struct TTri {
    TTri() :a(0), b(0), c(0) {}
    TTri(int _a, int _b, int _c) : a(_a), b(_b), c(_c) {}

    int a,b,c;
    bool rm = false;
};
struct FTri { // for flood fill
    int a, b, c;
    bool visited = false;
    pair<int, int> s1, s2, s3;
};

class TMesh
{
public:
    bool readAndUnify(const string& filename);
    void deTeeVtx();
    template<typename T>
    void save(const string& outname, vector<T>* faces = &m_tri);

    void addFTri(FTri& ft);
    void makeFlatPlaneSedgeIndex();
    bool floodFill(vector<int>& triidx);
    template<typename PF>
    bool fillAndExtract(const PF& pf);
    void findUnpaired(vector<int>& subidx, vector<pair<int, int>>& unpaired);
    void extractPoly(vector<pair<int, int>>& unpaired, vector<int>& polyline);
    void polySubSample(vector<int>& polyline, vector<int>& subs);
    void polySave(const string& outname, vector<int>& polyline);
    void getPoly(const string& outname);
    void polyMinMax(vector<int>& polyline, Vec2& mn, Vec2& mx);
    bool removeSelfIntersections(vector<int>& polyline);

    vector<Vec3> m_vtx;
    vector<TTri> m_tri;
    vector<FTri> m_facePlane; // triangles that are facing up

    map<pair<int, int>, set<int>> m_sedgeToFace; // sorted pair of an edge to the faces it is part of

};

bool TMesh::readAndUnify(const string& filename)
{
    ifstream ifs(filename);
    if (!ifs.good()) {
        cout << "failed opening " << filename << endl;
        return false;
    }

    map<Key3, int> univtx; // vec3->new index
    map<int, int> oldToNew;

    m_vtx.push_back(Vec3()); // dummy first since indices start at 1
    int vtxi = 1;
    int ncount = 1;
    while (!ifs.eof())
    {
        string cmd;
        ifs >> cmd;
        if (cmd[0] == 'v') {
            float x,y,z;
            ifs >> x >> y >> z;
            zerofy(x); zerofy(y); zerofy(z);
            auto t = makeKey3(x, y, z);
            auto uit = univtx.find(t);
            if (uit == univtx.end()) {
                univtx[t] = ncount;
                oldToNew[vtxi] = ncount;
                ++ncount;
                m_vtx.push_back(Vec3(x, y, z));
                //ofs << "v " << x << " " << y << " " << z << "\n";
            }
            else {
                oldToNew[vtxi] = uit->second; 
            }
            ++vtxi;

            if ((vtxi % 10000) == 0) 
                cout << "vtx " << vtxi << endl;
        }
        else if (cmd[0] == 'f') {
            int a,b,c;
            ifs >> a >> b >> c;
            auto ait = oldToNew.find(a);
            auto bit = oldToNew.find(b);
            auto cit = oldToNew.find(c);
            CHECK(ait != oldToNew.end() && bit != oldToNew.end() && cit != oldToNew.end(), "Unexpected index");
            m_tri.push_back(TTri(ait->second, bit->second, cit->second));
            //ofs << "f " << ait->second << " " << bit->second << " " << cit->second << "\n";
        }
        else if (!cmd.empty()){
            cout << "UNKNOWN CMD " << cmd << endl;
        }
    };

    cout << "Unify before=" << vtxi << " after=" << ncount << endl;
    return true;
}



bool isMod12(float f) {
    float intp = std::round(f);
    
    return (iabs(intp - f) < 0.01 && (((int)intp) % 12) == 0);
}

void TMesh::deTeeVtx()
{
    map<Key3, int> vtxIndex; // key3->index of vertex in vtx
    for(int i = 0; i < m_vtx.size(); ++i) {
        auto& v = m_vtx[i];
        vtxIndex[makeKey3(v.x, v.y, v.z)] = i;
    }

    auto checkAddVtx = [&](const Vec3& v)->int {
        auto k = makeKey3(v.x, v.y, v.z);
        auto it = vtxIndex.find(k);
        if (it != vtxIndex.end())
            return it->second;
        int ni = m_vtx.size();
        vtxIndex[k] = ni;
        m_vtx.push_back(v);
        return ni;
    };

    int rmed = 0;
    vector<TTri> newtri;
    for(auto& t: m_tri) {
        Vec3 a = m_vtx[t.a];
        Vec3 b = m_vtx[t.b];
        Vec3 c = m_vtx[t.c];
        if (isMod12(a.x) && isMod12(a.z) && isMod12(b.x) && isMod12(b.z) && isMod12(c.x) && isMod12(c.z)) {
            t.rm = true;
            ++rmed;
            // lookup vertices in vtx, connect them
            int ab = checkAddVtx((a + b) * 0.5);
            int bc = checkAddVtx((b + c) * 0.5);
            int ac = checkAddVtx((c + a) * 0.5);
            newtri.push_back(TTri(t.a, ab, ac));
            newtri.push_back(TTri(t.b, bc, ab));
            newtri.push_back(TTri(t.c, ac, bc));
            newtri.push_back(TTri(ab, bc, ac));
        }
    }

    vector<TTri> tri;
    tri.reserve(m_tri.size() - rmed + newtri.size());
    for(auto& f: m_tri)
        if (!f.rm)
            tri.push_back(f);
    tri.insert(tri.end(), newtri.begin(), newtri.end());
    m_tri = std::move(tri);

    cout << "deTee after=" << m_vtx.size() << endl;
}

template<typename T>
void TMesh::save(const string& outname, vector<T>* faces)
{
    cout << "saving " << outname << endl;
    ofstream ofs(outname);
    if (!ofs.good()) {
        cout << "failed opening " << outname << endl;
        return;
    }
    for(int i = 1; i < m_vtx.size(); ++i) {
        auto& v = m_vtx[i];
        ofs << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }

    for(auto& f: *faces)
        ofs << "f " << f.a << " " << f.b << " " << f.c << "\n";
}


void TMesh::addFTri(FTri& t)
{
    t.s1 = sedge(t.a, t.b);
    t.s2 = sedge(t.b, t.c);
    t.s3 = sedge(t.c, t.a);
    int fi = m_facePlane.size();
    m_facePlane.push_back(t);
    m_sedgeToFace[t.s1].insert(fi);
    m_sedgeToFace[t.s2].insert(fi);
    m_sedgeToFace[t.s3].insert(fi);
}

void TMesh::makeFlatPlaneSedgeIndex() 
{
    for(auto& f: m_tri) 
    {
        FTri t;
        t.a = f.a; t.b = f.b; t.c = f.c;
        auto pa = m_vtx[t.a];
        auto pb = m_vtx[t.b];
        auto pc = m_vtx[t.c];
        auto normal = crossProd(pb-pa, pc-pa);
        normal.normalize();

        if (iabs(normal.y) > 0.99 && iabs(m_vtx[t.a].y) < 0.02) 
        {
            addFTri(t);
            //ofs << "f " << t.a << " " << t.b << " " << t.c << "\n";
        }
    }
}


bool TMesh::floodFill(vector<int>& triidx)
{
    // search for unvisited
    int ti = 0;
    while (ti < m_facePlane.size() && m_facePlane[ti].visited)
        ++ti;
    if (ti == m_facePlane.size())
        return false;

    queue<int> qi;
    qi.push(ti);

    while(!qi.empty()) 
    {
        ti = qi.front();
        qi.pop();
        auto& t = m_facePlane[ti];
        if (t.visited)
            continue;
        t.visited = true;
        triidx.push_back(ti);
        for(int ni: m_sedgeToFace[t.s1]) {
            if (ni != ti)
                qi.push(ni);
        }
        for(int ni: m_sedgeToFace[t.s2]) {
            if (ni != ti)
                qi.push(ni);
        }
        for(int ni: m_sedgeToFace[t.s3]) {
            if (ni != ti)
                qi.push(ni);
        }
    }
    return true;
}

void TMesh::findUnpaired(vector<int>& subidx, vector<pair<int, int>>& unpaired)
{
    for(int fi: subidx) {
        auto& t = m_facePlane[fi];
        if (m_sedgeToFace[t.s1].size() == 1)
            unpaired.push_back(t.s1);
        if (m_sedgeToFace[t.s2].size() == 1)
            unpaired.push_back(t.s2);
        if (m_sedgeToFace[t.s3].size() == 1)
            unpaired.push_back(t.s3);
    }
}

void TMesh::extractPoly(vector<pair<int, int>>& unpaired, vector<int>& polyline)
{
    int len = 0;
    int atun = 0;
    int curv = unpaired[0].first;
    int stv = curv;
    int env = unpaired[0].second;
    unpaired.erase(unpaired.begin() + atun);
    while(curv != env) 
    {
        polyline.push_back(curv);
        //cout << "    " << curv << endl;
        int nextv = -1;
        for(int i = 0; i < unpaired.size(); ++i) 
        {
            auto& up = unpaired[i];
            if (up.first == curv)
                nextv = up.second;
            else if (up.second == curv)
                nextv = up.first;
            else
                continue;
            atun = i;
            break;
        }
        if (nextv == -1) {
            cout << "    broken" << endl;
            break;
        }
        curv = nextv;
        unpaired.erase(unpaired.begin() + atun);
        ++len;
    }
    polyline.push_back(curv);

    cout << "  e=" << env << " s=" << stv << " len=" << len << endl;
}

void TMesh::polySubSample(vector<int>& polyline, vector<int>& subs)
{
    set<int> added; // avoid duplicates that are caused by holes in the mesh
    int pi = polyline[0];
    auto last = m_vtx[pi];
    subs.push_back(pi);
    added.insert(pi);
    for(int pi: polyline) {
        auto p = m_vtx[pi];
        if (dist(p, last) > 6.0 && (added.count(pi) == 0)) {
            subs.push_back(pi);
            added.insert(pi);
            last = p;
        }
    }
}


Vec2 toHtmlCoord(const Vec2& v) {
    return v;
    return Vec2(v.x * 5.0 + 1800.0, v.y * 5.0 + 1500.0);
}

void TMesh::polySave(const string& outname, vector<int>& polyline)
{
    ofstream ofsp(outname);
    if (!ofsp.good()) {
        cout << "failed opening " << outname << endl;
        return;
    }
    for(int pi: polyline) {
        auto p = m_vtx[pi];
        auto tv = toHtmlCoord(Vec2(p.x, p.z));
        ofsp << "v " << tv.x  << " " << tv.y << " 0.0\n";
    }
}

template<typename PF>
bool TMesh::fillAndExtract(const PF& pf)
{
    vector<int> cur;
    if (!floodFill(cur))
        return false;

    vector<pair<int, int>> unpaired;
    // find all the unpaired sedges
    findUnpaired(cur, unpaired);

    cout << "fill " << cur.size() << " unpaired=" << unpaired.size() << endl;
    while (!unpaired.empty())
    {
        vector<int> polyline;

        extractPoly(unpaired, polyline);
        pf(polyline);
    }
    return true;
}

// inner and outer polyline
class TwoPoly
{
public:
    vector<vector<int>> v;
    Vec2 mn = Vec2(FLT_MAX, FLT_MAX);
    Vec2 mx = Vec2(-FLT_MAX, -FLT_MAX);
};

void TMesh::polyMinMax(vector<int>& polyline, Vec2& mn, Vec2& mx)
{
    for(auto& vi: polyline) {
        auto v = m_vtx[vi];
        if (v.x < mn.x)
            mn.x = v.x;
        if (v.z < mn.y)
            mn.y = v.z;
        if (v.x > mn.x)
            mx.x = v.x;
        if (v.z > mn.y)
            mx.y = v.z;
    }
}

//http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
bool TMesh::removeSelfIntersections(vector<int>& polyline)
{
    int countRemove = 0;
    while (true)
    {
        int removei = -1;
        int sz = polyline.size();
        for(int chai = 0; chai < sz-1 && removei == -1; ++chai) // check segment
        {
            int chbi = (chai + 1) % sz;
            Vec2 cha = m_vtx[polyline[chai]].toVec2();
            Vec2 chb = m_vtx[polyline[chbi]].toVec2();
            Vec2 q = cha;
            Vec2 s = chb-cha;
            for(int trai = chai + 1; trai < sz; ++trai) // target to check with
            {
                int trbi = (trai + 1) % sz;
                if (trai == chai || trai == chbi || trbi == chai) // don't intersect a segment with itself or its neighbors
                    continue;
                Vec2 tra = m_vtx[polyline[trai]].toVec2();
                Vec2 trb = m_vtx[polyline[trbi]].toVec2();
                Vec2 p = tra;
                Vec2 r = trb-tra;

                float drs = det(r, s);
                if (drs == 0.0f)
                    continue; // parallal or colinear
                Vec2 qp = q-p;
                float t = det(qp, s) / drs;
                float u = det(qp, r) / drs;
                if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) { // intersects
                    cout << "    intersect " << chai << "-" << chbi << ":" << trai << "-" << trbi << "  t=" << t << " u=" << u << endl;
                    removei = chbi;
                    break;
                }
            }
        }

        if (removei == -1)
            break;
        cout << "    remove " << removei << endl;
        polyline.erase(polyline.begin() + removei);
        ++countRemove;
        //break;
    }

    cout << "  Removed intersections " << countRemove << endl;

    return true;
}


void TMesh::getPoly(const string& outname)
{
    vector<TwoPoly> polys; 
    int mxdi = -1; // find the mesh that produces the largest area, that's probably the frame mesh that we don't want
    float mxd = 0;
    while (true) 
    {
        TwoPoly tp;
        bool didFill = fillAndExtract([&](vector<int>& polyline){
            if (polyline.size() > 10) {
                polyMinMax(polyline, tp.mn, tp.mx);
                tp.v.push_back(polyline);
            }
        });
        if (!didFill)
            break;
        if (tp.v.empty())
            continue;
        float d = distSq(tp.mn, tp.mx);
        if (d > mxd) {
            mxdi = polys.size();
            mxd = d;
        }
        polys.push_back(tp);
    }

    CHECK(mxdi >= 0, "unexpected mxdi");
    cout << "Skipping TwoPoly " << mxdi << " since its the frame " << mxd << "\n";
    polys.erase(polys.begin() + mxdi);

    ofstream ofsp(outname);
    if (!ofsp.good()) {
        cout << "failed opening " << outname << endl;
        return;
    }

    int tpi = 0;
    int pi = 0;
    for(auto& tp: polys)
    {
        int vpi = 0;
        for(auto& polyline: tp.v) 
        {
            cout << "poly " << pi++ << ": sz=" << polyline.size() << endl;
            vector<int> dwp;
            polySubSample(polyline, dwp);
            cout << "  reduced=" << dwp.size() << endl;
            removeSelfIntersections(dwp);
            cout << "  no-intersec=" << dwp.size() << endl;
            if (dwp.size() < 4)
                continue;
    //        polySave(outname + "_" + to_string(polyi) + ".obj", dwp);

            ofsp << "p:" << tpi << ":" << vpi << "\n";  
            for(int pi: dwp) {
                auto p = m_vtx[pi];
                auto tv = toHtmlCoord(Vec2(p.x, p.z));
                ofsp << "v " << tv.x  << " " << tv.y << "\n";
            }
            ++vpi;
        }
        ++tpi;
    }
}

void runExtract(const string& filename, const string& outname)
{
    cout << "******* " << filename << endl;
    TMesh m;
    if (!m.readAndUnify(filename))
        return;
    m.deTeeVtx();

    m.makeFlatPlaneSedgeIndex();
    //m.save(outname + "_flat.obj", &m.m_facePlane);

    m.getPoly(outname);
}

void terrainExtract()
{

    //string filename = "C:\\projects\\nav\\terrain\\MyCity_all_terrain.obj";
    //string filename = "C:\\projects\\nav\\terrain\\Mission_3.GOWScene.zip_all_terrain.obj";
    //string outname = "C:\\projects\\nav\\terrain\\MyCity_navmesh.obj";
    //string outname = "C:\\projects\\nav\\terrain\\Mission_3.txt";

    //runExtract("C:\\projects\\nav\\terrain\\Mission_20.GOWScene.zip_all_terrain.obj", "C:\\projects\\nav\\terrain\\Mission_20.txt");

    runExtract("C:\\projects\\nav\\terrain\\MyCity.GOWScene.zip_all_terrain.obj", "C:\\projects\\nav\\terrain\\MyCity.txt");


    for(int i = 20; i <= 20; ++i) {
        stringstream sf, so;
        sf << "C:\\projects\\nav\\terrain\\Mission_" << i << ".GOWScene.zip_all_terrain.obj";
        so << "C:\\projects\\nav\\terrain\\Mission_" << i << ".txt";
        runExtract(sf.str(), so.str());
    }

}


bool checkSelfIntersect(vector<Vec3>& vtx, vector<int>& pl)
{
    TMesh m;
    m.m_vtx = vtx;
    return m.removeSelfIntersections(pl);
}





