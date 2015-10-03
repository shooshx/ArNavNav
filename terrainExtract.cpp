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

class Vec3 {
public:
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    float x, y, z;

    float length() const {
        return sqrt(x*x + y*y + z*z);
    }
    void normalize() {
        float n = 1.0f/length();
        x *= n; y *= n; z *= n;
    }
};

static Vec3 crossProd(const Vec3 &a, const Vec3& b) {
    return Vec3(a.y * b.z - a.z * b.y, 
        a.z * b.x - a.x * b.z, 
        a.x * b.y - a.y * b.x);
}
inline Vec3 operator-(const Vec3& a, const Vec3& b) {
    return Vec3(a.x-b.x, a.y-b.y, a.z-b.z);
}
inline Vec3 operator+(const Vec3& a, const Vec3& b) {
    return Vec3(a.x+b.x, a.y+b.y, a.z+b.z);
}
inline Vec3 operator*(const Vec3& a, float v) {
    return Vec3(a.x*v, a.y*v, a.z*v);
}
inline float dist(const Vec3& a, const Vec3& b) {
    float dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

void zerofy(float& v) {
    if (iabs(v) < 0.001)
        v = 0;
}

typedef tuple<int, int, int> Key3;
Key3 makeKey3(float x, float y, float z) {
    return make_tuple((int)std::round(x*100.0), (int)std::round(y*100.0), (int)std::round(z*100.0));
}

void unifyVtx() 
{
    string filename = "C:\\projects\\nav\\terrain\\MyCity_all_terrain.obj";
    string outname = "C:\\projects\\nav\\terrain\\MyCity_unified.obj";

    ifstream ifs(filename);
    if (!ifs.good()) {
        cout << "failed opening " << filename << endl;
        return;
    }
    ofstream ofs(outname);
    if (!ofs.good()) {
        cout << "failed opening " << outname << endl;
        return;
    }

    map<Key3, int> univtx; // vec3->new index
    map<int, int> oldToNew;

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
                ofs << "v " << x << " " << y << " " << z << "\n";
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
            ofs << "f " << ait->second << " " << bit->second << " " << cit->second << "\n";
        }
        else if (!cmd.empty()){
            cout << "UNKNOWN CMD " << cmd << endl;
        }
    };

    cout << "Unify before=" << vtxi << " after=" << ncount << endl;

}

struct TTri {
    TTri() :a(0), b(0), c(0) {}
    TTri(int _a, int _b, int _c) : a(_a), b(_b), c(_c) {}
    int a,b,c;
    bool rm = false;
};

bool isMod12(float f) {
    float intp = std::round(f);
    
    return (iabs(intp - f) < 0.01 && (((int)intp) % 12) == 0);
}

void deTeeVtx()
{
    string filename = "C:\\projects\\nav\\terrain\\MyCity_unified.obj";
    string outname = "C:\\projects\\nav\\terrain\\MyCity_detee.obj";

    ifstream ifs(filename);
    if (!ifs.good()) {
        cout << "failed opening " << filename << endl;
        return;
    }
    ofstream ofs(outname);
    if (!ofs.good()) {
        cout << "failed opening " << outname << endl;
        return;
    }

    vector<Vec3> vtx;
    vtx.push_back(Vec3(0,0,0));
    vector<TTri> tri;
    map<Key3, int> vtxIndex; // key3->index of vertex in vtx

    int vtxi = 1;
    int ncount = 1;
    while (!ifs.eof())
    {
        string cmd;
        ifs >> cmd;
        if (cmd[0] == 'v') {
            Vec3 v;
            ifs >> v.x >> v.y >> v.z;
            vtxIndex[makeKey3(v.x, v.y, v.z)] = vtx.size();
            vtx.push_back(v);
            ++vtxi;
            if ((vtxi % 10000) == 0) 
                cout << "vtx " << vtxi << endl;
        }
        else if (cmd[0] == 'f') {
            TTri t;
            ifs >> t.a >> t.b >> t.c;
            tri.push_back(t);
        }
        else if (!cmd.empty()){
            cout << "UNKNOWN CMD " << cmd << endl;
        }
    };

    auto checkAddVtx = [&](const Vec3& v)->int {
        auto k = makeKey3(v.x, v.y, v.z);
        auto it = vtxIndex.find(k);
        if (it != vtxIndex.end())
            return it->second;
        int ni = vtx.size();
        vtxIndex[k] = ni;
        vtx.push_back(v);
        return ni;
    };

    vector<TTri> newtri;
    for(auto& t: tri) {
        Vec3 a = vtx[t.a];
        Vec3 b = vtx[t.b];
        Vec3 c = vtx[t.c];
        if (isMod12(a.x) && isMod12(a.z) && isMod12(b.x) && isMod12(b.z) && isMod12(c.x) && isMod12(c.z)) {
            t.rm = true;
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

    for(int i = 1; i < vtx.size(); ++i) {
        auto& v = vtx[i];
        ofs << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    for(auto& f: tri)
        if (!f.rm)
            ofs << "f " << f.a << " " << f.b << " " << f.c << "\n";
    for(auto& f: newtri)
        ofs << "f " << f.a << " " << f.b << " " << f.c << "\n";
}



struct Tri {
    int a, b, c;
    bool visited = false;
    pair<int, int> s1, s2, s3;
};

pair<int, int> sedge(int a, int b) {
    if (a < b)
        return make_pair(a, b);
    return make_pair(b, a);
}

Vec2 toHtmlCoord(const Vec2& v) {
    //return v;
    return Vec2(v.x * 5.0 + 1800.0, v.y * 5.0 + 1500.0);
}


void terrainExtract()
{
    //unifyVtx();
    //deTeeVtx();
    //return;

    string filename = "C:\\projects\\nav\\terrain\\MyCity_detee.obj";
    string outname = "C:\\projects\\nav\\terrain\\MyCity_poly.obj";

    ifstream ifs(filename);
    if (!ifs.good()) {
        cout << "failed opening " << filename << endl;
        return;
    }
    ofstream ofs(outname);
    if (!ofs.good()) {
        cout << "failed opening " << outname << endl;
        return;
    }

    vector<Vec3> pntsAll;
    pntsAll.push_back(Vec3(0,0,0)); // dummy since indices start at 1
    vector<Tri> facePlane;
    map<pair<int, int>, set<int>> sedgeToFace; // sorted pair of an edge to the faces it is part of

    map<int, int> vtxToLine;

    int vtxi = 1;
    int linei = 1;
    int polyi = 0;

    //int facei = 0;
    while (!ifs.eof())
    {
        string cmd;
        ifs >> cmd;
        if (cmd[0] == 'v') {
            float x,y,z;
            ifs >> x >> y >> z;
            pntsAll.push_back(Vec3(x,y,z));

            ofs << "v " << x << " " << y << " " << z << "\n";
            ++vtxi;
            ++linei;
            vtxToLine[vtxi] = linei;
            if ((vtxi % 10000) == 0) 
                cout << "vtx " << vtxi << endl;
        }
        else if (cmd[0] == 'f') {
            Tri t;
            ifs >> t.a >> t.b >> t.c;

            auto pa = pntsAll[t.a];
            auto pb = pntsAll[t.b];
            auto pc = pntsAll[t.c];
            auto normal = crossProd(pb-pa, pc-pa);
            normal.normalize();

            if (iabs(normal.y) > 0.99 && iabs(pntsAll[t.a].y) < 0.02) {
                t.s1 = sedge(t.a, t.b);
                t.s2 = sedge(t.b, t.c);
                t.s3 = sedge(t.c, t.a);
                int fi = facePlane.size();
                facePlane.push_back(t);
                sedgeToFace[t.s1].insert(fi);
                sedgeToFace[t.s2].insert(fi);
                sedgeToFace[t.s3].insert(fi);
                ofs << "f " << t.a << " " << t.b << " " << t.c << "\n";
                ++linei;
            }
            //++facei;
        }
        else if (!cmd.empty()){
            cout << "UNKNOWN CMD " << cmd << endl;
        }
    };

    vector<vector<int>> polys;

    while (true) // as long as there are unvisited faces
    {
        // search for unvisited
        int ti = 0;
        while (ti < facePlane.size() && facePlane[ti].visited)
            ++ti;
        if (ti == facePlane.size())
            break;

        vector<int> cur;
        queue<int> qi;
        qi.push(ti);

        while(!qi.empty()) 
        {
            ti = qi.front();
            qi.pop();
            Tri& t = facePlane[ti];
            if (t.visited)
                continue;
            t.visited = true;
            cur.push_back(ti);
            for(int ni: sedgeToFace[t.s1]) {
                if (ni != ti)
                    qi.push(ni);
            }
            for(int ni: sedgeToFace[t.s2]) {
                if (ni != ti)
                    qi.push(ni);
            }
            for(int ni: sedgeToFace[t.s3]) {
                if (ni != ti)
                    qi.push(ni);
            }
        }
        
        vector<pair<int, int>> unpaired;
        // find all the unpaired sedges
        for(int fi: cur) {
            Tri& t = facePlane[fi];
            if (sedgeToFace[t.s1].size() == 1)
                unpaired.push_back(t.s1);
            if (sedgeToFace[t.s2].size() == 1)
                unpaired.push_back(t.s2);
            if (sedgeToFace[t.s3].size() == 1)
                unpaired.push_back(t.s3);
        }

        cout << "fill " << cur.size() << " unpaired=" << unpaired.size() << endl;

        while (!unpaired.empty())
        {
            vector<int> polyline;
            int len = 0;
            int atun = 0;
            int curv = unpaired[0].first;
            int stv = curv;
            int env = unpaired[0].second;
            unpaired.erase(unpaired.begin()+atun);
            while(curv != env) 
            {
                polyline.push_back(curv);
                //cout << "    " << curv << endl;
                int nextv = -1;
                for(int i = 0; i < unpaired.size(); ++i) {
                    auto& up = unpaired[i];
                    //if (i == atun)
                    //    continue;
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
                unpaired.erase(unpaired.begin()+atun);
                ++len;
            }
            polyline.push_back(curv);

            cout << "  e=" << env << "(l=" << vtxToLine[env] << ")" << " s=" << stv << "(l=" << vtxToLine[stv] << ")  len=" << len << endl;
            if (polyline.size() > 10) {
                polys.push_back(polyline);
            }
        }
    }

    for(int polyi = 0; polyi < polys.size(); ++polyi)
    {
        auto& polyline = polys[polyi];

        vector<int> dwp;
        set<int> added; // avoid duplicates that are caused by holes in the mesh
        int pi = polyline[0];
        auto last = pntsAll[pi];
        dwp.push_back(pi);
        for(int pi: polyline) {
            auto p = pntsAll[pi];
            if (dist(p, last) > 6.0 && (added.count(pi) == 0)) {
                dwp.push_back(pi);
                added.insert(pi);
                last = p;
            }
        }

        string name = outname + "_" + to_string(polyi) + ".obj";
        cout << "  ===" << name << endl;
        ofstream ofsp(name);
        if (!ofsp.good()) {
            cout << "failed opening " << outname << endl;
            return;
        }
        for(int pi: dwp) {
            auto p = pntsAll[pi];
            auto tv = toHtmlCoord(Vec2(p.x, p.z));
            ofsp << "v " << tv.x  << " " << tv.y << "\n";
        }
    }



}








