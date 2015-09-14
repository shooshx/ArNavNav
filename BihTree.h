#pragma once

#include "Document.h"
#include <functional>


class BihTree
{
private:
    typedef int NodeRef;
    struct Node {
        int beginObj = 0; // first object
        int endObj = 0;  // one after last object
        NodeRef left = 0;  // smaller items
        NodeRef right = 0; // bigger items
        int axis = 0; // what axis does the division 0=X, 1=Y
        float maxOfLeft = 0;
        float minOfRight = 0;
    };

public:
    ~BihTree() {
        m_obj.clear();
        m_nodes.clear();
    }
    NodeRef buildRec(int begin, int end, int axis);

    template<typename T>
    void build(vector<T*>& objs)
    {
        m_obj.resize(objs.size());
        for(int i = 0; i < objs.size(); ++i)
            m_obj[i] = objs[i];
        m_nodes.reserve(m_obj.size()); // wild guess
        m_nodes.clear();
        buildRec(0, m_obj.size(), 1);
    }

    struct QueryState {
        Vec2 coord; 
        float radius;
        std::function<void(Object*)> callback;
    };
    void recQuery(NodeRef node, QueryState& qs);
    void query(const Vec2& coord, float radius, const std::function<void(Object*)>& callback);

    float avgNodeSize();

private:

    vector<Object*> m_obj;
    vector<Node> m_nodes;

};