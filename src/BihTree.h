#pragma once

#include <functional>
#include <vector>
#include "Vec2.h"

class Object;

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
    BihTree()
    {}
    ~BihTree() {
        m_nodes.clear();
    }
    NodeRef buildRec(int begin, int end, int axis);

    void build(std::vector<Object*>& objs)
    {
        m_obj = objs;
        m_nodes.clear();
        m_nodes.reserve(m_obj.size()); // wild guess
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

    std::vector<Object*> m_obj; // needs to be a copy of the objects list since we're going to rearrange in (nth_element)
    std::vector<Node> m_nodes;

};