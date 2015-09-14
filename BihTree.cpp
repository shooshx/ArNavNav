#include "BihTree.h"
#include "Except.h"

#include <algorithm>
#include <string.h>

#define MAX_IN_LEAF 5

static bool compX(const Object* a, const Object* b) {
    return a->m_position.x < b->m_position.x;
}
static bool compY(const Object* a, const Object* b) {
    return a->m_position.y < b->m_position.y;
}



BihTree::NodeRef BihTree::buildRec(int begin, int end, int axis) 
{
    CHECK(begin != end, "Empty node?");

    NodeRef newNode = m_nodes.size();
    m_nodes.resize(newNode + 1);
    Node* n = &m_nodes[newNode];

    n->beginObj = begin;
    n->endObj = end;
    if (end - begin <= MAX_IN_LEAF)
        return newNode;

    int mid = begin + (end - begin)/2;

    if (axis == 0)
        std::nth_element(m_obj.begin() + begin, m_obj.begin() + mid, m_obj.begin() + end, compX);
    else
        std::nth_element(m_obj.begin() + begin, m_obj.begin() + mid, m_obj.begin() + end, compY);

    float maxOfLeft = m_obj[begin]->m_position.v[axis] + m_obj[begin]->size.v[axis] * 0.5;
    for(int i = begin + 1; i < mid; ++i) {
        float v = m_obj[i]->m_position.v[axis] + m_obj[i]->size.v[axis];
        if (v > maxOfLeft)
            maxOfLeft = v;
    }
    float minOfRight = m_obj[mid]->m_position.v[axis] - m_obj[mid]->size.v[axis] * 0.5;
    for(int i = mid + 1; i < end; ++i) {
        float v = m_obj[i]->m_position.v[axis] - m_obj[i]->size.v[axis] * 0.5;
        if (v < minOfRight)
            minOfRight = v;
    }
    n->axis = axis;
    n->maxOfLeft = maxOfLeft;
    n->minOfRight = minOfRight;
    n = nullptr;
    // these calls may reallocate the vector so we won't be able to use n anymore
    NodeRef left = buildRec(begin, mid, 1 - axis);
    NodeRef right = buildRec(mid, end, 1 - axis);

    m_nodes[newNode].left = left;
    m_nodes[newNode].right = right;

    return newNode;
}



float BihTree::avgNodeSize() 
{
   float sum = 0, count = 0;
   for(auto &n: m_nodes) {
       if (n.left == 0) {
           sum += n.endObj - n.beginObj;
           ++count;
       }
   }
   float avg = sum/count;
   return avg;
}

void BihTree::recQuery(NodeRef noderef, QueryState& qs) 
{
    Node* node = &m_nodes[noderef];
    if (node->left == 0) { // its a leaf
        for(int i = node->beginObj; i < node->endObj; ++i) {
            qs.callback(m_obj[i]);
        }
        return;
    }

    float qcoord = qs.coord.v[node->axis];
    float qmax = qcoord + qs.radius;
    float qmin = qcoord - qs.radius;
    if (qmax < node->minOfRight) { // need only left
        recQuery(node->left, qs);
    }
    else if (qmin > node->maxOfLeft) { // need only right
        recQuery(node->right, qs);
    }
    else {
        recQuery(node->left, qs);
        recQuery(node->right, qs);
    }

}

void BihTree::query(const Vec2& coord, float radius, const std::function<void(Object*)>& callback) {
    QueryState qs{coord, radius, callback};
    recQuery(0, qs);
}
