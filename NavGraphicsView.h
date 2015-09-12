#pragma once

#include <QGraphicsView>
#include <QGraphicsItem>
#include "Document.h"
#include "Agent.h"

class NavDialog;


class NavGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    NavGraphicsView(QWidget *parent = 0);
    virtual ~NavGraphicsView() {}

    NavDialog* m_ctrl = nullptr;

protected:
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent * event);

};

class BaseItem : public QGraphicsItem
{
public:
    BaseItem(NavDialog* ctrl, Object* obj);
    virtual QString strDim() const = 0;
    virtual void setDim(const QString& s) = 0;

    void preparePainter(QPainter* painter, const QStyleOptionGraphicsItem* option);
    virtual void mousePressEvent(QGraphicsSceneMouseEvent * event);
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent * event);

    NavDialog* m_ctrl;
    Object* m_obj;
};


class CircleItem : public BaseItem
{
public:
    CircleItem::CircleItem(Circle* obj, NavDialog* ctrl)
        :BaseItem(ctrl, obj), m_cobj(obj)
    {}

    virtual QString strDim() const {
        return QString("%1").arg(m_obj->size.x / 2);
    }
    virtual void setDim(const QString& s) {
        auto v = s.toFloat() * 2;
        m_obj->setSize(Vec2(v, v));
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    virtual QRectF boundingRect() const;

   // virtual QVariant itemChange(GraphicsItemChange change, const QVariant &value);
   // virtual void mouseMoveEvent(QGraphicsSceneMouseEvent * event);


    Circle* m_cobj;
};

//#define VELOCITY_SCALE 30.0f
#define VELOCITY_SCALE 10.0f

// little indicators of velocity
class VelocityItem : public CircleItem
{
public:
    VelocityItem(Agent* obj, Circle* circ, NavDialog* ctrl)
        :CircleItem(circ, ctrl), m_cobj(obj)
    {}

    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent * event);

    Agent* m_cobj;
};

class AgentItem : public CircleItem
{
public:
    AgentItem(Agent* obj, VelocityItem* vel, NavDialog* ctrl)
        :CircleItem(obj, ctrl), m_vel(vel), m_cobj(obj)
    {}

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    virtual QRectF boundingRect() const;
    void mouseMoveEvent(QGraphicsSceneMouseEvent * event);

    VelocityItem* m_vel;
    Agent* m_cobj;
};

// velocity objects
class VOSItem : public QGraphicsItem
{
public:
    VOSItem(Agent* ofAgent, NavDialog* ctrl) :m_ctrl(ctrl), m_agent(ofAgent)
    {
        setCacheMode(NoCache);
        m_data = &m_ownData;
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    virtual QRectF boundingRect() const;

    VODump m_ownData;
    VODump *m_data;
    NavDialog* m_ctrl;
    Agent* m_agent;
    Vec2 m_ghostPos = INVALID_VEC2;
};



class AABBItem : public BaseItem
{
public:
    AABBItem(AABB* obj, NavDialog* ctrl)         
        :BaseItem(ctrl, obj), m_cobj(obj)
    {}
    virtual QString strDim() const {
        return QString("%1,%2").arg(m_obj->size.x).arg(m_obj->size.y);
    }
    virtual void setDim(const QString& s) {
        auto f = s.split(',');
        m_obj->setSize(Vec2(f[0].toFloat(), f[1].toFloat()));
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    virtual QRectF boundingRect() const;

    AABB* m_cobj;

};


class SegmentItem : public BaseItem
{
public:
    SegmentItem(Segment* obj, NavDialog* ctrl)
        :BaseItem(ctrl, obj), m_cobj(obj)
    {}

    virtual QString strDim() const {
        return QString();
    }
    virtual void setDim(const QString& s) {
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    virtual QRectF boundingRect() const;

    Segment* m_cobj;
};


// part of a triangulation
class TriItem  : public QGraphicsItem
{
public:
    TriItem(NavDialog* ctrl, Triangle* t) :m_ctrl(ctrl), m_t(t)
    {
        setCacheMode(NoCache);
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    virtual QRectF boundingRect() const;

    NavDialog* m_ctrl;
    Triangle* m_t;
};

class PolyPointItem : public QGraphicsItem 
{
public:
    PolyPointItem(NavDialog* ctrl, Vertex* v) :m_ctrl(ctrl), m_v(v)
    {
        setFlag(ItemIsMovable);
        setFlag(ItemIsSelectable);
        setCacheMode(NoCache);
        setPos(m_v->p.x, m_v->p.y);
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    virtual QRectF boundingRect() const;

    void mouseMoveEvent(QGraphicsSceneMouseEvent * event);

    NavDialog* m_ctrl;
    Vertex* m_v;
    QColor m_color = QColor(50, 50, 50);
    int m_radius = 5;
};

class GoalItem : public QGraphicsItem
{
public:
    GoalItem(NavDialog* ctrl, Goal* g) :m_ctrl(ctrl), m_g(g)
    {
        setFlag(ItemIsMovable);
        setFlag(ItemIsSelectable);
        setCacheMode(NoCache);
        setPos(m_g->p.x, m_g->p.y);
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    virtual QRectF boundingRect() const;

    void mouseMoveEvent(QGraphicsSceneMouseEvent * event);

    NavDialog* m_ctrl;
    Goal* m_g;
    QColor m_color = QColor(50, 50, 50);
    int m_radius = 5;
};

class MapDefItem : public QGraphicsItem 
{
public:
    MapDefItem(NavDialog* ctrl, MapDef* p) :m_ctrl(ctrl), m_p(p)
    {
        setCacheMode(NoCache);
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    virtual QRectF boundingRect() const;

    NavDialog* m_ctrl;
    MapDef* m_p;
};

class PathItem : public QGraphicsItem
{
public:
    PathItem(NavDialog* ctrl, Agent* agent) :m_ctrl(ctrl), m_agent(agent)
    {
        setCacheMode(NoCache);
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    virtual QRectF boundingRect() const;

    NavDialog* m_ctrl;
    vector<Vec2> m_pos;
    vector<Vec2> m_vel;
    Agent* m_agent;
    int m_atframe = -1; // -1 means no frame;
};


