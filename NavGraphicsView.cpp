#include "NavGraphicsView.h"
#include "NavDialog.h"
#include <QMouseEvent>
#include <iostream>


BaseItem::BaseItem(NavDialog* ctrl, Object* obj) 
    : m_ctrl(ctrl), m_obj(obj) 
{
    setFlag(ItemIsMovable);
    setFlag(ItemIsSelectable);
    setCacheMode(NoCache);
    setPos(m_obj->m_position.x, m_obj->m_position.y);
}

void BaseItem::preparePainter(QPainter* painter, const QStyleOptionGraphicsItem* option)
{
    QColor bc = QColor(255, 0, 0);
    if (m_obj->highlight) {
        bc = QColor(255,255,0);
    }
    if (option->state & QStyle::State_Selected) {
        bc = bc.lighter(170); // currently pressed.
    }
    painter->setBrush(QBrush(bc));
}

static ostream& operator<<(ostream& os, const Vec2& p) {
    os << p.x << "," << p.y;
    return os;
}

/*
std::pair<Vec2, Vec2> testDet(AABB* otherAab, const Vec2& m_position)
{
    Vec2 mp1, mp2;
    otherAab->spanningPoints(m_position, &mp1, &mp2);
    cout << det(mp1 - m_position, mp2 - m_position) << endl;
    return make_pair(mp1, mp2);
}
*/

void BaseItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event) {
    QGraphicsItem::mouseMoveEvent(event);
    auto p = pos();
    Vec2 vp(p.x(), p.y());
    m_obj->setPos(vp);

   // cout << sqrt(m_ctrl->m_doc->m_prob->distSqToSurface(vp)) << endl;
  //  auto r = testDet(dynamic_cast<AABB*>(m_ctrl->m_doc->m_prob), vp);

   // cout << r.first << "   " << r.second << endl;
  //  m_ctrl->m_mark1->setPos(r.first.x, r.first.y);
  //  m_ctrl->m_mark2->setPos(r.second.x, r.second.y);

    m_ctrl->update();
}


void BaseItem::mousePressEvent(QGraphicsSceneMouseEvent * event) {
    QGraphicsItem::mousePressEvent(event);
    m_ctrl->update();
}

//QVariant CircleItem::itemChange(GraphicsItemChange change, const QVariant &value)
//{
//    return QGraphicsItem::itemChange(change, value);
//}

static QPointF toQ(const Vec2& v) {
    return QPointF(v.x, v.y);
}

// ---------------------------------------

void CircleItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    preparePainter(painter, option);
    int radius = m_obj->size.x / 2;
    painter->drawEllipse(-radius, -radius, 2 * radius, 2 * radius);
}


QRectF CircleItem::boundingRect() const
{
    int radius = m_obj->size.x / 2;;
    return QRectF(-radius - 2, -radius - 2, 2 * radius + 3, 2 * radius + 3);
}

// -----------------------------------------

void AgentItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    CircleItem::paint(painter, option, widget);
    if (m_vel) {
        QPen pen(QColor(0,0,0));
        pen.setWidth(2);
        painter->setPen(pen);
        painter->drawLine(QPointF(0,0), m_vel->pos() - pos());
    }
}
QRectF AgentItem::boundingRect() const {
    return CircleItem::boundingRect();
}

void AgentItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event) {
    CircleItem::mouseMoveEvent(event);
    if (m_vel) {
        Vec2 velpos = m_cobj->m_position + m_cobj->m_velocity * VELOCITY_SCALE;
        m_vel->setPos(toQ(velpos));
    }
    m_ctrl->update();
}

void VelocityItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event) {
    CircleItem::mouseMoveEvent(event);
    auto p = pos();
    Vec2 me(p.x(), p.y());
    m_cobj->m_velocity = (me - m_cobj->m_position) / VELOCITY_SCALE;
    m_ctrl->update();
}

// -----------------------------------------------------



void VOSItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) 
{
    
    Vec2 center = m_agent->m_position;
    for(const auto& vo: m_data.vos) {
        QPointF p[3];
        p[0] = toQ(center + vo.m_apex);
        p[1] = toQ(center + vo.m_apex + vo.m_side1 * 200.0f);
        p[2] = toQ(center + vo.m_apex + vo.m_side2 * 200.0f);
        if (det(vo.m_side1, vo.m_side2) < 0)
            painter->setBrush(QBrush(QColor(150, 0, 0)));
        else
            painter->setBrush(QBrush(QColor(150, 150, 255)));
        painter->drawPolygon(p, 3);
    }
    painter->setBrush(QBrush(QColor(50, 50, 255)));
    for(auto it = m_data.candidates.begin(); it != m_data.candidates.end(); ++it) {
        auto& ca = it->second;
        auto q = toQ(center + ca.m_position * VELOCITY_SCALE);
        painter->drawEllipse(q, 4, 4);

    }

}

QRectF VOSItem::boundingRect() const {
    return QRectF();
}


// --------------------------------------------------------


void AABBItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    preparePainter(painter, option);
    auto sz = m_obj->size / 2;
    painter->drawRect(-sz.x, -sz.y, sz.x, sz.y); // object is centered at 0,0
}

QRectF AABBItem::boundingRect() const
{
    auto sz = m_obj->size / 2;
    return QRectF(-sz.x, -sz.y, sz.x, sz.y);
}

// --------------------------------------------------------


void SegmentItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    preparePainter(painter, option);
    auto sz = m_obj->size / 2;
    painter->drawLine(-sz.x, -sz.y, sz.x, sz.y);
}

QRectF SegmentItem::boundingRect() const
{
    auto sz = m_obj->size / 2;
    return QRectF(-sz.x, -sz.y, m_obj->size.x, m_obj->size.y);
}

// ------------------------------------------------------------------

void TriItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) 
{
    QPointF pv[] = { toQ(m_t->v[0]->p), toQ(m_t->v[1]->p), toQ(m_t->v[2]->p) };
    QPen pen(QColor(0,0,0));
    pen.setWidth(1);
    painter->setPen(pen);
    //painter->drawPolyline(pv, 3);
    QColor col(150, 150, 255);
    if (m_t->highlight == 1) 
        col = QColor(0, 190, 0);
    else if (m_t->highlight == 2)
        col = QColor(190, 0, 0);
    else if (m_t->highlight == 3)
        col = QColor(255, 150, 150);

    painter->setBrush(QBrush(col));
    painter->drawPolygon(pv, 3);

    painter->setPen(QPen());
    for(int i = 0; i < 3; ++i) {
        painter->drawEllipse(toQ(m_t->h[i]->midPnt), 2, 2);
    }
}
QRectF TriItem::boundingRect() const {
    Vec2 mx = m_t->v[0]->p, mn = m_t->v[0]->p;
    mx.mmax(m_t->v[1]->p);
    mn.mmin(m_t->v[1]->p);
    mx.mmax(m_t->v[2]->p);
    mn.mmin(m_t->v[2]->p);

    return QRectF(toQ(mn), toQ(mx));
}

// ------------------------------------------------------------------

void PolyPointItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) 
{
    auto qp = pos();
    //Vec2 p(pos().x(), pos.y());
    painter->setBrush(QBrush(m_color));
    painter->drawEllipse(-m_radius, -m_radius, 2 * m_radius, 2 * m_radius);
}
QRectF PolyPointItem::boundingRect() const {
    return QRectF(-m_radius - 2, -m_radius - 2, 2 * m_radius + 3, 2 * m_radius + 3);
}


void PolyPointItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event) {
    QGraphicsItem::mouseMoveEvent(event);
    auto p = pos();
    m_v->p = Vec2(p.x(), p.y()); 

    m_ctrl->update();
}

// ------------------------------------------------------------------

void MapDefItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) 
{
    QPen pen(QColor(0,0,0));
    pen.setWidth(1);
    painter->setPen(pen);
    painter->setBrush(QBrush());
    for(auto* pl: m_p->m_p) {
        if (pl->m_d.size() == 0)
            continue;
        vector<QPointF> qp;
        for(auto* pv: pl->m_d) {
            qp.push_back(toQ(pv->p));
        }
        painter->drawPolygon(&qp[0], qp.size());
    }
}
QRectF MapDefItem::boundingRect() const {
    return QRectF(-350, -350, 700, 700);
}

// ------------------------------------------------------------------

void PathItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) 
{
    if (m_v->size() == 0)
        return;
    QPen pen(QColor(0,0,0));
    pen.setWidth(2);
    painter->setPen(pen);
    QVector<QPointF> lp;
    for(auto v: *m_v) {
        lp.append(toQ(v));
    }
    painter->drawPolyline(lp);
}
QRectF PathItem::boundingRect() const {
    return QRectF(-350, -350, 700, 700);
}

// -----------------------------------------------------------------


NavGraphicsView::NavGraphicsView(QWidget *parent)
    : QGraphicsView(parent)
{
}

void NavGraphicsView::mousePressEvent(QMouseEvent *event)
{
    QPointF sp = mapToScene(event->pos());

    QGraphicsView::mousePressEvent(event);
    if (!event->isAccepted()) {
        m_ctrl->pointClicked(Vec2(sp.x(), sp.y()));
    }
}

void NavGraphicsView::mouseReleaseEvent(QMouseEvent *event)
{
    QGraphicsView::mouseReleaseEvent(event);

}

void NavGraphicsView::mouseMoveEvent(QMouseEvent * event)
{
    QGraphicsView::mouseMoveEvent(event);
}