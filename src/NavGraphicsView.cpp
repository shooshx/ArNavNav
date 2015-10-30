#include "NavGraphicsView.h"
#include "NavDialog.h"
#include <QMouseEvent>
#include <iostream>

namespace qui {



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
/*    if (m_obj->highlight) {
        bc = QColor(255,255,0);
    }*/
    if (option->state & QStyle::State_Selected) {
        bc = QColor(150, 0, 0); // currently pressed.
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
    Vec2 center = m_ghostPos;
    int alpha = 128;
    if (!m_ghostPos.isValid()) {
        center = m_agent->m_position;
        alpha = 255;
    }
    for(const auto& vo: m_data->vos) {
        QPointF p[3];
        Vec2 apx = center + vo.m_apex * VELOCITY_SCALE;
        p[0] = toQ(apx);
        p[1] = toQ(apx + vo.m_side1 * 200.0f);
        p[2] = toQ(apx + vo.m_side2 * 200.0f);
        if (det(vo.m_side1, vo.m_side2) < 0)
            painter->setBrush(QBrush(QColor(150, 0, 0, alpha)));
        else
            painter->setBrush(QBrush(QColor(150, 150, 255, alpha)));
        painter->drawPolygon(p, 3);
    }
    painter->setBrush(QBrush(QColor(50, 50, 255)));
    for(auto it = m_data->candidates.begin(); it != m_data->candidates.end(); ++it) {
        auto& ca = it->second;
        auto q = toQ(center + ca.m_position * VELOCITY_SCALE);
        painter->drawEllipse(q, 4, 4);
    }
    auto sq = toQ(center + m_data->selected * VELOCITY_SCALE);
    painter->setBrush(QBrush(QColor(50, 255, 255)));
    painter->drawEllipse(sq, 6, 6);
}

QRectF VOSItem::boundingRect() const {
    return QRectF(-2000,-2000,6000,6000);
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
    QColor col(200, 200, 255, 128);
   /* if (m_t->highlight == 1) 
        col = QColor(200, 255, 200);
    else if (m_t->highlight == 2)
        col = QColor(255, 200, 200);
    else if (m_t->highlight == 3)
        col = QColor(255, 255, 200);
     */   
    painter->setBrush(QBrush(col));
    painter->drawPolygon(pv, 3);

    painter->setPen(QPen());
    Vec2 trimid;
    for(int i = 0; i < 3; ++i) {
        painter->drawEllipse(toQ(*m_t->h[i]->curMidPntPtr), 2, 2);
        trimid += m_t->v[i]->p;
    }
    trimid /= 3.0f;

    for(int i = 0; i < 3; ++i) { // HalfEdge number
        const HalfEdge* he = m_t->h[i];
        Vec2 mid = (0.4 * he->to->p + 0.6 * he->from->p);
        float d = length(mid - trimid);
        Vec2 towardsMid = ((mid - trimid)*((d-10)/d)) + trimid;

        painter->drawText(towardsMid.x - 20, towardsMid.y - 20, 40, 40, Qt::AlignCenter, QString("%1").arg(he->index));
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

void GoalItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) 
{
    painter->setBrush(QBrush(m_color));
    painter->drawEllipse(-m_radius, -m_radius, 2 * m_radius, 2 * m_radius);
}
QRectF GoalItem::boundingRect() const {
    return QRectF(-m_radius - 2, -m_radius - 2, 2 * m_radius + 3, 2 * m_radius + 3);
}


void GoalItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event) {
    QGraphicsItem::mouseMoveEvent(event);
    auto p = pos();
    m_g->def.p = Vec2(p.x(), p.y()); 
    for(auto* agent: m_g->agents) {
        agent->setEndGoal(m_g->def, (void*)m_g);
    }
    m_ctrl->update();
}

// ------------------------------------------------------------------

void MapDefItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) 
{
    QPen pen(QColor(0,0,0));
    pen.setWidth(1);
    painter->setPen(pen);
    painter->setBrush(QBrush());
    for(const auto& pl: m_p->m_pl) {
        if (pl->m_di.size() == 0)
            continue;
        vector<QPointF> qp;
        for(int pvi: pl->m_di) {
            qp.push_back(toQ(m_p->m_vtx[pvi]->p));
        }
        painter->drawPolygon(&qp[0], qp.size());
    }
}
QRectF MapDefItem::boundingRect() const {
    return QRectF(-2000, -2000, 6000, 6000);;
}

// ------------------------------------------------------------------

extern Vec2 g1, g2;

void PathItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) 
{
    if (m_pos.size() == 0)
        return;
    QPen pen(QColor(0,0,0));
    pen.setWidth(0.5); // 2
    painter->setPen(pen);
    QVector<QPointF> lp;
    for(auto v: m_pos) {
        lp.append(toQ(v));
    }
    painter->drawPolyline(lp);
    
    if (m_atframe != -1) // ghost
    {
        CHECK(m_atframe < m_pos.size(), "unexpected atframe");
        pen.setWidth(0.5);
        painter->setPen(pen);
        painter->setBrush(QBrush(QColor(255, 200, 200, 180)));
        int radius = m_agent->size.x / 2;
        const auto& pos = m_pos[m_atframe];
        painter->drawEllipse(pos.x - radius, pos.y - radius, 2 * radius, 2 * radius);

        painter->setBrush(QBrush(QColor(255, 255, 0, 180)));
        Vec2 velpnt = pos + m_vel[m_atframe] * VELOCITY_SCALE;
        radius = 5;
        // perfVelocity yellow point
        //painter->drawEllipse(velpnt.x - radius, velpnt.y - radius, 2 * radius, 2 * radius);
    }

    if (m_agent->index != 0)
        return;
    QVector<QPointF> sp;
    for(const auto& subgoal: m_agent->m_plan.m_d) {
        sp.append(toQ(subgoal->representPoint()));
    }

    //painter->drawLine(0, 0, g1.x, g1.y);
    //painter->drawLine(0, 0, g2.x, g2.y);

    QPen pen2(QColor(255,0,0));
    pen2.setWidth(1);
    painter->setPen(pen2);
    painter->drawPolyline(sp);

}
QRectF PathItem::boundingRect() const {
    return QRectF(-2000, -2000, 6000, 6000); //QRectF(-350, -350, 700, 700);
}

// -----------------------------------------------------------------


NavGraphicsView::NavGraphicsView(QWidget *parent)
    : QGraphicsView(parent)
{
    //scale(2,2);
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

} //qui