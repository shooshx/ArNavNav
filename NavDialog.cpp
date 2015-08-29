#include "NavDialog.h"
#include "NavGraphicsView.h"
#include "BihTree.h"
#include "Agent.h"
#include "Simulator.h"
#include <QGraphicsScene>
#include <QMenu>
#include <QFileDialog>
#include <iostream>
#include <fstream>
#include <sstream>


NavDialog::NavDialog(QWidget *parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    auto menu = new QMenu(this);
    ui.fileBut->setMenu(menu);
    menu->addAction(ui.actionLoad);
    menu->addAction(ui.actionSave);

    m_scene = new QGraphicsScene(this);
    ui.gView->setScene(m_scene);
    ui.gView->m_ctrl = this;

    //ui.gView->setSceneRect(-50, -50, 100, 100);
    //m_scene->setSceneRect(-50, -50, 100, 100);
    //m_scene->setSceneRect(-350, -300, 700, 600);
    //m_scene->setSceneRect(-700, -700, 1400, 1400);
    m_scene->setBackgroundBrush(QBrush(QColor(Qt::white)));

    m_doc = new Document(this);
    readDoc();

}


static ostream& operator<<(ostream& os, const Vec2& p) {
    os << p.x << ", " << p.y;
    return os;
}

void NavDialog::readMesh()
{
    m_meshitems.clear();
    for(auto *tri : m_doc->m_mesh.m_tri) {
        auto i = new TriItem(this, tri);
        i->setZValue(-20);
        m_scene->addItem(i);
        m_meshitems.push_back(shared_ptr<TriItem>(i));
    }
}

void NavDialog::readPolyPoints() 
{
    m_polypointitems.clear();
    for(auto *pl : m_doc->m_mapdef.m_p) {
        for(auto pv : pl->m_d) {
            auto i = new PolyPointItem(this, pv);
            m_scene->addItem(i);
            m_polypointitems.push_back(shared_ptr<PolyPointItem>(i));
        }
    }
}

void NavDialog::readDoc()
{
//    m_gobjs.clear();

    if (m_doc->m_prob != nullptr) {
        m_vos = new VOSItem(dynamic_cast<Agent*>(m_doc->m_prob), this);
        m_scene->addItem(m_vos);
        m_vos->setZValue(-7);
        
    }

    m_objects.clear();
    for (auto* obj : m_doc->m_objs) 
    {
        BaseItem *p = nullptr;
        auto a = dynamic_cast<Agent*>(obj);
        if (a != nullptr) {
            VelocityItem *vp = nullptr;
           /* Vec2 velpos = a->m_position + a->m_velocity * VELOCITY_SCALE;
            VelocityItem *vp = new VelocityItem(a, new Circle(velpos, 6.0f, -a->index), this);
            m_scene->addItem(vp);*/
            p = new AgentItem(a, vp, this);
        }
        else {
            auto c = dynamic_cast<Circle*>(obj);
            if (c != nullptr)
                p = new CircleItem(c, this);
            else {
                auto b = dynamic_cast<AABB*>(obj);
                if (b != nullptr) 
                    p = new AABBItem(b, this);
                else {
                    auto s = dynamic_cast<Segment*>(obj);
                    if (s != nullptr)
                        p = new SegmentItem(s, this);
                }
            }
            if (p)
                p->setZValue(-5);
        }
        if (p) {
            m_objects.push_back(shared_ptr<BaseItem>(p));
            m_scene->addItem(p);
        }
        //m_gobjs.push_back(p);
    }

    readMesh();

    m_mapitem.reset(new MapDefItem(this, &m_doc->m_mapdef));
    m_mapitem->setZValue(-10);
    m_scene->addItem(m_mapitem.get());

    readPolyPoints();

    if (m_doc->m_start) {
        m_startitem.reset(new PolyPointItem(this, m_doc->m_start));
        m_startitem->m_color = QColor(50, 255, 50);
        m_startitem->m_radius = 7;
        m_scene->addItem(m_startitem.get());
    }
    if (m_doc->m_end) {
        m_enditem.reset(new PolyPointItem(this, m_doc->m_end));
        m_enditem->m_color = QColor(255, 50, 50);
        m_enditem->m_radius = 7;
        m_scene->addItem(m_enditem.get());
    }
    m_markeritems.clear();
    for(auto* m: m_doc->m_markers) {
        m_markeritems.push_back(shared_ptr<PolyPointItem>(new PolyPointItem(this, m)));
        m_scene->addItem(m_markeritems.back().get());
    }

    m_pathitem.reset(new PathItem(this, &m_doc->m_path));
    m_scene->addItem(m_pathitem.get());

  /*  m_mark1 = new CircleItem(new Circle(Vec2(50, -40), 6, -1), this);
    m_mark2 = new CircleItem(new Circle(Vec2(60, -40), 6, -1), this);
    m_gobjs.push_back(m_mark1);
    m_gobjs.push_back(m_mark2);
    m_scene->addItem(m_mark1);
    m_scene->addItem(m_mark2);*/
}

void NavDialog::on_dimEdit_textChanged(const QString& str) {
    float f = str.toFloat();
    auto items = m_scene->selectedItems();
    for(auto* item: items) {
        dynamic_cast<BaseItem*>(item)->setDim(str);
    }
    m_scene->update();
}

QString strFromVec2(const Vec2& v) {
    return QString("(%1, %2)").arg(v.x).arg(v.y);
}

static QPointF toQ(const Vec2& v) {
    return QPointF(v.x, v.y);
}



void NavDialog::update()
{
    auto items = m_scene->selectedItems();
    if (items.size() > 0)
    {
        auto item = dynamic_cast<BaseItem*>(items[0]);
        if (item != nullptr) {
            ui.dimEdit->setText(item->strDim());
            ui.coordText->setText("Coord: " + strFromVec2(item->m_obj->m_position));
            ui.indexText->setText("Index: " + QString("%1").arg(item->m_obj->index));
        }
        auto vi = dynamic_cast<PolyPointItem*>(items[0]);
        if (vi != nullptr) {
            ui.coordText->setText("Coord: " + strFromVec2(vi->m_v->p));
            ui.indexText->setText("Index: " + QString("%1").arg(vi->m_v->index));
        }
    }

    m_doc->runTriangulate();
    readMesh();


    for(auto* obj: m_doc->m_objs) {
        obj->highlight = false;
    }

    //---------------- path
    if (m_doc->m_prob) 
    {
        vector<Vec2> startPoss; // save start so we could recreate it
        // simulator test
        Simulator sim;
        for(auto* obj: m_doc->m_objs) 
        {
            sim.addObject(obj);
            startPoss.push_back(obj->m_position);

            auto *agent = dynamic_cast<Agent*>(obj);
            if (!agent)
                continue;
            agent->m_goalPos = m_doc->m_end->p;
            agent->m_velocity = Vec2();
        }

        m_agentPath.clear();
        m_agentPath.push_back( m_doc->m_prob->m_position );
        for(int i = 0; i < 500; ++i) {
            sim.doStep(0.25, true);
            m_agentPath.push_back( m_doc->m_prob->m_position );
        }

        m_pathitem->m_v = &m_agentPath;

        // go back to backup
        int i = 0;
        for(auto* obj: m_doc->m_objs) {
            obj->m_position = startPoss[i++];
            auto *agent = dynamic_cast<Agent*>(obj);
            if (!agent)
                continue;
            agent->m_velocity = Vec2();
        }

        //---------------------- VO

        //if (false) 
        {
            auto* agentProb = dynamic_cast<Agent*>(m_doc->m_prob);
            float origNeiDist = agentProb->m_neighborDist;
            agentProb->m_neighborDist = 50;
            sim.doStep(0.25, false);
            if (m_vos != nullptr)
                agentProb->computeNewVelocity(&m_vos->m_data);
            agentProb->m_neighborDist = origNeiDist;
        }
    }
    //std::cout << m_vos->m_data.selected << endl;
   
    for(int i = 0; i < m_markeritems.size(); ++i)
        m_markeritems[i]->setPos(toQ(m_doc->m_markers[i]->p));

    // markers intersection
/*    if (m_doc->m_markers.size() == 5) {
        Vec2 a = m_doc->m_markers[0]->p;
        Vec2 a2 = m_doc->m_markers[1]->p;
        Vec2 b = m_doc->m_markers[2]->p;
        Vec2 b2 = m_doc->m_markers[3]->p;

        Vec2 v = a2-a;
        Vec2 u = b2-b;
        float k = (v.x*(b.y-a.y) + v.y*(a.x-b.x))/(u.x*v.y - u.y*v.x);
        
        Vec2 p = b+k*u;
        m_doc->m_markers[4]->p = p;
        m_markeritems[4]->setPos(toQ(p));
    }*/


    // BihTree test
/*    BihTree bt;
    bt.build(m_doc->m_objs);

    bt.query(m_doc->m_prob->m_position, m_doc->m_prob->size.x / 2, [](Object* obj) {
        obj->highlight = true;
    });*/

    m_scene->update();
}

void NavDialog::on_addPolyBut_toggled(bool checked)
{
    if (checked)
        m_doc->m_mapdef.add();
}


void NavDialog::pointClicked(const Vec2& p)
{
    if (ui.addPolyBut->isChecked()) {
        auto pv = m_doc->m_mapdef.top()->add(p);
        auto i = new PolyPointItem(this, pv);
        m_scene->addItem(i);
        m_polypointitems.push_back(shared_ptr<PolyPointItem>(i));
        m_mapitem->update();
        update();
    }
}

string sFromWs(const wstring& ws) {
    string out;
    out.reserve(ws.size());
    for(wchar_t c: ws)
        out.append(1, (char)c); // not correct at all
    return out;
}


void NavDialog::on_actionSave_triggered(bool) 
{
    auto name = QFileDialog::getSaveFileName(this, "Save Polyline");
    ofstream ofs(sFromWs(wstring((wchar_t*)name.data())));
    if (!ofs.good())
        return;
    int count = 0;
    for(auto *pl : m_doc->m_mapdef.m_p) {
        ofs << "\npolyline\n";
        for(auto pv : pl->m_d) {
            ofs << "v " << pv->p.x << " " << pv->p.y << "\n";
            ++count;
        }
    }
    ofs << "start " << m_doc->m_start->p.x << " " << m_doc->m_start->p.y << "\n";
    ofs << "end " << m_doc->m_end->p.x << " " << m_doc->m_end->p.y << "\n";
    ofs << "prob " << m_doc->m_prob->m_position.x << " " << m_doc->m_prob->m_position.y << "\n";

    cout << "Saved " << count << " vertices, " << m_doc->m_mapdef.m_p.size() << " polylines" << endl;
}


void NavDialog::on_actionLoad_triggered(bool)
{
    auto name = QFileDialog::getOpenFileName(this, "Load Polyline");
    ifstream ifs( sFromWs(wstring((wchar_t*)name.data())) );
    if (!ifs.good())
        return;

    m_doc->m_mapdef.clear();
    int count = 0;
    while (!ifs.eof()) {
        string line;
        getline(ifs, line);
        istringstream iss(line);
        string h;
        iss >> h;
        if (h == "polyline") {
            m_doc->m_mapdef.add();
        }
        else if (h == "v") {
            Vec2 v;
            iss >> v.x >> v.y;
            m_doc->m_mapdef.top()->add(v);
            ++count;
        }
        else if (h == "start") {
            iss >> m_doc->m_start->p.x >> m_doc->m_start->p.y;
        }
        else if (h == "end") {
            iss >> m_doc->m_end->p.x >> m_doc->m_end->p.y;
        }
        else if (h == "prob") {
            Vec2 v;
            iss >> v.x >> v.y;
            m_doc->m_prob->m_position = v;
        }

    }
    cout << "Read " << count << " vertices " << m_doc->m_mapdef.m_p.size() << " polylines" << endl;
    readDoc();
    update();
}
