#ifndef NAV_H
#define NAV_H

#include <QtWidgets/QDialog>
#include "ui_nav.h"

#include <memory>
using namespace std;

namespace qui {

class NavDialog : public QDialog
{
    Q_OBJECT
public:
    NavDialog(QWidget *parent = 0);
    ~NavDialog() {}

    void readDoc();
    void readMesh();
    void readPolyPoints();

    void update();

    void pointClicked(const Vec2& p);

public slots:
    void on_dimEdit_textChanged(const QString& str);
    void on_addPolyBut_toggled(bool checked);
    void on_actionSave_triggered(bool);
    void on_actionLoad_triggered(bool);
    void on_actionReload_triggered(bool);
    void on_frameSlider_valueChanged(int);

private:
    void updateSliderVOs(int v);

public:
    Ui::navClass ui;

    QGraphicsScene* m_scene = nullptr;
    Document* m_doc = nullptr;

    vector<BaseItem*> m_gobjs;

    BaseItem *m_mark1 = nullptr, *m_mark2 = nullptr;

    shared_ptr<VOSItem> m_vos;
    vector<Vec2> m_currentPoly;

    vector<shared_ptr<BaseItem>> m_objects;
    vector<shared_ptr<TriItem>> m_meshitems;
    vector<shared_ptr<PolyPointItem>> m_polypointitems;
    shared_ptr<MapDefItem> m_mapitem;
    //shared_ptr<PolyPointItem> m_startitem;
    vector<shared_ptr<GoalItem>> m_goalitems;
    vector<shared_ptr<PolyPointItem>> m_markeritems;
    vector<shared_ptr<PathItem>> m_pathitems; // for every agent in m_doc->m_agents
    PathItem* m_probPath = nullptr; // used for ghost vo

    vector<VODump> m_pathVos;
};

} // qui

#endif // NAV_H
