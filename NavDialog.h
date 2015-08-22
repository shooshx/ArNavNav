#ifndef NAV_H
#define NAV_H

#include <QtWidgets/QDialog>
#include "ui_nav.h"

#include <memory>
using namespace std;

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

public:
    Ui::navClass ui;

    QGraphicsScene* m_scene = nullptr;
    Document* m_doc = nullptr;

    vector<BaseItem*> m_gobjs;

    BaseItem *m_mark1 = nullptr, *m_mark2 = nullptr;

    VOSItem* m_vos = nullptr;
    vector<Vec2> m_currentPoly;

    vector<shared_ptr<TriItem>> m_meshitems;
    vector<shared_ptr<PolyPointItem>> m_polypointitems;
    shared_ptr<MapDefItem> m_mapitem;
    shared_ptr<PolyPointItem> m_startitem, m_enditem;
    shared_ptr<PathItem> m_pathitem;
};

#endif // NAV_H
