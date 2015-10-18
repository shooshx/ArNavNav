#ifndef NAVWEB_H
#define NAVWEB_H

#include <QDialog>
#include "ui_NavWeb.h"
#include "js/js_main.h"
#include <string>

class QMyWebPage;

std::string fromq(const QString& qs);

class Actions : public QObject
{
    Q_OBJECT
public slots:
    Q_INVOKABLE void cpp_start() {
        ::cpp_start();
    }
    Q_INVOKABLE void added_poly_point(int x, int y) {
        ::added_poly_point(x, y);
    }
    Q_INVOKABLE void moved_object(PTR_T ptr, int x, int y) {
        ::moved_object(ptr, x, y);
    }
    Q_INVOKABLE void started_new_poly() {
        ::started_new_poly();
    }
    Q_INVOKABLE void added_agent(int x, int y, float radius, float speed) {
        ::added_agent(x, y, radius, speed);
    }
    Q_INVOKABLE PTR_T add_goal(int x, int y, float radius, int type) {
        return ::add_goal(x, y, radius, type);
    }
    Q_INVOKABLE void remove_goal(PTR_T ptr) {
        ::remove_goal(ptr);
    }
    Q_INVOKABLE void set_goal(PTR_T agentPtr, PTR_T goalPtr) {
        ::set_goal(agentPtr, goalPtr);
    }
    Q_INVOKABLE void cpp_progress(float deltaSec) {
        ::cpp_progress(deltaSec);
    }
    Q_INVOKABLE QString serialize() {
        return QString(::serialize());
    }
    Q_INVOKABLE void deserialize(QString qs) {
        auto s = fromq(qs);
        ::deserialize(s.c_str());
    }
    Q_INVOKABLE void go_to_frame(int f) {
        ::go_to_frame(f);
    }
    Q_INVOKABLE void change_size(PTR_T ptr, float size) { // of agent
        ::change_size(ptr, size);
    }
    Q_INVOKABLE void update_goal(PTR_T ptr, float radius, int type) {
        ::update_goal(ptr, radius, type);
    }
    Q_INVOKABLE void add_imported(QString qname, QString qtext) {
        auto name = fromq(qname);
        auto text = fromq(qtext);
        ::add_imported(name.c_str(), text.c_str());
    }
};

class NavWeb : public QDialog
{
    Q_OBJECT

public:
    NavWeb(QWidget *parent = 0);
    ~NavWeb();

public slots:
    void populateJavaScriptWindowObject();
    void pageLoadFinished(bool ok);

private:
    Ui::NavWeb ui;
    Actions m_actions;
    QMyWebPage *m_page;
};

#endif // NAVWEB_H
