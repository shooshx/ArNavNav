#include "NavWeb.h"
#include <QWebFrame>
#include <QNetworkRequest>
#include <iostream>

using namespace std;

string fromq(const QString& qs) {
    wstring ws((wchar_t*)qs.data());
    string s;
    for(auto c: ws)
        s.push_back(c);
    return s;
}

class QMyNetworkAccessManager : public QNetworkAccessManager {
public:
    virtual QNetworkReply *	createRequest(Operation op, const QNetworkRequest & req, QIODevice * outgoingData) {
        auto url = req.url();
        auto urls = url.toString();
        //cout << "REQ " << fromq(urls) << endl;
        QNetworkRequest nreq(req);
        if (urls.contains("js_main.js")) {
            url.setPath("non_existing_");
            nreq.setUrl(url);
        }
        return QNetworkAccessManager::createRequest(op, nreq, outgoingData);
    }
};

class QMyWebPage : public QWebPage {
public:
    QMyWebPage(QObject* parent) : QWebPage(parent) {
        setNetworkAccessManager(new QMyNetworkAccessManager);
    }
protected:
    virtual void javaScriptAlert(QWebFrame * frame, const QString & msg) {
        cout << "JSALERT " << fromq(msg) << endl;
    }
    virtual void javaScriptConsoleMessage(const QString & message, int lineNumber, const QString & sourceID ) {
        cout << "JS " << fromq(sourceID) << ":" << lineNumber << " " << fromq(message) << endl;
    }
};

QMyWebPage* g_page = nullptr;

NavWeb::NavWeb(QWidget *parent)
    : QDialog(parent)
{
    ui.setupUi(this);

    m_page = new QMyWebPage(ui.webView);
    auto url = QUrl::fromLocalFile("C:\\projects\\nav\\js\\page.html");
    m_page->mainFrame()->load(url);
    connect(m_page->mainFrame(), SIGNAL(javaScriptWindowObjectCleared()), this, SLOT(populateJavaScriptWindowObject()));
    connect(m_page->mainFrame(), SIGNAL(loadFinished(bool)), this, SLOT(pageLoadFinished(bool)));
    //ui.htmlView->load(QUrl("qrc:/html/page.html"));
    ui.webView->setPage(m_page);
    g_page = m_page;
}

NavWeb::~NavWeb()
{
}

void NavWeb::populateJavaScriptWindowObject() {
    QWebFrame *frame = ui.webView->page()->mainFrame();
    frame->addToJavaScriptWindowObject("qactions", &m_actions);
}

void NavWeb::pageLoadFinished(bool ok) 
{
    cout << "PageLoad " << ok << endl;
}

void js_run(const string& js)
{
    g_page->mainFrame()->evaluateJavaScript(js.c_str());
}
