#include "NavDialog.h"
#include "NavWeb.h"
#include <QtWidgets/QApplication>
#include <iostream>

void cpp_out(const char* s) {
    cout << s << endl;
}

void terrainExtract();

int main(int argc, char *argv[])
{
    //terrainExtract();
    //return 0;

    QApplication a(argc, argv);
    //qui::NavDialog w;  // qt app
    NavWeb w;  // webkit wrapper
    w.show();
    return a.exec();
}
