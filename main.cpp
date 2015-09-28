#include "NavDialog.h"
#include "NavWeb.h"
#include <QtWidgets/QApplication>
#include <iostream>

void cpp_out(const char* s) {
    cout << s << endl;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //NavDialog w;
    NavWeb w;
    w.show();
    return a.exec();
}
