#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("灭火小车控制窗口"); // 设置窗口名称
    w.show();
    return a.exec();
}
