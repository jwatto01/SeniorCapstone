#include "mainwindow.h"
#include <QApplication>
#include <string>
#include <iostream>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    string b;
    std::cin >> b;
    return a.exec();
}
