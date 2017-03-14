#include "mainwindow.h"
#include <QApplication>
#include <PosCalculator.h>
#include <Magnet.h>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
