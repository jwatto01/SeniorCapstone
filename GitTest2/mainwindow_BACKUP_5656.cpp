#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
<<<<<<< HEAD
    ui->setupUi();
    //blah
=======
    ui->setupUi(notthis);
>>>>>>> 805e6d176832eb6703845869ec748b5cb561120e
}

MainWindow::~MainWindow()
{
    delete ui;
}
