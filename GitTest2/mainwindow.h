#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <PosCalculator.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_StartTrackingBtn_clicked();

private:
    Ui::MainWindow *ui;
    PosCalculator driver;
};

#endif // MAINWINDOW_H
