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

    void on_loadDataBtn_clicked();

    void on_calibrateBtn_clicked();

    void on_readNoiseBtn_clicked();

    void on_stopTrackingBtn_clicked();

    void on_comboBox_currentIndexChanged(int index);

private:
    Ui::MainWindow *ui;
    PosCalculator driver;
    bool findLocations;
};

#endif // MAINWINDOW_H
