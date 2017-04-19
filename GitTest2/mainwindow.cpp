#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <fstream>
#include <QString>
#include <PosCalculator.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    findLocations = true;
    ui->setupUi(this);
    //depending on button presses: calibrate -> calibrateSystem
    //Read Noise -> gatherSampleCovarData()
    //StartTracking -> findFirstLocation() -> loop that contains startTracking()
    //within that loop write location at the end of each iteration update Plot function
    //have a way to load in data from text file (different calibrate)"load noise data"
    //find first location needs ot be able to check if the covariance has already been uploaded to sensors
    //before we find the first location we have to make sure that data is already in the sensors



}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_StartTrackingBtn_clicked()
{
    while (findLocations){
        driver.startTracking();
        QCoreApplication::processEvents();
    }
}

void MainWindow::on_loadDataBtn_clicked()
{
    QString filePath = ui->filePathTxt->toPlainText();
    MatrixXd initialDataSample = MatrixXd::Constant(100,3,0.0);
    ifstream myFile;
    myFile.open(filePath.toStdString());
    double a, b, c;
    int i;
    for (int j = 0; j < 8; j++){
        i = 0;
        while ( i < 100 && myFile >> a >> b >> c)
        {
            initialDataSample(i, 0) = a;
            initialDataSample(i, 1) = b;
            initialDataSample(i, 2) = c;
            i++;
        }
        allSensors.at(j).readInitialDataSample(initialDataSample);
    }
    myFile.close();
}

void MainWindow::on_calibrateBtn_clicked()
{
    driver.calibrateSystem();
}

void MainWindow::on_readNoiseBtn_clicked()
{
    driver.gatherSampleCovarData();
}

void MainWindow::on_stopTrackingBtn_clicked()
{
    findLocations = false;
}
