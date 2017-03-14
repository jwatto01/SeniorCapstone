#include "mainwindow.h"
#include "ui_mainwindow.h"


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
    }
}

void MainWindow::on_loadDataBtn_clicked()
{
    //call helper function and read in file
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
