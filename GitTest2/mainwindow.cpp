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
    QCPScatterStyle myScatter;
    myScatter.setShape(QCPScatterStyle::ssDisc);
    myScatter.setPen(QPen(Qt::blue));
    myScatter.setBrush(Qt::white);
    myScatter.setSize(5);

    ui->figure_1->addGraph();
    ui->figure_2->addGraph();
    ui->figure_3->addGraph();
    ui->figure_4->addGraph();
    ui->figure_5->addGraph();
    ui->figure_1->graph(0)->setScatterStyle(myScatter);
    ui->figure_2->graph(0)->setScatterStyle(myScatter);
    ui->figure_1->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->figure_2->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->figure_1->yAxis->setRange(-0.25,0.25);
    ui->figure_1->xAxis->setRange(-0.25,0.25);
    ui->figure_2->yAxis->setRange(-0.25,0.25);
    ui->figure_2->xAxis->setRange(-0.25,0.25);
    ui->figure_1->yAxis->setLabel("Y-axis (m)");
    ui->figure_1->xAxis->setLabel("X-axis (m)");
    ui->figure_2->yAxis->setLabel("Z-axis (m)");
    ui->figure_2->xAxis->setLabel("X-axis (m)");


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
    Vector3d curPosition;
    int trackCount = 0;
    while (findLocations){
        curPosition = driver.startTracking();
        //only plot values once we've locked into a solution
        if(trackCount >= 10){
            ui->figure_1->graph(0)->addData(curPosition(0),curPosition(1));
            ui->figure_2->graph(0)->addData(curPosition(0),curPosition(2));
            ui->figure_1->replot();
            ui->figure_2->replot();
        }
        trackCount++;
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
    ui->progressBar->setVisible(true);
    ui->progressBar->setValue(0);
    bool gathering = false;
    for(int i = 0; i<100; i++){
        driver.gatherSampleCovarData(gathering);
        if(i==0)
            gathering = true;
        ui->progressBar->setValue(i);
    }
    driver.storeNoiseData();
    ui->progressBar->setVisible(false);
}

void MainWindow::on_stopTrackingBtn_clicked()
{
    findLocations = false;
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    MatrixXd curInitDataSample = allSensors.at(index).getInitialDataSample();
    QVector<double> xVals(100), yVals(100);
    VectorXd xVect = curInitDataSample.block(1,1,100,1);
    VectorXd yVect = curInitDataSample.block<100,1>(1,2);
    memcpy(xVals.data(),xVect.data(),sizeof(double)*100);
    memcpy(yVals.data(),yVect.data(),sizeof(double)*100);
    ui->figure_3->graph(0)->setData(xVals,yVals);
}
