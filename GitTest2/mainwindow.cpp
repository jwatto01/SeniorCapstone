#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <fstream>
#include <QString>
#include <PosCalculator.h>
#include <math.h>
double gaussianFunction(double std, double mean, double x);

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
    ui->figure_3->yAxis->setLabel("Probability");
    ui->figure_3->xAxis->setLabel("X-axis Mag Field (Tesla)");
    ui->figure_4->yAxis->setLabel("Y-axis Mag Field (Tesla)");
    ui->figure_4->xAxis->setLabel("Probability");
    ui->figure_5->yAxis->setLabel("Z-axis Mag Field (Tesla)");
    ui->figure_5->xAxis->setLabel("Probability");


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
    QVector<double> freqBinsX(100),freqBinsY(100),freqBinsZ(100), xVals(100), yVals(100), zVals(100);
    VectorXd xVect = curInitDataSample.block(0,0,100,1);
    VectorXd yVect = curInitDataSample.block(0,1,100,1);
    VectorXd zVect = curInitDataSample.block(0,2,100,1);
    Vector3d meanVals = curInitDataSample.colwise().mean();
    double xTotal = 0.00;
    double yTotal = 0.00;
    double zTotal = 0.00;
    for(int i = 0; i < 100; i++){
        xTotal += (xVect[i] - meanVals[0]) * (xVect[i] - meanVals[0]);
        yTotal += (yVect[i] - meanVals[1]) * (yVect[i] - meanVals[1]);
        zTotal += (zVect[i] - meanVals[2]) * (zVect[i] - meanVals[2]);
    }
    double stdx = sqrt(1.0/99.0 * xTotal);
    double stdy = sqrt(1.0/99.0 * yTotal);
    double stdz = sqrt(1.0/99.0 * zTotal);
    Vector3d stdVals(stdx, stdy, stdz);
    //sqrt((1/99)*sum((xVect-meanVals(0))))
            //repeat stdVals calculation for all vectors
            //then calculate freqBins from min value in xVect to max value in xVect evenly spaced, and calculate gaussian probability from pdf with mean and stddev
            //then use memcpy to convert to QVector and then plot
            //then repeat for yVect and zVect

    double deltaX = stdx/25.0, deltaY = stdy/25.0, deltaZ = stdz/25.0;
    freqBinsX[0] = meanVals(0) - 2.0*stdx;
    freqBinsY[0] = meanVals(1) - 2.0*stdy;
    freqBinsZ[0] = meanVals(2) - 2.0*stdz;
    xVals[0] = gaussianFunction(stdx,meanVals[0],freqBinsX[0]);
    yVals[0] = gaussianFunction(stdy,meanVals[1],freqBinsY[0]);
    zVals[0] = gaussianFunction(stdz,meanVals[2],freqBinsZ[0]);
    for(int i = 1; i < 100; i++){
        freqBinsX[i] = freqBinsX[i-1] + deltaX;
        freqBinsY[i] = freqBinsY[i-1] + deltaY;
        freqBinsZ[i] = freqBinsZ[i-1] + deltaZ;
        xVals[i] = gaussianFunction(stdx,meanVals[0],freqBinsX[i]);
        yVals[i] = gaussianFunction(stdy,meanVals[1],freqBinsY[i]);
        zVals[i] = gaussianFunction(stdz,meanVals[2],freqBinsZ[i]);
    }
    double maxX = xVals[0], maxY = yVals[0], maxZ= zVals[0];
    for(int i = 0, i<100, i++){
        if(maxX < xVals[i])
            maxX = xVals[i];
        if(maxY < yVals[i])
            maxY = yVals[i];
        if(maxZ < zVals[i])
            maxZ = zVals[i];
    }

    ui->figure_3->xAxis->setRange(freqBinsX[0],freqBinsX[99]);
    ui->figure_3->yAxis->setRange(0,maxX);
    ui->figure_3->graph(0)->setData(freqBinsX,xVals);
    ui->figure_3->xAxis->setticklabelrotation(80);
    ui->figure_3->replot();

    ui->figure_4->xAxis->setRange(freqBinsY[0],freqBinsY[99]);
    ui->figure_4->yAxis->setRange(0,maxY);
    ui->figure_4->graph(0)->setData(freqBinsY,yVals);
    ui->figure_4->xAxis->setticklabelrotation(80);
    ui->figure_4->replot();

    ui->figure_5->xAxis->setRange(freqBinsZ[0],freqBinsZ[99]);
    ui->figure_5->yAxis->setRange(0,maxZ);
    ui->figure_5->graph(0)->setData(freqBinsZ,zVals);
    ui->figure_5->xAxis->setticklabelrotation(80);
    ui->figure_5->replot();
}

double gaussianFunction(double std, double mean, double x){
    double coeff = 1.0/(sqrt(6.28318530718)*std);
    double power = -1.0*(pow((x-mean),2)/(2.0*std*std));
    double fg = coeff*exp(power);
    return fg;
}
