#include "mainwindow.h"
#include <QApplication>
#include <PosCalculator.h>
#include <Magnet.h>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    PosCalculator driver = new PosCalculator();
    //depending on button presses: calibrate -> calibrateSystem
    //Read Noise -> gatherSampleCovarData()
    //StartTracking -> findFirstLocation() -> loop that contains startTracking()
    //within that loop write location at the end of each iteration update Plot function
    //have a way to load in data from text file (different calibrate)"load noise data"
    //find first location needs ot be able to check if the covariance has already been uploaded to sensors
    //before we find the first location we have to make sure that data is already in the sensors

    return a.exec();
}
