#ifndef POSCALCULATOR_H
#define POSCALCULATOR_H

#include <Magnet.h>
#include <Sensor.h>
#include "Magnet.h"
#include <stdio.h>
#include <windows.h>
#include <math.h>
#include <fstream>
#include <optimization.h>
#include <string.h>
#include <sstream>
#include <stdlib.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
class PosCalculator
{
public:
    //should likely break up these functions within the mainfn into things like calibrate(), acquireCoVar(), beginTracking(), etc...
    //and then make all other functions private
    int mainfn();
    string getPortNumber();
    HANDLE getHSerial();
    bool getConnected();
    COMSTAT getStatus();
    Magnet getM1();
    PosCalculator();
    //a
    PosCalculator(string, HANDLE, bool, COMSTAT, Magnet);
private:
    string portNumber;
    HANDLE hSerial;
    bool connected;
    COMSTAT status;
    Magnet M1;
};


#endif // POSCALCULATOR_H
