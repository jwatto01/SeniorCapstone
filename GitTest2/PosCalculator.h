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

    string getPortNumber();
    HANDLE getHSerial();
    bool getConnected();
    COMSTAT getStatus();
    Magnet getM1();
    Sensor* getSensors();
    MatrixXd getSetOfStartPoints();

    PosCalculator();

    int startTracking();
    Vector3d residual(sensor &curSensor);
    void funcVect(const real_1d_array &x, real_1d_array& fi, void* obj);
    void convertToMicroTesla(const Vector3i &rawData, Vector3d &retArr);
    //void meritFunc(const real_1d_array &x, double& fi, void* obj);
    void jacobian(const real_1d_array &x, real_1d_array &fi, real_2d_array &jac, void* obj);
    bool connectArduino(char *portName);
    int readData(char buffer[169]);
    bool writeData(char buffer[1]);
    void updateSensorReadings(char byteBuff[169]);
    void setZeroVals();
    void calibrateSystem();
    void gatherSampleCovarData();
    void storeNoiseData();
    void findFirstLocation();
private:
    string portNumber;
    HANDLE hSerial;
    bool connected;
    COMSTAT status;
    Magnet M1;
    Sensor allsensors[8];
    MatrixXd setOfStartPoints;
    real_1d_array params_result;
    real_1d_array lbound;
    real_1d_array hbound;
    real_1d_array scale;
    minlmreport rep;
    minlmstate state;
};


#endif // POSCALCULATOR_H
