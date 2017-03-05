//sensor.h
#ifndef SENSOR_H
#define SENSOR_H

#include "Magnet.h"
#include <initializer_list>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector3i;
class Sensor
{

public:
    Sensor();
    Sensor(const Vector3d &sensorPos, const Vector3d &sensorOr, int nS);
	void updateSenseVal(const Vector3d &newVal);
	void updatePosition(const Vector3d &newPos);
	void updateOffsets();
	void updateRawData(const Vector3i &rD);
	void updateNSensor(int n);
    void reset();
	int getNSensor();
	Vector3d getOffsets();
	Vector3d getScaledReading();
    Vector3d calculateMagField(Magnet &M1);
	Vector3d getReading();
	Vector3d getPosition();
	Vector3d getOrientation();
	Vector3d getAvgScaledVal();
	Matrix3d getsampleCoVar();
	Matrix3d getSQRTsampleCoVar();
    MatrixXd getInitialDataSample();
	int getNumMeasCount();
	int getNumAvgMeasCount();
	Vector3d getMeanNoise();
	
private:
	int numZeroVals;//counter for number of consecutive sensor data = 0 (used to determine if sensor is functioning)
	Matrix3d sampleCoVariance(const MatrixXd &data);
	int nSensor;//sensor number
	int nMeasCount, nAvgMeasCount;
	Vector3i rawData;
    Vector3d meanNoise; //want to store
	Vector3d orientation;//stored as angles with respect to coordinate z axis
	Vector3d senseVal;
	Vector3d scaledVal;
	MatrixXd initialDataSample;//used to store data to calculate std dev for each sensor axis 
	Vector3d sumScaledVal;//used to sum measurements for averaging
	Vector3d avgScaledVal;//averaged/filtered measurements
	Vector3d position;
	Vector3d offsets;
    Matrix3d sampleCoVar;//sample covariance from initial 100 measurements //want to store
    Matrix3d sqrtSampleCoVar;//matrix sqrt of covariance estimate (used for jacobian function) //want to store
};

#endif
