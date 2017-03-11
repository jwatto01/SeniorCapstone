//Sensor.cpp
//we can have multiple sensors and only one magnet.
//each sensor has xyz coordinates along with an orientation with respect
//to the magnet's vector (theta and phi, but phi has zero influence on
//magnetic field strength due to symmetry)

#include <iostream>
#include "Sensor.h"
#include "math.h"
//#include <numeric>
#include "Magnet.h"
#include <fstream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

const int numAverages = 10;
const double permeability_fs = 0.00000125663706;
const double PI = 3.14159265;
//constructor
Sensor::Sensor(){
	numZeroVals = 0;
	nSensor = -1;//default -1, meaning it is unset in application
	nMeasCount = 0;
	nAvgMeasCount = 0;
	position.setZero();
	orientation.setZero();
	avgScaledVal.setZero();
	sumScaledVal.setZero();
	senseVal.setZero();
	scaledVal.setZero();
	offsets.setZero();
	rawData.setZero();
	meanNoise.setZero();
	initialDataSample = MatrixXd::Constant(100,3,0.0);
}
Sensor::Sensor(const Vector3d &sensorPos, const Vector3d &sensorOr, int nS){
	numZeroVals = 0;
	position = sensorPos;
	orientation = sensorOr;
	nSensor = nS;
	nMeasCount = 0;
	nAvgMeasCount = 0;
	avgScaledVal.setZero();
	sumScaledVal.setZero();
	senseVal.setZero();
	scaledVal.setZero();
	offsets.setZero();
	rawData.setZero();
	meanNoise.setZero();
	initialDataSample = MatrixXd::Constant(100,3,0.0);
}
void Sensor::updateOffsets(){
	offsets = meanNoise;
	std::cout << "offsets: " << std::endl << offsets << std::endl;
}

Vector3d Sensor::getReading(){
	return senseVal;
}

Vector3d Sensor::getPosition(){
	return position;
}
//this is what we expect the magnetic field strength to be for a given sensor
Vector3d Sensor::calculateMagField(Magnet &M1){
	double m1dipoleMoment = M1.dipoleMomentVal();
	double thetaMagField, phiMagField;
	Vector3d magPos = M1.posVal();
	Vector3d magDipoleVect(0.0,0.0,0.0);
	double magnitude_r_Vect = 1.0;
	Vector3d magnetOrientation = M1.orientation();
	Vector3d magEvalLoc(0.0,0.0,0.0);
	Vector3d magEvalLocTMP(0.0,0.0,0.0);
	
	Matrix3d rotationMatrixD;
	Matrix3d rotationMatrixC;
	Matrix3d rotationMatrixB;
	Matrix3d rotationMatrix;//total rotation matrix	

	
	//want to calculate B( R_rotate * (r-r0) ) where r0 is magnet position, r is sensor position
	//where we wish to evaluate the mag field at rotated/translated location in global coordinate system
	//R_rotate is the rotation matrix to rotate the entire field by the magnet's orientation angles
	
	//calculate r-r0 (translate according to location of magnet)
	magEvalLocTMP = position - magPos;
	magDipoleVect(2) = m1dipoleMoment*1e-7;
	//use magnet's orientation angles (stored as euclidean angles)
	//to determine the rotation matrix we must apply to the magnetic field vector
	//at the sensor's position
	
	//define 3 rotation matricies to rotate the magnetic field from magnet's orientation to global coordinate system
	//phi angle
	rotationMatrixD << cos(magnetOrientation(0)), sin(magnetOrientation(0)), 0.0,
						-sin(magnetOrientation(0)), cos(magnetOrientation(0)), 0.0,
						0.0,0.0,1.0;
	//theta angle
	rotationMatrixC << 1.0,0.0,0.0,
						0.0,cos(magnetOrientation(1)), sin(magnetOrientation(1)),
						0.0, -sin(magnetOrientation(1)), cos(magnetOrientation(1));
	//psi angle
	rotationMatrixB << cos(magnetOrientation(2)), sin(magnetOrientation(2)), 0.0,
						-sin(magnetOrientation(2)), cos(magnetOrientation(2)), 0.0,
						0.0,0.0,1.0;

	//find rotation matrix
	rotationMatrix = rotationMatrixB * rotationMatrixC * rotationMatrixD;
	//now calculate new magEvalLoc values (rotated evaluation location)
	magEvalLoc =  magEvalLocTMP.transpose()*rotationMatrix;
	//magDipoleVect = rotationMatrix * magDipoleVect;
	//u0=4*pi*10^-7
	magnitude_r_Vect = magEvalLoc.norm();
	//magnitude_r_Vect = position.norm();
	thetaMagField = atan2(sqrt(pow(magEvalLoc(0),2) + pow(magEvalLoc(1),2)), magEvalLoc(2));//arctan(y/x) = theta
	phiMagField = atan2(magEvalLoc(1),sqrt(pow(magEvalLoc(0),2) + pow(magEvalLoc(1),2)));
	
	//report magnetic field values
	Vector3d magField, rhat;
	/*rhat = position/magnitude_r_Vect;
	double tmp = 1/pow(magnitude_r_Vect,3);
	magField = tmp*(3*((magDipoleVect*rhat.transpose()))*rhat - magDipoleVect);
	*/
	double tmp = (m1dipoleMoment*3e-7) / pow(magnitude_r_Vect,3);
	magField << tmp * sin(thetaMagField) * cos(thetaMagField) * cos(phiMagField),//x component
				tmp * sin(thetaMagField) * cos(thetaMagField) * sin(phiMagField),//y component
				tmp * (cos(thetaMagField) * cos(thetaMagField) - (1.0/3.0));//z component
	
	magField = magField.transpose() * rotationMatrix.transpose();//rotate evaluated field to be aligned with magnet orientation
	/*
	for(int i = 0; i<3; i++) magField(i) = (m1dipoleMoment*3e-7) / pow(magnitude_r_Vect,5);
	magField << magField(0) * magEvalLoc(0) * magEvalLoc(2),//x component
				magField(1) * magEvalLoc(1) * magEvalLoc(2),//y component
				magField(2) * (pow(magEvalLoc(2),2) - (1./3.)*pow(magnitude_r_Vect,2));//z component
	*/
	return magField;
}

void Sensor::updateNSensor(int n){
	nSensor = n;
}

int Sensor::getNSensor(){
	return nSensor;
}

void Sensor::updateRawData(const Vector3i &rData){
	rawData = rData;
}

Vector3d Sensor::getOffsets(){
	return offsets;
}

void Sensor::reset(){
    numZeroVals = 0;
    nMeasCount = 0;
    nAvgMeasCount = 0;
    avgScaledVal.setZero();
    sumScaledVal.setZero();
    senseVal.setZero();
    scaledVal.setZero();
    offsets.setZero();
    rawData.setZero();
    meanNoise.setZero();
    initialDataSample = MatrixXd::Constant(100,3,0.0);
}

void Sensor::updateSenseVal(const Vector3d &newVal){
	senseVal = newVal;
	scaledVal = newVal;
	Vector3d zeroVect;
	zeroVect.setZero();
	if(newVal == zeroVect) numZeroVals++;
	else numZeroVals = 0;//reset counter if we get a non-zero value
	if(numZeroVals > 10) std::cout << "Sensor " << nSensor << " seems to be malfunctioning!" << std::endl;
	if(numZeroVals > 100) std::cout << "Yep, Sensor " << nSensor << " is probably not working." << std::endl;
	//mag field components with sensor's unique orientation
	
	//euler-angle rotation for all sensor measurements
	Matrix3d rotationMatrixD;
	Matrix3d rotationMatrixC;
	Matrix3d rotationMatrixB;
	Matrix3d rotationMatrix;//total rotation matrix	
	
	//define 3 rotation matricies to rotate the measurement from sensor's orientation to global coordinate system
	//phi angle
	rotationMatrixD << cos(orientation(0)), sin(orientation(0)), 0.0,
						-sin(orientation(0)), cos(orientation(0)), 0.0,
						0.0,0.0,1.0;
	//theta angle
	rotationMatrixC << 1.0,0.0,0.0,
						0.0,cos(orientation(1)), sin(orientation(1)),
						0.0, -sin(orientation(1)), cos(orientation(1));
	//psi angle
	rotationMatrixB << cos(orientation(2)), sin(orientation(2)), 0.0,
						-sin(orientation(2)), cos(orientation(2)), 0.0,
						0.0,0.0,1.0;

	//find rotation matrix
	rotationMatrix = rotationMatrixB * rotationMatrixC * rotationMatrixD;
	//now calculate newly rotated vector
	scaledVal = scaledVal.transpose()*rotationMatrix;
	//update the sumScaledVal datamember and if we haven't taken 100 avg measurements yet, update the initialDataSample for standard deviation
	sumScaledVal += scaledVal;
	
	nMeasCount++;
	if(!(nMeasCount % numAverages)){
		avgScaledVal = sumScaledVal / (double)numAverages;
		sumScaledVal.setZero();
		if(nAvgMeasCount < 100)
			for(int i = 0; i<3; i++) initialDataSample(nAvgMeasCount,i) = avgScaledVal(i);
		nAvgMeasCount++;
	}
	
	//time to compute the covariance for the sensor!
	if(nAvgMeasCount == 100)
        //todo Store initialDataSample in data file to be read in rather than previous calculation
		sampleCoVar = sampleCoVariance(initialDataSample);
}

Vector3d Sensor::getOrientation(){
	return orientation;
}
int Sensor::getNumAvgMeasCount(){
	return nAvgMeasCount;
}
//calculate the covariance for current sensor
Matrix3d Sensor::sampleCoVariance(const MatrixXd &data){
	/*std::ofstream fout;
	fout.open("C:\\Users\\andre_000\\Desktop\\test.txt", std::ofstream::app);//make last argument ofstream::trunc to only keep one value in there at a time
	for(int i = 0; i<100; i++){
		for(int j = 0; j<3; j++){
			fout << data(i,j) << ",";
		}
		fout << std::endl;
	}
	fout << std::endl;

	fout.close();
	*/	
	meanNoise = data.colwise().mean();
	//calculate covariance from data
	MatrixXd centered = data.rowwise() - data.colwise().mean();
	MatrixXd tempCov = (centered.adjoint() * centered) / 99.0;
	Matrix3d cov = tempCov.block<3,3>(0,0);
	sqrtSampleCoVar = cov.sqrt();
	return cov;
}
Vector3d Sensor::getMeanNoise(){
	return meanNoise;
}
Matrix3d Sensor::getSQRTsampleCoVar(){
	return sqrtSampleCoVar;
}

Matrix3d Sensor::getsampleCoVar(){
	return sampleCoVar;
}

MatrixXd Sensor::getInitialDataSample(){
    return initialDataSample;
}

Vector3d Sensor::getAvgScaledVal(){
	return avgScaledVal;
}

int Sensor::getNumMeasCount(){
	return nMeasCount;
}

Vector3d Sensor::getScaledReading(){
	return scaledVal;
}
void Sensor::updatePosition(const Vector3d &newPos){
	position = newPos;
}

