//magnet.cpp
#include "magnet.h"
#include <cmath>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

const double PI = 3.14159265;
magnet::magnet(){
	//the default dipole moment is below. There is a tolerance spec of 3% on this value for the magnet from K&J
	dipoleMoment = 0.3;//Ampere-square Meters -- small magnet = 0.0738 or 0.738, I forget... (website says 0.238) big magnet = somewhere between 1.6 and 2.0
	position = Vector3d::Constant(0.0);
	Morientation = Vector3d::Constant(0.0);
}

void magnet::updateOrientation(const Vector3d &newOr){
	//orientation is represented in angles, not a unit vector!!
	//we've chosen to represent in Euler's angle notation (phi, theta, psi)
	//See http://mathworld.wolfram.com/EulerAngles.html for reference
	//Vector3d retOr = newOr;
	//retOr(1) = fmod(newOr(1),PI);
	Morientation = newOr;
	
}

Vector3d magnet::orientation(){
	return Morientation;
}

void magnet::updatePosition(const Vector3d &newPos){
	position = newPos;
}

void magnet::updateDipoleMoment(double newDM){
	dipoleMoment = newDM;
}

double magnet::dipoleMomentVal(){
	return dipoleMoment;
}

Vector3d magnet::posVal(){
	return position;
}
