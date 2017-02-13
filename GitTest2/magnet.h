//magnet.h
#ifndef MAGNET_H
#define MAGNET_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::Vector3d;

class magnet
{

public:
	magnet();
	void updatePosition(const Vector3d &newPos);
	void updateDipoleMoment(double newDM);
	void updateOrientation(const Vector3d &newOr);
	double dipoleMomentVal();
	Vector3d posVal();
	Vector3d orientation();
	
private:
	double dipoleMoment;
	Vector3d position;
	Vector3d Morientation;
};


#endif
