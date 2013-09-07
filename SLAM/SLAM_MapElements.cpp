#include "SLAM.h"

PointClass::PointClass() {
	X = 0;
	Y = 0;
	Bearing = 0;
	Range = 0;
}

CornerClass::CornerClass() {
	PointIndex = 0;
	X = 0;
	Y = 0;
	Angle = 0;
	Heading = 0;
	Covariance = Eigen::Matrix4f::Identity();
}

LineClass::LineClass() {
	Good = true;
	ClosedPointIndex = 0;
	Theta = 0;
	R = 0;
	Length = 0;
	Covariance = Eigen::Matrix2f::Identity();
}