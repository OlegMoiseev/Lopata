#ifndef LOPATAOBJECT_H
#define LOPATAOBJECT_H

#include "MathForIMU.h"
#include <opencv2/core/types.hpp>

struct Lopata
{
	unsigned int _centerXCoordinatesOfLopata;
	unsigned int _centerYCoordinatesOfLopata;
	double _altitude = 0.;
	Quaternion _quaternionOfLopataRotation;
	const double _distBetweenDiodes = 25.;
	double _realPixelDistanceBetweenDiodes;
	const double _radius = 22;
	Vector _axis;
	std::array<double, 3> _eulerDegrees{{0., 0., 0.}};
	std::array<double, 3> _oldEulerDegrees{{0., 0., 0.}};

	std::vector<cv::KeyPoint> _resultKeypointsOfDetectedDiodes;


	std::array<int, 3> _cartesianCoordinates;
	std::array<int, 3> _oldCartesianCoordinates{{0, 0, 0}};

};
#endif // LOPATAOBJECT_H
