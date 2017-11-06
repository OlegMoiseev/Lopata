#ifndef LOPATAOBJECT_H
#define LOPATAOBJECT_H

#include "MathForIMU.h"
#include <opencv2/core/types.hpp>

struct Lopata
{
	unsigned int _centerXCoordinatesOfLopata;
	unsigned int _centerYCoordinatesOfLopata;
	float _altitude = 0.;
	Quaternion _quaternionOfLopataRotation;
	const float _distBetweenDiodes = 25.;
	const float _radius = 22;

	std::array<float, 3> _eulerDegrees{{0., 0., 0.}};
	std::array<float, 3> _oldEulerDegrees{{0., 0., 0.}};

	std::vector<cv::KeyPoint> _resultKeypointsOfDetectedDiodes;


	std::array<int, 3> _cartesianCoordinates;
	std::array<int, 3> _oldCartesianCoordinates{{0, 0, 0}};


	void setCoordinates(unsigned int& x, unsigned int& y, unsigned int& z)
	{
		_centerXCoordinatesOfLopata = x;
		_centerYCoordinatesOfLopata = y;
		_altitude = z;
	}
};
#endif // LOPATAOBJECT_H
