#include "LopataObject.h"
#include <iostream>

void Lopata::scalingCoordinates()
{
	const double rel = _calibrationPixelDistanceBetweenDiodes / _realDistBetweenDiodes;
	_xBound = 640. * rel;
	_yBound = 480. * rel;

	_centerXCoordinatesOfLopata *= _xBound / 640.;
	_centerYCoordinatesOfLopata *= _yBound / 480.;
}

void Lopata::ñorrectCoordinates()
{
	Vector tmpToCenter(0, 0, 1);
	Vector v2Center = quatTransformVector(_quaternionOfLopataRotation, tmpToCenter);

	double mTmp[3][3];

	system("cls");

	const double roll = _eulerAngles[0];
	double angle = 0.;

	if (-45. < roll && roll <= 45.)
	{
		std::cout << roll << "\tUP" << std::endl;
		angle = _pi;
	}
	else if (45. < roll && roll <= 135.)
	{
		std::cout << roll << "\tRIGHT" << std::endl;
		angle = _morePi;
	}
	else if (135. < roll && roll <= 180.)
	{
		std::cout << roll << "\tDOWN" << std::endl;
		angle = 0.;
	}
	else if (-180. < roll && roll <= -135.)
	{
		std::cout << roll << "\tDOWN" << std::endl;
		angle = 0.;
	}
	else if (-135. < roll && roll <= -45.)
	{
		std::cout << roll << "\tLEFT" << std::endl;
		angle = _halfPi;
	}


	craftRotationMat(angle, _axis._x, _axis._y, _axis._z, mTmp); // create matrix to rotate vector from center to circle
	Vector mVect2 = vectMultMat(v2Center, mTmp); // rotate this vector into necessary quadrant
	mVect2.normalize(); // normalize this vector 

	const double tmpRadius = _radius * _lenghtPixelProjectionBetweenDiodes / _realDistBetweenDiodes;
	mVect2.scale(tmpRadius); // scale translation vector to real pixel size

	// correct our Cartesian coordinates
	_altitude -= mVect2._z;
	_centerXCoordinatesOfLopata += static_cast<int>(mVect2._x);
	_centerYCoordinatesOfLopata += static_cast<int>(mVect2._y);
}

void Lopata::calculateThirdCoordinate()
{
	Vector startVector(1, 0, 0);
	const Vector normalForFlatOfTheCamera(0, 0, 1);

	quaternionToEulerianAngle(_quaternionOfLopataRotation, _eulerAngles[0], _eulerAngles[1], _eulerAngles[2]);

	_axis = quatTransformVector(_quaternionOfLopataRotation, startVector);
	const double t = (normalForFlatOfTheCamera._x * _axis._x + normalForFlatOfTheCamera._y * _axis._y +
		normalForFlatOfTheCamera._z * _axis._z) / normalForFlatOfTheCamera.length();
	Vector projectionVector(_axis._x - t * normalForFlatOfTheCamera._x, _axis._y - t * normalForFlatOfTheCamera._y,
	                        _axis._z - t * normalForFlatOfTheCamera._z);

	const double cosAngle = vectorDotProduct(projectionVector, _axis) / (projectionVector.length() * _axis.length());
	_altitude = _calibrationPixelDistanceBetweenDiodes / _lenghtPixelProjectionBetweenDiodes * cosAngle *
		_calibrationHeigth;
}


void Lopata::allowSend()
{
	sendCoordinates = true;
}

void Lopata::forbidSend()
{
	sendCoordinates = false;
}