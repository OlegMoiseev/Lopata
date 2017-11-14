#include "LopataObject.h"
#include <iostream>

const double PI = acos(-1);
const double HALF_PI = PI / 2;
const double MORE_PI = PI * 3 / 2;

const int CALIBRATED_PIXEL_DISTANCE_BETWEEN_DIODES = 236;
const int CALIBRATED_HEIGTH = 50;

void Lopata::scalingCoordinates()
{
	const auto relation = _altitude / CALIBRATED_HEIGTH;
	_centerXCoordinatesOfLopata *= relation;
	_centerYCoordinatesOfLopata *= relation;
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
		angle = PI;
	}
	else
	{
		if (45. < roll && roll <= 135.)
		{
			std::cout << roll << "\tRIGHT" << std::endl;
			angle = MORE_PI;
		}
		else
		{
			if (135. < roll && roll <= 180.)
			{
				std::cout << roll << "\tDOWN" << std::endl;
				angle = 0.;
			}
			else
			{
				if (-180. < roll && roll <= -135.)
				{
					std::cout << roll << "\tDOWN" << std::endl;
					angle = 0.;
				}
				else
				{
					if (-135. < roll && roll <= -45.)
					{
						std::cout << roll << "\tLEFT" << std::endl;
						angle = HALF_PI;
					}
				}
			}
		}
	}

	craftRotationMat(angle, _axis._x, _axis._y, _axis._z, mTmp); // create matrix to rotate vector from center to circle
	Vector mVect2 = vectMultMat(v2Center, mTmp); // rotate this vector into necessary quadrant
	mVect2.normalize(); // normalize this vector 

	const double tmpRadius = _radius*_realPixelDistanceBetweenDiodes / _distBetweenDiodes;
	mVect2.scale(tmpRadius); // scale translation vector to real pixel size

							 // correct our Cartesian coordinates
	_altitude -= mVect2._z;
	_centerXCoordinatesOfLopata += mVect2._x;
	_centerYCoordinatesOfLopata += mVect2._y;

}

void Lopata::ñoordinatesIntoThreeDimensional()
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
	_altitude = CALIBRATED_PIXEL_DISTANCE_BETWEEN_DIODES / _realPixelDistanceBetweenDiodes * cosAngle *
		CALIBRATED_HEIGTH;
}
