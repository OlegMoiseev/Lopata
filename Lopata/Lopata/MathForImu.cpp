#include "MathForIMU.h"
#include "LopataObject.h"
#include <iostream>

const float PI = acos(-1);
const float HALF_PI = PI / 2;
const float MORE_PI = PI * 3 / 2;

const int CALIBRATED_PIXEL_DISTANCE_BETWEEN_DIODES = 236;
const int CALIBRATED_HEIGTH = 50;
void Quaternion::operator=(Quaternion& q)
{
	_w = q._w;
	_x = q._x;
	_y = q._y;
	_z = q._z;
}

Quaternion Quaternion::inverse() const
{
	return Quaternion(_w, -_x, -_y, -_z);
}

float Quaternion::lengthSqr() const
{
	return _w * _w + _x * _x + _y * _y + _z * _z;
}

Quaternion Quaternion::invForMult() const
{
	Quaternion tmp = this->inverse();
	const float tmpL = this->lengthSqr();
	tmp._w /= tmpL;
	tmp._x /= tmpL;
	tmp._y /= tmpL;
	tmp._z /= tmpL;
	return tmp;
}

inline std::ostream& operator<<(std::ostream& os, const Quaternion& q)
{
//os << "w: " << q.w << "\tx: " << q.x << "\ty: " << q.y << "\tz: " << q.z;
os << q._w << "\t" << q._x << "\t" << q._y << "\t" << q._z;
return os;
}


float Vector::length() const
{
	return std::pow(_x * _x + _y * _y + _z * _z, 0.5);
}

void Vector::normalize()
{
	const float tmp = this->length();
	_x /= tmp;
	_y /= tmp;
	_z /= tmp;
}

void Vector::scale(const float coeff)
{
	_x *= coeff;
	_y *= coeff;
	_z *= coeff;
}

inline std::ostream& operator<<(std::ostream& os, const Vector& v)
{
	os << "x: " << v._x << "\ty: " << v._y << "\tz: " << v._z;
	return os;
}


Quaternion quatMultQuat(Quaternion& a, Quaternion& b)
{
	Quaternion res;
	res._w = a._w * b._w - a._x * b._x - a._y * b._y - a._z * b._z;
	res._x = a._w * b._x + a._x * b._w + a._y * b._z - a._z * b._y;
	res._y = a._w * b._y - a._x * b._z + a._y * b._w + a._z * b._x;
	res._z = a._w * b._z + a._x * b._y - a._y * b._x + a._z * b._w;
	return res;
}

Quaternion quatMultVect(const Quaternion& a, Vector& b)
{
	Quaternion res;
	res._w = -a._x * b._x - a._y * b._y - a._z * b._z;
	res._x = a._w * b._x + a._y * b._z - a._z * b._y;
	res._y = a._w * b._y - a._x * b._z + a._z * b._x;
	res._z = a._w * b._z + a._x * b._y - a._y * b._x;
	return res;
}

Vector quatTransformVector(const Quaternion& q, Vector& v)
{
	Quaternion t = quatMultVect(q, v);
	const Quaternion n = quatMultQuat(t, q.inverse());

	Vector qVector;
	qVector._x = n._x;
	qVector._y = n._y;
	qVector._z = n._z;
	qVector.normalize();
	return qVector;
}

float vectorDotProduct(Vector& a, Vector& b)
{
	return a._x * b._x + a._y * b._y + a._z * b._z;
}

void quaternionToEulerianAngle(const Quaternion& q, float& roll, float& pitch, float& yaw)
{
	const float ysqr = q._y * q._y;

	// roll (x-axis rotation)
	const float t0 = +2.0 * (q._w * q._x + q._y * q._z);
	const float t1 = +1.0 - 2.0 * (q._x * q._x + ysqr);
	roll = std::atan2(t0, t1) / 3.1415 * 180.0;

	// pitch (y-axis rotation)
	float t2 = +2.0 * (q._w * q._y - q._z * q._x);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	pitch = std::asin(t2) / 3.1415 * 180.0;

	// yaw (z-axis rotation)
	const float t3 = +2.0 * (q._w * q._z + q._x * q._y);
	const float t4 = +1.0 - 2.0 * (ysqr + q._z * q._z);
	yaw = std::atan2(t3, t4) / 3.1415 * 180.0;
}

float degreesToRad(int& deg)
{
	return deg * PI / 180.;
}

Vector vectMultMat(Vector& v, float m[3][3])
{
	Vector tmp;
	tmp._x = v._x * m[0][0] + v._y * m[1][0] + v._z * m[2][0];
	tmp._y = v._x * m[0][1] + v._y * m[1][1] + v._z * m[2][1];
	tmp._z = v._x * m[0][2] + v._y * m[1][2] + v._z * m[2][2];
	return tmp;
}

void craftRotationMat(const float rad, float& x, float& y, float& z, float m[3][3])
{
	m[0][0] = cos(rad) + (1 - cos(rad)) * x * x;
	m[0][1] = (1 - cos(rad)) * x * y - sin(rad) * z;
	m[0][2] = (1 - cos(rad)) * x * z + sin(rad) * y;

	m[1][0] = (1 - cos(rad)) * y * x + sin(rad) * z;
	m[1][1] = cos(rad) + (1 - cos(rad)) * y * y;
	m[1][2] = (1 - cos(rad)) * y * z - sin(rad) * x;

	m[2][0] = (1 - cos(rad)) * z * x - sin(rad) * y;
	m[2][1] = (1 - cos(rad)) * z * y + sin(rad) * x;
	m[2][2] = cos(rad) + (1 - cos(rad)) * z * z;
}

void scalingCoordinates(Lopata& lopata)
{
	const auto relation = lopata._altitude / CALIBRATED_HEIGTH;
	lopata._centerXCoordinatesOfLopata *= relation;
	lopata._centerYCoordinatesOfLopata *= relation;
}

void ñorrectCoordinates(int& roll, Lopata &obj, Vector& axis, const float realPixelDistanceBetweenDiodes)
{
	
	Vector tmpToCenter(0, 0, 1);
	Vector v2Center = quatTransformVector(obj._quaternionOfLopataRotation, tmpToCenter);

	float mTmp[3][3];

	system("cls");

	double angle = 0;
	if (-45 < roll && roll <= 45)
	{
		std::cout << roll << "\tUP" << std::endl;
		angle = PI;
	}
	else
	{
		if (45 < roll && roll <= 135)
		{
			std::cout << roll << "\tRIGHT" << std::endl;
			angle = MORE_PI;
		}
		else
		{
			if (135 < roll && roll <= 180)
			{
				std::cout << roll << "\tDOWN" << std::endl;
				angle = 0;
			}
			else
			{
				if (-180 < roll && roll <= -135)
				{
					std::cout << roll << "\tDOWN" << std::endl;
					angle = 0;
				}
				else
				{
					if (-135 < roll && roll <= -45)
					{
						std::cout << roll << "\tLEFT" << std::endl;
						angle = HALF_PI;
					}
				}
			}
		}
	}

	craftRotationMat(angle, axis._x, axis._y, axis._z, mTmp); // create matrix to rotate vector from center to circle
	Vector mVect2 = vectMultMat(v2Center, mTmp); // rotate this vector into necessary quadrant
	mVect2.normalize(); // normalize this vector 

	const int tmpRadius = obj._radius*realPixelDistanceBetweenDiodes / obj._distBetweenDiodes;
	mVect2.scale(tmpRadius); // scale translation vector to real pixel size

	// correct our Cartesian coordinates
	obj._altitude -= mVect2._z;
	obj._centerXCoordinatesOfLopata += mVect2._x;
	obj._centerYCoordinatesOfLopata += mVect2._y;

	scalingCoordinates(obj);

}

void calculateCoordinates(Lopata &obj, const float& realPixelDistanceBetweenDiodes)
{
	Vector startVector(1, 0, 0);
	const Vector normalForFlatOfTheCamera(0, 0, 1);

	quaternionToEulerianAngle(obj._quaternionOfLopataRotation, obj._eulerDegrees[0], obj._eulerDegrees[1], obj._eulerDegrees[2]);
	
	Vector vTmp = quatTransformVector(obj._quaternionOfLopataRotation, startVector);
	const float t = (normalForFlatOfTheCamera._x * vTmp._x + normalForFlatOfTheCamera._y * vTmp._y +
		normalForFlatOfTheCamera._z * vTmp._z) / normalForFlatOfTheCamera.length();
	Vector projectionVector(vTmp._x - t * normalForFlatOfTheCamera._x, vTmp._y - t * normalForFlatOfTheCamera._y,
		vTmp._z - t * normalForFlatOfTheCamera._z);

	const float cosAngle = vectorDotProduct(projectionVector, vTmp) / (projectionVector.length() * vTmp.length());
	obj._altitude = CALIBRATED_PIXEL_DISTANCE_BETWEEN_DIODES / realPixelDistanceBetweenDiodes * cosAngle *
		CALIBRATED_HEIGTH;

	int roll = obj._eulerDegrees[0];
	ñorrectCoordinates(roll, obj, vTmp, realPixelDistanceBetweenDiodes);
}

