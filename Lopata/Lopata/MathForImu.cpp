#include "MathForIMU.h"
#include "LopataObject.h"

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

double Quaternion::lengthSqr() const
{
	return _w * _w + _x * _x + _y * _y + _z * _z;
}

Quaternion Quaternion::invForMult() const
{
	Quaternion tmp = this->inverse();
	const double tmpL = this->lengthSqr();
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


double Vector::length() const
{
	return std::pow(_x * _x + _y * _y + _z * _z, 0.5);
}

void Vector::normalize()
{
	const double tmp = this->length();
	_x /= tmp;
	_y /= tmp;
	_z /= tmp;
}

void Vector::scale(const double coeff)
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

double vectorDotProduct(Vector& a, Vector& b)
{
	return a._x * b._x + a._y * b._y + a._z * b._z;
}

void quaternionToEulerianAngle(const Quaternion& q, double& roll, double& pitch, double& yaw)
{
	const double ysqr = q._y * q._y;

	// roll (x-axis rotation)
	const double t0 = +2. * (q._w * q._x + q._y * q._z);
	const double t1 = +1. - 2. * (q._x * q._x + ysqr);
	roll = std::atan2(t0, t1) / 3.1415 * 180.;

	// pitch (y-axis rotation)
	double t2 = +2. * (q._w * q._y - q._z * q._x);
	t2 = t2 > 1. ? 1. : t2;
	t2 = t2 < -1. ? -1. : t2;
	pitch = std::asin(t2) / 3.1415 * 180.;

	// yaw (z-axis rotation)
	const double t3 = +2. * (q._w * q._z + q._x * q._y);
	const double t4 = +1. - 2. * (ysqr + q._z * q._z);
	yaw = std::atan2(t3, t4) / 3.1415 * 180.;
}

double degreesToRad(int& deg)
{
	return deg * acos(-1) / 180.;
}

Vector vectMultMat(Vector& v, double m[3][3])
{
	Vector tmp;
	tmp._x = v._x * m[0][0] + v._y * m[1][0] + v._z * m[2][0];
	tmp._y = v._x * m[0][1] + v._y * m[1][1] + v._z * m[2][1];
	tmp._z = v._x * m[0][2] + v._y * m[1][2] + v._z * m[2][2];
	return tmp;
}

void craftRotationMat(const double rad, double& x, double& y, double& z, double m[3][3])
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


