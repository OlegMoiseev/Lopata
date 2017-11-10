#ifndef MATH_FOR_IMU
#define MATH_FOR_IMU

#include <array>

struct Lopata;
class Quaternion
{
public:
	float _w;
	float _x;
	float _y;
	float _z;

	explicit Quaternion(float a = 0., float b = 0., float c = 0., float d = 1.) : _w(a), _x(b), _y(c), _z(d)
	{
	}

	void operator=(Quaternion& q);

	Quaternion inverse() const;

	float lengthSqr() const;

	Quaternion invForMult() const;

	friend std::ostream& operator<<(std::ostream& os, const Quaternion& v);
};

class Vector
{
public:
	float _x;
	float _y;
	float _z;

	explicit Vector(float a = 0., float b = 0., float c = 1.) : _x(a), _y(b), _z(c)
	{
	}

	float length() const;

	void normalize();

	void scale(const float coeff);

	friend std::ostream& operator<<(std::ostream& os, const Vector& v);
};

Quaternion quatMultQuat(Quaternion& a, Quaternion& b);

Quaternion quatMultVect(const Quaternion& a, Vector& b);

Vector quatTransformVector(const Quaternion& q, Vector& v);

float vectorDotProduct(Vector& a, Vector& b);

//=======================================================================================================================
// Function of translating quaternion to euler angles
//=======================================================================================================================
void quaternionToEulerianAngle(const Quaternion& q, float& roll, float& pitch, float& yaw);

float degreesToRad(int& deg);

Vector vectMultMat(Vector& v, float m[3][3]);

void craftRotationMat(const float rad, float& x, float& y, float& z, float m[3][3]);

void ñorrectCoordinates(int& roll, Lopata &obj, Vector& axis, const float realPixelDistanceBetweenDiodes);

void calculateCoordinates(Lopata &obj, const float& realPixelDistanceBetweenDiodes);

void scalingCoordinates(Lopata& lopata);

#endif
