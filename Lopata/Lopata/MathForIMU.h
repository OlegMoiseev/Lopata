#ifndef MATH_FOR_IMU
#define MATH_FOR_IMU

#include <array>

struct Lopata;

class Quaternion
{
public:
	double _w;
	double _x;
	double _y;
	double _z;

	explicit Quaternion(double a = 0., double b = 0., double c = 0., double d = 1.) : _w(a), _x(b), _y(c), _z(d)
	{
	}

	void operator=(Quaternion& q);

	Quaternion inverse() const;

	double lengthSqr() const;

	Quaternion invForMult() const;

	friend std::ostream& operator<<(std::ostream& os, const Quaternion& v);
};

class Vector
{
public:
	double _x;
	double _y;
	double _z;

	explicit Vector(const double a = 0., const double b = 0., const double c = 1.)
		: _x(a),
		  _y(b),
		  _z(c)
	{
	}

	double length() const;

	void normalize();

	void scale(const double coeff);

	friend std::ostream& operator<<(std::ostream& os, const Vector& v);
};

Quaternion quatMultQuat(Quaternion& a, Quaternion& b);

Quaternion quatMultVect(const Quaternion& a, Vector& b);

Vector quatTransformVector(const Quaternion& q, Vector& v);

double vectorDotProduct(Vector& a, Vector& b);

void quaternionToEulerianAngle(const Quaternion& q, double& roll, double& pitch, double& yaw);

double degreesToRad(int& deg);

Vector vectMultMat(Vector& v, double m[3][3]);

void craftRotationMat(const double rad, double& x, double& y, double& z, double m[3][3]);

/**
 * \brief Correcting coordinates depending on the position of Lopata
 * \param obj Lopata, the coordinates of which are adjusted
 */
void ñorrectCoordinates(Lopata& obj);

/**
 * \brief Calculating height of the Lopata
 * \param obj Lopata, the coordinates of which are transformed
 */
void ñoordinatesIntoThreeDimensional(Lopata& obj);

/**
 * \brief Scaling coordinates depending on the height
 * \param lopata Lopata, the coordinates of which are scaled
 */
void scalingCoordinates(Lopata& lopata);

#endif
