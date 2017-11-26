#ifndef MATH_FOR_IMU
#define MATH_FOR_IMU
#include <ostream>

//TODO: move fields to private


class Quaternion
{
public:
	/**
	 * \brief Real part of quaternion
	 */
	double _w;
	/**
	 * \brief First imagine part of quaternion
	 */
	double _x;
	/**
	* \brief Second imagine part of quaternion
	*/
	double _y;
	/**
	* \brief Third imagine part of quaternion
	*/
	double _z;

	/**
	 * \brief Constructor of the quaternion
	 * \param[in] a Real part of quaternion
	 * \param[in] b First imagine part of quaternion
	 * \param[in] c Second imagine part of quaternion
	 * \param[in] d Third imagine part of quaternion
	 */
	explicit Quaternion(double a = 0., double b = 0., double c = 0., double d = 1.);

	/**
	 * \brief The equality operator for quaternions
	 * \param[in] q Quaternion
	 */
	void operator=(Quaternion& q);

	/**
	 * \brief Quaternion inversion function
	 * \return Inverted quaternion
	 */
	Quaternion inverse() const;

	/**
	 * \brief Function of calculating length in square of the quaternion
	 * \return Length in square of the quaternion
	 */
	double lengthSqr() const;

	/**
	 * \brief Function of inversing quaternion for further multiplication
	 * \return Inversed and normalized quaternion
	 */
	Quaternion invForMult() const;

	/**
	 * \brief Operator of printing quaternion
	 * \param[in] os Output stream
	 * \param[in] v Quaternion to printing 
	 * \return Output stream
	 */
	friend std::ostream& operator<<(std::ostream& os, const Quaternion& v);
};

class Vector
{
public:
	/**
	 * \brief First coordinate of vector
	 */
	double _x;
	/**
	* \brief Second coordinate of vector
	*/
	double _y;
	/**
	* \brief Third coordinate of vector
	*/
	double _z;
	/**
	* \brief Constructor of the vector
	* \param[in] a First coordinate of vector
	* \param[in] b Second coordinate of vector
	* \param[in] c Third coordinate of vector
	*/
	explicit Vector(const double a = 0., const double b = 0., const double c = 1.);

	/**
	* \brief Function of calculating length of the vector
	* \return Length of the vector
	*/
	double length() const;

	/**
	 * \brief Normalizing of the vector
	 */
	void normalize();

	/**
	 * \brief Scaling of a vector by a given coefficient
	 * \param[in] coeff Coefficient
	 */
	void scale(const double coeff);
	/**
	* \brief Operator of printing vector
	* \param[in] os Output stream
	* \param[in] v Vector to printing
	* \return Output stream
	*/
	friend std::ostream& operator<<(std::ostream& os, const Vector& v);
};

/**
 * \brief The multiplication function of two quaternions
 * \param[in] a First quaternion
 * \param[in] b Second quaternion
 * \return Composition of quaternions
 */
Quaternion quatMultQuat(Quaternion& a, Quaternion& b);

/**
* \brief The multiplication function of quaternion and vector
* \param[in] a Quaternion
* \param[in] b Vector
* \return Composition of quaternion and vector
*/
Quaternion quatMultVect(const Quaternion& a, Vector& b);

/**
* \brief The transformation function of quaternion and vector
* \param[in] q Quaternion
* \param[in] v Vector
* \return Turned vector
*/
Vector quatTransformVector(const Quaternion& q, Vector& v);

/**
 * \brief The scalar product function of two vectors
 * \param[in] a First vector
 * \param[in] b Second vector
 * \return Scalar of multiplication of vectors
 */
double vectorDotProduct(Vector& a, Vector& b);

/**
 * \brief Function of converting quaternion into three angles
 * \param[in] q Quaternion for converting
 * \param[out] roll Roll from quaternion in degrees
 * \param[out] pitch Pitch from quaternion in degrees
 * \param[out] yaw Yaw from quaternion in degrees
 */
void quaternionToEulerianAngle(const Quaternion& q, double& roll, double& pitch, double& yaw);

/**
 * \brief Function of converting degrees to radians
 * \param[in] deg Degrees to convert
 * \return Radians
 */
double degreesToRad(int& deg);

/**
 * \brief Function of multiplying a vector by a matrix
 * \param[in] v Vector
 * \param[in] m Matrix 3x3
 * \return Transformed vector
 */
Vector vectMultMat(Vector& v, double m[3][3]);

/**
 * \brief Function of creating a rotation matrix about a given axis by a given angle
 * \param[in] rad Angle of turn
 * \param[in] x First axis coordinate
 * \param[in] y Second axis coordinate
 * \param[in] z Third axis coordinate
 * \param[out] m Created matrix
 */
void craftRotationMat(const double rad, double& x, double& y, double& z, double m[3][3]);

#endif
