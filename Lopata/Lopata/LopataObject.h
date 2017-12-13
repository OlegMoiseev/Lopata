#ifndef LOPATAOBJECT_H
#define LOPATAOBJECT_H

#include "MathForIMU.h"
#include <opencv2/core/types.hpp>
#include <array>

/**
 * \brief Struct which accumulate all data about Lopata
 */
struct Lopata
{
	const int _calibrationPixelDistanceBetweenDiodes = 236;
	const int _calibrationHeigth = 50;

	const double _pi = acos(-1);
	const double _halfPi = _pi / 2;
	const double _morePi = _pi * 3 / 2;

	double _xBound;
	double _yBound;

	/**
	 * \brief X coordinate of Lopata
	 */
	unsigned int _centerXCoordinatesOfLopata;
	/**
	* \brief Y coordinate of Lopata
	*/
	unsigned int _centerYCoordinatesOfLopata;
	/**
	* \brief Z coordinate of Lopata
	*/
	double _altitude = 0.;
	/**
	* \brief Quaternion which predetermine position of Lopata
	*/
	Quaternion _quaternionOfLopataRotation;
	/**
	 * \brief Distance between diodes on the Lopata in millimeters
	 */
	const double _realDistBetweenDiodes = 25.;
	/**
	* \brief Distance between diodes on the image from camera in pixels
	*/
	double _lenghtPixelProjectionBetweenDiodes;
	/**
	 * \brief Radius of the cylindrical part of the Lopata
	 */
	const double _radius = 22;
	/**
	 * \brief Axis to rotate translation vector
	 */
	Vector _axis;
	/**
	 * \brief Euler angles from quaternion of the rotation
	 */
	std::array<double, 3> _eulerAngles{{0., 0., 0.}};
	/**
	* \brief Euler angles from previous iteration of the algotithm
	*/
	std::array<double, 3> _oldEulerAngles{{0., 0., 0.}};

	/**
	 * \brief Keypoints on the image
	 */
	std::vector<cv::KeyPoint> _resultKeypointsOfDetectedDiodes;

	/**
	 * \brief Cartesian coordinates of the Lopata
	 */
	std::array<int, 3> _cartesianCoordinates;
	/**
	* \brief Cartesian coordinates from the previous iteration of the algorithm
	*/
	std::array<int, 3> _oldCartesianCoordinates{{0, 0, 0}};
	/**
	* \brief Correcting coordinates depending on the position of Lopata
	*/
	void ñorrectCoordinates();

	/**
	* \brief Calculating height of the Lopata
	*/
	void calculateThirdCoordinate();

	/**
	* \brief Scaling coordinates depending on the height
	*/
	void scalingCoordinates();
};
#endif // LOPATAOBJECT_H
