#ifndef LOPATAOBJECT_H
#define LOPATAOBJECT_H

#include "MathForIMU.h"
#include <opencv2/core/types.hpp>

/**
 * \brief Struct which accumulate all data about Lopata
 */
struct Lopata
{
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
	const double _distBetweenDiodes = 25.;
	/**
	* \brief Distance between diodes on the image fromcamera in pixels
	*/
	double _realPixelDistanceBetweenDiodes;
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
	void ñoordinatesIntoThreeDimensional();

	/**
	* \brief Scaling coordinates depending on the height
	*/
	void scalingCoordinates();
};
#endif // LOPATAOBJECT_H
