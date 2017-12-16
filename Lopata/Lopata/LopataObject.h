#ifndef LOPATAOBJECT_H
#define LOPATAOBJECT_H

#include <array>
#include <map>

#include <opencv2/core/types.hpp>

#include "MathForIMU.h"


/**
 * \brief Struct which accumulate all data about Lopata
 */
struct Lopata
{

	const double _pi = acos(-1);
	const double _halfPi = _pi / 2;
	const double _morePi = _pi * 3 / 2;

	// variables to correct draw on the image - save non-scaled coordinates
	double xCenterImg;
	double yCenterImg;

	double _xBound;
	double _yBound;

	const std::map<double, double> calibrateTable
	{
		{253.16, 90},
		{229.14, 100},
		{208.118, 110},
		{190.095, 120},
		{176.102, 130},
		{163.11, 140},
		{153.16, 150},
		{142.088, 160},
		{134.093, 170},
		{127.098, 180},
		{120.067, 190},
		{114.07, 200},
		{109.073, 210},
		{104.043, 220},
		{99.0454, 230},
		{95.0474, 240},
		{92.0489, 250},
		{88.0511, 260},
		{85.0529, 270},
		{82.0549, 280},
		{79.0253, 290},
		{77.0584, 300},
		{74.027, 310},
		{71.0282, 320},
		{70.0643, 330},
		{67.0298, 340},
		{65.0692, 350},
		{64.0312, 360},
		{62.0322, 370},
		{60.0333, 380},
		{59.0339, 390},
		{57.0351, 400},
		{56.0357, 410},
		{54.0093, 420},
		{53.0377, 430},
		{52.0384, 440},
		{50.01,	460},
		{49.0408, 470},
		{48.0104, 480},
		{47.0106, 490},
		{46.0109, 500},
		{45.0444, 510},
		{44.0114, 520},
		{43.0116, 540},
		{42.0119, 550},
		{40.0125, 570},
		{39.0128, 590},
		{38.0132, 600}
	};

	/**
	* \brief Flag of allowing send coordinates to the robot
	*/
	bool sendCoordinates;

	/**
	 * \brief X coordinate of Lopata
	 */
	double _centerXCoordinatesOfLopata;
	/**
	* \brief Y coordinate of Lopata
	*/
	double _centerYCoordinatesOfLopata;
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

	/**
	* \brief Allowing send data to the robot
	*/
	void allowSend();

	/**
	* \brief Forbidding send data to the robot
	*/
	void forbidSend();

	double calculatePointOnGraph(const double& hyp) const;
};
#endif // LOPATAOBJECT_H
