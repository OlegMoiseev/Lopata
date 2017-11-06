#include "Robots.h"
#include "Globals.h"

void robot::FanucM20iA::createRealCoords(int& xToRobot, int& yToRobot, int& zToRobot,
	unsigned int& centerXCoordinatesOfLopata, unsigned int& centerYCoordinatesOfLopata,
	float& altitude)
{
	xToRobot = maxValueOf._x - centerYCoordinatesOfLopata * mult._x;
	yToRobot = minValueOf._y + centerXCoordinatesOfLopata * mult._y;
	zToRobot = minValueOf._z + static_cast<int>(altitude) * mult._z;

	robot::FanucM20iA::checkCoordsLimits(xToRobot, yToRobot, zToRobot);
}

void robot::FanucM20iA::checkCoordsLimits(int& xToRobot, int& yToRobot, int& zToRobot)
{
	if (xToRobot > maxValueOf._x) xToRobot = maxValueOf._x;
	if (xToRobot < minValueOf._x) xToRobot = minValueOf._x;

	if (yToRobot > maxValueOf._y) yToRobot = maxValueOf._y;
	if (yToRobot < minValueOf._y) yToRobot = minValueOf._y;

	if (zToRobot > maxValueOf._z) zToRobot = maxValueOf._z;
	if (zToRobot < minValueOf._z) zToRobot = minValueOf._z;
}
