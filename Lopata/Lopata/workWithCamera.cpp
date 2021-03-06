#include "workWithCamera.h"
#include "CameraCalibration.h"
#include "Robots.h"
#include "ImuModule.h"
#include "LopataFinder.h"
#include <thread>

void detectControlHandlePosition(const bool& connection)
{
	robot::FanucM20iA robo(connection);

	PololuImuV5 imu(L"COM3");

	cv::VideoCapture webCamera(1);
	timur::CameraCalibration camCalib(cv::Size(9, 6), 0.026f);

	LopataFinder finder(webCamera, camCalib);
	Lopata object;
	while (true)
	{
		imu.startReading();

		std::thread imuThread(PololuImuV5::readOutImu, &imu, std::ref(object._quaternionOfLopataRotation));

		finder.detectDiodes(object);

		finder.calculateDiodesCoordinates(imu, robo, object, imuThread);

		object.ñoordinatesIntoThreeDimensional();

		object.ñorrectCoordinates();

		//std::cout << object._centerXCoordinatesOfLopata << '\t' << object._centerYCoordinatesOfLopata << '\t' << object._altitude << '\n';

		//object.scalingCoordinates();

		robo.sendCoordinates(object);

		finder.drawKeypoints(object);

		if (cv::waitKey(30) == 27)
		{
			robo.sendany("985000 0 940000 -180000 0 0 10 2 1");
			robo.finish();
			break;
		}
	}
}

void cvVersionOnScreen()
{
	std::cout << "Computer vision! OpenCV ver. " << CV_MAJOR_VERSION << '.' << CV_MINOR_VERSION << '.' <<
		CV_SUBMINOR_VERSION << std::endl;
}
