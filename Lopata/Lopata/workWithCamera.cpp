#include <thread>

#include "workWithCamera.h"
#include "CamCalibWI.h"
#include "Robots.h"
#include "ImuModule.h"
#include "LopataFinder.h"


void detectControlHandlePosition(const bool& connection)
{
	robot::FanucM20iA robo(connection);

	PololuImuV5 imu(L"COM3");

	cv::VideoCapture webCamera(0);

	timur::CamCalibWi camCalib("CamCalib.txt");

	LopataFinder finder(webCamera, camCalib);
	Lopata object;

	std::thread imuThread(PololuImuV5::readOutImu, &imu, std::ref(object._quaternionOfLopataRotation));
	while (true)
	{
		imu.startReading();

		finder.detectDiodes(object);

		finder.calculateDiodesCoordinates(imu, object);

		object.calculateThirdCoordinate();

		object.ñorrectCoordinates();

		object.scalingCoordinates();

		robo.sendCoordinates(object);

       /* system("cls");
        std::cout << object._localXCoordinatesOfLopata << '\t' << object._localYCoordinatesOfLopata
            << '\t' << object._altitude << '\n';*/
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
	std::cout << "Computer vision! OpenCV ver. " << CV_MAJOR_VERSION << '.' << CV_MINOR_VERSION << '.'
		<<
		CV_SUBMINOR_VERSION << std::endl;
}
