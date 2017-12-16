#include "LopataFinder.h"


void LopataFinder::frameSizeDetermination()
{
	_rows = _webCamera.get(cv::CAP_PROP_FRAME_HEIGHT);
	_cols = _webCamera.get(cv::CAP_PROP_FRAME_WIDTH);

	std::cout << _cols << "x" << _rows << std::endl;
}

void LopataFinder::detectorParamsSetup()
{
	_parametersForPrimaryDetectorDark.filterByColor = true;
	_parametersForPrimaryDetectorDark.blobColor = 0;

	// Change thresholds
	_parametersForPrimaryDetectorDark.minThreshold = 100.f;
	_parametersForPrimaryDetectorDark.maxThreshold = 255.f;
	//parametersForPrimaryDetectorDark.thresholdStep = 1;

	// Filter by Area.
	_parametersForPrimaryDetectorDark.filterByArea = true;
	_parametersForPrimaryDetectorDark.minArea = 36.f;

	// Filter by Circularity
	_parametersForPrimaryDetectorDark.filterByCircularity = true;
	_parametersForPrimaryDetectorDark.minCircularity = 0.7f;

	// Filter by Convexity
	_parametersForPrimaryDetectorDark.filterByConvexity = true;
	_parametersForPrimaryDetectorDark.minConvexity = 0.4f;

	// Filter by Inertia
	_parametersForPrimaryDetectorDark.filterByInertia = true;
	_parametersForPrimaryDetectorDark.minInertiaRatio = 0.3f;
	//================================================================================
	_parametersForPrimaryDetectorBright.filterByColor = true;
	_parametersForPrimaryDetectorBright.blobColor = 255;

	// Change thresholds
	_parametersForPrimaryDetectorBright.minThreshold = 100.f;
	_parametersForPrimaryDetectorBright.maxThreshold = 255.f;
	//parametersForPrimaryDetectorBright.thresholdStep = 1;

	// Filter by Area.
	_parametersForPrimaryDetectorBright.filterByArea = true;
	_parametersForPrimaryDetectorBright.minArea = 36.f;

	// Filter by Circularity
	_parametersForPrimaryDetectorBright.filterByCircularity = true;
	_parametersForPrimaryDetectorBright.minCircularity = 0.7f;

	// Filter by Convexity
	_parametersForPrimaryDetectorBright.filterByConvexity = true;
	_parametersForPrimaryDetectorBright.minConvexity = 0.4f;

	// Filter by Inertia
	_parametersForPrimaryDetectorBright.filterByInertia = true;
	_parametersForPrimaryDetectorBright.minInertiaRatio = 0.3f;
}

float LopataFinder::getDistanceBetweenPoints(const unsigned& firstX, const unsigned& secondX,
                                             const unsigned& firstY, const unsigned& secondY) const
{
	return static_cast<float>(std::pow((firstX - secondX) * (firstX - secondX)
	                                   + (firstY - secondY) * (firstY - secondY), 0.5));
}

void LopataFinder::getImageFromCamera()
{
	_webCamera >> _rawImageFromCamera;
	_rawImageFromCamera = _camCalib.undistort(_rawImageFromCamera);
}

void LopataFinder::detectKeypoints(cv::Ptr<cv::SimpleBlobDetector> primaryBlobDetector,
                                   const cv::Mat channelsOfHsvImage,
                                   std::vector<cv::KeyPoint> keypointsForEachChannel,
                                   cv::Mat& matrixOfKeypointsForOneChannel)
{
	primaryBlobDetector->detect(channelsOfHsvImage, keypointsForEachChannel);
	for (int i = 0; i < keypointsForEachChannel.size(); ++i)
	{
		cv::circle(matrixOfKeypointsForOneChannel,
		           cv::Point(static_cast<int>(keypointsForEachChannel[i].pt.x),
		                     static_cast<int>(keypointsForEachChannel[i].pt.y)),
		           static_cast<int>(keypointsForEachChannel[i].size / 2), cv::Scalar(255), -1);
	}
}

LopataFinder::LopataFinder(cv::VideoCapture& cam, timur::CamCalibWi& calib)
	: _webCamera(cam),
	  _camCalib(calib)
{
	if (!_webCamera.isOpened())
	{
		std::cout << "I can't open the camera!" << std::endl;
	}
	std::cout << "Camera opened successfully!" << std::endl;

	frameSizeDetermination();

	detectorParamsSetup();
	_primaryBlobDetectorDark = cv::SimpleBlobDetector::create(_parametersForPrimaryDetectorDark);
	_blobDetectorBright = cv::SimpleBlobDetector::create(_parametersForPrimaryDetectorBright);
}

void LopataFinder::firstOccurrenceFalse()
{
	_firstOccurrenceOfTwoPoints = false;
}

void LopataFinder::detectDiodes(Lopata& obj)
{
	std::array<cv::Mat, 3> matrixOfKeypointsForOneChannel = {
		cv::Mat::zeros(_rows, _cols, CV_8UC1),
		cv::Mat::zeros(_rows, _cols, CV_8UC1),
		cv::Mat::zeros(_rows, _cols, CV_8UC1)
	};
	cv::Mat matrixOfSumKeypointsFromAllChannels(_rows, _cols, CV_8UC1, cv::Scalar(0, 0, 0));

	getImageFromCamera();

	cv::cvtColor(_rawImageFromCamera, _hsvMatrixOfRawImage, CV_BGR2HSV);
	// convert camera image format from BGR to HSV
	split(_hsvMatrixOfRawImage, _channelsOfHsvImage);
	cv::threshold(_channelsOfHsvImage[2], _channelsOfHsvImage[2], 250, 255, CV_THRESH_BINARY);

	//----------------------------------------------------------------
	// Start threads with OpenCV
	std::thread t1Thread(detectKeypoints, _primaryBlobDetectorDark, _channelsOfHsvImage[0],
	                     _keypointsForEachChannel[0],
	                     std::ref(matrixOfKeypointsForOneChannel[0]));
	std::thread t2Thread(detectKeypoints, _primaryBlobDetectorDark, _channelsOfHsvImage[1],
	                     _keypointsForEachChannel[1],
	                     std::ref(matrixOfKeypointsForOneChannel[1]));
	std::thread t3Thread(detectKeypoints, _blobDetectorBright, _channelsOfHsvImage[2],
	                     _keypointsForEachChannel[2],
	                     std::ref(matrixOfKeypointsForOneChannel[2]));

	// End threads with OpenCV
	t1Thread.join();
	t2Thread.join();
	t3Thread.join();
	//----------------------------------------------------------------

	for (int i = 0; i < _rows; ++i)
	{
		for (int j = 0; j < _cols; ++j)
		{
			matrixOfSumKeypointsFromAllChannels.at<uchar>(i, j) =
					matrixOfKeypointsForOneChannel[0].at<uchar>(i, j) / 3 +
					matrixOfKeypointsForOneChannel[1].at<uchar>(i, j) / 3 + matrixOfKeypointsForOneChannel[2].at<
						uchar>(i, j) / 3;
		}
	}
	cv::threshold(matrixOfSumKeypointsFromAllChannels, matrixOfSumKeypointsFromAllChannels, 169, 255,
	              CV_THRESH_BINARY);

	_blobDetectorBright->detect(matrixOfSumKeypointsFromAllChannels,
	                            obj._resultKeypointsOfDetectedDiodes);
}

void LopataFinder::calculateDiodesCoordinates(PololuImuV5& imu, Lopata& obj, std::thread& imuThread)
{
	if (_firstOccurrenceOfTwoPoints && obj._resultKeypointsOfDetectedDiodes.size() == 2)
	{
		_previousPoints.first = obj._resultKeypointsOfDetectedDiodes[0];
		_previousPoints.second = obj._resultKeypointsOfDetectedDiodes[1];
		_firstOccurrenceOfTwoPoints = false;
		imu.stopReading();
		obj.allowSend();
		imuThread.join(); // End thread with IMU
	}
	else
	{
		if (!_firstOccurrenceOfTwoPoints)
		{
			if (obj._resultKeypointsOfDetectedDiodes.size() == 1)
			{
				obj._resultKeypointsOfDetectedDiodes.erase(obj._resultKeypointsOfDetectedDiodes.begin());
				obj._resultKeypointsOfDetectedDiodes.push_back(_previousPoints.first);
				obj._resultKeypointsOfDetectedDiodes.push_back(_previousPoints.second);
				obj.forbidSend();
			}
			if (obj._resultKeypointsOfDetectedDiodes.empty())
			{
				obj._resultKeypointsOfDetectedDiodes.push_back(_previousPoints.first);
				obj._resultKeypointsOfDetectedDiodes.push_back(_previousPoints.second);
				obj.forbidSend();
			}
			else
			{
				obj.allowSend();
			}
			_previousPoints.first = obj._resultKeypointsOfDetectedDiodes[0];
			_previousPoints.second = obj._resultKeypointsOfDetectedDiodes[1];

			const unsigned int firstX = static_cast<unsigned int>(obj._resultKeypointsOfDetectedDiodes[0].pt
			                                                                                             .x
			);
			const unsigned int secondX = static_cast<unsigned int>(obj._resultKeypointsOfDetectedDiodes[1].
				pt.x);
			const unsigned int firstY = static_cast<unsigned int>(obj._resultKeypointsOfDetectedDiodes[0].pt
			                                                                                             .y
			);
			const unsigned int secondY = static_cast<unsigned int>(obj._resultKeypointsOfDetectedDiodes[1].
				pt.y);

			obj._centerXCoordinatesOfLopata = static_cast<unsigned int>((firstX + secondX) / 2);
			obj._centerYCoordinatesOfLopata = static_cast<unsigned int>((firstY + secondY) / 2);

			obj._lenghtPixelProjectionBetweenDiodes = getDistanceBetweenPoints(firstX, secondX, firstY,
			                                                                   secondY);

			imu.stopReading();
			imuThread.join(); // End thread with IMU
			// avnike@outlook.com
		}
		else
		{
			imu.stopReading();
			imuThread.join(); // End thread with IMU
		}
	}
}

void LopataFinder::drawKeypoints(Lopata& obj)
{
	cv::Mat matrixOfDiodes = cv::Mat::zeros(_rows, _cols, cv::DataType<double>::type);
	cv::Mat m1 = cv::Mat::zeros(2, 2, cv::DataType<double>::type);

	const cv::Mat startPointerPosition(1, 2, cv::DataType<double>::type, {70, 1});

	constexpr int compassSide = 150;

	const cv::Point center(compassSide / 2, compassSide / 2);

	if (!_firstOccurrenceOfTwoPoints)
	{
		cv::Mat background = createCompassMatrix(obj, center, startPointerPosition);

		for (std::size_t i = 0; i < compassSide; ++i)
		{
			for (std::size_t j = 0; j < compassSide; ++j)
			{
				matrixOfDiodes.at<double>(i, j) = background.at<double>(i, j);
			}
		}


		cv::putText(matrixOfDiodes, std::to_string(obj._xBound), cv::Point(320, 15),
		            cv::HersheyFonts::FONT_HERSHEY_COMPLEX, .5, 255);
		cv::putText(matrixOfDiodes, std::to_string(obj._yBound), cv::Point(0, 240),
		            cv::HersheyFonts::FONT_HERSHEY_COMPLEX, .5, 255);

		cv::circle(matrixOfDiodes,
		           cv::Point(static_cast<int>(obj._resultKeypointsOfDetectedDiodes[0].pt.x),
		                     static_cast<int>(obj._resultKeypointsOfDetectedDiodes[0].pt.y)),
		           static_cast<int>(obj._resultKeypointsOfDetectedDiodes[0].size / 2), cv::Scalar(255),
		           -1);
		cv::circle(matrixOfDiodes,
		           cv::Point(static_cast<int>(obj._resultKeypointsOfDetectedDiodes[1].pt.x),
		                     static_cast<int>(obj._resultKeypointsOfDetectedDiodes[1].pt.y)),
		           static_cast<int>(obj._resultKeypointsOfDetectedDiodes[1].size / 2), cv::Scalar(255),
		           -1);

		cv::circle(matrixOfDiodes,
		           cv::Point(obj.xCenterImg, obj.yCenterImg),
		           static_cast<int>(obj._resultKeypointsOfDetectedDiodes[1].size / 2), cv::Scalar(255),
		           2);

		cv::circle(_rawImageFromCamera,
		           cv::Point(obj.xCenterImg, obj.yCenterImg),
		           static_cast<int>(obj._resultKeypointsOfDetectedDiodes[1].size / 2), cv::Scalar(255),
		           2);
	}

	cv::imshow("Raw image from camera", _rawImageFromCamera);
	cv::imshow("Detected diodes", matrixOfDiodes);
}

cv::Mat LopataFinder::createCompassMatrix(Lopata& obj, const cv::Point center,
                                          const cv::Mat tmp) const
{
	cv::Mat background(cv::Mat::zeros(150, 150, cv::DataType<double>::type));

	cv::circle(background, center, 50, 128);
	cv::Mat m1(2, 2, cv::DataType<double>::type);

	m1.at<double>(0, 0) = cos(obj._eulerAngles[2] * obj._pi / 180);
	m1.at<double>(0, 1) = sin(obj._eulerAngles[2] * obj._pi / 180);
	m1.at<double>(1, 0) = -sin(obj._eulerAngles[2] * obj._pi / 180);
	m1.at<double>(1, 1) = cos(obj._eulerAngles[2] * obj._pi / 180);

	cv::Mat t = tmp * m1;
	cv::arrowedLine(background, center - cv::Point(t.at<double>(0, 0) / 2, t.at<double>(0, 1) / 2),
	                center + cv::Point(t.at<double>(0, 0) / 2, t.at<double>(0, 1) / 2), 255);

	return background;
}
