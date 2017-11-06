#ifndef LOPATA_H
#define LOPATA_H

#include <opencv2/opencv.hpp>
#include "CameraCalibration.h"
#include "MathForIMU.h"
#include "LopataObject.h"
#include <thread>

class LopataFinder
{
	cv::VideoCapture _webCamera;
	timur::CameraCalibration _camCalib;
	int _rows;
	int _cols;
	cv::Ptr<cv::SimpleBlobDetector> _primaryBlobDetectorDark, _blobDetectorBright;
	cv::SimpleBlobDetector::Params _parametersForPrimaryDetectorDark, _parametersForPrimaryDetectorBright;

	bool _firstOccurrenceOfTwoPoints = true;

	cv::Mat _rawImageFromCamera;

	std::pair<cv::KeyPoint, cv::KeyPoint> _previousPoints; // Variable to save old coordinates of two points
	cv::Mat _hsvMatrixOfRawImage;
	std::vector<cv::Mat> _channelsOfHsvImage; // Vectors of matrixes - to split HSV image on channels
	std::array<std::vector<cv::KeyPoint>, 3> _keypointsForEachChannel;

	void frameSizeDetermination()
	{
		cv::Mat rawImageFromCamera;
		_webCamera >> rawImageFromCamera;
		_rows = rawImageFromCamera.rows;
		_cols = rawImageFromCamera.cols;
		std::cout << _cols << "x" << _rows << std::endl;
	}

	void detectorParamsSetup()
	{
		_parametersForPrimaryDetectorDark.filterByColor = true;
		_parametersForPrimaryDetectorDark.blobColor = 0;

		// Change thresholds
		_parametersForPrimaryDetectorDark.minThreshold = 100;
		_parametersForPrimaryDetectorDark.maxThreshold = 255;
		//parametersForPrimaryDetectorDark.thresholdStep = 1;

		// Filter by Area.
		_parametersForPrimaryDetectorDark.filterByArea = true;
		_parametersForPrimaryDetectorDark.minArea = 36;

		// Filter by Circularity
		_parametersForPrimaryDetectorDark.filterByCircularity = true;
		_parametersForPrimaryDetectorDark.minCircularity = 0.7;

		// Filter by Convexity
		_parametersForPrimaryDetectorDark.filterByConvexity = true;
		_parametersForPrimaryDetectorDark.minConvexity = 0.4;

		// Filter by Inertia
		_parametersForPrimaryDetectorDark.filterByInertia = true;
		_parametersForPrimaryDetectorDark.minInertiaRatio = 0.3;
		//================================================================================
		_parametersForPrimaryDetectorBright.filterByColor = true;
		_parametersForPrimaryDetectorBright.blobColor = 255;

		// Change thresholds
		_parametersForPrimaryDetectorBright.minThreshold = 100;
		_parametersForPrimaryDetectorBright.maxThreshold = 255;
		//parametersForPrimaryDetectorBright.thresholdStep = 1;

		// Filter by Area.
		_parametersForPrimaryDetectorBright.filterByArea = true;
		_parametersForPrimaryDetectorBright.minArea = 36;

		// Filter by Circularity
		_parametersForPrimaryDetectorBright.filterByCircularity = true;
		_parametersForPrimaryDetectorBright.minCircularity = 0.7;

		// Filter by Convexity
		_parametersForPrimaryDetectorBright.filterByConvexity = true;
		_parametersForPrimaryDetectorBright.minConvexity = 0.4;

		// Filter by Inertia
		_parametersForPrimaryDetectorBright.filterByInertia = true;
		_parametersForPrimaryDetectorBright.minInertiaRatio = 0.3;
	}

	float getDistanceBetweenPoints(const unsigned& firstX, const unsigned& secondX, const unsigned& firstY,
		const unsigned& secondY) const
	{
		return static_cast<float>(std::pow(
			(firstX - secondX) * (firstX - secondX) + (firstY - secondY) * (firstY - secondY),
			0.5));
	}

	void getImageFromCamera()
	{
		_webCamera >> _rawImageFromCamera;
		_rawImageFromCamera = _camCalib.undistort(_rawImageFromCamera);
	}

	static void detectKeypoints(cv::Ptr<cv::SimpleBlobDetector> primaryBlobDetector, const cv::Mat channelsOfHsvImage,
		std::vector<cv::KeyPoint> keypointsForEachChannel, cv::Mat& matrixOfKeypointsForOneChannel)
	{
		primaryBlobDetector->detect(channelsOfHsvImage, keypointsForEachChannel);
		for (size_t i = 0; i < keypointsForEachChannel.size(); ++i)
		{
			cv::circle(matrixOfKeypointsForOneChannel,
				cv::Point(static_cast<int>(keypointsForEachChannel[i].pt.x),
					static_cast<int>(keypointsForEachChannel[i].pt.y)),
				static_cast<int>(keypointsForEachChannel[i].size / 2), cv::Scalar(255), -1);
		}
	}

public:

	explicit LopataFinder(cv::VideoCapture &cam, timur::CameraCalibration &calib):_webCamera(cam), _camCalib(calib)
	{
		if (!_webCamera.isOpened())
		{
			std::cout << "I can't open the camera!" << std::endl;
		}
		std::cout << "Camera opened successfully!" << std::endl;

		frameSizeDetermination();
		_camCalib.loadCameraCalibration("CamCalib.txt");

		detectorParamsSetup();
		_primaryBlobDetectorDark = cv::SimpleBlobDetector::create(_parametersForPrimaryDetectorDark);
		_blobDetectorBright = cv::SimpleBlobDetector::create(_parametersForPrimaryDetectorBright);
	}

	void firstOccurrenceFalse()
	{
		_firstOccurrenceOfTwoPoints = false;
	}

	void detectDiodes(Lopata &obj)
	{
		std::array<cv::Mat, 3> matrixOfKeypointsForOneChannel = {
			cv::Mat::zeros(_rows, _cols, CV_8UC1), cv::Mat::zeros(_rows, _cols, CV_8UC1), cv::Mat::zeros(_rows, _cols, CV_8UC1)
		};
		cv::Mat matrixOfSumKeypointsFromAllChannels(_rows, _cols, CV_8UC1, cv::Scalar(0, 0, 0));

		getImageFromCamera();

		cv::cvtColor(_rawImageFromCamera, _hsvMatrixOfRawImage, CV_BGR2HSV); // convert camera image format from BGR to HSV
		split(_hsvMatrixOfRawImage, _channelsOfHsvImage);
		cv::threshold(_channelsOfHsvImage[2], _channelsOfHsvImage[2], 250, 255, CV_THRESH_BINARY);

		//----------------------------------------------------------------
		// Start threads with OpenCV
		std::thread t1Thread(detectKeypoints, _primaryBlobDetectorDark, _channelsOfHsvImage[0], _keypointsForEachChannel[0],
			std::ref(matrixOfKeypointsForOneChannel[0]));
		std::thread t2Thread(detectKeypoints, _primaryBlobDetectorDark, _channelsOfHsvImage[1], _keypointsForEachChannel[1],
			std::ref(matrixOfKeypointsForOneChannel[1]));
		std::thread t3Thread(detectKeypoints, _blobDetectorBright, _channelsOfHsvImage[2], _keypointsForEachChannel[2],
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
				matrixOfSumKeypointsFromAllChannels.at<uchar>(i, j) = matrixOfKeypointsForOneChannel[0].at<uchar>(i, j) / 3 +
					matrixOfKeypointsForOneChannel[1].at<uchar>(i, j) / 3 + matrixOfKeypointsForOneChannel[2].at<uchar>(i, j) / 3;
			}
		}
		cv::threshold(matrixOfSumKeypointsFromAllChannels, matrixOfSumKeypointsFromAllChannels, 169, 255, CV_THRESH_BINARY);

		_blobDetectorBright->detect(matrixOfSumKeypointsFromAllChannels, obj._resultKeypointsOfDetectedDiodes);
	}

	void cameraDataProcessing(PololuImuV5 &imu, robot::FanucM20iA &robo, Lopata &obj, std::thread& imuThread)
	{
		if (_firstOccurrenceOfTwoPoints && obj._resultKeypointsOfDetectedDiodes.size() == 2)
		{
			_previousPoints.first = obj._resultKeypointsOfDetectedDiodes[0];
			_previousPoints.second = obj._resultKeypointsOfDetectedDiodes[1];
			_firstOccurrenceOfTwoPoints = false;
			imu.stopReading();
			robo.allowSend();
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
					robo.forbidSend();
				}
				if (obj._resultKeypointsOfDetectedDiodes.size() == 0)
				{
					obj._resultKeypointsOfDetectedDiodes.push_back(_previousPoints.first);
					obj._resultKeypointsOfDetectedDiodes.push_back(_previousPoints.second);
					robo.forbidSend();
				}
				else
				{
					robo.allowSend();
				}
				_previousPoints.first = obj._resultKeypointsOfDetectedDiodes[0];
				_previousPoints.second = obj._resultKeypointsOfDetectedDiodes[1];

				const unsigned int firstX = static_cast<unsigned int>(obj._resultKeypointsOfDetectedDiodes[0].pt.x);
				const unsigned int secondX = static_cast<unsigned int>(obj._resultKeypointsOfDetectedDiodes[1].pt.x);
				const unsigned int firstY = static_cast<unsigned int>(obj._resultKeypointsOfDetectedDiodes[0].pt.y);
				const unsigned int secondY = static_cast<unsigned int>(obj._resultKeypointsOfDetectedDiodes[1].pt.y);

				obj._centerXCoordinatesOfLopata = static_cast<unsigned int>((firstX + secondX) / 2);
				obj._centerYCoordinatesOfLopata = static_cast<unsigned int>((firstY + secondY) / 2);

				const float realPixelDistanceBetweenDiodes = getDistanceBetweenPoints(firstX, secondX, firstY, secondY);

				//=======================================================================================================

				imu.stopReading();
				imuThread.join(); // End thread with IMU

				calculateCoordinates(obj, realPixelDistanceBetweenDiodes);

				std::cout << obj._centerXCoordinatesOfLopata << "\t" << obj._centerYCoordinatesOfLopata << "\t" << obj._altitude << std::endl;

				robo.sendCoordinates(obj);
				// avnike@outlook.com
				//=======================================================================================================
			}
			else
			{
				imu.stopReading();
				imuThread.join(); // End thread with IMU
			}
		}
	}

	void drawKeypoints(Lopata &obj)
	{
		cv::Mat matrixOfDiodes = cv::Mat::zeros(_rows, _cols, CV_8UC1);
		if (!_firstOccurrenceOfTwoPoints)
		{
			cv::circle(matrixOfDiodes,
				cv::Point(static_cast<int>(obj._resultKeypointsOfDetectedDiodes[0].pt.x),
					static_cast<int>(obj._resultKeypointsOfDetectedDiodes[0].pt.y)),
				static_cast<int>(obj._resultKeypointsOfDetectedDiodes[0].size / 2), cv::Scalar(255), -1);
			cv::circle(matrixOfDiodes,
				cv::Point(static_cast<int>(obj._resultKeypointsOfDetectedDiodes[1].pt.x),
					static_cast<int>(obj._resultKeypointsOfDetectedDiodes[1].pt.y)),
				static_cast<int>(obj._resultKeypointsOfDetectedDiodes[1].size / 2), cv::Scalar(255), -1);

			cv::circle(matrixOfDiodes,
				cv::Point(obj._centerXCoordinatesOfLopata, obj._centerYCoordinatesOfLopata),
				static_cast<int>(obj._resultKeypointsOfDetectedDiodes[1].size / 2), cv::Scalar(255), 2);

			cv::circle(_rawImageFromCamera,
				cv::Point(obj._centerXCoordinatesOfLopata, obj._centerYCoordinatesOfLopata),
				static_cast<int>(obj._resultKeypointsOfDetectedDiodes[1].size / 2), cv::Scalar(255), 2);
		}

		cv::imshow("Raw image from camera", _rawImageFromCamera);
		cv::imshow("Detected diodes", matrixOfDiodes);
	}
};
#endif // LOPATA_H
