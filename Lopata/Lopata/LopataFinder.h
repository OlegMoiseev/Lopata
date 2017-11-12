#ifndef LOPATA_H
#define LOPATA_H

#include <opencv2/opencv.hpp>
#include "CameraCalibration.h"
#include "LopataObject.h"
#include <thread>

class LopataFinder
{
	/**
	 * \brief Camera from OpenCV
	 */
	cv::VideoCapture _webCamera;
	/**
	 * \brief Calibration of camera by Timur
	 */
	timur::CameraCalibration _camCalib;
	/**
	 * \brief Size of capture: rows
	 */
	int _rows;
	/**
	 * \brief Size of capture: columns
	 */
	int _cols;
	/**
	 * \brief Initializing of detector of dark areas
	 */
	cv::Ptr<cv::SimpleBlobDetector> _primaryBlobDetectorDark;
	/**
	* \brief Initializing of detector of bright areas
	*/
	cv::Ptr<cv::SimpleBlobDetector> _blobDetectorBright;
	/**
	 * \brief Initializing of parameters for dark detector
	 */
	cv::SimpleBlobDetector::Params _parametersForPrimaryDetectorDark;
	/**
	* \brief Initializing of parameters for bright detector
	 */
	cv::SimpleBlobDetector::Params _parametersForPrimaryDetectorBright;

	/**
	 * \brief Flag of first occurence of two points of Lopata
	 */
	bool _firstOccurrenceOfTwoPoints = true;

	/**
	 * \brief Matrix for raw image from camera
	 */
	cv::Mat _rawImageFromCamera;

	/**
	 * \brief Variable to save old coordinates of two points
	 */
	std::pair<cv::KeyPoint, cv::KeyPoint> _previousPoints;
	/**
	 * \brief Matrix with raw BGR image translated into HSV form
	 */
	cv::Mat _hsvMatrixOfRawImage;
	/**
	 * \brief Vector of matrixes with 3 channels of HSV image
	 */
	std::vector<cv::Mat> _channelsOfHsvImage;
	/**
	 * \brief Array of 3 vectors of KeyPoints for each channel
	 */
	std::array<std::vector<cv::KeyPoint>, 3> _keypointsForEachChannel;

	/**
	 * \brief Function to set variables _rows and _cols
	 */
	void frameSizeDetermination()
	{
		cv::Mat rawImageFromCamera;
		_webCamera >> rawImageFromCamera;
		_rows = rawImageFromCamera.rows;
		_cols = rawImageFromCamera.cols;
		std::cout << _cols << "x" << _rows << std::endl;
	}

	/**
	 * \brief Function to set parameters for blob detectors
	 */
	void detectorParamsSetup()
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

	/**
	 * \brief Function to get distance between two points, given by their coordinates on the plane
	 * \param[in] firstX X coordinate of first point
	 * \param[in] secondX Y coordinate of first point
	 * \param[in] firstY X coordinate of second point
	 * \param[in] secondY Y coordinate of second point
	 * \return Distance between points
	 */
	float getDistanceBetweenPoints(const unsigned& firstX, const unsigned& secondX, const unsigned& firstY,
		const unsigned& secondY) const
	{
		return static_cast<float>(std::pow(
			(firstX - secondX) * (firstX - secondX) + (firstY - secondY) * (firstY - secondY),
			0.5));
	}

	/**
	 * \brief Function of getting undistorted image from camera
	 */
	void getImageFromCamera()
	{
		_webCamera >> _rawImageFromCamera;
		_rawImageFromCamera = _camCalib.undistort(_rawImageFromCamera);
	}

	/**
	 * \brief Function of detecting keypoints on the image
	 * \param[in] primaryBlobDetector Blob detector, which will do the work
	 * \param[in] channelsOfHsvImage Image, on which will be detected keypoints
	 * \param[in] keypointsForEachChannel Detected keypoints
	 * \param[in] matrixOfKeypointsForOneChannel Matrix with drawn keypoints
	 */
	static void detectKeypoints(cv::Ptr<cv::SimpleBlobDetector> primaryBlobDetector, const cv::Mat channelsOfHsvImage,
		std::vector<cv::KeyPoint> keypointsForEachChannel, cv::Mat& matrixOfKeypointsForOneChannel)
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

public:

	/**
	 * \brief Constructor for finder of the Lopata
	 * \param cam Used camera
	 * \param calib Used calibration
	 */
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

	/**
	 * \brief Function of occurence of two points
	 */
	void firstOccurrenceFalse()
	{
		_firstOccurrenceOfTwoPoints = false;
	}

	/**
	 * \brief Function of detection diodes on the image
	 * Result of work - _resultKeypointsOfDetectedDiodes
	 * \param[out] obj Object whose diodes we're finding
	 */
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

	/**
	 * \brief Function of final processing of gotten data and send it to robot, if it is possible
	 * \param[in] imu IMU sensor
	 * \param[in] robo Robot which we control
	 * \param[in] obj Lopata object
	 * \param[in] imuThread Thread of getting data from IMU
	 */
	void calculateDiodesCoordinates(PololuImuV5 &imu, robot::FanucM20iA &robo, Lopata &obj, std::thread& imuThread)
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

				obj._realPixelDistanceBetweenDiodes = getDistanceBetweenPoints(firstX, secondX, firstY, secondY);

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

	/**
	 * \brief Function of drawing diodes and their common sensor on the raw and clear images
	 * \param[in] obj The object whose diodes will be drawn
	 */
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
