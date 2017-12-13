#ifndef LOPATA_H
#define LOPATA_H

#include <thread>
#include <array>

#include <opencv2/opencv.hpp>

#include "CameraCalibration.h"
#include "LopataObject.h"
#include "ImuModule.h"
#include "CamCalibWI.h"


class LopataFinder
{
	/**
	 * \brief Camera from OpenCV
	 */
	cv::VideoCapture _webCamera;
	/**
	 * \brief Calibration of camera by Timur
	 */
	timur::CamCalibWi _camCalib;
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
	void frameSizeDetermination();

	/**
	 * \brief Function to set parameters for blob detectors
	 */
	void detectorParamsSetup();

	/**
	 * \brief Function to get distance between two points, given by their coordinates on the plane
	 * \param[in] firstX X coordinate of first point
	 * \param[in] secondX Y coordinate of first point
	 * \param[in] firstY X coordinate of second point
	 * \param[in] secondY Y coordinate of second point
	 * \return Distance between points
	 */
	float getDistanceBetweenPoints(const unsigned& firstX, const unsigned& secondX,
	                               const unsigned& firstY, const unsigned& secondY) const;

	/**
	 * \brief Function of getting undistorted image from camera
	 */
	void getImageFromCamera();

	/**
	 * \brief Function of detecting keypoints on the image
	 * \param[in] primaryBlobDetector Blob detector, which will do the work
	 * \param[in] channelsOfHsvImage Image, on which will be detected keypoints
	 * \param[in] keypointsForEachChannel Detected keypoints
	 * \param[in] matrixOfKeypointsForOneChannel Matrix with drawn keypoints
	 */
	static void detectKeypoints(cv::Ptr<cv::SimpleBlobDetector> primaryBlobDetector,
	                            const cv::Mat channelsOfHsvImage,
	                            std::vector<cv::KeyPoint> keypointsForEachChannel,
	                            cv::Mat& matrixOfKeypointsForOneChannel);

public:

	/**
	 * \brief Constructor for finder of the Lopata
	 * \param cam Used camera
	 * \param calib Used calibration
	 */
	explicit LopataFinder(cv::VideoCapture& cam, timur::CamCalibWi& calib);

	/**
	 * \brief Function of occurence of two points
	 */
	void firstOccurrenceFalse();

	/**
	 * \brief Function of detection diodes on the image
	 * Result of work - _resultKeypointsOfDetectedDiodes
	 * \param[out] obj Object whose diodes we're finding
	 */
	void detectDiodes(Lopata& obj);

	/**
	 * \brief Function of final processing of gotten data and send it to robot, if it is possible
	 * \param[in] imu IMU sensor
	 * \param[in] obj Lopata object
	 * \param[in] imuThread Thread of getting data from IMU
	 */
	void calculateDiodesCoordinates(PololuImuV5& imu, Lopata& obj,
	                                std::thread& imuThread);

	/**
	 * \brief Function of drawing diodes and their common sensor on the raw and clear images
	 * \param[in] obj The object whose diodes will be drawn
	 */
	void drawKeypoints(Lopata& obj);

	cv::Mat createCompassMatrix(Lopata& obj, const cv::Point center, const cv::Mat tmp) const;
};
#endif // LOPATA_H
