#include "LopataFinder.h"

void LopataFinder::frameSizeDetermination()
{
	cv::Mat rawImageFromCamera;
	_webCamera >> rawImageFromCamera;
	_rows = rawImageFromCamera.rows;
	_cols = rawImageFromCamera.cols;
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

/**
* \brief Function to get distance between two points, given by their coordinates on the plane
* \param[in] firstX X coordinate of first point
* \param[in] secondX Y coordinate of first point
* \param[in] firstY X coordinate of second point
* \param[in] secondY Y coordinate of second point
* \return Distance between points
*/
float LopataFinder::getDistanceBetweenPoints(const unsigned& firstX, const unsigned& secondX, const unsigned& firstY,
	const unsigned& secondY) const
{
	return static_cast<float>(std::pow(
		(firstX - secondX) * (firstX - secondX) + (firstY - secondY) * (firstY - secondY),
		0.5));
}

/**
* \brief Function of getting undistorted image from camera
*/
void LopataFinder::getImageFromCamera()
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
void LopataFinder::detectKeypoints(cv::Ptr<cv::SimpleBlobDetector> primaryBlobDetector, const cv::Mat channelsOfHsvImage,
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