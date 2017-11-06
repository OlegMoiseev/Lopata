#include "CameraCalibration.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/shape/hist_cost.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

void timur::CameraCalibration::createKnownBoardPosition(std::vector<cv::Point3f>& corners) const
{
	for (uint i = 0; i < _chessboardDimension.height; ++i)
	{
		for (uint j = 0; j < _chessboardDimension.width; ++j)
		{
			corners.push_back(cv::Point3f(j * _calibrationSquareDimenshion, i * _calibrationSquareDimenshion, 0.0f));
		}
	}
}

void timur::CameraCalibration::getChessboardCorners(const std::vector<cv::Mat>& images,
                                                    std::vector<std::vector<cv::Point2f>>& allFoundCorners) const
{
	for (auto iter = images.begin(); iter != images.end(); ++iter)
	{
		std::vector<cv::Point2f> pointBuf;
		const bool found = cv::findChessboardCorners(*iter, _chessboardDimension, pointBuf,
		                                             CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE |
		                                             CV_CALIB_CB_FAST_CHECK);

		if (found)
		{
			cv::Mat imageGray;
			cv::cvtColor(*iter, imageGray, CV_BGR2GRAY);
			cv::cornerSubPix(imageGray, pointBuf, cv::Size(11, 11),
			                 cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			allFoundCorners.push_back(pointBuf);
		}
	}
}

void timur::CameraCalibration::saveCameraCalibration(const std::string& name) const
{
	std::ofstream outStream(name);
	if (outStream)
	{
		int rows = _cameraMatrix.rows;
		int columns = _cameraMatrix.cols;

		outStream << rows << std::endl;
		outStream << columns << std::endl;

		for (int r = 0; r < rows; ++r)
		{
			for (int c = 0; c < columns; ++c)
			{
				outStream << _cameraMatrix.at<double>(r, c) << std::endl;
			}
		}

		rows = _distanceCoefficients.rows;
		columns = _distanceCoefficients.cols;

		outStream << rows << std::endl;
		outStream << columns << std::endl;

		for (int r = 0; r < rows; ++r)
		{
			for (int c = 0; c < columns; ++c)
			{
				outStream << _distanceCoefficients.at<double>(r, c) << std::endl;
			}
		}

		outStream.close();
	}
}

void timur::CameraCalibration::cameraCalibration(const std::vector<cv::Mat>& calibrationImages)
{
	std::vector<std::vector<cv::Point2f>> chessboardImageSpacePoints;
	getChessboardCorners(calibrationImages, chessboardImageSpacePoints);

	std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1);
	createKnownBoardPosition(worldSpaceCornerPoints[0]);
	worldSpaceCornerPoints.resize(chessboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

	std::vector<cv::Mat> rVectors;
	std::vector<cv::Mat> tVectors;

	cv::calibrateCamera(worldSpaceCornerPoints, chessboardImageSpacePoints, _chessboardDimension, _cameraMatrix,
	                    _distanceCoefficients, rVectors, tVectors);
	system("mkdir OrImagesCamCalib");
	int i = 0;
	for (auto iter = calibrationImages.begin(); iter != calibrationImages.end(); ++iter)
	{
		cv::imwrite("OrImagesCamCalib/" + std::to_string(i) + ".png", *iter);

		cv::Mat tmp;
		cv::undistort(*iter, tmp, _cameraMatrix, _distanceCoefficients);
		cv::imwrite("OrImagesCamCalib/" + std::to_string(i) + '_' + ".png", tmp);
		++i;
	}
}

timur::CameraCalibration::CameraCalibration(const cv::Size chessboardDimension,
                                            const float calibrationSquareDimenshion) :
	_chessboardDimension(chessboardDimension), _calibrationSquareDimenshion(calibrationSquareDimenshion)
{
	_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	_distanceCoefficients = cv::Mat::zeros(8, 1, CV_64F);
}

timur::CameraCalibration::~CameraCalibration()
{
}

cv::Mat timur::CameraCalibration::cameraMatrix() const { return _cameraMatrix; }

cv::Mat timur::CameraCalibration::distanceCoefficients() const { return _distanceCoefficients; }

void timur::CameraCalibration::loadCameraCalibration(const std::string& name)
{
	std::ifstream inStream(name);
	if (inStream)
	{
		int rows;
		int columns;

		inStream >> rows;
		inStream >> columns;

		_cameraMatrix = cv::Mat(cv::Size(columns, rows), CV_64F);

		for (int r = 0; r < rows; ++r)
		{
			for (int c = 0; c < columns; ++c)
			{
				inStream >> _cameraMatrix.at<double>(r, c);
				std::cout << _cameraMatrix.at<double>(r, c) << std::endl;
			}
		}

		inStream >> rows;
		inStream >> columns;

		_distanceCoefficients = cv::Mat::zeros(rows, columns, CV_64F);

		for (int r = 0; r < rows; ++r)
		{
			for (int c = 0; c < columns; ++c)
			{
				inStream >> _distanceCoefficients.at<double>(r, c);
				std::cout << _distanceCoefficients.at<double>(r, c) << std::endl;
			}
		}

		inStream.close();
	}
}

void timur::CameraCalibration::cameraCalibrationImagesCollection(const std::string& folderName,
                                                                 const uint countOfImages)
{
	std::vector<cv::Mat> calibrationImages;
	for (uint i = 0; i < countOfImages; ++i)
	{
		calibrationImages.push_back(cv::imread(folderName + '/' + std::to_string(i) + ".png"));
	}
	cameraCalibration(calibrationImages);
}

void timur::CameraCalibration::cameraCalibrationProcess(cv::VideoCapture& vid, const uint countOfFrames)
{
	if (!vid.isOpened())
	{
		return;
	}

	cv::Mat frame, frameToDraw;
	std::vector<cv::Mat> savedImages;
	std::vector<cv::Vec2f> foundPoints;

	cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
	uint countOfGoodFrames = 0;
	while (true)
	{
		if (!vid.read(frame))
			break;
		frame.copyTo(frameToDraw);
		const bool found = cv::findChessboardCorners(frame, _chessboardDimension, foundPoints,
		                                             CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE |
		                                             CV_CALIB_CB_FAST_CHECK);
		if (found)
		{
			cv::Mat imageGray;
			cv::cvtColor(frame, imageGray, CV_BGR2GRAY);
			cv::cornerSubPix(imageGray, foundPoints, cv::Size(11, 11),
			                 cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			cv::drawChessboardCorners(frameToDraw, _chessboardDimension, cv::Mat(foundPoints), found);
		}
		cv::imshow("Webcam", frameToDraw);
		if (cv::waitKey(1) == ' ' && found)
		{
			++countOfGoodFrames;
			cv::Mat temp;
			frame.copyTo(temp);
			savedImages.push_back(temp);
			std::cout << countOfGoodFrames << "/" << countOfFrames << std::endl;
			if (savedImages.size() >= countOfFrames)
			{
				std::cout << "Started calibration.." << std::endl;
				cameraCalibration(savedImages);
				std::cout << "Saving calibration parametrs.." << std::endl;
				saveCameraCalibration("CamCalib.txt");
				std::cout << "Saved!" << std::endl;
				break;
			}
		}
	}
}

cv::Mat timur::CameraCalibration::undistort(cv::Mat& inputImage) const
{
	cv::Mat outputImage;
	cv::undistort(inputImage, outputImage, _cameraMatrix, _distanceCoefficients);
	return outputImage;
}

float timur::CameraCalibration::calcBlurriness(const cv::Mat& src)
{
	cv::Mat gx, gy;
	cv::Sobel(src, gx, CV_32F, 1, 0);
	cv::Sobel(src, gy, CV_32F, 0, 1);
	const double normGx = cv::norm(gx);
	const double normGy = cv::norm(gy);
	const double sumSq = normGx * normGx + normGy * normGy;
	return static_cast<float>(1. / (sumSq / src.size().area() + 1e-6));
}

void timur::CameraCalibration::focusSetting(cv::VideoCapture& vid)
{
	while (true) {
		cv::Mat temp;
		vid >> temp;
		cv::putText(temp, std::to_string(calcBlurriness(temp)), cv::Point(50,50),
			cv::FONT_HERSHEY_SIMPLEX, 2,
			cv::Scalar(0, 0, 255));
		imshow("Image View", temp);
		if (cv::waitKey(1) == 27) break;
	}
}
