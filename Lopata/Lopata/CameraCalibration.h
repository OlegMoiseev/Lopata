/**
 * \file
 * \brief Header file with class description for camera calibration.
*/
#ifndef CAMERA_CALIBRATION_2017
#define CAMERA_CALIBRATION_2017
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <vector>
#include <string>

namespace timur
{
	/**
	 * \brief Class for working with camera (calibration and removal of distortion).
	 */
	class CameraCalibration
	{
	private:
		/**
		 * \brief Dimension of chessboard(the intersection of cells).
		 */
		const cv::Size _chessboardDimension;

		/**
		 * \brief The length of the chessboard cell side.
		 */
		const float _calibrationSquareDimenshion;

		/**
		 * \brief Camera matrix.
		 * 
		 * Camera matrix:  
		 * |fx 0 cx|
		 * |0 fy cy|
		 * |0  0  1|
		 * focal length (fx,fy), optical centers (cx,cy).
		 */
		cv::Mat _cameraMatrix;

		/**
		 * \brief Distortion coefficients = (k1 k2 p1 p2 k3)
		 * 
		 * Due to radial distortion, straight lines will appear curved. Its effect is more as we move away from the center of image.
		 * \f[x_{distorted}=x(1+k_1r^2+k_2r^4+k_3r^6)\f]
		 * \f[y_{distorted}=y(1+k_1r^2+k_2r^4+k_3r^6)\f].
		 * Similarly, another distortion is the tangential distortion which occurs because image taking lense is not aligned perfectly parallel to the imaging plane.
		 * \f[x_{distorted} = x+[2p_1xy+p_2(r^2+2x^2)]\f]
		 * \f[y_{distorted} = y+[p_1(r^2+2y^2)+2p_2xy]\f].
		 */
		cv::Mat _distanceCoefficients;

		/**
		 * \brief Creation of real 3-dimensional coordinates for chessboard points. 
		 * \param[out] corners Output vector of 3-dimenshion points.
		 */
		void createKnownBoardPosition(std::vector<cv::Point3f>& corners) const;

		/**
		 * \brief Find chessboard points on collection of images.
		 * \param[in] images Input vector of matrices, where need to find a chessboard.
		 * \param[out] allFoundCorners Output vector of 2-dimenshion coordinates of chessboard points.
		 */
		void getChessboardCorners(const std::vector<cv::Mat>& images,
		                          std::vector<std::vector<cv::Point2f>>& allFoundCorners) const;

		/**
		 * \brief Saving camera calibration parameters in file.
		 * \param[in] name Name of file to save.
		 */
		void saveCameraCalibration(const std::string& name) const;

		/**
		 * \brief Calibrating the camera on the image collection and saving images without distortion to a folder "OrImagesCamCalib"
		 * \param[in] calibrationImages Input vector of images with chessboard.
		 */
		void cameraCalibration(const std::vector<cv::Mat>& calibrationImages);

		/**
		 * \brief Calculates image blurriness
		 * \param[in] src Input image.
		 * \return Rate of blurriness.
		 */
		static float calcBlurriness(const cv::Mat& src);

	public:
		/**
		 * \brief CameraCalibration constructor.
		 * \param[in] chessboardDimension Dimension of chessboard(the intersection of cells).
		 * \param[in] calibrationSquareDimenshion The length of the chessboard cell side.
		 */
		CameraCalibration(cv::Size chessboardDimension, float calibrationSquareDimenshion);

		/**
		 * \brief CameraCalibration destructor.
		 * \warning Empty.
		 */
		~CameraCalibration();

		/**
		 * \brief Returning property of _cameraMatrix.
		 * \return Value of the private field _cameraMatrix.
		 */
		cv::Mat cameraMatrix() const;

		/**
		 * \brief Returning property of _distanceCoefficients.
		 * \return Value of the private field _distanceCoefficients.
		 */
		cv::Mat distanceCoefficients() const;

		/**
		 * \brief Download camera calibration parameters from file.
		 * \param[in] name File name for download.
		 */
		void loadCameraCalibration(const std::string& name);

		/**
		 * \brief Download images from file and calibrate camera. (image name: i.png, i = 0,countOfImages)
		 * \param[in] folderName File name for download images.
		 * \param[in] countOfImages Count of images.
		 */
		void cameraCalibrationImagesCollection(const std::string& folderName, uint countOfImages);

		/**
		 * \brief Starting camera calibration.
		 * \param[in] vid Opencv camera object initialized with needed camera.
		 * \param[in] countOfFrames Count of images for calibration.
		 */
		void cameraCalibrationProcess(cv::VideoCapture& vid, uint countOfFrames);

		/**
		 * \brief Transforms an image to compensate for lens distortion.
		 * \param[in] input Input image.
		 * \return Output image without distortion.
		 */
		cv::Mat undistort(cv::Mat& input) const;

		/**
		 * \brief Helps adjust the camera's manual focus.
		 * \param vid Opencv camera object initialized with needed camera.
		 */
		static void focusSetting(cv::VideoCapture& vid);
	};
}

#endif
