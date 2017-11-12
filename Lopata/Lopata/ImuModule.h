#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Windows.h>
#include "MathForIMU.h"

/**
 * \brief Class of IMU module Pololu v5
 */
class PololuImuV5
{
private:
	/**
	 * \brief Announsment of using COM port
	 */
	HANDLE _hSerial = nullptr;
	/**
	 * \brief Name of using COM port
	 */
	const LPCTSTR _sPortName;
	/**
	 * \brief Flag of stopping read to synchronize flows
	 */
	bool _stopReading = false;

	/**
	 * \brief Function of initializing of COM port
	 * \return 0 if all was OK
	 * \return 100* if something went wrong
	 * \TODO rewrite with exceptions
	 */
	int initComPortImu();

	/**
	 * \brief Start work with IMU module
	 */
	void startImu();

public:
	/**
	 * \brief Constructor with starting work with IMU sensor
	 * \param[in] sPortName Name of the port, to which sensor connected
	 */
	explicit PololuImuV5(const LPCTSTR sPortName = L"COM3");

	/**
	 * \brief Allow data reading from sensor
	 */
	void startReading();

	/**
	 * \brief Forbid data reading from sensor
	 */
	void stopReading();

	/**
	 * \brief Function of data reading from sensor
	 * \param[out] q Data output in the form of a quaternion
	 */
	void readComImu(Quaternion& q) const;

	/**
	 * \brief Function of reading data if it was allowed
	 * \param[in] instance Sensor to reading 
	 * \param[in] data Quaternion to accumulate data
	 */
	static void readOutImu(const PololuImuV5* instance, Quaternion& data);
};
#endif // IMU_MODULE_H
