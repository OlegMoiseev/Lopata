#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <iostream>
#include <Windows.h>
#include <time.h>
#include "MathForIMU.h"

/*
 * \TODO take out all methods
 */
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
	int initComPortImu()
	{
		_hSerial = ::CreateFile(_sPortName, GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL,
		                        nullptr);
		if (_hSerial == INVALID_HANDLE_VALUE)
		{
			if (GetLastError() == ERROR_FILE_NOT_FOUND)
			{
				std::cout << "Serial port does not exist" << std::endl;
				return 1001;
			}
			std::cout << "Some other error occurred" << std::endl;
			return 1002;
		}
		DCB dcbSerialParams = {0};
		dcbSerialParams.DCBlength = sizeof dcbSerialParams;
		if (!GetCommState(_hSerial, &dcbSerialParams))
		{
			std::cout << "Getting state error" << std::endl;
			return 1003;
		}
		dcbSerialParams.BaudRate = CBR_256000;
		dcbSerialParams.ByteSize = 8;
		dcbSerialParams.StopBits = ONESTOPBIT;
		dcbSerialParams.Parity = NOPARITY;
		if (!SetCommState(_hSerial, &dcbSerialParams))
		{
			std::cout << "Error setting serial port state" << std::endl;
			return 1004;
		}
		std::cout << "Connected successfully!" << std::endl;

		DWORD iSize;
		char skip;

		do
		{
			ReadFile(_hSerial, &skip, 1, &iSize, nullptr); // получаем 1 байт кода ответа
		}
		while (skip != 'd'); // скип того, что приходит с Ардуино во время инициализации
		std::cout << "Skipped successfylly!" << std::endl;
		return 0;
	}

	/**
	 * \brief Start work with IMU module
	 */
	void startImu()
	{
		if (initComPortImu() == 0)
		{
			const time_t startT = time(nullptr);
			std::cout << "Start of module";
			int flag4Anim = 0;
			Quaternion quaternionOfLopataRotation;
			while (time(nullptr) - startT < 3)
			{
				readComImu(quaternionOfLopataRotation);
				if (time(nullptr) - startT == 1 && flag4Anim == 0)
				{
					std::cout << '.';
					++flag4Anim;
				}
				if (time(nullptr) - startT == 2 && flag4Anim == 1)
				{
					std::cout << '.';
					++flag4Anim;
				}
				if (time(nullptr) - startT == 3 && flag4Anim == 2)
				{
					std::cout << '.' << std::endl;
					++flag4Anim;
				}
			}
		}
	}

public:
	/**
	 * \brief Constructor with starting work with IMU sensor
	 * \param[in] sPortName Name of the port, to which sensor connected
	 */
	explicit PololuImuV5(const LPCTSTR sPortName = L"COM3")
		: _sPortName(sPortName)
	{
		startImu();
	}

	/**
	 * \brief Allow data reading from sensor
	 */
	void startReading()
	{
		_stopReading = false;
	}

	/**
	 * \brief Forbid data reading from sensor
	 */
	void stopReading()
	{
		_stopReading = true;
	}

	/**
	 * \brief Function of data reading from sensor
	 * \param[out] q Data output in the form of a quaternion
	 */
	void readComImu(Quaternion& q) const
	{
		DWORD iSize;
#ifdef DEBUG
		std::cout << "Accel&Gyro: ";
#endif // DEBUG
		for (int i = 0; i < 4; ++i)
		{
			char sum[64];
			int j = 0;
			char in = '1';
			while (in != ' ' && in != '\n' && in != '\r' && in != '\t')
			{
				ReadFile(_hSerial, &in, 1, &iSize, nullptr); // получаем 1 байт ответа
				if (i == 0 && in == '\r')
				{
					// при старте передачи датчик шлёт мусор, а именно два байта: '\r\n', которые мы должны откинуть, чтобы получить дальнеёшую работу без смещения
					// поэтому, идём сюда, если в начале передачи мы получили '\r'
					ReadFile(_hSerial, &in, 1, &iSize, nullptr); // считываем символ '\n'
					ReadFile(_hSerial, &in, 1, &iSize, nullptr); // считываем уже актуальные данные
				}
				sum[j] = in;
				++j;
			}

			switch (i)
			{
			case 0: q._w = atof(sum);
				break;
			case 1: q._x = atof(sum);
				break;
			case 2: q._y = atof(sum);
				break;
			default: q._z = atof(sum);
				break;
			}
		}
	}

	static void readOutImu(const PololuImuV5* instance, Quaternion& data)
	{
		while (!instance->_stopReading)
		{
			instance->readComImu(data);
		}
	}
};
#endif // IMU_MODULE_H
