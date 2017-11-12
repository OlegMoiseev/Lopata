#include "ImuModule.h"
#include <iostream>
#include <time.h>

int PololuImuV5::initComPortImu()
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
	DCB dcbSerialParams = { 0 };
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
	} while (skip != 'd'); // скип того, что приходит с Ардуино во время инициализации
	std::cout << "Skipped successfylly!" << std::endl;
	return 0;
}

void PololuImuV5::startImu()
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

PololuImuV5::PololuImuV5(const LPCTSTR sPortName)
	: _sPortName(sPortName)
{
	startImu();
}

/**
* \brief Allow data reading from sensor
*/
void PololuImuV5::startReading()
{
	_stopReading = false;
}

/**
* \brief Forbid data reading from sensor
*/
void PololuImuV5::stopReading()
{
	_stopReading = true;
}

/**
* \brief Function of data reading from sensor
* \param[out] q Data output in the form of a quaternion
*/
void PololuImuV5::readComImu(Quaternion& q) const
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

void PololuImuV5::readOutImu(const PololuImuV5* instance, Quaternion& data)
{
	while (!instance->_stopReading)
	{
		instance->readComImu(data);
	}
}