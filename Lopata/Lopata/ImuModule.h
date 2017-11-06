#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <iostream>
#include <Windows.h>
#include <time.h>
#include "MathForIMU.h"

class PololuImuV5
{
private:
	HANDLE _hSerial = nullptr;
	const LPCTSTR _sPortName;
	bool _stopReading = false;

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
			ReadFile(_hSerial, &skip, 1, &iSize, nullptr); // �������� 1 ���� ���� ������
		}
		while (skip != 'd'); // ���� ����, ��� �������� � ������� �� ����� �������������
		std::cout << "Skipped successfylly!" << std::endl;
		return 0;
	}

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
	explicit PololuImuV5(const LPCTSTR sPortName = L"COM3") : _sPortName(sPortName)
	{
		startImu();
	}

	void startReading()
	{
		_stopReading = false;
	}

	void stopReading()
	{
		_stopReading = true;
	}

	void readComImu(Quaternion& d) const
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
				ReadFile(_hSerial, &in, 1, &iSize, nullptr); // �������� 1 ���� ������
				if (i == 0 && in == '\r')
				{
					// ��� ������ �������� ������ ��� �����, � ������ ��� �����: '\r\n', ������� �� ������ ��������, ����� �������� ��������� ������ ��� ��������
					// �������, ��� ����, ���� � ������ �������� �� �������� '\r'
					ReadFile(_hSerial, &in, 1, &iSize, nullptr); // ��������� ������ '\n'
					ReadFile(_hSerial, &in, 1, &iSize, nullptr); // ��������� ��� ���������� ������
				}
				sum[j] = in;
				++j;
			}

			switch (i)
			{
			case 0: d._w = atof(sum);
				break;
			case 1: d._x = atof(sum);
				break;
			case 2: d._y = atof(sum);
				break;
			default: d._z = atof(sum);
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
