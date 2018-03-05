#include <iostream>
#include <sstream>

#include "Robots.h"
#include "LopataObject.h"


struct ExtremeValuesMax
{
	const int _x = 1380000;
	const int _y = 465000;
	const int _z = 1200000;
} maxValueOf;

struct ExtremeValuesMin
{
	const int _x = 620000;
	const int _y = -465000;
	const int _z = 410000;
} minValueOf;

struct MultConstants
{
	const int _x = 1000;
	const int _y = 1000;
	const int _z = 1000;
	const int _degrees = 1000;
} MULT;

const std::array<double, 6> robot::FanucM20iA::homepos = { 985000., 0., 940000., 180000., 0., 0 };

robot::FanucM20iA::FanucM20iA(const int c)
	: _connect(c)
{
	if (_connect)
	{
		if (initSockets())
		{
			conSocket();
			standCoord(2);
		}
	}
}

bool robot::FanucM20iA::canSendCoordinates(Lopata& obj) const
{
	return obj.sendCoordinates & _connect;
}

bool robot::FanucM20iA::initSockets()
{
	if (WSAStartup(0x202, reinterpret_cast<WSADATA *>(&_buff[0])))
	{
		std::cout << "WSAStart error %d" << WSAGetLastError() << std::endl;
		return false;
	}

	_mySock = socket(AF_INET, SOCK_STREAM, 0);
	_sockrecv = socket(AF_INET, SOCK_STREAM, 0);
	return true;
}

void robot::FanucM20iA::info() const
{
	std::cout << "Port to send: " << _ports << std::endl << "Port to recieve: " << _portr << std::endl
			<< "IP: " <<
			_tmpAddrString << std::endl;
}

bool robot::FanucM20iA::start()
{
	if (WSAStartup(0x202, reinterpret_cast<WSADATA *>(&_buff[0])))
	{
		std::cout << "WSAStart error %d" << WSAGetLastError() << std::endl;
		return false;
	}
	std::cout << "Started successfuly!" << std::endl;
	return true;
}

bool robot::FanucM20iA::conSocket() const
{
	const char* serveraddr = _tmpAddrString.c_str();
	sockaddr_in destAddr;
	destAddr.sin_family = AF_INET;
	destAddr.sin_port = htons(_ports);
	destAddr.sin_addr.s_addr = inet_addr(serveraddr);
	if (connect(_mySock, reinterpret_cast<sockaddr *>(&destAddr), sizeof(destAddr)) == SOCKET_ERROR)
	{
		std::cout << "Connect error " << WSAGetLastError() << std::endl;
		system("pause");
		return false;
	}
	sockaddr_in recvAddr;
	recvAddr.sin_family = AF_INET;
	recvAddr.sin_port = htons(_portr);
	recvAddr.sin_addr.s_addr = inet_addr(serveraddr);

	if (connect(_sockrecv, reinterpret_cast<sockaddr *>(&recvAddr), sizeof(recvAddr)) == SOCKET_ERROR)
	{
		std::cout << "Connect error " << WSAGetLastError() << std::endl;
		system("pause");
		return false;
	}
	return true;
}

bool robot::FanucM20iA::sendany(const char* str) const
{
	const int iResult = send(_mySock, str, static_cast<int>(strlen(str)), 0);
	//посылаем данные из буфера
	if (iResult == SOCKET_ERROR)
	{
		std::cout << "Send failed with error: " << WSAGetLastError() << std::endl;
		closesocket(_mySock);
		WSACleanup();
		return false;
	}
	return true;
}

bool robot::FanucM20iA::standCoord(const int num) const
{
	std::ostringstream tmpCoord;
	tmpCoord << num;
	return robot::FanucM20iA::sendany(tmpCoord.str().c_str());
}

bool robot::FanucM20iA::finish() const
{
	if (shutdown(_mySock, SD_SEND) == SOCKET_ERROR)
	{
		std::cout << "Shutdown failed with error: " << WSAGetLastError() << std::endl;
		closesocket(_mySock);
		closesocket(_sockrecv);
		WSACleanup();
		return false;
	}
	WSACleanup();
	return true;
}

void robot::FanucM20iA::createCartesianCoordinates(Lopata& obj)
{
	obj._cartesianCoordinates[0] = homepos.at(0) + obj._localYCoordinatesOfLopata * MULT._x;
	obj._cartesianCoordinates[1] = homepos.at(1) + obj._localXCoordinatesOfLopata * MULT._y;
	obj._cartesianCoordinates[2] = minValueOf._z + static_cast<int>(obj._altitude) * MULT._z;
}

void robot::FanucM20iA::checkCoordsLimits(Lopata& obj)
{
	if (obj._cartesianCoordinates[0] > maxValueOf._x)
		obj._cartesianCoordinates[0] = maxValueOf._x;
	if (obj._cartesianCoordinates[0] < minValueOf._x)
		obj._cartesianCoordinates[0] = minValueOf._x;

	if (obj._cartesianCoordinates[1] > maxValueOf._y)
		obj._cartesianCoordinates[1] = maxValueOf._y;
	if (obj._cartesianCoordinates[1] < minValueOf._y)
		obj._cartesianCoordinates[1] = minValueOf._y;

	if (obj._cartesianCoordinates[2] > maxValueOf._z)
		obj._cartesianCoordinates[2] = maxValueOf._z;
	if (obj._cartesianCoordinates[2] < minValueOf._z)
		obj._cartesianCoordinates[2] = minValueOf._z;
}

void robot::FanucM20iA::sendCoordinates(Lopata& obj)
{

    movingAverageFilter(obj);

	robot::FanucM20iA::createCartesianCoordinates(obj);
	robot::FanucM20iA::checkCoordsLimits(obj);

	const char* sendbuf = robot::FanucM20iA::createStringToSend(obj);

    system("cls");
    std::cout << sendbuf << '\n';

	if (this->canSendCoordinates(obj))
	{
        if (this->_firstPoint)
        {
            this->dontFirst();
        }
        else
        {
            this->sendany(sendbuf);
        }
    }
}

void robot::FanucM20iA::thresholdFilterAngles(Lopata& obj, std::ostringstream& tmpBuf)
{
	const int minDeltaDegrees = 1;
	for (int i = 0; i < 3; ++i)
	{
		if (abs(obj._oldEulerAngles[i] - obj._eulerAngles[i]) > minDeltaDegrees)
		{
			tmpBuf << static_cast<int>(obj._eulerAngles[i] * MULT._degrees) << ' ';
		}
		else
		{
			tmpBuf << static_cast<int>(obj._oldEulerAngles[i] * MULT._degrees) << ' ';
		}
	}
}

void robot::FanucM20iA::thresholdFilterCartesianCoordinates(Lopata& obj, std::ostringstream& tmpBuf)
{
	const int minDeltaMillimeters = 10;
	for (int i = 0; i < 3; ++i)
	{
		if (abs(obj._oldCartesianCoordinates[i] - obj._cartesianCoordinates[i]) > minDeltaMillimeters)
		{
			// std::cout << "YES!!!" << std::endl;
			tmpBuf << obj._cartesianCoordinates[i] << ' ';
		}
		else
		{
			// std::cout << "NO!!!" << std::endl;
			tmpBuf << obj._oldCartesianCoordinates[i] << ' ';
		}
	}
}

void robot::FanucM20iA::movingAverageFilter(Lopata& obj)
{
    if (obj._oldCartesianForAverage.size() == 5)
    {
        std::array<int, 3> cartesianTmp = { {0, 0, 0} };

        for (size_t i = 0; i < obj._oldCartesianForAverage.size(); ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                if (i != obj._oldCartesianForAverage.size() - 1)
                {
                    cartesianTmp.at(j) += obj._oldCartesianForAverage.at(i).at(j) / obj._oldCartesianForAverage.size();
                }
                else
                {
                    cartesianTmp.at(j) += obj._cartesianCoordinates.at(j) / obj._oldCartesianForAverage.size();
                }
            }
        }
        
        obj._cartesianCoordinates = cartesianTmp;
        obj._oldCartesianForAverage.pop_front();
        obj._oldCartesianForAverage.emplace_back(cartesianTmp);
    }
    else
    {
        obj._oldCartesianForAverage.emplace_back(obj._cartesianCoordinates);
    }
}

const char* robot::FanucM20iA::createStringToSend(Lopata& obj)
{
	std::ostringstream tmpBuf;
	const int tmpSeg = 10, tmpTypeOfMoving = 2, tmpControl = 0;

	thresholdFilterCartesianCoordinates(obj, tmpBuf);
	thresholdFilterAngles(obj, tmpBuf);

	tmpBuf << tmpSeg << ' ' << tmpTypeOfMoving << ' ' << tmpControl;

	for (int i = 0; i < 3; ++i)
	{
		obj._oldCartesianCoordinates[i] = obj._cartesianCoordinates[i];
		obj._oldEulerAngles[i] = obj._eulerAngles[i];
	}

	return _strdup(tmpBuf.str().c_str());
}
