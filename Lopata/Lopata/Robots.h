#ifndef ROBOTS
#define ROBOTS

#include <winsock2.h>
#include <string>
#include <iostream>
#include <array>

#pragma comment(lib, "ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

struct Lopata;

namespace robot {
	class FanucM20iA {
		SOCKET _mySock;
		SOCKET _sockrecv;
		const int _ports = 59002;
		const int _portr = 59003;
		std::string _tmpAddrString = "192.168.0.21";
		char _buff[1024];

		bool _connect;
		bool _sendCoordinates;

		bool initSockets();
		bool conSocket() const;
		bool start();

		static const char* robot::FanucM20iA::createStringToSend(Lopata &obj);

		static void createCartesianCoordinates(Lopata& obj);

		static void checkCoordsLimits(Lopata& obj);

		static void thresholdFilterDegrees(Lopata& obj, std::ostringstream &tmpBuf);
		static void thresholdFilterCartesianCoordinates(Lopata& obj, std::ostringstream &tmpBuf);

	public:
		explicit FanucM20iA(const int c);
		void info() const;
		bool standCoord(const int num) const;
		bool finish() const;
		bool sendany(const char* str) const;

		void allowSend();
		void forbidSend();
		bool canSendCoordinates() const;

		void sendCoordinates(Lopata& obj) const;
	};
}
#endif // !_ROBOTS
