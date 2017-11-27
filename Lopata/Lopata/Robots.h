#ifndef ROBOTS
#define ROBOTS

#include <winsock2.h>
#include <string>
#include <array>

#pragma comment(lib, "ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

/**
 * \brief Struct which accumulate all data about object
 */
struct Lopata;

/**
 * \brief Namespace of all robots
 */
namespace robot {
	/**
	 * \brief Class of robot with connecting and sending coordinates
	 */
	class FanucM20iA {
		/**
		 * \brief Socket to sending data to the robot
		 */
		SOCKET _mySock;
		/**
		* \brief Socket to recieving data from the robot
		*/
		SOCKET _sockrecv;
		/**
		 * \brief Port to sending socket
		 */
		const int _ports = 59002;
		/**
		* \brief Port to recieving socket
		*/
		const int _portr = 59003;
		/**
		 * \brief Robot's IP
		 */
		//std::string _tmpAddrString = "192.168.0.21";
		
		std::string _tmpAddrString = "127.0.0.1";

		/**
		 * \brief Buffer to initialize WinSock
		 */
		char _buff[1024];

		/**
		 * \brief Flag of allowing connection to the robot
		 */
		bool _connect;
		/**
		* \brief Flag of allowing send coordinates to the robot
		*/
		bool _sendCoordinates;

		/**
		 * \brief Function of the initializing connection to the robot
		 * \return True if initialization successfull
		 */
		bool initSockets();
		/**
		 * \brief Function of the connecting to the sockets
		 * \return True if connection successfull
		 */
		bool conSocket() const;
		/**
		 * \brief Start working with WinSock
		 * \return True if start successfull
		 */
		bool start();

		/**
		 * \brief Creating string with coordinates to send them
		 * \param[in] obj Object which coordinates will be sent
		 * \return Array of chars - coordinates in string
		 */
		static const char* robot::FanucM20iA::createStringToSend(Lopata &obj);

		/**
		 * \brief Create cartesian coordinates of the object in the robot's coordinate system
		 * \param[in] obj Object which coordinates will be transformed
		 */
		static void createCartesianCoordinates(Lopata& obj);

		/**
		 * \brief Verification of getting into the permitted area
		 * \param[in] obj Object which coordinates need to check
		 */
		static void checkCoordsLimits(Lopata& obj);

		/**
		 * \brief Checking angles for change and send it to the string
		 * \param[in] obj Object which angles need to check
		 * \param[in] tmpBuf String to write to
		 */
		static void thresholdFilterAngles(Lopata& obj, std::ostringstream &tmpBuf);
		/**
		* \brief Checking cartesian coordinates for change and send it to the string
		* \param[in] obj Object which coordinates need to check
		* \param[in] tmpBuf String to write to
		*/
		static void thresholdFilterCartesianCoordinates(Lopata& obj, std::ostringstream &tmpBuf);

	public:
		/**
		 * \brief Constructor with initializing and connecting to the Robot
		 * \param[in] c If true connect to the robot 
		 */
		explicit FanucM20iA(const int c);
		/**
		 * \brief Get info about connection
		 */
		void info() const;
		/**
		 * \brief Function to stand robot's coordinate system
		 * \param[in] num Number of coordinate system (2 - World frame, 0 - Joint frame)
		 * \return True if successfull
		 */
		bool standCoord(const int num) const;
		/**
		 * \brief Stop work with robot
		 * \return True if stopped successfull
		 */
		bool finish() const;
		/**
		 * \brief Send any array of chars to the robot
		 * \param[in] str String to send
		 * \return True if sent successfully
		 */
		bool sendany(const char* str) const;

		/**
		 * \brief Allowing send data to the robot
		 */
		void allowSend();
		/**
		* \brief Forbidding send data to the robot
		*/
		void forbidSend();
		/**
		 * \brief Checking possibility of sending data to the robot
		 * \return True if send allowed, else - False
		 */
		bool canSendCoordinates() const;

		/**
		 * \brief Function of sending coordinates to the robot with all checks
		 * \param obj Lopata, which coordinates need to send
		 */
		void sendCoordinates(Lopata& obj) const;
	};
}
#endif // !_ROBOTS
