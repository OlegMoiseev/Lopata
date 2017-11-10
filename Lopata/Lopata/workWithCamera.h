#ifndef WORK_WITH_CAMERA
#define WORK_WITH_CAMERA

/**
 * \brief Main function - detect lopata, calculate coordinates and can send it to the robot
 * \param[in] connection If true - coordinates will be send to the robot, else - no
 */
void detectControlHandlePosition(const bool &connection);

/**
 * \brief Information function - display current version of OpenCV
 */
void cvVersionOnScreen();

#endif
