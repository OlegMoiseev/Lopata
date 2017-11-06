#include "workWithCamera.h"

#pragma comment (lib, "opencv_world330.lib")

int main()
{
	const bool connection = false;
	cvVersionOnScreen();
	detectControlHandlePosition(connection);
}
