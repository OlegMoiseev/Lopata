#include "workWithCamera.h"

#pragma comment (lib, "opencv_world331.lib")

int main()
{
	const bool connection = false;
	cvVersionOnScreen();
	detectControlHandlePosition(connection);
	return 0;
}
