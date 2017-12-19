#include "UnitTest2.h"


void unitTestsLopataScale::CheckScaleFunctions::testScale0()
{
	// 640x480
	_lopataToTest._altitude = 300.;
	_lopataToTest._localXCoordinatesOfLopata = 320;
	_lopataToTest._localYCoordinatesOfLopata = 240;

	_lopataToTest.scalingCoordinates();

	const double tmpAnswerX = _lopataToTest._xBound;
	const double tmpAnswerY = _lopataToTest._yBound;
	const double expectedAnswerX = 640.;
	const double expectedAnswerY = 480.;

	const double tmpAnswerCoordX = _lopataToTest._localXCoordinatesOfLopata;
	const double tmpAnswerCoordY = _lopataToTest._localYCoordinatesOfLopata;
	const double expectedAnswerCoordX = 0.;
	const double expectedAnswerCoordY = 0.;

	Assert::AreEqual(expectedAnswerX, tmpAnswerX, 1e-5, L"!ERROR_X!");
	Assert::AreEqual(expectedAnswerY, tmpAnswerY, 1e-5, L"!ERROR_Y!");

	Assert::AreEqual(expectedAnswerCoordX, tmpAnswerCoordX, 1e-5, L"!ERROR_COORD_X!");
	Assert::AreEqual(expectedAnswerCoordY, tmpAnswerCoordY, 1e-5, L"!ERROR_COORD_Y!");

}

void unitTestsLopataScale::CheckScaleFunctions::testScale1()
{
	// 640x480
	_lopataToTest._altitude = 600.;
	_lopataToTest._localXCoordinatesOfLopata = 320;
	_lopataToTest._localYCoordinatesOfLopata = 240;

	_lopataToTest.scalingCoordinates();

	const double tmpAnswerX = _lopataToTest._xBound;
	const double tmpAnswerY = _lopataToTest._yBound;
	const double expectedAnswerX = 1280.;
	const double expectedAnswerY = 960.;

	const double tmpAnswerCoordX = _lopataToTest._localXCoordinatesOfLopata;
	const double tmpAnswerCoordY = _lopataToTest._localYCoordinatesOfLopata;
	const double expectedAnswerCoordX = 0.;
	const double expectedAnswerCoordY = 0.;

	Assert::AreEqual(expectedAnswerX, tmpAnswerX, 1e-5, L"!ERROR_X!");
	Assert::AreEqual(expectedAnswerY, tmpAnswerY, 1e-5, L"!ERROR_Y!");

	Assert::AreEqual(expectedAnswerCoordX, tmpAnswerCoordX, 1e-5, L"!ERROR_COORD_X!");
	Assert::AreEqual(expectedAnswerCoordY, tmpAnswerCoordY, 1e-5, L"!ERROR_COORD_Y!");

}

void unitTestsLopataScale::CheckScaleFunctions::testScale2()
{
	// 640x480
	_lopataToTest._altitude = 300.;
	_lopataToTest._localXCoordinatesOfLopata = 340;
	_lopataToTest._localYCoordinatesOfLopata = 260;

	_lopataToTest.scalingCoordinates();

	const double tmpAnswerX = _lopataToTest._xBound;
	const double tmpAnswerY = _lopataToTest._yBound;
	const double expectedAnswerX = 640.;
	const double expectedAnswerY = 480.;

	const double tmpAnswerCoordX = _lopataToTest._localXCoordinatesOfLopata;
	const double tmpAnswerCoordY = _lopataToTest._localYCoordinatesOfLopata;
	const double expectedAnswerCoordX = 20.;
	const double expectedAnswerCoordY = 20.;

	Assert::AreEqual(expectedAnswerX, tmpAnswerX, 1e-5, L"!ERROR_X!");
	Assert::AreEqual(expectedAnswerY, tmpAnswerY, 1e-5, L"!ERROR_Y!");

	Assert::AreEqual(expectedAnswerCoordX, tmpAnswerCoordX, 1e-5, L"!ERROR_COORD_X!");
	Assert::AreEqual(expectedAnswerCoordY, tmpAnswerCoordY, 1e-5, L"!ERROR_COORD_Y!");

}

void unitTestsLopataScale::CheckScaleFunctions::testScale3()
{
	// 640x480
	_lopataToTest._altitude = 600.;
	_lopataToTest._localXCoordinatesOfLopata = 330;
	_lopataToTest._localYCoordinatesOfLopata = 250;

	_lopataToTest.scalingCoordinates();

	const double tmpAnswerX = _lopataToTest._xBound;
	const double tmpAnswerY = _lopataToTest._yBound;
	const double expectedAnswerX = 1280.;
	const double expectedAnswerY = 960.;

	const double tmpAnswerCoordX = _lopataToTest._localXCoordinatesOfLopata;
	const double tmpAnswerCoordY = _lopataToTest._localYCoordinatesOfLopata;
	const double expectedAnswerCoordX = 20.;
	const double expectedAnswerCoordY = 20.;

	Assert::AreEqual(expectedAnswerX, tmpAnswerX, 1e-5, L"!ERROR_X!");
	Assert::AreEqual(expectedAnswerY, tmpAnswerY, 1e-5, L"!ERROR_Y!");

	Assert::AreEqual(expectedAnswerCoordX, tmpAnswerCoordX, 1e-5, L"!ERROR_COORD_X!");
	Assert::AreEqual(expectedAnswerCoordY, tmpAnswerCoordY, 1e-5, L"!ERROR_COORD_Y!");

}

void unitTestsLopataScale::CheckScaleFunctions::testScale4()
{
	// 640x480
	_lopataToTest._altitude = 100.;
	_lopataToTest._localXCoordinatesOfLopata = 320;
	_lopataToTest._localYCoordinatesOfLopata = 240;

	_lopataToTest.scalingCoordinates();

	const double tmpAnswerX = _lopataToTest._xBound;
	const double tmpAnswerY = _lopataToTest._yBound;
	const double expectedAnswerX = 213.33;
	const double expectedAnswerY = 160.;

	const double tmpAnswerCoordX = _lopataToTest._localXCoordinatesOfLopata;
	const double tmpAnswerCoordY = _lopataToTest._localYCoordinatesOfLopata;
	const double expectedAnswerCoordX = 0.;
	const double expectedAnswerCoordY = 0.;

	Assert::AreEqual(expectedAnswerX, tmpAnswerX, 1e-2, L"!ERROR_X!");
	Assert::AreEqual(expectedAnswerY, tmpAnswerY, 1e-2, L"!ERROR_Y!");

	Assert::AreEqual(expectedAnswerCoordX, tmpAnswerCoordX, 1e-2, L"!ERROR_COORD_X!");
	Assert::AreEqual(expectedAnswerCoordY, tmpAnswerCoordY, 1e-2, L"!ERROR_COORD_Y!");
}

void unitTestsLopataScale::CheckScaleFunctions::testScale5()
{
	// 640x480
	_lopataToTest._altitude = 600.;
	_lopataToTest._localXCoordinatesOfLopata = 370;
	_lopataToTest._localYCoordinatesOfLopata = 290;

	_lopataToTest.scalingCoordinates();

	const double tmpAnswerX = _lopataToTest._xBound;
	const double tmpAnswerY = _lopataToTest._yBound;
	const double expectedAnswerX = 1280.;
	const double expectedAnswerY = 960.;

	const double tmpAnswerCoordX = _lopataToTest._localXCoordinatesOfLopata;
	const double tmpAnswerCoordY = _lopataToTest._localYCoordinatesOfLopata;
	const double expectedAnswerCoordX = 100.;
	const double expectedAnswerCoordY = 100.;

	Assert::AreEqual(expectedAnswerX, tmpAnswerX, 1e-2, L"!ERROR_X!");
	Assert::AreEqual(expectedAnswerY, tmpAnswerY, 1e-2, L"!ERROR_Y!");

	Assert::AreEqual(expectedAnswerCoordX, tmpAnswerCoordX, 1e-2, L"!ERROR_COORD_X!");
	Assert::AreEqual(expectedAnswerCoordY, tmpAnswerCoordY, 1e-2, L"!ERROR_COORD_Y!");
}

void unitTestsLopataScale::CheckScaleFunctions::testScale6()
{
	// 640x480
	_lopataToTest._altitude = 600.;
	_lopataToTest._localXCoordinatesOfLopata = 370;
	_lopataToTest._localYCoordinatesOfLopata = 240;

	_lopataToTest.scalingCoordinates();

	const double tmpAnswerX = _lopataToTest._xBound;
	const double tmpAnswerY = _lopataToTest._yBound;
	const double expectedAnswerX = 1280.;
	const double expectedAnswerY = 960.;

	const double tmpAnswerCoordX = _lopataToTest._localXCoordinatesOfLopata;
	const double tmpAnswerCoordY = _lopataToTest._localYCoordinatesOfLopata;
	const double expectedAnswerCoordX = 100.;
	const double expectedAnswerCoordY = 0.;

	Assert::AreEqual(expectedAnswerX, tmpAnswerX, 1e-2, L"!ERROR_X!");
	Assert::AreEqual(expectedAnswerY, tmpAnswerY, 1e-2, L"!ERROR_Y!");

	Assert::AreEqual(expectedAnswerCoordX, tmpAnswerCoordX, 1e-2, L"!ERROR_COORD_X!");
	Assert::AreEqual(expectedAnswerCoordY, tmpAnswerCoordY, 1e-2, L"!ERROR_COORD_Y!");
}

void unitTestsLopataScale::CheckScaleFunctions::testScale7()
{
	// 640x480
	_lopataToTest._altitude = 150.;
	_lopataToTest._localXCoordinatesOfLopata = 370;
	_lopataToTest._localYCoordinatesOfLopata = 240;

	_lopataToTest.scalingCoordinates();

	const double tmpAnswerX = _lopataToTest._xBound;
	const double tmpAnswerY = _lopataToTest._yBound;
	const double expectedAnswerX = 320.;
	const double expectedAnswerY = 240.;

	const double tmpAnswerCoordX = _lopataToTest._localXCoordinatesOfLopata;
	const double tmpAnswerCoordY = _lopataToTest._localYCoordinatesOfLopata;
	const double expectedAnswerCoordX = 25.;
	const double expectedAnswerCoordY = 0.;

	Assert::AreEqual(expectedAnswerX, tmpAnswerX, 1e-2, L"!ERROR_X!");
	Assert::AreEqual(expectedAnswerY, tmpAnswerY, 1e-2, L"!ERROR_Y!");

	Assert::AreEqual(expectedAnswerCoordX, tmpAnswerCoordX, 1e-2, L"!ERROR_COORD_X!");
	Assert::AreEqual(expectedAnswerCoordY, tmpAnswerCoordY, 1e-2, L"!ERROR_COORD_Y!");
}

void unitTestsLopataScale::CheckScaleFunctions::testScale8()
{
	// 640x480
	_lopataToTest._altitude = 1000.;
	_lopataToTest._localXCoordinatesOfLopata = 0;
	_lopataToTest._localYCoordinatesOfLopata = 0;

	_lopataToTest.scalingCoordinates();

	const double tmpAnswerX = _lopataToTest._xBound;
	const double tmpAnswerY = _lopataToTest._yBound;
	const double expectedAnswerX = 2133.33;
	const double expectedAnswerY = 1600.;

	const double tmpAnswerCoordX = _lopataToTest._localXCoordinatesOfLopata;
	const double tmpAnswerCoordY = _lopataToTest._localYCoordinatesOfLopata;
	const double expectedAnswerCoordX = -1066.67;
	const double expectedAnswerCoordY = -800.;

	Assert::AreEqual(expectedAnswerX, tmpAnswerX, 1e-2, L"!ERROR_X!");
	Assert::AreEqual(expectedAnswerY, tmpAnswerY, 1e-2, L"!ERROR_Y!");

	Assert::AreEqual(expectedAnswerCoordX, tmpAnswerCoordX, 1e-2, L"!ERROR_COORD_X!");
	Assert::AreEqual(expectedAnswerCoordY, tmpAnswerCoordY, 1e-2, L"!ERROR_COORD_Y!");
}