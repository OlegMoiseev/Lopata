#include "UnitTest2.h"


void unitTestsLopataHeight::CheckScaleFunctions::testMid0()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(104.043);
	const double expectedAnswer = 220;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 1e-5, L"!ERROR!");
}

void unitTestsLopataHeight::CheckScaleFunctions::testMid1()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(40.0125);
	const double expectedAnswer = 570;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 1e-5, L"!ERROR!");
}

void unitTestsLopataHeight::CheckScaleFunctions::testMid2()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(39.0128);
	const double expectedAnswer = 590;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 1e-5, L"!ERROR!");
}
