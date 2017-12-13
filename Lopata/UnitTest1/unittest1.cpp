#include "UnitTest1.h"

void unitTestsLopata::CheckCalcFunctions::testMethod1()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(38.0132);
	const double expectedAnswer = 600;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 0.00001, L"!ERROR!");
}

void unitTestsLopata::CheckCalcFunctions::testMethod2()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(253.16);
	const double expectedAnswer = 90;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 0.00001, L"!ERROR!");
}

void unitTestsLopata::CheckCalcFunctions::testMethod3()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(104.043);
	const double expectedAnswer = 220;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 0.00001, L"!ERROR!");
}