#include "UnitTest1.h"

void unitTestsLopataHeight::CheckCalcFunctions::testMinBound()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(38.0132);
	const double expectedAnswer = 600;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 1e-5, L"!ERROR!");
}

void unitTestsLopataHeight::CheckCalcFunctions::testMaxBound()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(253.16);
	const double expectedAnswer = 90;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 1e-5, L"!ERROR!");
}

void unitTestsLopataHeight::CheckCalcFunctions::testMid0()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(104.043);
	const double expectedAnswer = 220;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 1e-5, L"!ERROR!");
}

void unitTestsLopataHeight::CheckCalcFunctions::testMid1()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(40.0125);
	const double expectedAnswer = 570;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 1e-5, L"!ERROR!");
}

void unitTestsLopataHeight::CheckCalcFunctions::testMid2()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(39.0128);
	const double expectedAnswer = 590;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 1e-5, L"!ERROR!");
}

void unitTestsLopataHeight::CheckCalcFunctions::testNonTableMid0()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(107.06);
	const double expectedAnswer = 214.002;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 1e-2, L"!ERROR!");
}

void unitTestsLopataHeight::CheckCalcFunctions::testLessThanTableHeight0()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(270.55);
	const double expectedAnswer = 82.76;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 1e-2, L"!ERROR!");
}

void unitTestsLopataHeight::CheckCalcFunctions::testMoreThanTableHeight0()
{
	const double tmpAnswer = _lopataToTest.calculatePointOnGraph(35.72);
	const double expectedAnswer = 622.941;
	Assert::AreEqual(expectedAnswer, tmpAnswer, 1e-2, L"!ERROR!");
}