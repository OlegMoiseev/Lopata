#pragma once
#include <CppUnitTest.h>
#include "../Lopata/LopataObject.h"
// ReSharper disable CppUnusedIncludeDirective
#include "../Lopata/MathForIMU.h"
#include "../Lopata/LopataObject.cpp"
#include "../Lopata/MathForIMU.cpp"
// ReSharper restore CppUnusedIncludeDirective

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace unitTestsLopata
{
	TEST_CLASS(CheckCalcFunctions)
	{
	private:

		Lopata _lopataToTest;
	public:

		TEST_METHOD(testMinBound);
		TEST_METHOD(testMaxBound);
		TEST_METHOD(testMid0);
		TEST_METHOD(testMid1);
		TEST_METHOD(testMid2);
		TEST_METHOD(testNonTableMid0);
		TEST_METHOD(testLessThanTableHeight0);
		TEST_METHOD(testMoreThanTableHeight0);

	};
}