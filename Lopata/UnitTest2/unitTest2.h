#pragma once
#include <CppUnitTest.h>
#include "../Lopata/LopataObject.h"
// ReSharper disable CppUnusedIncludeDirective
#include "../Lopata/MathForIMU.h"
#include "../Lopata/LopataObject.cpp"
#include "../Lopata/MathForIMU.cpp"
// ReSharper restore CppUnusedIncludeDirective

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace unitTestsLopataHeight
{
	TEST_CLASS(CheckScaleFunctions)
	{
	private:
		Lopata _lopataToTest;
	public:

		TEST_METHOD(testMid0);
		TEST_METHOD(testMid1);
		TEST_METHOD(testMid2);

	};
}