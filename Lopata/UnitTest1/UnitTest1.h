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

		TEST_METHOD(testMethod1);
		TEST_METHOD(testMethod2);
		TEST_METHOD(testMethod3);

	};
}