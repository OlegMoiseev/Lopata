#pragma once
#include <CppUnitTest.h>
#include "../Lopata/LopataObject.h"
// ReSharper disable CppUnusedIncludeDirective
#include "../Lopata/MathForIMU.h"
#include "../Lopata/LopataObject.cpp"
#include "../Lopata/MathForIMU.cpp"
// ReSharper restore CppUnusedIncludeDirective

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace unitTestsLopataScale
{
	TEST_CLASS(CheckScaleFunctions)
	{
	private:
		Lopata _lopataToTest;
	public:

		TEST_METHOD(testScale0);
		TEST_METHOD(testScale1);
		TEST_METHOD(testScale2);
		TEST_METHOD(testScale3);

		TEST_METHOD(testScale4);
		TEST_METHOD(testScale5);
		TEST_METHOD(testScale6);
		TEST_METHOD(testScale7);
		TEST_METHOD(testScale8);
	};
}