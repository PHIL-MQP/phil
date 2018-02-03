#include "DriftTest.h"
#include "Wait.h"
#include "Circle.h"

DriftTest::DriftTest(int n) : CommandGroup("DriftTest"), n(n){
	for (int i = 0; i < n; i++) {
		AddSequential(new Circle(1));
		AddSequential(new Wait(10));
	}
}
