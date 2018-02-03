#include "DriftTest.h"
#include "Wait.h"
#include "Circle.h"

DriftTest::DriftTest(int n) : CommandGroup("DriftTest"), n(n){
	for (int i = 0; i < n; i++) {
		for (int j = 0; j <= i; j++) {
			AddSequential(new Circle(0.8));
		}
		AddSequential(new Wait(10));
	}
}
