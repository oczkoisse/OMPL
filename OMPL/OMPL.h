#pragma once

#define EXPORT __declspec(dllexport)

typedef int(*ValidityChecker)(double *, int);

extern "C"
{
	EXPORT bool Reset();

	EXPORT bool AddDimension(double min, double max);
	EXPORT int DimensionCount();
	
	EXPORT bool SetValidityChecker(ValidityChecker checker);
	EXPORT bool HasSetValidityChecker();

	EXPORT bool Solve(double *initial, double *goal, int length, double limit, int *steps);

	EXPORT bool GetSolution(int steps, int dimensions, double *solution);
}