#include <memory>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/base/ScopedState.h"
#include "ompl/geometric/SimpleSetup.h"

#include "OMPL.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

static std::shared_ptr<ob::RealVectorStateSpace> stateSpace = std::make_shared<ob::RealVectorStateSpace>();;
static std::unique_ptr<og::SimpleSetup> simpleSetup = std::make_unique<og::SimpleSetup>(stateSpace);;
// Managed Validity Checker should return SUCCESS when a state is valid, FAILURE otherwise
static ValidityChecker managedValidityChecker = nullptr;

static bool isSetup()
{
	return stateSpace != nullptr && simpleSetup != nullptr;
}

static bool stateValidityChecker(const ob::State *state)
{
	if (managedValidityChecker == nullptr)
		return false;

	double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
	int count = DimensionCount();
	return managedValidityChecker(values, count);
}

extern "C"
{
	bool Reset()
	{
		// Only SimpleSetup and the StateSpace itself share ownership
		// of the StateSpace instance at any given time.
		if (stateSpace.use_count() > 2)
			return false;

		try
		{
			stateSpace = std::make_shared<ob::RealVectorStateSpace>();
			simpleSetup = std::make_unique<og::SimpleSetup>(stateSpace);
			simpleSetup->setStateValidityChecker(stateValidityChecker);

			managedValidityChecker = nullptr;

			return true;
		}
		catch (std::exception &)
		{
			return false;
		}
	}
	
	bool AddDimension(double min, double max)
	{
		try
		{
			stateSpace->addDimension(min, max);
			return true;
		}
		catch (std::exception &)
		{
			return false;
		}
	}

	int DimensionCount()
	{
		if (!isSetup())
			return -1;
	
		try
		{
			return stateSpace->getDimension();
		}
		catch (std::exception &)
		{
			return -1;
		}
	}

	bool SetValidityChecker(ValidityChecker checker)
	{
		if (!isSetup())
			return false;

		managedValidityChecker = checker;
		return true;
	}

	bool HasSetValidityChecker()
	{
		return managedValidityChecker != nullptr;
	}

	bool Solve(double *initial, double *goal, int dimensions, double time, int *steps)
	{
		*steps = -1;

		if (!isSetup())
			return false;

		try
		{
			ob::ScopedState<ob::RealVectorStateSpace> initialState(stateSpace);
			ob::ScopedState<ob::RealVectorStateSpace> goalState(stateSpace);

			for (int i = 0; i < dimensions; i++)
			{
				initialState[i] = initial[i];
				goalState[i] = goal[i];
			}

			simpleSetup->setStartAndGoalStates(initialState, goalState);

			bool solved = simpleSetup->solve(time);

			if (solved)
			{
				simpleSetup->simplifySolution();
				*steps = (int) (simpleSetup->getSolutionPath().getStateCount());

				return true;
			}
			else
				return false;
		}
		catch (std::exception &)
		{
			return false;
		}
	}

	/* UNSAFE*/
	bool GetSolution(int steps, int dimensions, double *solution)
	{
		if (!isSetup() || !simpleSetup->getLastPlannerStatus())
			return false;

		int solutionLength = simpleSetup->getSolutionPath().getStateCount();
		if (solutionLength != steps)
			return false;


		int dimensionCount = DimensionCount();
		if (dimensionCount < 0 || dimensionCount != dimensions)
			return false;

		const auto &sol = simpleSetup->getSolutionPath().getStates();

		for (int i = 0; i < steps; i++)
		{
			const double *values = sol[i]->as<ob::RealVectorStateSpace::StateType>()->values;

			for (int j = 0; j < dimensions; j++)
			{
				solution[i * dimensions + j] = values[j];
			}
		}

		return true;
	}
}