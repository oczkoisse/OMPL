using System;

namespace OMPLUnity
{
    public static class OMPL
    {
        public static void Reset()
        {
            bool result = NativeMethods.Reset();
            if (!result)
            {
                throw new OMPLException("Failed to reset");
            }
        }

        public static bool TryReset() => NativeMethods.Reset();
        

        public static void AddDimension(double min, double max)
        {
            bool result = NativeMethods.AddDimension(min, max);
            if (!result)
                throw new OMPLException($"Failed to add dimension with bounds ({min}, {max})");
        }

        public static bool TryAddDimension(double min, double max) => NativeMethods.AddDimension(min, max);

        public static int DimensionCount
        {
            get
            {
                int result = NativeMethods.DimensionCount();
                if (result < 0)
                    throw new InvalidOperationException("Failed to get dimension count");
                else
                    return result;
            }
        }

        public static bool TryGetDimensionCount(out int dimensions)
        {
            dimensions = NativeMethods.DimensionCount();
            return dimensions >= 0;
        }

        public static void SetValidityChecker(ValidityChecker checker) => NativeMethods.SetValidityChecker(checker);

        public static bool HasSetValidityChecker() => NativeMethods.HasSetValidityChecker();

        public static double[,] Solve(double[] initial, double[] goal, double limit)
        {
            if (limit <= 0.0)
                throw new ArgumentOutOfRangeException("limit must be a positive floating-point value");
            if (initial == null)
                throw new ArgumentNullException("initial is null");
            if (goal == null)
                throw new ArgumentNullException("goal is null");

            if (initial.Length != goal.Length)
                throw new ArgumentException("initial and goal must have same number of dimensions");

            int dimensions = DimensionCount;

            if (initial.Length != DimensionCount)
                throw new ArgumentException($"initial does not have same length ({initial.Length}) as number of dimensions ({dimensions})");

            if (goal.Length != DimensionCount)
                throw new ArgumentException($"goal does not have same length ({goal.Length}) as number of dimensions ({dimensions})");

            if (!NativeMethods.Solve(initial, goal, dimensions, limit, out int steps))
                throw new OMPLException("Could not create a solution");

            double[] solution = new double[steps * dimensions];

            if (!NativeMethods.GetSolution(steps, dimensions, solution))
                throw new OMPLException("Could not retrieve the solution");

            double[,] solution2d = new double[steps, dimensions];

            for (int i = 0; i < solution2d.GetLength(0); i++)
            {
                int d = solution2d.GetLength(1);
                for (int j = 0; j < solution2d.GetLength(1); j++)
                    solution2d[i, j] = solution[i * d + j];
            }

            return solution2d;
        }

        public static bool TrySolve(double[] initial, double[] goal, double limit, out double[,] solution)
        {
            try
            {
                solution = Solve(initial, goal, limit);
                return true;
            }
            catch(Exception)
            {
                solution = null;
                return false;
            }
        }
    }
}
