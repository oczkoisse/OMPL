using System;

namespace OMPLUnity
{
    /// <summary>
    /// Managed interface to the native OMPL layer with methods that generate exceptions along with exception safe methods.
    /// </summary>
    public static class OMPL
    {
        // Necessary to allow passing of lambda functions as the delegate object
        // Otherwise, they'll be garbage collected soon after since no reference
        // to them may exist on the managed side
        private static ValidityChecker checker;

        /// <summary>
        /// Resets the state space to <c>0</c> dimensions so that 
        /// <see cref="NativeMethods.DimensionCount()"/> returns <c>0</c>.
        /// The <see cref="ValidityChecker"/> reference is also reset so that
        /// <see cref="NativeMethods.HasSetValidityChecker()"/> returns <c>null</c>.
        /// </summary>
        /// <exception cref="OMPLException">Thrown when the attempt to reset fails.</exception>
        public static void Reset()
        {
            if (!TryReset())
                throw new OMPLException("Unable to reset");
        }

        /// <summary>
        /// Resets the state space to <c>0</c> dimensions so that 
        /// <see cref="NativeMethods.DimensionCount()"/> returns <c>0</c>.
        /// The <see cref="ValidityChecker"/> reference is also reset so that
        /// <see cref="NativeMethods.HasSetValidityChecker()"/> returns <c>null</c>.
        /// </summary>
        /// <returns><c>true</c> if the call was successful, <c>false</c> otherwise</returns>
        public static bool TryReset()
        {
            bool result = NativeMethods.Reset();
            if (result)
                checker = null;
            return result;
        }

        /// <summary>
        /// Adds a new dimension with bounds [<paramref name="min"/>, <paramref name="max"/>] to the state space.
        /// </summary>
        /// <param name="min">Minimum value for the dimension</param>
        /// <param name="max">Maximum value for the dimension</param>
        /// <exception cref="OMPLException">Thrown when the attempt to add dimension fails.</exception>
        public static void AddDimension(double min, double max)
        {
            bool result = NativeMethods.AddDimension(min, max);
            if (!result)
                throw new OMPLException($"Failed to add dimension with bounds ({min}, {max})");
        }

        /// <summary>
        /// Adds a new dimension with bounds [<paramref name="min"/>, <paramref name="max"/>] to the state space.
        /// </summary>
        /// <param name="min">Minimum value for the dimension</param>
        /// <param name="max">Maximum value for the dimension</param>
        /// <returns><c>true</c> if the call was successful, <c>false</c> otherwise</returns>
        public static bool TryAddDimension(double min, double max) => NativeMethods.AddDimension(min, max);

        /// <summary>
        /// Get the number of dimensions in the state space.
        /// </summary>
        /// <exception cref="InvalidOperationException">Thrown when the attempt to get dimension count fails.</exception>
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

        /// <summary>
        /// Get the number of dimensions in the state space.
        /// </summary>
        /// <returns>The number of dimensions. A value of <c>-1</c> is returned if the call is unsuccessful.</returns>
        public static bool TryGetDimensionCount(out int dimensions)
        {
            dimensions = NativeMethods.DimensionCount();
            return dimensions >= 0;
        }

        /// <summary>
        /// Sets the validity checker for the state space.
        /// </summary>
        /// <param name="checker">A <see cref="ValidityChecker"/> instance</param>
        /// <returns><c>true</c> if the call was successful, <c>false</c> otherwise</returns>
        public static bool TrySetValidityChecker(ValidityChecker checker)
        {
            bool result = NativeMethods.SetValidityChecker(checker);
            if (result)
                OMPL.checker = checker;
            return result;
        }

        /// <summary>
        /// Sets the validity checker for the state space.
        /// </summary>
        /// <param name="checker">A <see cref="ValidityChecker"/> instance</param>
        /// <exception cref="OMPLException">Thrown when the native call fails</exception>
        public static void SetValidityChecker(ValidityChecker checker)
        {
            if (!TrySetValidityChecker(checker))
                throw new OMPLException("Unable to set validity checker");
        }

        /// <summary>
        /// Check if the validity checker is set or not.
        /// </summary>
        /// <returns><c>true</c> if validity checker is set, <c>false</c> otherwise.</returns>
        public static bool HasSetValidityChecker() => NativeMethods.HasSetValidityChecker();

        /// <summary>
        /// Sets the resolution for the validity checker
        /// </summary>
        /// <param name="resolution">the resolution as a percentages of the space's maximum extent at which to check for valid states</param>
        /// <returns><c>true</c> if the native call was successful, <c>false</c> otherwise</returns>
        public static bool TrySetValidityCheckerResolution(double resolution) => NativeMethods.SetValidityCheckerResolution(resolution);

        /// <summary>
        /// Sets the resolution for the validity checker
        /// </summary>
        /// <param name="resolution">the resolution as a percentages of the space's maximum extent at which to check for valid states</param>
        /// <exception cref="OMPLException">Thrown when the native call to set validity checker resolution fails</exception>
        public static void SetValidityCheckerResolution(double resolution)
        {
            bool result = TrySetValidityCheckerResolution(resolution);
            if (!result)
                throw new OMPLException("Failed to set validity checker resolution");
        }

        /// <summary>
        /// Find a solution in the state space given initial state <paramref name="initial"/> and goal state <paramref name="goal"/>
        /// within the number of seconds specified by <paramref name="limit"/>
        /// </summary>
        /// <param name="initial">The initial state. Must have length equal to number of dimensions in the state space.</param>
        /// <param name="goal">The initial state. Must have length equal to number of dimensions in the state space.</param>
        /// <param name="limit">Number of seconds alloted to find a solution.</param>
        /// <returns>A 2D array with shape (steps, dimensions).</returns>
        /// <exception cref="ArgumentNullException">Thrown if <paramref name="initial"/> or <paramref name="goal"/> is <c>null</c></exception>
        /// <exception cref="ArgumentOutOfRangeException">Thrown if <paramref name="limit"/> <c>&lt;= 0.0</c></exception>
        /// <exception cref="ArgumentException">
        /// Thrown if either the lengths of <paramref name="initial"/> and <paramref name="goal"/> mismatch
        /// or if their lengths mismatch with the number of dimensions in the state space.
        /// </exception>
        /// <exception cref="OMPLException">Thrown if unable to create or retrieve the solution.</exception>
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

        /// <summary>
        /// Find a solution in the state space given initial state <paramref name="initial"/> and goal state <paramref name="goal"/>
        /// within the number of seconds specified by <paramref name="limit"/>
        /// </summary>
        /// <param name="initial">The initial state. Must have length equal to number of dimensions in the state space.</param>
        /// <param name="goal">The initial state. Must have length equal to number of dimensions in the state space.</param>
        /// <param name="limit">Number of seconds alloted to find a solution.</param>
        /// <param name="solution">Solution is stored in this paramter as a 2D array of shape (steps, dimensions)</param>
        /// <returns><c>true</c> if the attempt to find solution was successful, <c>false</c> otherwise.</returns>
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
