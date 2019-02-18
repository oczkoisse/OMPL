using System.Runtime.InteropServices;
using System;

namespace OMPLUnity
{
    /// <summary>
    /// Checks the validity of a given state, for example, against collisions.
    /// </summary>
    /// <param name="state">A 1D array with length <paramref name="length"/></param>
    /// <param name="length">The length of <paramref name="state"/></param>
    /// <returns><c>true</c> if the state is free, <c>false</c> otherwie</returns>
    [return: MarshalAs(UnmanagedType.I1)]
    public delegate bool ValidityChecker(double[] state, int length);


    internal static class NativeMethods
    {
        /// <summary>
        /// Resets the state space to <c>0</c> dimensions so that 
        /// <see cref="NativeMethods.DimensionCount()"/> returns <c>0</c>.
        /// The <see cref="ValidityChecker"/> reference is also reset so that
        /// <see cref="NativeMethods.HasSetValidityChecker()"/> returns <c>null</c>.
        /// </summary>
        /// <returns><c>true</c> if the call was successful, <c>false</c> otherwise</returns>
        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool Reset();

        /// <summary>
        /// Adds a new dimension with bounds [<paramref name="min"/>, <paramref name="max"/>] to the state space.
        /// </summary>
        /// <param name="min">Minimum value for the dimension</param>
        /// <param name="max">Maximum value for the dimension</param>
        /// <returns><c>true</c> if the call was successful, <c>false</c> otherwise</returns>
        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention=CallingConvention.Cdecl)]
        public static extern bool AddDimension(double min, double max);

        /// <summary>
        /// Get the number of dimensions in the state space.
        /// </summary>
        /// <returns>The number of dimensions. A value of <c>-1</c> is returned if the call is unsuccessful.</returns>
        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern int DimensionCount();

        /// <summary>
        /// Sets the validity checker for the state space.
        /// </summary>
        /// <param name="checker">A <see cref="ValidityChecker"/> instance</param>
        /// <returns><c>true</c> if the call was successful, <c>false</c> otherwise</returns>
        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetValidityChecker(ValidityChecker checker);

        /// <summary>
        /// Check if the validity checker is set or not.
        /// </summary>
        /// <returns><c>true</c> if validity checker is set, <c>false</c> otherwise.</returns>
        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool HasSetValidityChecker();

        /// <summary>
        /// Find a solution in the state space given initial state <paramref name="initial"/> and goal state <paramref name="goal"/>.
        /// The solution must be figured out within the number of seconds specified by <paramref name="limit"/>.
        /// If a solution is figured out, the number of steps in that solution is stored in <paramref name="steps"/>,
        /// otherwise it has a value of <c>-1</c>.
        /// </summary>
        /// <param name="initial">The initial state. Must have length equal to <paramref name="dimensions"/>.</param>
        /// <param name="goal">The initial state. Must have length equal to <paramref name="dimensions"/>.</param>
        /// <param name="dimensions">The length of initial and goal states. Should be equal to the number of dimensions in the state space itself.</param>
        /// <param name="limit">Number of seconds alloted to find a solution.</param>
        /// <param name="steps">The number of steps in the resulting solution if found, otherwise a value of <c>-1</c></param>
        /// <returns><c>true</c> if the call was successful, <c>false</c> otherwise</returns>
        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool Solve(double[] initial , double[] goal, int dimensions, double limit, out int steps);

        /// <summary>
        /// Retrieve a solution created by an earlier call of <see cref="NativeMethods.Solve(double[], double[], int, double, out int)"/>
        /// </summary>
        /// <param name="steps">Number of steps in the solution. Must be equal to the value output by <see cref="NativeMethods.Solve(double[], double[], int, double, out int)"/></param>
        /// <param name="dimensions">Number of dimensions in the state space</param>
        /// <param name="solution">A 1-D array that must have length equal to <paramref name="steps"/> times <paramref name="dimensions"/></param>
        /// <returns><c>true</c> if the call was successful, <c>false</c> otherwise</returns>
        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetSolution(int steps, int dimensions, double[] solution);
    }
}
