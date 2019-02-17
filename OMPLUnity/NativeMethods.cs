using System.Runtime.InteropServices;
using System;

namespace OMPLUnity
{
    [return: MarshalAs(UnmanagedType.I1)]
    public delegate bool ValidityChecker(double[] state, int length);


    internal static class NativeMethods
    {
        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool Reset();

        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention=CallingConvention.Cdecl)]
        public static extern bool AddDimension(double min, double max);

        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern int DimensionCount();

        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetValidityChecker(ValidityChecker checker);

        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool HasSetValidityChecker();

        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool Solve(double[] initial , double[] goal, int dimensions, double limit, out int steps);

        [return: MarshalAs(UnmanagedType.I1)]
        [DllImport("OMPL.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetSolution(int steps, int dimensions, double[] solution);
    }
}
