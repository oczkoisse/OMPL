using System;
using System.Collections.Generic;
using System.Text;

namespace OMPLUnity
{
    /// <summary>
    /// An exception denoting the failure of native OMPL layer.
    /// </summary>
    public sealed class OMPLException : Exception
    {
        public OMPLException() { }

        public OMPLException(string message) : base(message) { }

        public OMPLException(string message, Exception inner) : base(message, inner) { }
    }
}
