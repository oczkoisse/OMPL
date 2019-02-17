using System;
using System.Collections.Generic;
using System.Text;

namespace OMPLUnity
{
    public sealed class OMPLException : Exception
    {
        public OMPLException() { }

        public OMPLException(string message) : base(message) { }

        public OMPLException(string message, Exception inner) : base(message, inner) { }
    }
}
