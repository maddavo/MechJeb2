using System;

namespace MuMech;

public class OperationException : Exception
{
	public OperationException(string message)
		: base(message)
	{
	}
}
