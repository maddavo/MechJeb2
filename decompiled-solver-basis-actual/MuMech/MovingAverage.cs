namespace MuMech;

public class MovingAverage
{
	private readonly double[] _store;

	private readonly int _storeSize;

	private int _nextIndex;

	public double Value
	{
		get
		{
			double num = 0.0;
			for (int i = 0; i < _store.Length; i++)
			{
				num += _store[i];
			}
			return num / (double)_storeSize;
		}
		set
		{
			_store[_nextIndex] = value;
			_nextIndex = (_nextIndex + 1) % _storeSize;
		}
	}

	public MovingAverage(int size = 10, double startingValue = 0.0)
	{
		_storeSize = size;
		_store = new double[size];
		Force(startingValue);
	}

	private void Force(double newValue)
	{
		for (int i = 0; i < _storeSize; i++)
		{
			_store[i] = newValue;
		}
	}

	public static implicit operator double(MovingAverage v)
	{
		return v.Value;
	}

	public override string ToString()
	{
		return Value.ToString();
	}

	public string ToString(string format)
	{
		return Value.ToString(format);
	}
}
