namespace MuMech;

public class MovingAverage3d
{
	private readonly Vector3d[] _store;

	private readonly int _storeSize;

	private int _nextIndex;

	public Vector3d Value
	{
		get
		{
			//IL_0000: Unknown result type (might be due to invalid IL or missing references)
			//IL_0005: Unknown result type (might be due to invalid IL or missing references)
			//IL_000a: Unknown result type (might be due to invalid IL or missing references)
			//IL_0012: Unknown result type (might be due to invalid IL or missing references)
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_002c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0034: Unknown result type (might be due to invalid IL or missing references)
			Vector3d val = Vector3d.zero;
			for (int i = 0; i < _store.Length; i++)
			{
				val += _store[i];
			}
			return val / (double)_storeSize;
		}
		set
		{
			//IL_000c: Unknown result type (might be due to invalid IL or missing references)
			//IL_000d: Unknown result type (might be due to invalid IL or missing references)
			_store[_nextIndex] = value;
			_nextIndex = (_nextIndex + 1) % _storeSize;
		}
	}

	public MovingAverage3d(int size = 10, Vector3d startingValue = default(Vector3d))
	{
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		_storeSize = size;
		_store = (Vector3d[])(object)new Vector3d[size];
		Force(startingValue);
	}

	private void Force(Vector3d newValue)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		for (int i = 0; i < _storeSize; i++)
		{
			_store[i] = newValue;
		}
	}

	public static implicit operator Vector3d(MovingAverage3d v)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		return v.Value;
	}

	public override string ToString()
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		Vector3d value = Value;
		return ((object)(Vector3d)(ref value)).ToString();
	}

	public string ToString(string format)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		return MuUtils.PrettyPrint(Value, format);
	}
}
