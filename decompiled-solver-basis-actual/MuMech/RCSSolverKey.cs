using UnityEngine;

namespace MuMech;

public class RCSSolverKey
{
	private static int _precision = 2;

	private readonly int _hash;

	public static void SetPrecision(int precision)
	{
		_precision = precision;
	}

	private float Bucketize(double d, int precision)
	{
		return (float)Mathf.RoundToInt((float)d * (float)precision) / (float)precision;
	}

	public RCSSolverKey(ref Vector3 d, Vector3 rot)
	{
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		if (d == Vector3.zero)
		{
			_hash = 0;
			return;
		}
		((Vector3)(ref d)).Normalize();
		float num = Mathf.Max(Mathf.Abs(d.x), Mathf.Max(Mathf.Abs(d.y), Mathf.Abs(d.z)));
		d.x = Bucketize(d.x / num, _precision);
		d.y = Bucketize(d.y / num, _precision);
		d.z = Bucketize(d.z / num, _precision);
		int num2 = (int)(d.x * 127f);
		int num3 = (int)(d.y * 127f);
		int num4 = (int)(d.z * 127f);
		_hash = ((num2 & 0xFF) << 16) + ((num3 & 0xFF) << 8) + (num4 & 0xFF);
	}

	public override bool Equals(object other)
	{
		if (other is RCSSolverKey rCSSolverKey)
		{
			return _hash == rCSSolverKey._hash;
		}
		return false;
	}

	public override int GetHashCode()
	{
		return _hash;
	}

	public override string ToString()
	{
		return _hash.ToString("x6");
	}
}
