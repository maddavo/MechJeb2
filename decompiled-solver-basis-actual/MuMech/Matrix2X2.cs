using UnityEngine;

namespace MuMech;

public class Matrix2X2
{
	private readonly double _a;

	private readonly double _b;

	private readonly double _c;

	private readonly double _d;

	public Matrix2X2(double a, double b, double c, double d)
	{
		_a = a;
		_b = b;
		_c = c;
		_d = d;
	}

	public Matrix2X2 Inverse()
	{
		double num = _a * _d - _b * _c;
		return new Matrix2X2(_d / num, (0.0 - _b) / num, (0.0 - _c) / num, _a / num);
	}

	public static Vector2d operator *(Matrix2X2 m, Vector2d vec)
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		return new Vector2d(m._a * vec.x + m._b * vec.y, m._c * vec.x + m._d * vec.y);
	}
}
