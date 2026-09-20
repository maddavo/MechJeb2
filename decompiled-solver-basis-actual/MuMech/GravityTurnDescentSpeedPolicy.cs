namespace MuMech;

internal class GravityTurnDescentSpeedPolicy : IDescentSpeedPolicy
{
	private readonly double _terrainRadius;

	private readonly double _g;

	private readonly double _thrust;

	public GravityTurnDescentSpeedPolicy(double terrainRadius, double g, double thrust)
	{
		_terrainRadius = terrainRadius;
		_g = g;
		_thrust = thrust;
	}

	public double MaxAllowedSpeed(Vector3d pos, Vector3d vel)
	{
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		double num = ((Vector3d)(ref pos)).magnitude - _terrainRadius;
		double num2 = 0.0;
		double num3 = 1.1 * ((Vector3d)(ref vel)).magnitude;
		while (num3 - num2 > 0.1)
		{
			double num4 = (num3 + num2) / 2.0;
			if (GravityTurnFallDistance(pos, num4 * ((Vector3d)(ref vel)).normalized) < num)
			{
				num2 = num4;
			}
			else
			{
				num3 = num4;
			}
		}
		return 0.95 * ((num3 + num2) / 2.0);
	}

	private double GravityTurnFallDistance(Vector3d x, Vector3d v)
	{
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		//IL_0075: Unknown result type (might be due to invalid IL or missing references)
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0088: Unknown result type (might be due to invalid IL or missing references)
		//IL_008d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_0094: Unknown result type (might be due to invalid IL or missing references)
		//IL_0096: Unknown result type (might be due to invalid IL or missing references)
		double magnitude = ((Vector3d)(ref x)).magnitude;
		for (int i = 0; i < 10; i++)
		{
			Vector3d val = (0.0 - _g) * ((Vector3d)(ref x)).normalized;
			Vector3d val2 = (0.0 - _thrust) * ((Vector3d)(ref v)).normalized;
			double num = 1.0 / (double)(10 - i) * (((Vector3d)(ref v)).magnitude / _thrust);
			Vector3d val3 = v + num * (val2 + val);
			x += num * (v + val3) / 2.0;
			v = val3;
		}
		double magnitude2 = ((Vector3d)(ref x)).magnitude;
		magnitude2 -= ((Vector3d)(ref v)).sqrMagnitude / (2.0 * (_thrust - _g));
		return magnitude - magnitude2;
	}
}
