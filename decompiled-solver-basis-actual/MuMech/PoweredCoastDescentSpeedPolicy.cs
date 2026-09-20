using System;

namespace MuMech;

internal class PoweredCoastDescentSpeedPolicy : IDescentSpeedPolicy
{
	private readonly float _terrainRadius;

	private readonly float _g;

	private readonly float _thrust;

	public PoweredCoastDescentSpeedPolicy(double terrainRadius, double g, double thrust)
	{
		_terrainRadius = (float)terrainRadius;
		_g = (float)g;
		_thrust = (float)thrust;
	}

	public double MaxAllowedSpeed(Vector3d pos, Vector3d vel)
	{
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		if ((double)_terrainRadius < ((Vector3d)(ref pos)).magnitude)
		{
			return double.MaxValue;
		}
		double num = Vector3d.Dot(vel, ((Vector3d)(ref pos)).normalized);
		double num2 = (num + Math.Sqrt(num * num + (double)(2f * _g) * (((Vector3d)(ref pos)).magnitude - (double)_terrainRadius))) / (double)_g;
		return 0.8 * (double)(_thrust - _g) * num2;
	}
}
