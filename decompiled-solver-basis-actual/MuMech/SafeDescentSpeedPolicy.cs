using System;

namespace MuMech;

internal class SafeDescentSpeedPolicy : IDescentSpeedPolicy
{
	private readonly double _terrainRadius;

	private readonly double _g;

	private readonly double _thrust;

	public SafeDescentSpeedPolicy(double terrainRadius, double g, double thrust)
	{
		_terrainRadius = terrainRadius;
		_g = g;
		_thrust = thrust;
	}

	public double MaxAllowedSpeed(Vector3d pos, Vector3d vel)
	{
		double num = ((Vector3d)(ref pos)).magnitude - _terrainRadius;
		return 0.9 * Math.Sqrt(2.0 * (_thrust - _g) * num);
	}
}
