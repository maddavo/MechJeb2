using System;

namespace MuMech;

public class ReferenceFrame
{
	private double _epoch;

	private Vector3d _lat0Lon0AtStart;

	private Vector3d _lat0Lon90AtStart;

	private Vector3d _lat90AtStart;

	private CelestialBody _referenceBody;

	public void UpdateAtCurrentTime(CelestialBody body)
	{
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0050: Unknown result type (might be due to invalid IL or missing references)
		//IL_0055: Unknown result type (might be due to invalid IL or missing references)
		_lat0Lon0AtStart = body.GetSurfaceNVector(0.0, 0.0);
		_lat0Lon90AtStart = body.GetSurfaceNVector(0.0, 90.0);
		_lat90AtStart = body.GetSurfaceNVector(90.0, 0.0);
		_epoch = Planetarium.GetUniversalTime();
		_referenceBody = body;
	}

	public AbsoluteVector ToAbsolute(Vector3d vector3d, double ut)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		AbsoluteVector absoluteVector = default(AbsoluteVector);
		absoluteVector.Latitude = Latitude(vector3d);
		AbsoluteVector result = absoluteVector;
		double num = 180.0 / Math.PI * Math.Atan2(Vector3d.Dot(((Vector3d)(ref vector3d)).normalized, _lat0Lon90AtStart), Vector3d.Dot(((Vector3d)(ref vector3d)).normalized, _lat0Lon0AtStart));
		num -= 360.0 * (ut - _epoch) / _referenceBody.rotationPeriod;
		result.Longitude = MuUtils.ClampDegrees180(num);
		result.Radius = ((Vector3d)(ref vector3d)).magnitude;
		result.UT = ut;
		return result;
	}

	public Vector3d WorldPositionAtCurrentTime(AbsoluteVector absolute)
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		return _referenceBody.position + WorldVelocityAtCurrentTime(absolute);
	}

	public Vector3d BodyPositionAtCurrentTime(AbsoluteVector absolute)
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		return _referenceBody.position + absolute.Radius * _referenceBody.GetSurfaceNVector(absolute.Latitude, absolute.Longitude);
	}

	public Vector3d WorldVelocityAtCurrentTime(AbsoluteVector absolute)
	{
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		double universalTime = Planetarium.GetUniversalTime();
		double num = MuUtils.ClampDegrees360(absolute.Longitude - 360.0 * (universalTime - absolute.UT) / _referenceBody.rotationPeriod);
		return absolute.Radius * _referenceBody.GetSurfaceNVector(absolute.Latitude, num);
	}

	public double Latitude(Vector3d vector3d)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		return 180.0 / Math.PI * Math.Asin(Vector3d.Dot(((Vector3d)(ref vector3d)).normalized, _lat90AtStart));
	}
}
