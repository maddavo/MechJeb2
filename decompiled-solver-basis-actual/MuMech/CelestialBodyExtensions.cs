using System;
using UnityEngine;

namespace MuMech;

public static class CelestialBodyExtensions
{
	public static double TerrainAltitude(this CelestialBody body, Vector3d worldPosition)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		return body.TerrainAltitude(body.GetLatitude(worldPosition, false), body.GetLongitude(worldPosition, false), false);
	}

	public static void GetLatLngAltAtUT(this CelestialBody body, double ut, Vector3d localPosition, out double lat, out double lon, out double alt)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0003: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		Vector3d currentSurfacePositionFromUT = body.GetCurrentSurfacePositionFromUT(ut, localPosition);
		LatLon.GetLatLongAlt(body.BodyFrame, Vector3d.zero, body.Radius, currentSurfacePositionFromUT, ref lat, ref lon, ref alt);
	}

	public static Vector3d GetCurrentSurfacePositionFromUT(this CelestialBody body, double ut, Vector3d localPosition)
	{
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		double num = ut - Planetarium.GetUniversalTime();
		return QuaternionD.AngleAxis(0.0 - 360.0 / body.rotationPeriod * num, new Vector3d(0.0, -1.0, 0.0)) * localPosition;
	}

	public static double DragLength(this CelestialBody body, Vector3d pos, double dragCoeff, double mass)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		double atmDensity = FlightGlobals.getAtmDensity(FlightGlobals.getStaticPressure(pos, body), FlightGlobals.getExternalTemperature(pos, body), (CelestialBody)null);
		if (atmDensity <= 0.0)
		{
			return double.MaxValue;
		}
		return mass / (0.0005 * (double)PhysicsGlobals.DragMultiplier * atmDensity * dragCoeff);
	}

	public static double DragLength(this CelestialBody body, double altitudeASL, double dragCoeff, double mass)
	{
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		return body.DragLength(body.GetWorldSurfacePosition(0.0, 0.0, altitudeASL), dragCoeff, mass);
	}

	public static double RealMaxAtmosphereAltitude(this CelestialBody body)
	{
		if (body.atmosphere)
		{
			return body.atmosphereDepth;
		}
		return 0.0;
	}

	public static double AltitudeForPressure(this CelestialBody body, double pressure)
	{
		if (!body.atmosphere)
		{
			return 0.0;
		}
		double num = body.atmosphereDepth;
		double num2 = 0.0;
		while (num - num2 > 10.0)
		{
			double num3 = (num + num2) * 0.5;
			if (FlightGlobals.getStaticPressure(num3, body) < pressure)
			{
				num = num3;
			}
			else
			{
				num2 = num3;
			}
		}
		return (num + num2) * 0.5;
	}

	public static string GetExperimentBiomeSafe(this CelestialBody body, double lat, double lon)
	{
		if ((Object)(object)body.BiomeMap == (Object)null || body.BiomeMap.Attributes.Length == 0)
		{
			return string.Empty;
		}
		return ScienceUtil.GetExperimentBiomeLocalized(body, lat, lon);
	}

	public static float GetPQSSlopeDegrees(this CelestialBody body, double latitude, double longitude, double sampleRadiusMeters = 50.0)
	{
		double num = sampleRadiusMeters / (body.Radius * Math.PI / 180.0);
		double num2 = body.TerrainAltitude(latitude + num, longitude, true);
		double num3 = body.TerrainAltitude(latitude - num, longitude, true);
		double num4 = body.TerrainAltitude(latitude, longitude + num, true);
		double num5 = body.TerrainAltitude(latitude, longitude - num, true);
		double num6 = body.Radius * Math.PI / 180.0;
		double num7 = (num4 - num5) / (2.0 * num * num6);
		double num8 = (num2 - num3) / (2.0 * num * num6);
		return (float)(Math.Atan(Math.Sqrt(num7 * num7 + num8 * num8)) * 180.0 / Math.PI);
	}
}
