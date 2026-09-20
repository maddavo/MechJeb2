using System;
using System.Runtime.CompilerServices;
using MechJebLib.Functions;
using MechJebLib.Primitives;
using MechJebLib.Utils;
using MechJebLibBindings;
using UnityEngine;

namespace MuMech;

public static class OrbitExtensions
{
	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Vector3d WorldOrbitalVelocityAtUT(this Orbit o, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		Vector3d orbitalVelocityAtUT = o.getOrbitalVelocityAtUT(ut);
		return ((Vector3d)(ref orbitalVelocityAtUT)).xzy;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Vector3d WorldBCIPositionAtUT(this Orbit o, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		Vector3d relativePositionAtUT = o.getRelativePositionAtUT(ut);
		return ((Vector3d)(ref relativePositionAtUT)).xzy;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Vector3d WorldPositionAtUT(this Orbit o, double ut)
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		return o.referenceBody.position + o.WorldBCIPositionAtUT(ut);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static V3 RightHandedOrbitalVelocityAtUT(this Orbit o, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		return MathExtensions.ToV3(o.getOrbitalVelocityAtUT(ut));
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static V3 RightHandedBCIPositionAtUT(this Orbit o, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		return MathExtensions.ToV3(o.getRelativePositionAtUT(ut));
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static (V3 pos, V3 vel) RightHandedStateVectorsAtUT(this Orbit o, double ut)
	{
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = default(Vector3d);
		Vector3d val2 = default(Vector3d);
		o.GetOrbitalStateVectorsAtTrueAnomaly(o.TrueAnomalyAtT(o.getObtAtUT(ut)), ut, false, ref val, ref val2);
		val = ((CelestialFrame)(ref Planetarium.Zup)).WorldToLocal(val);
		val2 = ((CelestialFrame)(ref Planetarium.Zup)).WorldToLocal(val2);
		return (pos: MathExtensions.ToV3(val), vel: MathExtensions.ToV3(val2));
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static void FixedGetOrbitalStateVectorsAtUT(this Orbit o, double ut, out Vector3d pos, out Vector3d vel)
	{
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		o.GetOrbitalStateVectorsAtTrueAnomaly(o.TrueAnomalyAtT(o.getObtAtUT(ut)), ut, false, ref pos, ref vel);
		pos = ((CelestialFrame)(ref Planetarium.Zup)).WorldToLocal(pos);
		vel = ((CelestialFrame)(ref Planetarium.Zup)).WorldToLocal(vel);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Vector3d OrbitNormal(this Orbit o)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = o.GetOrbitNormal();
		val = ((Vector3d)(ref val)).xzy;
		return -((Vector3d)(ref val)).normalized;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Vector3d RadialPlus(this Orbit o, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = Vector3d.Exclude(o.Prograde(ut), o.Up(ut));
		return ((Vector3d)(ref val)).normalized;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Vector3d NormalPlus(this Orbit o, double ut)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		return o.OrbitNormal();
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Vector3d Horizontal(this Orbit o, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = Vector3d.Exclude(o.Up(ut), o.Prograde(ut));
		return ((Vector3d)(ref val)).normalized;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Vector3d HorizontalVelocity(this Orbit o, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		return Vector3d.Exclude(o.Up(ut), o.WorldOrbitalVelocityAtUT(ut));
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Vector3d VerticalVelocity(this Orbit o, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		return Vector3d.Dot(o.Up(ut), o.WorldOrbitalVelocityAtUT(ut)) * o.Up(ut);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Vector3d North(this Orbit o, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = Vector3d.Exclude(o.Up(ut), ((Component)o.referenceBody).transform.up * (float)o.referenceBody.Radius - o.WorldBCIPositionAtUT(ut));
		return ((Vector3d)(ref val)).normalized;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Vector3d East(this Orbit o, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		return Vector3d.Cross(o.Up(ut), o.North(ut));
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double Radius(this Orbit o, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = o.WorldBCIPositionAtUT(ut);
		return ((Vector3d)(ref val)).magnitude;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static Orbit PerturbedOrbit(this Orbit o, double ut, Vector3d dV)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		return MuUtils.OrbitFromStateVectors(o.WorldPositionAtUT(ut), o.WorldOrbitalVelocityAtUT(ut) + dV, o.referenceBody, ut);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double CircularOrbitSpeed(this Orbit o)
	{
		return Astro.CircularVelocity(o.referenceBody.gravParameter, o.radius);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double CircularOrbitPeriod(this Orbit o)
	{
		return Math.PI * 2.0 * o.radius / o.CircularOrbitSpeed();
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double Separation(this Orbit a, Orbit b, double ut)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = a.WorldPositionAtUT(ut) - b.WorldPositionAtUT(ut);
		return ((Vector3d)(ref val)).magnitude;
	}

	public static double NextClosestApproachTime(this Orbit a, Orbit b, double ut)
	{
		double num = ut;
		double num2 = double.MaxValue;
		double num3 = ut;
		double num4 = a.period;
		if (a.eccentricity > 1.0)
		{
			num4 = 100.0 / a.meanMotion;
		}
		double num5 = ut + num4;
		for (int i = 0; i < 8; i++)
		{
			double num6 = (num5 - num3) / 20.0;
			for (int j = 0; j < 20; j++)
			{
				double num7 = num3 + (double)j * num6;
				double num8 = a.Separation(b, num7);
				if (num8 < num2)
				{
					num2 = num8;
					num = num7;
				}
			}
			num3 = Statics.Clamp(num - num6, ut, ut + num4);
			num5 = Statics.Clamp(num + num6, ut, ut + num4);
		}
		return num;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double NextClosestApproachDistance(this Orbit a, Orbit b, double ut)
	{
		return a.Separation(b, a.NextClosestApproachTime(b, ut));
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double MeanAnomalyAtUT(this Orbit o, double ut)
	{
		double num = (o.ObTAtEpoch + (ut - o.epoch)) * o.meanMotion;
		if (o.eccentricity < 1.0)
		{
			num = Statics.Clamp2Pi(num);
		}
		return num;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double UTAtMeanAnomaly(this Orbit o, double meanAnomaly, double ut)
	{
		double num = o.MeanAnomalyAtUT(ut);
		double num2 = meanAnomaly - num;
		if (o.eccentricity < 1.0)
		{
			num2 = Statics.Clamp2Pi(num2);
		}
		return ut + num2 / o.meanMotion;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double NextPeriapsisTime(this Orbit o, double ut)
	{
		if (o.eccentricity < 1.0)
		{
			return o.TimeOfTrueAnomaly(0.0, ut);
		}
		return ut - o.MeanAnomalyAtUT(ut) / o.meanMotion;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double NextApoapsisTime(this Orbit o, double ut)
	{
		if (o.eccentricity < 1.0)
		{
			return o.TimeOfTrueAnomaly(Math.PI, ut);
		}
		throw new ArgumentException("OrbitExtensions.NextApoapsisTime cannot be called on hyperbolic orbits");
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	private static double AscendingNodeTrueAnomaly(this Orbit a, Orbit b)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = Vector3d.Cross(a.OrbitNormal(), b.OrbitNormal());
		return a.TrueAnomalyFromVector(val);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	private static double DescendingNodeTrueAnomaly(this Orbit a, Orbit b)
	{
		return Statics.Clamp2Pi(a.AscendingNodeTrueAnomaly(b) + Math.PI);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	private static double AscendingNodeEquatorialTrueAnomaly(this Orbit o)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = Vector3d.Cross(Vector3d.op_Implicit(((Component)o.referenceBody).transform.up), o.OrbitNormal());
		return o.TrueAnomalyFromVector(val);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	private static double DescendingNodeEquatorialTrueAnomaly(this Orbit o)
	{
		return Statics.Clamp2Pi(o.AscendingNodeEquatorialTrueAnomaly() + Math.PI);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	private static double MaximumTrueAnomaly(this Orbit o)
	{
		if (o.eccentricity < 1.0)
		{
			return Math.PI;
		}
		return Math.Acos(-1.0 / o.eccentricity);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static bool AscendingNodeExists(this Orbit a, Orbit b)
	{
		return Math.Abs(Statics.ClampPi(a.AscendingNodeTrueAnomaly(b))) <= a.MaximumTrueAnomaly();
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static bool DescendingNodeExists(this Orbit a, Orbit b)
	{
		return Math.Abs(Statics.ClampPi(a.DescendingNodeTrueAnomaly(b))) <= a.MaximumTrueAnomaly();
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static bool AscendingNodeEquatorialExists(this Orbit o)
	{
		return Math.Abs(Statics.ClampPi(o.AscendingNodeEquatorialTrueAnomaly())) <= o.MaximumTrueAnomaly();
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static bool DescendingNodeEquatorialExists(this Orbit o)
	{
		return Math.Abs(Statics.ClampPi(o.DescendingNodeEquatorialTrueAnomaly())) <= o.MaximumTrueAnomaly();
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	private static Vector3d WorldBCIPositionAtPeriapsis(this Orbit o)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = Vector3d.op_Implicit(Quaternion.AngleAxis(0f - (float)o.LAN, Vector3d.op_Implicit(Planetarium.up)) * Vector3d.op_Implicit(Planetarium.right));
		Vector3d val2 = Vector3d.op_Implicit(Quaternion.AngleAxis((float)o.argumentOfPeriapsis, Vector3d.op_Implicit(o.OrbitNormal())) * Vector3d.op_Implicit(val));
		return o.PeR * val2;
	}

	public static Vector3d WorldBCIPositionAtApoapsis(this Orbit o)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = Vector3d.op_Implicit(Quaternion.AngleAxis(0f - (float)o.LAN, Vector3d.op_Implicit(Planetarium.up)) * Vector3d.op_Implicit(Planetarium.right));
		Vector3d val2 = Vector3d.op_Implicit(Quaternion.AngleAxis((float)o.argumentOfPeriapsis, Vector3d.op_Implicit(o.OrbitNormal())) * Vector3d.op_Implicit(val));
		Vector3d val3 = (0.0 - o.ApR) * val2;
		if (double.IsNaN(val3.x))
		{
			Debug.LogError((object)"OrbitExtensions.WorldBCIPositionAtApoapsis got a NaN result!");
			Debug.LogError((object)("o.LAN = " + o.LAN));
			Debug.LogError((object)("o.inclination = " + o.inclination));
			Debug.LogError((object)("o.argumentOfPeriapsis = " + o.argumentOfPeriapsis));
			Vector3d val4 = o.OrbitNormal();
			Debug.LogError((object)("o.OrbitNormal() = " + ((object)(Vector3d)(ref val4)).ToString()));
		}
		return val3;
	}

	public static double TrueAnomalyFromVector(this Orbit o, Vector3d vec)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = o.OrbitNormal();
		Vector3d val2 = Vector3d.Exclude(val, vec);
		Vector3d val3 = o.WorldBCIPositionAtPeriapsis();
		double num = Vector3d.Angle(val3, val2);
		if (Math.Abs(Vector3d.Angle(val2, Vector3d.Cross(val, val3))) < 90.0)
		{
			return num * (Math.PI / 180.0);
		}
		return (360.0 - num) * (Math.PI / 180.0);
	}

	private static double GetEccentricAnomalyAtTrueAnomaly(this Orbit o, double trueAnomaly)
	{
		double eccentricity = o.eccentricity;
		trueAnomaly = Statics.Clamp2Pi(trueAnomaly);
		if (eccentricity < 1.0)
		{
			double num = (eccentricity + Math.Cos(trueAnomaly)) / (1.0 + eccentricity * Math.Cos(trueAnomaly));
			double num2 = Math.Sqrt(1.0 - num * num);
			if (trueAnomaly > Math.PI)
			{
				num2 *= -1.0;
			}
			return Statics.Clamp2Pi(Math.Atan2(num2, num));
		}
		double num3 = (eccentricity + Math.Cos(trueAnomaly)) / (1.0 + eccentricity * Math.Cos(trueAnomaly));
		if (num3 < 1.0)
		{
			throw new ArgumentException("OrbitExtensions.GetEccentricAnomalyAtTrueAnomaly: True anomaly of " + trueAnomaly + " radians is not attained by orbit with eccentricity " + o.eccentricity);
		}
		double num4 = Statics.Acosh(num3);
		if (trueAnomaly > Math.PI)
		{
			num4 *= -1.0;
		}
		return num4;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	private static double GetMeanAnomalyAtEccentricAnomaly(this Orbit o, double eanom)
	{
		double eccentricity = o.eccentricity;
		if (eccentricity < 1.0)
		{
			return Statics.Clamp2Pi(eanom - eccentricity * Math.Sin(eanom));
		}
		return eccentricity * Math.Sinh(eanom) - eanom;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double GetMeanAnomalyAtTrueAnomaly(this Orbit o, double tanom)
	{
		return o.GetMeanAnomalyAtEccentricAnomaly(o.GetEccentricAnomalyAtTrueAnomaly(tanom));
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double TimeOfAscendingNode(this Orbit a, Orbit b, double ut)
	{
		return a.TimeOfTrueAnomaly(a.AscendingNodeTrueAnomaly(b), ut);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double TimeOfDescendingNode(this Orbit a, Orbit b, double ut)
	{
		return a.TimeOfTrueAnomaly(a.DescendingNodeTrueAnomaly(b), ut);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double TimeOfAscendingNodeEquatorial(this Orbit o, double ut)
	{
		return o.TimeOfTrueAnomaly(o.AscendingNodeEquatorialTrueAnomaly(), ut);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double TimeOfDescendingNodeEquatorial(this Orbit o, double ut)
	{
		return o.TimeOfTrueAnomaly(o.DescendingNodeEquatorialTrueAnomaly(), ut);
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double SynodicPeriod(this Orbit a, Orbit b)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		int num = ((Vector3d.Dot(a.OrbitNormal(), b.OrbitNormal()) > 0.0) ? 1 : (-1));
		return Math.Abs(1.0 / (1.0 / a.period - (double)num * 1.0 / b.period));
	}

	public static double PhaseAngle(this Orbit a, Orbit b, double ut)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = a.OrbitNormal();
		Vector3d val2 = a.WorldBCIPositionAtUT(ut);
		Vector3d val3 = Vector3d.Exclude(val, b.WorldBCIPositionAtUT(ut));
		double num = Vector3d.Angle(val2, val3);
		if (Vector3d.Dot(Vector3d.Cross(val, val2), val3) < 0.0)
		{
			num = 360.0 - num;
		}
		return num;
	}

	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	public static double RelativeInclination(this Orbit a, Orbit b)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		return Math.Abs(Vector3d.Angle(a.OrbitNormal(), b.OrbitNormal()));
	}

	public static double NextTimeOfRadius(this Orbit o, double ut, double radius)
	{
		if (radius < o.PeR || (o.eccentricity < 1.0 && radius > o.ApR))
		{
			throw new ArgumentException("OrbitExtensions.NextTimeOfRadius: given radius of " + radius + " is never achieved: o.PeR = " + o.PeR + " and o.ApR = " + o.ApR);
		}
		double num = o.TrueAnomalyAtRadius(radius);
		double num2 = Math.PI * 2.0 - num;
		double num3 = o.TimeOfTrueAnomaly(num, ut);
		double num4 = o.TimeOfTrueAnomaly(num2, ut);
		if (num4 < num3 && num4 > ut)
		{
			return num4;
		}
		return num3;
	}

	public static Vector3d DeltaVToManeuverNodeCoordinates(this Orbit o, double ut, Vector3d dV)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		return new Vector3d(Vector3d.Dot(o.RadialPlus(ut), dV), Vector3d.Dot(-o.NormalPlus(ut), dV), Vector3d.Dot(o.Prograde(ut), dV));
	}

	public static Orbit TopParentOrbit(this Orbit orbit)
	{
		Orbit val = orbit;
		while ((Object)(object)val.referenceBody != (Object)(object)Planetarium.fetch.Sun)
		{
			val = val.referenceBody.orbit;
		}
		return val;
	}

	public static string MuString(this Orbit o)
	{
		return "PeA:" + o.PeA + " ApA:" + o.ApA + " SMA:" + o.semiMajorAxis + " ECC:" + o.eccentricity + " INC:" + o.inclination + " LAN:" + o.LAN + " ArgP:" + o.argumentOfPeriapsis + " TA:" + o.trueAnomaly;
	}
}
