using System;
using System.Collections.Generic;
using MechJebLib.Lambert;
using MechJebLib.Maneuvers;
using MechJebLib.Primitives;
using MechJebLib.Rootfinding;
using MechJebLib.Utils;
using MechJebLibBindings;
using Smooth.Delegates;
using Smooth.Pools;
using UnityEngine;

namespace MuMech;

public static class OrbitalManeuverCalculator
{
	public static readonly Pool<Orbit> OrbitPool = new Pool<Orbit>((DelegateFunc<Orbit>)createOrbit, (DelegateAction<Orbit>)resetOrbit);

	private static readonly SolverParameters solverParameters = new SolverParameters();

	public static Vector3d DeltaVToCircularize(Orbit o, double ut)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		var (val, val2) = o.RightHandedStateVectorsAtUT(ut);
		return MathExtensions.V3ToWorld(Simple.DeltaVToCircularize(o.referenceBody.gravParameter, val, val2));
	}

	public static Vector3d DeltaVToEllipticize(Orbit o, double ut, double newPeR, double newApR)
	{
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0056: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		double num = o.Radius(ut);
		newPeR = Statics.Clamp(newPeR, 1.0, num - 1.0);
		newApR = Math.Max(newApR, num + 1.0);
		var (val, val2) = o.RightHandedStateVectorsAtUT(ut);
		return MathExtensions.V3ToWorld(Simple.DeltaVToEllipticize(o.referenceBody.gravParameter, val, val2, newPeR, newApR));
	}

	public static Vector3d DeltaVToChangePeriapsis(Orbit o, double ut, double newPeR)
	{
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_004c: Unknown result type (might be due to invalid IL or missing references)
		double num = o.Radius(ut);
		newPeR = Statics.Clamp(newPeR, 1.0, num - 1.0);
		var (val, val2) = o.RightHandedStateVectorsAtUT(ut);
		return MathExtensions.V3ToWorld(ChangeOrbitalElement.ChangePeriapsis(o.referenceBody.gravParameter, val, val2, newPeR, false));
	}

	public static Vector3d DeltaVToChangeApoapsis(Orbit o, double ut, double newApR)
	{
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		double num = o.Radius(ut);
		if (newApR > 0.0)
		{
			newApR = Math.Max(newApR, num + 1.0);
		}
		var (val, val2) = o.RightHandedStateVectorsAtUT(ut);
		return MathExtensions.V3ToWorld(ChangeOrbitalElement.ChangeApoapsis(o.referenceBody.gravParameter, val, val2, newApR, false));
	}

	public static Vector3d DeltaVToChangeEccentricity(Orbit o, double ut, double newEcc)
	{
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		if (newEcc < 0.0)
		{
			newEcc = 0.0;
		}
		var (val, val2) = o.RightHandedStateVectorsAtUT(ut);
		return MathExtensions.V3ToWorld(ChangeOrbitalElement.ChangeECC(o.referenceBody.gravParameter, val, val2, newEcc, false));
	}

	public static Vector3d DeltaVForSemiMajorAxis(Orbit o, double ut, double newSMA)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		var (val, val2) = o.RightHandedStateVectorsAtUT(ut);
		return MathExtensions.V3ToWorld(ChangeOrbitalElement.ChangeSMA(o.referenceBody.gravParameter, val, val2, newSMA, false));
	}

	public static double HeadingForLaunchInclination(Orbit o, double inclinationDegrees, double desiredApoapsis)
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		(V3 pos, V3 vel) tuple = o.RightHandedStateVectorsAtUT(Planetarium.GetUniversalTime());
		V3 item = tuple.pos;
		V3 item2 = tuple.vel;
		double num = Math.PI * 2.0 / o.referenceBody.rotationPeriod;
		return Statics.Rad2Deg(Simple.HeadingForLaunchInclination(o.referenceBody.gravParameter, item, item2, Statics.Deg2Rad(inclinationDegrees), num, o.referenceBody.Radius + desiredApoapsis));
	}

	public static Vector3d DeltaVToChangeInclination(Orbit o, double ut, double newInclination)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		var (val, val2) = o.RightHandedStateVectorsAtUT(ut);
		return MathExtensions.V3ToWorld(Simple.DeltaVToChangeInclination(val, val2, Statics.Deg2Rad(newInclination)));
	}

	public static Vector3d DeltaVAndTimeToMatchPlanesAscending(Orbit o, Orbit target, double UT, out double burnUT)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		burnUT = o.TimeOfAscendingNode(target, UT);
		Vector3d val = Vector3d.Cross(target.OrbitNormal(), o.Up(burnUT));
		Vector3d val2 = Vector3d.Exclude(o.Up(burnUT), o.WorldOrbitalVelocityAtUT(burnUT));
		return ((Vector3d)(ref val2)).magnitude * val - val2;
	}

	public static Vector3d DeltaVAndTimeToMatchPlanesDescending(Orbit o, Orbit target, double UT, out double burnUT)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		burnUT = o.TimeOfDescendingNode(target, UT);
		Vector3d val = Vector3d.Cross(target.OrbitNormal(), o.Up(burnUT));
		Vector3d val2 = Vector3d.Exclude(o.Up(burnUT), o.WorldOrbitalVelocityAtUT(burnUT));
		return ((Vector3d)(ref val2)).magnitude * val - val2;
	}

	public static (Vector3d dV1, double UT1, Vector3d dV2, double UT2) DeltaVAndTimeForHohmannTransfer(Orbit o, Orbit target, double ut, double lagTime = double.NaN, bool fixedTime = false, bool coplanar = true, bool rendezvous = true, bool capture = true)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_0048: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_0071: Unknown result type (might be due to invalid IL or missing references)
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		var (val, val2) = o.RightHandedStateVectorsAtUT(ut);
		var (val3, val4) = target.RightHandedStateVectorsAtUT(ut);
		var (val5, num, val6, num2) = TwoImpulseTransfer.NextManeuver(o.referenceBody.gravParameter, val, val2, val3, val4, 50, lagTime, coplanar, rendezvous, capture, false, false);
		return (dV1: MathExtensions.V3ToWorld(val5), UT1: ut + num, dV2: MathExtensions.V3ToWorld(val6), UT2: ut + num2);
	}

	public static (Vector3d v1, Vector3d v2) DeltaVToInterceptAtTime(Orbit o, double t0, Orbit target, double dt, double offsetDistance = 0.0, bool prograde = true)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_0075: Unknown result type (might be due to invalid IL or missing references)
		//IL_0076: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_008a: Unknown result type (might be due to invalid IL or missing references)
		//IL_008f: Unknown result type (might be due to invalid IL or missing references)
		//IL_009b: Unknown result type (might be due to invalid IL or missing references)
		//IL_009c: Unknown result type (might be due to invalid IL or missing references)
		//IL_009e: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bf: Unknown result type (might be due to invalid IL or missing references)
		(V3 pos, V3 vel) tuple = o.RightHandedStateVectorsAtUT(t0);
		V3 item = tuple.pos;
		V3 item2 = tuple.vel;
		(V3 pos, V3 vel) tuple2 = target.RightHandedStateVectorsAtUT(t0 + dt);
		V3 item3 = tuple2.pos;
		V3 item4 = tuple2.vel;
		TransferGeometry val = (TransferGeometry)(prograde ? 2 : 3);
		var (val2, val3) = Gooding.Solve(o.referenceBody.gravParameter, item, item3, dt, val, 0, (V3?)V3.Cross(item, item2));
		if (offsetDistance != 0.0)
		{
			V3 val4 = item3;
			V3 val5 = V3.Cross(item4, item3);
			item3 = val4 - offsetDistance * ((V3)(ref val5)).normalized;
			(val2, val3) = Gooding.Solve(o.referenceBody.gravParameter, item, item3, dt, val, 0, (V3?)V3.Cross(item, item2));
		}
		return (v1: MathExtensions.V3ToWorld(val2 - item2), v2: MathExtensions.V3ToWorld(item4 - val3));
	}

	public static Vector3d DeltaVAndTimeForCheapestCourseCorrection(Orbit o, double UT, Orbit target, out double burnUT)
	{
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_009b: Unknown result type (might be due to invalid IL or missing references)
		//IL_007d: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		double num = o.NextClosestApproachTime(target, UT + 2.0);
		burnUT = UT;
		Vector3d result = DeltaVToInterceptAtTime(o, burnUT, target, num - burnUT).v1;
		for (double num2 = 0.5; num2 < 20.0; num2 += 1.0)
		{
			double num3 = UT + (num - UT) * num2 / 20.0;
			Vector3d item = DeltaVToInterceptAtTime(o, num3, target, num - num3).v1;
			if (((Vector3d)(ref item)).magnitude < ((Vector3d)(ref result)).magnitude)
			{
				result = item;
				burnUT = num3;
			}
		}
		return result;
	}

	public static (Vector3d dv, double dt1) DeltaVAndTimeForCourseCorrectionToCelestial(Orbit o, double ut, CelestialBody targetBody, double per, double dt = double.NaN, double inc = double.NaN)
	{
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_003d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0052: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_0084: Unknown result type (might be due to invalid IL or missing references)
		//IL_0089: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_0094: Unknown result type (might be due to invalid IL or missing references)
		Orbit orbit = targetBody.orbit;
		double gravParameter = o.referenceBody.gravParameter;
		double gravParameter2 = targetBody.gravParameter;
		double sphereOfInfluence = targetBody.sphereOfInfluence;
		double num = o.EndUT - ut;
		var (val, val2) = o.RightHandedStateVectorsAtUT(ut);
		var (val3, val4) = orbit.RightHandedStateVectorsAtUT(ut);
		var (val5, item, _) = new FineTuneClosestApproachToCelestial().Maneuver(gravParameter, val, val2, gravParameter2, val3, val4, sphereOfInfluence, num, Statics.Clamp(per, 0.0, sphereOfInfluence), dt, Statics.Deg2Rad(inc));
		return (dv: MathExtensions.V3ToWorld(val5), dt1: item);
	}

	public static Vector3d DeltaVAndTimeForCheapestCourseCorrection(Orbit o, double UT, Orbit target, double caDistance, out double burnUT)
	{
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0050: Unknown result type (might be due to invalid IL or missing references)
		//IL_0055: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0078: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Unknown result type (might be due to invalid IL or missing references)
		//IL_0082: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		//IL_0084: Unknown result type (might be due to invalid IL or missing references)
		//IL_0093: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a6: Unknown result type (might be due to invalid IL or missing references)
		Vector3d dV = DeltaVAndTimeForCheapestCourseCorrection(o, UT, target, out burnUT);
		double num = o.PerturbedOrbit(burnUT, dV).NextClosestApproachTime(target, burnUT);
		Vector3d val = target.WorldPositionAtUT(num) + target.NormalPlus(num) * caDistance;
		V3 val2 = MathExtensions.ToV3(o.WorldBCIPositionAtUT(burnUT));
		V3 val3 = MathExtensions.ToV3(o.WorldOrbitalVelocityAtUT(burnUT));
		V3 val4 = MathExtensions.ToV3(val - o.referenceBody.position);
		return MathExtensions.ToVector3d(Gooding.Solve(o.referenceBody.gravParameter, val2, val4, num - burnUT, (TransferGeometry)2, 0, (V3?)V3.Cross(val2, val3)).Item1) - o.WorldOrbitalVelocityAtUT(burnUT);
	}

	public static (Vector3d dv, double dt) DeltaVAndTimeForMoonReturnEjection(Orbit o, double ut, double peR, double inc = double.NaN)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_0048: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0062: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		//IL_0088: Unknown result type (might be due to invalid IL or missing references)
		//IL_0091: Unknown result type (might be due to invalid IL or missing references)
		//IL_0093: Unknown result type (might be due to invalid IL or missing references)
		ReturnFromMoon val = new ReturnFromMoon();
		CelestialBody referenceBody = o.referenceBody;
		CelestialBody referenceBody2 = referenceBody.referenceBody;
		(V3 pos, V3 vel) tuple = referenceBody.orbit.RightHandedStateVectorsAtUT(ut);
		V3 item = tuple.pos;
		V3 item2 = tuple.vel;
		double sphereOfInfluence = referenceBody.sphereOfInfluence;
		(V3 pos, V3 vel) tuple2 = o.RightHandedStateVectorsAtUT(ut);
		V3 item3 = tuple2.pos;
		V3 item4 = tuple2.vel;
		double sphereOfInfluence2 = referenceBody2.sphereOfInfluence;
		var (val2, num) = val.NextManeuver(referenceBody2.gravParameter, referenceBody.gravParameter, item, item2, sphereOfInfluence, item3, item4, Statics.Clamp(peR, 0.0, sphereOfInfluence2), Statics.Deg2Rad(inc));
		return (dv: MathExtensions.V3ToWorld(val2), dt: ut + num);
	}

	public static Vector3d DeltaVToMatchVelocities(Orbit o, double UT, Orbit target)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		return target.WorldOrbitalVelocityAtUT(UT) - o.WorldOrbitalVelocityAtUT(UT);
	}

	public static Vector3d DeltaVToResonantOrbit(Orbit o, double UT, double f)
	{
		//IL_00c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d7: Unknown result type (might be due to invalid IL or missing references)
		double apR = o.ApR;
		double peR = o.PeR;
		double num = Math.Pow(Math.Pow(apR, 3.0) * Math.Pow(f, 2.0) + 3.0 * Math.Pow(apR, 2.0) * Math.Pow(f, 2.0) * peR + 3.0 * apR * Math.Pow(f, 2.0) * Math.Pow(peR, 2.0) + Math.Pow(f, 2.0) * Math.Pow(peR, 3.0), 1.0 / 3.0) - apR;
		if (num < 0.0)
		{
			return Vector3d.zero;
		}
		if (f > 1.0)
		{
			return DeltaVToChangeApoapsis(o, UT, num);
		}
		return DeltaVToChangePeriapsis(o, UT, num);
	}

	public static double Distance(double lat_a, double long_a, double lat_b, double long_b)
	{
		double num = Math.PI / 180.0 * lat_a;
		double num2 = Math.PI / 180.0 * lat_b;
		double num3 = Math.PI / 180.0 * (long_b - long_a);
		return 180.0 / Math.PI * Math.Atan2(Math.Sqrt(Math.Pow(Math.Cos(num2) * Math.Sin(num3), 2.0) + Math.Pow(Math.Cos(num) * Math.Sin(num2) - Math.Sin(num) * Math.Cos(num2) * Math.Cos(num3), 2.0)), Math.Sin(num) * Math.Sin(num2) + Math.Cos(num) * Math.Cos(num2) * Math.Cos(num3));
	}

	public static double Heading(double lat_a, double long_a, double lat_b, double long_b)
	{
		double num = Math.PI / 180.0 * lat_a;
		double a = Math.PI / 180.0 * lat_b;
		double num2 = Math.PI / 180.0 * (long_b - long_a);
		return MuUtils.ClampDegrees360(180.0 / Math.PI * Math.Atan2(Math.Sin(num2), Math.Cos(num) * Math.Tan(a) - Math.Sin(num) * Math.Cos(num2)));
	}

	public static Vector3d DeltaVToShiftLAN(Orbit o, double UT, double newLAN)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00db: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0101: Unknown result type (might be due to invalid IL or missing references)
		//IL_0106: Unknown result type (might be due to invalid IL or missing references)
		//IL_0125: Unknown result type (might be due to invalid IL or missing references)
		//IL_012a: Unknown result type (might be due to invalid IL or missing references)
		//IL_012f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0131: Unknown result type (might be due to invalid IL or missing references)
		//IL_0133: Unknown result type (might be due to invalid IL or missing references)
		//IL_0138: Unknown result type (might be due to invalid IL or missing references)
		//IL_013a: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = o.WorldPositionAtUT(UT);
		double latitude = o.referenceBody.GetLatitude(val, false);
		double long_a = o.referenceBody.GetLongitude(val, false) + o.referenceBody.rotationAngle;
		double num = 0.0;
		if (o.AscendingNodeEquatorialExists() && o.DescendingNodeEquatorialExists())
		{
			num = ((!(o.TimeOfDescendingNodeEquatorial(UT) < o.TimeOfAscendingNodeEquatorial(UT))) ? MuUtils.ClampDegrees360(newLAN) : MuUtils.ClampDegrees360(newLAN + 180.0));
		}
		else if (o.AscendingNodeEquatorialExists() && !o.DescendingNodeEquatorialExists())
		{
			num = MuUtils.ClampDegrees360(newLAN);
		}
		else
		{
			if (o.AscendingNodeEquatorialExists() || !o.DescendingNodeEquatorialExists())
			{
				throw new ArgumentException("OrbitalManeuverCalculator.DeltaVToShiftLAN: No Equatorial Nodes");
			}
			num = MuUtils.ClampDegrees360(newLAN + 180.0);
		}
		double num2 = MuUtils.ClampDegrees360(Heading(latitude, long_a, 0.0, num));
		Vector3d val2 = Vector3d.Exclude(o.Up(UT), o.WorldOrbitalVelocityAtUT(UT));
		Vector3d val3 = ((Vector3d)(ref val2)).magnitude * Math.Sin(Math.PI / 180.0 * num2) * o.East(UT);
		Vector3d val4 = ((Vector3d)(ref val2)).magnitude * Math.Cos(Math.PI / 180.0 * num2) * o.North(UT);
		return val3 + val4 - val2;
	}

	public static Vector3d DeltaVToShiftNodeLongitude(Orbit o, double UT, double newNodeLong)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = o.WorldPositionAtUT(UT);
		double num = o.Radius(UT);
		double num2 = 0.0;
		double num3 = (UT - Planetarium.GetUniversalTime()) * 360.0 / o.referenceBody.rotationPeriod;
		double num4 = o.referenceBody.GetLongitude(val, false) - num3 - newNodeLong;
		int num5 = -1;
		double num6 = 0.0;
		while (num2 - o.referenceBody.Radius < (double)o.referenceBody.timeWarpAltitudeLimits[4] && num5 < 20)
		{
			num5++;
			double num7 = o.referenceBody.rotationPeriod * (num4 / 360.0 + (double)num5);
			num6 = Math.Pow(o.referenceBody.gravParameter * num7 * num7 / 39.47841760435743, 1.0 / 3.0);
			num2 = 2.0 * num6 - num;
		}
		return DeltaVForSemiMajorAxis(o, UT, num6);
	}

	private static Orbit createOrbit()
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Expected O, but got Unknown
		return new Orbit();
	}

	private static void resetOrbit(Orbit o)
	{
	}

	public static void PatchedConicInterceptBody(Orbit initial, CelestialBody target, Vector3d dV, double burnUT, double arrivalUT, out Orbit intercept)
	{
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		Orbit val = OrbitPool.Borrow();
		val.UpdateFromStateVectors(initial.getRelativePositionAtUT(burnUT), initial.getOrbitalVelocityAtUT(burnUT) + ((Vector3d)(ref dV)).xzy, initial.referenceBody, burnUT);
		val.StartUT = burnUT;
		val.EndUT = ((val.eccentricity >= 1.0) ? val.period : (burnUT + val.period));
		Orbit val2 = OrbitPool.Borrow();
		for (bool flag = PatchedConics.CalculatePatch.Invoke(val, val2, burnUT, solverParameters, (CelestialBody)null); flag && (Object)(object)val.referenceBody != (Object)(object)target && val.EndUT < arrivalUT; flag = PatchedConics.CalculatePatch.Invoke(val, val2, val.StartUT, solverParameters, (CelestialBody)null))
		{
			OrbitPool.Release(val);
			val = val2;
			val2 = OrbitPool.Borrow();
		}
		intercept = val;
		intercept.UpdateFromOrbitAtUT(val, arrivalUT, val.referenceBody);
		OrbitPool.Release(val);
		OrbitPool.Release(val2);
	}

	public static List<ManeuverParameters> OptimizeEjectionToTarget(Orbit o, MechJebModuleTargetController target, double targetPeR, double epoch, double arrivalDt, double arrivalDtLower = 0.0, double arrivalDtUpper = double.PositiveInfinity)
	{
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_0048: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_0101: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		Orbit targetOrbit = target.TargetOrbit;
		ITargetable target2 = target.Target;
		ITargetable obj = ((target2 is CelestialBody) ? target2 : null);
		CelestialBody referenceBody = o.referenceBody;
		Orbit orbit = referenceBody.orbit;
		(V3 pos, V3 vel) tuple = o.RightHandedStateVectorsAtUT(epoch);
		V3 item = tuple.pos;
		V3 item2 = tuple.vel;
		double gravParameter = referenceBody.gravParameter;
		(V3 pos, V3 vel) tuple2 = orbit.RightHandedStateVectorsAtUT(epoch);
		V3 item3 = tuple2.pos;
		V3 item4 = tuple2.vel;
		double sphereOfInfluence = referenceBody.sphereOfInfluence;
		double num = ((CelestialBody)(obj?)).gravParameter ?? 0.0;
		(V3 pos, V3 vel) tuple3 = targetOrbit.RightHandedStateVectorsAtUT(epoch);
		V3 item5 = tuple3.pos;
		V3 item6 = tuple3.vel;
		double num2 = ((CelestialBody)(obj?)).sphereOfInfluence ?? 0.0;
		double gravParameter2 = orbit.referenceBody.gravParameter;
		double num3 = Statics.Clamp(targetPeR, 0.0, num2);
		var (val, num4, _, _) = new InterplanetaryTransfer().Maneuver(item, item2, gravParameter, item3, item4, sphereOfInfluence, num, item5, item6, num2, gravParameter2, arrivalDt, arrivalDtLower, arrivalDtUpper, num3, double.NaN, false, false);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(MathExtensions.V3ToWorld(val), epoch + num4)
		};
	}

	public static void SOI_intercept(Orbit transfer, CelestialBody target, double UT1, double UT2, out double UT)
	{
		if ((Object)(object)transfer.referenceBody != (Object)(object)target.orbit.referenceBody)
		{
			throw new ArgumentException("[MechJeb] SOI_intercept: transfer orbit must be in the same SOI as the target celestial");
		}
		Func<double, object, double> func = delegate(double UT, object ign)
		{
			//IL_0007: Unknown result type (might be due to invalid IL or missing references)
			//IL_0018: Unknown result type (might be due to invalid IL or missing references)
			//IL_001d: Unknown result type (might be due to invalid IL or missing references)
			//IL_0022: Unknown result type (might be due to invalid IL or missing references)
			Vector3d val = transfer.getRelativePositionAtUT(UT) - target.orbit.getRelativePositionAtUT(UT);
			return ((Vector3d)(ref val)).magnitude - target.sphereOfInfluence;
		};
		UT = 0.0;
		try
		{
			UT = BrentRoot.Solve(func, UT1, UT2, (object)null, 100, 2.220446049250313E-16, 0);
		}
		catch (TimeoutException)
		{
			Debug.Log((object)"[MechJeb] Brents method threw a timeout error (supressed)");
		}
	}
}
