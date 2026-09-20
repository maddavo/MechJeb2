using System;
using KSP.Localization;
using MechJebLib.Utils;

namespace MuMech;

public class MechJebModuleRendezvousAutopilot : ComputerModule
{
	[Persistent(pass = 4)]
	public readonly EditableDouble desiredDistance = 100.0;

	[Persistent(pass = 4)]
	public readonly EditableDouble maxPhasingOrbits = 5.0;

	[Persistent(pass = 4)]
	public readonly EditableDouble maxClosingSpeed = 100.0;

	public string status = "";

	public MechJebModuleRendezvousAutopilot(MechJebCore core)
		: base(core)
	{
	}

	protected override void OnModuleEnabled()
	{
		base.Vessel.RemoveAllManeuverNodes();
		if (!MuUtils.PhysicsRunning())
		{
			Core.Warp.MinimumWarp();
		}
	}

	protected override void OnModuleDisabled()
	{
		Core.Node.Abort();
	}

	public override void Drive(FlightCtrlState s)
	{
		//IL_0149: Unknown result type (might be due to invalid IL or missing references)
		//IL_014e: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0526: Unknown result type (might be due to invalid IL or missing references)
		//IL_052b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0535: Unknown result type (might be due to invalid IL or missing references)
		//IL_054c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0551: Unknown result type (might be due to invalid IL or missing references)
		//IL_0556: Unknown result type (might be due to invalid IL or missing references)
		//IL_0568: Unknown result type (might be due to invalid IL or missing references)
		//IL_057f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0584: Unknown result type (might be due to invalid IL or missing references)
		//IL_0589: Unknown result type (might be due to invalid IL or missing references)
		//IL_046e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0473: Unknown result type (might be due to invalid IL or missing references)
		//IL_0481: Unknown result type (might be due to invalid IL or missing references)
		//IL_0300: Unknown result type (might be due to invalid IL or missing references)
		//IL_0305: Unknown result type (might be due to invalid IL or missing references)
		//IL_032c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0342: Unknown result type (might be due to invalid IL or missing references)
		//IL_0347: Unknown result type (might be due to invalid IL or missing references)
		//IL_034c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0689: Unknown result type (might be due to invalid IL or missing references)
		//IL_068e: Unknown result type (might be due to invalid IL or missing references)
		//IL_05f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_06e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_03b7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b74: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b79: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b48: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b4d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a60: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a65: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a73: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b87: Unknown result type (might be due to invalid IL or missing references)
		//IL_07f1: Unknown result type (might be due to invalid IL or missing references)
		//IL_07f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0804: Unknown result type (might be due to invalid IL or missing references)
		//IL_083a: Unknown result type (might be due to invalid IL or missing references)
		//IL_083f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0849: Unknown result type (might be due to invalid IL or missing references)
		//IL_0911: Unknown result type (might be due to invalid IL or missing references)
		//IL_0916: Unknown result type (might be due to invalid IL or missing references)
		//IL_0924: Unknown result type (might be due to invalid IL or missing references)
		//IL_088b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0890: Unknown result type (might be due to invalid IL or missing references)
		//IL_089e: Unknown result type (might be due to invalid IL or missing references)
		//IL_08d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_08d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_08e3: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Target.NormalTargetExists)
		{
			Users.Clear();
			return;
		}
		Core.Node.Autowarp = Core.Node.Autowarp && Core.Target.Distance > 1000f;
		if ((double)Core.Target.Distance < (double)desiredDistance && base.Vessel.patchedConicSolver.maneuverNodes.Count > 0 && base.Vessel.patchedConicSolver.maneuverNodes[0].UT > base.VesselState.Time + 1.0)
		{
			base.Vessel.RemoveAllManeuverNodes();
		}
		if (base.Vessel.patchedConicSolver.maneuverNodes.Count > 0)
		{
			if (!Core.Node.Enabled)
			{
				Core.Node.ExecuteAllNodes(this);
			}
			return;
		}
		Vector3d val;
		if ((double)Core.Target.Distance < (double)desiredDistance * 1.05 + 2.0)
		{
			val = Core.Target.RelativeVelocity;
			if (((Vector3d)(ref val)).magnitude < 1.0)
			{
				Users.Clear();
				Core.Thrust.ThrustOff();
				status = Localizer.Format("#MechJeb_RZauto_statu1");
				return;
			}
		}
		if ((double)Core.Target.Distance < (double)desiredDistance * 1.05 + 2.0)
		{
			double time = base.VesselState.Time;
			Vector3d dV = OrbitalManeuverCalculator.DeltaVToMatchVelocities(base.Orbit, time, Core.Target.TargetOrbit);
			base.Vessel.PlaceManeuverNode(base.Orbit, dV, time);
			status = Localizer.Format("#MechJeb_RZauto_statu2", new string[1] { desiredDistance.ToString() });
		}
		else if ((double)Core.Target.Distance < base.VesselState.Radius / 25.0)
		{
			if (base.Orbit.NextClosestApproachDistance(Core.Target.TargetOrbit, base.VesselState.Time) < (double)desiredDistance && base.Orbit.NextClosestApproachTime(Core.Target.TargetOrbit, base.VesselState.Time) < base.VesselState.Time + 150.0)
			{
				double num = base.Orbit.NextClosestApproachTime(Core.Target.TargetOrbit, base.VesselState.Time);
				Vector3d dV2 = OrbitalManeuverCalculator.DeltaVToMatchVelocities(base.Orbit, num, Core.Target.TargetOrbit);
				double num2 = base.Orbit.Separation(Core.Target.TargetOrbit, num);
				val = base.Orbit.WorldOrbitalVelocityAtUT(num) - Core.Target.TargetOrbit.WorldOrbitalVelocityAtUT(num);
				double magnitude = ((Vector3d)(ref val)).magnitude;
				if (num2 < (double)desiredDistance)
				{
					num -= Math.Sqrt(Math.Abs((double)desiredDistance * (double)desiredDistance - num2 * num2)) / magnitude;
				}
				if (magnitude > 10.0)
				{
					num -= 1.0;
				}
				base.Vessel.PlaceManeuverNode(base.Orbit, dV2, num);
				status = Localizer.Format("#MechJeb_RZauto_statu3");
			}
			else
			{
				double num3 = Core.Target.Distance / 100f;
				if (num3 > (double)maxClosingSpeed)
				{
					num3 = maxClosingSpeed;
				}
				num3 = Math.Max(0.01, num3);
				double dt = (double)Core.Target.Distance / num3;
				double num4 = base.VesselState.Time + 15.0;
				Vector3d item = OrbitalManeuverCalculator.DeltaVToInterceptAtTime(base.Orbit, num4, Core.Target.TargetOrbit, dt).v1;
				base.Vessel.PlaceManeuverNode(base.Orbit, item, num4);
				status = Localizer.Format("#MechJeb_RZauto_statu4");
			}
		}
		else if (base.Orbit.NextClosestApproachDistance(Core.Target.TargetOrbit, base.VesselState.Time) < Core.Target.TargetOrbit.semiMajorAxis / 25.0)
		{
			double num5 = base.Orbit.NextClosestApproachTime(Core.Target.TargetOrbit, base.VesselState.Time);
			Vector3d dV3 = OrbitalManeuverCalculator.DeltaVToMatchVelocities(base.Orbit, num5, Core.Target.TargetOrbit);
			val = base.Orbit.WorldPositionAtUT(num5) - Core.Target.TargetOrbit.WorldPositionAtUT(num5);
			double magnitude2 = ((Vector3d)(ref val)).magnitude;
			val = base.Orbit.WorldOrbitalVelocityAtUT(num5) - Core.Target.TargetOrbit.WorldOrbitalVelocityAtUT(num5);
			double magnitude3 = ((Vector3d)(ref val)).magnitude;
			if (magnitude2 < (double)desiredDistance)
			{
				num5 -= Math.Sqrt(Math.Abs((double)desiredDistance * (double)desiredDistance - magnitude2 * magnitude2)) / magnitude3;
			}
			if (magnitude3 > 10.0)
			{
				num5 -= 1.0;
			}
			base.Vessel.PlaceManeuverNode(base.Orbit, dV3, num5);
			status = Localizer.Format("#MechJeb_RZauto_statu5");
		}
		else if (base.Orbit.RelativeInclination(Core.Target.TargetOrbit) < 0.05 && base.Orbit.eccentricity < 0.05)
		{
			(Vector3d dV1, double UT1, Vector3d dV2, double UT2) tuple = OrbitalManeuverCalculator.DeltaVAndTimeForHohmannTransfer(base.Orbit, Core.Target.TargetOrbit, base.VesselState.Time, double.NaN, fixedTime: false, coplanar: false);
			Vector3d item2 = tuple.dV1;
			double item3 = tuple.UT1;
			double num6 = (item3 - base.VesselState.Time) / base.Orbit.period;
			double num7 = Math.Max(maxPhasingOrbits, 5.0);
			if (num6 < num7)
			{
				base.Vessel.PlaceManeuverNode(base.Orbit, item2, item3);
				status = Localizer.Format("#MechJeb_RZauto_statu6", new string[1] { num6.ToString("F2") });
				return;
			}
			double num8 = Math.Pow(1.0 + 1.25 / num7, 2.0 / 3.0);
			double num9 = Core.Target.TargetOrbit.semiMajorAxis / num8;
			double num10 = Core.Target.TargetOrbit.semiMajorAxis * num8;
			double num11 = ((num9 > base.MainBody.Radius + base.MainBody.RealMaxAtmosphereAltitude() + 3000.0 && base.Orbit.semiMajorAxis < Core.Target.TargetOrbit.semiMajorAxis) ? num9 : num10);
			if (base.Orbit.ApR < num11)
			{
				double num12 = base.VesselState.Time + 15.0;
				Vector3d dV4 = OrbitalManeuverCalculator.DeltaVToChangeApoapsis(base.Orbit, num12, num11);
				base.Vessel.PlaceManeuverNode(base.Orbit, dV4, num12);
				Orbit nextPatch = base.Vessel.patchedConicSolver.maneuverNodes[0].nextPatch;
				double num13 = nextPatch.NextApoapsisTime(num12);
				Vector3d dV5 = OrbitalManeuverCalculator.DeltaVToCircularize(nextPatch, num13);
				base.Vessel.PlaceManeuverNode(nextPatch, dV5, num13);
			}
			else if (base.Orbit.PeR > num11)
			{
				double num14 = base.VesselState.Time + 15.0;
				Vector3d dV6 = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(base.Orbit, num14, num11);
				base.Vessel.PlaceManeuverNode(base.Orbit, dV6, num14);
				Orbit nextPatch2 = base.Vessel.patchedConicSolver.maneuverNodes[0].nextPatch;
				double num15 = nextPatch2.NextPeriapsisTime(num14);
				Vector3d dV7 = OrbitalManeuverCalculator.DeltaVToCircularize(nextPatch2, num15);
				base.Vessel.PlaceManeuverNode(nextPatch2, dV7, num15);
			}
			else
			{
				double num16 = base.Orbit.NextTimeOfRadius(base.VesselState.Time, num11);
				Vector3d dV8 = OrbitalManeuverCalculator.DeltaVToCircularize(base.Orbit, num16);
				base.Vessel.PlaceManeuverNode(base.Orbit, dV8, num16);
			}
			status = Localizer.Format("#MechJeb_RZauto_statu7", new string[3]
			{
				num6.ToString("F1"),
				maxPhasingOrbits.Text,
				Statics.ToSI(num11 - base.MainBody.Radius, 0, int.MaxValue)
			});
		}
		else if (base.Orbit.RelativeInclination(Core.Target.TargetOrbit) < 0.05)
		{
			bool flag = base.Orbit.eccentricity > 1.0 || Math.Abs(base.Orbit.PeR - Core.Target.TargetOrbit.semiMajorAxis) < Math.Abs(base.Orbit.ApR - Core.Target.TargetOrbit.semiMajorAxis);
			double num17 = ((!flag) ? base.Orbit.NextApoapsisTime(base.VesselState.Time) : Math.Max(base.VesselState.Time, base.Orbit.NextPeriapsisTime(base.VesselState.Time)));
			Vector3d dV9 = OrbitalManeuverCalculator.DeltaVToCircularize(base.Orbit, num17);
			base.Vessel.PlaceManeuverNode(base.Orbit, dV9, num17);
			status = Localizer.Format("#MechJeb_RZauto_statu8");
		}
		else
		{
			bool flag2 = ((base.Orbit.eccentricity < 1.0) ? ((base.Orbit.TimeOfAscendingNode(Core.Target.TargetOrbit, base.VesselState.Time) < base.Orbit.TimeOfDescendingNode(Core.Target.TargetOrbit, base.VesselState.Time)) ? true : false) : (base.Orbit.AscendingNodeExists(Core.Target.TargetOrbit) ? true : false));
			VesselExtensions.PlaceManeuverNode(dV: (!flag2) ? OrbitalManeuverCalculator.DeltaVAndTimeToMatchPlanesDescending(base.Orbit, Core.Target.TargetOrbit, base.VesselState.Time, out var burnUT) : OrbitalManeuverCalculator.DeltaVAndTimeToMatchPlanesAscending(base.Orbit, Core.Target.TargetOrbit, base.VesselState.Time, out burnUT), vessel: base.Vessel, ignoredParameterThatNeedsDeleting: base.Orbit, UT: burnUT);
			status = Localizer.Format("#MechJeb_RZauto_statu9");
		}
	}
}
