using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech.Landing;

public class DeorbitBurn : AutopilotStep
{
	private const double MinimumDeorbitAcceleration = 3.0;

	private const double MaximumDeorbitTwr = 4.0;

	private const double DeorbitBurnTimeConstant = 0.5;

	private const double MinimumTerminalDeltaV = 0.05;

	private bool _deorbitBurnTriggered;

	private double _deorbitThrottle;

	public DeorbitBurn(MechJebCore core)
		: base(core)
	{
	}

	public override AutopilotStep Drive(FlightCtrlState s)
	{
		if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 5.0)
		{
			Core.Thrust.RequestActiveThrottle((float)_deorbitThrottle, enforceMinimum: true, allowZero: true);
		}
		else if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 10.0 && Core.Thrust.LimiterMinThrottle)
		{
			Core.Thrust.RequestActiveThrottle(0f, enforceMinimum: true, allowZero: true);
		}
		else
		{
			Core.Thrust.ThrustOff();
		}
		return this;
	}

	public override AutopilotStep OnFixedUpdate()
	{
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0071: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ef: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0104: Unknown result type (might be due to invalid IL or missing references)
		//IL_010c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0111: Unknown result type (might be due to invalid IL or missing references)
		//IL_0116: Unknown result type (might be due to invalid IL or missing references)
		//IL_011b: Unknown result type (might be due to invalid IL or missing references)
		//IL_011c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0121: Unknown result type (might be due to invalid IL or missing references)
		//IL_0126: Unknown result type (might be due to invalid IL or missing references)
		//IL_012b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0132: Unknown result type (might be due to invalid IL or missing references)
		//IL_0137: Unknown result type (might be due to invalid IL or missing references)
		//IL_0138: Unknown result type (might be due to invalid IL or missing references)
		//IL_013d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0145: Unknown result type (might be due to invalid IL or missing references)
		//IL_014a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0152: Unknown result type (might be due to invalid IL or missing references)
		//IL_0157: Unknown result type (might be due to invalid IL or missing references)
		//IL_015c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0161: Unknown result type (might be due to invalid IL or missing references)
		//IL_0165: Unknown result type (might be due to invalid IL or missing references)
		//IL_016a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0172: Unknown result type (might be due to invalid IL or missing references)
		//IL_017d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0182: Unknown result type (might be due to invalid IL or missing references)
		//IL_0187: Unknown result type (might be due to invalid IL or missing references)
		//IL_0189: Unknown result type (might be due to invalid IL or missing references)
		//IL_018b: Unknown result type (might be due to invalid IL or missing references)
		//IL_018c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0191: Unknown result type (might be due to invalid IL or missing references)
		//IL_019a: Unknown result type (might be due to invalid IL or missing references)
		//IL_019c: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ee: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_03a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_0258: Unknown result type (might be due to invalid IL or missing references)
		//IL_025a: Unknown result type (might be due to invalid IL or missing references)
		//IL_025c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0261: Unknown result type (might be due to invalid IL or missing references)
		//IL_0270: Unknown result type (might be due to invalid IL or missing references)
		if (base.Orbit.ApA < base.MainBody.RealMaxAtmosphereAltitude())
		{
			Core.Thrust.ThrustOff();
			return new CourseCorrection(Core);
		}
		Vector3d val = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(base.Orbit, base.VesselState.Time, 0.9 * base.MainBody.Radius);
		double num = base.Orbit.PerturbedOrbit(base.VesselState.Time, val).NextTimeOfRadius(base.VesselState.Time, base.MainBody.Radius) - base.VesselState.Time;
		double num2 = 360.0 * num / base.MainBody.rotationPeriod;
		Vector3d val2 = base.MainBody.GetWorldSurfacePosition((double)Core.Target.targetLatitude, (double)Core.Target.targetLongitude, 0.0) - base.MainBody.position;
		Vector3d val3 = Vector3d.op_Implicit(Quaternion.AngleAxis((float)num2, Vector3d.op_Implicit(base.MainBody.angularVelocity)) * Vector3d.op_Implicit(val2));
		Vector3d val4 = base.MainBody.position + val3;
		Vector3d val5 = Vector3d.Exclude(base.VesselState.Up, val4 - base.VesselState.CoM);
		Vector3d normalized = ((Vector3d)(ref val5)).normalized;
		Vector3d val6 = Vector3d.Exclude(base.VesselState.Up, base.VesselState.OrbitalVelocity);
		val5 = val6 + val;
		Vector3d val7 = ((Vector3d)(ref val5)).magnitude * normalized;
		Vector3d val8 = base.VesselState.CoM - base.MainBody.position;
		double num3 = Vector3d.Angle(base.Orbit.OrbitNormal(), val3);
		num3 = Math.Min(num3, 180.0 - num3);
		double num4 = Vector3d.Angle(val8, val3);
		double num5 = Vector3d.Angle(val6, normalized);
		if (num3 < 10.0 || (num4 < 90.0 && num4 > 60.0 && num5 < 90.0))
		{
			_deorbitBurnTriggered = true;
		}
		if (_deorbitBurnTriggered)
		{
			if (!MuUtils.PhysicsRunning())
			{
				Core.Warp.MinimumWarp();
			}
			Vector3d val9 = val7 - val6;
			Core.Attitude.attitudeTo(((Vector3d)(ref val9)).normalized, AttitudeReference.INERTIAL, Core.Landing);
			double limitedMaxThrustAcceleration = base.VesselState.LimitedMaxThrustAcceleration;
			if (limitedMaxThrustAcceleration <= 0.0)
			{
				Core.Thrust.ThrustOff();
				return this;
			}
			double val10 = Math.Max(3.0, 4.0 * base.MainBody.GeeASL * 9.81);
			double num6 = Math.Min(limitedMaxThrustAcceleration, val10);
			double num7 = base.VesselState.CurrentThrustAcceleration * base.VesselState.MaxEngineResponseTime;
			double num8 = Math.Max(0.0, (((Vector3d)(ref val9)).magnitude - num7) / 0.5);
			_deorbitThrottle = Math.Min(num8 / limitedMaxThrustAcceleration, num6 / limitedMaxThrustAcceleration);
			double num9 = Math.Max(0.05, num6 * ((double)TimeWarp.fixedDeltaTime + base.VesselState.MaxEngineResponseTime));
			if (((Vector3d)(ref val9)).magnitude <= num9)
			{
				Core.Thrust.ThrustOff();
				return new CourseCorrection(Core);
			}
			base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status7");
		}
		else
		{
			Core.Attitude.attitudeTo(Vector3d.back, AttitudeReference.ORBIT, Core.Landing);
			if (Core.Node.Autowarp)
			{
				Core.Warp.WarpRegularAtRate((float)(base.Orbit.period / 10.0));
			}
			base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status8");
		}
		return this;
	}
}
