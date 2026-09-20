using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech.Landing;

public class PlaneChange : AutopilotStep
{
	private bool _planeChangeTriggered;

	private double _planeChangeDVLeft;

	public PlaneChange(MechJebCore core)
		: base(core)
	{
	}

	private Vector3d ComputePlaneChange()
	{
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		//IL_0084: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ee: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		//IL_0108: Unknown result type (might be due to invalid IL or missing references)
		//IL_010d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0115: Unknown result type (might be due to invalid IL or missing references)
		//IL_011a: Unknown result type (might be due to invalid IL or missing references)
		//IL_011c: Unknown result type (might be due to invalid IL or missing references)
		//IL_011d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0122: Unknown result type (might be due to invalid IL or missing references)
		//IL_0127: Unknown result type (might be due to invalid IL or missing references)
		//IL_012b: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = ((PartModule)Core).vessel.mainBody.GetWorldSurfacePosition((double)Core.Target.targetLatitude, (double)Core.Target.targetLongitude, 0.0) - base.MainBody.position;
		Vector3d val2 = Core.VesselState.CoM - ((PartModule)Core).vessel.mainBody.position;
		double num = Vector3d.Angle(val, val2);
		double num2 = base.Orbit.TimeOfTrueAnomaly(((PartModule)Core).vessel.orbit.trueAnomaly * (180.0 / Math.PI) + num, base.VesselState.Time) - base.VesselState.Time;
		Vector3d val3 = Vector3d.op_Implicit(Quaternion.AngleAxis((float)(360.0 * num2 / base.MainBody.rotationPeriod), Vector3d.op_Implicit(base.MainBody.angularVelocity)) * Vector3d.op_Implicit(val));
		Vector3d val4 = Vector3d.Exclude(base.VesselState.Up, val3 - val2);
		return ((Vector3d)(ref val4)).normalized;
	}

	public override AutopilotStep Drive(FlightCtrlState s)
	{
		if (_planeChangeTriggered && Core.Attitude.attitudeAngleFromTarget() < 2.0)
		{
			Core.Thrust.RequestActiveThrottle(Mathf.Clamp01((float)(_planeChangeDVLeft / (2.0 * Core.VesselState.MaxThrustAcceleration))));
		}
		else if (_planeChangeTriggered && Core.Attitude.attitudeAngleFromTarget() < 10.0 && Core.Thrust.LimiterMinThrottle)
		{
			Core.Thrust.RequestActiveThrottle(0f);
		}
		else
		{
			Core.Thrust.ThrustOff();
		}
		return this;
	}

	public override AutopilotStep OnFixedUpdate()
	{
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		//IL_006a: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0101: Unknown result type (might be due to invalid IL or missing references)
		//IL_010c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0111: Unknown result type (might be due to invalid IL or missing references)
		//IL_0116: Unknown result type (might be due to invalid IL or missing references)
		//IL_011b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0120: Unknown result type (might be due to invalid IL or missing references)
		//IL_0122: Unknown result type (might be due to invalid IL or missing references)
		//IL_012a: Unknown result type (might be due to invalid IL or missing references)
		//IL_012f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0134: Unknown result type (might be due to invalid IL or missing references)
		//IL_013c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0147: Unknown result type (might be due to invalid IL or missing references)
		//IL_014c: Unknown result type (might be due to invalid IL or missing references)
		//IL_014e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0153: Unknown result type (might be due to invalid IL or missing references)
		//IL_0158: Unknown result type (might be due to invalid IL or missing references)
		//IL_0164: Unknown result type (might be due to invalid IL or missing references)
		//IL_016c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0193: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = base.MainBody.GetWorldSurfacePosition((double)Core.Target.targetLatitude, (double)Core.Target.targetLongitude, 0.0) - base.MainBody.position;
		Vector3d val2 = base.VesselState.CoM - base.MainBody.position;
		double num = Vector3d.Angle(val, val2);
		bool flag = Vector3d.Dot(val - val2, base.VesselState.OrbitalVelocity) > 0.0;
		if (!_planeChangeTriggered && flag && num > 80.0 && num < 90.0)
		{
			if (!MuUtils.PhysicsRunning())
			{
				Core.Warp.MinimumWarp(instant: true);
			}
			_planeChangeTriggered = true;
		}
		if (_planeChangeTriggered)
		{
			Vector3d val3 = ComputePlaneChange();
			Vector3d val4 = Vector3d.op_Implicit(Quaternion.FromToRotation(Vector3d.op_Implicit(base.VesselState.HorizontalOrbit), Vector3d.op_Implicit(val3)) * Vector3d.op_Implicit(base.VesselState.OrbitalVelocity));
			Vector3d val5 = val4 - base.VesselState.OrbitalVelocity;
			Vector3d direction = Vector3d.Exclude(base.VesselState.Up, Vector3d.Exclude(base.VesselState.OrbitalVelocity, val5));
			_planeChangeDVLeft = Math.PI / 180.0 * Vector3d.Angle(val4, base.VesselState.OrbitalVelocity) * base.VesselState.SpeedOrbitalHorizontal;
			Core.Attitude.attitudeTo(direction, AttitudeReference.INERTIAL, Core.Landing);
			base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status14", new string[1] { _planeChangeDVLeft.ToString("F0") });
			if (_planeChangeDVLeft < 0.10000000149011612)
			{
				Core.Thrust.ThrustOff();
				return new DeorbitBurn(Core);
			}
		}
		else
		{
			if (Core.Node.Autowarp)
			{
				Core.Warp.WarpRegularAtRate((float)(base.Orbit.period / 6.0));
			}
			base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status15");
		}
		return this;
	}
}
