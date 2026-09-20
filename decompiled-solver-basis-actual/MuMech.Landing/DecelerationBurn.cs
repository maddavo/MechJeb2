using System;
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace MuMech.Landing;

public class DecelerationBurn : AutopilotStep
{
	private bool _decelerationBurnTriggered;

	public DecelerationBurn(MechJebCore core)
		: base(core)
	{
	}

	public override AutopilotStep OnFixedUpdate()
	{
		//IL_01fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0200: Unknown result type (might be due to invalid IL or missing references)
		//IL_0205: Unknown result type (might be due to invalid IL or missing references)
		//IL_0224: Unknown result type (might be due to invalid IL or missing references)
		//IL_0229: Unknown result type (might be due to invalid IL or missing references)
		//IL_0258: Unknown result type (might be due to invalid IL or missing references)
		//IL_025c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0261: Unknown result type (might be due to invalid IL or missing references)
		//IL_0266: Unknown result type (might be due to invalid IL or missing references)
		//IL_026b: Unknown result type (might be due to invalid IL or missing references)
		//IL_026f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0274: Unknown result type (might be due to invalid IL or missing references)
		//IL_027b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0286: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		//IL_0105: Unknown result type (might be due to invalid IL or missing references)
		//IL_0114: Unknown result type (might be due to invalid IL or missing references)
		//IL_0119: Unknown result type (might be due to invalid IL or missing references)
		//IL_011e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0123: Unknown result type (might be due to invalid IL or missing references)
		//IL_0127: Unknown result type (might be due to invalid IL or missing references)
		//IL_012c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0139: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_04a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f4: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_035a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0365: Unknown result type (might be due to invalid IL or missing references)
		//IL_0388: Unknown result type (might be due to invalid IL or missing references)
		//IL_0398: Unknown result type (might be due to invalid IL or missing references)
		//IL_039d: Unknown result type (might be due to invalid IL or missing references)
		//IL_03be: Unknown result type (might be due to invalid IL or missing references)
		//IL_03c9: Unknown result type (might be due to invalid IL or missing references)
		if (base.VesselState.AltitudeASL < Core.Landing.DecelerationEndAltitude() + 5.0)
		{
			Core.Warp.MinimumWarp();
			if (Core.Landing.UseAtmosphereToBrake())
			{
				return new FinalDescent(Core);
			}
			return new KillHorizontalVelocity(Core);
		}
		double num = (Core.Landing.Prediction.Trajectory.Any() ? Core.Landing.Prediction.Trajectory.First().UT : base.VesselState.Time);
		if (num - base.VesselState.Time > 5.0 && !_decelerationBurnTriggered)
		{
			Core.Thrust.ThrustOff();
			base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status4");
			Vector3d val = -base.Orbit.WorldOrbitalVelocityAtUT(num);
			val += base.MainBody.getRFrmVel(base.Orbit.WorldPositionAtUT(num));
			val = ((Vector3d)(ref val)).normalized;
			Core.Attitude.attitudeTo(val, AttitudeReference.INERTIAL, Core.Landing);
			if (Core.Attitude.attitudeAngleFromTarget() < 5.0 && (double)((Vector3)(ref ((PartModule)Core).vessel.angularVelocity)).magnitude < 0.001 && Core.Node.Autowarp)
			{
				Core.Warp.WarpToUT(num - 5.0);
			}
			else if (!MuUtils.PhysicsRunning())
			{
				Core.Warp.MinimumWarp();
			}
			return this;
		}
		if (!_decelerationBurnTriggered)
		{
			_decelerationBurnTriggered = true;
		}
		Vector3d val2 = -((Vector3d)(ref base.VesselState.SurfaceVelocity)).normalized;
		Vector3d val3 = Core.Landing.ComputeCourseCorrection(allowPrograde: false);
		double val4 = ((Vector3d)(ref val3)).magnitude / (2.0 * base.VesselState.LimitedMaxThrustAcceleration);
		val4 = Math.Min(0.1, val4);
		Vector3d val5 = val2 + val4 * ((Vector3d)(ref val3)).normalized;
		val2 = ((Vector3d)(ref val5)).normalized;
		if (Vector3d.Dot(base.VesselState.SurfaceVelocity, base.VesselState.Up) > 0.0 || Vector3d.Dot(base.VesselState.Forward, val2) < 0.75)
		{
			Core.Thrust.RequestActiveThrottle(0f);
			base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status5");
		}
		else
		{
			double num2 = base.VesselState.SpeedSurface * (double)Math.Sign(Vector3d.Dot(base.VesselState.SurfaceVelocity, base.VesselState.Up));
			double num3 = 0.0 - Core.Landing.MaxAllowedSpeed();
			double num4 = 0.0 - Core.Landing.MaxAllowedSpeedAfterDt(base.VesselState.DeltaT);
			double num5 = (0.0 - base.VesselState.LocalGravity) * Math.Abs(Vector3d.Dot(((Vector3d)(ref base.VesselState.SurfaceVelocity)).normalized, base.VesselState.Up));
			double num6 = base.VesselState.MaxThrustAcceleration * Vector3d.Dot(base.VesselState.Forward, -((Vector3d)(ref base.VesselState.SurfaceVelocity)).normalized) - base.VesselState.LocalGravity * Math.Abs(Vector3d.Dot(((Vector3d)(ref base.VesselState.SurfaceVelocity)).normalized, base.VesselState.Up));
			double num7 = (num3 - num2) / 0.3 + (num4 - num3) / base.VesselState.DeltaT;
			if (num6 - num5 > 0.0)
			{
				Core.Thrust.RequestActiveThrottle(Mathf.Clamp((float)((num7 - num5) / (num6 - num5)), 0f, 1f));
			}
			else
			{
				Core.Thrust.RequestActiveThrottle(0f);
			}
			base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status6", new string[1] { (num3 >= double.MaxValue) ? "∞" : Math.Abs(num3).ToString("F1") });
		}
		Core.Attitude.attitudeTo(val2, AttitudeReference.INERTIAL, Core.Landing);
		return this;
	}
}
