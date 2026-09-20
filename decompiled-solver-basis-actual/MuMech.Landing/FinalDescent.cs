using System;
using KSP.Localization;

namespace MuMech.Landing;

public class FinalDescent : AutopilotStep
{
	private const double MinimumTerminalDescentSpeed = 5.0;

	private const double MaximumTerminalDescentSpeed = 25.0;

	private const double TerminalDescentGravityTime = 4.0;

	private IDescentSpeedPolicy _aggressivePolicy;

	public FinalDescent(MechJebCore core)
		: base(core)
	{
	}

	public override AutopilotStep OnFixedUpdate()
	{
		return this;
	}

	public override AutopilotStep Drive(FlightCtrlState s)
	{
		//IL_00d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_0233: Unknown result type (might be due to invalid IL or missing references)
		//IL_0257: Unknown result type (might be due to invalid IL or missing references)
		//IL_028d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0292: Unknown result type (might be due to invalid IL or missing references)
		//IL_0297: Unknown result type (might be due to invalid IL or missing references)
		//IL_029c: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0304: Unknown result type (might be due to invalid IL or missing references)
		//IL_0309: Unknown result type (might be due to invalid IL or missing references)
		//IL_0314: Unknown result type (might be due to invalid IL or missing references)
		//IL_018f: Unknown result type (might be due to invalid IL or missing references)
		//IL_019a: Unknown result type (might be due to invalid IL or missing references)
		//IL_019f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0102: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bf: Unknown result type (might be due to invalid IL or missing references)
		if (base.Vessel.LandedOrSplashed)
		{
			Core.Landing.StopLanding();
			return null;
		}
		double num = Math.Min(base.VesselState.AltitudeBottom, Math.Min(base.VesselState.AltitudeASL, base.VesselState.AltitudeTrue));
		if (base.VesselState.LimitedMaxThrustAcceleration < ((Vector3d)(ref base.VesselState.GravityForce)).magnitude)
		{
			Core.Thrust.Tmode = MechJebModuleThrustController.TMode.KEEP_VERTICAL;
			Core.Thrust.TransKillH = true;
			Core.Thrust.TransSpdAct = 0f;
		}
		else if (num > 300.0)
		{
			if (((Vector3d)(ref base.VesselState.SurfaceVelocity)).magnitude > 5.0 && Vector3d.Angle(base.VesselState.SurfaceVelocity, base.VesselState.Up) < 80.0)
			{
				Core.Attitude.attitudeTo(Vector3d.up, AttitudeReference.SURFACE_NORTH, null);
				Core.Thrust.Tmode = MechJebModuleThrustController.TMode.DIRECT;
				Core.Thrust.TransSpdAct = (Core.Thrust.LimiterMinThrottle ? (100f * (float)(double)Core.Thrust.MinThrottle) : 0f);
			}
			else if (((Vector3d)(ref base.VesselState.SurfaceVelocity)).magnitude > 5.0 && Vector3d.Angle(base.VesselState.Forward, -base.VesselState.SurfaceVelocity) > 45.0)
			{
				Core.Attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, null);
				Core.Thrust.Tmode = MechJebModuleThrustController.TMode.DIRECT;
				Core.Thrust.TransSpdAct = (Core.Thrust.LimiterMinThrottle ? (100f * (float)(double)Core.Thrust.MinThrottle) : 0f);
			}
			else
			{
				Core.Attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, null);
				Core.Thrust.Tmode = MechJebModuleThrustController.TMode.KEEP_SURFACE;
				Vector3d worldPosition = base.VesselState.CoM + ((Vector3d)(ref base.VesselState.SurfaceVelocity)).sqrMagnitude / (2.0 * base.VesselState.LimitedMaxThrustAcceleration) * ((Vector3d)(ref base.VesselState.SurfaceVelocity)).normalized;
				double terrainRadius = base.MainBody.Radius + base.MainBody.TerrainAltitude(worldPosition);
				_aggressivePolicy = new GravityTurnDescentSpeedPolicy(terrainRadius, base.MainBody.GeeASL * 9.81, base.VesselState.LimitedMaxThrustAcceleration);
				Core.Thrust.TransSpdAct = (float)_aggressivePolicy.MaxAllowedSpeed(base.VesselState.CoM - base.MainBody.position, base.VesselState.SurfaceVelocity);
			}
		}
		else
		{
			double num2 = Math.Max(0.0, base.VesselState.LimitedMaxThrustAcceleration - base.VesselState.LocalGravity);
			double val = Math.Sqrt(2.0 * num2 * Math.Max(0.0, num)) * 0.9;
			double val2 = Math.Max(5.0, Math.Min(25.0, 4.0 * base.VesselState.LocalGravity));
			double val3 = 0.0 - Math.Min(val, val2);
			Core.Thrust.Tmode = MechJebModuleThrustController.TMode.KEEP_VERTICAL;
			Core.Thrust.TransSpdAct = (float)Math.Min(0.0 - (double)Core.Landing.TouchdownSpeed, val3);
			Core.Thrust.TransKillH = true;
		}
		base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status9", new string[1] { base.VesselState.AltitudeBottom.ToString("F0") });
		return this;
	}
}
