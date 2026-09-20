using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech.Landing;

public class LowDeorbitBurn : AutopilotStep
{
	private bool _deorbitBurnTriggered;

	private double _lowDeorbitBurnMaxThrottle;

	private bool _lowDeorbitEndConditionSet;

	private bool _lowDeorbitEndOnLandingSiteNearer;

	private const double LOW_DEORBIT_BURN_TRIGGER_FACTOR = 2.0;

	public LowDeorbitBurn(MechJebCore core)
		: base(core)
	{
	}

	public override AutopilotStep Drive(FlightCtrlState s)
	{
		if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 5.0)
		{
			Core.Thrust.RequestActiveThrottle(Mathf.Clamp01((float)_lowDeorbitBurnMaxThrottle), enforceMinimum: true, allowZero: true);
		}
		else if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 10.0 && Core.Thrust.LimiterMinThrottle)
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
		//IL_0065: Unknown result type (might be due to invalid IL or missing references)
		//IL_0075: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_008a: Unknown result type (might be due to invalid IL or missing references)
		//IL_008f: Unknown result type (might be due to invalid IL or missing references)
		//IL_017d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0182: Unknown result type (might be due to invalid IL or missing references)
		//IL_0187: Unknown result type (might be due to invalid IL or missing references)
		//IL_0498: Unknown result type (might be due to invalid IL or missing references)
		//IL_01be: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01de: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0209: Unknown result type (might be due to invalid IL or missing references)
		//IL_0214: Unknown result type (might be due to invalid IL or missing references)
		//IL_0219: Unknown result type (might be due to invalid IL or missing references)
		//IL_021e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0223: Unknown result type (might be due to invalid IL or missing references)
		//IL_0227: Unknown result type (might be due to invalid IL or missing references)
		//IL_022c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0237: Unknown result type (might be due to invalid IL or missing references)
		//IL_0239: Unknown result type (might be due to invalid IL or missing references)
		//IL_023b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0240: Unknown result type (might be due to invalid IL or missing references)
		//IL_0245: Unknown result type (might be due to invalid IL or missing references)
		//IL_0273: Unknown result type (might be due to invalid IL or missing references)
		//IL_0275: Unknown result type (might be due to invalid IL or missing references)
		//IL_0277: Unknown result type (might be due to invalid IL or missing references)
		//IL_027c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0280: Unknown result type (might be due to invalid IL or missing references)
		//IL_0285: Unknown result type (might be due to invalid IL or missing references)
		//IL_028d: Unknown result type (might be due to invalid IL or missing references)
		//IL_029d: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0259: Unknown result type (might be due to invalid IL or missing references)
		//IL_026c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0271: Unknown result type (might be due to invalid IL or missing references)
		//IL_02e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_03bc: Unknown result type (might be due to invalid IL or missing references)
		//IL_03cc: Unknown result type (might be due to invalid IL or missing references)
		double num = Math.Pow(base.VesselState.SpeedSurfaceHorizontal, 2.0) / (2.0 * base.VesselState.LimitedMaxThrustAcceleration);
		double num2 = 2.0 * num;
		double num3 = base.VesselState.AltitudeASL - Core.Landing.DecelerationEndAltitude();
		if (num2 < num3)
		{
			num2 = num3;
		}
		Vector3d val = Vector3d.Exclude(base.VesselState.Up, Core.Target.GetPositionTargetPosition() - base.VesselState.CoM);
		double magnitude = ((Vector3d)(ref val)).magnitude;
		if (!_deorbitBurnTriggered && magnitude < num2)
		{
			if (!MuUtils.PhysicsRunning())
			{
				Core.Warp.MinimumWarp(instant: true);
			}
			_deorbitBurnTriggered = true;
		}
		base.Status = Localizer.Format(_deorbitBurnTriggered ? "#MechJeb_LandingGuidance_Status11" : "#MechJeb_LandingGuidance_Status12");
		if (!_deorbitBurnTriggered && Core.Node.Autowarp && magnitude > 2.0 * num2 && (double)((Vector3)(ref ((PartModule)Core).vessel.angularVelocity)).magnitude < 0.001)
		{
			Core.Warp.WarpRegularAtRate((float)(base.Orbit.period / 6.0));
		}
		if (magnitude < num2 && !MuUtils.PhysicsRunning())
		{
			Core.Warp.MinimumWarp();
		}
		Vector3d val2 = -((Vector3d)(ref base.VesselState.SurfaceVelocity)).normalized;
		_lowDeorbitBurnMaxThrottle = 1.0;
		if (_deorbitBurnTriggered && Core.Landing.PredictionReady)
		{
			val = Vector3d.Exclude(base.VesselState.Up, Core.Landing.LandingSite - base.VesselState.CoM);
			Vector3d normalized = ((Vector3d)(ref val)).normalized;
			val = Vector3d.Exclude(base.VesselState.Up, Core.Target.GetPositionTargetPosition() - base.VesselState.CoM);
			Vector3d normalized2 = ((Vector3d)(ref val)).normalized;
			Vector3d val3 = 4.0 * (normalized2 - normalized);
			if (((Vector3d)(ref val3)).magnitude > 0.1)
			{
				val3 *= 0.1 / ((Vector3d)(ref val3)).magnitude;
			}
			val = val2 + val3;
			val2 = ((Vector3d)(ref val)).normalized;
			val = Vector3d.Exclude(base.VesselState.Up, Core.Landing.LandingSite - base.VesselState.CoM);
			double magnitude2 = ((Vector3d)(ref val)).magnitude;
			double num4 = Core.Landing.MaxAllowedSpeed();
			if (!_lowDeorbitEndConditionSet && Vector3d.Distance(Core.Landing.LandingSite, base.VesselState.CoM) < base.MainBody.Radius + base.VesselState.AltitudeASL)
			{
				_lowDeorbitEndOnLandingSiteNearer = magnitude2 > magnitude;
				_lowDeorbitEndConditionSet = true;
			}
			_lowDeorbitBurnMaxThrottle = 1.0;
			if (base.Orbit.PeA < 0.0)
			{
				if (magnitude2 > magnitude)
				{
					if (_lowDeorbitEndConditionSet && !_lowDeorbitEndOnLandingSiteNearer)
					{
						Core.Thrust.ThrustOff();
						return new DecelerationBurn(Core);
					}
					double num5 = Core.Landing.MaxAllowedSpeedAfterDt(base.VesselState.DeltaT);
					double num6 = base.VesselState.SpeedSurface + base.VesselState.DeltaT * Vector3d.Dot(base.VesselState.GravityForce, ((Vector3d)(ref base.VesselState.SurfaceVelocity)).normalized);
					double num7 = ((!(base.VesselState.SpeedSurface < num4)) ? ((num6 - num5) / (base.VesselState.DeltaT * base.VesselState.MaxThrustAcceleration)) : 0.0);
					_lowDeorbitBurnMaxThrottle = num7 + 1.0 * (magnitude2 / magnitude - 1.0) + 0.2;
				}
				else
				{
					if (_lowDeorbitEndConditionSet && _lowDeorbitEndOnLandingSiteNearer)
					{
						Core.Thrust.ThrustOff();
						return new DecelerationBurn(Core);
					}
					_lowDeorbitBurnMaxThrottle = 0.0;
					base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status13");
				}
			}
		}
		Core.Attitude.attitudeTo(val2, AttitudeReference.INERTIAL, Core.Landing);
		return this;
	}
}
