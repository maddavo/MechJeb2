using System;
using KSP.Localization;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModuleAscentClassicAutopilot : MechJebModuleAscentBaseAutopilot
{
	private enum AscentMode
	{
		VERTICAL_ASCENT,
		GRAVITY_TURN,
		COAST_TO_APOAPSIS,
		EXIT
	}

	private double _actualTurnStart;

	private AscentMode _mode;

	private double _desiredHeading;

	private double _desiredPitch;

	public MechJebModuleAscentClassicAutopilot(MechJebCore core)
		: base(core)
	{
	}

	protected override void OnModuleEnabled()
	{
		base.OnModuleEnabled();
		_mode = AscentMode.VERTICAL_ASCENT;
	}

	protected override void OnModuleDisabled()
	{
		base.OnModuleDisabled();
		base.AscentSettings.Enabled = false;
	}

	public double VerticalAscentEnd()
	{
		if (!base.AscentSettings.AutoPath)
		{
			return base.AscentSettings.TurnStartAltitude;
		}
		return base.AscentSettings.AutoTurnStartAltitude;
	}

	private double SpeedAscentEnd()
	{
		if (!base.AscentSettings.AutoPath)
		{
			return base.AscentSettings.TurnStartVelocity;
		}
		return base.AscentSettings.AutoTurnStartVelocity;
	}

	private bool IsVerticalAscent(double altitude, double velocity)
	{
		_actualTurnStart = Math.Min(_actualTurnStart, base.AscentSettings.AutoTurnStartAltitude);
		if (altitude < VerticalAscentEnd() && velocity < SpeedAscentEnd())
		{
			_actualTurnStart = Math.Max(_actualTurnStart, altitude);
			return true;
		}
		return false;
	}

	public double FlightPathAngle(double altitude, double velocity)
	{
		double num = (base.AscentSettings.AutoPath ? base.AscentSettings.AutoTurnEndAltitude : ((double)base.AscentSettings.TurnEndAltitude));
		if (IsVerticalAscent(altitude, velocity))
		{
			return 90.0;
		}
		if (altitude > num)
		{
			return base.AscentSettings.TurnEndAngle;
		}
		return Mathf.Clamp((float)(90.0 - Math.Pow((altitude - _actualTurnStart) / (num - _actualTurnStart), base.AscentSettings.TurnShapeExponent) * (90.0 - (double)base.AscentSettings.TurnEndAngle)), 0.01f, 89.99f);
	}

	protected override bool DriveAscent2()
	{
		switch (_mode)
		{
		case AscentMode.VERTICAL_ASCENT:
			DriveVerticalAscent();
			break;
		case AscentMode.GRAVITY_TURN:
			DriveGravityTurn();
			break;
		case AscentMode.COAST_TO_APOAPSIS:
			DriveCoastToApoapsis();
			break;
		case AscentMode.EXIT:
			return false;
		}
		return true;
	}

	private void DriveVerticalAscent()
	{
		if (!IsVerticalAscent(base.VesselState.AltitudeTrue, base.VesselState.SpeedSurface))
		{
			_mode = AscentMode.GRAVITY_TURN;
		}
		if (base.Orbit.ApA > (double)base.AscentSettings.DesiredOrbitAltitude)
		{
			_mode = AscentMode.COAST_TO_APOAPSIS;
		}
		VerticalHeadingTo(OrbitalManeuverCalculator.HeadingForLaunchInclination(base.Vessel.orbit, base.AscentSettings.DesiredInclination, base.AscentSettings.DesiredOrbitAltitude.Val));
		Core.Thrust.TargetThrottle = 1f;
		if (!base.Vessel.LiftedOff() || base.Vessel.Landed)
		{
			Status = Localizer.Format("#MechJeb_Ascent_status6");
		}
		else
		{
			Status = Localizer.Format("#MechJeb_Ascent_status18");
		}
	}

	private void DriveGravityTurn()
	{
		if (base.Orbit.ApA > (double)base.AscentSettings.DesiredOrbitAltitude)
		{
			_mode = AscentMode.COAST_TO_APOAPSIS;
			return;
		}
		if (IsVerticalAscent(base.VesselState.AltitudeTrue, base.VesselState.SpeedSurface))
		{
			_mode = AscentMode.VERTICAL_ASCENT;
			return;
		}
		Core.Thrust.TargetThrottle = ThrottleToRaiseApoapsis(base.Orbit.ApR, (double)base.AscentSettings.DesiredOrbitAltitude + base.MainBody.Radius);
		if (Core.Thrust.TargetThrottle < 1f)
		{
			AttitudeTo(_desiredPitch * (180.0 / Math.PI), _desiredHeading);
			Status = Localizer.Format("#MechJeb_Ascent_status21");
			return;
		}
		_desiredPitch = FlightPathAngle(base.VesselState.AltitudeASL, base.VesselState.SpeedSurface) * (Math.PI / 180.0);
		if (base.AscentSettings.CorrectiveSteering)
		{
			double num = Math.Atan2(base.VesselState.SpeedVertical, base.VesselState.SpeedSurfaceHorizontal);
			double num2 = _desiredPitch - num;
			double num3 = ((Vector3d)(ref base.VesselState.SurfaceVelocity)).magnitude * 0.02 / base.VesselState.ThrustAccel(Core.Thrust.TargetThrottle);
			num3 = Statics.Clamp(num3, 0.1, 1.0);
			double num4 = Statics.Clamp(Math.Asin((double)base.AscentSettings.CorrectiveSteeringGain * num3 * num2), -Math.PI / 6.0, Math.PI / 6.0);
			_desiredPitch = Statics.Clamp(_desiredPitch + num4, -Math.PI / 2.0, Math.PI / 2.0);
		}
		_desiredHeading = OrbitalManeuverCalculator.HeadingForLaunchInclination(base.Vessel.orbit, base.AscentSettings.DesiredInclination, base.AscentSettings.DesiredOrbitAltitude.Val);
		AttitudeTo(_desiredPitch * (180.0 / Math.PI), _desiredHeading);
		Status = Localizer.Format("#MechJeb_Ascent_status22");
	}

	private void DriveCoastToApoapsis()
	{
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		Core.Thrust.TargetThrottle = 0f;
		if (base.VesselState.AltitudeASL > base.MainBody.RealMaxAtmosphereAltitude())
		{
			_mode = AscentMode.EXIT;
			Core.Warp.MinimumWarp();
			return;
		}
		Core.Thrust.TargetThrottle = 0f;
		AttitudeTo(base.VesselState.OrbitalVelocity);
		if (base.Orbit.ApA < (double)base.AscentSettings.DesiredOrbitAltitude)
		{
			Core.Thrust.TargetThrottle = ThrottleToRaiseApoapsis(base.Orbit.ApR, (double)base.AscentSettings.DesiredOrbitAltitude + base.MainBody.Radius);
		}
		if (Core.Node.Autowarp)
		{
			Core.Warp.WarpPhysicsAtRate(2f);
		}
		Status = Localizer.Format("#MechJeb_Ascent_status23");
	}
}
