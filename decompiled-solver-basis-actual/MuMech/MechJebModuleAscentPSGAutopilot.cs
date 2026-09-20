using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleAscentPSGAutopilot : MechJebModuleAscentBaseAutopilot
{
	private enum AscentMode
	{
		VERTICAL_ASCENT,
		PITCHPROGRAM,
		GUIDANCE,
		EXIT
	}

	private AscentMode _mode;

	private double _pitchStartTime;

	private MechJebModuleAscentSettings _ascentSettings => Core.AscentSettings;

	public MechJebModuleAscentPSGAutopilot(MechJebCore core)
		: base(core)
	{
	}

	protected override void OnModuleEnabled()
	{
		base.OnModuleEnabled();
		Debug.Log((object)"Enabling PSG Ascent Autopilot");
		_mode = AscentMode.VERTICAL_ASCENT;
		Core.Guidance.Users.Add(this);
		Core.Guidance.CascadeDisable(this);
		Core.Glueball.Users.Add(this);
	}

	protected override void OnModuleDisabled()
	{
		base.OnModuleDisabled();
		Debug.Log((object)"Disabling PSG Ascent Autopilot");
		Core.Guidance.Users.Remove(this);
		Core.Glueball.Users.Remove(this);
	}

	public override void Drive(FlightCtrlState s)
	{
		if (TimedLaunch)
		{
			if (base.TMinus <= (double)(int)base.AscentSettings.WarpCountDown)
			{
				SetTarget();
				Core.Guidance.AssertStart(allowExecution: false);
			}
		}
		else
		{
			SetTarget();
			Core.Guidance.AssertStart();
		}
		base.Drive(s);
	}

	protected override bool DriveAscent2()
	{
		switch (_mode)
		{
		case AscentMode.VERTICAL_ASCENT:
			DriveVerticalAscent();
			break;
		case AscentMode.PITCHPROGRAM:
			DrivePitchProgram();
			break;
		case AscentMode.GUIDANCE:
			DriveGuidance();
			break;
		}
		return _mode != AscentMode.EXIT;
	}

	private void SetTarget()
	{
		double peR = base.MainBody.Radius + (double)base.AscentSettings.DesiredOrbitAltitude;
		double apR = base.MainBody.Radius + (double)base.AscentSettings.DesiredApoapsis;
		if ((double)_ascentSettings.DesiredApoapsis < 0.0)
		{
			apR = _ascentSettings.DesiredApoapsis;
		}
		double attR = base.MainBody.Radius + (double)((!base.AscentSettings.OptimizeStageFlag) ? base.AscentSettings.DesiredAttachAltFixed : base.AscentSettings.DesiredAttachAlt);
		bool lanflag = _ascentSettings.LaunchingToPlane || _ascentSettings.LaunchingToMatchLan || _ascentSettings.LaunchingToLan;
		double lan = ((_ascentSettings.LaunchingToPlane || _ascentSettings.LaunchingToMatchLan) ? Core.Target.TargetOrbit.LAN : ((double)base.AscentSettings.DesiredLan));
		double num = base.AscentSettings.DesiredInclination;
		bool attachAltFlag = !base.AscentSettings.OptimizeStageFlag || base.AscentSettings.AttachAltFlag;
		if (_ascentSettings.LaunchingToPlane)
		{
			num = (double)Math.Sign(num) * Core.Target.TargetOrbit.inclination;
		}
		Core.Glueball.SetTarget(peR, apR, attR, num, lan, base.AscentSettings.DesiredFPA, attachAltFlag, lanflag);
	}

	private void DriveVerticalAscent()
	{
		VerticalHeadingTo(Core.Guidance.Heading);
		if (!base.Vessel.LiftedOff() || base.Vessel.Landed)
		{
			Status = Localizer.Format("#MechJeb_Ascent_status12");
		}
		else if (base.VesselState.AltitudeBottom > (double)base.AscentSettings.PitchStartHeight)
		{
			_mode = AscentMode.PITCHPROGRAM;
			_pitchStartTime = base.MET;
		}
		else
		{
			double num = (double)base.AscentSettings.PitchStartHeight - base.VesselState.AltitudeBottom;
			Status = $"Vertical ascent {num:F2}m to go";
		}
	}

	private void DrivePitchProgram()
	{
		double num = (base.MET - _pitchStartTime) * (double)base.AscentSettings.PitchRate;
		double num2 = 90.0 - num;
		Status = Localizer.Format("#MechJeb_Ascent_status15", new string[1] { $"{num2 - Core.Guidance.Pitch:F}" });
		if (CheckForGuidanceTransition(num2))
		{
			_mode = AscentMode.GUIDANCE;
		}
		else
		{
			AttitudeTo(num2, Core.Guidance.Heading);
		}
	}

	private bool CheckForGuidanceTransition(double pitch)
	{
		if (!base.MainBody.atmosphere)
		{
			return true;
		}
		if (pitch <= Core.Guidance.Pitch && Core.Guidance.IsStable())
		{
			return true;
		}
		return false;
	}

	private void DriveGuidance()
	{
		//IL_007b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		if (Core.Guidance.Status == PSGStatus.FINISHED)
		{
			_mode = AscentMode.EXIT;
		}
		else if (!Core.Guidance.IsStable())
		{
			double desiredPitch = Math.Min(Math.Min(90.0, SrfvelPitch()), base.VesselState.Pitch);
			AttitudeTo(desiredPitch, SrfvelHeading());
			Status = Localizer.Format("#MechJeb_Ascent_status16");
		}
		else
		{
			double num = Vector3d.Angle(Core.Guidance.Inertial, base.VesselState.Forward);
			Status = $"Stable Guidance: {num:F}° deviation";
			AttitudeTo(Core.Guidance.Inertial);
		}
	}
}
