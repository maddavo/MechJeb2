using System;
using System.Collections.Generic;
using MechJebLib.Functions;
using MechJebLib.PSG;
using MechJebLib.Primitives;
using MechJebLib.Utils;
using MechJebLibBindings;
using UnityEngine;

namespace MuMech;

public class MechJebModuleGuidanceController : ComputerModule
{
	[Persistent(pass = 6)]
	public readonly EditableDouble UllageLeadTime = 20.0;

	[Persistent(pass = 6)]
	public bool ShouldDrawTrajectory = true;

	public double Pitch;

	public double Heading;

	private V3 _inertial = V3.zero;

	private double _throttle;

	public Vector3d Inertial = Vector3d.zero;

	public double Tgo;

	public double Vgo;

	public double StartCoast;

	public bool hasCoasted;

	public Solution? Solution;

	public PSGStatus Status;

	private bool _allowExecution;

	private readonly List<Vector3d> _trajectory = new List<Vector3d>();

	private readonly Orbit _finalOrbit = new Orbit();

	private MechJebModuleAscentSettings _ascentSettings => Core.AscentSettings;

	public MechJebModuleGuidanceController(MechJebCore core)
		: base(core)
	{
	}//IL_001c: Unknown result type (might be due to invalid IL or missing references)
	//IL_0021: Unknown result type (might be due to invalid IL or missing references)
	//IL_0027: Unknown result type (might be due to invalid IL or missing references)
	//IL_002c: Unknown result type (might be due to invalid IL or missing references)
	//IL_003d: Unknown result type (might be due to invalid IL or missing references)
	//IL_0047: Expected O, but got Unknown


	public override void OnStart(StartState state)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0003: Unknown result type (might be due to invalid IL or missing references)
		//IL_0005: Invalid comparison between Unknown and I4
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Expected O, but got Unknown
		if ((int)state != 0 && (int)state != 1)
		{
			Core.AddToPostDrawQueue(new Callback(DrawTrajectory));
		}
	}

	protected override void OnModuleEnabled()
	{
		Debug.Log((object)"MechJebModuleGuidanceController: Enabled");
		Status = PSGStatus.ENABLED;
		Core.Attitude.Users.Add(this);
		Core.Thrust.Users.Add(this);
		Core.Spinup.Users.Add(this);
		Solution = null;
		_allowExecution = false;
		hasCoasted = false;
	}

	protected override void OnModuleDisabled()
	{
		Debug.Log((object)"MechJebModuleGuidanceController: Disabled");
		Core.Attitude.attitudeDeactivate();
		if (!Core.RssMode)
		{
			Core.Thrust.ThrustOff();
		}
		Core.Thrust.Users.Remove(this);
		Core.Staging.Users.Remove(this);
		Core.Spinup.Users.Remove(this);
		Solution = null;
		Status = PSGStatus.FINISHED;
	}

	public void AssertStart(bool allowExecution = true)
	{
		_allowExecution = allowExecution;
	}

	public override void OnFixedUpdate()
	{
		UpdatePitchAndHeading();
		if (!HighLogic.LoadedSceneIsFlight)
		{
			Debug.Log((object)"MechJebModuleGuidanceController [BUG]: PSG enabled in non-flight mode.  How does this happen?");
			Done();
		}
		if (!base.Enabled || Status == PSGStatus.ENABLED)
		{
			return;
		}
		if (Status == PSGStatus.FINISHED)
		{
			Done();
			return;
		}
		if ((Status == PSGStatus.BURNING || Status == PSGStatus.TERMINAL) && base.VesselState.ThrustCurrent < base.VesselState.ThrustMinimum * 0.98)
		{
			Core.Attitude.Controller.Reset();
		}
		HandleTerminal();
		HandleSpinup();
		HandleThrottle();
		DrawTrajectory();
	}

	private bool WillDoRCSButNotYet()
	{
		if (Solution == null)
		{
			return false;
		}
		if (base.Vessel.hasEnabledRCSModules() && base.VesselState.RCSThrustAvailable.Up > 0.1 * base.VesselState.RCSThrustAvailable.MaxMagnitude() && Status != PSGStatus.TERMINAL_RCS)
		{
			return base.Vessel.currentStage == Solution.TerminalKSPStage();
		}
		return false;
	}

	private void HandleTerminal()
	{
		//IL_01b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01db: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_0206: Unknown result type (might be due to invalid IL or missing references)
		//IL_0207: Unknown result type (might be due to invalid IL or missing references)
		//IL_020d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0213: Unknown result type (might be due to invalid IL or missing references)
		//IL_0218: Unknown result type (might be due to invalid IL or missing references)
		//IL_021d: Unknown result type (might be due to invalid IL or missing references)
		//IL_026b: Unknown result type (might be due to invalid IL or missing references)
		//IL_026c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0271: Unknown result type (might be due to invalid IL or missing references)
		//IL_0272: Unknown result type (might be due to invalid IL or missing references)
		if (Solution == null || !IsThrustOn() || IsGrounded())
		{
			return;
		}
		if (base.Vessel.currentStage < Solution.TerminalKSPStage())
		{
			Done();
			return;
		}
		if (Solution.OptimizeKSPStage() < 0 && base.Vessel.currentStage <= Solution.TerminalKSPStage() && Solution.Tgo(base.VesselState.Time) <= 0.0 && base.VesselState.ThrustAvailable == 0.0)
		{
			Done();
			return;
		}
		if (Status == PSGStatus.TERMINAL_STAGING)
		{
			if (base.Vessel.currentStage == Solution.OptimizeKSPStage())
			{
				return;
			}
			Status = PSGStatus.BURNING;
		}
		if (base.Vessel.currentStage != Solution.OptimizeKSPStage() || Solution.TgoForKSPStage(base.VesselState.Time, base.Vessel.currentStage) > 10.0 || IsCoasting() || (_ascentSettings.CoastLocation == 0 && base.Vessel.currentStage == Solution.CoastKSPStage() && Solution.WillCoast(base.VesselState.Time)))
		{
			return;
		}
		if (Status != PSGStatus.TERMINAL_RCS)
		{
			Status = PSGStatus.TERMINAL;
		}
		Core.Warp.MinimumWarp();
		if (Status == PSGStatus.TERMINAL_RCS && !base.Vessel.ActionGroups[(KSPActionGroup)8])
		{
			Debug.Log((object)"[MechJebModuleGuidanceController] terminating guidance due to manual deactivation of RCS.");
			TerminalDone();
			return;
		}
		int num = ((!WillDoRCSButNotYet()) ? 1 : 2);
		Vector3d acceleration_immediate = base.Vessel.acceleration_immediate;
		double num2 = (float)num * TimeWarp.fixedDeltaTime;
		Vector3d val = base.VesselState.OrbitalVelocity + acceleration_immediate * num2;
		Vector3d val2 = base.VesselState.OrbitalPosition + base.VesselState.OrbitalVelocity * num2 + 0.5 * acceleration_immediate * num2 * num2;
		bool flag = false;
		if (Status == PSGStatus.TERMINAL && base.VesselState.ThrustCurrent == 0.0 && base.Vessel.currentStage < Solution.TerminalKSPStage())
		{
			Debug.Log((object)"[MechJebModuleGuidanceController] no thrust in last stage.");
			flag = true;
		}
		if (Solution.TerminalGuidanceSatisfied(MathExtensions.WorldToV3Rotated(val2), MathExtensions.WorldToV3Rotated(val), base.VesselState.Time))
		{
			flag = true;
		}
		if (!flag)
		{
			return;
		}
		if (WillDoRCSButNotYet())
		{
			Debug.Log((object)"[MechJebModuleGuidanceController] transition to RCS terminal guidance.");
			Status = PSGStatus.TERMINAL_RCS;
			if (!base.Vessel.ActionGroups[(KSPActionGroup)8])
			{
				base.Vessel.ActionGroups.SetGroup((KSPActionGroup)8, true);
			}
		}
		else
		{
			Debug.Log((object)"[MechJebModuleGuidanceController] terminal guidance completed.");
			TerminalDone();
		}
	}

	private void HandleSpinup()
	{
		if (base.Vessel.currentStage == _ascentSettings.SpinupStage)
		{
			Core.Spinup.AssertStart();
			Core.Spinup.RollAngularVelocity = _ascentSettings.SpinupAngularVelocity;
		}
	}

	public bool IsTerminal()
	{
		if (Status != PSGStatus.TERMINAL_RCS && Status != PSGStatus.TERMINAL_STAGING)
		{
			return Status == PSGStatus.TERMINAL;
		}
		return true;
	}

	public bool IsStable()
	{
		if (!IsNormal())
		{
			return IsTerminal();
		}
		return true;
	}

	public bool IsReady()
	{
		if (Status != 0)
		{
			return IsNormal();
		}
		return true;
	}

	public bool IsNormal()
	{
		if (Status != PSGStatus.INITIALIZED && Status != PSGStatus.BURNING)
		{
			return Status == PSGStatus.COASTING;
		}
		return true;
	}

	public bool IsCoasting()
	{
		return Status == PSGStatus.COASTING;
	}

	private bool IsThrustOn()
	{
		if (!IsBurning())
		{
			return IsTerminal();
		}
		return true;
	}

	private bool IsBurning()
	{
		return Status == PSGStatus.BURNING;
	}

	public bool IsInitializing()
	{
		if (Status != 0)
		{
			return Status == PSGStatus.INITIALIZED;
		}
		return true;
	}

	private void HandleThrottle()
	{
		if (Solution == null || !_allowExecution)
		{
			return;
		}
		if (Status == PSGStatus.TERMINAL_RCS)
		{
			RCSOn();
			return;
		}
		if (Status == PSGStatus.TERMINAL || Status == PSGStatus.TERMINAL_STAGING)
		{
			ThrottleOn();
			return;
		}
		int num = Solution.CoastKSPStage();
		if (num >= 0 && base.Vessel.currentStage >= num && Solution.WillCoast(base.VesselState.Time))
		{
			Core.Staging.AutoStageLimitRequest(num, this);
		}
		else
		{
			Core.Staging.AutoStageLimitRequest(Solution.TerminalKSPStage(), this);
		}
		if (Solution.Coast(base.VesselState.Time))
		{
			if (!IsCoasting())
			{
				DoCoast();
			}
			if (Solution.StageTimeLeft(base.VesselState.Time) < (double)UllageLeadTime)
			{
				RCSOn();
			}
			ThrustOff();
		}
		else
		{
			ThrottleOn();
			Status = PSGStatus.BURNING;
		}
	}

	private bool IsGrounded()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Invalid comparison between Unknown and I4
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Invalid comparison between Unknown and I4
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Invalid comparison between Unknown and I4
		if ((int)base.Vessel.situation != 1 && (int)base.Vessel.situation != 4)
		{
			return (int)base.Vessel.situation == 2;
		}
		return true;
	}

	private void UpdatePitchAndHeading()
	{
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0112: Unknown result type (might be due to invalid IL or missing references)
		//IL_0117: Unknown result type (might be due to invalid IL or missing references)
		//IL_011c: Unknown result type (might be due to invalid IL or missing references)
		if (Solution != null)
		{
			if (IsGrounded())
			{
				Solution.T0 = base.VesselState.Time;
			}
			if (Status != PSGStatus.TERMINAL_RCS)
			{
				Tgo = Solution.Tgo(base.VesselState.Time);
				Vgo = Solution.Vgo(base.VesselState.Time);
			}
			V3 val = MathExtensions.WorldToV3Rotated(base.VesselState.OrbitalPosition);
			int num = Solution.IndexForKSPStage(base.Vessel.currentStage, Core.Guidance.IsCoasting());
			if (IsGrounded() || Solution.Tgo(base.VesselState.Time, num) > 2.0)
			{
				(_inertial, _throttle) = Solution.InertialGuidance(base.VesselState.Time);
			}
			ValueTuple<double, double> valueTuple = Astro.ECIToPitchHeading(val, _inertial);
			double item = valueTuple.Item1;
			double item2 = valueTuple.Item2;
			Inertial = MathExtensions.V3ToWorldRotated(_inertial);
			Pitch = Statics.Rad2Deg(item);
			Heading = Statics.Rad2Deg(item2);
		}
	}

	private void DrawTrajectory()
	{
		//IL_0074: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Unknown result type (might be due to invalid IL or missing references)
		//IL_0084: Unknown result type (might be due to invalid IL or missing references)
		//IL_0089: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f1: Unknown result type (might be due to invalid IL or missing references)
		//IL_0107: Unknown result type (might be due to invalid IL or missing references)
		//IL_010c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0111: Unknown result type (might be due to invalid IL or missing references)
		//IL_0115: Unknown result type (might be due to invalid IL or missing references)
		//IL_011a: Unknown result type (might be due to invalid IL or missing references)
		//IL_011f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0128: Unknown result type (might be due to invalid IL or missing references)
		//IL_012f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0150: Unknown result type (might be due to invalid IL or missing references)
		if (Solution != null && base.Enabled && MapView.MapIsEnabled && ShouldDrawTrajectory)
		{
			_trajectory.Clear();
			double num = (Solution.Tf - Solution.T0) / 50.0;
			for (int i = 0; i <= 50; i++)
			{
				double num2 = Solution.T0 + num * (double)i;
				_trajectory.Add(MathExtensions.V3ToWorldRotated(Solution.R(num2)) + base.MainBody.position);
			}
			GLUtils.DrawPath(base.MainBody, _trajectory, Color.red, MapView.MapIsEnabled);
			QuaternionD rotation = Planetarium.fetch.rotation;
			Vector3d val = MathExtensions.ToVector3d(Solution.R(Solution.Tf));
			Vector3d val2 = rotation * ((Vector3d)(ref val)).xzy;
			QuaternionD rotation2 = Planetarium.fetch.rotation;
			val = MathExtensions.ToVector3d(Solution.V(Solution.Tf));
			Vector3d val3 = rotation2 * ((Vector3d)(ref val)).xzy;
			_finalOrbit.UpdateFromStateVectors(((Vector3d)(ref val2)).xzy, ((Vector3d)(ref val3)).xzy, base.MainBody, Solution.Tf);
			GLUtils.DrawOrbit(_finalOrbit, Color.yellow);
		}
	}

	private void ThrottleOn()
	{
		Core.Thrust.TargetThrottle = ((_throttle > 0.0) ? ((float)_throttle) : 1f);
	}

	private void RCSOn()
	{
		Core.Thrust.ThrustOff();
		base.Vessel.ctrlState.Z = -1f;
	}

	private void ThrustOff()
	{
		Core.Thrust.ThrustOff();
	}

	private void TerminalDone()
	{
		if (Solution == null)
		{
			Done();
		}
		else if (base.Vessel.currentStage == Solution.CoastKSPStage() && Solution.WillCoast(base.VesselState.Time))
		{
			ThrustOff();
			DoCoast();
		}
		else if (Solution.TerminalKSPStage() != base.Vessel.currentStage && Solution.OptimizeKSPStage() == base.Vessel.currentStage)
		{
			ThrustOff();
			Core.Staging.ImmediateStage();
			Status = PSGStatus.TERMINAL_STAGING;
		}
		else
		{
			Done();
		}
	}

	private void Done()
	{
		Users.Clear();
		ThrustOff();
		Status = PSGStatus.FINISHED;
		Solution = null;
		base.Enabled = false;
	}

	private void DoCoast()
	{
		StartCoast = base.VesselState.Time;
		if (!base.Vessel.ActionGroups[(KSPActionGroup)8])
		{
			base.Vessel.ActionGroups.SetGroup((KSPActionGroup)8, true);
		}
		Status = PSGStatus.COASTING;
		hasCoasted = true;
	}

	public void SetSolution(Solution solution)
	{
		Solution = solution;
		if (Status == PSGStatus.ENABLED)
		{
			Status = PSGStatus.INITIALIZED;
		}
	}
}
