using System;
using KSP.Localization;
using KSP.UI.Screens;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public abstract class MechJebModuleAscentBaseAutopilot : ComputerModule
{
	private enum AscentMode
	{
		PRELAUNCH,
		ASCEND,
		CIRCULARIZE
	}

	public string Status = "";

	public bool TimedLaunch;

	private double _launchTime;

	public double CurrentMaxAoA;

	private double _launchStarted;

	public double LaunchLongitude;

	private AscentMode _mode;

	private bool _placedCircularizeNode;

	private double _lastTMinus = 999.0;

	private float _raiseApoapsisLastThrottle;

	private double _raiseApoapsisLastApR;

	private double _raiseApoapsisLastUT;

	private readonly MovingAverage _raiseApoapsisRatePerThrottle = new MovingAverage(3);

	protected MechJebModuleAscentSettings AscentSettings => Core.AscentSettings;

	public double TMinus => _launchTime - base.VesselState.Time;

	protected double MET => base.VesselState.Time - ((_launchStarted > base.Vessel.launchTime) ? _launchStarted : base.Vessel.launchTime);

	protected MechJebModuleAscentBaseAutopilot(MechJebCore core)
		: base(core)
	{
	}

	private void OnLaunch(EventReport report)
	{
		_launchStarted = base.VesselState.Time;
		Debug.Log((object)("[MechJebModuleAscentAutopilot] LaunchStarted = " + _launchStarted));
	}

	public override void OnStart(StartState state)
	{
		_launchStarted = -1.0;
		GameEvents.onLaunch.Add((OnEvent<EventReport>)OnLaunch);
	}

	private void FixupLaunchStart()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Invalid comparison between Unknown and I4
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Invalid comparison between Unknown and I4
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Invalid comparison between Unknown and I4
		if ((int)base.Vessel.situation == 1 || (int)base.Vessel.situation == 4 || (int)base.Vessel.situation == 2)
		{
			_launchStarted = base.VesselState.Time;
			LaunchLongitude = base.VesselState.CelestialLongitude;
		}
	}

	protected override void OnModuleEnabled()
	{
		_mode = AscentMode.PRELAUNCH;
		_placedCircularizeNode = false;
		Core.Attitude.Users.Add(this);
		Core.Thrust.Users.Add(this);
		if (AscentSettings.Autostage)
		{
			Core.Staging.Users.Add(this);
		}
		Status = Localizer.Format("#MechJeb_Ascent_status1");
	}

	protected override void OnModuleDisabled()
	{
		Core.Attitude.attitudeDeactivate();
		if (!Core.RssMode)
		{
			Core.Thrust.ThrustOff();
		}
		Core.Thrust.Users.Remove(this);
		Core.Staging.Users.Remove(this);
		if (_placedCircularizeNode)
		{
			Core.Node.Abort();
		}
		Status = Localizer.Format("#MechJeb_Ascent_status2");
	}

	public void StartCountdown(double time)
	{
		if (AscentSettings.OverrideWarpToPlane)
		{
			TimedLaunch = false;
			_launchTime = base.VesselState.Time;
			_lastTMinus = 0.0;
		}
		else
		{
			TimedLaunch = true;
			_launchTime = time;
			_lastTMinus = 999.0;
		}
	}

	public override void OnFixedUpdate()
	{
		if (AscentSettings.AscentType == AscentType.PSG)
		{
			Core.StageStats.RequestUpdate();
		}
		FixupLaunchStart();
		if (!TimedLaunch)
		{
			return;
		}
		if (TMinus < 3.0 * base.VesselState.DeltaT || (TMinus > 10.0 && _lastTMinus < 1.0))
		{
			if (base.Enabled && base.VesselState.ThrustAvailable < 0.001)
			{
				StageManager.ActivateNextStage();
			}
			TimedLaunch = false;
		}
		else if (Core.Node.Autowarp)
		{
			Core.Warp.WarpToUT(_launchTime - (double)(int)AscentSettings.WarpCountDown);
		}
		_lastTMinus = TMinus;
	}

	public override void Drive(FlightCtrlState s)
	{
		AscentSettings.LimitingAoA = false;
		switch (_mode)
		{
		case AscentMode.PRELAUNCH:
			DrivePrelaunch();
			break;
		case AscentMode.ASCEND:
			DriveAscent();
			break;
		case AscentMode.CIRCULARIZE:
			DriveCircularizationBurn();
			break;
		}
	}

	private void DriveDeployableComponents()
	{
		if (AscentSettings.AutoDeploySolarPanels)
		{
			if (base.VesselState.AltitudeASL > base.MainBody.RealMaxAtmosphereAltitude())
			{
				Core.Solarpanel.ExtendAll();
			}
			else
			{
				Core.Solarpanel.RetractAll();
			}
		}
		if (AscentSettings.AutoDeployAntennas)
		{
			if (base.VesselState.AltitudeASL > base.MainBody.RealMaxAtmosphereAltitude())
			{
				Core.AntennaControl.ExtendAll();
			}
			else
			{
				Core.AntennaControl.RetractAll();
			}
		}
	}

	private void DrivePrelaunch()
	{
		if (base.Vessel.LiftedOff() && !base.Vessel.Landed)
		{
			Status = Localizer.Format("#MechJeb_Ascent_status4");
			_mode = AscentMode.ASCEND;
			return;
		}
		Core.Thrust.ThrustOff();
		Core.Attitude.SetAxisControl(pitch: false, yaw: false, roll: false);
		if (TimedLaunch && TMinus > 10.0)
		{
			Status = Localizer.Format("#MechJeb_Ascent_status1");
		}
		else if (AscentSettings.AutoDeploySolarPanels && base.MainBody.atmosphere)
		{
			Core.Solarpanel.RetractAll();
			if (Core.Solarpanel.AllRetracted())
			{
				Debug.Log((object)"Prelaunch -> Ascend");
				_mode = AscentMode.ASCEND;
			}
			else
			{
				Status = Localizer.Format("#MechJeb_Ascent_status5");
			}
		}
		else
		{
			_mode = AscentMode.ASCEND;
		}
	}

	private void DriveAscent()
	{
		if (TimedLaunch)
		{
			Status = Localizer.Format("#MechJeb_Ascent_status6");
			Core.Attitude.SetAxisControl(pitch: false, yaw: false, roll: false);
			return;
		}
		DriveDeployableComponents();
		if (DriveAscent2())
		{
			if (GameSettings.VERBOSE_DEBUG_LOG)
			{
				Debug.Log((object)"Remaining in Ascent");
			}
			return;
		}
		if (GameSettings.VERBOSE_DEBUG_LOG)
		{
			Debug.Log((object)"Ascend -> Circularize");
		}
		_mode = AscentMode.CIRCULARIZE;
	}

	private void DriveCircularizationBurn()
	{
		//IL_0093: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00af: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c1: Unknown result type (might be due to invalid IL or missing references)
		if (!base.Vessel.patchedConicsUnlocked() || AscentSettings.SkipCircularization)
		{
			Users.Clear();
			return;
		}
		DriveDeployableComponents();
		if (_placedCircularizeNode)
		{
			if (base.Vessel.patchedConicSolver.maneuverNodes.Count == 0)
			{
				Users.Clear();
				return;
			}
		}
		else
		{
			base.Vessel.RemoveAllManeuverNodes();
			double num = base.Orbit.NextApoapsisTime(base.VesselState.Time);
			Vector3d val = OrbitalManeuverCalculator.DeltaVToChangeInclination(base.Orbit, num, AscentSettings.DesiredInclination);
			Vector3d val2 = OrbitalManeuverCalculator.DeltaVToCircularize(base.Orbit.PerturbedOrbit(num, val), num);
			Vector3d dV = val + val2;
			base.Vessel.PlaceManeuverNode(base.Orbit, dV, num);
			_placedCircularizeNode = true;
			Core.Node.ExecuteOneNode(this);
		}
		Status = Localizer.Format((Core.Node.State == MechJebModuleNodeExecutor.States.BURN) ? "#MechJeb_Ascent_status7" : "#MechJeb_Ascent_status8");
	}

	protected abstract bool DriveAscent2();

	protected float ThrottleToRaiseApoapsis(double currentApR, double finalApR)
	{
		float num;
		if (currentApR > finalApR + 5.0)
		{
			num = 0f;
		}
		else if (_raiseApoapsisLastUT > base.VesselState.Time - 1.0)
		{
			double val = (base.Orbit.ApR - _raiseApoapsisLastApR) / ((base.VesselState.Time - _raiseApoapsisLastUT) * (double)_raiseApoapsisLastThrottle);
			val = Math.Max(1.0, val);
			_raiseApoapsisRatePerThrottle.Value = val;
			num = Mathf.Clamp((float)((finalApR - currentApR) / 1.0 / (double)_raiseApoapsisRatePerThrottle), 0.05f, 1f);
		}
		else
		{
			num = 1f;
		}
		_raiseApoapsisLastThrottle = num;
		_raiseApoapsisLastApR = base.Orbit.ApR;
		_raiseApoapsisLastUT = base.VesselState.Time;
		return num;
	}

	protected double SrfvelPitch()
	{
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		return 90.0 - Vector3d.Angle(base.VesselState.SurfaceVelocity, base.VesselState.Up);
	}

	protected double SrfvelHeading()
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		return base.VesselState.HeadingFromDirection(base.VesselState.SurfaceVelocity.ProjectOnPlane(base.VesselState.Up));
	}

	protected void AttitudeTo(double desiredPitch, double desiredHeading)
	{
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_0056: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0081: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = Math.Sin(desiredHeading * (Math.PI / 180.0)) * base.VesselState.East + Math.Cos(desiredHeading * (Math.PI / 180.0)) * base.VesselState.North;
		Vector3d desiredThrustVector = Math.Cos(desiredPitch * (Math.PI / 180.0)) * val + Math.Sin(desiredPitch * (Math.PI / 180.0)) * base.VesselState.Up;
		AttitudeTo(desiredThrustVector);
	}

	protected void AttitudeTo(Vector3d desiredThrustVector)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0074: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cb: Unknown result type (might be due to invalid IL or missing references)
		desiredThrustVector = ((Vector3d)(ref desiredThrustVector)).normalized;
		if (AscentSettings.LimitQaEnabled)
		{
			desiredThrustVector = ApplyQAlphaAoALimiter(desiredThrustVector);
		}
		else if (AscentSettings.LimitAoA)
		{
			desiredThrustVector = ApplyStockAOALimiter(desiredThrustVector);
		}
		double pitch = 90.0 - Vector3d.Angle(desiredThrustVector, base.VesselState.Up);
		double heading = MuUtils.ClampDegrees360(180.0 / Math.PI * Math.Atan2(Vector3d.Dot(desiredThrustVector, base.VesselState.East), Vector3d.Dot(desiredThrustVector, base.VesselState.North)));
		if (AscentSettings.ForceRoll)
		{
			Core.Attitude.attitudeTo(heading, pitch, AscentSettings.TurnRoll, this, AxisCtrlPitch: true, AxisCtrlYaw: true, AxisCtrlRoll: true, fixCOT: true);
		}
		else
		{
			Core.Attitude.attitudeTo(desiredThrustVector, AttitudeReference.INERTIAL_COT, this);
		}
		Core.Attitude.SetActuationControl();
		Core.Attitude.SetAxisControl(pitch: true, yaw: true, AscentSettings.ForceRoll);
	}

	protected void VerticalHeadingTo(double desiredHeading)
	{
		Core.Attitude.attitudeTo(desiredHeading, 90.0, AscentSettings.VerticalRoll, this, AxisCtrlPitch: true, AxisCtrlYaw: true, AxisCtrlRoll: true, fixCOT: true);
		bool flag = base.Vessel.LiftedOff() && !base.Vessel.Landed;
		Core.Attitude.SetActuationControl(flag, flag, flag);
		Core.Attitude.SetAxisControl(flag, flag, flag && AscentSettings.ForceRoll && base.VesselState.AltitudeBottom > (double)AscentSettings.RollAltitude);
	}

	private Vector3d ApplyQAlphaAoALimiter(Vector3d desiredThrustVector)
	{
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00be: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c6: Unknown result type (might be due to invalid IL or missing references)
		double num = Statics.Clamp((double)AscentSettings.LimitQa, 0.0, 10000.0);
		AscentSettings.LimitingAoA = base.VesselState.DynamicPressure * (double)Vector3.Angle(Vector3d.op_Implicit(base.VesselState.SurfaceVelocity), Vector3d.op_Implicit(desiredThrustVector)) * (Math.PI / 180.0) > num;
		if (AscentSettings.LimitingAoA)
		{
			CurrentMaxAoA = num / base.VesselState.DynamicPressure * (180.0 / Math.PI);
			Vector3d val = MathExtensions.RotateTowards(base.VesselState.SurfaceVelocity, desiredThrustVector, (float)(CurrentMaxAoA * (Math.PI / 180.0)), 1.0);
			desiredThrustVector = ((Vector3d)(ref val)).normalized;
		}
		return desiredThrustVector;
	}

	private Vector3d ApplyStockAOALimiter(Vector3d desiredThrustVector)
	{
		//IL_008f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0094: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00de: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00eb: Unknown result type (might be due to invalid IL or missing references)
		double num = ((base.VesselState.DynamicPressure < (double)AscentSettings.AOALimitFadeoutPressure) ? ((double)AscentSettings.AOALimitFadeoutPressure / base.VesselState.DynamicPressure) : 1.0);
		CurrentMaxAoA = Math.Min(num * (double)AscentSettings.MaxAoA, 180.0);
		AscentSettings.LimitingAoA = base.Vessel.altitude < base.MainBody.atmosphereDepth && Vector3d.Angle(base.VesselState.SurfaceVelocity, desiredThrustVector) > CurrentMaxAoA;
		if (AscentSettings.LimitingAoA)
		{
			Vector3d val = MathExtensions.RotateTowards(base.VesselState.SurfaceVelocity, desiredThrustVector, (float)(CurrentMaxAoA * 0.01745329238474369), 1.0);
			desiredThrustVector = ((Vector3d)(ref val)).normalized;
		}
		return desiredThrustVector;
	}
}
