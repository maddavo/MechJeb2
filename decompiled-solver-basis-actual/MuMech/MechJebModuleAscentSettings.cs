using System;
using System.Collections.Generic;
using MechJebLibBindings;

namespace MuMech;

public class MechJebModuleAscentSettings : ComputerModule
{
	[Persistent(pass = 4)]
	public bool ForceResetROSettings = true;

	[Persistent(pass = 6)]
	public readonly EditableDouble PitchStartHeight = new EditableDouble(100.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble PitchRate = new EditableDouble(5.0);

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult DesiredApoapsis = new EditableDoubleMult(0.0, 1000.0);

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult DesiredAttachAlt = new EditableDoubleMult(110000.0, 1000.0);

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult DesiredAttachAltFixed = new EditableDoubleMult(110000.0, 1000.0);

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult DesiredFPA = new EditableDoubleMult(0.0, Math.PI / 180.0);

	[Persistent(pass = 6)]
	public bool AttachAltFlag;

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult DesiredArgP = new EditableDoubleMult(0.0, Math.PI / 180.0);

	[Persistent(pass = 6)]
	public bool DesiredArgPFlag;

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult TurnStartAltitude = new EditableDoubleMult(500.0, 1000.0);

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult TurnStartVelocity = new EditableDoubleMult(50.0);

	[Persistent(pass = 6)]
	public int AscentTypeInteger;

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult DesiredOrbitAltitude = new EditableDoubleMult(100000.0, 1000.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble DesiredInclination = new EditableDouble(0.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble DesiredLan = new EditableDouble(0.0);

	[Persistent(pass = 6)]
	public bool RelativeLAN;

	[Persistent(pass = 6)]
	public bool CorrectiveSteering;

	[Persistent(pass = 6)]
	public readonly EditableDouble CorrectiveSteeringGain = new EditableDouble(3.0);

	[Persistent(pass = 6)]
	public bool ForceRoll = true;

	[Persistent(pass = 6)]
	public readonly EditableDouble VerticalRoll = new EditableDouble(0.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble TurnRoll = new EditableDouble(0.0);

	[Persistent(pass = 6)]
	public bool AutoDeploySolarPanels = true;

	[Persistent(pass = 6)]
	public bool AutoDeployAntennas = true;

	[Persistent(pass = 6)]
	public bool SkipCircularization;

	[Persistent(pass = 6)]
	public readonly EditableDouble RollAltitude = new EditableDouble(50.0);

	[Persistent(pass = 6)]
	public bool _autostage = true;

	[Persistent(pass = 6)]
	public bool LimitAoA = true;

	[Persistent(pass = 6)]
	public readonly EditableDouble MaxAoA = 5.0;

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult AOALimitFadeoutPressure = new EditableDoubleMult(2500.0);

	[Persistent(pass = 6)]
	public bool LimitingAoA;

	[Persistent(pass = 6)]
	public readonly EditableDouble LimitQa = new EditableDouble(2000.0);

	[Persistent(pass = 6)]
	public bool LimitQaEnabled = true;

	[Persistent(pass = 6)]
	public readonly EditableDouble LaunchLANDifference = 0.0;

	[Persistent(pass = 4)]
	public readonly EditableInt WarpCountDown = 11;

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult TurnEndAltitude = new EditableDoubleMult(60000.0, 1000.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble TurnEndAngle = 0.0;

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult TurnShapeExponent = new EditableDoubleMult(0.4, 0.01);

	[Persistent(pass = 6)]
	public bool AutoPath = true;

	[Persistent(pass = 6)]
	public float AutoTurnPerc = 0.05f;

	[Persistent(pass = 6)]
	public float AutoTurnSpdFactor = 18.5f;

	[Persistent(pass = 6)]
	public readonly EditableDouble MinDeltaV = 40.0;

	[Persistent(pass = 6)]
	public readonly EditableDouble MaxCoast = 450.0;

	[Persistent(pass = 6)]
	public readonly EditableDouble MinCoast = 0.0;

	[Persistent(pass = 2)]
	public int CoastLocation = -1;

	[Persistent(pass = 2)]
	public readonly EditableDouble PreStageTime = 10.0;

	[Persistent(pass = 2)]
	public readonly EditableDouble OptimizerPauseTime = 5.0;

	[Persistent(pass = 2)]
	public readonly EditableInt LastStage = -1;

	[Persistent(pass = 2)]
	public readonly EditableInt CoastStageInternal = -1;

	[Persistent(pass = 2)]
	public bool CoastStageFlag;

	[Persistent(pass = 2)]
	public bool SpinupStageFlag = true;

	[Persistent(pass = 2)]
	public readonly EditableInt SpinupStageInternal = -1;

	[Persistent(pass = 2)]
	public readonly EditableDouble SpinupLeadTime = 50.0;

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult SpinupAngularVelocity = new EditableDoubleMult(Math.PI / 3.0, Math.PI / 30.0);

	[Persistent(pass = 2)]
	public readonly EditableIntList UnguidedStagesInternal = new EditableIntList();

	[Persistent(pass = 2)]
	public bool UnguidedStagesFlag;

	private readonly List<int> _emptyList = new List<int>();

	[Persistent(pass = 2)]
	public readonly EditableIntList FixedStagesInternal = new EditableIntList();

	[Persistent(pass = 2)]
	public bool FixedStagesFlag;

	public bool OptimizeStageFlag;

	[Persistent(pass = 6)]
	public readonly EditableDouble Cd = new EditableDouble(0.5);

	[Persistent(pass = 6)]
	public readonly EditableDouble Aref = new EditableDouble(0.0);

	public bool LaunchingToPlane;

	public bool LaunchingToMatchLan;

	public bool LaunchingToLan;

	public bool OverrideWarpToPlane;

	private readonly AscentType[] _values = (AscentType[])Enum.GetValues(typeof(AscentType));

	private const double LAUNCH_LAN_DIFFERENCE = 0.0;

	private const double MIN_COAST_DEFAULT = 0.0;

	private const double MAX_COAST_DEFAULT = 450.0;

	private const double MIN_DELTAV_DEFAULT = 40.0;

	private const double DESIRED_ATTACH_ALT_DEFAULT = 110000.0;

	private const double PITCH_START_HEIGHT_DEFAULT = 100.0;

	private const double PITCH_RATE_DEFAULT = 5.0;

	private const bool ATTACH_ALT_FLAG_DEFAULT = false;

	private const double DESIRED_ARGP_DEFAULT = 0.0;

	private const bool DESIRED_ARGP_FLAG_DEFAULT = false;

	private const double DESIRED_FPA_DEFAULT = 0.0;

	private const double LIMIT_QA_DEFAULT = 2000.0;

	private const bool LIMIT_QA_ENABLED_DEFAULT = true;

	private const double PRE_STAGE_TIME_DEFAULT = 10.0;

	private const double OPTIMIZER_PAUSE_TIME_DEFAULT = 5.0;

	public AscentType AscentType
	{
		get
		{
			return (AscentType)AscentTypeInteger;
		}
		set
		{
			AscentTypeInteger = (int)value;
			DisableAscentModules();
		}
	}

	public MechJebModuleAscentBaseAutopilot AscentAutopilot => GetAscentModule(AscentType);

	public bool Autostage
	{
		get
		{
			return _autostage;
		}
		set
		{
			bool num = value != _autostage;
			_autostage = value;
			if (num)
			{
				if (_autostage && base.Enabled)
				{
					Core.Staging.Users.Add(AscentAutopilot);
				}
				else if (!_autostage)
				{
					Core.Staging.Users.Remove(AscentAutopilot);
				}
			}
		}
	}

	public double AutoTurnStartAltitude
	{
		get
		{
			if (!base.Vessel.mainBody.atmosphere)
			{
				return base.Vessel.terrainAltitude + 25.0;
			}
			return base.Vessel.mainBody.RealMaxAtmosphereAltitude() * (double)AutoTurnPerc;
		}
	}

	public double AutoTurnStartVelocity
	{
		get
		{
			if (!base.Vessel.mainBody.atmosphere)
			{
				return double.PositiveInfinity;
			}
			return AutoTurnSpdFactor * AutoTurnSpdFactor * AutoTurnSpdFactor * (1f / 64f);
		}
	}

	public double AutoTurnEndAltitude
	{
		get
		{
			if (!base.Vessel.mainBody.atmosphere)
			{
				return Math.Min(30000.0, (double)DesiredOrbitAltitude * 0.85);
			}
			return Math.Min(base.Vessel.mainBody.RealMaxAtmosphereAltitude() * 0.85, DesiredOrbitAltitude);
		}
	}

	public int CoastStage
	{
		get
		{
			if (!CoastStageFlag)
			{
				return -1;
			}
			return CoastStageInternal.Val;
		}
	}

	public int SpinupStage
	{
		get
		{
			if (!SpinupStageFlag)
			{
				return -1;
			}
			return SpinupStageInternal.Val;
		}
	}

	public List<int> UnguidedStages
	{
		get
		{
			if (!UnguidedStagesFlag)
			{
				return _emptyList;
			}
			return UnguidedStagesInternal.Val;
		}
	}

	public List<int> FixedStages
	{
		get
		{
			if (!FixedStagesFlag)
			{
				return _emptyList;
			}
			return FixedStagesInternal.Val;
		}
	}

	public MechJebModuleAscentSettings(MechJebCore core)
		: base(core)
	{
	}

	private MechJebModuleAscentBaseAutopilot GetAscentModule(AscentType type)
	{
		return type switch
		{
			AscentType.CLASSIC => Core.GetComputerModule<MechJebModuleAscentClassicAutopilot>(), 
			AscentType.PSG => Core.GetComputerModule<MechJebModuleAscentPSGAutopilot>(), 
			_ => Core.GetComputerModule<MechJebModuleAscentClassicAutopilot>(), 
		};
	}

	private void DisableAscentModules()
	{
		AscentType[] values = _values;
		foreach (AscentType ascentType in values)
		{
			if (ascentType != AscentType)
			{
				GetAscentModule(ascentType).Enabled = false;
			}
		}
	}

	public override void OnFixedUpdate()
	{
		DisableAscentModules();
	}

	public override void OnStart(StartState state)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		base.OnStart(state);
		if (ForceResetROSettings && ReflectionUtils.IsLoadedRealismOverhaul)
		{
			ApplyRODefaults();
			ForceResetROSettings = false;
		}
	}

	private void ApplyRODefaults()
	{
		PitchStartHeight.Val = 100.0;
		PitchRate.Val = 5.0;
		DesiredAttachAlt.Val = 110000.0;
		DesiredAttachAltFixed.Val = 110000.0;
		DesiredFPA.Val = 0.0;
		AttachAltFlag = false;
		DesiredArgP.Val = 0.0;
		DesiredArgPFlag = false;
		LimitQa.Val = 2000.0;
		LimitQaEnabled = true;
		MinDeltaV.Val = 40.0;
		MaxCoast.Val = 450.0;
		MinCoast.Val = 0.0;
		LaunchLANDifference.Val = 0.0;
		PreStageTime.Val = 10.0;
		OptimizerPauseTime.Val = 5.0;
		SpinupStageFlag = false;
		SpinupStageInternal.Val = -1;
		CoastStageFlag = false;
		CoastStageInternal.Val = -1;
		UnguidedStagesFlag = false;
		FixedStagesFlag = false;
		DesiredOrbitAltitude.Val = 145000.0;
		DesiredAttachAlt.Val = 145000.0;
		Core.Guidance.UllageLeadTime.Val = 20.0;
		Core.Settings.RssMode = true;
		Core.Thrust.LimitToPreventUnstableIgnition = false;
		Core.Thrust.AutoRCSUllaging = true;
		Core.Thrust.MinThrottle.Val = 0.05;
		Core.Thrust.LimiterMinThrottle = true;
		Core.Thrust.LimitThrottle = false;
		Core.Thrust.LimitAcceleration = false;
		Core.Thrust.LimitToPreventOverheats = false;
		Core.Thrust.LimitDynamicPressure = false;
		Core.Thrust.MaxDynamicPressure.Val = 20000.0;
		Autostage = true;
		Core.Staging.AutostagePreDelay.Val = 0.0;
		Core.Staging.AutostagePostDelay.Val = 0.5;
		Core.Staging.AutostageLimit.Val = 0;
		Core.Staging.FairingMaxDynamicPressure.Val = 5000.0;
		Core.Staging.FairingMinAltitude.Val = 50000.0;
		Core.Staging.ClampAutoStageThrustPct.Val = 0.99;
		Core.Staging.FairingMaxAerothermalFlux.Val = 1135.0;
		Core.Staging.HotStaging = true;
		Core.Staging.HotStagingLeadTime.Val = 2.0;
		Core.Staging.DropSolids = true;
		Core.Staging.DropSolidsTwrPct.Val = 0.5;
		AscentType = AscentType.PSG;
		MechJebModuleAscentMenu computerModule = Core.GetComputerModule<MechJebModuleAscentMenu>();
		computerModule._lastPSGSettingsEnabled = true;
		computerModule._lastSettingsMenuEnabled = true;
		Core.StageStats.LiveSLT = true;
		Core.GetComputerModule<MechJebModuleInfoItems>().StageDisplayState = 1;
		Core.Thrust.MaxDynamicPressure.Val = 50000.0;
		Core.Thrust.LimitDynamicPressure = false;
		Core.Node.KillRollRotation = false;
	}
}
