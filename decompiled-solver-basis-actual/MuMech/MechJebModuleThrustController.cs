using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using KSP.Localization;
using MechJebLibBindings;
using UnityEngine;

namespace MuMech;

public class MechJebModuleThrustController : ComputerModule
{
	public enum DifferentialThrottleStatus
	{
		SUCCESS,
		ALL_ENGINES_OFF,
		MORE_ENGINES_REQUIRED,
		SOLVER_FAILED
	}

	public enum LimitMode
	{
		NONE,
		TEMPERATURE,
		FLAMEOUT,
		ACCELERATION,
		THROTTLE,
		DYNAMIC_PRESSURE,
		MIN_THROTTLE,
		ELECTRIC,
		UNSTABLE_IGNITION,
		AUTO_RCS_ULLAGE
	}

	public enum TMode
	{
		OFF,
		KEEP_ORBITAL,
		KEEP_SURFACE,
		KEEP_VERTICAL,
		DIRECT
	}

	public float TransSpdAct;

	private float _transPrevThrust;

	public bool TransKillH;

	[Persistent(pass = 4)]
	public bool LimitDynamicPressure;

	[Persistent(pass = 4)]
	public readonly EditableDouble MaxDynamicPressure = new EditableDouble(20000.0);

	[Persistent(pass = 4)]
	public bool LimitToPreventOverheats;

	[ToggleInfoItem("#MechJeb_SmoothThrottle", InfoItem.Category.Thrust)]
	[Persistent(pass = 4)]
	public bool SmoothThrottle;

	[Persistent(pass = 4)]
	public double ThrottleSmoothingTime = 1.0;

	[Persistent(pass = 4)]
	public bool LimitToPreventFlameout;

	[Persistent(pass = 4)]
	public bool LimitToPreventUnstableIgnition;

	[Persistent(pass = 4)]
	public bool AutoRCSUllaging = true;

	[Persistent(pass = 4)]
	public readonly EditableDouble FlameoutSafetyPct = 5.0;

	[ToggleInfoItem("#MechJeb_ManageAirIntakes", InfoItem.Category.Thrust)]
	[Persistent(pass = 4)]
	public bool ManageIntakes;

	[Persistent(pass = 4)]
	public bool LimitAcceleration;

	[Persistent(pass = 4)]
	public readonly EditableDouble MaxAcceleration = 40.0;

	[Persistent(pass = 1)]
	public bool LimitThrottle;

	[Persistent(pass = 1)]
	public readonly EditableDoubleMult MaxThrottle = new EditableDoubleMult(1.0, 0.01);

	[Persistent(pass = 7)]
	public bool LimiterMinThrottle = true;

	[Persistent(pass = 7)]
	public readonly EditableDoubleMult MinThrottle = new EditableDoubleMult(0.05, 0.01);

	[Persistent(pass = 2)]
	public bool DifferentialThrottle;

	public Vector3d DifferentialThrottleDemandedTorque;

	public DifferentialThrottleStatus DifferentialThrottleSuccess;

	private readonly Dictionary<ModuleEngines, float> _thrustLimitsBeforeDifferentialThrottle = new Dictionary<ModuleEngines, float>();

	[Persistent(pass = 1)]
	public bool ElectricThrottle;

	[Persistent(pass = 1)]
	public readonly EditableDoubleMult ElectricThrottleLo = new EditableDoubleMult(0.05, 0.01);

	[Persistent(pass = 1)]
	public readonly EditableDoubleMult ElectricThrottleHi = new EditableDoubleMult(0.15, 0.01);

	public LimitMode Limiter;

	public float TargetThrottle;

	private bool _tmodeChanged;

	private PIDController _pid;

	public float LastThrottle;

	private int _userCommandingRotationSmoothed;

	private bool _lastDisableThrusters;

	private TMode _tmode;

	private ScreenMessage _preventingUnstableIgnitionsMessage;

	private static bool _isLoadedRealFuels => ReflectionUtils.IsAssemblyLoaded("RealFuels");

	private bool _userCommandingRotation => _userCommandingRotationSmoothed > 0;

	public TMode Tmode
	{
		get
		{
			return _tmode;
		}
		set
		{
			if (_tmode != value)
			{
				_tmode = value;
				_tmodeChanged = true;
			}
		}
	}

	public float ThrottleLimit { get; set; }

	public float ThrottleFixedLimit { get; set; }

	public MechJebModuleThrustController(MechJebCore core)
		: base(core)
	{
		Priority = 200;
	}

	[GeneralInfoItem("#MechJeb_LimittoMaxQ", InfoItem.Category.Thrust)]
	public void LimitToMaxDynamicPressureInfoItem()
	{
		GUIStyle toggleStyle = ((Limiter == LimitMode.DYNAMIC_PRESSURE) ? GuiUtils.GreenToggle : null);
		GuiUtils.ToggledTextBox(ref LimitDynamicPressure, CachedLocalizer.Instance.MechJebAscentCheckbox11, MaxDynamicPressure, "Pa", toggleStyle, 80f);
	}

	[GeneralInfoItem("#MechJeb_PreventEngineOverheats", InfoItem.Category.Thrust)]
	public void LimitToPreventOverheatsInfoItem()
	{
		GUIStyle val = ((Limiter == LimitMode.TEMPERATURE) ? GuiUtils.GreenToggle : GuiUtils.Skin.toggle);
		LimitToPreventOverheats = GUILayout.Toggle(LimitToPreventOverheats, CachedLocalizer.Instance.MechJebAscentCheckbox12, val, Array.Empty<GUILayoutOption>());
	}

	[GeneralInfoItem("#MechJeb_PreventJetFlameout", InfoItem.Category.Thrust)]
	public void LimitToPreventFlameoutInfoItem()
	{
		GUIStyle val = ((Limiter == LimitMode.FLAMEOUT) ? GuiUtils.GreenToggle : GuiUtils.Skin.toggle);
		LimitToPreventFlameout = GUILayout.Toggle(LimitToPreventFlameout, CachedLocalizer.Instance.MechJebAscentCheckbox13, val, Array.Empty<GUILayoutOption>());
	}

	[GeneralInfoItem("#MechJeb_PreventUnstableIgnition", InfoItem.Category.Thrust)]
	public void LimitToPreventUnstableIgnitionInfoItem()
	{
		GUIStyle val = ((Limiter == LimitMode.UNSTABLE_IGNITION) ? GuiUtils.GreenToggle : GuiUtils.Skin.toggle);
		LimitToPreventUnstableIgnition = GUILayout.Toggle(LimitToPreventUnstableIgnition, CachedLocalizer.Instance.MechJebAscentCheckbox14, val, Array.Empty<GUILayoutOption>());
	}

	[GeneralInfoItem("#MechJeb_UseRCStoullage", InfoItem.Category.Thrust)]
	public void AutoRCsUllageInfoItem()
	{
		GUIStyle val = ((Limiter == LimitMode.AUTO_RCS_ULLAGE) ? GuiUtils.GreenToggle : GuiUtils.Skin.toggle);
		AutoRCSUllaging = GUILayout.Toggle(AutoRCSUllaging, CachedLocalizer.Instance.MechJebAscentCheckbox15, val, Array.Empty<GUILayoutOption>());
	}

	[GeneralInfoItem("#MechJeb_LimitAcceleration", InfoItem.Category.Thrust)]
	public void LimitAccelerationInfoItem()
	{
		GUIStyle toggleStyle = ((Limiter == LimitMode.ACCELERATION) ? GuiUtils.GreenToggle : null);
		GuiUtils.ToggledTextBox(ref LimitAcceleration, CachedLocalizer.Instance.MechJebAscentCheckbox16, MaxAcceleration, "m/s²", toggleStyle, 30f);
	}

	[GeneralInfoItem("#MechJeb_LimitThrottle", InfoItem.Category.Thrust)]
	public void LimitThrottleInfoItem()
	{
		GUIStyle toggleStyle = ((Limiter != LimitMode.THROTTLE) ? null : (((double)MaxThrottle > 0.0) ? GuiUtils.GreenToggle : GuiUtils.RedToggle));
		GuiUtils.ToggledTextBox(ref LimitThrottle, CachedLocalizer.Instance.MechJebAscentCheckbox17, MaxThrottle, "%", toggleStyle, 30f);
	}

	[GeneralInfoItem("#MechJeb_LowerThrottleLimit", InfoItem.Category.Thrust)]
	public void LimiterMinThrottleInfoItem()
	{
		GUIStyle toggleStyle = ((Limiter == LimitMode.MIN_THROTTLE) ? GuiUtils.GreenToggle : null);
		GuiUtils.ToggledTextBox(ref LimiterMinThrottle, CachedLocalizer.Instance.MechJebAscentCheckbox18, MinThrottle, "%", toggleStyle, 30f);
	}

	[GeneralInfoItem("#MechJeb_DifferentialThrottle", InfoItem.Category.Thrust)]
	public void DifferentialThrottleMenu()
	{
		bool differentialThrottle = Core.Thrust.DifferentialThrottle;
		GUIStyle val = ((!DifferentialThrottle || !base.Vessel.LiftedOff()) ? GuiUtils.Skin.toggle : ((Core.Thrust.DifferentialThrottleSuccess == DifferentialThrottleStatus.SUCCESS) ? GuiUtils.GreenToggle : GuiUtils.YellowToggle));
		DifferentialThrottle = GUILayout.Toggle(DifferentialThrottle, CachedLocalizer.Instance.MechJebAscentCheckbox19, val, Array.Empty<GUILayoutOption>());
		if (differentialThrottle && !Core.Thrust.DifferentialThrottle)
		{
			Core.Thrust.DisableDifferentialThrottle();
		}
	}

	[GeneralInfoItem("#MechJeb_ElectricLimit", InfoItem.Category.Thrust)]
	public void LimitElectricInfoItem()
	{
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUIStyle val = GuiUtils.Skin.label;
		if (Limiter == LimitMode.ELECTRIC)
		{
			val = (((double)base.VesselState.ThrottleLimit < 0.001) ? GuiUtils.RedLabel : GuiUtils.YellowLabel);
		}
		else if (ElectricEngineRunning())
		{
			val = GuiUtils.GreenLabel;
		}
		ElectricThrottle = GUILayout.Toggle(ElectricThrottle, CachedLocalizer.Instance.MechJebAscentCheckbox20, val, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(110f) });
		GuiUtils.SimpleTextField(ElectricThrottleLo, 30f);
		GUILayout.Label("% Hi", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.ExpandWidth(b: false) });
		GuiUtils.SimpleTextField(ElectricThrottleHi, 30f);
		GUILayout.Label("%", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.ExpandWidth(b: false) });
		GUILayout.EndHorizontal();
	}

	public override void OnStart(StartState state)
	{
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Expected O, but got Unknown
		//IL_0060: Unknown result type (might be due to invalid IL or missing references)
		_preventingUnstableIgnitionsMessage = new ScreenMessage(Localizer.Format("#MechJeb_Ascent_srcmsg1"), 2f, (ScreenMessageStyle)0);
		_pid = new PIDController(0.05, 1E-06, 0.05);
		Users.Add(this);
		base.OnStart(state);
	}

	public void ThrustOff()
	{
		if (!((Object)(object)base.Vessel == (Object)null) && base.Vessel.ctrlState != null)
		{
			TargetThrottle = 0f;
			base.Vessel.ctrlState.mainThrottle = 0f;
			Tmode = TMode.OFF;
			SetFlightGlobals(0.0);
		}
	}

	public void RequestActiveThrottle(float target, bool enforceMinimum = true, bool allowZero = false)
	{
		if (enforceMinimum && LimiterMinThrottle)
		{
			if (allowZero && target == 0f)
			{
				TargetThrottle = 0f;
			}
			else
			{
				TargetThrottle = Mathf.Max(target, (float)(double)MinThrottle);
			}
		}
		else
		{
			TargetThrottle = target;
		}
	}

	private void SetFlightGlobals(double throttle)
	{
		if (FlightGlobals.ActiveVessel != null && (Object)(object)base.Vessel == (Object)(object)FlightGlobals.ActiveVessel)
		{
			FlightInputHandler.state.mainThrottle = (float)throttle;
		}
	}

	public void ThrustForDv(double dV, double timeConstant)
	{
		timeConstant += base.VesselState.MaxEngineResponseTime;
		double num = base.VesselState.CurrentThrustAcceleration * base.VesselState.MaxEngineResponseTime;
		double num2 = (dV - num) / timeConstant;
		TargetThrottle = Mathf.Clamp((float)(num2 / base.VesselState.MaxThrustAcceleration), 0.01f, 1f);
	}

	private void SetTempLimit(float limit, LimitMode mode)
	{
		ThrottleLimit = limit;
		Limiter = mode;
	}

	private void SetFixedLimit(float limit, LimitMode mode)
	{
		if (ThrottleLimit > limit)
		{
			ThrottleLimit = limit;
		}
		ThrottleFixedLimit = limit;
		Limiter = mode;
	}

	public override void Drive(FlightCtrlState s)
	{
		//IL_0162: Unknown result type (might be due to invalid IL or missing references)
		//IL_0167: Unknown result type (might be due to invalid IL or missing references)
		//IL_0172: Unknown result type (might be due to invalid IL or missing references)
		//IL_0177: Unknown result type (might be due to invalid IL or missing references)
		//IL_017c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0181: Unknown result type (might be due to invalid IL or missing references)
		//IL_0183: Unknown result type (might be due to invalid IL or missing references)
		//IL_0185: Unknown result type (might be due to invalid IL or missing references)
		//IL_0190: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_028a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0295: Unknown result type (might be due to invalid IL or missing references)
		//IL_0244: Unknown result type (might be due to invalid IL or missing references)
		//IL_0249: Unknown result type (might be due to invalid IL or missing references)
		//IL_024e: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_025b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0232: Unknown result type (might be due to invalid IL or missing references)
		//IL_0234: Unknown result type (might be due to invalid IL or missing references)
		//IL_0239: Unknown result type (might be due to invalid IL or missing references)
		//IL_023e: Unknown result type (might be due to invalid IL or missing references)
		//IL_084e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0859: Unknown result type (might be due to invalid IL or missing references)
		//IL_0865: Unknown result type (might be due to invalid IL or missing references)
		//IL_0873: Unknown result type (might be due to invalid IL or missing references)
		//IL_0878: Unknown result type (might be due to invalid IL or missing references)
		//IL_087d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0882: Unknown result type (might be due to invalid IL or missing references)
		bool num = !Mathfx.Approx(s.pitch, s.pitchTrim, 0.1f) || !Mathfx.Approx(s.yaw, s.yawTrim, 0.1f) || !Mathfx.Approx(s.roll, s.rollTrim, 0.1f);
		bool flag = !(Math.Abs(s.X) < 0.1f) || !(Math.Abs(s.Y) < 0.1f) || !(Math.Abs(s.Z) < 0.1f);
		if (num && !flag)
		{
			_userCommandingRotationSmoothed = 2;
		}
		else if (_userCommandingRotationSmoothed > 0)
		{
			_userCommandingRotationSmoothed--;
		}
		if (Core.GetComputerModule<MechJebModuleThrustWindow>().Hidden && Core.GetComputerModule<MechJebModuleAscentMenu>().Hidden)
		{
			return;
		}
		if (Tmode != 0 && base.VesselState.ThrustAvailable > 0.0)
		{
			double num2 = 0.0;
			switch (Tmode)
			{
			case TMode.KEEP_ORBITAL:
				num2 = base.VesselState.SpeedOrbital;
				break;
			case TMode.KEEP_SURFACE:
				num2 = base.VesselState.SpeedSurface;
				break;
			case TMode.KEEP_VERTICAL:
				num2 = base.VesselState.SpeedVertical;
				if (TransKillH)
				{
					Vector3 val = Vector3.ProjectOnPlane(Vector3d.op_Implicit(base.VesselState.SurfaceVelocity), Vector3d.op_Implicit(base.VesselState.Up));
					Vector3 val2 = Vector3d.op_Implicit(-val + base.VesselState.Up * Math.Max(Math.Abs(num2), 20.0 * base.MainBody.GeeASL));
					Vector3d direction;
					if (Math.Min(base.VesselState.AltitudeASL, base.VesselState.AltitudeTrue) > 5000.0 && (double)((Vector3)(ref val)).magnitude > Math.Max(Math.Abs(num2), 100.0 * base.MainBody.GeeASL) * 2.0)
					{
						Tmode = TMode.DIRECT;
						TransSpdAct = 100f;
						direction = Vector3d.op_Implicit(-val);
					}
					else
					{
						direction = Vector3d.op_Implicit(((Vector3)(ref val2)).normalized);
					}
					Core.Attitude.attitudeTo(direction, AttitudeReference.INERTIAL, null);
				}
				break;
			}
			double num3 = ((double)TransSpdAct - num2) / base.VesselState.MaxThrustAcceleration;
			if ((Tmode == TMode.KEEP_ORBITAL && Vector3d.Dot(base.VesselState.Forward, base.VesselState.OrbitalVelocity) < 0.0) || (Tmode == TMode.KEEP_SURFACE && Vector3d.Dot(base.VesselState.Forward, base.VesselState.SurfaceVelocity) < 0.0))
			{
				num3 *= -1.0;
			}
			double num4 = _pid.Compute(num3);
			if (Tmode != TMode.KEEP_VERTICAL || !TransKillH || Core.Attitude.attitudeError < 2.0 || (Math.Min(base.VesselState.AltitudeASL, base.VesselState.AltitudeTrue) < 1000.0 && Core.Attitude.attitudeError < 90.0))
			{
				if (Tmode == TMode.DIRECT)
				{
					_transPrevThrust = (TargetThrottle = TransSpdAct / 100f);
				}
				else
				{
					_transPrevThrust = (TargetThrottle = Mathf.Clamp01(_transPrevThrust + (float)num4));
				}
			}
			else
			{
				bool flag2 = base.VesselState.TorqueGimbal.Positive.x > base.VesselState.TorqueAvailable.x * 10.0 || base.VesselState.TorqueGimbal.Positive.z > base.VesselState.TorqueAvailable.z * 10.0;
				bool flag3 = base.VesselState.TorqueDifferentialThrottle.x > base.VesselState.TorqueAvailable.x * 10.0 || base.VesselState.TorqueDifferentialThrottle.z > base.VesselState.TorqueAvailable.z * 10.0;
				if (Core.Attitude.attitudeError >= 2.0 && (flag2 || (flag3 && Core.Thrust.DifferentialThrottle)))
				{
					_transPrevThrust = (TargetThrottle = 0.1f);
					ComputerModule.Print(" targetThrottle = 0.1F");
				}
				else
				{
					_transPrevThrust = (TargetThrottle = 0f);
				}
			}
		}
		if (Users.Count > 1)
		{
			s.mainThrottle = TargetThrottle;
		}
		ThrottleLimit = 1f;
		ThrottleFixedLimit = 1f;
		Limiter = LimitMode.NONE;
		if (LimitThrottle && (double)MaxThrottle < (double)ThrottleLimit)
		{
			SetFixedLimit((float)(double)MaxThrottle, LimitMode.THROTTLE);
		}
		if (LimitDynamicPressure)
		{
			float num5 = MaximumDynamicPressureThrottle();
			if (num5 < ThrottleLimit)
			{
				SetFixedLimit(num5, LimitMode.DYNAMIC_PRESSURE);
			}
		}
		if (LimitToPreventOverheats)
		{
			float num6 = (float)TemperatureSafetyThrottle();
			if (num6 < ThrottleLimit)
			{
				SetFixedLimit(num6, LimitMode.TEMPERATURE);
			}
		}
		if (LimitAcceleration)
		{
			float num7 = AccelerationLimitedThrottle();
			if (num7 < ThrottleLimit)
			{
				SetFixedLimit(num7, LimitMode.ACCELERATION);
			}
		}
		if (ElectricThrottle && ElectricEngineRunning())
		{
			float num8 = ElectricThrottleLimit();
			if (num8 < ThrottleLimit)
			{
				SetFixedLimit(num8, LimitMode.ELECTRIC);
			}
		}
		if (LimitToPreventFlameout)
		{
			float num9 = FlameoutSafetyThrottle();
			if (num9 < ThrottleLimit)
			{
				SetFixedLimit(num9, LimitMode.FLAMEOUT);
			}
		}
		if (LimiterMinThrottle && Limiter != 0)
		{
			if ((double)MinThrottle > (double)ThrottleFixedLimit)
			{
				SetFixedLimit((float)(double)MinThrottle, LimitMode.MIN_THROTTLE);
			}
			if ((double)MinThrottle > (double)ThrottleLimit)
			{
				SetTempLimit((float)(double)MinThrottle, LimitMode.MIN_THROTTLE);
			}
		}
		ProcessUllage(s);
		if (LimitToPreventUnstableIgnition && s.mainThrottle > 0f && ThrottleLimit > 0f && base.VesselState.LowestUllage < 0.996)
		{
			ScreenMessages.PostScreenMessage(_preventingUnstableIgnitionsMessage);
			Debug.Log((object)("MechJeb Unstable Ignitions: preventing ignition in state: " + base.VesselState.LowestUllage));
			SetTempLimit(0f, LimitMode.UNSTABLE_IGNITION);
		}
		if (Core.RssMode)
		{
			SetFlightGlobals(s.mainThrottle);
		}
		if (double.IsNaN(ThrottleLimit))
		{
			ThrottleLimit = 1f;
		}
		ThrottleLimit = Mathf.Clamp01(ThrottleLimit);
		if (double.IsNaN(ThrottleFixedLimit))
		{
			ThrottleFixedLimit = 1f;
		}
		ThrottleFixedLimit = Mathf.Clamp01(ThrottleFixedLimit);
		base.VesselState.ThrottleLimit = ThrottleLimit;
		base.VesselState.ThrottleFixedLimit = ThrottleFixedLimit;
		if (s.mainThrottle < ThrottleLimit)
		{
			Limiter = LimitMode.NONE;
		}
		s.mainThrottle = Mathf.Min(s.mainThrottle, ThrottleLimit);
		if (SmoothThrottle)
		{
			s.mainThrottle = ApplySmoothThrottle(s.mainThrottle);
		}
		if (double.IsNaN(s.mainThrottle))
		{
			s.mainThrottle = 0f;
		}
		s.mainThrottle = Mathf.Clamp01(s.mainThrottle);
		if (s.Z == 0f && Core.RCS.rcsThrottle && base.VesselState.RCSThrust)
		{
			s.Z = 0f - s.mainThrottle;
		}
		LastThrottle = s.mainThrottle;
		if (!Core.Attitude.Enabled)
		{
			Vector3d val3 = default(Vector3d);
			((Vector3d)(ref val3))._002Ector((double)s.pitch, (double)s.yaw, (double)s.roll);
			DifferentialThrottleDemandedTorque = -Vector3d.Scale(((Vector3d)(ref val3)).xzy, base.VesselState.TorqueDifferentialThrottle * (double)s.mainThrottle * 0.5);
		}
	}

	public override void OnFixedUpdate()
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		if (DifferentialThrottle)
		{
			DifferentialThrottleSuccess = ComputeDifferentialThrottle(DifferentialThrottleDemandedTorque);
			return;
		}
		DisableDifferentialThrottle();
		DifferentialThrottleSuccess = DifferentialThrottleStatus.SUCCESS;
	}

	public override void OnVesselWasModified(Vessel v)
	{
		DisableDifferentialThrottle();
	}

	public override void OnDestroy()
	{
		DisableDifferentialThrottle();
		base.OnDestroy();
	}

	private float MaximumDynamicPressureThrottle()
	{
		if ((double)MaxDynamicPressure <= 0.0)
		{
			return 1f;
		}
		double num = base.VesselState.DynamicPressure / (double)MaxDynamicPressure;
		if (num < 1.0)
		{
			return 1f;
		}
		return Mathf.Clamp((float)(1.0 - 15.0 * (num - 1.0)), 0f, 1f);
	}

	private double TemperatureSafetyThrottle()
	{
		double num = base.Vessel.parts.Max((Part p) => p.temperature / p.maxTemp);
		if (num < 0.9499999992549419)
		{
			return 1.0;
		}
		return (1.0 - num) / 0.05000000074505806;
	}

	private float ApplySmoothThrottle(float mainThrottle)
	{
		return Mathf.Clamp(mainThrottle, (float)((double)LastThrottle - base.VesselState.DeltaT / ThrottleSmoothingTime), (float)((double)LastThrottle + base.VesselState.DeltaT / ThrottleSmoothingTime));
	}

	private float FlameoutSafetyThrottle()
	{
		float num = 1f;
		foreach (VesselState.ResourceInfo value in base.VesselState.Resources.Values)
		{
			if (value.Intakes.Count != 0)
			{
				double val = (1.0 + 0.01 * (double)FlameoutSafetyPct) * value.RequiredAtMaxThrottle;
				val = Math.Max(val, value.Required);
				if (ManageIntakes)
				{
					OptimizeIntakes(value, val);
				}
				double intakeProvided = value.IntakeProvided;
				if (value.Required >= intakeProvided)
				{
					num = 0f;
					continue;
				}
				double num2 = intakeProvided / val;
				num = Mathf.Min(num, (float)num2);
			}
		}
		return num;
	}

	private void OptimizeIntakes(VesselState.ResourceInfo info, double requiredFlow)
	{
		//IL_012e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0135: Expected O, but got Unknown
		List<List<ModuleResourceIntake>> list = new List<List<ModuleResourceIntake>>();
		Dictionary<ModuleResourceIntake, int> dictionary = new Dictionary<ModuleResourceIntake, int>();
		Dictionary<ModuleResourceIntake, VesselState.ResourceInfo.IntakeData> dictionary2 = new Dictionary<ModuleResourceIntake, VesselState.ResourceInfo.IntakeData>();
		foreach (VesselState.ResourceInfo.IntakeData intake2 in info.Intakes)
		{
			ModuleResourceIntake intake = intake2.Intake;
			dictionary2[intake] = intake2;
			if (dictionary.ContainsKey(intake))
			{
				continue;
			}
			int count = list.Count;
			List<ModuleResourceIntake> list2 = new List<ModuleResourceIntake>();
			list.Add(list2);
			Stack<Part> stack = new Stack<Part>();
			stack.Push(((PartModule)intake).part);
			while (stack.Count > 0)
			{
				Part val = stack.Pop();
				ModuleResourceIntake val2 = ((IEnumerable)val.Modules).OfType<ModuleResourceIntake>().FirstOrDefault();
				if ((Object)(object)val2 == (Object)null || dictionary.ContainsKey(val2))
				{
					continue;
				}
				dictionary[val2] = count;
				list2.Add(val2);
				foreach (Part symmetryCounterpart in val.symmetryCounterparts)
				{
					stack.Push(symmetryCounterpart);
				}
			}
		}
		double num = 0.0;
		KSPActionParam val3 = new KSPActionParam((KSPActionGroup)0, (KSPActionType)0);
		foreach (List<ModuleResourceIntake> item in list.OrderBy((List<ModuleResourceIntake> grp) => grp.Count))
		{
			if (num < requiredFlow)
			{
				foreach (ModuleResourceIntake item2 in item)
				{
					double predictedMassFlow = dictionary2[item2].PredictedMassFlow;
					if (!item2.intakeEnabled)
					{
						item2.ToggleAction(val3);
					}
					num += predictedMassFlow;
				}
				continue;
			}
			foreach (ModuleResourceIntake item3 in item)
			{
				if (item3.intakeEnabled)
				{
					item3.ToggleAction(val3);
				}
			}
		}
	}

	private bool ElectricEngineRunning()
	{
		return (from p in base.Vessel.parts
			where p.inverseStage >= base.Vessel.currentStage && p.IsEngine() && !p.IsSepratron()
			select ((IEnumerable)p.Modules).OfType<ModuleEngines>().First((ModuleEngines e) => ((PartModule)e).isEnabled)).SelectMany((ModuleEngines eng) => eng.propellants).Any((Propellant p) => p.id == PartResourceLibrary.ElectricityHashcode);
	}

	private float ElectricThrottleLimit()
	{
		double num = base.Vessel.MaxResourceAmount(PartResourceLibrary.ElectricityHashcode);
		double num2 = num * (double)ElectricThrottleLo;
		double num3 = num * (double)ElectricThrottleHi;
		double num4 = base.Vessel.TotalResourceAmount(PartResourceLibrary.ElectricityHashcode);
		if (num4 <= num2)
		{
			return 0f;
		}
		if (num4 >= num3)
		{
			return 1f;
		}
		if (Math.Abs(num3 - num2) < 0.01)
		{
			return 1f;
		}
		return Mathf.Clamp((float)((num4 - num2) / (num3 - num2)), 0f, 1f);
	}

	private float AccelerationLimitedThrottle()
	{
		return Mathf.Clamp((float)(((double)MaxAcceleration - base.VesselState.MinThrustAcceleration) / (base.VesselState.MaxThrustAcceleration - base.VesselState.MinThrustAcceleration)), 0f, 1f);
	}

	public override void OnUpdate()
	{
		if (Core.GetComputerModule<MechJebModuleThrustWindow>().Hidden && Core.GetComputerModule<MechJebModuleAscentMenu>().Hidden)
		{
			return;
		}
		if (_tmodeChanged)
		{
			if (TransKillH && Tmode == TMode.OFF)
			{
				Core.Attitude.attitudeDeactivate();
			}
			_pid.Reset();
			_tmodeChanged = false;
			FlightInputHandler.SetNeutralControls();
		}
		bool flag = _userCommandingRotation && !Core.RCS.rcsForRotation;
		if (flag == _lastDisableThrusters)
		{
			return;
		}
		_lastDisableThrusters = flag;
		foreach (ModuleRCS item in base.Vessel.FindPartModulesImplementing<ModuleRCS>())
		{
			if (flag)
			{
				item.enablePitch = (item.enableRoll = (item.enableYaw = false));
			}
			else
			{
				item.enablePitch = (item.enableRoll = (item.enableYaw = true));
			}
		}
	}

	private void ProcessUllage(FlightCtrlState s)
	{
		if (!_isLoadedRealFuels || !AutoRCSUllaging || s.mainThrottle <= 0f || ThrottleLimit <= 0f || !base.Vessel.hasEnabledRCSModules())
		{
			return;
		}
		bool flag = base.VesselState.LowestUllage >= 0.996;
		if (flag && base.VesselState.ThrustCurrent > base.VesselState.RCSThrustAvailable.Up)
		{
			return;
		}
		double num = (base.VesselState.ThrustAvailable - base.VesselState.ThrustMinimum) * (double)s.mainThrottle + base.VesselState.ThrustMinimum;
		if (!flag || !(num < base.VesselState.RCSThrustAvailable.Up))
		{
			if (LastThrottle <= 0f && !flag)
			{
				SetTempLimit(0f, LimitMode.AUTO_RCS_ULLAGE);
			}
			if (!base.Vessel.ActionGroups[(KSPActionGroup)8])
			{
				base.Vessel.ActionGroups.SetGroup((KSPActionGroup)8, true);
			}
			s.Z = -1f;
		}
	}

	private DifferentialThrottleStatus ComputeDifferentialThrottle(Vector3d torque)
	{
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		//IL_0078: Unknown result type (might be due to invalid IL or missing references)
		//IL_008d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0203: Unknown result type (might be due to invalid IL or missing references)
		//IL_0208: Unknown result type (might be due to invalid IL or missing references)
		//IL_021c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0221: Unknown result type (might be due to invalid IL or missing references)
		float num = base.Vessel.ctrlState.mainThrottle;
		if (num == 0f)
		{
			torque = Vector3d.zero;
			num = 1f;
		}
		int count = base.VesselState.EngineWrappers.Count;
		double num2 = 0.0;
		double num3 = 0.0;
		Vector3d val = default(Vector3d);
		for (int i = 0; i < count; i++)
		{
			torque -= base.VesselState.EngineWrappers[i].ConstantTorque;
			double num4 = num2;
			Vector3d val2 = base.VesselState.EngineWrappers[i].MaxVariableTorque;
			num2 = num4 + ((Vector3d)(ref val2)).magnitude;
			val += Vector3d.Dot((double)num * base.VesselState.EngineWrappers[i].MaxVariableForce, Vector3d.up) * Vector3d.up;
			double num5 = num3;
			val2 = base.VesselState.EngineWrappers[i].MaxVariableForce;
			num3 = num5 + ((Vector3d)(ref val2)).magnitude * 10.0;
		}
		List<VesselState.EngineWrapper> list = base.VesselState.EngineWrappers.Where((VesselState.EngineWrapper eng) => !eng.Engine.throttleLocked).ToList();
		int count2 = list.Count;
		switch (count)
		{
		case 0:
			return DifferentialThrottleStatus.ALL_ENGINES_OFF;
		default:
			if (count2 != 0)
			{
				double[,] array = new double[count2, count2];
				double[] array2 = new double[count2];
				double[] array3 = new double[count2];
				double[] array4 = new double[count2];
				for (int j = 0; j < count2; j++)
				{
					for (int k = 0; k < count2; k++)
					{
						array[j, k] = Vector3d.Dot(list[j].MaxVariableTorque, list[k].MaxVariableTorque) / (num2 * num2) + Vector3d.Dot(list[j].MaxVariableForce, list[k].MaxVariableForce) / (num3 * num3);
					}
					array2[j] = (0.0 - Vector3d.Dot(list[j].MaxVariableTorque, torque)) / (num2 * num2) - Vector3d.Dot(list[j].MaxVariableForce, val) / (num3 * num3);
					array3[j] = 0.0;
					array4[j] = num;
				}
				minqpstate val3 = default(minqpstate);
				alglib.minqpcreate(count2, ref val3);
				alglib.minqpsetquadraticterm(val3, array, false);
				alglib.minqpsetlinearterm(val3, array2);
				alglib.minqpsetbc(val3, array3, array4);
				alglib.minqpsetalgodensegenipm(val3, 0.0);
				alglib.minqpoptimize(val3);
				double[] array5 = default(double[]);
				minqpreport val4 = default(minqpreport);
				alglib.minqpresults(val3, ref array5, ref val4);
				if (array5.Any(double.IsNaN))
				{
					return DifferentialThrottleStatus.SOLVER_FAILED;
				}
				for (int l = 0; l < count2; l++)
				{
					if (!_thrustLimitsBeforeDifferentialThrottle.ContainsKey(list[l].Engine))
					{
						_thrustLimitsBeforeDifferentialThrottle.Add(list[l].Engine, list[l].Engine.thrustPercentage);
					}
					list[l].ThrustRatio = (float)(array5[l] / (double)num);
				}
				return DifferentialThrottleStatus.SUCCESS;
			}
			goto case 1;
		case 1:
			return DifferentialThrottleStatus.MORE_ENGINES_REQUIRED;
		}
	}

	private void DisableDifferentialThrottle()
	{
		foreach (KeyValuePair<ModuleEngines, float> item in _thrustLimitsBeforeDifferentialThrottle)
		{
			if ((Object)(object)item.Key != (Object)null)
			{
				item.Key.thrustPercentage = item.Value;
			}
		}
		_thrustLimitsBeforeDifferentialThrottle.Clear();
	}
}
