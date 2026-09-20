using System;
using KSP.Localization;
using MechJebLib.Functions;
using MechJebLib.PSG;
using UnityEngine;

namespace MuMech;

public class MechJebModuleAscentMenu : DisplayModule
{
	private readonly string[] _ascentPathList = new string[2]
	{
		Localizer.Format("#MechJeb_Ascent_ascentPathList1"),
		Localizer.Format("#MechJeb_Ascent_ascentPathList3")
	};

	[Persistent(pass = 4)]
	public bool _lastPSGSettingsEnabled;

	[Persistent(pass = 4)]
	public bool _lastSettingsMenuEnabled;

	private static GUIStyle _btNormal;

	private static GUIStyle _btActive;

	private DateTime _lastRefresh = DateTime.MinValue;

	private string vgo;

	private string heading;

	private string tgo;

	private string pitch;

	private string label26;

	private string label27;

	private string label28;

	private string n;

	private string label29;

	private string znorm;

	private string label30;

	private string launchTimer;

	private string autopilotStatus;

	private TimeSpan _refreshInterval = TimeSpan.FromSeconds(0.1);

	[Persistent(pass = 4)]
	public EditableInt _refreshRate = 10;

	private bool _launchingToPlane
	{
		get
		{
			return _ascentSettings.LaunchingToPlane;
		}
		set
		{
			_ascentSettings.LaunchingToPlane = value;
		}
	}

	private bool _launchingToMatchLan
	{
		get
		{
			return _ascentSettings.LaunchingToMatchLan;
		}
		set
		{
			_ascentSettings.LaunchingToMatchLan = value;
		}
	}

	private bool _launchingToLan
	{
		get
		{
			return _ascentSettings.LaunchingToLan;
		}
		set
		{
			_ascentSettings.LaunchingToLan = value;
		}
	}

	private bool _launchingWithAnyPlaneControl
	{
		get
		{
			if (!_launchingToPlane && !_launchingToMatchLan)
			{
				return _launchingToLan;
			}
			return true;
		}
	}

	private MechJebModuleAscentBaseAutopilot _autopilot => Core.Ascent;

	private MechJebModuleAscentSettings _ascentSettings => Core.AscentSettings;

	private MechJebModuleAscentClassicPathMenu _classicPathMenu => Core.GetComputerModule<MechJebModuleAscentClassicPathMenu>();

	private MechJebModuleAscentPSGSettingsMenu _psgSettingsMenu => Core.GetComputerModule<MechJebModuleAscentPSGSettingsMenu>();

	private MechJebModuleAscentSettingsMenu _settingsMenu => Core.GetComputerModule<MechJebModuleAscentSettingsMenu>();

	public MechJebModuleAscentMenu(MechJebCore core)
		: base(core)
	{
	}

	protected override void OnModuleEnabled()
	{
		_psgSettingsMenu.Enabled = _lastPSGSettingsEnabled;
		_settingsMenu.Enabled = _lastSettingsMenuEnabled;
	}

	protected override void OnModuleDisabled()
	{
		_launchingToPlane = false;
		_launchingToMatchLan = false;
		_lastPSGSettingsEnabled = _psgSettingsMenu.Enabled;
		_lastSettingsMenuEnabled = _settingsMenu.Enabled;
		_psgSettingsMenu.Enabled = false;
		_settingsMenu.Enabled = false;
	}

	private void SetupButtonStyles()
	{
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Expected O, but got Unknown
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_0097: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_009e: Unknown result type (might be due to invalid IL or missing references)
		//IL_009f: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c7: Expected O, but got Unknown
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d6: Expected O, but got Unknown
		if (_btNormal == null)
		{
			_btNormal = new GUIStyle(GUI.skin.button);
			GUIStyleState normal = _btNormal.normal;
			Color textColor = (_btNormal.focused.textColor = Color.white);
			normal.textColor = textColor;
			GUIStyleState hover = _btNormal.hover;
			textColor = (_btNormal.active.textColor = Color.yellow);
			hover.textColor = textColor;
			GUIStyleState onNormal = _btNormal.onNormal;
			GUIStyleState onFocused = _btNormal.onFocused;
			GUIStyleState onHover = _btNormal.onHover;
			Color val = (_btNormal.onActive.textColor = Color.green);
			Color val3 = (onHover.textColor = val);
			textColor = (onFocused.textColor = val3);
			onNormal.textColor = textColor;
			_btNormal.padding = new RectOffset(8, 8, 8, 8);
			_btActive = new GUIStyle(_btNormal);
			_btActive.active = _btActive.onActive;
			_btActive.normal = _btActive.onNormal;
			_btActive.onFocused = _btActive.focused;
			_btActive.hover = _btActive.onHover;
		}
	}

	private void VisibleSectionsGUIElements()
	{
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		if (_autopilot.Enabled && GUILayout.Button(CachedLocalizer.Instance.MechJebAscentButton1, Array.Empty<GUILayoutOption>()))
		{
			_autopilot.Users.Remove(this);
		}
		else if (!_autopilot.Enabled && GUILayout.Button(CachedLocalizer.Instance.MechJebAscentButton2, Array.Empty<GUILayoutOption>()))
		{
			_autopilot.Users.Add(this);
		}
		GUILayout.EndVertical();
	}

	private void ShowTargetingGUIElements()
	{
		//IL_02dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f6: Expected O, but got Unknown
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		if (_ascentSettings.AscentType == AscentType.PSG)
		{
			if (_ascentSettings.OptimizeStageFlag)
			{
				GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel1, _ascentSettings.DesiredOrbitAltitude, "km");
				GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel2, _ascentSettings.DesiredApoapsis, "km");
				GuiUtils.ToggledTextBox(ref _ascentSettings.AttachAltFlag, CachedLocalizer.Instance.MechJebAscentAttachAlt, _ascentSettings.DesiredAttachAlt, "km");
				GuiUtils.ToggledTextBox(ref _ascentSettings.DesiredArgPFlag, "Arg Periapsis:", _ascentSettings.DesiredArgP, "°");
			}
			else
			{
				if (!_launchingWithAnyPlaneControl)
				{
					GuiUtils.SimpleTextBox("Flight Path Angle", _ascentSettings.DesiredFPA, "°");
				}
				GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentAttachAlt, _ascentSettings.DesiredAttachAltFixed, "km");
			}
			if ((double)_ascentSettings.DesiredApoapsis + base.MainBody.Radius >= 0.0 && (double)_ascentSettings.DesiredApoapsis < (double)_ascentSettings.DesiredOrbitAltitude)
			{
				GUILayout.Label(CachedLocalizer.Instance.MechJebAscentLabel3, GuiUtils.YellowLabel, Array.Empty<GUILayoutOption>());
			}
			else if (_ascentSettings.AttachAltFlag && (double)_ascentSettings.DesiredAttachAlt > (double)_ascentSettings.DesiredApoapsis)
			{
				GUILayout.Label(CachedLocalizer.Instance.MechJebAscentWarnAttachAltHigh, GuiUtils.OrangeLabel, Array.Empty<GUILayoutOption>());
			}
			if ((double)_ascentSettings.DesiredApoapsis + base.MainBody.Radius < 0.0)
			{
				GUILayout.Label(CachedLocalizer.Instance.MechJebAscentLabel4, GuiUtils.OrangeLabel, Array.Empty<GUILayoutOption>());
			}
			if (_ascentSettings.AttachAltFlag && (double)_ascentSettings.DesiredAttachAlt < (double)_ascentSettings.DesiredOrbitAltitude)
			{
				GUILayout.Label(CachedLocalizer.Instance.MechJebAscentWarnAttachAltLow, GuiUtils.OrangeLabel, Array.Empty<GUILayoutOption>());
			}
		}
		else
		{
			GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel5, _ascentSettings.DesiredOrbitAltitude, "km");
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel6, _ascentSettings.DesiredInclination, "º", 75f, GuiUtils.Skin.label, horizontalFraming: false);
		if (GUILayout.Button(new GUIContent(CachedLocalizer.Instance.MechJebAscentButton13, "Sets inclination to launch site latitude (for due-east launch). Lower inclinations require a costly plane change."), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.ExpandWidth(b: false) }))
		{
			_ascentSettings.DesiredInclination.Val = Math.Round(base.VesselState.Latitude, 3);
		}
		GUILayout.EndHorizontal();
		double num = Math.Abs(_ascentSettings.DesiredInclination);
		double num2 = Math.Abs(base.VesselState.Latitude) - ((num < 90.0) ? num : (180.0 - num));
		if (2.001 < num2)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Ascent_label7", new object[1] { num2 }), GuiUtils.RedLabel, Array.Empty<GUILayoutOption>());
		}
		GUILayout.EndVertical();
	}

	private void ShowStatusGUIElements()
	{
		if (_ascentSettings.AscentType != AscentType.PSG)
		{
			return;
		}
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		if (Core.Guidance.Solution != null)
		{
			Solution solution = Core.Guidance.Solution;
			for (int num = solution.Segments - 1; num >= 0; num--)
			{
				GUILayout.Label(PhaseString(solution, base.VesselState.Time, num) ?? "", Array.Empty<GUILayoutOption>());
			}
			GUILayout.Label(solution.TerminalString(), Array.Empty<GUILayoutOption>());
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(vgo, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(90f) });
		GUILayout.Label(heading, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(140f) });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(tgo, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(90f) });
		GUILayout.Label(pitch, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(140f) });
		GUILayout.EndHorizontal();
		GUIStyle val = (Core.Guidance.IsStable() ? GuiUtils.GreenLabel : ((!Core.Guidance.IsInitializing() && Core.Guidance.Status != PSGStatus.FINISHED) ? GuiUtils.RedLabel : GuiUtils.OrangeLabel));
		GUILayout.Label(label26, val, Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(label27, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(90f) });
		GUILayout.Label(label29, Array.Empty<GUILayoutOption>());
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(n, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(90f) });
		GUILayout.Label(znorm, Array.Empty<GUILayoutOption>());
		GUILayout.EndHorizontal();
		if (Core.Glueball.LastFailureMessage != null)
		{
			GUILayout.Label(label30, GuiUtils.RedLabel, Array.Empty<GUILayoutOption>());
		}
		GUILayout.EndVertical();
	}

	private void ShowAutoWarpGUIElements()
	{
		if (!base.Vessel.LandedOrSplashed)
		{
			return;
		}
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		if (Core.Node.Autowarp)
		{
			GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel33, _ascentSettings.WarpCountDown, "s", 35f);
		}
		bool flag = Core.Target.NormalTargetExists && (Object)(object)Core.Target.TargetOrbit?.referenceBody == (Object)(object)base.VesselState.MainBody;
		if (!_launchingWithAnyPlaneControl && !flag)
		{
			bool launchingToPlane = (_launchingToMatchLan = false);
			_launchingToPlane = launchingToPlane;
			if (Core.Target.NormalTargetExists)
			{
				GUILayout.Label(CachedLocalizer.Instance.MechJebAscentWarnInvalidTarget, GuiUtils.OrangeLabel, Array.Empty<GUILayoutOption>());
			}
			else
			{
				GUILayout.Label(CachedLocalizer.Instance.MechJebAscentLabel34, Array.Empty<GUILayoutOption>());
			}
		}
		if (!_launchingWithAnyPlaneControl)
		{
			if (flag && GuiUtils.ButtonTextBox(CachedLocalizer.Instance.MechJebAscentButton15, _ascentSettings.LaunchLANDifference, "º", null, 40f))
			{
				_launchingToPlane = true;
				var (num, val) = Astro.MinimumTimeToPlane(base.MainBody.rotationPeriod, base.VesselState.Latitude, base.VesselState.CelestialLongitude, Core.Target.TargetOrbit.LAN - (double)_ascentSettings.LaunchLANDifference, Core.Target.TargetOrbit.inclination);
				_autopilot.StartCountdown(base.VesselState.Time + num);
				_ascentSettings.DesiredInclination.Val = val;
			}
			if (flag && _ascentSettings.AscentType == AscentType.PSG && GuiUtils.ButtonTextBox(CachedLocalizer.Instance.MechJebAscentLaunchToTargetLan, _ascentSettings.LaunchLANDifference, "º", null, 40f))
			{
				_launchingToMatchLan = true;
				_autopilot.StartCountdown(base.VesselState.Time + Astro.TimeToPlane(base.MainBody.rotationPeriod, base.VesselState.Latitude, base.VesselState.CelestialLongitude, Core.Target.TargetOrbit.LAN - (double)_ascentSettings.LaunchLANDifference, (double)_ascentSettings.DesiredInclination));
			}
			if (_ascentSettings.AscentType == AscentType.PSG)
			{
				if (GuiUtils.ButtonTextBox(CachedLocalizer.Instance.MechJebAscentLaunchToLan, _ascentSettings.DesiredLan, "º", null, 40f))
				{
					_launchingToLan = true;
					_autopilot.StartCountdown(base.VesselState.Time + Astro.TimeToPlane(base.MainBody.rotationPeriod, base.VesselState.Latitude, base.VesselState.CelestialLongitude, (double)_ascentSettings.DesiredLan, (double)_ascentSettings.DesiredInclination));
				}
				_ascentSettings.RelativeLAN = GUILayout.Toggle(_ascentSettings.RelativeLAN, "Use LAN relative to launch site", Array.Empty<GUILayoutOption>());
			}
		}
		if (_launchingWithAnyPlaneControl)
		{
			GUILayout.Label(launchTimer, Array.Empty<GUILayoutOption>());
			if (GUILayout.Button(CachedLocalizer.Instance.MechJebAscentButton17, Array.Empty<GUILayoutOption>()))
			{
				bool flag4 = (_launchingToLan = (_autopilot.TimedLaunch = false));
				bool launchingToPlane = (_launchingToMatchLan = flag4);
				_launchingToPlane = launchingToPlane;
			}
		}
		_ascentSettings.OverrideWarpToPlane = GUILayout.Toggle(_ascentSettings.OverrideWarpToPlane, "Override Warp to Plane", Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
	}

	private void UpdateStrings()
	{
		DateTime now = DateTime.Now;
		if (!(now <= _lastRefresh + _refreshInterval))
		{
			_lastRefresh = now;
			vgo = $"vgo: {Core.Guidance.Vgo:F1}";
			heading = $"heading: {Core.Guidance.Heading:F1}";
			tgo = $"tgo: {Core.Guidance.Tgo:F3}";
			pitch = $"pitch: {Core.Guidance.Pitch:F1}";
			label26 = $"{CachedLocalizer.Instance.MechJebAscentLabel26}{Core.Guidance.Status}";
			label27 = $"{CachedLocalizer.Instance.MechJebAscentLabel27}{Core.Glueball.SuccessfulConverges}";
			label28 = $"{CachedLocalizer.Instance.MechJebAscentLabel28}{Core.Glueball.LastLmStatus}";
			n = $"n: {Core.Glueball.LastLmIterations}({Core.Glueball.MaxLmIterations})";
			label29 = CachedLocalizer.Instance.MechJebAscentLabel29 + " " + GuiUtils.TimeToDHMS(Core.Glueball.Staleness);
			znorm = $"infeasibility: {Core.Glueball.LastInfeasibility:G5}";
			if (Core.Glueball.LastFailureMessage != null)
			{
				label30 = CachedLocalizer.Instance.MechJebAscentLabel30 + Core.Glueball.LastFailureMessage;
			}
			if (_launchingToPlane)
			{
				launchTimer = CachedLocalizer.Instance.MechJebAscentMsg2;
			}
			else if (_launchingToMatchLan)
			{
				launchTimer = CachedLocalizer.Instance.MechJebAscentLaunchingToTargetLAN;
			}
			else if (_launchingToLan)
			{
				launchTimer = CachedLocalizer.Instance.MechJebAscentLaunchingToManualLAN;
			}
			else
			{
				launchTimer = string.Empty;
			}
			if (_autopilot.TMinus > 3.0 * base.VesselState.DeltaT)
			{
				launchTimer = launchTimer + ": T-" + GuiUtils.TimeToDHMS(_autopilot.TMinus, 1);
			}
			autopilotStatus = CachedLocalizer.Instance.MechJebAscentLabel35 + _autopilot.Status;
		}
	}

	private void RefreshRateGUI()
	{
		int num = _refreshRate;
		if (GuiUtils.ShowAdvancedWindowSettings)
		{
			GuiUtils.SimpleTextBox("Update Interval", _refreshRate, "Hz");
		}
		if (num != (int)_refreshRate)
		{
			_refreshRate = Math.Max(_refreshRate, 1);
			_refreshInterval = TimeSpan.FromSeconds(1.0 / (double)(int)_refreshRate);
		}
	}

	protected override void WindowGUI(int windowID)
	{
		SetupButtonStyles();
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		UpdateStrings();
		VisibleSectionsGUIElements();
		ShowTargetingGUIElements();
		_ascentSettings.LimitQaEnabled = _ascentSettings.AscentType == AscentType.PSG;
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		_settingsMenu.Enabled = GUILayout.Toggle(_settingsMenu.Enabled, "Ascent Settings", Array.Empty<GUILayoutOption>());
		if (_ascentSettings.AscentType == AscentType.PSG)
		{
			Core.StageStats.RequestUpdate();
			Core.StageStats.LiveSLT = true;
			_psgSettingsMenu.Enabled = GUILayout.Toggle(_psgSettingsMenu.Enabled, "PSG Settings", Array.Empty<GUILayoutOption>());
		}
		GUILayout.EndHorizontal();
		GUILayout.EndVertical();
		ShowStatusGUIElements();
		ShowAutoWarpGUIElements();
		if (_autopilot.Enabled)
		{
			GUILayout.Label(autopilotStatus, Array.Empty<GUILayoutOption>());
		}
		if (Core.DeactivateControl)
		{
			GUILayout.Label(CachedLocalizer.Instance.MechJebAscentLabel36, GuiUtils.RedLabel, Array.Empty<GUILayoutOption>());
		}
		if (!base.Vessel.patchedConicsUnlocked() && _ascentSettings.AscentType != AscentType.PSG)
		{
			GUILayout.Label(CachedLocalizer.Instance.MechJebAscentLabel37, Array.Empty<GUILayoutOption>());
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		_ascentSettings.AscentType = (AscentType)GuiUtils.ComboBox.Box((int)_ascentSettings.AscentType, _ascentPathList, this);
		GUILayout.EndHorizontal();
		if (_ascentSettings.AscentType == AscentType.CLASSIC)
		{
			_classicPathMenu.Enabled = GUILayout.Toggle(_classicPathMenu.Enabled, CachedLocalizer.Instance.MechJebAscentCheckbox10, Array.Empty<GUILayoutOption>());
		}
		RefreshRateGUI();
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	private string PhaseString(Solution solution, double t, int psgPhase)
	{
		//IL_006a: Unknown result type (might be due to invalid IL or missing references)
		int num = solution.MJPhase(psgPhase);
		int num2 = solution.KSPStage(psgPhase);
		if (solution.CoastPhase(psgPhase))
		{
			return $"coast: {num2} {solution.Tgo(t, psgPhase):F1}s";
		}
		double num3 = 0.0;
		if (num < Core.StageStats.VacStats.Count)
		{
			num3 = Core.StageStats.VacStats[num].DeltaV;
		}
		double num4 = num3 - solution.DV(t, psgPhase);
		if (Math.Abs(num4) < 2.5)
		{
			num4 = 0.0;
		}
		return $"burn: {num2} {solution.Tgo(t, psgPhase):F1}s {solution.DV(t, psgPhase):F1}m/s ({num4:F1}m/s)";
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(275f),
			GUILayout.Height(30f)
		};
	}

	public override string GetName()
	{
		return CachedLocalizer.Instance.MechJebAscentTitle;
	}

	public override string IconName()
	{
		return "Ascent Guidance";
	}
}
