using System;
using UnityEngine;

namespace MuMech;

public class MechJebModuleAscentSettingsMenu : DisplayModule
{
	private readonly string _climbString = CachedLocalizer.Instance.MechJebAscentLabel22 + ": ";

	private readonly string _turnString = CachedLocalizer.Instance.MechJebAscentLabel23 + ": ";

	private MechJebModuleAscentSettings _ascentSettings => Core.AscentSettings;

	private MechJebModuleAscentBaseAutopilot _autopilot => Core.Ascent;

	public MechJebModuleAscentSettingsMenu(MechJebCore core)
		: base(core)
	{
		Hidden = true;
	}

	private void ShowAscentSettingsGUIElements()
	{
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		Core.Thrust.LimitToPreventOverheatsInfoItem();
		Core.Thrust.LimitToMaxDynamicPressureInfoItem();
		Core.Thrust.LimitAccelerationInfoItem();
		if (_ascentSettings.AscentType != AscentType.PSG)
		{
			Core.Thrust.LimitThrottleInfoItem();
		}
		Core.Thrust.LimiterMinThrottleInfoItem();
		if (_ascentSettings.AscentType != AscentType.PSG)
		{
			Core.Thrust.LimitElectricInfoItem();
		}
		if (_ascentSettings.AscentType == AscentType.PSG)
		{
			Core.Thrust.LimitThrottle = false;
			Core.Thrust.ElectricThrottle = false;
		}
		_ascentSettings.ForceRoll = GUILayout.Toggle(_ascentSettings.ForceRoll, CachedLocalizer.Instance.MechJebAscentCheckbox2, Array.Empty<GUILayoutOption>());
		if (_ascentSettings.ForceRoll)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Space(15f);
			GuiUtils.SimpleTextBox(_climbString, _ascentSettings.VerticalRoll, "º", 30f);
			GuiUtils.SimpleTextBox(_turnString, _ascentSettings.TurnRoll, "º", 30f);
			GuiUtils.SimpleTextBox("Alt: ", _ascentSettings.RollAltitude, "m", 30f);
			GUILayout.EndHorizontal();
		}
		if (_ascentSettings.AscentType != AscentType.PSG)
		{
			GUIStyle toggleStyle = (_ascentSettings.LimitingAoA ? GuiUtils.GreenToggle : null);
			string rightLabel = $"º ({_autopilot.CurrentMaxAoA:F1}°)";
			GuiUtils.ToggledTextBox(ref _ascentSettings.LimitAoA, CachedLocalizer.Instance.MechJebAscentCheckbox3, _ascentSettings.MaxAoA, rightLabel, toggleStyle, 30f);
			if (_ascentSettings.LimitAoA)
			{
				GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
				GUILayout.Space(25f);
				GUIStyle leftLabelStyle = ((_ascentSettings.LimitingAoA && base.VesselState.DynamicPressure < (double)_ascentSettings.AOALimitFadeoutPressure) ? GuiUtils.GreenLabel : GuiUtils.Skin.label);
				GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel24, _ascentSettings.AOALimitFadeoutPressure, "Pa", 50f, leftLabelStyle);
				GUILayout.EndHorizontal();
			}
			_ascentSettings.LimitQaEnabled = false;
		}
		if (_ascentSettings.AscentType == AscentType.CLASSIC)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			_ascentSettings.CorrectiveSteering = GUILayout.Toggle(_ascentSettings.CorrectiveSteering, CachedLocalizer.Instance.MechJebAscentCheckbox4, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.ExpandWidth(b: false) });
			if (_ascentSettings.CorrectiveSteering)
			{
				GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel25, _ascentSettings.CorrectiveSteeringGain, null, 40f, null, horizontalFraming: false);
			}
			GUILayout.EndHorizontal();
		}
		_ascentSettings.Autostage = GUILayout.Toggle(_ascentSettings.Autostage, CachedLocalizer.Instance.MechJebAscentCheckbox5, Array.Empty<GUILayoutOption>());
		if (_ascentSettings.Autostage)
		{
			Core.Staging.AutostageSettingsInfoItem();
		}
		_ascentSettings.AutoDeploySolarPanels = GUILayout.Toggle(_ascentSettings.AutoDeploySolarPanels, CachedLocalizer.Instance.MechJebAscentCheckbox6, Array.Empty<GUILayoutOption>());
		_ascentSettings.AutoDeployAntennas = GUILayout.Toggle(_ascentSettings.AutoDeployAntennas, CachedLocalizer.Instance.MechJebAscentCheckbox7, Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		Core.Node.Autowarp = GUILayout.Toggle(Core.Node.Autowarp, CachedLocalizer.Instance.MechJebAscentCheckbox8, Array.Empty<GUILayoutOption>());
		if (_ascentSettings.AscentType != AscentType.PSG)
		{
			_ascentSettings.SkipCircularization = GUILayout.Toggle(_ascentSettings.SkipCircularization, CachedLocalizer.Instance.MechJebAscentCheckbox9, Array.Empty<GUILayoutOption>());
		}
		else
		{
			_ascentSettings.SkipCircularization = true;
		}
		GUILayout.EndHorizontal();
		if (_ascentSettings.AscentType == AscentType.PSG)
		{
			Core.Settings.RssMode = GUILayout.Toggle(Core.Settings.RssMode, "Module disabling does not kill throttle", Array.Empty<GUILayoutOption>());
		}
		GUILayout.EndVertical();
	}

	protected override void WindowGUI(int windowID)
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		ShowAscentSettingsGUIElements();
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
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
		return "Ascent Settings";
	}

	public override string IconName()
	{
		return "Ascent Settings";
	}
}
