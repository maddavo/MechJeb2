using System;
using KSP.Localization;
using MechJebLibBindings;
using UnityEngine;

namespace MuMech;

public class MechJebModuleThrustWindow : DisplayModule
{
	[Persistent(pass = 1)]
	public bool autostageSavedState;

	public MechJebModuleThrustWindow(MechJebCore core)
		: base(core)
	{
	}

	public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		base.OnLoad(local, type, global);
		if (autostageSavedState && !Core.Staging.Users.Contains(this))
		{
			Core.Staging.Users.Add(this);
		}
	}

	[GeneralInfoItem("#MechJeb_AutostageOnce", InfoItem.Category.Misc)]
	public void AutostageOnceItem()
	{
		if (Core.Staging.Enabled)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Utilities_label1", new string[1] { Core.Staging.AutostagingOnce ? Localizer.Format("#MechJeb_Utilities_label1_1") : " " }), Array.Empty<GUILayoutOption>());
		}
		if (!Core.Staging.Enabled && GUILayout.Button(Localizer.Format("#MechJeb_Utilities_button1"), Array.Empty<GUILayoutOption>()))
		{
			Core.Staging.AutostageOnce(this);
		}
	}

	[GeneralInfoItem("#MechJeb_Autostage", InfoItem.Category.Misc)]
	public void Autostage()
	{
		bool flag = Core.Staging.Users.Contains(this);
		bool flag2 = GUILayout.Toggle(flag, Localizer.Format("#MechJeb_Utilities_checkbox1"), Array.Empty<GUILayoutOption>());
		if (flag2 && !flag)
		{
			Core.Staging.Users.Add(this);
		}
		if (!flag2 && flag)
		{
			Core.Staging.Users.Remove(this);
		}
		autostageSavedState = flag2;
	}

	protected override void WindowGUI(int windowID)
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		Core.Thrust.LimitToMaxDynamicPressureInfoItem();
		Core.Thrust.LimitToPreventOverheatsInfoItem();
		Core.Thrust.LimitAccelerationInfoItem();
		Core.Thrust.LimitThrottleInfoItem();
		Core.Thrust.LimiterMinThrottleInfoItem();
		Core.Thrust.LimitElectricInfoItem();
		Core.Thrust.LimitToPreventFlameoutInfoItem();
		if (ReflectionUtils.IsLoadedRealFuels)
		{
			Core.Thrust.LimitToPreventUnstableIgnitionInfoItem();
			Core.Thrust.AutoRCsUllageInfoItem();
		}
		Core.Thrust.SmoothThrottle = GUILayout.Toggle(Core.Thrust.SmoothThrottle, Localizer.Format("#MechJeb_Utilities_checkbox2"), Array.Empty<GUILayoutOption>());
		Core.Thrust.ManageIntakes = GUILayout.Toggle(Core.Thrust.ManageIntakes, Localizer.Format("#MechJeb_Utilities_checkbox3"), Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal((GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		try
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Utilities_label2"), Array.Empty<GUILayoutOption>());
			Core.Thrust.FlameoutSafetyPct.Text = GUILayout.TextField(Core.Thrust.FlameoutSafetyPct.Text, 5, Array.Empty<GUILayoutOption>());
			GUILayout.Label("%", Array.Empty<GUILayoutOption>());
		}
		finally
		{
			GUILayout.EndHorizontal();
		}
		Core.Thrust.DifferentialThrottleMenu();
		if (Core.Thrust.DifferentialThrottle && base.Vessel.LiftedOff())
		{
			switch (Core.Thrust.DifferentialThrottleSuccess)
			{
			case MechJebModuleThrustController.DifferentialThrottleStatus.MORE_ENGINES_REQUIRED:
				GUILayout.Label(Localizer.Format("#MechJeb_Utilities_label3"), GuiUtils.YellowLabel, Array.Empty<GUILayoutOption>());
				break;
			case MechJebModuleThrustController.DifferentialThrottleStatus.ALL_ENGINES_OFF:
				GUILayout.Label(Localizer.Format("#MechJeb_Utilities_label4"), GuiUtils.YellowLabel, Array.Empty<GUILayoutOption>());
				break;
			case MechJebModuleThrustController.DifferentialThrottleStatus.SOLVER_FAILED:
				GUILayout.Label(Localizer.Format("#MechJeb_Utilities_label5"), GuiUtils.YellowLabel, Array.Empty<GUILayoutOption>());
				break;
			}
		}
		Core.Solarpanel.SolarPanelDeployButton();
		Core.AntennaControl.AntennaDeployButton();
		Autostage();
		if (!Core.Staging.Enabled && GUILayout.Button(Localizer.Format("#MechJeb_Utilities_button1"), Array.Empty<GUILayoutOption>()))
		{
			Core.Staging.AutostageOnce(this);
		}
		if (Core.Staging.Enabled)
		{
			Core.Staging.AutostageSettingsInfoItem();
		}
		if (Core.Staging.Enabled)
		{
			GUILayout.Label(Core.Staging.AutostageStatus(), Array.Empty<GUILayoutOption>());
		}
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(250f),
			GUILayout.Height(30f)
		};
	}

	public override bool IsActive()
	{
		return Core.Thrust.Limiter != MechJebModuleThrustController.LimitMode.NONE;
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_Utilities_title");
	}

	public override string IconName()
	{
		return "Utilities";
	}
}
