using System;
using System.Collections.Generic;
using MechJebLib.FuelFlowSimulation;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModuleAscentPSGSettingsMenu : DisplayModule
{
	private static GUIStyle _btNormal;

	private static GUIStyle _btActive;

	private MechJebModuleAscentSettings _ascentSettings => Core.AscentSettings;

	public MechJebModuleAscentPSGSettingsMenu(MechJebCore core)
		: base(core)
	{
		Hidden = true;
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(300f),
			GUILayout.Height(100f)
		};
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
			_btNormal.padding = new RectOffset(0, 0, 0, 0);
			_btActive = new GUIStyle(_btNormal);
			_btActive.active = _btActive.onActive;
			_btActive.normal = _btActive.onNormal;
			_btActive.onFocused = _btActive.focused;
			_btActive.hover = _btActive.onHover;
		}
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0119: Unknown result type (might be due to invalid IL or missing references)
		//IL_0124: Unknown result type (might be due to invalid IL or missing references)
		//IL_0149: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		//IL_0170: Unknown result type (might be due to invalid IL or missing references)
		SetupButtonStyles();
		int num = -1;
		List<FuelStats> vacStats = Core.StageStats.VacStats;
		if (vacStats.Count > 0 && (int)_ascentSettings.LastStage <= vacStats[vacStats.Count - 1].KSPStage)
		{
			GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
			_ascentSettings.LastStage.Val = Statics.Clamp(_ascentSettings.LastStage.Val, 0, vacStats[vacStats.Count - 1].KSPStage);
			_ascentSettings.OptimizeStageFlag = false;
			foreach (FuelStats vacStat in Core.StageStats.VacStats)
			{
				if (vacStat.KSPStage >= (int)_ascentSettings.LastStage && !(vacStat.DeltaV < _ascentSettings.MinDeltaV.Val))
				{
					if (num < 0)
					{
						num = vacStat.KSPStage;
					}
					GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
					GUILayout.Label($"{vacStat.KSPStage,3} {vacStat.DeltaV:##,###0} m/s", Array.Empty<GUILayoutOption>());
					if (_ascentSettings.UnguidedStages.Contains(vacStat.KSPStage))
					{
						GUILayout.Label(" (unguided)", Array.Empty<GUILayoutOption>());
					}
					if (_ascentSettings.FixedStages.Contains(vacStat.KSPStage))
					{
						GUILayout.Label(" (fixed)", Array.Empty<GUILayoutOption>());
					}
					else
					{
						_ascentSettings.OptimizeStageFlag = true;
					}
					GUILayout.EndHorizontal();
				}
			}
			GUILayout.EndVertical();
		}
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox("Min ΔV: ", _ascentSettings.MinDeltaV, "m/s", 30f);
		GuiUtils.SimpleTextBox("Last Stage: ", _ascentSettings.LastStage);
		GuiUtils.ToggledTextBox(ref _ascentSettings.FixedStagesFlag, "Fixed Burn Stages: ", _ascentSettings.FixedStagesInternal);
		GuiUtils.ToggledTextBox(ref _ascentSettings.UnguidedStagesFlag, "Unguided Stages: ", _ascentSettings.UnguidedStagesInternal);
		GUILayout.EndVertical();
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		GuiUtils.ToggledTextBox(ref _ascentSettings.CoastStageFlag, "Coast Stage: ", _ascentSettings.CoastStageInternal);
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (GUILayout.Toggle(_ascentSettings.CoastLocation == -1, "Coast Before", Array.Empty<GUILayoutOption>()))
		{
			_ascentSettings.CoastLocation = -1;
		}
		if (GUILayout.Toggle(_ascentSettings.CoastLocation == 0, "Coast During", Array.Empty<GUILayoutOption>()))
		{
			_ascentSettings.CoastLocation = 0;
		}
		if (GUILayout.Toggle(_ascentSettings.CoastLocation == 1, "Coast After", Array.Empty<GUILayoutOption>()))
		{
			_ascentSettings.CoastLocation = 1;
		}
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox("Min Coast:", _ascentSettings.MinCoast, "s", 40f);
		GuiUtils.SimpleTextBox("Max Coast:", _ascentSettings.MaxCoast, "s", 40f);
		GUILayout.EndHorizontal();
		GuiUtils.SimpleTextBox("Ullage lead time: ", Core.Guidance.UllageLeadTime, "s", 60f);
		GUILayout.EndVertical();
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GuiUtils.ToggledTextBox(ref _ascentSettings.SpinupStageFlag, "Spinup stage: ", _ascentSettings.SpinupStageInternal, null, null, 30f);
		GuiUtils.SimpleTextBox("ω: ", _ascentSettings.SpinupAngularVelocity, "rpm", 30f);
		GUILayout.EndHorizontal();
		GuiUtils.SimpleTextBox("Spinup lead time: ", _ascentSettings.SpinupLeadTime, "s", 60f);
		GUILayout.EndVertical();
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox("Cd:", _ascentSettings.Cd, "", 40f);
		GuiUtils.SimpleTextBox("Aref:", _ascentSettings.Aref, "m²", 40f);
		GUILayout.EndHorizontal();
		GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel13, _ascentSettings.PitchStartHeight, "m", 40f);
		GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel14, _ascentSettings.PitchRate, "°/s", 40f);
		GUILayout.EndVertical();
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel17, _ascentSettings.LimitQa, "Pa-rad");
		if ((double)_ascentSettings.LimitQa < 1000.0 || (double)_ascentSettings.LimitQa > 4000.0)
		{
			if ((double)_ascentSettings.LimitQa < 0.0 || (double)_ascentSettings.LimitQa > 10000.0)
			{
				GUILayout.Label("Qα limit has been clamped to between 0 and 10000 Pa-rad", GuiUtils.RedLabel, Array.Empty<GUILayoutOption>());
			}
			else
			{
				GUILayout.Label(CachedLocalizer.Instance.MechJebAscentLabel20, GuiUtils.YellowLabel, Array.Empty<GUILayoutOption>());
			}
		}
		Core.Guidance.ShouldDrawTrajectory = GUILayout.Toggle(Core.Guidance.ShouldDrawTrajectory, "Draw Trajectory on Map", Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	public override string GetName()
	{
		return "PSG Settings";
	}

	public override string IconName()
	{
		return "PSGSettings";
	}
}
