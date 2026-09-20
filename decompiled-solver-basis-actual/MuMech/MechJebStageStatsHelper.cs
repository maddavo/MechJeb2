using System;
using System.Collections.Generic;
using KSP.Localization;
using MechJebLib.FuelFlowSimulation;
using MechJebLib.Utils;
using MechJebLibBindings;
using UnityEngine;

namespace MuMech;

public class MechJebStageStatsHelper
{
	private enum StageData
	{
		KSPStage,
		InitialMass,
		FinalMass,
		StagedMass,
		BurnedMass,
		Thrust,
		VacInitialTWR,
		VacMaxTWR,
		AtmoInitialTWR,
		AtmoMaxTWR,
		Isp,
		AtmoDeltaV,
		VacDeltaV,
		Time,
		AtmoCumulativeDeltaV,
		VacCumulativeDeltaV,
		ControllableMass,
		RcsUllageTime
	}

	private static readonly bool _isLoadedRP0;

	private bool showStagedMass;

	private bool showBurnedMass;

	private bool showInitialMass;

	private bool showFinalMass;

	private bool showThrust;

	private bool showVacInitialTWR;

	private bool showAtmoInitialTWR;

	private bool showAtmoMaxTWR;

	private bool showVacMaxTWR;

	private bool showAtmoDeltaV;

	private bool showVacDeltaV;

	private bool showTime;

	private bool showISP;

	private bool showEmpty;

	private bool showRcs;

	private bool timeSeconds;

	private bool liveSLT;

	private bool showAtmoCumulativeDeltaV;

	private bool showVacCumulativeDeltaV;

	private bool showControllableMass;

	private bool showRcsUllageTime;

	private int TWRBody;

	private float altSLTScale;

	private float machScale;

	private readonly MechJebModuleInfoItems infoItems;

	private readonly MechJebCore core;

	private readonly MechJebModuleStageStats stats;

	private static readonly List<StageData> AllStages;

	private static readonly string[] StageDisplayStates;

	private readonly string[] bodies;

	private readonly List<int> stages = new List<int>(8);

	private readonly Dictionary<StageData, bool> stageVisibility = new Dictionary<StageData, bool>(12);

	private readonly Dictionary<StageData, List<string>> stageDisplayInfo = new Dictionary<StageData, List<string>>(12);

	private readonly Dictionary<StageData, string> stageHeaderData = new Dictionary<StageData, string>(12);

	private static GUIStyle _columnStyle;

	private int StageDisplayState
	{
		get
		{
			return infoItems.StageDisplayState;
		}
		set
		{
			infoItems.StageDisplayState = value;
		}
	}

	public static GUIStyle ColumnStyle
	{
		get
		{
			//IL_000e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0013: Unknown result type (might be due to invalid IL or missing references)
			//IL_001a: Unknown result type (might be due to invalid IL or missing references)
			//IL_0021: Unknown result type (might be due to invalid IL or missing references)
			//IL_0026: Unknown result type (might be due to invalid IL or missing references)
			//IL_0030: Expected O, but got Unknown
			//IL_0030: Unknown result type (might be due to invalid IL or missing references)
			//IL_0036: Expected O, but got Unknown
			object obj = _columnStyle;
			if (obj == null)
			{
				GUIStyle val = new GUIStyle(GuiUtils.YellowOnHover)
				{
					alignment = (TextAnchor)5,
					wordWrap = false,
					padding = new RectOffset(2, 2, 0, 0)
				};
				_columnStyle = val;
				obj = (object)val;
			}
			return (GUIStyle)obj;
		}
	}

	static MechJebStageStatsHelper()
	{
		AllStages = new List<StageData>
		{
			StageData.KSPStage,
			StageData.ControllableMass,
			StageData.InitialMass,
			StageData.FinalMass,
			StageData.StagedMass,
			StageData.BurnedMass,
			StageData.Thrust,
			StageData.VacInitialTWR,
			StageData.VacMaxTWR,
			StageData.AtmoInitialTWR,
			StageData.AtmoMaxTWR,
			StageData.Isp,
			StageData.RcsUllageTime,
			StageData.AtmoCumulativeDeltaV,
			StageData.VacCumulativeDeltaV,
			StageData.AtmoDeltaV,
			StageData.VacDeltaV,
			StageData.Time
		};
		StageDisplayStates = new string[4]
		{
			Localizer.Format("#MechJeb_InfoItems_button1"),
			Localizer.Format("#MechJeb_InfoItems_button2"),
			Localizer.Format("#MechJeb_InfoItems_button3"),
			Localizer.Format("#MechJeb_InfoItems_button4")
		};
		_isLoadedRP0 = ReflectionUtils.IsAssemblyLoaded("RP0");
	}

	public MechJebStageStatsHelper(MechJebModuleInfoItems items)
	{
		infoItems = items;
		core = items.Core;
		stats = core.GetComputerModule<MechJebModuleStageStats>();
		showStagedMass = items.showStagedMass;
		showBurnedMass = items.showBurnedMass;
		showInitialMass = items.showInitialMass;
		showFinalMass = items.showFinalMass;
		showVacInitialTWR = items.showVacInitialTWR;
		showAtmoInitialTWR = items.showAtmoInitialTWR;
		showAtmoMaxTWR = items.showAtmoMaxTWR;
		showVacMaxTWR = items.showVacMaxTWR;
		showAtmoDeltaV = items.showAtmoDeltaV;
		showVacDeltaV = items.showVacDeltaV;
		showAtmoCumulativeDeltaV = items.showAtmoCumulativeDeltaV;
		showVacCumulativeDeltaV = items.showVacCumulativeDeltaV;
		showControllableMass = items.showControllableMass;
		showRcsUllageTime = items.showRcsUllageTime;
		showTime = items.showTime;
		showISP = items.showISP;
		showThrust = items.showThrust;
		showRcs = items.showRcs;
		showEmpty = items.showEmpty;
		timeSeconds = items.timeSeconds;
		liveSLT = items.liveSLT;
		altSLTScale = items.altSLTScale;
		machScale = items.machScale;
		TWRBody = items.TWRBody;
		bodies = ((HighLogic.LoadedSceneIsFlight || HighLogic.LoadedSceneIsEditor) ? FlightGlobals.Bodies.ConvertAll((CelestialBody b) => b.GetName()).ToArray() : new string[1] { "None" });
		InitializeStageInfo();
		SetVisibility(StageDisplayState);
	}

	private void InitializeStageInfo()
	{
		stageVisibility.Clear();
		stageDisplayInfo.Clear();
		foreach (StageData allStage in AllStages)
		{
			stageVisibility.Add(allStage, value: false);
			stageDisplayInfo.Add(allStage, new List<string>(16));
		}
		InitalizeStageHeaderData();
	}

	private void InitalizeStageHeaderData()
	{
		stageHeaderData.Clear();
		stageHeaderData.Add(StageData.KSPStage, "Stage   ");
		stageHeaderData.Add(StageData.ControllableMass, "Avionics   ");
		stageHeaderData.Add(StageData.InitialMass, CachedLocalizer.Instance.MechJebInfoItemsStatsColumn1 + "   ");
		stageHeaderData.Add(StageData.FinalMass, CachedLocalizer.Instance.MechJebInfoItemsStatsColumn2 + "   ");
		stageHeaderData.Add(StageData.StagedMass, CachedLocalizer.Instance.MechJebInfoItemsStatsColumn3 + "   ");
		stageHeaderData.Add(StageData.BurnedMass, CachedLocalizer.Instance.MechJebInfoItemsStatsColumn4 + "   ");
		stageHeaderData.Add(StageData.Thrust, CachedLocalizer.Instance.MechJebInfoItemsStatsColumn13 + "   ");
		stageHeaderData.Add(StageData.VacInitialTWR, CachedLocalizer.Instance.MechJebInfoItemsStatsColumn5 + "   ");
		stageHeaderData.Add(StageData.VacMaxTWR, CachedLocalizer.Instance.MechJebInfoItemsStatsColumn6 + "   ");
		stageHeaderData.Add(StageData.AtmoInitialTWR, CachedLocalizer.Instance.MechJebInfoItemsStatsColumn7 + "   ");
		stageHeaderData.Add(StageData.AtmoMaxTWR, CachedLocalizer.Instance.MechJebInfoItemsStatsColumn8 + "   ");
		stageHeaderData.Add(StageData.Isp, CachedLocalizer.Instance.MechJebInfoItemsStatsColumn9 + "   ");
		stageHeaderData.Add(StageData.RcsUllageTime, "RCS Ullage   ");
		stageHeaderData.Add(StageData.AtmoDeltaV, (showRcs ? "RCS ΔVmin" : CachedLocalizer.Instance.MechJebInfoItemsStatsColumn10) + "   ");
		stageHeaderData.Add(StageData.VacDeltaV, (showRcs ? "RCS ΔVmax" : CachedLocalizer.Instance.MechJebInfoItemsStatsColumn11) + "   ");
		stageHeaderData.Add(StageData.AtmoCumulativeDeltaV, "Σ Atmo ΔV   ");
		stageHeaderData.Add(StageData.VacCumulativeDeltaV, "Σ Vac ΔV   ");
		stageHeaderData.Add(StageData.Time, CachedLocalizer.Instance.MechJebInfoItemsStatsColumn12 + "   ");
	}

	private void GatherStages(List<int> stages)
	{
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		stages.Clear();
		for (int i = 0; i < stats.AtmoStats.Count; i++)
		{
			if (infoItems.showEmpty)
			{
				stages.Add(i);
			}
			else if (infoItems.showRcs && stats.AtmoStats[i].MinRcsDeltaV > 0.0)
			{
				stages.Add(i);
			}
			else if (!infoItems.showRcs && stats.AtmoStats[i].DeltaV > 0.0)
			{
				stages.Add(i);
			}
		}
	}

	private double _isp(int index)
	{
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		if (!showRcs)
		{
			return stats.VacStats[index].Isp;
		}
		return stats.VacStats[index].RcsISP;
	}

	private double _thrust(int index)
	{
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		if (!showRcs)
		{
			return stats.VacStats[index].Thrust;
		}
		return stats.VacStats[index].RcsThrust;
	}

	private double _burnedMass(int index)
	{
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		if (!showRcs)
		{
			FuelStats val = stats.VacStats[index];
			return ((FuelStats)(ref val)).ResourceMass;
		}
		return stats.VacStats[index].RcsMass;
	}

	private double _deltaTime(int index)
	{
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		if (!showRcs)
		{
			return stats.VacStats[index].DeltaTime;
		}
		return stats.VacStats[index].RcsDeltaTime;
	}

	private double _atmoDv(int index)
	{
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		if (!showRcs)
		{
			return stats.AtmoStats[index].DeltaV;
		}
		return stats.VacStats[index].MinRcsDeltaV;
	}

	private double _vacDv(int index)
	{
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		if (!showRcs)
		{
			return stats.VacStats[index].DeltaV;
		}
		return stats.VacStats[index].MaxRcsDeltaV;
	}

	private double _vacStartTWR(int index, double geeASL)
	{
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		FuelStats val;
		if (!showRcs)
		{
			val = stats.VacStats[index];
			return ((FuelStats)(ref val)).StartTWR(geeASL);
		}
		val = stats.VacStats[index];
		return ((FuelStats)(ref val)).RcsStartTWR(geeASL);
	}

	private double _vacEndTWR(int index, double geeASL)
	{
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		FuelStats val;
		if (!showRcs)
		{
			val = stats.VacStats[index];
			return ((FuelStats)(ref val)).MaxTWR(geeASL);
		}
		val = stats.VacStats[index];
		return ((FuelStats)(ref val)).RcsMaxTWR(geeASL);
	}

	private double _atmoStartTWR(int index, double geeASL)
	{
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		FuelStats val;
		if (!showRcs)
		{
			val = stats.AtmoStats[index];
			return ((FuelStats)(ref val)).StartTWR(geeASL);
		}
		val = stats.AtmoStats[index];
		return ((FuelStats)(ref val)).RcsStartTWR(geeASL);
	}

	private double _atmoEndTWR(int index, double geeASL)
	{
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		FuelStats val;
		if (!showRcs)
		{
			val = stats.AtmoStats[index];
			return ((FuelStats)(ref val)).MaxTWR(geeASL);
		}
		val = stats.AtmoStats[index];
		return ((FuelStats)(ref val)).RcsMaxTWR(geeASL);
	}

	private double CalculateCumulativeVacDeltaV(List<int> stages, int currentStageIndex)
	{
		double num = 0.0;
		for (int i = currentStageIndex; i < stages.Count; i++)
		{
			num += _vacDv(stages[i]);
		}
		return num;
	}

	private double CalculateCumulativeAtmoDeltaV(List<int> stages, int currentStageIndex)
	{
		double num = 0.0;
		for (int i = currentStageIndex; i < stages.Count; i++)
		{
			num += _atmoDv(stages[i]);
		}
		return num;
	}

	private void UpdateStageDisplayInfo(List<int> stages, double geeASL)
	{
		//IL_0075: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0143: Unknown result type (might be due to invalid IL or missing references)
		//IL_0187: Unknown result type (might be due to invalid IL or missing references)
		//IL_0378: Unknown result type (might be due to invalid IL or missing references)
		//IL_034b: Unknown result type (might be due to invalid IL or missing references)
		foreach (KeyValuePair<StageData, List<string>> item in stageDisplayInfo)
		{
			item.Value.Clear();
		}
		foreach (int stage in stages)
		{
			int currentStageIndex = stages.IndexOf(stage);
			stageDisplayInfo[StageData.KSPStage].Add($"{stats.AtmoStats[stage].KSPStage}   ");
			if (stageVisibility[StageData.ControllableMass])
			{
				stageDisplayInfo[StageData.ControllableMass].Add($"{stats.AtmoStats[stage].ControllableMass:F3} t   ");
			}
			if (stageVisibility[StageData.InitialMass])
			{
				stageDisplayInfo[StageData.InitialMass].Add($"{stats.AtmoStats[stage].StartMass:F3} t   ");
			}
			if (stageVisibility[StageData.FinalMass])
			{
				stageDisplayInfo[StageData.FinalMass].Add($"{stats.AtmoStats[stage].EndMass:F3} t   ");
			}
			if (stageVisibility[StageData.StagedMass])
			{
				stageDisplayInfo[StageData.StagedMass].Add($"{stats.AtmoStats[stage].StagedMass:F3} t   ");
			}
			if (stageVisibility[StageData.BurnedMass])
			{
				stageDisplayInfo[StageData.BurnedMass].Add($"{_burnedMass(stage):F3} t   ");
			}
			if (stageVisibility[StageData.Thrust])
			{
				stageDisplayInfo[StageData.Thrust].Add($"{_thrust(stage):F3} kN   ");
			}
			if (stageVisibility[StageData.VacInitialTWR])
			{
				stageDisplayInfo[StageData.VacInitialTWR].Add($"{_vacStartTWR(stage, geeASL):F2}   ");
			}
			if (stageVisibility[StageData.VacMaxTWR])
			{
				stageDisplayInfo[StageData.VacMaxTWR].Add($"{_vacEndTWR(stage, geeASL):F2}   ");
			}
			if (stageVisibility[StageData.AtmoInitialTWR])
			{
				stageDisplayInfo[StageData.AtmoInitialTWR].Add($"{_atmoStartTWR(stage, geeASL):F2}   ");
			}
			if (stageVisibility[StageData.AtmoMaxTWR])
			{
				stageDisplayInfo[StageData.AtmoMaxTWR].Add($"{_atmoEndTWR(stage, geeASL):F2}   ");
			}
			if (stageVisibility[StageData.Isp])
			{
				stageDisplayInfo[StageData.Isp].Add($"{_isp(stage):F2}   ");
			}
			if (stageVisibility[StageData.RcsUllageTime])
			{
				stageDisplayInfo[StageData.RcsUllageTime].Add(timeSeconds ? $"{stats.AtmoStats[stage].RcsUllageTime:F2}s   " : (GuiUtils.TimeToDHMS(stats.AtmoStats[stage].RcsUllageTime, 1) + "   "));
			}
			if (stageVisibility[StageData.AtmoDeltaV])
			{
				stageDisplayInfo[StageData.AtmoDeltaV].Add($"{_atmoDv(stage):F0} m/s   ");
			}
			if (stageVisibility[StageData.VacDeltaV])
			{
				stageDisplayInfo[StageData.VacDeltaV].Add($"{_vacDv(stage):F0} m/s   ");
			}
			if (stageVisibility[StageData.AtmoCumulativeDeltaV])
			{
				stageDisplayInfo[StageData.AtmoCumulativeDeltaV].Add($"{CalculateCumulativeAtmoDeltaV(stages, currentStageIndex):F0} m/s   ");
			}
			if (stageVisibility[StageData.VacCumulativeDeltaV])
			{
				stageDisplayInfo[StageData.VacCumulativeDeltaV].Add($"{CalculateCumulativeVacDeltaV(stages, currentStageIndex):F0} m/s   ");
			}
			if (stageVisibility[StageData.Time])
			{
				stageDisplayInfo[StageData.Time].Add(timeSeconds ? $"{_deltaTime(stage):F2}s   " : (GuiUtils.TimeToDHMS(_deltaTime(stage), 1) + "   "));
			}
		}
	}

	private void SetAllStageVisibility(bool state)
	{
		foreach (StageData allStage in AllStages)
		{
			stageVisibility[allStage] = state;
		}
		stageVisibility[StageData.KSPStage] = true;
	}

	public void UpdateStageStats()
	{
		double geeASL = (HighLogic.LoadedSceneIsEditor ? FlightGlobals.Bodies[TWRBody].GeeASL : stats.MainBody.GeeASL);
		stats.RequestUpdate();
		GatherStages(stages);
		UpdateStageDisplayInfo(stages, geeASL);
	}

	public void AllStageStats()
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(CachedLocalizer.Instance.MechJebInfoItemsLabel1, Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(timeSeconds ? "s" : "dhms", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			timeSeconds = !timeSeconds;
			infoItems.timeSeconds = timeSeconds;
		}
		if (GUILayout.Button(showEmpty ? CachedLocalizer.Instance.MechJebInfoItemsShowEmpty : CachedLocalizer.Instance.MechJebInfoItemsHideEmpty, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			showEmpty = !showEmpty;
			infoItems.showEmpty = showEmpty;
		}
		if (GUILayout.Button(StageDisplayStates[StageDisplayState], (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			StageDisplayState = (StageDisplayState + 1) % StageDisplayStates.Length;
			SetVisibility(StageDisplayState);
		}
		if (GUILayout.Button(showRcs ? "RCS" : "Engine", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			showRcs = !showRcs;
			infoItems.showRcs = showRcs;
			InitalizeStageHeaderData();
			SetVisibility(StageDisplayState);
		}
		if (!HighLogic.LoadedSceneIsEditor)
		{
			if (GUILayout.Button(liveSLT ? CachedLocalizer.Instance.MechJebInfoItemsButton5 : CachedLocalizer.Instance.MechJebInfoItemsButton6, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
			{
				liveSLT = !liveSLT;
				infoItems.liveSLT = liveSLT;
			}
			stats.LiveSLT = liveSLT;
		}
		GUILayout.EndHorizontal();
		if (HighLogic.LoadedSceneIsEditor)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			TWRBody = GuiUtils.ComboBox.Box(TWRBody, bodies, this, expandWidth: false);
			infoItems.TWRBody = TWRBody;
			stats.EditorBody = FlightGlobals.Bodies[TWRBody];
			GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			altSLTScale = GUILayout.HorizontalSlider(altSLTScale, 0f, 1f, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			infoItems.altSLTScale = altSLTScale;
			stats.AltSLT = Math.Pow(altSLTScale, 2.0) * stats.EditorBody.atmosphereDepth;
			GUILayout.Label(Statics.ToSI(stats.AltSLT, 4, int.MaxValue) + "m", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(80f) });
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			machScale = GUILayout.HorizontalSlider(machScale, 0f, 1f, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			infoItems.machScale = machScale;
			stats.Mach = Math.Pow(machScale * 2f, 3.0);
			GUILayout.Label(stats.Mach.ToString("F1") + " M", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(80f) });
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
			GUILayout.EndHorizontal();
		}
		else
		{
			stats.EditorBody = stats.MainBody;
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		bool flag = true;
		foreach (StageData allStage in AllStages)
		{
			if (stageVisibility[allStage])
			{
				bool num = flag;
				Dictionary<StageData, bool> dictionary = stageVisibility;
				string header = stageHeaderData[allStage];
				List<string> data = stageDisplayInfo[allStage];
				bool flag3 = (dictionary[allStage] = !DrawStageStatsColumn(header, in data));
				flag = num && flag3;
			}
		}
		stageVisibility[StageData.KSPStage] = true;
		if (!flag)
		{
			StageDisplayState = 3;
			SaveStageVisibility();
		}
		GUILayout.EndHorizontal();
		GUILayout.EndVertical();
	}

	private bool DrawStageStatsColumn(string header, in List<string> data)
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		bool result = GUILayout.Button(header, ColumnStyle, Array.Empty<GUILayoutOption>());
		foreach (string datum in data)
		{
			GUILayout.Label(datum, ColumnStyle, Array.Empty<GUILayoutOption>());
		}
		GUILayout.EndVertical();
		return result;
	}

	private void LoadStageVisibility()
	{
		stageVisibility[StageData.StagedMass] = showStagedMass;
		stageVisibility[StageData.BurnedMass] = showBurnedMass;
		stageVisibility[StageData.InitialMass] = showInitialMass;
		stageVisibility[StageData.FinalMass] = showFinalMass;
		stageVisibility[StageData.Thrust] = showThrust;
		stageVisibility[StageData.VacInitialTWR] = showVacInitialTWR;
		stageVisibility[StageData.AtmoInitialTWR] = showAtmoInitialTWR;
		stageVisibility[StageData.AtmoMaxTWR] = showAtmoMaxTWR;
		stageVisibility[StageData.VacMaxTWR] = showVacMaxTWR;
		stageVisibility[StageData.AtmoDeltaV] = showAtmoDeltaV;
		stageVisibility[StageData.VacDeltaV] = showVacDeltaV;
		stageVisibility[StageData.Time] = showTime;
		stageVisibility[StageData.Isp] = showISP;
		stageVisibility[StageData.AtmoCumulativeDeltaV] = showAtmoCumulativeDeltaV;
		stageVisibility[StageData.VacCumulativeDeltaV] = showVacCumulativeDeltaV;
		stageVisibility[StageData.ControllableMass] = showControllableMass;
		stageVisibility[StageData.RcsUllageTime] = showRcsUllageTime;
	}

	private void SaveStageVisibility()
	{
		showStagedMass = (infoItems.showStagedMass = stageVisibility[StageData.StagedMass]);
		showBurnedMass = (infoItems.showBurnedMass = stageVisibility[StageData.BurnedMass]);
		showInitialMass = (infoItems.showInitialMass = stageVisibility[StageData.InitialMass]);
		showFinalMass = (infoItems.showFinalMass = stageVisibility[StageData.FinalMass]);
		showThrust = (infoItems.showThrust = stageVisibility[StageData.Thrust]);
		showVacInitialTWR = (infoItems.showVacInitialTWR = stageVisibility[StageData.VacInitialTWR]);
		showAtmoInitialTWR = (infoItems.showAtmoInitialTWR = stageVisibility[StageData.AtmoInitialTWR]);
		showAtmoMaxTWR = (infoItems.showAtmoMaxTWR = stageVisibility[StageData.AtmoMaxTWR]);
		showVacMaxTWR = (infoItems.showVacMaxTWR = stageVisibility[StageData.VacMaxTWR]);
		showAtmoDeltaV = (infoItems.showAtmoDeltaV = stageVisibility[StageData.AtmoDeltaV]);
		showVacDeltaV = (infoItems.showVacDeltaV = stageVisibility[StageData.VacDeltaV]);
		showTime = (infoItems.showTime = stageVisibility[StageData.Time]);
		showISP = (infoItems.showISP = stageVisibility[StageData.Isp]);
		showAtmoCumulativeDeltaV = (infoItems.showAtmoCumulativeDeltaV = stageVisibility[StageData.AtmoCumulativeDeltaV]);
		showVacCumulativeDeltaV = (infoItems.showVacCumulativeDeltaV = stageVisibility[StageData.VacCumulativeDeltaV]);
		showControllableMass = (infoItems.showControllableMass = stageVisibility[StageData.ControllableMass]);
		showRcsUllageTime = (infoItems.showRcsUllageTime = stageVisibility[StageData.RcsUllageTime]);
	}

	private void SetVisibility(int state)
	{
		switch (state)
		{
		case 0:
			SetAllStageVisibility(state: false);
			stageVisibility[StageData.ControllableMass] = _isLoadedRP0;
			stageVisibility[StageData.VacInitialTWR] = true;
			stageVisibility[StageData.AtmoInitialTWR] = true;
			stageVisibility[StageData.VacCumulativeDeltaV] = true;
			stageVisibility[StageData.VacDeltaV] = true;
			stageVisibility[StageData.AtmoDeltaV] = true;
			stageVisibility[StageData.Time] = true;
			break;
		case 1:
			SetAllStageVisibility(state: true);
			stageVisibility[StageData.ControllableMass] = _isLoadedRP0;
			stageVisibility[StageData.RcsUllageTime] = _isLoadedRP0;
			stageVisibility[StageData.AtmoCumulativeDeltaV] = false;
			stageVisibility[StageData.AtmoMaxTWR] = false;
			stageVisibility[StageData.Thrust] = false;
			stageVisibility[StageData.StagedMass] = false;
			stageVisibility[StageData.BurnedMass] = false;
			stageVisibility[StageData.Isp] = false;
			break;
		case 2:
			SetAllStageVisibility(state: true);
			stageVisibility[StageData.RcsUllageTime] = _isLoadedRP0;
			stageVisibility[StageData.ControllableMass] = _isLoadedRP0;
			break;
		case 3:
			LoadStageVisibility();
			break;
		}
	}
}
