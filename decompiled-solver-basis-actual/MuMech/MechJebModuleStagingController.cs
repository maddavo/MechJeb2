using System;
using System.Collections.Generic;
using System.Linq;
using KSP.UI.Screens;
using MechJebLib.FuelFlowSimulation;
using MechJebLibBindings;
using Smooth.Delegates;
using Smooth.Slinq;
using Smooth.Slinq.Context;
using UnityEngine;

namespace MuMech;

public class MechJebModuleStagingController : ComputerModule
{
	private enum RemoteStagingState
	{
		DISABLED,
		WAITING_FOCUS,
		FOCUS_FINISHED
	}

	[Persistent(pass = 6)]
	public readonly EditableDouble AutostagePreDelay = 0.0;

	[Persistent(pass = 6)]
	public readonly EditableDouble AutostagePostDelay = PhysicsGlobals.StagingCooldownTimer;

	[Persistent(pass = 6)]
	public readonly EditableInt AutostageLimit = 0;

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult FairingMaxDynamicPressure = new EditableDoubleMult(5000.0, 1000.0);

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult FairingMinAltitude = new EditableDoubleMult(50000.0, 1000.0);

	[Persistent(pass = 2)]
	public readonly EditableDouble ClampAutoStageThrustPct = 0.99;

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult FairingMaxAerothermalFlux = new EditableDoubleMult(1135.0);

	[Persistent(pass = 6)]
	public bool HotStaging;

	[Persistent(pass = 6)]
	public readonly EditableDouble HotStagingLeadTime = 2.0;

	[Persistent(pass = 6)]
	public bool DropSolids;

	[Persistent(pass = 6)]
	public readonly EditableDoubleMult DropSolidsTwrPct = new EditableDoubleMult(0.5, 0.01);

	public bool AutostagingOnce;

	private bool _waitingForFirstStaging;

	private readonly Dictionary<object, int> _autoStageModuleLimit = new Dictionary<object, int>();

	private readonly List<ModuleEngines> _activeModuleEngines = new List<ModuleEngines>(16);

	private readonly List<ModuleEngines> _allModuleEngines = new List<ModuleEngines>(16);

	private readonly List<PartModule> _allDecouplers = new List<PartModule>(16);

	private readonly List<int> _burnedResources = new List<int>(16);

	private readonly Dictionary<int, bool> _inverseStageHasEngines = new Dictionary<int, bool>(16);

	private readonly Dictionary<int, bool> _inverseStageFiresDecouplerCache = new Dictionary<int, bool>(16);

	private readonly Dictionary<int, bool> _inverseStageReleasesClampsCache = new Dictionary<int, bool>(16);

	private readonly Dictionary<int, bool> _hasStayingChutesCache = new Dictionary<int, bool>(16);

	private readonly Dictionary<int, bool> _hasFairingCache = new Dictionary<int, bool>(16);

	private RemoteStagingState _remoteStagingStatus;

	private readonly string _sFairingMinDynamicPressure = "  " + CachedLocalizer.Instance.MechJebAscentLabel39 + " <";

	private readonly string _sFairingMinAltitude = "  " + CachedLocalizer.Instance.MechJebAscentLabel40 + " >";

	private readonly string _sFairingMaxAerothermalFlux = "  " + CachedLocalizer.Instance.MechJebAscentLabel41 + " <";

	private readonly string _sHotstaging = CachedLocalizer.Instance.MechJebAscentHotStaging + " " + CachedLocalizer.Instance.MechJebAscentLeadTime;

	private readonly string _sDropSolids = CachedLocalizer.Instance.MechJebAscentDropSolids + " " + CachedLocalizer.Instance.MechJebAscentLeadTime;

	private readonly string _sLeadTime = CachedLocalizer.Instance.MechJebAscentLeadTime + ": ";

	private double _lastStageTime;

	private bool? _shouldDropSolids;

	private bool _countingDown;

	private double _stageCountdownStart;

	private Vessel _currentActiveVessel;

	private bool _initializedOnce;

	private readonly List<Part> _partsInStage = new List<Part>();

	private MechJebModuleStageStats _stats => Core.GetComputerModule<MechJebModuleStageStats>();

	private List<FuelStats> _vacStats => _stats.VacStats;

	private bool _droppingSolids
	{
		get
		{
			bool valueOrDefault = _shouldDropSolids.GetValueOrDefault();
			if (!_shouldDropSolids.HasValue)
			{
				valueOrDefault = ShouldDropSolids();
				_shouldDropSolids = valueOrDefault;
				return valueOrDefault;
			}
			return valueOrDefault;
		}
	}

	public MechJebModuleStagingController(MechJebCore core)
		: base(core)
	{
		Priority = 1000;
	}

	public void AutoStageLimitRequest(int stage, object user)
	{
		_autoStageModuleLimit[user] = stage;
	}

	public void AutoStageLimitRemove(object user)
	{
		if (_autoStageModuleLimit.ContainsKey(user))
		{
			_autoStageModuleLimit.Remove(user);
		}
	}

	private int ActiveAutoStageModuleLimit()
	{
		int num = 0;
		foreach (int value in _autoStageModuleLimit.Values)
		{
			if (value > num)
			{
				num = value;
			}
		}
		return num;
	}

	public override void OnStart(StartState state)
	{
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Invalid comparison between Unknown and I4
		if ((Object)(object)base.Vessel != (Object)null && (int)base.Vessel.situation == 4)
		{
			_waitingForFirstStaging = true;
		}
		GameEvents.onStageActivate.Add((OnEvent<int>)StageActivate);
		GameEvents.onVesselResumeStaging.Add((OnEvent<Vessel>)VesselResumeStage);
	}

	private void RegenerateCaches()
	{
		ClearCaches();
		BuildEnginesCache(_allModuleEngines);
		BuildDecouplersCache(_allDecouplers);
	}

	private void OnGUIStageSequenceModified()
	{
		RegenerateCaches();
	}

	private void OnVesselModified(Vessel v)
	{
		if ((Object)(object)base.Vessel == (Object)(object)v)
		{
			RegenerateCaches();
		}
	}

	private void BuildEnginesCache(List<ModuleEngines> engines)
	{
		engines.Clear();
		foreach (Part part in base.Vessel.Parts)
		{
			if (part.IsEngine() && !part.IsSepratron())
			{
				engines.AddRange(part.FindModulesImplementing<ModuleEngines>());
			}
		}
	}

	private void BuildDecouplersCache(List<PartModule> decouplers)
	{
		decouplers.Clear();
		foreach (Part part in base.Vessel.Parts)
		{
			if (!part.IsDecoupler())
			{
				continue;
			}
			foreach (PartModule module in part.Modules)
			{
				if (module is ModuleDecouplerBase || module is ModuleDockingNode || module.moduleName == "ProceduralFairingDecoupler")
				{
					decouplers.Add(module);
				}
			}
		}
	}

	private void ClearCaches()
	{
		_inverseStageHasEngines.Clear();
		_inverseStageFiresDecouplerCache.Clear();
		_inverseStageReleasesClampsCache.Clear();
		_hasStayingChutesCache.Clear();
		_hasFairingCache.Clear();
	}

	private void VesselResumeStage(Vessel data)
	{
		if (_remoteStagingStatus == RemoteStagingState.WAITING_FOCUS)
		{
			_remoteStagingStatus = RemoteStagingState.FOCUS_FINISHED;
		}
	}

	public override void OnDestroy()
	{
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Expected O, but got Unknown
		GameEvents.onStageActivate.Remove((OnEvent<int>)StageActivate);
		GameEvents.onVesselResumeStaging.Remove((OnEvent<Vessel>)VesselResumeStage);
		GameEvents.onVesselWasModified.Remove((OnEvent<Vessel>)OnVesselModified);
		StageManager.OnGUIStageSequenceModified.Remove(new OnEvent(OnGUIStageSequenceModified));
	}

	private void StageActivate(int data)
	{
		_waitingForFirstStaging = false;
	}

	public void AutostageOnce(object user)
	{
		Users.Add(user);
		AutostagingOnce = true;
	}

	protected override void OnModuleEnabled()
	{
		_autoStageModuleLimit.Clear();
	}

	protected override void OnModuleDisabled()
	{
		AutostagingOnce = false;
	}

	[GeneralInfoItem("#MechJeb_AutostagingSettings", InfoItem.Category.Misc)]
	public void AutostageSettingsInfoItem()
	{
		GUILayout.BeginVertical(GUI.skin.box, Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel42, AutostageLimit, null, 40f);
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Delays: ", Array.Empty<GUILayoutOption>());
		GUILayout.Space(30f);
		GuiUtils.SimpleTextBox("pre: ", AutostagePreDelay, "s", 35f, null, horizontalFraming: false);
		GUILayout.FlexibleSpace();
		GuiUtils.SimpleTextBox("post: ", AutostagePostDelay, "s", 35f, null, horizontalFraming: false);
		GUILayout.EndHorizontal();
		ClampAutostageThrust();
		GUILayout.Label(CachedLocalizer.Instance.MechJebAscentLabel38, Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox(_sFairingMinDynamicPressure, FairingMaxDynamicPressure, "kPa", 50f);
		GuiUtils.SimpleTextBox(_sFairingMinAltitude, FairingMinAltitude, "km", 50f);
		GuiUtils.SimpleTextBox(_sFairingMaxAerothermalFlux, FairingMaxAerothermalFlux, "W/m²", 50f);
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		HotStaging = GUILayout.Toggle(HotStaging, CachedLocalizer.Instance.MechJebAscentHotStaging, Array.Empty<GUILayoutOption>());
		GUILayout.FlexibleSpace();
		GuiUtils.SimpleTextBox(_sLeadTime, HotStagingLeadTime, "s", 35f);
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		DropSolids = GUILayout.Toggle(DropSolids, CachedLocalizer.Instance.MechJebAscentDropSolids, Array.Empty<GUILayoutOption>());
		GUILayout.FlexibleSpace();
		GuiUtils.SimpleTextBox("", DropSolidsTwrPct, "% rocket accel", 35f);
		GUILayout.EndHorizontal();
		GUILayout.EndVertical();
	}

	[ValueInfoItem("#MechJeb_Autostagingstatus", InfoItem.Category.Misc)]
	public string AutostageStatus()
	{
		if (!base.Enabled)
		{
			return CachedLocalizer.Instance.MechJebAscentStatus9;
		}
		if (AutostagingOnce)
		{
			return CachedLocalizer.Instance.MechJebAscentStatus10;
		}
		return CachedLocalizer.Instance.MechJebAscentStatus11 + (int)AutostageLimit;
	}

	[GeneralInfoItem("#MechJeb_ClampAutostageThrust", InfoItem.Category.Misc)]
	private void ClampAutostageThrust()
	{
		double num = Core.Staging.ClampAutoStageThrustPct;
		GuiUtils.SimpleTextBox(CachedLocalizer.Instance.MechJebAscentLabel44, Core.Staging.ClampAutoStageThrustPct, "%", 50f);
		if (num != (double)Core.Staging.ClampAutoStageThrustPct)
		{
			Core.Staging.ClampAutoStageThrustPct.Val = UtilMath.Clamp((double)Core.Staging.ClampAutoStageThrustPct, 0.0, 100.0);
		}
	}

	public override void OnFixedUpdate()
	{
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Expected O, but got Unknown
		if (!_initializedOnce)
		{
			if (HighLogic.LoadedSceneIsFlight)
			{
				GameEvents.onVesselWasModified.Add((OnEvent<Vessel>)OnVesselModified);
				StageManager.OnGUIStageSequenceModified.Add(new OnEvent(OnGUIStageSequenceModified));
			}
			_initializedOnce = true;
			RegenerateCaches();
		}
		if (_waitingForFirstStaging || base.Vessel.currentStage <= 0 || base.Vessel.currentStage <= (int)AutostageLimit || base.VesselState.Time - _lastStageTime < (double)AutostagePostDelay)
		{
			return;
		}
		if (base.Vessel.currentStage <= ActiveAutoStageModuleLimit())
		{
			if (HasFairing(base.Vessel.currentStage - 1) && !WaitingForFairing())
			{
				Stage();
			}
			return;
		}
		UpdateActiveModuleEngines(_allModuleEngines);
		UpdateBurnedResources();
		_shouldDropSolids = null;
		if (InverseStageDecouplesActiveOrIdleEngineOrTank(base.Vessel.currentStage - 1, _burnedResources, _activeModuleEngines) && !InverseStageReleasesClamps(base.Vessel.currentStage - 1))
		{
			return;
		}
		if (InverseStageHasUnstableEngines(base.Vessel.currentStage - 1) && Core.Thrust.AutoRCSUllaging && base.Vessel.hasEnabledRCSModules() && Core.Thrust.LastThrottle > 0f)
		{
			if (!base.Vessel.ActionGroups[(KSPActionGroup)8])
			{
				base.Vessel.ActionGroups.SetGroup((KSPActionGroup)8, true);
			}
		}
		else if (!InverseStageHasActiveEngines(base.Vessel.currentStage))
		{
			Stage();
		}
		else
		{
			if ((HotStaging && InverseStageHasEngines(base.Vessel.currentStage - 1) && !InverseStageFiresDecoupler(base.Vessel.currentStage - 1) && !InverseStageReleasesClamps(base.Vessel.currentStage - 1) && LastNonZeroDVStageBurnTime() > (double)HotStagingLeadTime) || HasStayingChutes(base.Vessel.currentStage - 1))
			{
				return;
			}
			if (InverseStageDecouplesDeactivatedEngineOrTank(base.Vessel.currentStage - 1))
			{
				Stage();
			}
			else if (!WaitingForFairing())
			{
				if ((base.VesselState.ThrustCurrent / base.VesselState.ThrustAvailable < (double)ClampAutoStageThrustPct || AnyFailedEngines(_allModuleEngines)) && InverseStageReleasesClamps(base.Vessel.currentStage - 1))
				{
					Core.Attitude.Controller.Reset();
				}
				else
				{
					Stage();
				}
			}
		}
	}

	private bool WaitingForFairing()
	{
		if (!HasFairing(base.Vessel.currentStage - 1))
		{
			return false;
		}
		if (Core.VesselState.DynamicPressure > (double)FairingMaxDynamicPressure)
		{
			return true;
		}
		if (Core.VesselState.AltitudeASL < (double)FairingMinAltitude)
		{
			return true;
		}
		if (Core.VesselState.FreeMolecularAerothermalFlux > (double)FairingMaxAerothermalFlux)
		{
			return true;
		}
		return false;
	}

	public void ImmediateStage()
	{
		if (InverseStageFiresDecoupler(base.Vessel.currentStage - 1))
		{
			_lastStageTime = base.VesselState.Time;
		}
		if (!base.Vessel.isActiveVessel)
		{
			_currentActiveVessel = FlightGlobals.ActiveVessel;
			Debug.Log((object)("Mechjeb Autostage: Switching from " + ((Object)FlightGlobals.ActiveVessel).name + " to vessel " + ((Object)base.Vessel).name + " to stage"));
			_remoteStagingStatus = RemoteStagingState.WAITING_FOCUS;
			FlightGlobals.ForceSetActiveVessel(base.Vessel);
		}
		else
		{
			Debug.Log((object)("Mechjeb Autostage: Executing next stage on " + ((Object)FlightGlobals.ActiveVessel).name));
			switch (_remoteStagingStatus)
			{
			case RemoteStagingState.DISABLED:
				StageManager.ActivateNextStage();
				break;
			case RemoteStagingState.FOCUS_FINISHED:
				StageManager.ActivateNextStage();
				FlightGlobals.ForceSetActiveVessel(_currentActiveVessel);
				Debug.Log((object)("Mechjeb Autostage: Has switching back to " + ((Object)FlightGlobals.ActiveVessel).name + " "));
				_remoteStagingStatus = RemoteStagingState.DISABLED;
				break;
			}
		}
		_countingDown = false;
		if (AutostagingOnce)
		{
			Users.Clear();
		}
	}

	public void Stage()
	{
		if (!_countingDown)
		{
			_countingDown = true;
			_stageCountdownStart = base.VesselState.Time;
		}
		if (base.VesselState.Time - _stageCountdownStart >= (double)AutostagePreDelay)
		{
			ImmediateStage();
		}
	}

	private bool InverseStageDecouplesActiveOrIdleEngineOrTank(int inverseStage, List<int> tankResources, List<ModuleEngines> activeModuleEngines)
	{
		foreach (PartModule allDecoupler in _allDecouplers)
		{
			if (allDecoupler.part.inverseStage == inverseStage && allDecoupler.IsUnfiredDecoupler(out var decoupledPart) && HasActiveOrIdleEngineOrTankDescendant(decoupledPart, tankResources, activeModuleEngines))
			{
				return true;
			}
		}
		return false;
	}

	private double LastNonZeroDVStageBurnTime()
	{
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0084: Unknown result type (might be due to invalid IL or missing references)
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		_stats.RequestUpdate();
		int num = -1;
		double num2 = 0.0;
		int num3;
		for (num3 = _vacStats.Count - 1; num3 >= 0; num3--)
		{
			if (_vacStats[num3].DeltaTime > 0.0)
			{
				num = _vacStats[num3].KSPStage;
				break;
			}
		}
		while (num3 >= 0 && _vacStats[num3].KSPStage == num)
		{
			num2 += _vacStats[num3].DeltaTime;
			num3--;
		}
		return num2;
	}

	private bool InverseStageHasActiveEngines(int inverseStage)
	{
		foreach (ModuleEngines allModuleEngine in _allModuleEngines)
		{
			if (((PartModule)allModuleEngine).part.inverseStage >= inverseStage && allModuleEngine.EngineHasFuel())
			{
				return true;
			}
		}
		return false;
	}

	private bool InverseStageHasEngines(int inverseStage)
	{
		if (_inverseStageHasEngines.TryGetValue(inverseStage, out var value))
		{
			return value;
		}
		value = (Object)(object)_allModuleEngines.FirstOrDefault((ModuleEngines me) => ((PartModule)me).part.inverseStage == inverseStage) != (Object)null;
		_inverseStageHasEngines.Add(inverseStage, value);
		return value;
	}

	private bool InverseStageHasUnstableEngines(int inverseStage)
	{
		foreach (ModuleEngines allModuleEngine in _allModuleEngines)
		{
			if (((PartModule)allModuleEngine).part.inverseStage == inverseStage && ((PartModule)allModuleEngine).part.UnstableUllage())
			{
				return true;
			}
		}
		return false;
	}

	private bool AnyFailedEngines(List<ModuleEngines> allEngines)
	{
		foreach (ModuleEngines allEngine in allEngines)
		{
			Part part = ((PartModule)allEngine).part;
			if (part.inverseStage >= base.Vessel.currentStage && !part.IsDecoupledInStage(base.Vessel.currentStage - 1) && ((PartModule)allEngine).isEnabled && !allEngine.EngineIgnited && allEngine.allowShutdown)
			{
				return true;
			}
		}
		return false;
	}

	private void UpdateActiveModuleEngines(List<ModuleEngines> allEngines)
	{
		_activeModuleEngines.Clear();
		foreach (ModuleEngines allEngine in allEngines)
		{
			Part part = ((PartModule)allEngine).part;
			if (part.inverseStage >= base.Vessel.currentStage && !part.IsDecoupledInStage(base.Vessel.currentStage - 1) && ((PartModule)allEngine).isEnabled)
			{
				_activeModuleEngines.Add(allEngine);
			}
		}
	}

	private void UpdateBurnedResources()
	{
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_0059: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		_burnedResources.Clear();
		Slinq.Select<int, Propellant, SelectSlinqContext<Propellant, IListContext<Propellant>, ModuleEngines, IListContext<ModuleEngines>>>(Slinq.SelectMany<Propellant, IListContext<Propellant>, ModuleEngines, IListContext<ModuleEngines>>(Slinqable.Slinq<ModuleEngines>((IList<ModuleEngines>)_activeModuleEngines), (DelegateFunc<ModuleEngines, Slinq<Propellant, IListContext<Propellant>>>)((ModuleEngines eng) => Slinqable.Slinq<Propellant>((IList<Propellant>)eng.propellants))), (DelegateFunc<Propellant, int>)((Propellant prop) => prop.id)).AddTo<List<int>>(_burnedResources);
	}

	private bool IsBurnedOutSrbDecoupledInNextStage(Part p)
	{
		if (DropSolids && p.IsThrottleLockedEngine() && p.IsDecoupledInStage(base.Vessel.currentStage - 1))
		{
			return _droppingSolids;
		}
		return false;
	}

	private bool ShouldDropSolids()
	{
		if (!DropSolids)
		{
			return false;
		}
		double num = base.VesselState.CurrentThrustAcceleration * (double)DropSolidsTwrPct;
		double num2 = 0.0;
		foreach (PartModule allDecoupler in _allDecouplers)
		{
			if (allDecoupler.part.inverseStage != base.Vessel.currentStage - 1 || !allDecoupler.IsUnfiredDecoupler(out var decoupledPart))
			{
				continue;
			}
			double thrust = 0.0;
			double mass = 0.0;
			SumStackThrustAndMass(decoupledPart, ref thrust, ref mass);
			if (!(mass <= 0.0))
			{
				double num3 = thrust / mass;
				if (num3 > num2)
				{
					num2 = num3;
				}
			}
		}
		return num2 <= num;
	}

	private static void SumStackThrustAndMass(Part p, ref double thrust, ref double mass)
	{
		if (p == null)
		{
			return;
		}
		mass += p.mass + p.GetResourceMass();
		for (int i = 0; i < p.Modules.Count; i++)
		{
			PartModule obj = p.Modules[i];
			ModuleEngines val = (ModuleEngines)(object)((obj is ModuleEngines) ? obj : null);
			if (val != null)
			{
				thrust += val.finalThrust;
			}
		}
		for (int j = 0; j < p.children.Count; j++)
		{
			SumStackThrustAndMass(p.children[j], ref thrust, ref mass);
		}
	}

	private bool HasActiveOrIdleEngineOrTankDescendant(Part p, List<int> tankResources, List<ModuleEngines> activeModuleEngines)
	{
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Invalid comparison between Unknown and I4
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c9: Unknown result type (might be due to invalid IL or missing references)
		if (p == null)
		{
			return false;
		}
		if (!p.IsSepratron() && !IsBurnedOutSrbDecoupledInNextStage(p))
		{
			if (((int)p.State == 1 || (int)p.State == 0) && p.EngineHasFuel() && !PartExtensions.IsUnrestartableDeadEngine(p))
			{
				return true;
			}
			foreach (ModuleEngines activeModuleEngine in activeModuleEngines)
			{
				foreach (Propellant propellant in activeModuleEngine.propellants)
				{
					if (!p.Resources.Contains(propellant.id))
					{
						continue;
					}
					PartResource val = p.Resources.Get(propellant.id);
					if (val.amount <= p.resourceRequestRemainingThreshold || val.info.id == PartResourceLibrary.ElectricityHashcode || !tankResources.Contains(val.info.id))
					{
						continue;
					}
					if ((int)propellant.GetFlowMode() == 0)
					{
						if ((Object)(object)((PartModule)activeModuleEngine).part == (Object)(object)p)
						{
							return true;
						}
					}
					else if (((PartModule)activeModuleEngine).part.crossfeedPartSet.ContainsPart(p))
					{
						return true;
					}
				}
			}
		}
		for (int i = 0; i < p.children.Count; i++)
		{
			if (HasActiveOrIdleEngineOrTankDescendant(p.children[i], tankResources, activeModuleEngines))
			{
				return true;
			}
		}
		return false;
	}

	private bool InverseStageFiresDecoupler(int inverseStage)
	{
		if (_inverseStageFiresDecouplerCache.TryGetValue(inverseStage, out var value))
		{
			return value;
		}
		value = (Object)(object)base.Vessel.Parts.FirstOrDefault((Part p) => p.inverseStage == inverseStage && p.IsUnfiredDecoupler(out var _)) != (Object)null;
		_inverseStageFiresDecouplerCache.Add(inverseStage, value);
		return value;
	}

	private bool InverseStageReleasesClamps(int inverseStage)
	{
		if (_inverseStageReleasesClampsCache.TryGetValue(inverseStage, out var value))
		{
			return value;
		}
		value = (Object)(object)base.Vessel.Parts.FirstOrDefault((Part p) => p.inverseStage == inverseStage && p.IsLaunchClamp()) != (Object)null;
		_inverseStageReleasesClampsCache.Add(inverseStage, value);
		return value;
	}

	private bool InverseStageDecouplesDeactivatedEngineOrTank(int inverseStage)
	{
		foreach (PartModule allDecoupler in _allDecouplers)
		{
			if (allDecoupler.part.inverseStage == inverseStage && allDecoupler.IsUnfiredDecoupler(out var decoupledPart) && HasDeactivatedEngineOrTankDescendant(decoupledPart))
			{
				return true;
			}
		}
		return false;
	}

	private static bool HasDeactivatedEngineOrTankDescendant(Part p)
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Invalid comparison between Unknown and I4
		if (p == null)
		{
			return false;
		}
		if ((int)p.State == 2 && p.IsEngine() && !p.IsSepratron())
		{
			return true;
		}
		bool flag = false;
		bool flag2 = false;
		for (int i = 0; i < p.Resources.Count; i++)
		{
			PartResource val = p.Resources[i];
			if (val.info.id != PartResourceLibrary.ElectricityHashcode)
			{
				if (val.maxAmount > p.resourceRequestRemainingThreshold)
				{
					flag = true;
				}
				if (val.amount > p.resourceRequestRemainingThreshold)
				{
					flag2 = true;
				}
			}
		}
		if (flag && !flag2)
		{
			return true;
		}
		if (p.IsEngine() && !p.EngineHasFuel())
		{
			return true;
		}
		for (int j = 0; j < p.children.Count; j++)
		{
			if (HasDeactivatedEngineOrTankDescendant(p.children[j]))
			{
				return true;
			}
		}
		return false;
	}

	private bool HasStayingChutes(int inverseStage)
	{
		if (_hasStayingChutesCache.TryGetValue(inverseStage, out var value))
		{
			return value;
		}
		value = (Object)(object)base.Vessel.Parts.FirstOrDefault((Part p) => p.inverseStage == inverseStage && p.IsParachute() && !p.IsDecoupledInStage(inverseStage)) != (Object)null;
		_hasStayingChutesCache.Add(inverseStage, value);
		return value;
	}

	private bool HasFairing(int inverseStage)
	{
		if (_hasFairingCache.TryGetValue(inverseStage, out var value))
		{
			return value;
		}
		value = HasFairingUncached(inverseStage);
		_hasFairingCache.Add(inverseStage, value);
		return value;
	}

	private bool HasFairingUncached(int inverseStage)
	{
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0055: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_0089: Unknown result type (might be due to invalid IL or missing references)
		//IL_008e: Unknown result type (might be due to invalid IL or missing references)
		_partsInStage.Clear();
		Slinq.Where<Part, IListContext<Part>, int>(Slinqable.Slinq<Part>((IList<Part>)base.Vessel.parts), (DelegateFunc<Part, int, bool>)((Part p, int s) => p.hasStagingIcon && p.inverseStage == s), inverseStage).AddTo<List<Part>>(_partsInStage);
		if (Slinqable.Slinq<Part>((IList<Part>)_partsInStage).Any((DelegateFunc<Part, bool>)((Part p) => p.IsProceduralFairing())))
		{
			return Slinqable.Slinq<Part>((IList<Part>)_partsInStage).All((DelegateFunc<Part, bool>)((Part p) => p.IsProceduralFairingPayloadFairing()));
		}
		return Slinqable.Slinq<Part>((IList<Part>)_partsInStage).All((DelegateFunc<Part, bool>)((Part p) => ((p.IsDecoupler() && p.children.Count == 0) || p.HasModule<ModuleProceduralFairing>()) && !p.IsLaunchClamp()));
	}
}
