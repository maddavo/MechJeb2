using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using KSP.Localization;
using MechJebLib.Utils;
using UnityEngine;
using UnityToolbag;

namespace MuMech;

public class MechJebCore : PartModule, IComparable<MechJebCore>
{
	private readonly List<ComputerModule> _unorderedComputerModules = new List<ComputerModule>();

	private readonly List<ComputerModule> _modulesToLoad = new List<ComputerModule>();

	private readonly Dictionary<Type, List<ComputerModule>> _sortedModules = new Dictionary<Type, List<ComputerModule>>();

	private readonly Dictionary<object, List<DisplayModule>> _sortedDisplayModules = new Dictionary<object, List<DisplayModule>>();

	private static readonly Dictionary<string, ConfigNode> _savedConfig = new Dictionary<string, ConfigNode>();

	private readonly List<Callback> _postDrawQueue = new List<Callback>();

	private static List<Type> _moduleRegistry;

	private bool _ready;

	public MechJebModuleGuidanceController Guidance;

	public MechJebModulePSGGlueBall Glueball;

	public MechJebModuleAttitudeController Attitude;

	public MechJebModuleStagingController Staging;

	public MechJebModuleThrustController Thrust;

	public MechJebModuleTargetController Target;

	public MechJebModuleWarpController Warp;

	public MechJebModuleRCSController RCS;

	public MechJebModuleRCSBalancer Rcsbal;

	public MechJebModuleRoverController Rover;

	public MechJebModuleNodeExecutor Node;

	public MechJebModuleSolarPanelController Solarpanel;

	public MechJebModuleDeployableAntennaController AntennaControl;

	public MechJebModuleLandingAutopilot Landing;

	public MechJebModuleSettings Settings;

	public MechJebModuleStageStats StageStats;

	public MechJebModuleAscentSettings AscentSettings;

	public MechJebModuleSpinupController Spinup;

	public MechJebModuleHoverslamSimulation Hoverslam;

	public MechJebModuleSmartASS SmartASS;

	public readonly VesselState VesselState;

	[KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "MechJeb")]
	[UI_Toggle(disabledText = "#MechJeb_Disabled", enabledText = "#MechJeb_Enabled")]
	public bool running = true;

	private Vessel _controlledVessel;

	public string version = "";

	private bool _deactivateControl;

	private float _currentThrottle;

	[KSPField(isPersistant = false)]
	public string blacklist = "";

	[KSPField]
	private ConfigNode _partSettings;

	[KSPField(isPersistant = false)]
	public bool eduMode;

	private bool _weLockedInputs;

	private float _lastSettingsSaveTime;

	private bool _wasMasterAndFocus;

	private static Vessel _lastFocus;

	public bool someModuleAreLocked;

	private const string LOCK_ID = "MechJeb_noclick";

	public MechJebModuleAscentBaseAutopilot Ascent => AscentSettings.AscentAutopilot;

	public MechJebCore MasterMechJeb => ((PartModule)this).vessel.GetMasterMechJeb();

	public bool DeactivateControl
	{
		get
		{
			if (((PartModule)this).vessel.GetMasterMechJeb() != null)
			{
				return ((PartModule)this).vessel.GetMasterMechJeb()._deactivateControl;
			}
			return false;
		}
		set
		{
			MechJebCore masterMechJeb = ((PartModule)this).vessel.GetMasterMechJeb();
			if (!((Object)(object)masterMechJeb == (Object)null))
			{
				if (value && !masterMechJeb._deactivateControl)
				{
					_controlledVessel.ctrlState.mainThrottle = _currentThrottle;
				}
				masterMechJeb._deactivateControl = value;
			}
		}
	}

	public bool RssMode => Settings.RssMode;

	public bool ShowGui { get; private set; } = true;


	[KSPAction("#MechJeb_OrbitPrograde")]
	public void OnOrbitProgradeAction(KSPActionParam param)
	{
		EngageSmartASSOrbitalControl(MechJebModuleSmartASS.Target.PROGRADE);
	}

	[KSPAction("#MechJeb_OrbitRetrograde")]
	public void OnOrbitRetrogradeAction(KSPActionParam param)
	{
		EngageSmartASSOrbitalControl(MechJebModuleSmartASS.Target.RETROGRADE);
	}

	[KSPAction("#MechJeb_OrbitNormal")]
	public void OnOrbitNormalAction(KSPActionParam param)
	{
		EngageSmartASSOrbitalControl(MechJebModuleSmartASS.Target.NORMAL_PLUS);
	}

	[KSPAction("#MechJeb_OrbitAntinormal")]
	public void OnOrbitAntinormalAction(KSPActionParam param)
	{
		EngageSmartASSOrbitalControl(MechJebModuleSmartASS.Target.NORMAL_MINUS);
	}

	[KSPAction("#MechJeb_OrbitRadialIn")]
	public void OnOrbitRadialInAction(KSPActionParam param)
	{
		EngageSmartASSOrbitalControl(MechJebModuleSmartASS.Target.RADIAL_MINUS);
	}

	[KSPAction("#MechJeb_OrbitRadialOut")]
	public void OnOrbitRadialOutAction(KSPActionParam param)
	{
		EngageSmartASSOrbitalControl(MechJebModuleSmartASS.Target.RADIAL_PLUS);
	}

	[KSPAction("#MechJeb_OrbitKillRotation")]
	public void OnKillRotationAction(KSPActionParam param)
	{
		EngageSmartASSOrbitalControl(MechJebModuleSmartASS.Target.KILLROT);
	}

	[KSPAction("#MechJeb_DeactivateSmartACS")]
	public void OnDeactivateSmartASSAction(KSPActionParam param)
	{
		EngageSmartASSOrbitalControl(MechJebModuleSmartASS.Target.OFF);
	}

	[KSPAction("#MechJeb_LandSomewhere")]
	public void OnLandsomewhereAction(KSPActionParam param)
	{
		LandSomewhere();
	}

	[KSPAction("#MechJeb_LandatKSC")]
	public void OnLandTargetAction(KSPActionParam param)
	{
		LandTarget();
	}

	private void LandTarget()
	{
		if (!((Object)(object)((PartModule)this).vessel.GetMasterMechJeb() == (Object)null))
		{
			GetComputerModule<MechJebModuleLandingGuidance>()?.SetAndLandTargetKSC();
		}
	}

	private void LandSomewhere()
	{
		if (!((Object)(object)((PartModule)this).vessel.GetMasterMechJeb() == (Object)null))
		{
			GetComputerModule<MechJebModuleLandingGuidance>()?.LandSomewhere();
		}
	}

	private void EngageSmartASSOrbitalControl(MechJebModuleSmartASS.Target smartassTarget)
	{
		MechJebCore masterMechJeb = ((PartModule)this).vessel.GetMasterMechJeb();
		if ((Object)(object)masterMechJeb == (Object)null)
		{
			Debug.LogError((object)Localizer.Format("#MechJeb_LogError_msg0"));
			return;
		}
		MechJebModuleSmartASS computerModule = masterMechJeb.GetComputerModule<MechJebModuleSmartASS>();
		if (computerModule != null && !computerModule.Hidden)
		{
			computerModule.mode = MechJebModuleSmartASS.Mode.ORBITAL;
			computerModule.target = smartassTarget;
			computerModule.Engage();
		}
		else
		{
			Debug.LogError((object)Localizer.Format("#MechJeb_LogError_msg1"));
		}
	}

	[KSPAction("#MechJeb_PANIC")]
	public void OnPanicAction(KSPActionParam param)
	{
		MechJebCore masterMechJeb = ((PartModule)this).vessel.GetMasterMechJeb();
		if (!((Object)(object)masterMechJeb == (Object)null))
		{
			MechJebModuleTranslatron computerModule = masterMechJeb.GetComputerModule<MechJebModuleTranslatron>();
			if (computerModule != null && !computerModule.Hidden)
			{
				computerModule.PanicSwitch();
			}
		}
	}

	[KSPAction("#MechJeb_TranslatronOFF")]
	public void OnTranslatronOffAction(KSPActionParam param)
	{
		EngageTranslatronControl(MechJebModuleThrustController.TMode.OFF);
	}

	[KSPAction("#MechJeb_TranslatronKeepVert")]
	public void OnTranslatronKeepVertAction(KSPActionParam param)
	{
		EngageTranslatronControl(MechJebModuleThrustController.TMode.KEEP_VERTICAL);
	}

	[KSPAction("#MechJeb_TranslatronZerospeed")]
	public void OnTranslatronZeroSpeedAction(KSPActionParam param)
	{
		SetTranslatronSpeed(0f);
	}

	[KSPAction("#MechJeb_TranslatronPlusspeed")]
	public void OnTranslatronPlusOneSpeedAction(KSPActionParam param)
	{
		SetTranslatronSpeed(1f, relative: true);
	}

	[KSPAction("#MechJeb_TranslatronMinusspeed")]
	public void OnTranslatronMinusOneSpeedAction(KSPActionParam param)
	{
		SetTranslatronSpeed(-1f, relative: true);
	}

	[KSPAction("#MechJeb_TranslatronToggleHS")]
	public void OnTranslatronToggleHSAction(KSPActionParam param)
	{
		MechJebCore masterMechJeb = ((PartModule)this).vessel.GetMasterMechJeb();
		if ((Object)(object)masterMechJeb != (Object)null)
		{
			MechJebModuleTranslatron computerModule = masterMechJeb.GetComputerModule<MechJebModuleTranslatron>();
			if (computerModule != null && !computerModule.Hidden)
			{
				Thrust.TransKillH = !Thrust.TransKillH;
			}
			else
			{
				Debug.LogError((object)Localizer.Format("#MechJeb_LogError_msg2"));
			}
		}
		else
		{
			Debug.LogError((object)"MechJeb couldn't find the master MechJeb module for the current vessel.");
		}
	}

	[KSPAction("#MechJeb_AscentAPtoggle")]
	public void OnAscentAPToggleAction(KSPActionParam param)
	{
		MechJebModuleAscentBaseAutopilot ascentAutopilot = ((PartModule)this).vessel.GetMasterMechJeb().AscentSettings.AscentAutopilot;
		MechJebModuleAscentMenu computerModule = GetComputerModule<MechJebModuleAscentMenu>();
		if (ascentAutopilot != null && computerModule != null)
		{
			if (ascentAutopilot.Enabled)
			{
				ascentAutopilot.Users.Remove(computerModule);
			}
			else
			{
				ascentAutopilot.Users.Add(computerModule);
			}
		}
	}

	private void EngageTranslatronControl(MechJebModuleThrustController.TMode mode)
	{
		MechJebCore masterMechJeb = ((PartModule)this).vessel.GetMasterMechJeb();
		if ((Object)(object)masterMechJeb == (Object)null)
		{
			Debug.LogError((object)"MechJeb couldn't find the master MechJeb module for the current vessel.");
			return;
		}
		MechJebModuleTranslatron computerModule = masterMechJeb.GetComputerModule<MechJebModuleTranslatron>();
		if (computerModule != null && !computerModule.Hidden)
		{
			if (Thrust.Users.Count <= 1 || Thrust.Users.Contains(computerModule))
			{
				computerModule.SetMode(mode);
			}
		}
		else
		{
			Debug.LogError((object)"MechJeb couldn't find MechJebModuleTranslatron for translatron control via action group.");
		}
	}

	private void SetTranslatronSpeed(float speed, bool relative = false)
	{
		MechJebCore masterMechJeb = ((PartModule)this).vessel.GetMasterMechJeb();
		if ((Object)(object)masterMechJeb == (Object)null)
		{
			Debug.LogError((object)"MechJeb couldn't find the master MechJeb module for the current vessel.");
			return;
		}
		MechJebModuleTranslatron computerModule = masterMechJeb.GetComputerModule<MechJebModuleTranslatron>();
		if (computerModule != null && !computerModule.Hidden)
		{
			Thrust.TransSpdAct = (relative ? Thrust.TransSpdAct : 0f) + speed;
		}
		else
		{
			Debug.LogError((object)"MechJeb couldn't find MechJebModuleTranslatron for translatron control via action group.");
		}
	}

	public MechJebCore()
	{
		VesselState = new VesselState(this);
	}

	private bool CheckControlledVessel()
	{
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Expected O, but got Unknown
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Expected O, but got Unknown
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0075: Expected O, but got Unknown
		//IL_0075: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Expected O, but got Unknown
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_009c: Expected O, but got Unknown
		//IL_009c: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a6: Expected O, but got Unknown
		if ((Object)(object)_controlledVessel == (Object)(object)((PartModule)this).vessel)
		{
			return true;
		}
		if ((Object)(object)_controlledVessel != (Object)null)
		{
			Vessel controlledVessel = _controlledVessel;
			controlledVessel.OnFlyByWire = (FlightInputCallback)Delegate.Remove((Delegate)(object)controlledVessel.OnFlyByWire, (Delegate)new FlightInputCallback(OnFlyByWire));
		}
		if ((Object)(object)((PartModule)this).vessel != (Object)null)
		{
			Vessel vessel = ((PartModule)this).vessel;
			vessel.OnFlyByWire = (FlightInputCallback)Delegate.Remove((Delegate)(object)vessel.OnFlyByWire, (Delegate)new FlightInputCallback(OnFlyByWire));
			Vessel vessel2 = ((PartModule)this).vessel;
			vessel2.OnFlyByWire = (FlightInputCallback)Delegate.Combine((Delegate)(object)vessel2.OnFlyByWire, (Delegate)new FlightInputCallback(OnFlyByWire));
		}
		_controlledVessel = ((PartModule)this).vessel;
		return false;
	}

	public int GetImportance()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Invalid comparison between Unknown and I4
		if ((int)((PartModule)this).part.State != 3)
		{
			return ((Object)this).GetInstanceID();
		}
		return 0;
	}

	public int CompareTo(MechJebCore other)
	{
		if ((Object)(object)other == (Object)null)
		{
			return 1;
		}
		return GetImportance().CompareTo(other.GetImportance());
	}

	public T GetComputerModule<T>() where T : ComputerModule
	{
		for (int i = 0; i < _unorderedComputerModules.Count; i++)
		{
			if (_unorderedComputerModules[i] is T result)
			{
				return result;
			}
		}
		return null;
	}

	public List<ComputerModule> GetComputerModules<T>() where T : ComputerModule
	{
		Type typeFromHandle = typeof(T);
		if (_sortedModules.TryGetValue(typeFromHandle, out var value))
		{
			return value;
		}
		return _sortedModules[typeFromHandle] = (from ComputerModule m in _unorderedComputerModules.OfType<T>()
			orderby m
			select m).ToList();
	}

	public List<DisplayModule> GetDisplayModules(IComparer<DisplayModule> comparer)
	{
		if (_sortedDisplayModules.TryGetValue(comparer, out var value))
		{
			return value;
		}
		return _sortedDisplayModules[comparer] = _unorderedComputerModules.OfType<DisplayModule>().OrderBy((DisplayModule m) => m, comparer).ToList();
	}

	public ComputerModule GetComputerModule(string type)
	{
		return _unorderedComputerModules.FirstOrDefault((ComputerModule a) => a.GetType().Name.ToLowerInvariant() == type.ToLowerInvariant());
	}

	public void AddComputerModule(ComputerModule module)
	{
		_unorderedComputerModules.Add(module);
		ClearModulesCache();
	}

	private void ClearModulesCache()
	{
		_sortedModules.Clear();
		_sortedDisplayModules.Clear();
	}

	public void AddComputerModuleLater(ComputerModule module)
	{
		_modulesToLoad.Add(module);
	}

	private void LoadDelayedModules()
	{
		if (_modulesToLoad.Count > 0)
		{
			_unorderedComputerModules.AddRange(_modulesToLoad);
			_modulesToLoad.Clear();
			ClearModulesCache();
		}
	}

	public void RemoveComputerModule(ComputerModule module)
	{
		_unorderedComputerModules.Remove(module);
		ClearModulesCache();
	}

	public void ReloadAllComputerModules()
	{
		//IL_0065: Unknown result type (might be due to invalid IL or missing references)
		//IL_006f: Expected O, but got Unknown
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Expected O, but got Unknown
		foreach (ComputerModule unorderedComputerModule in _unorderedComputerModules)
		{
			unorderedComputerModule.OnDestroy();
		}
		_unorderedComputerModules.Clear();
		ClearModulesCache();
		if ((Object)(object)((PartModule)this).vessel != (Object)null)
		{
			Vessel vessel = ((PartModule)this).vessel;
			vessel.OnFlyByWire = (FlightInputCallback)Delegate.Remove((Delegate)(object)vessel.OnFlyByWire, (Delegate)new FlightInputCallback(OnFlyByWire));
		}
		_controlledVessel = null;
		((PartModule)this).OnLoad((ConfigNode)null);
		((PartModule)this).OnStart((StartState)(HighLogic.LoadedSceneIsEditor ? 1 : 16));
	}

	public override void OnStart(StartState state)
	{
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Expected O, but got Unknown
		//IL_0048: Unknown result type (might be due to invalid IL or missing references)
		//IL_0052: Expected O, but got Unknown
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_013a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0144: Expected O, but got Unknown
		//IL_0144: Unknown result type (might be due to invalid IL or missing references)
		//IL_014e: Expected O, but got Unknown
		//IL_0161: Unknown result type (might be due to invalid IL or missing references)
		//IL_016b: Expected O, but got Unknown
		//IL_016b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0175: Expected O, but got Unknown
		if (HighLogic.LoadedSceneIsEditor)
		{
			_ready = true;
		}
		if ((int)state == 0)
		{
			return;
		}
		if (_unorderedComputerModules.Count == 0)
		{
			((PartModule)this).OnLoad((ConfigNode)null);
		}
		GameEvents.onShowUI.Add(new OnEvent(OnShowGUI));
		GameEvents.onHideUI.Add(new OnEvent(OnHideGUI));
		GameEvents.onVesselChange.Add((OnEvent<Vessel>)UnlockControl);
		GameEvents.onVesselWasModified.Add((OnEvent<Vessel>)OnVesselWasModified);
		GameEvents.onVesselStandardModification.Add((OnEvent<Vessel>)OnVesselStandardModification);
		_lastSettingsSaveTime = Time.time;
		foreach (ComputerModule computerModule in GetComputerModules<ComputerModule>())
		{
			try
			{
				computerModule.OnStart(state);
			}
			catch (Exception ex)
			{
				Debug.LogError((object)("MechJeb module " + computerModule.GetType().Name + " threw an exception in OnStart: " + ex));
			}
		}
		if ((Object)(object)((PartModule)this).vessel != (Object)null && (Object)(object)this != (Object)(object)((PartModule)this).vessel.GetMasterMechJeb())
		{
			Vessel vessel = ((PartModule)this).vessel;
			vessel.OnFlyByWire = (FlightInputCallback)Delegate.Remove((Delegate)(object)vessel.OnFlyByWire, (Delegate)new FlightInputCallback(OnFlyByWire));
			Vessel vessel2 = ((PartModule)this).vessel;
			vessel2.OnFlyByWire = (FlightInputCallback)Delegate.Combine((Delegate)(object)vessel2.OnFlyByWire, (Delegate)new FlightInputCallback(OnFlyByWire));
			_controlledVessel = ((PartModule)this).vessel;
		}
		Logger.GlobalRegister((Action<object>)SafePrint);
	}

	public override void OnActive()
	{
		foreach (ComputerModule computerModule in GetComputerModules<ComputerModule>())
		{
			try
			{
				computerModule.OnActive();
			}
			catch (Exception ex)
			{
				Debug.LogError((object)("MechJeb module " + computerModule.GetType().Name + " threw an exception in OnActive: " + ex));
			}
		}
	}

	public override void OnInactive()
	{
		foreach (ComputerModule computerModule in GetComputerModules<ComputerModule>())
		{
			try
			{
				computerModule.OnInactive();
			}
			catch (Exception ex)
			{
				Debug.LogError((object)("MechJeb module " + computerModule.GetType().Name + " threw an exception in OnInactive: " + ex));
			}
		}
	}

	public override void OnAwake()
	{
		CachedLocalizer.Bootstrap();
		Dispatcher.CreateDispatcher();
		foreach (ComputerModule computerModule in GetComputerModules<ComputerModule>())
		{
			try
			{
				computerModule.OnAwake();
			}
			catch (Exception ex)
			{
				Debug.LogError((object)("MechJeb module " + computerModule.GetType().Name + " threw an exception in OnAwake: " + ex));
			}
		}
	}

	public void FixedUpdate()
	{
		if (!FlightGlobals.ready)
		{
			return;
		}
		LoadDelayedModules();
		CheckControlledVessel();
		if ((Object)(object)this != (Object)(object)((PartModule)this).vessel.GetMasterMechJeb() || (HighLogic.LoadedSceneIsFlight && !((PartModule)this).vessel.isActiveVessel))
		{
			_wasMasterAndFocus = false;
		}
		if ((Object)(object)this != (Object)(object)((PartModule)this).vessel.GetMasterMechJeb())
		{
			return;
		}
		if (!_wasMasterAndFocus && (HighLogic.LoadedSceneIsEditor || ((PartModule)this).vessel.isActiveVessel))
		{
			if (HighLogic.LoadedSceneIsFlight && (Object)(object)_lastFocus != (Object)null && _lastFocus.loaded && (Object)(object)_lastFocus.GetMasterMechJeb() != (Object)null)
			{
				Print("Focus changed! Forcing " + _lastFocus.vesselName + " to save");
				((PartModule)_lastFocus.GetMasterMechJeb()).OnSave((ConfigNode)null);
			}
			ClearModulesCache();
			((PartModule)this).OnLoad((ConfigNode)null);
			_wasMasterAndFocus = true;
			_lastFocus = ((PartModule)this).vessel;
		}
		if ((Object)(object)((PartModule)this).vessel == (Object)null)
		{
			return;
		}
		_ready = VesselState.Update();
		foreach (ComputerModule computerModule in GetComputerModules<ComputerModule>())
		{
			if (computerModule == Staging)
			{
				continue;
			}
			try
			{
				if (computerModule.Enabled)
				{
					computerModule.OnFixedUpdate();
				}
			}
			catch (Exception ex)
			{
				Debug.LogError((object)("MechJeb module " + computerModule.GetType().Name + " threw an exception in OnFixedUpdate: " + ex));
			}
		}
		if (Staging == null || !Staging.Enabled)
		{
			return;
		}
		try
		{
			Staging.OnFixedUpdate();
		}
		catch (Exception ex2)
		{
			Debug.LogError((object)("MechJeb module " + Staging.GetType().Name + " threw an exception in OnFixedUpdate: " + ex2));
		}
	}

	private bool NeedToSave()
	{
		bool flag = false;
		foreach (ComputerModule computerModule in GetComputerModules<ComputerModule>())
		{
			flag |= computerModule.Dirty;
		}
		return flag;
	}

	public void Update()
	{
		if ((Object)(object)this != (Object)(object)((PartModule)this).vessel.GetMasterMechJeb() || (!FlightGlobals.ready && HighLogic.LoadedSceneIsFlight) || !_ready)
		{
			return;
		}
		if (Input.GetKeyDown((KeyCode)118) && (Input.GetKey((KeyCode)306) || Input.GetKey((KeyCode)305)))
		{
			GetComputerModule<MechJebModuleCustomWindowEditor>()?.CreateWindowFromSharingString(MuUtils.SystemClipboard);
		}
		if ((HighLogic.LoadedSceneIsEditor || ((Object)(object)((PartModule)this).vessel != (Object)null && ((PartModule)this).vessel.isActiveVessel)) && Time.time > _lastSettingsSaveTime + 5f)
		{
			if (NeedToSave())
			{
				((PartModule)this).OnSave((ConfigNode)null);
			}
			_lastSettingsSaveTime = Time.time;
		}
		if ((Object)(object)ResearchAndDevelopment.Instance != (Object)null && _unorderedComputerModules.Any((ComputerModule a) => !a.UnlockChecked))
		{
			foreach (ComputerModule computerModule in GetComputerModules<ComputerModule>())
			{
				try
				{
					computerModule.UnlockCheck();
				}
				catch (Exception ex)
				{
					Debug.LogError((object)("MechJeb module " + computerModule.GetType().Name + " threw an exception in UnlockCheck: " + ex));
				}
			}
		}
		GetComputerModule<MechJebModuleMenu>().OnMenuUpdate();
		if ((Object)(object)((PartModule)this).vessel == (Object)null)
		{
			return;
		}
		foreach (ComputerModule computerModule2 in GetComputerModules<ComputerModule>())
		{
			try
			{
				if (computerModule2.Enabled)
				{
					computerModule2.OnUpdate();
				}
			}
			catch (Exception ex2)
			{
				Debug.LogError((object)("MechJeb module " + computerModule2.GetType().Name + " threw an exception in OnUpdate: " + ex2));
			}
		}
	}

	private void LoadComputerModules()
	{
		if (_moduleRegistry == null)
		{
			_moduleRegistry = new List<Type>();
			Assembly[] assemblies = AppDomain.CurrentDomain.GetAssemblies();
			foreach (Assembly assembly in assemblies)
			{
				try
				{
					foreach (Type item in (from t in assembly.GetTypes()
						where t.IsSubclassOf(typeof(ComputerModule)) && !t.IsAbstract
						select t).ToList())
					{
						_moduleRegistry.Add(item);
					}
				}
				catch (Exception ex)
				{
					Debug.LogError((object)("MechJeb moduleRegistry creation threw an exception in LoadComputerModules loading " + assembly.FullName + ": " + ex));
				}
			}
		}
		Assembly executingAssembly = Assembly.GetExecutingAssembly();
		FileVersionInfo versionInfo = FileVersionInfo.GetVersionInfo(executingAssembly.Location);
		Attribute[] customAttributes = Attribute.GetCustomAttributes(executingAssembly, typeof(AssemblyInformationalVersionAttribute));
		string text = "";
		if (customAttributes.Length != 0)
		{
			text = ((AssemblyInformationalVersionAttribute)customAttributes[0]).InformationalVersion;
		}
		version = ((text == "") ? $"{versionInfo.FileMajorPart}.{versionInfo.FileMinorPart}.{versionInfo.FileBuildPart}" : text);
		if (HighLogic.LoadedSceneIsEditor || HighLogic.LoadedSceneIsFlight)
		{
			Print("Loading Mechjeb " + version);
		}
		try
		{
			foreach (Type item2 in _moduleRegistry)
			{
				if (item2 != typeof(ComputerModule) && item2 != typeof(DisplayModule) && item2 != typeof(MechJebModuleCustomInfoWindow) && item2 != typeof(AutopilotModule) && item2 != typeof(MechJebModuleAscentBaseAutopilot) && !blacklist.Contains(item2.Name) && GetComputerModule(item2.Name) == null)
				{
					ConstructorInfo constructor = item2.GetConstructor(new Type[1] { typeof(MechJebCore) });
					if (constructor != null)
					{
						AddComputerModule((ComputerModule)constructor.Invoke(new object[1] { this }));
					}
				}
			}
		}
		catch (Exception ex2)
		{
			Debug.LogError((object)("MechJeb moduleRegistry loading threw an exception in LoadComputerModules: " + ex2));
		}
		Attitude = GetComputerModule<MechJebModuleAttitudeController>();
		Staging = GetComputerModule<MechJebModuleStagingController>();
		Thrust = GetComputerModule<MechJebModuleThrustController>();
		Target = GetComputerModule<MechJebModuleTargetController>();
		Warp = GetComputerModule<MechJebModuleWarpController>();
		RCS = GetComputerModule<MechJebModuleRCSController>();
		Rcsbal = GetComputerModule<MechJebModuleRCSBalancer>();
		Rover = GetComputerModule<MechJebModuleRoverController>();
		Node = GetComputerModule<MechJebModuleNodeExecutor>();
		Solarpanel = GetComputerModule<MechJebModuleSolarPanelController>();
		AntennaControl = GetComputerModule<MechJebModuleDeployableAntennaController>();
		Landing = GetComputerModule<MechJebModuleLandingAutopilot>();
		Settings = GetComputerModule<MechJebModuleSettings>();
		Guidance = GetComputerModule<MechJebModuleGuidanceController>();
		Glueball = GetComputerModule<MechJebModulePSGGlueBall>();
		StageStats = GetComputerModule<MechJebModuleStageStats>();
		AscentSettings = GetComputerModule<MechJebModuleAscentSettings>();
		Spinup = GetComputerModule<MechJebModuleSpinupController>();
		Hoverslam = GetComputerModule<MechJebModuleHoverslamSimulation>();
		SmartASS = GetComputerModule<MechJebModuleSmartASS>();
	}

	public override void OnLoad(ConfigNode sfsNode)
	{
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_004c: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ea: Expected O, but got Unknown
		//IL_0192: Unknown result type (might be due to invalid IL or missing references)
		//IL_0199: Expected O, but got Unknown
		//IL_0091: Unknown result type (might be due to invalid IL or missing references)
		//IL_0097: Expected O, but got Unknown
		if ((Object)(object)GuiUtils.Skin == (Object)null)
		{
			new GameObject("zombieGUILoader", new Type[1] { typeof(ZombieGUILoader) });
		}
		try
		{
			bool flag = false;
			((PartModule)this).OnLoad(sfsNode);
			if (!_savedConfig.ContainsKey(((Object)((PartModule)this).part).name))
			{
				if ((int)HighLogic.LoadedScene == 0)
				{
					_savedConfig.Add(((Object)((PartModule)this).part).name, sfsNode);
				}
			}
			else
			{
				_partSettings = _savedConfig[((Object)((PartModule)this).part).name];
			}
			LoadComputerModules();
			ConfigNode val = new ConfigNode("MechJebGlobalSettings");
			if (MuUtils.FileExistsCreateDirectory(MuUtils.GetCfgPath("mechjeb_settings_global.cfg")))
			{
				try
				{
					val = ConfigNode.Load(MuUtils.GetCfgPath("mechjeb_settings_global.cfg"));
				}
				catch (Exception ex)
				{
					Debug.LogError((object)("MechJebCore.OnLoad caught an exception trying to load mechjeb_settings_global.cfg: " + ex));
					flag = true;
				}
			}
			else
			{
				flag = true;
			}
			ConfigNode val2 = new ConfigNode("MechJebTypeSettings");
			string text = (((Object)(object)((PartModule)this).vessel != (Object)null) ? string.Join("_", ((PartModule)this).vessel.vesselName.Split(Path.GetInvalidFileNameChars())) : "");
			if ((Object)(object)((PartModule)this).vessel != (Object)null && MuUtils.FileExistsCreateDirectory(MuUtils.GetCfgPath("mechjeb_settings_type_" + text + ".cfg")))
			{
				try
				{
					val2 = ConfigNode.Load(MuUtils.GetCfgPath("mechjeb_settings_type_" + text + ".cfg"));
				}
				catch (Exception ex2)
				{
					Debug.LogError((object)("MechJebCore.OnLoad caught an exception trying to load mechjeb_settings_type_" + text + ".cfg: " + ex2));
				}
			}
			ConfigNode val3 = new ConfigNode("MechJebLocalSettings");
			if (sfsNode != null && sfsNode.HasNode("MechJebLocalSettings"))
			{
				val3 = sfsNode.GetNode("MechJebLocalSettings");
			}
			else if (_partSettings != null && _partSettings.HasNode("MechJebLocalSettings"))
			{
				val3 = _partSettings.GetNode("MechJebLocalSettings");
			}
			else if (sfsNode == null)
			{
				foreach (ComputerModule computerModule3 in GetComputerModules<ComputerModule>())
				{
					try
					{
						computerModule3.OnSave(val3.AddNode(computerModule3.GetType().Name), null, null);
					}
					catch (Exception ex3)
					{
						Debug.LogError((object)("MechJeb module " + computerModule3.GetType().Name + " threw an exception in OnLoad: " + ex3));
					}
				}
			}
			while (true)
			{
				MechJebModuleCustomInfoWindow computerModule = GetComputerModule<MechJebModuleCustomInfoWindow>();
				if (computerModule == null)
				{
					break;
				}
				RemoveComputerModule(computerModule);
			}
			foreach (ComputerModule computerModule4 in GetComputerModules<ComputerModule>())
			{
				try
				{
					string name = computerModule4.GetType().Name;
					ConfigNode local = (val3.HasNode(name) ? val3.GetNode(name) : null);
					ConfigNode type = (val2.HasNode(name) ? val2.GetNode(name) : null);
					ConfigNode global = (val.HasNode(name) ? val.GetNode(name) : null);
					computerModule4.OnLoad(local, type, global);
				}
				catch (Exception ex4)
				{
					Debug.LogError((object)("MechJeb module " + computerModule4.GetType().Name + " threw an exception in OnLoad: " + ex4));
				}
			}
			LoadDelayedModules();
			MechJebModuleCustomWindowEditor computerModule2 = GetComputerModule<MechJebModuleCustomWindowEditor>();
			if (flag || computerModule2.RegenerateDefaultWindows)
			{
				computerModule2.AddDefaultWindows();
			}
		}
		catch (ReflectionTypeLoadException ex5)
		{
			Debug.LogError((object)"MechJeb caught a ReflectionTypeLoadException. Those DLL are not built for this KSP version:");
			foreach (Assembly item in (from x in ex5.Types
				where x != null
				select x.Assembly).Distinct())
			{
				Debug.LogError((object)(item.GetName().Name + " " + item.GetName().Version?.ToString() + " " + item.Location.Remove(0, Path.GetFullPath(KSPUtil.ApplicationRootPath).Length)));
			}
		}
		catch (Exception ex6)
		{
			Debug.LogError((object)("MechJeb caught exception in core OnLoad: " + ex6));
		}
	}

	public override void OnSave(ConfigNode sfsNode)
	{
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		//IL_0074: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		if ((!HighLogic.LoadedSceneIsEditor && !HighLogic.LoadedSceneIsFlight) || (Object)(object)this != (Object)(object)((PartModule)this).vessel.GetMasterMechJeb() || _unorderedComputerModules.Count == 0 || (HighLogic.LoadedSceneIsFlight && (Object)(object)((PartModule)this).vessel != (Object)null && ((PartModule)this).vessel.vesselName == null))
		{
			return;
		}
		try
		{
			LoadDelayedModules();
			ConfigNode val = ((sfsNode == null) ? ((ConfigNode)null) : new ConfigNode("MechJebLocalSettings"));
			ConfigNode val2 = ((sfsNode != null) ? ((ConfigNode)null) : new ConfigNode("MechJebTypeSettings"));
			ConfigNode val3 = ((sfsNode != null) ? ((ConfigNode)null) : new ConfigNode("MechJebGlobalSettings"));
			foreach (ComputerModule computerModule in GetComputerModules<ComputerModule>())
			{
				try
				{
					string name = computerModule.GetType().Name;
					computerModule.OnSave((val != null) ? val.AddNode(name) : null, (val2 != null) ? val2.AddNode(name) : null, (val3 != null) ? val3.AddNode(name) : null);
				}
				catch (Exception ex)
				{
					Debug.LogError((object)("MechJeb module " + computerModule.GetType().Name + " threw an exception in OnSave: " + ex));
				}
			}
			if (sfsNode != null)
			{
				sfsNode.nodes.Add(val);
			}
			if (val2 != null && (Object)(object)((PartModule)this).vessel != (Object)null)
			{
				string text = string.Join("_", ((PartModule)this).vessel.vesselName.Split(Path.GetInvalidFileNameChars()));
				val2.Save(MuUtils.GetCfgPath("mechjeb_settings_type_" + text + ".cfg"));
			}
			if (val3 != null && (Object)(object)_lastFocus == (Object)(object)((PartModule)this).vessel)
			{
				val3.Save(MuUtils.GetCfgPath("mechjeb_settings_global.cfg"));
			}
		}
		catch (Exception ex2)
		{
			Debug.LogError((object)("MechJeb caught exception in core OnSave: " + ex2));
		}
	}

	public void OnDestroy()
	{
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Expected O, but got Unknown
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Expected O, but got Unknown
		//IL_013e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0148: Expected O, but got Unknown
		//IL_0148: Unknown result type (might be due to invalid IL or missing references)
		//IL_0152: Expected O, but got Unknown
		if ((Object)(object)this == (Object)(object)((PartModule)this).vessel.GetMasterMechJeb() && ((Object)(object)((PartModule)this).vessel == (Object)null || ((PartModule)this).vessel.isActiveVessel))
		{
			((PartModule)this).OnSave((ConfigNode)null);
		}
		GameEvents.onShowUI.Remove(new OnEvent(OnShowGUI));
		GameEvents.onHideUI.Remove(new OnEvent(OnHideGUI));
		GameEvents.onVesselChange.Remove((OnEvent<Vessel>)UnlockControl);
		GameEvents.onVesselWasModified.Remove((OnEvent<Vessel>)OnVesselWasModified);
		GameEvents.onVesselStandardModification.Remove((OnEvent<Vessel>)OnVesselStandardModification);
		if (_weLockedInputs)
		{
			UnlockControl();
			ManeuverGizmoBase.HasMouseFocus = false;
		}
		foreach (ComputerModule computerModule in GetComputerModules<ComputerModule>())
		{
			try
			{
				computerModule.OnDestroy();
			}
			catch (Exception ex)
			{
				Debug.LogError((object)("MechJeb module " + computerModule.GetType().Name + " threw an exception in OnDestroy: " + ex));
			}
		}
		if ((Object)(object)((PartModule)this).vessel != (Object)null)
		{
			Vessel vessel = ((PartModule)this).vessel;
			vessel.OnFlyByWire = (FlightInputCallback)Delegate.Remove((Delegate)(object)vessel.OnFlyByWire, (Delegate)new FlightInputCallback(OnFlyByWire));
		}
		_controlledVessel = null;
	}

	private void OnFlyByWire(FlightCtrlState s)
	{
		if (!_deactivateControl && CheckControlledVessel() && !((Object)(object)this != (Object)(object)((PartModule)this).vessel.GetMasterMechJeb()))
		{
			Drive(s);
			CheckFlightCtrlState(s);
			_currentThrottle = s.mainThrottle;
		}
	}

	private void Drive(FlightCtrlState s)
	{
		_ready = VesselState.Update();
		if (!((Object)(object)this == (Object)(object)((PartModule)this).vessel.GetMasterMechJeb()))
		{
			return;
		}
		foreach (ComputerModule computerModule in GetComputerModules<ComputerModule>())
		{
			try
			{
				if (computerModule.Enabled)
				{
					computerModule.Drive(s);
				}
			}
			catch (Exception ex)
			{
				Debug.LogError((object)("MechJeb module " + computerModule.GetType().Name + " threw an exception in Drive: " + ex));
			}
		}
	}

	private static void CheckFlightCtrlState(FlightCtrlState s)
	{
		if (float.IsNaN(s.mainThrottle))
		{
			s.mainThrottle = 0f;
		}
		if (float.IsNaN(s.yaw))
		{
			s.yaw = 0f;
		}
		if (float.IsNaN(s.pitch))
		{
			s.pitch = 0f;
		}
		if (float.IsNaN(s.roll))
		{
			s.roll = 0f;
		}
		if (float.IsNaN(s.X))
		{
			s.X = 0f;
		}
		if (float.IsNaN(s.Y))
		{
			s.Y = 0f;
		}
		if (float.IsNaN(s.Z))
		{
			s.Z = 0f;
		}
		s.mainThrottle = Mathf.Clamp01(s.mainThrottle);
		s.yaw = Mathf.Clamp(s.yaw, -1f, 1f);
		s.pitch = Mathf.Clamp(s.pitch, -1f, 1f);
		s.roll = Mathf.Clamp(s.roll, -1f, 1f);
		s.X = Mathf.Clamp(s.X, -1f, 1f);
		s.Y = Mathf.Clamp(s.Y, -1f, 1f);
		s.Z = Mathf.Clamp(s.Z, -1f, 1f);
	}

	private void OnShowGUI()
	{
		ShowGui = true;
	}

	private void OnHideGUI()
	{
		ShowGui = false;
	}

	private void OnGUI()
	{
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0074: Unknown result type (might be due to invalid IL or missing references)
		//IL_0088: Unknown result type (might be due to invalid IL or missing references)
		//IL_008d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0134: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Invalid comparison between Unknown and I4
		if (!ShowGui || (Object)(object)this != (Object)(object)((PartModule)this).vessel.GetMasterMechJeb() || (!FlightGlobals.ready && HighLogic.LoadedSceneIsFlight) || !_ready || (!HighLogic.LoadedSceneIsEditor && (!FlightGlobals.ready || !((Object)(object)((PartModule)this).vessel == (Object)(object)FlightGlobals.ActiveVessel) || (int)((PartModule)this).part.State == 3)))
		{
			return;
		}
		Matrix4x4 matrix = GUI.matrix;
		GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(GuiUtils.Scale, GuiUtils.Scale, 1f));
		GuiUtils.ComboBox.DrawGUI();
		GuiUtils.LoadSkin((GuiUtils.SkinType)Settings.SkinId);
		GUI.skin = GuiUtils.Skin;
		foreach (DisplayModule computerModule in GetComputerModules<DisplayModule>())
		{
			try
			{
				if (computerModule.Enabled)
				{
					computerModule.DrawGUI(HighLogic.LoadedSceneIsEditor);
				}
			}
			catch (Exception ex)
			{
				Debug.LogError((object)("MechJeb module " + computerModule.GetType().Name + " threw an exception in DrawGUI: " + ex));
			}
		}
		PreventClickthrough();
		GUI.matrix = matrix;
		for (int i = 0; i < _postDrawQueue.Count; i++)
		{
			_postDrawQueue[i].Invoke();
		}
	}

	internal void AddToPostDrawQueue(Callback c)
	{
		_postDrawQueue.Add(c);
	}

	public override string GetInfo()
	{
		return Localizer.Format("#MechJeb_MechJebInfo_VABSPH");
	}

	private void PreventClickthrough()
	{
		bool flag = GuiUtils.MouseIsOverWindow(this);
		if (!_weLockedInputs)
		{
			if (flag && !Input.GetMouseButton(1))
			{
				LockControl();
			}
		}
		else if (!flag)
		{
			UnlockControl();
		}
	}

	private void LockControl()
	{
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		if (HighLogic.LoadedSceneIsEditor)
		{
			EditorLogic.fetch.Lock(true, true, true, "MechJeb_noclick");
		}
		else if (HighLogic.LoadedSceneIsFlight || HighLogic.LoadedSceneHasPlanetarium)
		{
			InputLockManager.SetControlLock((ControlTypes)900719925474097919L, "MechJeb_noclick");
		}
		_weLockedInputs = true;
	}

	private void UnlockControl(Vessel v)
	{
		UnlockControl();
	}

	private void UnlockControl()
	{
		if (HighLogic.LoadedSceneIsEditor)
		{
			EditorLogic.fetch.Unlock("MechJeb_noclick");
		}
		else if (HighLogic.LoadedSceneIsFlight || HighLogic.LoadedSceneHasPlanetarium)
		{
			InputLockManager.RemoveControlLock("MechJeb_noclick");
		}
		_weLockedInputs = false;
	}

	private void OnVesselWasModified(Vessel v)
	{
		if ((Object)(object)v != (Object)(object)((PartModule)this).vessel || (Object)(object)this != (Object)(object)((PartModule)this).vessel.GetMasterMechJeb())
		{
			return;
		}
		foreach (ComputerModule item in from x in GetComputerModules<ComputerModule>()
			where x.Enabled
			select x)
		{
			try
			{
				item.OnVesselWasModified(v);
			}
			catch (Exception arg)
			{
				Debug.LogError((object)string.Format("MechJeb module {0} threw an exception in {1}: {2}", item.GetType().Name, "MechJebCore.OnVesselWasModified", arg));
			}
		}
	}

	private void OnVesselStandardModification(Vessel v)
	{
		if ((Object)(object)v != (Object)(object)((PartModule)this).vessel || (Object)(object)this != (Object)(object)((PartModule)this).vessel.GetMasterMechJeb())
		{
			return;
		}
		foreach (ComputerModule item in from x in GetComputerModules<ComputerModule>()
			where x.Enabled
			select x)
		{
			try
			{
				item.OnVesselStandardModification(v);
			}
			catch (Exception arg)
			{
				Debug.LogError((object)string.Format("MechJeb module {0} threw an exception in {1}: {2}", item.GetType().Name, "MechJebCore.OnVesselStandardModification", arg));
			}
		}
	}

	public static void Print(object message)
	{
		MonoBehaviour.print((object)("[MechJeb2] " + message));
	}

	public static void SafePrint(object message)
	{
		Dispatcher.InvokeAsync(delegate
		{
			MonoBehaviour.print((object)("[MechJeb2] " + message));
		});
	}
}
