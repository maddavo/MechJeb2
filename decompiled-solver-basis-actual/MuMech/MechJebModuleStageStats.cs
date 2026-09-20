using System;
using System.Collections.Generic;
using System.Diagnostics;
using MechJebLib.FuelFlowSimulation;
using MechJebLib.Primitives;
using MechJebLib.Utils;
using MechJebLibBindings;
using MechJebLibBindings.FuelFlowSimulation;
using Unity.Profiling;
using UnityEngine;

namespace MuMech;

public class MechJebModuleStageStats : ComputerModule
{
	[ToggleInfoItem("#MechJeb_DVincludecosinelosses", InfoItem.Category.Thrust, showInEditor = true)]
	public readonly bool DVLinearThrust = true;

	public CelestialBody EditorBody;

	public bool LiveSLT = true;

	public double AltSLT;

	public double Mach;

	private int _vabRebuildTimer = 1;

	public readonly List<FuelStats> AtmoStats = new List<FuelStats>();

	public readonly List<FuelStats> VacStats = new List<FuelStats>();

	public double AtmoT;

	public double VacT;

	public V3 AtmoR;

	public V3 VacR;

	public V3 AtmoV;

	public V3 VacV;

	public V3 AtmoU;

	public V3 VacU;

	private bool _vesselModified = true;

	private readonly SimVesselManager _vesselManagerAtmo = new SimVesselManager();

	private readonly SimVesselManager _vesselManagerVac = new SimVesselManager();

	private static ProfilerMarker _newRunSimulationProfile = new ProfilerMarker("RunSimulation");

	private static ProfilerMarker _newBuildProfile = new ProfilerMarker("Build");

	private static ProfilerMarker _newUpdateProfile = new ProfilerMarker("Update");

	private static ProfilerMarker _newVacProfile = new ProfilerMarker("Vac");

	private static ProfilerMarker _newAtmoProfile = new ProfilerMarker("Atmo");

	private readonly Stopwatch _stopwatch = new Stopwatch();

	public MechJebModuleStageStats(MechJebCore core)
		: base(core)
	{
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_003d: Expected O, but got Unknown
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0048: Expected O, but got Unknown
		base.Enabled = true;
	}

	protected override void OnModuleEnabled()
	{
		_vesselModified = true;
	}

	protected override void OnModuleDisabled()
	{
		_vesselManagerAtmo.Release();
		_vesselManagerVac.Release();
	}

	public override void OnFixedUpdate()
	{
		GetResults();
	}

	public override void OnUpdate()
	{
		GetResults();
	}

	private void GetResults()
	{
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		//IL_015d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0162: Unknown result type (might be due to invalid IL or missing references)
		//IL_0169: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_0097: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c7: Unknown result type (might be due to invalid IL or missing references)
		if (((AsyncJob)_vesselManagerAtmo.FuelFlowSimulation).IsStopped)
		{
			if (((AsyncJob)_vesselManagerAtmo.FuelFlowSimulation).IsCompleted)
			{
				AtmoStats.Clear();
				foreach (FuelStats segment in _vesselManagerAtmo.FuelFlowSimulation.Segments)
				{
					AtmoStats.Add(segment);
				}
				AtmoT = _vesselManagerAtmo.T;
				AtmoR = _vesselManagerAtmo.R;
				AtmoV = _vesselManagerAtmo.V;
				AtmoU = _vesselManagerAtmo.U;
			}
			else
			{
				Debug.Log((object)"[MechJebModuleStageStats] atmo stats failed");
				if (((AsyncJob)_vesselManagerAtmo.FuelFlowSimulation).Exception != null)
				{
					Debug.Log((object)((AsyncJob)_vesselManagerAtmo.FuelFlowSimulation).Exception);
				}
			}
			if (!((AsyncJob)_vesselManagerAtmo.FuelFlowSimulation).TryMarkReady())
			{
				throw new Exception("[MechJebModuleStageStats] Tried to mark a running atmo stage stats as ready.");
			}
		}
		if (!((AsyncJob)_vesselManagerVac.FuelFlowSimulation).IsStopped)
		{
			return;
		}
		if (((AsyncJob)_vesselManagerVac.FuelFlowSimulation).IsCompleted)
		{
			VacStats.Clear();
			foreach (FuelStats segment2 in _vesselManagerVac.FuelFlowSimulation.Segments)
			{
				VacStats.Add(segment2);
			}
			VacT = _vesselManagerVac.T;
			VacR = _vesselManagerVac.R;
			VacV = _vesselManagerVac.V;
			VacU = _vesselManagerVac.U;
		}
		else
		{
			Debug.Log((object)"[MechJebModuleStageStats] vac stats failed");
			if (((AsyncJob)_vesselManagerVac.FuelFlowSimulation).Exception != null)
			{
				Debug.Log((object)((AsyncJob)_vesselManagerVac.FuelFlowSimulation).Exception);
			}
		}
		if (!((AsyncJob)_vesselManagerVac.FuelFlowSimulation).TryMarkReady())
		{
			throw new Exception("[MechJebModuleStageStats] Tried to mark a running vac stage stats as ready.");
		}
	}

	private void RunSimulation()
	{
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00da: Unknown result type (might be due to invalid IL or missing references)
		//IL_00df: Unknown result type (might be due to invalid IL or missing references)
		//IL_013a: Unknown result type (might be due to invalid IL or missing references)
		//IL_013f: Unknown result type (might be due to invalid IL or missing references)
		//IL_016c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0171: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_021d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0222: Unknown result type (might be due to invalid IL or missing references)
		//IL_0261: Unknown result type (might be due to invalid IL or missing references)
		//IL_0266: Unknown result type (might be due to invalid IL or missing references)
		//IL_0271: Unknown result type (might be due to invalid IL or missing references)
		//IL_0276: Unknown result type (might be due to invalid IL or missing references)
		//IL_0281: Unknown result type (might be due to invalid IL or missing references)
		//IL_0286: Unknown result type (might be due to invalid IL or missing references)
		AutoScope val = ((ProfilerMarker)(ref _newRunSimulationProfile)).Auto();
		try
		{
			CelestialBody val2 = (HighLogic.LoadedSceneIsEditor ? EditorBody : base.Vessel.mainBody);
			double num = ((!HighLogic.LoadedSceneIsEditor && LiveSLT) ? base.Vessel.staticPressurekPa : (val2.atmosphere ? val2.GetPressure(AltSLT) : 0.0));
			double num2 = ((HighLogic.LoadedSceneIsEditor || !LiveSLT) ? val2.GetDensity(val2.GetPressure(AltSLT), val2.GetTemperature(0.0)) : base.Vessel.atmDensity) / 1.225;
			double num3 = (HighLogic.LoadedSceneIsEditor ? Mach : base.Vessel.mach);
			if (_vesselModified || HighLogic.LoadedSceneIsEditor)
			{
				AutoScope val3 = ((ProfilerMarker)(ref _newBuildProfile)).Auto();
				try
				{
					IShipconstruct obj;
					if (!HighLogic.LoadedSceneIsEditor)
					{
						IShipconstruct vessel = (IShipconstruct)(object)base.Vessel;
						obj = vessel;
					}
					else
					{
						IShipconstruct vessel = (IShipconstruct)(object)EditorLogic.fetch.ship;
						obj = vessel;
					}
					IShipconstruct val4 = obj;
					_vesselManagerAtmo.Build(val4);
					_vesselManagerVac.Build(val4);
					_vesselModified = false;
				}
				finally
				{
					((IDisposable)(AutoScope)(ref val3)).Dispose();
				}
			}
			else
			{
				AutoScope val5 = ((ProfilerMarker)(ref _newUpdateProfile)).Auto();
				try
				{
					_vesselManagerAtmo.Update();
					_vesselManagerVac.Update();
				}
				finally
				{
					((IDisposable)(AutoScope)(ref val5)).Dispose();
				}
			}
			AutoScope val6 = ((ProfilerMarker)(ref _newVacProfile)).Auto();
			try
			{
				_vesselManagerVac.DVLinearThrust = DVLinearThrust;
				_vesselManagerVac.SetConditions(0.0, 0.0, 0.0);
				_vesselManagerVac.SetInitial(base.VesselState.Time, MathExtensions.WorldToV3Rotated(base.VesselState.OrbitalPosition), MathExtensions.WorldToV3Rotated(base.VesselState.OrbitalVelocity), MathExtensions.WorldToV3Rotated(base.VesselState.Forward));
				if (!_vesselManagerVac.TryStartFuelFlowSimulationJob())
				{
					throw new Exception("[MechJebModuleStageStats] could not start vac stats job");
				}
			}
			finally
			{
				((IDisposable)(AutoScope)(ref val6)).Dispose();
			}
			val6 = ((ProfilerMarker)(ref _newAtmoProfile)).Auto();
			try
			{
				_vesselManagerAtmo.DVLinearThrust = DVLinearThrust;
				_vesselManagerAtmo.SetConditions(num2, num * PhysicsGlobals.KpaToAtmospheres, num3);
				_vesselManagerAtmo.SetInitial(base.VesselState.Time, MathExtensions.WorldToV3Rotated(base.VesselState.OrbitalPosition), MathExtensions.WorldToV3Rotated(base.VesselState.OrbitalVelocity), MathExtensions.WorldToV3Rotated(base.VesselState.Forward));
				if (!_vesselManagerAtmo.TryStartFuelFlowSimulationJob())
				{
					throw new Exception("[MechJebModuleStageStats] could not start atmo stats job");
				}
			}
			finally
			{
				((IDisposable)(AutoScope)(ref val6)).Dispose();
			}
		}
		finally
		{
			((IDisposable)(AutoScope)(ref val)).Dispose();
		}
	}

	private void StartSimulation()
	{
		if (HighLogic.LoadedSceneIsEditor)
		{
			if (_vabRebuildTimer > 0)
			{
				PartSet.BuildPartSets(EditorLogic.fetch.ship.parts, (Vessel)null);
				_vabRebuildTimer--;
				_vesselModified = true;
			}
		}
		else
		{
			base.Vessel.UpdateResourceSetsIfDirty();
		}
		RunSimulation();
	}

	private bool SimulationReady()
	{
		if (((AsyncJob)_vesselManagerAtmo.FuelFlowSimulation).IsReady)
		{
			return ((AsyncJob)_vesselManagerVac.FuelFlowSimulation).IsReady;
		}
		return false;
	}

	private void TryStartSimulation()
	{
		if (!SimulationReady())
		{
			return;
		}
		if (HighLogic.LoadedSceneIsEditor)
		{
			if (EditorBody == null)
			{
				return;
			}
		}
		else if (base.Vessel == null)
		{
			return;
		}
		double num = (HighLogic.LoadedSceneIsEditor ? 500 : 100);
		if (!_stopwatch.IsRunning || !((double)_stopwatch.ElapsedMilliseconds < num))
		{
			_stopwatch.Restart();
			StartSimulation();
		}
	}

	public override void OnStart(StartState state)
	{
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Expected O, but got Unknown
		GameEvents.onVesselStandardModification.Add((OnEvent<Vessel>)onVesselStandardModification);
		StageManager.OnGUIStageSequenceModified.Add(new OnEvent(OnGUIStageSequenceModified));
		if (HighLogic.LoadedSceneIsEditor)
		{
			GameEvents.onEditorShipModified.Add((OnEvent<ShipConstruct>)OnEditorShipModified);
			GameEvents.onPartCrossfeedStateChange.Add((OnEvent<Part>)OnPartCrossfeedStateChange);
		}
	}

	public override void OnDestroy()
	{
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Expected O, but got Unknown
		GameEvents.onVesselStandardModification.Remove((OnEvent<Vessel>)onVesselStandardModification);
		StageManager.OnGUIStageSequenceModified.Remove(new OnEvent(OnGUIStageSequenceModified));
		GameEvents.onEditorShipModified.Remove((OnEvent<ShipConstruct>)OnEditorShipModified);
		GameEvents.onPartCrossfeedStateChange.Remove((OnEvent<Part>)OnPartCrossfeedStateChange);
	}

	private void OnPartCrossfeedStateChange(Part data)
	{
		_vesselModified = true;
		_vabRebuildTimer = 2;
	}

	private void OnEditorShipModified(ShipConstruct data)
	{
		_vesselModified = true;
		_vabRebuildTimer = 2;
	}

	private void OnGUIStageSequenceModified()
	{
		_vesselModified = true;
	}

	private void onVesselStandardModification(Vessel data)
	{
		_vesselModified = true;
	}

	public void RequestUpdate()
	{
		GetResults();
		TryStartSimulation();
	}
}
