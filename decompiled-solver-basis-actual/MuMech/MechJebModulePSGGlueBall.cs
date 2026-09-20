using System;
using MechJebLib.FuelFlowSimulation;
using MechJebLib.PSG;
using MechJebLib.Primitives;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModulePSGGlueBall : ComputerModule
{
	private double _blockOptimizerUntilTime;

	public int SuccessfulConverges;

	public int LastLmStatus;

	public int MaxLmIterations;

	public int LastLmIterations;

	public string? LastFailureMessage;

	public double Staleness;

	public double LastInfeasibility;

	private double _lastTime;

	private Ascent? _ascent;

	private MechJebModuleAscentSettings _ascentSettings => Core.AscentSettings;

	public MechJebModulePSGGlueBall(MechJebCore core)
		: base(core)
	{
	}

	protected override void OnModuleEnabled()
	{
		Debug.Log((object)"Enabling PSG GlueBall");
		SuccessfulConverges = (LastLmStatus = (MaxLmIterations = 0));
		LastLmStatus = (LastLmIterations = 0);
		Staleness = (LastInfeasibility = (_lastTime = 0.0));
		LastFailureMessage = null;
		_ascent = null;
	}

	protected override void OnModuleDisabled()
	{
		Debug.Log((object)"Disabling PSG GlueBall");
		_ascent = null;
	}

	public override void OnFixedUpdate()
	{
		Core.StageStats.RequestUpdate();
	}

	public override void OnStart(StartState state)
	{
		GameEvents.onStageActivate.Add((OnEvent<int>)HandleStageEvent);
	}

	public override void OnDestroy()
	{
		GameEvents.onStageActivate.Remove((OnEvent<int>)HandleStageEvent);
	}

	private void HandleStageEvent(int data)
	{
		_blockOptimizerUntilTime = base.VesselState.Time + (double)_ascentSettings.OptimizerPauseTime;
	}

	private bool IsUnguided(int s)
	{
		return _ascentSettings.UnguidedStages.Contains(s);
	}

	private bool IsFixed(int s)
	{
		return _ascentSettings.FixedStages.Contains(s);
	}

	private void HandleDoneTask()
	{
		Ascent ascent = _ascent;
		if (ascent == null || !((AsyncJob)ascent).IsCompleted)
		{
			return;
		}
		try
		{
			Optimizer optimizer = _ascent.GetOptimizer();
			if (optimizer != null)
			{
				LastLmStatus = optimizer.TerminationType;
				LastLmIterations = optimizer.Iterations;
				LastInfeasibility = optimizer.PrimalFeasibility;
				if (LastLmIterations > MaxLmIterations)
				{
					MaxLmIterations = LastLmIterations;
				}
				if (optimizer.Success() && optimizer.Solution != null)
				{
					Core.Guidance.SetSolution(optimizer.Solution);
					SuccessfulConverges++;
					_lastTime = base.VesselState.Time;
					Staleness = 0.0;
					LastFailureMessage = null;
				}
				else
				{
					Debug.Log((object)("failed guidance, znorm: " + optimizer.PrimalFeasibility));
				}
			}
		}
		finally
		{
			((AsyncJob)_ascent).TryMarkReady();
		}
	}

	private void GatherException()
	{
		Ascent ascent = _ascent;
		if (ascent != null && ((AsyncJob)ascent).IsFaulted)
		{
			LastFailureMessage = ((AsyncJob)_ascent).Exception?.Message;
			if (((AsyncJob)_ascent).Exception != null)
			{
				Debug.Log((object)((AsyncJob)_ascent).Exception);
			}
		}
	}

	private void MarkReady()
	{
		Ascent ascent = _ascent;
		if (ascent == null || !((AsyncJob)ascent).IsStopped || ((AsyncJob)_ascent).TryMarkReady())
		{
			return;
		}
		throw new Exception("[MechJebModulePSGGlueBall] could not mark job as ready");
	}

	public void SetTarget(double peR, double apR, double attR, double inclination, double lan, double fpa, bool attachAltFlag, bool lanflag)
	{
		//IL_00dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_0274: Unknown result type (might be due to invalid IL or missing references)
		//IL_0284: Unknown result type (might be due to invalid IL or missing references)
		//IL_0294: Unknown result type (might be due to invalid IL or missing references)
		//IL_03cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_03d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_03d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_03e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0448: Unknown result type (might be due to invalid IL or missing references)
		//IL_044d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0461: Unknown result type (might be due to invalid IL or missing references)
		//IL_047f: Unknown result type (might be due to invalid IL or missing references)
		//IL_049d: Unknown result type (might be due to invalid IL or missing references)
		//IL_04b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_06da: Unknown result type (might be due to invalid IL or missing references)
		//IL_06f4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0705: Unknown result type (might be due to invalid IL or missing references)
		//IL_0716: Unknown result type (might be due to invalid IL or missing references)
		//IL_0727: Unknown result type (might be due to invalid IL or missing references)
		//IL_0565: Unknown result type (might be due to invalid IL or missing references)
		//IL_0699: Unknown result type (might be due to invalid IL or missing references)
		//IL_0686: Unknown result type (might be due to invalid IL or missing references)
		//IL_057f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0590: Unknown result type (might be due to invalid IL or missing references)
		//IL_05a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_05b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_06ad: Unknown result type (might be due to invalid IL or missing references)
		if (_lastTime == 0.0)
		{
			_lastTime = base.VesselState.Time;
		}
		Staleness = base.VesselState.Time - _lastTime;
		Ascent ascent = _ascent;
		if (ascent != null && ((AsyncJob)ascent).IsRunning)
		{
			return;
		}
		GatherException();
		HandleDoneTask();
		MarkReady();
		if (_ascentSettings.OptimizeStageFlag)
		{
			if (apR > 0.0 && apR < peR)
			{
				apR = peR;
			}
			if (attR < peR)
			{
				attR = peR;
			}
			if (apR > 0.0 && attR > apR)
			{
				attR = apR;
			}
		}
		if (base.Vessel.VesselOffGround())
		{
			bool flag = false;
			for (int num = Core.StageStats.VacStats.Count - 1; num >= 0; num--)
			{
				double deltaV = Core.StageStats.VacStats[num].DeltaV;
				int kSPStage = Core.StageStats.VacStats[num].KSPStage;
				if (kSPStage < (int)_ascentSettings.LastStage)
				{
					break;
				}
				if (!IsCurrentCoastAfterStage(kSPStage) && deltaV != 0.0 && !IsUnguided(kSPStage))
				{
					flag = true;
				}
			}
			if (!flag)
			{
				return;
			}
		}
		if (!Core.Guidance.IsReady())
		{
			return;
		}
		if (Core.Guidance.Solution != null)
		{
			int num2 = Core.Guidance.Solution.IndexForKSPStage(base.Vessel.currentStage, Core.Guidance.IsCoasting());
			if (num2 >= 0)
			{
				Solution? solution = Core.Guidance.Solution;
				if (((solution != null) ? new double?(solution.Tgo(base.VesselState.Time, num2)) : null) < (double)_ascentSettings.PreStageTime)
				{
					_blockOptimizerUntilTime = base.VesselState.Time + (double)_ascentSettings.OptimizerPauseTime;
					return;
				}
			}
		}
		if (_blockOptimizerUntilTime > base.VesselState.Time)
		{
			return;
		}
		double num3 = _ascentSettings.DesiredArgP;
		bool desiredArgPFlag = _ascentSettings.DesiredArgPFlag;
		AscentBuilder val = Ascent.Builder().Initial(Core.StageStats.VacR, Core.StageStats.VacV, Core.StageStats.VacU, Core.StageStats.VacT, base.MainBody.gravParameter, base.MainBody.Radius).SetTarget(peR, apR, attR, Statics.Deg2Rad(inclination), Statics.Deg2Rad(lan), num3, fpa, attachAltFlag, lanflag, desiredArgPFlag);
		if (base.MainBody.atmosphere)
		{
			double num4 = base.MainBody.atmosphereDepth * 0.15;
			double atmDensityASL = base.MainBody.atmDensityASL;
			double density = base.MainBody.GetDensity(base.MainBody.GetPressure(num4), base.MainBody.GetTemperature(num4));
			double num5 = num4 / Math.Log(atmDensityASL / density);
			double num6 = _ascentSettings.Cd;
			double num7 = _ascentSettings.Aref;
			double num8 = _ascentSettings.LimitQa;
			double num9 = (Core.Thrust.LimitDynamicPressure ? Core.Thrust.MaxDynamicPressure.Val : 0.0);
			V3 val2 = Math.PI * 2.0 / base.MainBody.rotationPeriod * V3.northpole;
			val.AerodynamicConstants(num6, num7, atmDensityASL, num8, num9, num5, val2);
		}
		if (Core.Guidance.Solution != null)
		{
			val.OldSolution(Core.Guidance.Solution);
		}
		bool flag2 = false;
		for (int num10 = Core.StageStats.VacStats.Count - 1; num10 >= 0; num10--)
		{
			FuelStats val3 = Core.StageStats.VacStats[num10];
			int kSPStage2 = Core.StageStats.VacStats[num10].KSPStage;
			double isp = Core.StageStats.AtmoStats[num10].Isp;
			double num11 = Core.StageStats.VacStats[num10].MinThrust / Core.StageStats.VacStats[num10].MaxThrust;
			if (kSPStage2 < (int)_ascentSettings.LastStage)
			{
				break;
			}
			bool flag3 = false;
			if (((!flag2 && Core.Guidance.IsCoasting()) || !Core.Guidance.hasCoasted) && ((kSPStage2 == _ascentSettings.CoastStage && (CoastingBefore() || CoastingDuring())) || (kSPStage2 == _ascentSettings.CoastStage - 1 && CoastingAfter())))
			{
				if (CoastingDuring() && !Core.Guidance.hasCoasted && val3.DeltaV > (double)_ascentSettings.MinDeltaV)
				{
					val.AddStage(val3.StartMass * 1000.0, val3.EndMass * 1000.0, val3.MaxThrust * 1000.0, val3.Isp, kSPStage2, num10, IsUnguided(kSPStage2), !IsFixed(kSPStage2), false, isp, num11);
					flag3 = true;
				}
				flag2 = true;
				double num12 = _ascentSettings.MaxCoast;
				double num13 = _ascentSettings.MinCoast;
				if (Core.Guidance.IsCoasting())
				{
					num12 = Math.Max(num12 - (base.VesselState.Time - Core.Guidance.StartCoast), 0.0);
					num13 = Math.Max(num13 - (base.VesselState.Time - Core.Guidance.StartCoast), 0.0);
				}
				bool flag4 = IsUnguided(kSPStage2);
				double num14 = (CoastingDuring() ? (val3.EndMass * 1000.0) : (val3.StartMass * 1000.0));
				val.AddCoast(val3.StartMass * 1000.0, num14, num13, num12, _ascentSettings.CoastStage, num10, flag4, false);
			}
			if (!(val3.DeltaV < (double)_ascentSettings.MinDeltaV))
			{
				val.AddStage(val3.StartMass * 1000.0, val3.EndMass * 1000.0, val3.MaxThrust * 1000.0, val3.Isp, kSPStage2, num10, IsUnguided(kSPStage2), !IsFixed(kSPStage2), flag3, isp, num11);
			}
		}
		_ascent = val.Build();
		if (!((AsyncJob)_ascent).TryStartJob((object)null))
		{
			throw new Exception("[MechJebModulePSGGlueBall] could not start optimizer job");
		}
		_blockOptimizerUntilTime = base.VesselState.Time + 1.0;
	}

	private bool IsCurrentCoastAfterStage(int kspStage)
	{
		if (kspStage == base.Vessel.currentStage && Core.Guidance.IsCoasting() && CoastingAfter())
		{
			return true;
		}
		return false;
	}

	private bool CoastingBefore()
	{
		return _ascentSettings.CoastLocation == -1;
	}

	private bool CoastingDuring()
	{
		return _ascentSettings.CoastLocation == 0;
	}

	private bool CoastingAfter()
	{
		return _ascentSettings.CoastLocation == 1;
	}
}
