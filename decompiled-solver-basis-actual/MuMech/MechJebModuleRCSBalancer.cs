using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using KSP.Localization;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModuleRCSBalancer : ComputerModule
{
	[Persistent(pass = 6)]
	[ToggleInfoItem("#MechJeb_smartTranslation", InfoItem.Category.Thrust)]
	public bool smartTranslation;

	[Persistent(pass = 6)]
	[EditableInfoItem("#MechJeb_RCSBalancerOverdrive", InfoItem.Category.Thrust, rightLabel = "%")]
	public EditableDoubleMult overdrive = new EditableDoubleMult(1.0, 0.01);

	[Persistent(pass = 6)]
	public bool advancedOptions;

	[Persistent(pass = 6)]
	public readonly EditableDouble overdriveScale = 0.9;

	[Persistent(pass = 6)]
	public readonly EditableDouble tuningParamFactorTorque = 1.0;

	[Persistent(pass = 6)]
	public readonly EditableDouble tuningParamFactorTranslate = 0.005;

	[Persistent(pass = 6)]
	public readonly EditableDouble tuningParamFactorWaste = 1.0;

	private readonly RCSSolverThread solverThread = new RCSSolverThread();

	private List<RCSSolver.Thruster> thrusters;

	private double[] throttles;

	[EditableInfoItem("#MechJeb_RCSBalancerPrecision", InfoItem.Category.Thrust)]
	public readonly EditableInt calcPrecision = 3;

	[GeneralInfoItem("#MechJeb_RCSBalancerInfo", InfoItem.Category.Thrust)]
	public void RCSBalancerInfoItem()
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RCSBalancerInfo_Label1"), (solverThread.CalculationTime * 1000.0).ToString("F0") + " ms");
		GuiUtils.SimpleLabelInt(Localizer.Format("#MechJeb_RCSBalancerInfo_Label2"), solverThread.TaskCount);
		GuiUtils.SimpleLabelInt(Localizer.Format("#MechJeb_RCSBalancerInfo_Label3"), solverThread.CacheSize);
		GuiUtils.SimpleLabelInt(Localizer.Format("#MechJeb_RCSBalancerInfo_Label4"), solverThread.CacheHits);
		GuiUtils.SimpleLabelInt(Localizer.Format("#MechJeb_RCSBalancerInfo_Label5"), solverThread.CacheMisses);
		GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RCSBalancerInfo_Label6"), Statics.ToSI(solverThread.ComError, 4, int.MaxValue) + "m");
		GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RCSBalancerInfo_Label7"), Statics.ToSI(solverThread.ComErrorThreshold, 4, int.MaxValue) + "m");
		GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RCSBalancerInfo_Label8"), Statics.ToSI(solverThread.MaxComError, 4, int.MaxValue) + "m");
		GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RCSBalancerInfo_Label9"), solverThread.StatusString);
		string errorString = solverThread.ErrorString;
		if (!string.IsNullOrEmpty(errorString))
		{
			GUILayout.Label(errorString, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		}
		GUILayout.EndVertical();
	}

	[GeneralInfoItem("#MechJeb_RCSThrusterStates", InfoItem.Category.Thrust)]
	private void RCSThrusterStateInfoItem()
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_RCSThrusterStates_Label1"), Array.Empty<GUILayoutOption>());
		bool flag = true;
		string text = "";
		for (int i = 0; i < base.Vessel.parts.Count; i++)
		{
			foreach (ModuleRCS item in ((IEnumerable)base.Vessel.parts[i].Modules).OfType<ModuleRCS>())
			{
				if (!flag)
				{
					text += " ";
				}
				flag = false;
				text += $"({item.thrusterPower * 9f:F0}:";
				for (int j = 0; j < item.thrustForces.Length; j++)
				{
					if (j != 0)
					{
						text += ",";
					}
					text += (item.thrustForces[j] * 9f).ToString("F0");
				}
				text += ")";
			}
		}
		GUILayout.Label(text, Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
	}

	[GeneralInfoItem("#MechJeb_RCSPartThrottles", InfoItem.Category.Thrust)]
	private void RCSPartThrottlesInfoItem()
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		bool flag = true;
		string text = "";
		for (int i = 0; i < base.Vessel.parts.Count; i++)
		{
			foreach (ModuleRCS item in ((IEnumerable)base.Vessel.parts[i].Modules).OfType<ModuleRCS>())
			{
				if (!flag)
				{
					text += " ";
				}
				flag = false;
				text += item.thrusterPower.ToString("F1");
			}
		}
		GUILayout.Label(text, Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
	}

	[GeneralInfoItem("#MechJeb_ControlVector", InfoItem.Category.Thrust)]
	private void ControlVectorInfoItem()
	{
		FlightCtrlState state = FlightInputHandler.state;
		string rightLabel = $"{state.X:F2} {state.Y:F2} {state.Z:F2}";
		string rightLabel2 = $"{state.roll:F2} {state.pitch:F2} {state.yaw:F2}";
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleLabel("X/Y/Z", rightLabel);
		GuiUtils.SimpleLabel("R/P/Y", rightLabel2);
		GUILayout.EndVertical();
	}

	public MechJebModuleRCSBalancer(MechJebCore core)
		: base(core)
	{
		Priority = 700;
	}

	protected override void OnModuleEnabled()
	{
		UpdateTuningParameters();
		solverThread.Start();
		base.OnModuleEnabled();
	}

	protected override void OnModuleDisabled()
	{
		solverThread.Stop();
		base.OnModuleDisabled();
	}

	public void ResetThrusterForces()
	{
		solverThread.ResetThrusterForces();
	}

	public void GetThrottles(Vector3 direction, out double[] throttles, out List<RCSSolver.Thruster> thrusters)
	{
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		solverThread.GetThrottles(base.Vessel, base.VesselState, direction, out throttles, out thrusters);
	}

	protected void AdjustRCSThrottles(FlightCtrlState s)
	{
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		bool flag = false;
		if (s.X == 0f && s.Y == 0f && s.Z == 0f)
		{
			solverThread.ResetThrusterForces();
		}
		Vector3 direction = default(Vector3);
		((Vector3)(ref direction))._002Ector(0f - s.X, 0f - s.Z, 0f - s.Y);
		RCSSolverKey.SetPrecision(calcPrecision);
		GetThrottles(direction, out throttles, out thrusters);
		if (throttles.Length != thrusters.Count)
		{
			throttles = new double[thrusters.Count];
			flag = true;
		}
		if (flag)
		{
			for (int i = 0; i < throttles.Length; i++)
			{
				throttles[i] = 0.0;
			}
		}
		for (int j = 0; j < thrusters.Count; j++)
		{
			thrusters[j].PartModule.thrusterPower = (float)throttles[j];
		}
	}

	public void UpdateTuningParameters()
	{
		double wasteThreshold = (double)overdrive * (double)overdriveScale;
		RCSSolverTuningParams rCSSolverTuningParams = new RCSSolverTuningParams();
		rCSSolverTuningParams.WasteThreshold = wasteThreshold;
		rCSSolverTuningParams.FactorTorque = tuningParamFactorTorque;
		rCSSolverTuningParams.FactorTranslate = tuningParamFactorTranslate;
		rCSSolverTuningParams.FactorWaste = tuningParamFactorWaste;
		solverThread.UpdateTuningParameters(rCSSolverTuningParams);
	}

	public double GetCalculationTime()
	{
		return solverThread.CalculationTime;
	}

	public override void Drive(FlightCtrlState s)
	{
		if (smartTranslation)
		{
			AdjustRCSThrottles(s);
		}
		base.Drive(s);
	}
}
