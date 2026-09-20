using System;
using System.Collections.Generic;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class OperationGeneric : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_Hohm_title");

	[Persistent(pass = 4)]
	public bool Capture = true;

	[Persistent(pass = 4)]
	public bool PlanCapture = true;

	[Persistent(pass = 4)]
	public bool MatchOrbit;

	[Persistent(pass = 4)]
	public EditableDouble LagTime = 0.0;

	[Persistent(pass = 4)]
	public EditableTime MinDepartureUT = 0.0;

	[Persistent(pass = 4)]
	public EditableTime MaxDepartureUT = 0.0;

	[Persistent(pass = 4)]
	public bool Coplanar;

	private static readonly TimeReference[] _timeReferences = new TimeReference[11]
	{
		TimeReference.COMPUTED,
		TimeReference.PERIAPSIS,
		TimeReference.APOAPSIS,
		TimeReference.X_FROM_NOW,
		TimeReference.ALTITUDE,
		TimeReference.EQ_DESCENDING,
		TimeReference.EQ_ASCENDING,
		TimeReference.REL_NEAREST_AD,
		TimeReference.REL_ASCENDING,
		TimeReference.REL_DESCENDING,
		TimeReference.CLOSEST_APPROACH
	};

	private static readonly TimeSelector _timeSelector = new TimeSelector(_timeReferences);

	public override string GetName()
	{
		return _name;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		bool flag = target.Target is CelestialBody;
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (GUILayout.Toggle(Capture, flag ? Localizer.Format("#MechJeb_Hohm_transfer") : Localizer.Format("#MechJeb_Hohm_rendezvous"), Array.Empty<GUILayoutOption>()))
		{
			Capture = true;
		}
		if (GUILayout.Toggle(!Capture, flag ? Localizer.Format("#MechJeb_Hohm_flyby") : Localizer.Format("#MechJeb_Hohm_intercept"), Array.Empty<GUILayoutOption>()))
		{
			Capture = false;
		}
		GUILayout.EndHorizontal();
		if (Capture)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			MatchOrbit = GUILayout.Toggle(MatchOrbit, Localizer.Format("#MechJeb_Hohm_matchOrbit"), Array.Empty<GUILayoutOption>());
			GUILayout.EndHorizontal();
		}
		if (Capture && !MatchOrbit)
		{
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_Hohm_arrivalDelay"), LagTime, "sec");
		}
		if (Capture && (!flag || MatchOrbit || LagTime.Val != 0.0))
		{
			PlanCapture = GUILayout.Toggle(PlanCapture, Localizer.Format("#MechJeb_Hohm_createArrivalNode"), Array.Empty<GUILayoutOption>());
		}
		else
		{
			PlanCapture = false;
		}
		Coplanar = GUILayout.Toggle(Coplanar, Localizer.Format("#MechJeb_Hohm_simpleTransfer"), Array.Empty<GUILayoutOption>());
		_timeSelector.DoChooseTimeGUI();
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_0136: Unknown result type (might be due to invalid IL or missing references)
		//IL_013b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0145: Unknown result type (might be due to invalid IL or missing references)
		//IL_014a: Unknown result type (might be due to invalid IL or missing references)
		//IL_018c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0169: Unknown result type (might be due to invalid IL or missing references)
		//IL_0177: Unknown result type (might be due to invalid IL or missing references)
		if (!target.NormalTargetExists)
		{
			throw new OperationException(Localizer.Format("#MechJeb_Hohm_Exception1"));
		}
		if ((Object)(object)o.referenceBody != (Object)(object)target.TargetOrbit.referenceBody)
		{
			throw new OperationException(Localizer.Format("#MechJeb_Hohm_Exception2"));
		}
		Orbit targetOrbit = target.TargetOrbit;
		double lagTime = ((!MatchOrbit) ? LagTime.Val : 0.0);
		bool fixedTime = false;
		if (_timeSelector.TimeReference != 0)
		{
			bool flag = o.AscendingNodeExists(target.TargetOrbit);
			bool flag2 = o.DescendingNodeExists(target.TargetOrbit);
			if (_timeSelector.TimeReference == TimeReference.REL_ASCENDING && !flag)
			{
				throw new OperationException(Localizer.Format("#MechJeb_Hohm_Exception3"));
			}
			if (_timeSelector.TimeReference == TimeReference.REL_DESCENDING && !flag2)
			{
				throw new OperationException(Localizer.Format("#MechJeb_Hohm_Exception4"));
			}
			if (_timeSelector.TimeReference == TimeReference.REL_NEAREST_AD && !(flag || flag2))
			{
				throw new OperationException(Localizer.Format("#MechJeb_Hohm_Exception5"));
			}
			universalTime = _timeSelector.ComputeManeuverTime(o, universalTime, target);
			fixedTime = true;
		}
		var (dV, ut, dV2, ut2) = OrbitalManeuverCalculator.DeltaVAndTimeForHohmannTransfer(o, targetOrbit, universalTime, lagTime, fixedTime, Coplanar, Capture && !MatchOrbit, Capture);
		if (Capture && PlanCapture)
		{
			return new List<ManeuverParameters>
			{
				new ManeuverParameters(dV, ut),
				new ManeuverParameters(dV2, ut2)
			};
		}
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, ut)
		};
	}
}
