using System;
using System.Collections.Generic;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class OperationCourseCorrection : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_approach_title");

	[Persistent(pass = 4)]
	public EditableDoubleMult Periapsis = new EditableDoubleMult(200000.0, 1000.0);

	[Persistent(pass = 4)]
	public EditableDouble Inclination = new EditableDouble(90.0);

	[Persistent(pass = 4)]
	public bool InclinationFlag;

	[Persistent(pass = 4)]
	public EditableDoubleMult InterceptDistance = new EditableDoubleMult(200.0);

	private static readonly TimeReference[] _timeReferences = new TimeReference[8]
	{
		TimeReference.COMPUTED,
		TimeReference.X_FROM_NOW,
		TimeReference.ALTITUDE,
		TimeReference.EQ_DESCENDING,
		TimeReference.EQ_ASCENDING,
		TimeReference.REL_NEAREST_AD,
		TimeReference.REL_ASCENDING,
		TimeReference.REL_DESCENDING
	};

	private static readonly TimeSelector _timeSelector = new TimeSelector(_timeReferences);

	public override string GetName()
	{
		return _name;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		if (target.Target is CelestialBody)
		{
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_approach_label1"), Periapsis, "km");
			GuiUtils.ToggledTextBox(ref InclinationFlag, "Inclination", Inclination, "°");
		}
		else
		{
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_approach_label2"), InterceptDistance, "m");
		}
		if (target.Target is CelestialBody)
		{
			_timeSelector.DoChooseTimeGUI();
		}
		else
		{
			GUILayout.Label(Localizer.Format("#MechJeb_approach_label3"), Array.Empty<GUILayoutOption>());
		}
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double ut, MechJebModuleTargetController target)
	{
		//IL_00fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		//IL_021d: Unknown result type (might be due to invalid IL or missing references)
		//IL_020b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0210: Unknown result type (might be due to invalid IL or missing references)
		if (!target.NormalTargetExists)
		{
			throw new OperationException(Localizer.Format("#MechJeb_approach_Exception1"));
		}
		Orbit val;
		for (val = o; val != null; val = ((PartModule)target.Core).vessel.GetNextPatch(val))
		{
			if ((Object)(object)val.referenceBody == (Object)(object)target.TargetOrbit.referenceBody)
			{
				o = val;
				ut = val.StartUT;
				break;
			}
		}
		if (val == null || (Object)(object)val.referenceBody != (Object)(object)target.TargetOrbit.referenceBody)
		{
			throw new OperationException(Localizer.Format("#MechJeb_approach_Exception2"));
		}
		if (o.NextClosestApproachTime(target.TargetOrbit, ut) < ut + 1.0 || o.NextClosestApproachDistance(target.TargetOrbit, ut) > target.TargetOrbit.semiMajorAxis * 0.2)
		{
			ErrorMessage = Localizer.Format("#MechJeb_Approach_errormsg");
		}
		ITargetable target2 = target.Target;
		CelestialBody val2 = (CelestialBody)(object)((target2 is CelestialBody) ? target2 : null);
		double num = 0.0;
		Vector3d dV;
		if (val2 == null)
		{
			dV = OrbitalManeuverCalculator.DeltaVAndTimeForCheapestCourseCorrection(o, ut, target.TargetOrbit, InterceptDistance, out ut);
		}
		else
		{
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
				ut = _timeSelector.ComputeManeuverTime(o, ut, target);
			}
			double dt = ((_timeSelector.TimeReference == TimeReference.COMPUTED) ? double.NaN : 0.0);
			double inc = (InclinationFlag ? Inclination.Val : double.NaN);
			(dV, num) = OrbitalManeuverCalculator.DeltaVAndTimeForCourseCorrectionToCelestial(o, ut, val2, val2.Radius + (double)Periapsis, dt, inc);
		}
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, ut + num)
		};
	}
}
