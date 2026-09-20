using System;
using System.Collections.Generic;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class OperationLongitude : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_la_title");

	private static readonly TimeReference[] _timeReferences = new TimeReference[2]
	{
		TimeReference.APOAPSIS,
		TimeReference.PERIAPSIS
	};

	private static readonly TimeSelector _timeSelector = new TimeSelector(_timeReferences);

	public override string GetName()
	{
		return _name;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		_timeSelector.DoChooseTimeGUI();
		GUILayout.Label(Localizer.Format("#MechJeb_la_label"), Array.Empty<GUILayoutOption>());
		target.targetLongitude.DrawEditGUI(EditableAngle.Direction.EW);
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		double num = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		Vector3d dV = OrbitalManeuverCalculator.DeltaVToShiftNodeLongitude(o, num, target.targetLongitude);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, num)
		};
	}
}
