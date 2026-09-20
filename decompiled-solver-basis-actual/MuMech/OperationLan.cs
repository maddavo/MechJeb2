using System;
using System.Collections.Generic;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class OperationLan : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_AN_title");

	private static readonly TimeReference[] _timeReferences = new TimeReference[3]
	{
		TimeReference.APOAPSIS,
		TimeReference.PERIAPSIS,
		TimeReference.X_FROM_NOW
	};

	private static readonly TimeSelector _timeSelector = new TimeSelector(_timeReferences);

	public override string GetName()
	{
		return _name;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		_timeSelector.DoChooseTimeGUI();
		GUILayout.Label(Localizer.Format("#MechJeb_AN_label"), Array.Empty<GUILayoutOption>());
		target.targetLongitude.DrawEditGUI(EditableAngle.Direction.EW);
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_0050: Unknown result type (might be due to invalid IL or missing references)
		//IL_0055: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		if (o.inclination < 10.0)
		{
			ErrorMessage = Localizer.Format("#MechJeb_AN_error", new object[1] { o.inclination });
		}
		double num = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		Vector3d dV = OrbitalManeuverCalculator.DeltaVToShiftLAN(o, num, target.targetLongitude);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, num)
		};
	}
}
